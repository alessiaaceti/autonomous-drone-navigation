#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "drone_navigation_cpp/msg/target_state.hpp"
#include "drone_navigation_cpp/msg/decision_state.hpp"


class DecisionLayer : public rclcpp::Node
{
public:

    DecisionLayer()
        : Node("decision_layer"),
          obstacle_state_("UNKNOWN"),
          target_received_(false),
          obstacle_received_(false),
          obstacle_timeout_(std::chrono::milliseconds(500)),
          target_timeout_(std::chrono::milliseconds(500))
    {
        target_subscription_ =
            this->create_subscription<
                drone_navigation_cpp::msg::TargetState>(
                "/target_state",
                10,
                std::bind(
                    &DecisionLayer::target_callback,
                    this,
                    std::placeholders::_1));

        obstacle_subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "/obstacle_state",
                10,
                std::bind(
                    &DecisionLayer::obstacle_callback,
                    this,
                    std::placeholders::_1));

        decision_publisher_ =
            this->create_publisher<
                drone_navigation_cpp::msg::DecisionState>(
                "/decision",
                10);

        decision_timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &DecisionLayer::decision_callback,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Decision Layer started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Safety priority: OBSTACLE > TARGET");

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle watchdog: 500 ms");

        RCLCPP_INFO(
            this->get_logger(),
            "Target watchdog: 500 ms");
    }


private:

    void target_callback(
        const drone_navigation_cpp::msg::TargetState::SharedPtr msg)
    {
        latest_target_ = *msg;
        target_received_ = true;
        last_target_time_ = this->get_clock()->now();
    }


    void obstacle_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        obstacle_state_ = msg->data;
        obstacle_received_ = true;
        last_obstacle_time_ = this->get_clock()->now();
    }


    bool obstacle_data_is_fresh() const
    {
        if (!obstacle_received_)
        {
            return false;
        }

        const rclcpp::Time now =
            this->get_clock()->now();

        const auto age =
            now - last_obstacle_time_;

        return age <= obstacle_timeout_;
    }


    bool target_data_is_fresh() const
    {
        if (!target_received_)
        {
            return false;
        }

        const rclcpp::Time now =
            this->get_clock()->now();

        const auto age =
            now - last_target_time_;

        return age <= target_timeout_;
    }


    void fill_target_information(
        drone_navigation_cpp::msg::DecisionState &decision,
        bool target_fresh)
    {
        /*
         * A stale target must not remain latched as detected.
         *
         * If the target stream stops, the safe interpretation
         * is that there is currently no confirmed target.
         */
        decision.target_detected =
            target_fresh &&
            latest_target_.detected;

        if (target_fresh)
        {
            decision.target_confidence =
                latest_target_.confidence;

            decision.target_error_x =
                latest_target_.error_x;

            decision.target_error_y =
                latest_target_.error_y;

            decision.target_area =
                latest_target_.area;
        }
        else
        {
            decision.target_confidence = 0.0f;
            decision.target_error_x = 0.0f;
            decision.target_error_y = 0.0f;
            decision.target_area = 0.0f;
        }
    }


    void decision_callback()
    {
        drone_navigation_cpp::msg::DecisionState decision;

        const bool obstacle_fresh =
            obstacle_data_is_fresh();

        const bool target_fresh =
            target_data_is_fresh();


        /*
         * ----------------------------------------------------
         * SAFETY WATCHDOG
         * ----------------------------------------------------
         *
         * Obstacle perception is safety-critical.
         *
         * If the detector has never produced a state, or the
         * latest state is too old, we MUST NOT assume CLEAR.
         */
        if (!obstacle_fresh)
        {
            decision.mode = "STOP";

            if (!obstacle_received_)
            {
                decision.reason =
                    "Waiting for obstacle state";
            }
            else
            {
                decision.reason =
                    "Obstacle state timeout";
            }

            decision.obstacle_state = "UNKNOWN";
            decision.safety_override = true;

            fill_target_information(
                decision,
                target_fresh);

            publish_decision(decision);
            return;
        }


        /*
         * ----------------------------------------------------
         * EXPLICIT UNKNOWN
         * ----------------------------------------------------
         *
         * UNKNOWN is never treated as CLEAR.
         */
        if (obstacle_state_ == "UNKNOWN")
        {
            decision.mode = "STOP";
            decision.reason =
                "Obstacle perception is unknown";
            decision.obstacle_state = "UNKNOWN";
            decision.safety_override = true;

            fill_target_information(
                decision,
                target_fresh);

            publish_decision(decision);
            return;
        }


        /*
         * ----------------------------------------------------
         * SAFETY PRIORITY
         * ----------------------------------------------------
         *
         * A blocked path always overrides target tracking.
         */
        if (obstacle_state_ == "BLOCKED")
        {
            decision.mode = "STOP";
            decision.reason =
                "Path completely blocked";
            decision.safety_override = true;
        }


        /*
         * Any detected obstacle has priority over
         * semantic target tracking.
         */
        else if (
            obstacle_state_ == "LEFT" ||
            obstacle_state_ == "RIGHT" ||
            obstacle_state_ == "CENTER" ||
            obstacle_state_ == "CENTER_LEFT" ||
            obstacle_state_ == "CENTER_RIGHT")
        {
            decision.mode = "AVOID_OBSTACLE";
            decision.reason =
                "Obstacle detected; safety has priority";
            decision.safety_override = true;
        }


        /*
         * ----------------------------------------------------
         * TARGET TRACKING
         * ----------------------------------------------------
         *
         * Target tracking is allowed only when:
         *
         *   1. obstacle data is fresh
         *   2. obstacle state is CLEAR
         *   3. target data is fresh
         *   4. target is detected
         */
        else if (
            obstacle_state_ == "CLEAR" &&
            target_fresh &&
            latest_target_.detected)
        {
            decision.mode = "TRACK_TARGET";
            decision.reason =
                "Target detected and path is clear";
            decision.safety_override = false;
        }


        /*
         * CLEAR but no valid target.
         */
        else if (obstacle_state_ == "CLEAR")
        {
            decision.mode = "SEARCH_TARGET";
            decision.reason =
                "Path clear but no target detected";
            decision.safety_override = false;
        }


        /*
         * Unknown/unexpected obstacle state.
         *
         * Conservative fallback.
         */
        else
        {
            decision.mode = "STOP";
            decision.reason =
                "Unknown obstacle state";
            decision.safety_override = true;
        }


        decision.obstacle_state =
            obstacle_state_;

        fill_target_information(
            decision,
            target_fresh);

        publish_decision(decision);
    }


    void publish_decision(
        const drone_navigation_cpp::msg::DecisionState &decision)
    {
        decision_publisher_->publish(decision);

        /*
         * Print only when the decision mode changes.
         * The message is still published every 100 ms.
         */
        if (decision.mode != last_mode_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "DECISION: %-16s | reason: %s | obstacle: %s | target: %s",
                decision.mode.c_str(),
                decision.reason.c_str(),
                decision.obstacle_state.c_str(),
                decision.target_detected ? "YES" : "NO");

            last_mode_ = decision.mode;
        }
    }


    rclcpp::Subscription<
        drone_navigation_cpp::msg::TargetState>::SharedPtr
        target_subscription_;

    rclcpp::Subscription<
        std_msgs::msg::String>::SharedPtr
        obstacle_subscription_;

    rclcpp::Publisher<
        drone_navigation_cpp::msg::DecisionState>::SharedPtr
        decision_publisher_;

    rclcpp::TimerBase::SharedPtr
        decision_timer_;

    drone_navigation_cpp::msg::TargetState
        latest_target_;

    std::string obstacle_state_;
    std::string last_mode_;

    bool target_received_;
    bool obstacle_received_;

    rclcpp::Time last_target_time_;
    rclcpp::Time last_obstacle_time_;

    std::chrono::milliseconds obstacle_timeout_;
    std::chrono::milliseconds target_timeout_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DecisionLayer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}