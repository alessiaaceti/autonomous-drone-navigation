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
          obstacle_received_(false)
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
    }


private:

    void target_callback(
        const drone_navigation_cpp::msg::TargetState::SharedPtr msg)
    {
        latest_target_ = *msg;
        target_received_ = true;
    }


    void obstacle_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        obstacle_state_ = msg->data;
        obstacle_received_ = true;
    }


    void decision_callback()
    {
        drone_navigation_cpp::msg::DecisionState decision;

        /*
         * Default safety behaviour.
         *
         * Until we have received a valid obstacle state,
         * do not authorize target tracking.
         */
        if (!obstacle_received_)
        {
            decision.mode = "STOP";
            decision.reason = "Waiting for obstacle state";
            decision.obstacle_state = "UNKNOWN";

            decision.target_detected =
                target_received_ && latest_target_.detected;

            decision.safety_override = true;

            decision.target_confidence =
                target_received_
                    ? latest_target_.confidence
                    : 0.0f;

            decision.target_error_x =
                target_received_
                    ? latest_target_.error_x
                    : 0.0f;

            decision.target_error_y =
                target_received_
                    ? latest_target_.error_y
                    : 0.0f;

            decision.target_area =
                target_received_
                    ? latest_target_.area
                    : 0.0f;

            publish_decision(decision);
            return;
        }


        /*
         * SAFETY PRIORITY
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
         * No obstacle: target tracking can be used.
         */
        else if (
            obstacle_state_ == "CLEAR" &&
            target_received_ &&
            latest_target_.detected)
        {
            decision.mode = "TRACK_TARGET";
            decision.reason =
                "Target detected and path is clear";
            decision.safety_override = false;
        }


        /*
         * No obstacle and no target.
         */
        else if (obstacle_state_ == "CLEAR")
        {
            decision.mode = "SEARCH_TARGET";
            decision.reason =
                "Path clear but no target detected";
            decision.safety_override = false;
        }


        /*
         * Unknown state: conservative behaviour.
         */
        else
        {
            decision.mode = "STOP";
            decision.reason =
                "Unknown obstacle state";
            decision.safety_override = true;
        }


        decision.obstacle_state = obstacle_state_;

        decision.target_detected =
            target_received_ && latest_target_.detected;

        decision.target_confidence =
            target_received_
                ? latest_target_.confidence
                : 0.0f;

        decision.target_error_x =
            target_received_
                ? latest_target_.error_x
                : 0.0f;

        decision.target_error_y =
            target_received_
                ? latest_target_.error_y
                : 0.0f;

        decision.target_area =
            target_received_
                ? latest_target_.area
                : 0.0f;

        publish_decision(decision);
    }


    void publish_decision(
        const drone_navigation_cpp::msg::DecisionState &decision)
    {
        decision_publisher_->publish(decision);

        /*
         * Only print when the decision changes.
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

    rclcpp::TimerBase::SharedPtr decision_timer_;

    drone_navigation_cpp::msg::TargetState latest_target_;

    std::string obstacle_state_;
    std::string last_mode_;

    bool target_received_;
    bool obstacle_received_;
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
