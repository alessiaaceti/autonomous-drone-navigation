#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance()
        : Node("obstacle_avoidance")
    {
        obstacle_subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "/obstacle_state",
                10,
                std::bind(
                    &ObstacleAvoidance::obstacle_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle avoidance node started.");
    }

private:
    void obstacle_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &state = msg->data;

        std::string decision;

        if (state == "CLEAR")
        {
            decision = "FORWARD";
        }
        else if (state == "LEFT")
        {
            decision = "AVOID_RIGHT";
        }
        else if (state == "RIGHT")
        {
            decision = "AVOID_LEFT";
        }
        else if (state == "CENTER")
        {
            decision = "CHOOSE_SIDE";
        }
        else if (state == "CENTER_LEFT")
        {
            decision = "AVOID_RIGHT";
        }
        else if (state == "CENTER_RIGHT")
        {
            decision = "AVOID_LEFT";
        }
        else if (state == "BLOCKED")
        {
            decision = "STOP";
        }
        else
        {
            decision = "UNKNOWN";
        }

        // Only report the decision when the state or decision changes.
        if (state == last_state_ &&
            decision == last_decision_)
        {
            return;
        }

        last_state_ = state;
        last_decision_ = decision;

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle state: %-13s -> Decision: %s",
            state.c_str(),
            decision.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        obstacle_subscription_;

    std::string last_state_;
    std::string last_decision_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleAvoidance>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}