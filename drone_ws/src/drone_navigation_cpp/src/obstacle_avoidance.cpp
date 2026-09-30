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

        avoidance_command_publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "/avoidance_command",
                10);

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
            decision = "MOVE_RIGHT";
        }
        else if (state == "RIGHT")
        {
            decision = "MOVE_LEFT";
        }
        else if (state == "CENTER")
        {
            decision = "CHOOSE_SIDE";
        }
        else if (state == "CENTER_LEFT")
        {
            decision = "MOVE_RIGHT";
        }
        else if (state == "CENTER_RIGHT")
        {
            decision = "MOVE_LEFT";
        }
        else if (state == "BLOCKED")
        {
            decision = "STOP";
        }
        else
        {
            decision = "UNKNOWN";
        }

        // Only publish when the decision changes.
        if (decision == last_decision_)
        {
            return;
        }

        last_decision_ = decision;

        std_msgs::msg::String command_msg;
        command_msg.data = decision;

        avoidance_command_publisher_->publish(command_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle state: %-13s -> Command: %s",
            state.c_str(),
            decision.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        obstacle_subscription_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        avoidance_command_publisher_;

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