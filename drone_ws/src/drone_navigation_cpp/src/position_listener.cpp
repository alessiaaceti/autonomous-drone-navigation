#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

class PositionListener : public rclcpp::Node
{
public:
    PositionListener() : Node("position_listener")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&PositionListener::callback, this, std::placeholders::_1));
    }

private:
    void callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Position -> x: %.2f y: %.2f z: %.2f",
                    msg->x, msg->y, msg->z);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionListener>());
    rclcpp::shutdown();
    return 0;
}
