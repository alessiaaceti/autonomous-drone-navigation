#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class OffboardPath : public rclcpp::Node
{
public:
    OffboardPath(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("offboard_path", options)
    {
        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 10);

        offboard_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);

        traj_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);

        // Subscribe to the vehicle's estimated local position from PX4.
        position_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &OffboardPath::position_callback,
                    this,
                    std::placeholders::_1));
        
        // Square flight path: x, y, z
        // PX4 uses NED coordinates, therefore altitude is negative.
        path_ = {
            {0.0f, 0.0f, -2.5f},
            {3.0f, 0.0f, -2.5f},
            {3.0f, 3.0f, -2.5f},
            {0.0f, 3.0f, -2.5f},
            {0.0f, 0.0f, -2.5f}
        };

        current_wp_ = 0;
        cycle_count_ = 0;

        // Publish setpoints at 20 Hz.
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&OffboardPath::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Offboard Path node started.");
    }

private:
    void timer_callback()
    {
        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        // Give PX4 some setpoints before requesting OFFBOARD.
        cycle_count_++;

        if (!offboard_requested_ && cycle_count_ >= 20) {
            request_offboard_mode();
            offboard_requested_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Offboard mode command sent.");
        }

        // Arm only after requesting OFFBOARD.
        if (!arm_requested_ && cycle_count_ >= 40) {
            arm_vehicle();
            arm_requested_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Arm command sent.");
        }

        update_waypoint();
    }

    void position_callback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;

        position_valid_ =
            msg->xy_valid && msg->z_valid;
    }

    void update_waypoint()
    {
        // We need valid PX4 position feedback before changing waypoints.
        if (!position_valid_) {
            return;
        }

        // Keep the first waypoint as the takeoff position.
        // Once the drone has reached approximately 2.5 m altitude,
        // start the square at waypoint 1.
        if (current_wp_ == 0) {
            if (arm_requested_ && current_z_ <= -2.0f) {
                current_wp_ = 1;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Takeoff altitude reached. Moving to waypoint %zu: "
                    "[%.1f, %.1f, %.1f]",
                    current_wp_,
                    path_[current_wp_][0],
                    path_[current_wp_][1],
                    path_[current_wp_][2]);
            }

            return;
        }

        // Calculate the 3D distance between the drone and the current waypoint.
        const float dx = current_x_ - path_[current_wp_][0];
        const float dy = current_y_ - path_[current_wp_][1];
        const float dz = current_z_ - path_[current_wp_][2];

        const float distance =
            std::sqrt(dx * dx + dy * dy + dz * dz);

        // Move to the next waypoint when the current one is reached.
        constexpr float waypoint_tolerance = 0.30f;

        if (distance <= waypoint_tolerance) {
            current_wp_ = (current_wp_ + 1) % path_.size();

            RCLCPP_INFO(
                this->get_logger(),
                "Waypoint reached. Moving to waypoint %zu: "
                "[%.1f, %.1f, %.1f]",
                current_wp_,
                path_[current_wp_][0],
                path_[current_wp_][1],
                path_[current_wp_][2]);
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;

        msg.timestamp = now_microseconds();

        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg;

        msg.timestamp = now_microseconds();

        msg.position[0] = path_[current_wp_][0];
        msg.position[1] = path_[current_wp_][1];
        msg.position[2] = path_[current_wp_][2];

        msg.yaw = 0.0f;

        traj_pub_->publish(msg);
    }

    void request_offboard_mode()
    {
        px4_msgs::msg::VehicleCommand msg;

        msg.timestamp = now_microseconds();

        msg.param1 = 1.0f;
        msg.param2 = 6.0f;

        msg.command =
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    void arm_vehicle()
    {
        px4_msgs::msg::VehicleCommand msg;

        msg.timestamp = now_microseconds();

        msg.param1 = 1.0f;

        msg.command =
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.from_external = true;

        vehicle_command_pub_->publish(msg);
    }

    uint64_t now_microseconds()
    {
        return static_cast<uint64_t>(
            this->get_clock()->now().nanoseconds() / 1000);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_mode_pub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        traj_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        position_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<float>> path_;

    size_t current_wp_;
    uint32_t cycle_count_;

    float current_x_ = 0.0f;
    float current_y_ = 0.0f;
    float current_z_ = 0.0f;

    bool position_valid_ = false;
    bool offboard_requested_ = false;
    bool arm_requested_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;

    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    auto node = std::make_shared<OffboardPath>(options);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}