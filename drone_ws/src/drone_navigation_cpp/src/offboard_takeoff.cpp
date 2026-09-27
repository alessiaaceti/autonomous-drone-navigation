#include <cstdint>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using namespace std::chrono_literals;

class OffboardTakeoff : public rclcpp::Node
{
public:
    OffboardTakeoff()
        : Node("offboard_takeoff"),
          setpoint_counter_(0),
          offboard_requested_(false),
          arm_requested_(false)
    {
        // Publisher for PX4 vehicle commands
        vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", 10);

        // Publisher for Offboard control mode
        offboard_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);

        // Publisher for trajectory setpoints
        trajectory_pub_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);

        // Publish at 20 Hz
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&OffboardTakeoff::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Offboard Takeoff node started.");
    }

private:

    void timer_callback()
    {
        /*
         * PX4 requires a continuous stream of Offboard setpoints
         * before entering Offboard mode.
         */

        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        /*
         * Send the Offboard request only after a number of
         * setpoints have already been published.
         */
        if (setpoint_counter_ == 20 && !offboard_requested_) {
            publish_offboard_command();
            offboard_requested_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Offboard mode command sent.");
        }

        /*
         * Arm only after requesting Offboard mode.
         */
        if (setpoint_counter_ == 40 && !arm_requested_) {
            publish_arm_command();
            arm_requested_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Arm command sent.");
        }

        /*
         * Keep the counter from growing indefinitely.
         */
        if (setpoint_counter_ < 1000) {
            setpoint_counter_++;
        }
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.timestamp = timestamp();

        // We control the vehicle position.
        msg.position = true;

        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.timestamp = timestamp();

        /*
         * PX4 local position uses the NED convention:
         *
         * x = North
         * y = East
         * z = Down
         *
         * Therefore z = -2.5 means 2.5 meters above
         * the local origin.
         */
        msg.position[0] = 0.0f;
        msg.position[1] = 0.0f;
        msg.position[2] = -2.5f;

        // Keep the vehicle heading north.
        msg.yaw = 0.0f;

        trajectory_pub_->publish(msg);
    }

    void publish_offboard_command()
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.timestamp = timestamp();

        msg.command =
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

        /*
         * PX4 custom mode:
         * param1 = 1 -> custom mode
         * param2 = 6 -> Offboard
         */
        msg.param1 = 1.0f;
        msg.param2 = 6.0f;

        set_command_target(msg);

        vehicle_command_pub_->publish(msg);
    }

    void publish_arm_command()
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.timestamp = timestamp();

        msg.command =
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

        // param1 = 1 -> arm
        msg.param1 = 1.0f;

        set_command_target(msg);

        vehicle_command_pub_->publish(msg);
    }

    void set_command_target(px4_msgs::msg::VehicleCommand & msg)
    {
        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.confirmation = 0;

        // The command originates from an external controller.
        msg.from_external = true;
    }

    uint64_t timestamp() const
    {
        /*
         * PX4 timestamps are expressed in microseconds.
         */
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    // PX4 publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_mode_pub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    uint32_t setpoint_counter_;
    bool offboard_requested_;
    bool arm_requested_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardTakeoff>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}