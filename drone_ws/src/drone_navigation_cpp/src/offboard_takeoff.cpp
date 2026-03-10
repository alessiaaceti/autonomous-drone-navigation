#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;

class OffboardTakeoff : public rclcpp::Node
{
public:
    OffboardTakeoff() : Node("offboard_takeoff")
    {
        // Publisher per i comandi
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardTakeoff::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Offboard Takeoff node started!");
    }

private:
    void timer_callback()
    {
        // Pubblica OFFBOARD mode continuamente
        px4_msgs::msg::OffboardControlMode offboard_msg;
        offboard_msg.position = true;   // vogliamo controllare posizione
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_mode_pub_->publish(offboard_msg);

        // Inizializza e pubblica un setpoint (hover)
        px4_msgs::msg::TrajectorySetpoint traj_msg;
        traj_msg.position[0] = 0.0;   // x
	traj_msg.position[1] = 0.0;   // y
	traj_msg.position[2] = -2.5;  // z (negative = up)
	traj_msg.yaw = 0.0;
        traj_pub_->publish(traj_msg);

        // Arm & Offboard solo la prima volta
        if (!armed_) {
            arm();
            armed_ = true;
        }
    }

    void arm()
    {
        // Comando Arm
        px4_msgs::msg::VehicleCommand arm_cmd;
        arm_cmd.timestamp = this->now().nanoseconds();
        arm_cmd.param1 = 1.0; // 1 = arm
        arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        arm_cmd.target_system = 1;
        arm_cmd.target_component = 1;
        arm_cmd.source_system = 1;
        arm_cmd.source_component = 1;
        arm_cmd.confirmation = 0;
        vehicle_command_pub_->publish(arm_cmd);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");

        // Modalità Offboard
        px4_msgs::msg::VehicleCommand offboard_cmd;
        offboard_cmd.timestamp = this->now().nanoseconds();
        offboard_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        offboard_cmd.param1 = 1;   // custom mode
        offboard_cmd.param2 = 6;   // OFFBOARD
        offboard_cmd.target_system = 1;
        offboard_cmd.target_component = 1;
        vehicle_command_pub_->publish(offboard_cmd);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool armed_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardTakeoff>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
