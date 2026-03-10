#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;

class OffboardPath : public rclcpp::Node
{
public:
    OffboardPath(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("offboard_path", options)
    {
        // Publisher PX4
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // Path quadrato (x, y, z)
        path_ = {
            {0.0, 0.0, -2.5},
            {3.0, 0.0, -2.5},
            {3.0, 3.0, -2.5},
            {0.0, 3.0, -2.5},
            {0.0, 0.0, -2.5}
        };
        current_wp_ = 0;

        // Inizializzo tempo dell'ultimo waypoint
        last_wp_time_ = this->get_clock()->now();

        // Timer 100ms
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardPath::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Offboard Path node started!");
    }

private:
    void timer_callback()
    {
        // Pubblica Offboard mode
        px4_msgs::msg::OffboardControlMode offboard_msg;
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_mode_pub_->publish(offboard_msg);

        // Pubblica setpoint corrente
        px4_msgs::msg::TrajectorySetpoint traj_msg;
        traj_msg.position[0] = path_[current_wp_][0];
        traj_msg.position[1] = path_[current_wp_][1];
        traj_msg.position[2] = path_[current_wp_][2];
        traj_msg.yaw = 0.0;
        traj_pub_->publish(traj_msg);

        // Cambio waypoint ogni 2 secondi
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_wp_time_).seconds() > 2.0) {
            current_wp_ = (current_wp_ + 1) % path_.size();
            last_wp_time_ = now;
        }

        // Arma e setta offboard una sola volta
        if (!armed_) {
            arm_and_offboard();
            armed_ = true;
        }
    }

    void arm_and_offboard()
    {
        // Arm
        px4_msgs::msg::VehicleCommand arm_cmd;
        arm_cmd.timestamp = this->now().nanoseconds();
        arm_cmd.param1 = 1.0; // arm
        arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        arm_cmd.target_system = 1;
        arm_cmd.target_component = 1;
        arm_cmd.source_system = 1;
        arm_cmd.source_component = 1;
        arm_cmd.confirmation = 0;
        vehicle_command_pub_->publish(arm_cmd);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");

        // Offboard mode
        px4_msgs::msg::VehicleCommand offboard_cmd;
        offboard_cmd.timestamp = this->now().nanoseconds();
        offboard_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        offboard_cmd.param1 = 1; // custom mode
        offboard_cmd.param2 = 6; // OFFBOARD
        offboard_cmd.target_system = 1;
        offboard_cmd.target_component = 1;
        vehicle_command_pub_->publish(offboard_cmd);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }

    // Publishers e timer
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool armed_ = false;
    std::vector<std::vector<float>> path_;
    size_t current_wp_;
    rclcpp::Time last_wp_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Crea un nodo con parametro use_sim_time = true
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    auto node = std::make_shared<OffboardPath>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
