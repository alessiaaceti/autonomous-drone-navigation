#include <cmath>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;

class OffboardAvoidance : public rclcpp::Node
{
public:
    OffboardAvoidance()
        : Node("offboard_avoidance"),
          current_x_(0.0f),
          current_y_(0.0f),
          current_z_(0.0f),
          target_x_(0.0f),
          target_y_(0.0f),
          target_z_(-2.5f),
          position_received_(false),
          target_initialized_(false),
          command_("STOP"),
          offboard_counter_(0)
    {
        // ------------------------------------------------------------
        // Subscribers
        // ------------------------------------------------------------

        avoidance_subscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "/avoidance_command",
                10,
                std::bind(
                    &OffboardAvoidance::avoidance_callback,
                    this,
                    std::placeholders::_1));

        position_subscription_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &OffboardAvoidance::position_callback,
                    this,
                    std::placeholders::_1));

        // ------------------------------------------------------------
        // Publishers
        // ------------------------------------------------------------

        offboard_control_mode_publisher_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode",
                10);

        trajectory_setpoint_publisher_ =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint",
                10);

        vehicle_command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                10);

        timer_ =
            this->create_wall_timer(
                50ms,
                std::bind(
                    &OffboardAvoidance::control_loop,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Offboard avoidance controller started.");
    }

private:

    // ------------------------------------------------------------
    // Receive avoidance command
    // ------------------------------------------------------------

    void avoidance_callback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string new_command = msg->data;

        if (new_command == command_)
        {
            return;
        }

        command_ = new_command;

        if (!position_received_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Avoidance command received before PX4 position.");
            return;
        }

        update_target_from_command();

        RCLCPP_INFO(
            this->get_logger(),
            "Command: %s | Target: X=%.2f Y=%.2f Z=%.2f",
            command_.c_str(),
            target_x_,
            target_y_,
            target_z_);
    }

    // ------------------------------------------------------------
    // Receive PX4 local position
    // ------------------------------------------------------------

    void position_callback(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;

        position_received_ = true;

        // Initialize the first target from the actual
        // drone position.
        if (!target_initialized_)
        {
            target_x_ = current_x_;
            target_y_ = current_y_;
            target_z_ = -2.5f;

            target_initialized_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Initial target: X=%.2f Y=%.2f Z=%.2f",
                target_x_,
                target_y_,
                target_z_);
        }
    }

    // ------------------------------------------------------------
    // Convert avoidance command into a new position target
    // ------------------------------------------------------------

    void update_target_from_command()
    {
        const float step = 1.0f;

        if (command_ == "FORWARD")
        {
            target_x_ = current_x_ + step;
            target_y_ = current_y_;
        }
        else if (command_ == "MOVE_LEFT")
        {
            target_x_ = current_x_;
            target_y_ = current_y_ - step;
        }
        else if (command_ == "MOVE_RIGHT")
        {
            target_x_ = current_x_;
            target_y_ = current_y_ + step;
        }
        else if (command_ == "STOP")
        {
            target_x_ = current_x_;
            target_y_ = current_y_;
        }
        else if (command_ == "CHOOSE_SIDE")
        {
            target_x_ = current_x_;
            target_y_ = current_y_;
        }
    }

    // ------------------------------------------------------------
    // Publish Offboard control mode
    // ------------------------------------------------------------

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        offboard_control_mode_publisher_->publish(msg);
    }

    // ------------------------------------------------------------
    // Publish current trajectory target
    // ------------------------------------------------------------

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = {
            target_x_,
            target_y_,
            target_z_
        };

        msg.yaw = NAN;

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        trajectory_setpoint_publisher_->publish(msg);
    }

    // ------------------------------------------------------------
    // Send PX4 vehicle command
    // ------------------------------------------------------------

    void publish_vehicle_command(
        uint16_t command,
        float param1 = 0.0f,
        float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;

        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;

        msg.from_external = true;

        msg.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        vehicle_command_publisher_->publish(msg);
    }

    // ------------------------------------------------------------
    // Request Offboard mode
    // ------------------------------------------------------------

    void set_offboard_mode()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,
            6.0f);
    }

    // ------------------------------------------------------------
    // Arm
    // ------------------------------------------------------------

    void arm()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f);
    }

    // ------------------------------------------------------------
    // Main control loop
    // ------------------------------------------------------------

    void control_loop()
    {
        publish_offboard_control_mode();

        if (!position_received_)
        {
            return;
        }

        publish_trajectory_setpoint();

        // PX4 requires a stream of setpoints before
        // entering Offboard mode.
        if (offboard_counter_ < 20)
        {
            offboard_counter_++;
            return;
        }

        if (offboard_counter_ == 20)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Requesting Offboard mode.");

            set_offboard_mode();

            RCLCPP_INFO(
                this->get_logger(),
                "Arming vehicle.");

            arm();
        }

        offboard_counter_++;
    }

    // ------------------------------------------------------------
    // ROS interfaces
    // ------------------------------------------------------------

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        avoidance_subscription_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        position_subscription_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_publisher_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_publisher_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------------
    // Current PX4 position
    // ------------------------------------------------------------

    float current_x_;
    float current_y_;
    float current_z_;

    // ------------------------------------------------------------
    // Desired position
    // ------------------------------------------------------------

    float target_x_;
    float target_y_;
    float target_z_;

    bool position_received_;
    bool target_initialized_;

    std::string command_;

    int offboard_counter_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<OffboardAvoidance>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}