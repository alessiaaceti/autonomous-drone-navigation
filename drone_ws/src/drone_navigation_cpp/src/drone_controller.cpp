#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

class DroneController : public rclcpp::Node {
public:
    DroneController() : Node("drone_controller") {
        error_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_error_x", 10, std::bind(&DroneController::error_callback, this, std::placeholders::_1));

        // OffboardControlMode senza underscore
        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DroneController::publish_commands, this));
    }

private:
    void error_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int error_x = msg->data;
        float kp = 0.005; 
        current_yaw_velocity_ = - (error_x * kp); 
    }

    void publish_commands() {
        // Pubblichiamo la modalità Offboard
        px4_msgs::msg::OffboardControlMode ocm{};
        ocm.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ocm.position = false;
        ocm.velocity = true;
        ocm.acceleration = false;
        offboard_mode_pub_->publish(ocm);

        // Usiamo l'array velocity[3] invece di vx, vy, vz
        px4_msgs::msg::TrajectorySetpoint ts{};
        ts.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ts.velocity[0] = 0.0; // vx
        ts.velocity[1] = 0.0; // vy
        ts.velocity[2] = 0.0; // vz
        ts.yawspeed = current_yaw_velocity_; 
        
        trajectory_pub_->publish(ts);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr error_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float current_yaw_velocity_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
