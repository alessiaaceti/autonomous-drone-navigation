#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

class DroneController : public rclcpp::Node {
public:
    DroneController() : Node("drone_controller") {
        error_x_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_error_x", 10, std::bind(&DroneController::error_x_callback, this, std::placeholders::_1));
        
        error_y_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_error_y", 10, std::bind(&DroneController::error_y_callback, this, std::placeholders::_1));

        area_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_area", 10, std::bind(&DroneController::area_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&DroneController::odom_callback, this, std::placeholders::_1));

        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DroneController::publish_commands, this));
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    }

    void error_x_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        float kp_yaw = 0.002; 
        current_yaw_velocity_ = msg->data * kp_yaw;
    }

    // NUOVA FUNZIONE: Controlla la salita/discesa
    void error_y_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        float kp_z = 0.002; // Sensibilità per l'altezza
        current_z_velocity_ = msg->data * kp_z;
        
        // Limiti per non farlo schizzare in orbita
        if (current_z_velocity_ > 0.3) current_z_velocity_ = 0.3;
        if (current_z_velocity_ < -0.3) current_z_velocity_ = -0.3;
    }

    void area_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 0) { 
            current_forward_velocity_ = 0.0;
            current_z_velocity_ = 0.0; // Se perde il target, smette anche di salire/scendere
            return;
        }

        int target_area = 20000; 
        float kp_forward = 0.00002; 
        int area_error = target_area - msg->data;
        
        current_forward_velocity_ = area_error * kp_forward;

        if (current_forward_velocity_ > 0.2) current_forward_velocity_ = 0.2;
        if (current_forward_velocity_ < -0.2) current_forward_velocity_ = -0.2;
    }

    void publish_commands() {
        px4_msgs::msg::OffboardControlMode ocm{};
        ocm.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ocm.position = false;
        ocm.velocity = true;
        ocm.acceleration = false;
        offboard_mode_pub_->publish(ocm);

        px4_msgs::msg::TrajectorySetpoint ts{};
        ts.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        ts.position[0] = std::nanf("");
        ts.position[1] = std::nanf("");
        ts.position[2] = std::nanf("");
        ts.acceleration[0] = std::nanf("");
        ts.acceleration[1] = std::nanf("");
        ts.acceleration[2] = std::nanf("");
        ts.yaw = std::nanf("");
        
        ts.velocity[0] = current_forward_velocity_ * std::cos(current_yaw_);
        ts.velocity[1] = current_forward_velocity_ * std::sin(current_yaw_); 
        
        // IL DRONE ORA SALE E SCENDE!
        ts.velocity[2] = current_z_velocity_; 
        
        ts.yawspeed = current_yaw_velocity_; 
        
        trajectory_pub_->publish(ts);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr error_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr error_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr area_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float current_yaw_velocity_ = 0.0;
    float current_forward_velocity_ = 0.0;
    float current_z_velocity_ = 0.0; // NUOVA VARIABILE
    float current_yaw_ = 0.0; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
