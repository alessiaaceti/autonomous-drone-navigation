#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp> // Serve per inviare il comando di atterraggio
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
        
        // NUOVO: Publisher per i comandi di sistema (Atterraggio)
        command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

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
        current_error_x_ = msg->data;
        current_yaw_velocity_ = current_error_x_ * 0.002;
    }

    void error_y_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_error_y_ = msg->data;
        current_z_velocity_ = current_error_y_ * 0.002;
        
        if (current_z_velocity_ > 0.3) current_z_velocity_ = 0.3;
        if (current_z_velocity_ < -0.3) current_z_velocity_ = -0.3;
    }

    void area_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_area_ = msg->data;
        
        if (current_area_ == 0) { 
            current_forward_velocity_ = 0.0;
            return;
        }

        // Abbiamo aumentato l'area target e la velocità per farlo avvicinare!
        int target_area = 40000; 
        float kp_forward = 0.00004; 
        int area_error = target_area - current_area_;
        
        current_forward_velocity_ = area_error * kp_forward;

        if (current_forward_velocity_ > 0.3) current_forward_velocity_ = 0.3;
        if (current_forward_velocity_ < -0.3) current_forward_velocity_ = -0.3;
    }

    // Invia il comando di atterraggio a PX4
    void send_land_command() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        cmd.param1 = 0.0;
        cmd.param2 = 0.0;
        cmd.command = 21; // 21 è il codice ufficiale per VEHICLE_CMD_NAV_LAND
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        
        command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "BERSAGLIO RAGGIUNTO! Allineamento perfetto. Inizio atterraggio automatico...");
    }

    void publish_commands() {
        // Se stiamo già atterrando, smettiamo di inviare comandi di movimento e lasciamo fare a PX4
        if (is_landing_) {
            return; 
        }

        // LOGICA DI ATTERRAGGIO: Vicino (area > 35000) E centrato (errori X e Y vicini a zero)
        if (current_area_ > 35000 && std::abs(current_error_x_) < 30 && std::abs(current_error_y_) < 30) {
            send_land_command();
            is_landing_ = true;
            return;
        }

        // Se non stiamo atterrando, continuiamo a inseguire (Offboard Control)
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
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_; // Publisher per l'atterraggio
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Variabili per capire la situazione attuale
    int current_area_ = 0;
    int current_error_x_ = 0;
    int current_error_y_ = 0;
    bool is_landing_ = false; // Memoria: ci ricorda se abbiamo già dato l'ordine di atterrare
    
    float current_yaw_velocity_ = 0.0;
    float current_forward_velocity_ = 0.0;
    float current_z_velocity_ = 0.0; 
    float current_yaw_ = 0.0; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
