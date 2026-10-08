#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "drone_navigation_cpp/msg/decision_state.hpp"


class DroneController : public rclcpp::Node
{
public:
    DroneController()
        : Node("drone_controller")
    {
        decision_sub_ =
            this->create_subscription<
                drone_navigation_cpp::msg::DecisionState>(
                "/decision",
                10,
                std::bind(
                    &DroneController::decision_callback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &DroneController::odom_callback,
                    this,
                    std::placeholders::_1));

        offboard_mode_pub_ =
            this->create_publisher<
                px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode",
                10);

        trajectory_pub_ =
            this->create_publisher<
                px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint",
                10);

        command_pub_ =
            this->create_publisher<
                px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                10);

        decision_timeout_ms_ = 500.0;

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &DroneController::publish_commands,
                    this));

        RCLCPP_INFO(
            this->get_logger(),
            "Drone Controller started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Decision Layer is the movement safety gate.");
    }

private:

    void decision_callback(
        const drone_navigation_cpp::msg::DecisionState::SharedPtr msg)
    {
        last_decision_time_ = this->get_clock()->now();
        decision_received_ = true;

        current_mode_ = msg->mode;
        current_safety_override_ = msg->safety_override;

        if (msg->mode == "TRACK_TARGET")
        {
            current_target_detected_ =
                msg->target_detected;

            current_error_x_ =
                static_cast<int>(msg->target_error_x);

            current_error_y_ =
                static_cast<int>(msg->target_error_y);

            current_area_ =
                static_cast<int>(msg->target_area);

            update_target_control();
        }
        else
        {
            current_target_detected_ = false;

            current_error_x_ = 0;
            current_error_y_ = 0;
            current_area_ = 0;

            current_yaw_velocity_ = 0.0f;
            current_forward_velocity_ = 0.0f;
            current_z_velocity_ = 0.0f;
        }

        if (current_mode_ != last_logged_mode_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Controller mode: %s | safety_override=%s",
                current_mode_.c_str(),
                current_safety_override_ ? "true" : "false");

            last_logged_mode_ = current_mode_;
        }
    }


    void update_target_control()
    {
        current_yaw_velocity_ =
            static_cast<float>(current_error_x_) * 0.002f;

        current_z_velocity_ =
            static_cast<float>(current_error_y_) * 0.002f;

        if (current_z_velocity_ > 0.3f)
        {
            current_z_velocity_ = 0.3f;
        }

        if (current_z_velocity_ < -0.3f)
        {
            current_z_velocity_ = -0.3f;
        }


        if (current_area_ == 0)
        {
            current_forward_velocity_ = 0.0f;
            return;
        }


        const int target_area = 40000;

        const float kp_forward = 0.00004f;

        const int area_error =
            target_area - current_area_;

        current_forward_velocity_ =
            static_cast<float>(area_error) * kp_forward;

        if (current_forward_velocity_ > 0.3f)
        {
            current_forward_velocity_ = 0.3f;
        }

        if (current_forward_velocity_ < -0.3f)
        {
            current_forward_velocity_ = -0.3f;
        }
    }


    void odom_callback(
        const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        const float q0 = msg->q[0];
        const float q1 = msg->q[1];
        const float q2 = msg->q[2];
        const float q3 = msg->q[3];

        current_yaw_ =
            std::atan2(
                2.0f * (q0 * q3 + q1 * q2),
                1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }


    void send_land_command()
    {
        px4_msgs::msg::VehicleCommand cmd{};

        cmd.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        cmd.param1 = 0.0f;
        cmd.param2 = 0.0f;

        cmd.command = 21;

        cmd.target_system = 1;
        cmd.target_component = 1;

        cmd.source_system = 1;
        cmd.source_component = 1;

        cmd.confirmation = 0;
        cmd.from_external = true;

        command_pub_->publish(cmd);

        RCLCPP_INFO(
            this->get_logger(),
            "TARGET REACHED: initiating automatic landing.");
    }


    void publish_commands()
    {
        /*
         * ----------------------------------------------------
         * SAFETY GATE
         * ----------------------------------------------------
         *
         * Only TRACK_TARGET is currently allowed to generate
         * movement commands.
         *
         * STOP, AVOID_OBSTACLE, SEARCH_TARGET and unknown
         * states all produce zero velocity.
         */
        /*
         * ----------------------------------------------------
         * DECISION WATCHDOG
         * ----------------------------------------------------
         *
         * If the Decision Layer stops publishing, the
         * controller must not keep executing the last command.
         */
        if (!decision_received_)
        {
            publish_zero_velocity();
            return;
        }

        const double decision_age_ms =
            (this->get_clock()->now() - last_decision_time_).seconds()
            * 1000.0;

        if (decision_age_ms > decision_timeout_ms_)
        {
            publish_zero_velocity();
            return;
        }


        if (current_mode_ != "TRACK_TARGET" ||
            current_safety_override_ ||
            !current_target_detected_)
        {
            publish_zero_velocity();
            return;
        }


        /*
         * LANDING CONDITION
         *
         * Close enough to the target and sufficiently centered.
         */
        if (current_area_ > 35000 &&
            std::abs(current_error_x_) < 30 &&
            std::abs(current_error_y_) < 30)
        {
            if (!is_landing_)
            {
                send_land_command();
                is_landing_ = true;
            }

            return;
        }


        if (is_landing_)
        {
            return;
        }


        px4_msgs::msg::OffboardControlMode ocm{};

        ocm.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        ocm.position = false;
        ocm.velocity = true;
        ocm.acceleration = false;

        offboard_mode_pub_->publish(ocm);


        px4_msgs::msg::TrajectorySetpoint ts{};

        ts.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        ts.position[0] = std::nanf("");
        ts.position[1] = std::nanf("");
        ts.position[2] = std::nanf("");

        ts.acceleration[0] = std::nanf("");
        ts.acceleration[1] = std::nanf("");
        ts.acceleration[2] = std::nanf("");

        ts.yaw = std::nanf("");


        ts.velocity[0] =
            current_forward_velocity_ *
            std::cos(current_yaw_);

        ts.velocity[1] =
            current_forward_velocity_ *
            std::sin(current_yaw_);

        ts.velocity[2] =
            current_z_velocity_;

        ts.yawspeed =
            current_yaw_velocity_;


        trajectory_pub_->publish(ts);
    }


    void publish_zero_velocity()
    {
        px4_msgs::msg::OffboardControlMode ocm{};

        ocm.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        ocm.position = false;
        ocm.velocity = true;
        ocm.acceleration = false;

        offboard_mode_pub_->publish(ocm);


        px4_msgs::msg::TrajectorySetpoint ts{};

        ts.timestamp =
            this->get_clock()->now().nanoseconds() / 1000;

        ts.position[0] = std::nanf("");
        ts.position[1] = std::nanf("");
        ts.position[2] = std::nanf("");

        ts.acceleration[0] = std::nanf("");
        ts.acceleration[1] = std::nanf("");
        ts.acceleration[2] = std::nanf("");

        ts.velocity[0] = 0.0f;
        ts.velocity[1] = 0.0f;
        ts.velocity[2] = 0.0f;

        ts.yaw = std::nanf("");
        ts.yawspeed = 0.0f;

        trajectory_pub_->publish(ts);
    }


    rclcpp::Subscription<
        drone_navigation_cpp::msg::DecisionState>::SharedPtr
        decision_sub_;

    rclcpp::Subscription<
        px4_msgs::msg::VehicleOdometry>::SharedPtr
        odom_sub_;

    rclcpp::Publisher<
        px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_mode_pub_;

    rclcpp::Publisher<
        px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_pub_;

    rclcpp::Publisher<
        px4_msgs::msg::VehicleCommand>::SharedPtr
        command_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


    std::string current_mode_ = "STOP";
    std::string last_logged_mode_ = "";

    bool decision_received_ = false;
    rclcpp::Time last_decision_time_;
    double decision_timeout_ms_ = 500.0;

    bool current_safety_override_ = true;
    bool current_target_detected_ = false;

    int current_area_ = 0;
    int current_error_x_ = 0;
    int current_error_y_ = 0;

    bool is_landing_ = false;

    float current_yaw_velocity_ = 0.0f;
    float current_forward_velocity_ = 0.0f;
    float current_z_velocity_ = 0.0f;
    float current_yaw_ = 0.0f;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<DroneController>());

    rclcpp::shutdown();

    return 0;
}
