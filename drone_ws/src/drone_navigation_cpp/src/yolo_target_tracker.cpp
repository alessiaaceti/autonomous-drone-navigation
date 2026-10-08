#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <string>
#include <limits>
#include <cmath>
#include <functional>

#include "drone_navigation_cpp/msg/target_state.hpp"

class YoloTargetTracker : public rclcpp::Node
{
public:
    YoloTargetTracker()
        : Node("yolo_target_tracker"),
          consecutive_detections_(0)
    {
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/yolo/detections",
            10,
            std::bind(
                &YoloTargetTracker::detection_callback,
                this,
                std::placeholders::_1));

        target_pub_ = this->create_publisher<drone_navigation_cpp::msg::TargetState>(
            "/target_state",
            10);

        // Gazebo X500 camera resolution.
        image_width_ = 1920.0f;
        image_height_ = 1080.0f;

        // Defense-in-depth: never accept weak detections.
        min_confidence_ = 0.10f;

        // Temporal persistence parameters.
        this->declare_parameter("confirm_frames", 3);
        this->declare_parameter("confirm_window_ms", 500);

        confirm_frames_ =
            this->get_parameter("confirm_frames").as_int();

        confirm_window_ms_ =
            this->get_parameter("confirm_window_ms").as_int();

        if (confirm_frames_ < 1)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "confirm_frames must be >= 1. Using 1.");
            confirm_frames_ = 1;
        }

        if (confirm_window_ms_ < 1)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "confirm_window_ms must be >= 1 ms. Using 1 ms.");
            confirm_window_ms_ = 1;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "YOLO Target Tracker started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Minimum confidence: %.2f",
            min_confidence_);

        RCLCPP_INFO(
            this->get_logger(),
            "Target confirmation: %d consecutive detections within %d ms",
            confirm_frames_,
            confirm_window_ms_);
    }

private:
    void reset_confirmation()
    {
        consecutive_detections_ = 0;
        first_detection_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    bool update_confirmation(const rclcpp::Time &detection_time)
    {
        if (consecutive_detections_ == 0)
        {
            first_detection_time_ = detection_time;
            consecutive_detections_ = 1;
            return false;
        }

        const double elapsed_ms =
            (detection_time - first_detection_time_).seconds() * 1000.0;

        if (elapsed_ms > static_cast<double>(confirm_window_ms_))
        {
            first_detection_time_ = detection_time;
            consecutive_detections_ = 1;
            return false;
        }

        ++consecutive_detections_;

        return consecutive_detections_ >= confirm_frames_;
    }

    void detection_callback(
        const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        drone_navigation_cpp::msg::TargetState target_msg;

        // Propagate the timestamp and frame from the YOLO detection.
        target_msg.header = msg->header;

        target_msg.detected = false;
        target_msg.class_name = "";
        target_msg.confidence = 0.0f;
        target_msg.error_x = 0.0f;
        target_msg.error_y = 0.0f;
        target_msg.area = 0.0f;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received %zu YOLO detections",
            msg->detections.size());

        float best_confidence =
            -std::numeric_limits<float>::infinity();

        const vision_msgs::msg::Detection2D *best_detection = nullptr;

        for (const auto &detection : msg->detections)
        {
            if (detection.results.empty())
            {
                continue;
            }

            const auto &hypothesis =
                detection.results[0].hypothesis;

            const std::string &class_name =
                hypothesis.class_id;

            const float confidence =
                static_cast<float>(hypothesis.score);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "YOLO detection: class=%s confidence=%.3f",
                class_name.c_str(),
                confidence);

            if (!is_target_class(class_name))
            {
                continue;
            }

            // Defense-in-depth confidence gate.
            if (confidence < min_confidence_)
            {
                continue;
            }

            if (confidence > best_confidence)
            {
                best_confidence = confidence;
                best_detection = &detection;
            }
        }

        // No valid target in this frame.
        if (best_detection == nullptr)
        {
            reset_confirmation();

            target_pub_->publish(target_msg);
            return;
        }

        const auto &bbox = best_detection->bbox;

        const float center_x =
            static_cast<float>(bbox.center.position.x);

        const float center_y =
            static_cast<float>(bbox.center.position.y);

        const float area =
            static_cast<float>(
                bbox.size_x * bbox.size_y);

        const float image_center_x =
            image_width_ / 2.0f;

        const float image_center_y =
            image_height_ / 2.0f;

        const rclcpp::Time detection_time =
            rclcpp::Time(msg->header.stamp);

        const bool confirmed =
            update_confirmation(detection_time);

        if (!confirmed)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Target candidate: %d/%d confirmations",
                consecutive_detections_,
                confirm_frames_);

            target_pub_->publish(target_msg);
            return;
        }

        target_msg.detected = true;

        target_msg.class_name =
            best_detection->results[0].hypothesis.class_id;

        target_msg.confidence =
            best_confidence;

        target_msg.error_x =
            center_x - image_center_x;

        target_msg.error_y =
            center_y - image_center_y;

        target_msg.area = area;

        target_pub_->publish(target_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "TARGET CONFIRMED: class=%s confidence=%.3f error_x=%.1f error_y=%.1f area=%.1f",
            target_msg.class_name.c_str(),
            target_msg.confidence,
            target_msg.error_x,
            target_msg.error_y,
            target_msg.area);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Target: %s | confidence: %.2f | error_x: %.1f | error_y: %.1f | area: %.1f",
            target_msg.class_name.c_str(),
            target_msg.confidence,
            target_msg.error_x,
            target_msg.error_y,
            target_msg.area);
    }

    bool is_target_class(
        const std::string &class_name) const
    {
        return class_name == "pedestrian" ||
               class_name == "car" ||
               class_name == "van" ||
               class_name == "truck" ||
               class_name == "bus";
    }

    rclcpp::Subscription<
        vision_msgs::msg::Detection2DArray>::SharedPtr
        detection_sub_;

    rclcpp::Publisher<
        drone_navigation_cpp::msg::TargetState>::SharedPtr
        target_pub_;

    float image_width_;
    float image_height_;
    float min_confidence_;

    int confirm_frames_;
    int confirm_window_ms_;

    int consecutive_detections_;

    rclcpp::Time first_detection_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloTargetTracker>());

    rclcpp::shutdown();

    return 0;
}
