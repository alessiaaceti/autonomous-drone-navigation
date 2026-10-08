#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include "drone_navigation_cpp/msg/target_state.hpp"

class YoloTargetTracker : public rclcpp::Node
{
public:
    YoloTargetTracker()
        : Node("yolo_target_tracker")
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

        RCLCPP_INFO(
            this->get_logger(),
            "YOLO Target Tracker started.");
    }

private:
    void detection_callback(
        const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        drone_navigation_cpp::msg::TargetState target_msg;

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

        float best_confidence = -std::numeric_limits<float>::infinity();
        const vision_msgs::msg::Detection2D *best_detection = nullptr;

        for (const auto &detection : msg->detections)
        {
            if (detection.results.empty())
            {
                continue;
            }

            const auto &hypothesis = detection.results[0].hypothesis;

            const std::string &class_name = hypothesis.class_id;
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

            if (confidence > best_confidence)
            {
                best_confidence = confidence;
                best_detection = &detection;
            }
        }

        if (best_detection == nullptr)
        {
            target_pub_->publish(target_msg);
            return;
        }

        const auto &bbox = best_detection->bbox;

        const float center_x =
            static_cast<float>(bbox.center.position.x);

        const float center_y =
            static_cast<float>(bbox.center.position.y);

        const float area =
            static_cast<float>(bbox.size_x * bbox.size_y);

        const float image_center_x = image_width_ / 2.0f;
        const float image_center_y = image_height_ / 2.0f;

        target_msg.detected = true;
        target_msg.class_name =
            best_detection->results[0].hypothesis.class_id;

        target_msg.confidence = best_confidence;

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
            "TARGET STATE: class=%s confidence=%.3f error_x=%.1f error_y=%.1f area=%.1f",
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

    bool is_target_class(const std::string &class_name) const
    {
        return class_name == "pedestrian" ||
               class_name == "car" ||
               class_name == "truck";
    }

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detection_sub_;

    rclcpp::Publisher<drone_navigation_cpp::msg::TargetState>::SharedPtr
        target_pub_;

    float image_width_;
    float image_height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloTargetTracker>());

    rclcpp::shutdown();

    return 0;
}
