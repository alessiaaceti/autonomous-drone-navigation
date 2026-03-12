#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DroneVisionNode : public rclcpp::Node {
public:
    DroneVisionNode() : Node("drone_vision_node") {
        std::string topic_name = "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image";

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name, 
            rclcpp::SensorDataQoS(),
            std::bind(&DroneVisionNode::image_callback, this, std::placeholders::_1)
        );

        error_pub_ = this->create_publisher<std_msgs::msg::Int32>("/target_error_x", 10);

        // ECCO LA FRASE GIUSTA!
        RCLCPP_INFO(this->get_logger(), "Nodo Target Tracking avviato. Pubblico l'errore su /target_error_x");
    }

    ~DroneVisionNode() {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            cv::Mat hsv, mask;

            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            cv::Scalar lower_blue(90, 50, 50);
            cv::Scalar upper_blue(150, 255, 255);

            cv::inRange(hsv, lower_blue, upper_blue, mask);

            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

            cv::Moments m = cv::moments(mask, true);
            
            int cx_screen = frame.cols / 2;
            int cy_screen = frame.rows / 2;
            
            cv::drawMarker(frame, cv::Point(cx_screen, cy_screen), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);

            if (m.m00 > 1000) { 
                int cx_obj = m.m10 / m.m00;
                int cy_obj = m.m01 / m.m00;

                cv::circle(frame, cv::Point(cx_obj, cy_obj), 10, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(cx_screen, cy_screen), cv::Point(cx_obj, cy_obj), cv::Scalar(255, 0, 0), 2);
                
                int error_x = cx_obj - cx_screen;
                
                cv::putText(frame, "Tracking... Err X: " + std::to_string(error_x), 
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                std_msgs::msg::Int32 error_msg;
                error_msg.data = error_x;
                error_pub_->publish(error_msg);

            } else {
                cv::putText(frame, "Target PERSO", 
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            cv::imshow("Drone Camera (Target Tracking)", frame);
            cv::imshow("Maschera HSV", mask);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub_; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneVisionNode>());
    rclcpp::shutdown();
    return 0;
}
