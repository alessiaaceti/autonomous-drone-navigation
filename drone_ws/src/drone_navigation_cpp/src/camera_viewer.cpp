#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer() : Node("camera_viewer")
    {
        // Sottoscrizione al topic bridgato Gazebo -> ROS2
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/world/default/model/x500_0/link/base_link/sensor/front_camera/image",
            10,
            std::bind(&CameraViewer::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera Viewer node started.");
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Converte il messaggio ROS Image in cv::Mat
            auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

            // Se il topic è in RGB, convertiamo in BGR per OpenCV
            if (msg->encoding == "rgb8")
            {
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
            }

            // Mostra l'immagine in finestra OpenCV
            cv::imshow("Drone Camera", cv_ptr->image);
            cv::waitKey(1); // Aggiornamento finestra
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraViewer>();

    // Spin del nodo
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
