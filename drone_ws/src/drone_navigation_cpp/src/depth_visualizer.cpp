#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

class DepthVisualizer : public rclcpp::Node
{
public:
    DepthVisualizer()
        : Node("depth_visualizer"),
          obstacle_threshold_(1.5f)
    {
        depth_subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/depth_camera",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &DepthVisualizer::depth_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Depth visualizer started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle threshold: %.2f m",
            obstacle_threshold_);
    }

private:
    void depth_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::TYPE_32FC1);

            const cv::Mat &depth = cv_ptr->image;

            if (depth.empty())
            {
                return;
            }

            const double max_depth = 10.0;

            // Normalize depth for visualization.
            cv::Mat normalized;

            depth.convertTo(
                normalized,
                CV_8UC1,
                -255.0 / max_depth,
                255.0);

            // Convert grayscale depth to color.
            cv::Mat visualization;

            cv::applyColorMap(
                normalized,
                visualization,
                cv::COLORMAP_JET);

            const int width = depth.cols;
            const int height = depth.rows;

            // Mark invalid pixels as black.
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    const float distance =
                        depth.at<float>(y, x);

                    if (!std::isfinite(distance) ||
                        distance <= 0.05f)
                    {
                        visualization.at<cv::Vec3b>(y, x) =
                            cv::Vec3b(0, 0, 0);
                    }
                }
            }

            const int sector_width = width / 3;

            // Draw sector boundaries.
            cv::line(
                visualization,
                cv::Point(sector_width, 0),
                cv::Point(sector_width, height),
                cv::Scalar(255, 255, 255),
                2);

            cv::line(
                visualization,
                cv::Point(sector_width * 2, 0),
                cv::Point(sector_width * 2, height),
                cv::Scalar(255, 255, 255),
                2);

            // Highlight pixels closer than the obstacle threshold.
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    const float distance =
                        depth.at<float>(y, x);

                    if (std::isfinite(distance) &&
                        distance > 0.05f &&
                        distance <= obstacle_threshold_)
                    {
                        visualization.at<cv::Vec3b>(y, x) =
                            cv::Vec3b(255, 255, 255);
                    }
                }
            }

            // Sector labels.
            cv::putText(
                visualization,
                "LEFT",
                cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(255, 255, 255),
                2);

            cv::putText(
                visualization,
                "CENTER",
                cv::Point(sector_width + 20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(255, 255, 255),
                2);

            cv::putText(
                visualization,
                "RIGHT",
                cv::Point(sector_width * 2 + 20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(255, 255, 255),
                2);

            // Display threshold information.
            std::ostringstream threshold_text;

            threshold_text
                << "Obstacle threshold: "
                << std::fixed
                << std::setprecision(1)
                << obstacle_threshold_
                << " m";

            cv::putText(
                visualization,
                threshold_text.str(),
                cv::Point(20, height - 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(255, 255, 255),
                2);

            cv::imshow(
                "Drone Depth - Obstacle Visualization",
                visualization);

            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge error: %s",
                e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        depth_subscription_;

    float obstacle_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<DepthVisualizer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}