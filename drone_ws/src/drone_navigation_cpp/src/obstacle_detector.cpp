#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>


class ObstacleDetector : public rclcpp::Node
{
public:

    ObstacleDetector()
        : Node("obstacle_detector"),
          obstacle_threshold_(1.5f),
          occupancy_threshold_(5.0f),
          min_valid_percent_(20.0f)
    {
        depth_subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/depth_camera",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ObstacleDetector::depth_callback,
                    this,
                    std::placeholders::_1));

        // Distance publishers
        left_distance_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_distance_left", 10);

        center_distance_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_distance_center", 10);

        right_distance_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_distance_right", 10);

        // Occupancy publishers
        left_occupancy_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_occupancy_left", 10);

        center_occupancy_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_occupancy_center", 10);

        right_occupancy_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(
                "/obstacle_occupancy_right", 10);

        // Final obstacle state
        state_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/obstacle_state", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle detector started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Distance threshold: %.2f m",
            obstacle_threshold_);

        RCLCPP_INFO(
            this->get_logger(),
            "Occupancy threshold: %.1f%%",
            occupancy_threshold_);

        RCLCPP_INFO(
            this->get_logger(),
            "Minimum valid depth: %.1f%%",
            min_valid_percent_);
    }


private:

    struct SectorData
    {
        float median_distance;
        float occupancy_percent;
        float valid_percent;
    };


    SectorData analyze_sector(
        const cv::Mat &depth,
        int x_start,
        int x_end,
        int y_start,
        int y_end)
    {
        std::vector<float> distances;

        const int total_pixels =
            (x_end - x_start) *
            (y_end - y_start);

        distances.reserve(total_pixels);

        int valid_pixels = 0;
        int obstacle_pixels = 0;

        for (int y = y_start; y < y_end; ++y)
        {
            for (int x = x_start; x < x_end; ++x)
            {
                const float distance =
                    depth.at<float>(y, x);

                // Ignore invalid depth values.
                if (!std::isfinite(distance))
                {
                    continue;
                }

                // Ignore zero and extremely small values.
                if (distance <= 0.05f)
                {
                    continue;
                }

                valid_pixels++;
                distances.push_back(distance);

                // Count pixels that see an obstacle.
                if (distance <= obstacle_threshold_)
                {
                    obstacle_pixels++;
                }
            }
        }

        const float valid_percent =
            100.0f *
            static_cast<float>(valid_pixels) /
            static_cast<float>(total_pixels);

        // No valid depth data.
        if (distances.empty())
        {
            return {
                std::numeric_limits<float>::quiet_NaN(),
                0.0f,
                0.0f
            };
        }

        // Calculate median distance.
        const std::size_t middle =
            distances.size() / 2;

        std::nth_element(
            distances.begin(),
            distances.begin() + middle,
            distances.end());

        const float median_distance =
            distances[middle];

        // Calculate percentage of valid pixels
        // that see an obstacle.
        const float occupancy_percent =
            100.0f *
            static_cast<float>(obstacle_pixels) /
            static_cast<float>(valid_pixels);

        return {
            median_distance,
            occupancy_percent,
            valid_percent
        };
    }


    void depth_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::TYPE_32FC1);

            const cv::Mat &depth =
                cv_ptr->image;

            if (depth.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Received an empty depth image.");

                publish_unknown_state();
                return;
            }

            const int width = depth.cols;
            const int height = depth.rows;

            if (width < 3 || height < 3)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Depth image is too small.");

                publish_unknown_state();
                return;
            }

            // Ignore the upper and lower parts of the image.
            // This focuses detection on the area directly in front
            // of the drone.
            const int y_start = height / 3;
            const int y_end = (height * 2) / 3;

            // Divide the image into three horizontal sectors.
            const int sector_width = width / 3;

            const SectorData left =
                analyze_sector(
                    depth,
                    0,
                    sector_width,
                    y_start,
                    y_end);

            const SectorData center =
                analyze_sector(
                    depth,
                    sector_width,
                    sector_width * 2,
                    y_start,
                    y_end);

            const SectorData right =
                analyze_sector(
                    depth,
                    sector_width * 2,
                    width,
                    y_start,
                    y_end);


            // ------------------------------------------------
            // Publish distances
            // ------------------------------------------------

            std_msgs::msg::Float32 left_distance_msg;
            left_distance_msg.data =
                left.median_distance;
            left_distance_pub_->publish(
                left_distance_msg);

            std_msgs::msg::Float32 center_distance_msg;
            center_distance_msg.data =
                center.median_distance;
            center_distance_pub_->publish(
                center_distance_msg);

            std_msgs::msg::Float32 right_distance_msg;
            right_distance_msg.data =
                right.median_distance;
            right_distance_pub_->publish(
                right_distance_msg);


            // ------------------------------------------------
            // Publish occupancy percentages
            // ------------------------------------------------

            std_msgs::msg::Float32 left_occupancy_msg;
            left_occupancy_msg.data =
                left.occupancy_percent;
            left_occupancy_pub_->publish(
                left_occupancy_msg);

            std_msgs::msg::Float32 center_occupancy_msg;
            center_occupancy_msg.data =
                center.occupancy_percent;
            center_occupancy_pub_->publish(
                center_occupancy_msg);

            std_msgs::msg::Float32 right_occupancy_msg;
            right_occupancy_msg.data =
                right.occupancy_percent;
            right_occupancy_pub_->publish(
                right_occupancy_msg);


            // ------------------------------------------------
            // Validate depth quality
            // ------------------------------------------------

            const bool left_valid =
                left.valid_percent >= min_valid_percent_;

            const bool center_valid =
                center.valid_percent >= min_valid_percent_;

            const bool right_valid =
                right.valid_percent >= min_valid_percent_;

            /*
             * If any sector contains too little valid depth,
             * we do not trust the obstacle classification.
             *
             * This is deliberately conservative:
             * insufficient perception data must not become CLEAR.
             */
            if (!left_valid ||
                !center_valid ||
                !right_valid)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "Insufficient valid depth data. "
                    "Publishing UNKNOWN.");

                publish_unknown_state();
                return;
            }


            // ------------------------------------------------
            // Determine obstacle state
            // ------------------------------------------------

            const bool left_obstacle =
                left.median_distance <= obstacle_threshold_ &&
                left.occupancy_percent >= occupancy_threshold_;

            const bool center_obstacle =
                center.median_distance <= obstacle_threshold_ &&
                center.occupancy_percent >= occupancy_threshold_;

            const bool right_obstacle =
                right.median_distance <= obstacle_threshold_ &&
                right.occupancy_percent >= occupancy_threshold_;


            std::string state = "CLEAR";

            /*
             * If all three sectors are blocked,
             * the path is completely blocked.
             */
            if (left_obstacle &&
                center_obstacle &&
                right_obstacle)
            {
                state = "BLOCKED";
            }

            /*
             * If the center is blocked and one side is also
             * blocked, explicitly report the combination.
             */
            else if (center_obstacle &&
                     left_obstacle)
            {
                state = "CENTER_LEFT";
            }

            else if (center_obstacle &&
                     right_obstacle)
            {
                state = "CENTER_RIGHT";
            }

            /*
             * Center blocked by itself.
             */
            else if (center_obstacle)
            {
                state = "CENTER";
            }

            /*
             * Both lateral sectors blocked.
             *
             * There is no safe lateral direction even though
             * the center sector is currently clear.
             */
            else if (left_obstacle &&
                     right_obstacle)
            {
                state = "BLOCKED";
            }

            else if (left_obstacle)
            {
                state = "LEFT";
            }

            else if (right_obstacle)
            {
                state = "RIGHT";
            }


            // ------------------------------------------------
            // Publish final state
            // ------------------------------------------------

            std_msgs::msg::String state_msg;
            state_msg.data = state;

            state_pub_->publish(state_msg);


            // ------------------------------------------------
            // Console output
            // ------------------------------------------------

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "L: %.2f m | %.1f%% | %.1f%% valid | "
                "C: %.2f m | %.1f%% | %.1f%% valid | "
                "R: %.2f m | %.1f%% | %.1f%% valid | "
                "State: %s",
                left.median_distance,
                left.occupancy_percent,
                left.valid_percent,
                center.median_distance,
                center.occupancy_percent,
                center.valid_percent,
                right.median_distance,
                right.occupancy_percent,
                right.valid_percent,
                state.c_str());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge exception: %s",
                e.what());

            publish_unknown_state();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Depth processing exception: %s",
                e.what());

            publish_unknown_state();
        }
    }


    void publish_unknown_state()
    {
        std_msgs::msg::String state_msg;
        state_msg.data = "UNKNOWN";
        state_pub_->publish(state_msg);
    }


    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr
        depth_subscription_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        left_distance_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        center_distance_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        right_distance_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        left_occupancy_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        center_occupancy_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr
        right_occupancy_pub_;

    rclcpp::Publisher<
        std_msgs::msg::String>::SharedPtr
        state_pub_;

    float obstacle_threshold_;
    float occupancy_threshold_;
    float min_valid_percent_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}