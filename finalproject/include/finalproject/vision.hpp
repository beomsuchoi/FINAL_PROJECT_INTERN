// vision.hpp
#ifndef FINALPROJECT_VISION_HPP
#define FINALPROJECT_VISION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <vector>

class Vision : public rclcpp::Node
{
public:
    explicit Vision(const std::string &node_name);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yellow_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr line_pub_;
};

#endif // FINALPROJECT_VISION_HPP
