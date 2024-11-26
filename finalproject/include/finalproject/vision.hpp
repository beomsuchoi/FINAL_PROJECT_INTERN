// vision.hpp
#ifndef FINALPROJECT_VISION_HPP
#define FINALPROJECT_VISION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <array> // std::array를 위해 추가

class Vision : public rclcpp::Node
{
public:
    explicit Vision(const std::string &node_name);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool isLineValid(std::array<bool, 10> &detection_array, bool current_detection);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yellow_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr line_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yellow_detected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr white_detected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blue_sign_detected_pub_;

    bool blue_sign_detected;
    bool yellow_line_detected;
    bool white_line_detected;
    int yellow_line_count;
    int white_line_count;

    static const int ARRAY_SIZE = 10;
    static const int DETECTION_THRESHOLD = 7;
    std::array<bool, 10> yellow_detection_array;
    std::array<bool, 10> white_detection_array;
    int array_index;
    bool yellow_line_valid;
    bool white_line_valid;

    float yellow_line_x;
    float white_line_x;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yellow_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr white_pos_pub_;
};

#endif // FINALPROJECT_VISION_HPP