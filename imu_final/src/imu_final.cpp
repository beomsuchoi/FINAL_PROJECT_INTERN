#include "imu_final/imu_final.hpp"
#include <cmath>

ImuFinal::ImuFinal(const std::string &node_name) : Node(node_name)
{
    imu_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/ebimu_data", 10,
        std::bind(&ImuFinal::imuCallback, this, std::placeholders::_1));
        
    roll_pub_ = this->create_publisher<std_msgs::msg::Float32>("/roll", 10);
    pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pitch", 10);
    yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/yaw", 10);
}

void ImuFinal::imuCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::vector<double> values;
    std::stringstream ss(msg->data);
    std::string token;
    
    // Remove brackets
    std::string data = msg->data.substr(1, msg->data.length() - 2);
    std::stringstream ss2(data);
    
    while (std::getline(ss2, token, ',')) {
        values.push_back(std::stod(token));
    }
    
    if (values.size() >= 4) {
        processImuData(values);
    }
}

void ImuFinal::processImuData(const std::vector<double>& data)
{
    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();
    
    // Assuming the first three values are roll, pitch, yaw
    roll_msg.data = static_cast<float>(data[0]);
    pitch_msg.data = static_cast<float>(data[1]);
    yaw_msg.data = static_cast<float>(data[2]);
    
    roll_pub_->publish(roll_msg);
    pitch_pub_->publish(pitch_msg);
    yaw_pub_->publish(yaw_msg);
    
    RCLCPP_INFO(this->get_logger(), "Angles - Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
        data[0], data[1], data[2]);
}