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

void ImuFinal::quaternionToEuler(const double q0, const double q1, 
                                const double q2, const double q3,
                                double& roll, double& pitch, double& yaw)
{
    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // Convert to degrees if needed
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
}

void ImuFinal::imuCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::vector<double> values;
    // 대괄호 제거용
    std::string data = msg->data.substr(1, msg->data.length() - 2);
    std::stringstream ss2(data);
    std::string token;
    
    while (std::getline(ss2, token, ',')) {
        values.push_back(std::stod(token));
    }
    
    if (values.size() >= 4) {
        processImuData(values);
    }
}

void ImuFinal::processImuData(const std::vector<double>& data)
{
    double roll, pitch, yaw;
    quaternionToEuler(data[0], data[1], data[2], data[3], roll, pitch, yaw);
    
    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();
    
    roll_msg.data = static_cast<float>(static_cast<int>(roll));    // 소수점 제거
    pitch_msg.data = static_cast<float>(static_cast<int>(pitch));  // 소수점 제거
    yaw_msg.data = static_cast<float>(static_cast<int>(yaw));      // 소수점 제거
    
    roll_pub_->publish(roll_msg);
    pitch_pub_->publish(pitch_msg);
    yaw_pub_->publish(yaw_msg);
}