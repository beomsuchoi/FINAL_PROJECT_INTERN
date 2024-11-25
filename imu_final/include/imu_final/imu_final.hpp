#ifndef IMU_FINAL_HPP
#define IMU_FINAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <sstream>

class ImuFinal : public rclcpp::Node
{
public:
    explicit ImuFinal(const std::string &node_name);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

    void imuCallback(const std_msgs::msg::String::SharedPtr msg);
    void processImuData(const std::vector<double>& data);
    void quaternionToEuler(const double q0, const double q1, 
                          const double q2, const double q3,
                          double& roll, double& pitch, double& yaw);
};

#endif