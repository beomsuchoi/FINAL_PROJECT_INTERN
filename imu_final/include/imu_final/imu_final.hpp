#ifndef IMU_FINAL_HPP
#define IMU_FINAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <sstream>

class LowPassFilter {
private:
    float alpha;
    float filtered_value;
    bool first_run;
public:
    LowPassFilter(float a = 0.1) : alpha(a), first_run(true) {}
    float update(float input);
};

class KalmanFilter {
private:
    float Q, R, P, X, K;
    bool first_run;
public:
    KalmanFilter(float q = 0.1, float r = 0.1) : Q(q), R(r), first_run(true) {}
    float update(float measurement);
};

class ImuFinal : public rclcpp::Node
{
public:
    explicit ImuFinal(const std::string &node_name);

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

    LowPassFilter roll_lpf{0.1}, pitch_lpf{0.1}, yaw_lpf{0.1};
    KalmanFilter roll_kf{0.1, 0.1}, pitch_kf{0.1, 0.1}, yaw_kf{0.1, 0.1};

    void imuCallback(const std_msgs::msg::String::SharedPtr msg);
    void processImuData(const std::vector<double>& data);
    void quaternionToEuler(const double q0, const double q1, 
                          const double q2, const double q3,
                          double& roll, double& pitch, double& yaw);
};

#endif