#include "imu_final/imu_final.hpp"
#include <cmath>

float LowPassFilter::update(float input)
{
    if (first_run)
    {
        filtered_value = input;
        first_run = false;
        return filtered_value;
    }
    filtered_value = alpha * input + (1 - alpha) * filtered_value;
    return filtered_value;
}

float KalmanFilter::update(float measurement)
{
    if (first_run)
    {
        X = measurement;
        first_run = false;
        return X;
    }
    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;
    return X;
}

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
                                 double &roll, double &pitch, double &yaw)
{
    double sinr_cosp = 2 * (q3 * q2 + q1 * q0);
    double cosr_cosp = 1 - 2 * (q2 * q2 + q1 * q1);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q3 * q1 - q0 * q2);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (q3 * q0 + q2 * q1);
    double cosy_cosp = 1 - 2 * (q1 * q1 + q0 * q0);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
}

void ImuFinal::resetAngles()
{
    // yaw 오프셋만 현재 값으로 설정
    yaw_offset = yaw_kf.getState();
    
    // yaw 관련 필터만 초기화
    yaw_lpf = LowPassFilter();
    yaw_kf = KalmanFilter();
}

void ImuFinal::imuCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::vector<double> values;
    std::string data = msg->data.substr(1, msg->data.length() - 2);
    std::stringstream ss2(data);
    std::string token;

    while (std::getline(ss2, token, ','))
    {
        values.push_back(std::stod(token));
    }

    if (values.size() >= 4)
    {
        processImuData(values);
    }
}

void ImuFinal::processImuData(const std::vector<double>& data)
{
    double roll, pitch, yaw;
    quaternionToEuler(data[0], data[1], data[2], data[3], roll, pitch, yaw);

    // yaw에만 오프셋 적용
    yaw -= yaw_offset;
    
    float filtered_roll = roll_lpf.update(roll);
    float filtered_pitch = pitch_lpf.update(pitch);
    float filtered_yaw = yaw_lpf.update(yaw);

    float final_roll = roll_kf.update(filtered_roll);
    float final_pitch = pitch_kf.update(filtered_pitch);
    float final_yaw = yaw_kf.update(filtered_yaw);

    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();

    roll_msg.data = static_cast<float>(static_cast<int>(final_roll));
    pitch_msg.data = static_cast<float>(static_cast<int>(final_pitch));
    yaw_msg.data = static_cast<float>(static_cast<int>(final_yaw));

    roll_pub_->publish(roll_msg);
    pitch_pub_->publish(pitch_msg);
    yaw_pub_->publish(yaw_msg);
}