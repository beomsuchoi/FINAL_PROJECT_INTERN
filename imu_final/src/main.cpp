#include "imu_final/imu_final.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFinal>("imu_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}