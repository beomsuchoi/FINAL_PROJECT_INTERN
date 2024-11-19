#include "finalproject/vision.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vision>("vision_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
