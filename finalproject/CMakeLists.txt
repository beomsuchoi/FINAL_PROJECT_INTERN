cmake_minimum_required(VERSION 3.8)
project(finalproject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(vision_node src/vision.cpp src/main.cpp)
ament_target_dependencies(vision_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(vision_node ${OpenCV_LIBS})

# Install executable
install(TARGETS vision_node DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Package
ament_package()