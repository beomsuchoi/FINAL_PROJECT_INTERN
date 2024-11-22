cmake_minimum_required(VERSION 3.8)
project(imu_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(imu_node src/imu_final.cpp src/main.cpp)
ament_target_dependencies(imu_node rclcpp std_msgs sensor_msgs)
target_link_libraries(imu_node ${OpenCV_LIBS})

# Install executable
install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
