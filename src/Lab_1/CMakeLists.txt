cmake_minimum_required(VERSION 3.5)
project(Lab_1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)

#Publisher
add_executable(talker src/MinimalPublisher.cpp)
ament_target_dependencies(talker rclcpp custom_msg)

#Subscriber
add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp custom_msg)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
ament_package()