cmake_minimum_required(VERSION 3.8)
project(ex1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Genera le interfacce dal file .msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotInfo.msg"
  DEPENDENCIES std_msgs
)

# Get typesupport target (needed for custom messages)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Eseguibile Robot (Publisher)
add_executable(robot src/robot/Robot.cpp)
ament_target_dependencies(robot rclcpp std_msgs)
target_link_libraries(robot "${cpp_typesupport_target}")

# Eseguibile Charging Station (Subscriber)
add_executable(charging_station src/charging_station/ChargingStation.cpp)
ament_target_dependencies(charging_station rclcpp std_msgs)
target_link_libraries(charging_station "${cpp_typesupport_target}")

# Installa gli eseguibili
install(TARGETS
  robot
  charging_station
  DESTINATION lib/${PROJECT_NAME})



ament_package()