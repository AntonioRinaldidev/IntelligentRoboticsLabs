#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ex1/msg/robot_info.hpp"

/**
 * @brief ChargingStationNode class - Represents the robot's charging station
 * 
 * This node subscribes to the robot's status messages and displays
 * information about the robot's current location, state, and battery level.
 * It acts as a monitoring station for the autonomous vacuum cleaner.
 */
class ChargingStationNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - Initializes the charging station node
   * 
   * Sets up the subscription to the /robot_status topic to receive
   * updates from the robot at 5Hz.
   */
  ChargingStationNode()
  : Node("charging_station")
  {
    // Create subscription to robot status topic
    subscription_ = this->create_subscription<ex1::msg::RobotInfo>(
      "/charge_status", 
      10,
      [this](const ex1::msg::RobotInfo::SharedPtr msg) {
        this->robot_status_callback(msg);
      });
    
    RCLCPP_INFO(this->get_logger(), 
                "Charging Station started - listening for robot status...");
  }

private:
  /**
   * @brief Callback function called when a robot status message is received
   * 
   * This function is triggered every time the robot publishes its status (5Hz).
   * It logs all relevant information about the robot's current state.
   * 
   * @param msg Shared pointer to the received RobotInfo message
   */
  void robot_status_callback(const ex1::msg::RobotInfo::SharedPtr msg)
  {
    // Display received robot information
    RCLCPP_INFO(this->get_logger(), 
                "\nReceived Robot Status:\n"
                "   Status: %s\n"
                "   State: %s\n"
                "   Location: %s (ID: %ld)\n"
                "   Battery: %.1f%%",
                msg->robot_status.c_str(),
                msg->robot_state.c_str(),
                msg->room_name.c_str(),
                msg->room_id,
                msg->battery_level);
  }

  // ROS2 communication
  rclcpp::Subscription<ex1::msg::RobotInfo>::SharedPtr subscription_;
};

/**
 * @brief Main function - Entry point of the charging station node
 */
int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create and spin the charging station node
  rclcpp::spin(std::make_shared<ChargingStationNode>());
  
  // Shutdown ROS2
  rclcpp::shutdown();
  
  return 0;
}