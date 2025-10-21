#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ex1/msg/robot_info.hpp"
#include "ex1/room_constants.hpp"

using namespace std::chrono_literals;
using namespace ex1;

/**
 * @brief RobotNode class - Represents an autonomous vacuum cleaner robot
 * 
 * This node simulates a robot that cleans multiple rooms in sequence.
 * It publishes its status (location, battery level, state) to the /robot_status topic.
 */
class RobotNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - Initializes the robot node
   * 
   * Sets up the publisher, timers, and initial state.
   * The robot starts in OFFLINE mode and transitions to ONLINE.
   */
  RobotNode()
  : Node("robot"), 
    current_room_index_(0),
    battery_level_(100.0f),
    status_(RobotStatus::OFFLINE),
    state_(RobotState::IDLE)
  {
    // Initialize the list of rooms to clean
    room_ids_ = {
      static_cast<int8_t>(RoomID::ROBOT_VISION_LAB),
      static_cast<int8_t>(RoomID::SSL_LAB),
      static_cast<int8_t>(RoomID::NEUROROBOTICS_LAB),
      static_cast<int8_t>(RoomID::IAS_LAB),
      static_cast<int8_t>(RoomID::AUTONOMOUS_ROBOTICS_LAB)
    };
    
    // Create publisher for robot status
    publisher_ = this->create_publisher<msg::RobotInfo>("/charge_status", 10);

    // Set robot status to ONLINE
    status_ = RobotStatus::ONLINE;
    
    // Create timer to publish status at 5Hz (every 200ms)
    publish_timer_ = this->create_wall_timer(
      200ms,
      [this]() { this->publish_callback(); });
    
    RCLCPP_INFO(this->get_logger(), "Robot node started");
    
    // Start cleaning the first room
    start_cleaning();
  }

private:
  /**
   * @brief Publishes robot status at 5Hz
   * 
   * This callback is called every 200ms to publish the current state,
   * battery level, and location of the robot. It also simulates battery
   * discharge based on the current activity.
   */
  void publish_callback()
  {
    // Don't publish if robot is offline or finished all rooms
    if (status_ == RobotStatus::OFFLINE || current_room_index_ >= room_ids_.size()) {
      return;
    }
    
    // Simulate battery discharge based on robot state
    if (state_ == RobotState::CLEANING) {
      battery_level_ -= 0.15f;  // Cleaning consumes more battery
    }
    else if (state_ == RobotState::MOVING) {
      battery_level_ -= 0.05f;  // Moving consumes less battery
    }
    
    // Ensure battery doesn't go below 0%
    if (battery_level_ < 0.0f) {
      battery_level_ = 0.0f;
    }
    
    // Create and populate the message
    auto message = ex1::msg::RobotInfo();
    message.robot_status = (status_ == RobotStatus::ONLINE) ? "ONLINE" : "OFFLINE";
    message.robot_state = state_to_string(state_);
    
    // Set location based on current state
    if (state_ == RobotState::MOVING) {
      message.room_id = 0;
      message.room_name = "Hallway";
    }
    else {
      message.room_id = room_ids_[current_room_index_];
      message.room_name = ROOM_NAMES.at(static_cast<RoomID>(message.room_id));
    }
    
    message.battery_level = battery_level_;

    // Log current status
    RCLCPP_INFO(this->get_logger(),
      "\n Status: %s\n State: %s\n Location: %s (ID:%d)\n Battery level: %.1f%%",
      message.robot_status.c_str(),
      message.robot_state.c_str(),
      message.room_name.c_str(),
      static_cast<int>(message.room_id),
      static_cast<float>(message.battery_level));
    
    // Publish the message
    publisher_->publish(message);
  }
  
  /**
   * @brief Starts cleaning the current room
   * 
   * Transitions the robot to CLEANING state and sets up a timer
   * for the cleaning duration (5 seconds per room).
   */
  void start_cleaning()
  {
    // Check if all rooms have been cleaned
    if (current_room_index_ >= room_ids_.size()) {
      RCLCPP_INFO(this->get_logger(), "All rooms cleaned");
      publish_timer_->cancel();
      return;
    }
    
    // Set state to CLEANING
    state_ = RobotState::CLEANING;

    RCLCPP_INFO(this->get_logger(), "Entering room: %s",
      ROOM_NAMES.at(static_cast<RoomID>(room_ids_[current_room_index_])).c_str());

    // Create timer for cleaning duration (5 seconds)
    room_cleaning_timer_ = this->create_wall_timer(
      5s,
      [this]() { this->finish_cleaning_callback(); });
  }

  /**
   * @brief Called when cleaning is finished for the current room
   * 
   * Checks if all rooms are done. If so, publishes a final message.
   * Otherwise, transitions to MOVING state to go to the next room.
   */
  void finish_cleaning_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Finished cleaning: %s",
      ROOM_NAMES.at(static_cast<RoomID>(room_ids_[current_room_index_])).c_str());
    room_cleaning_timer_->cancel();

    // Check if this was the last room
    if (current_room_index_ >= room_ids_.size() - 1) {
      state_ = RobotState::FINISHED;
      
      // Publish final completion message
      auto final_message = ex1::msg::RobotInfo();
      final_message.robot_status = "ONLINE";
      final_message.robot_state = "FINISHED";
      final_message.room_id = -1;  // Special ID for completion
      final_message.room_name = "ALL ROOMS CLEANED";
      final_message.battery_level = battery_level_;
      
      publisher_->publish(final_message);
      
      RCLCPP_INFO(this->get_logger(), "All rooms cleaned! Robot shutting down.");
      
      // Wait for message to be sent before shutting down
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      // Shutdown robot
      status_ = RobotStatus::OFFLINE;
      publish_timer_->cancel();
      return;
    }
    
    // Transition to MOVING state to go to next room
    state_ = RobotState::MOVING;
    
    // Create timer for moving duration (2 seconds)
    room_change_timer_ = this->create_wall_timer(
      2s,
      [this]() { this->change_room_callback(); });
  }

  /**
   * @brief Called when the robot finishes moving to the next room
   * 
   * Increments the room index and starts cleaning the next room.
   */
  void change_room_callback()
  {
    room_change_timer_->cancel();
    
    // Safety check
    if (current_room_index_ >= room_ids_.size()) {
      return;
    }
    
    // Move to next room
    current_room_index_++;
    RCLCPP_INFO(this->get_logger(), "Moving to next room");

    // Start cleaning the next room
    start_cleaning();
  }
  
  /**
   * @brief Converts RobotState enum to string
   * @param state The robot state to convert
   * @return String representation of the state
   */
  std::string state_to_string(RobotState state)
  {
    switch (state) {
      case RobotState::IDLE:     return "IDLE";
      case RobotState::CLEANING: return "CLEANING";
      case RobotState::MOVING:   return "MOVING";
      case RobotState::FINISHED: return "FINISHED";
      default:                   return "UNKNOWN";
    }
  }

  // ROS2 communication
  rclcpp::Publisher<ex1::msg::RobotInfo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr room_cleaning_timer_;
  rclcpp::TimerBase::SharedPtr room_change_timer_;
  
  // Robot state variables
  std::vector<int8_t> room_ids_;
  size_t current_room_index_;
  float battery_level_;
  RobotStatus status_;
  RobotState state_;
};

/**
 * @brief Main function - Entry point of the robot node
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
  return 0;
}