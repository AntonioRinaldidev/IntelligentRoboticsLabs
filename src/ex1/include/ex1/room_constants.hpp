/**
 * @file room_constants.hpp
 * @brief Defines constants and enumerations for the robot cleaning system
 * 
 * This header file contains all the constant definitions used by the robot
 * and charging station nodes, including room identifiers, robot states,
 * and robot status enumerations.
 */

#ifndef EX1_ROOM_CONSTANTS_HPP
#define EX1_ROOM_CONSTANTS_HPP

#include <string>
#include <map>

namespace ex1 {

/**
 * @brief Enum representing the robot's connection status
 * 
 * ONLINE:  Robot is powered on and operational
 * OFFLINE: Robot is powered off or disconnected
 */
enum class RobotStatus {
  ONLINE,
  OFFLINE
};

/**
 * @brief Enum representing the robot's current activity state
 * 
 * IDLE:     Robot is idle and waiting for commands
 * CLEANING: Robot is actively cleaning a room
 * MOVING:   Robot is moving between rooms (in hallway)
 * FINISHED: Robot has completed all cleaning tasks
 */
enum class RobotState {
  IDLE,
  CLEANING,
  MOVING,
  FINISHED
};

/**
 * @brief Enum representing unique identifiers for each room
 * 
 * Each room in the laboratory has a unique ID assigned to it.
 * The underlying type is int8_t to optimize memory usage.
 */
enum class RoomID : int8_t {
  ROBOT_VISION_LAB = 1,
  SSL_LAB = 2,
  NEUROROBOTICS_LAB = 3,
  IAS_LAB = 4,
  AUTONOMOUS_ROBOTICS_LAB = 5
};

/**
 * @brief Map associating room IDs with their human-readable names
 * 
 * This constant map provides a lookup table to convert room IDs
 * into descriptive room names for logging and display purposes.
 */
const std::map<RoomID, std::string> ROOM_NAMES = {
  {RoomID::ROBOT_VISION_LAB, "Robot Vision Lab"},
  {RoomID::SSL_LAB, "SSL Lab"},
  {RoomID::NEUROROBOTICS_LAB, "Neurorobotics Lab"},
  {RoomID::IAS_LAB, "IAS-Lab"},
  {RoomID::AUTONOMOUS_ROBOTICS_LAB, "Autonomous Robotics Lab"}
};

}  // namespace ex1

#endif  // EX1_ROOM_CONSTANTS_HPP