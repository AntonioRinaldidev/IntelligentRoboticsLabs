#ifndef EX1_ROOM_CONSTANTS_HPP
#define EX1_ROOM_CONSTANTS_HPP

#include<string>
#include<map>

namespace ex1 {
enum class RobotStatus{
  ONLINE,
  OFFLINE
};

enum class RobotState{
  IDLE,
  CLEANING,
  MOVING,
  FINISHED
};
enum class RoomID: int8_t{
    ROBOT_VISION_LAB = 1,
    SSL_LAB= 2,
    NEUROROBOTICS_LAB=3,
    IAS_LAB=4,
    AUTONOMOUS_ROBOTICS_LAB=5,
};

const std::map<RoomID,std::string> ROOM_NAMES ={
    {RoomID::ROBOT_VISION_LAB,"Robot Vision Lab"},
    {RoomID::SSL_LAB,"SSL Lab"},
    {RoomID::NEUROROBOTICS_LAB,"Neurorobotics Lab"},
    {RoomID::IAS_LAB,"IAS-Lab"},
    {RoomID::AUTONOMOUS_ROBOTICS_LAB,"Autonomous Robotics Lab"}
};

};

#endif