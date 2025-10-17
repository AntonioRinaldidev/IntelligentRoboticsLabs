#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ex1/msg/robot_info.hpp"
#include "ex1/room_constants.hpp"

using namespace std::chrono_literals;
using namespace ex1;


class RobotNode : public rclcpp::Node
{
public:
  RobotNode()
  : Node("robot"), current_room_index_(0),
  battery_level_(100.0f),
  status_(RobotStatus::OFFLINE),
    state_(RobotState::IDLE)
    
    
  {
    // Vector with all the RoomIDs
    room_ids_ = {
      static_cast<int8_t>(RoomID::ROBOT_VISION_LAB),
      static_cast<int8_t>(RoomID::SSL_LAB),
      static_cast<int8_t>(RoomID::NEUROROBOTICS_LAB),
      static_cast<int8_t>(RoomID::IAS_LAB),
      static_cast<int8_t>(RoomID::AUTONOMOUS_ROBOTICS_LAB)
    };
    
    publisher_ = this->create_publisher<msg::RobotInfo>("/robot_status", 10);

    status_= RobotStatus::ONLINE;
    
    publish_timer_ = this->create_wall_timer(
      200ms,
      [this](){this->publish_callback();});
    

    RCLCPP_INFO(this->get_logger(), "Robot node started");
    start_cleaning();
  }

private:
void publish_callback(){
  if(status_== RobotStatus::OFFLINE || current_room_index_>=room_ids_.size()){
    return;
  }
      if(state_ == RobotState::CLEANING){
      battery_level_ -= 0.15f;  
    }
    else if(state_ == RobotState::MOVING){
      battery_level_ -= 0.05f;  
    }
    
   
    if(battery_level_ < 0.0f){
      battery_level_ = 0.0f;
    }
    auto message = ex1::msg::RobotInfo();

    message.robot_status = (status_ == RobotStatus::ONLINE) ? "ONLINE" : "OFFLINE";
    message.robot_state = state_to_string(state_);  

    
      if(state_ == RobotState::MOVING){
        message.room_id = 0;
        message.room_name = "Hallway";
      }else{
        message.room_id = room_ids_[current_room_index_];
        message.room_name = ex1::ROOM_NAMES.at(static_cast<ex1::RoomID>(message.room_id));
      }
    
    message.battery_level = battery_level_;

    RCLCPP_INFO(this->get_logger(),
      "\n Status: %s\n State: %s\n Location: %s (ID:%d)\n Battery level: %.1f%%",
      message.robot_status.c_str(),
      message.robot_state.c_str(),
      message.room_name.c_str(),
      static_cast<int>(message.room_id),
      static_cast<float>(message.battery_level));
    
    publisher_-> publish(message);
};
  void start_cleaning(){
    if(current_room_index_ >=room_ids_.size()
    ){
      RCLCPP_INFO(this->get_logger(), "All room cleaned");
      publish_timer_->cancel();
      return;
    };

    state_ = RobotState::CLEANING;

    RCLCPP_INFO(this->get_logger(),"Entering room: %s",
      ex1::ROOM_NAMES.at(static_cast<ex1::RoomID>(room_ids_[current_room_index_])).c_str());

    room_cleaning_timer_ = this->create_wall_timer(
      5s,
      [this](){this->finish_cleaning_callback();});
  };

  void finish_cleaning_callback(){
    RCLCPP_INFO(this->get_logger(),"Finished cleaning: %s",
      ex1::ROOM_NAMES.at(static_cast<ex1::RoomID>(room_ids_[current_room_index_])).c_str());
    room_cleaning_timer_->cancel();

     if(current_room_index_ >= room_ids_.size() - 1){
    state_ = RobotState::FINISHED;
    
    
    auto final_message = ex1::msg::RobotInfo();
    final_message.robot_status = "ONLINE";
    final_message.robot_state = "FINISHED";
    final_message.room_id = -1;
    final_message.room_name = "ALL ROOMS CLEANED";  
    final_message.battery_level = battery_level_;
    
    publisher_->publish(final_message);
    
    RCLCPP_INFO(this->get_logger(), "All rooms cleaned! Robot shutting down.");
    
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    status_ = RobotStatus::OFFLINE;
    publish_timer_->cancel();
    return;
  }
      state_= RobotState::MOVING;
      
      
      room_change_timer_= this->create_wall_timer(
        2s,
        [this](){this->change_room_callback();}
      );}

    void change_room_callback(){
      room_change_timer_->cancel();
      if(current_room_index_>=room_ids_.size()){
        return;
      }
      current_room_index_++;
      RCLCPP_INFO(this->get_logger(),"Changing room");

      start_cleaning();

    
  }
  
  rclcpp::Publisher<ex1::msg::RobotInfo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr room_cleaning_timer_;
  rclcpp::TimerBase::SharedPtr room_change_timer_;
  std::vector<int8_t> room_ids_;
  size_t current_room_index_;
  float battery_level_; 
  RobotStatus status_;
  RobotState state_;

  std::string state_to_string(RobotState state){
    switch(state){
      case RobotState::IDLE: return "IDLE";
      case RobotState::CLEANING: return "CLEANING";
      case RobotState::MOVING: return "MOVING";
      case RobotState::FINISHED: return "FINISHED";
      default: return "UNKNOWN";
    }
}
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
  return 0;
}