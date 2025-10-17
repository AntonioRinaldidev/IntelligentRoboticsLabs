#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ex1/msg/robot_info.hpp"

class ChargingStationNode : public rclcpp::Node
{
public:
  ChargingStationNode()
  : Node("charging_station")
  {
    subscription_ = this->create_subscription<ex1::msg::RobotInfo>(
      "/robot_status", 
      10,
      [this](const ex1::msg::RobotInfo::SharedPtr msg){
        this->robot_status_callback(msg);
      });
    
    RCLCPP_INFO(this->get_logger(), "Charging Station started - listening for robot status...");
  }

private:
  void robot_status_callback(const ex1::msg::RobotInfo::SharedPtr msg)
  {
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

  rclcpp::Subscription<ex1::msg::RobotInfo>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargingStationNode>());
  rclcpp::shutdown();
  return 0;
}