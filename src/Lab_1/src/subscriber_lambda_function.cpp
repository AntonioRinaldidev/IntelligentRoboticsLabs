
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/num.hpp"

//Class
class MinimalSubscriber : public rclcpp::Node
{

//Public Constructor
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    //* Receives the string message data published over the topic and writes it to the console 
    auto topic_callback =
      [this](custom_msg::msg::Num::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%ld'",msg->num);
      };
    subscription_ =
      this->create_subscription<custom_msg::msg::Num>("topic", 10, topic_callback);
  }

//Private Constructor
private:
  rclcpp::Subscription<custom_msg::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
