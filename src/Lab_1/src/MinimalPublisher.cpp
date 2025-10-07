#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 function to shorten the callback syntax, at the expense of making the
 code somewhat more difficult to understand at first glance. */

//Class 
class MinimalPublisher : public rclcpp::Node
{
  //Public Constructor
public:
  MinimalPublisher()
  //*Name of the node and count initialization
  : Node("minimal_publisher"), count_(0)
  {
    //*Initialization of the publisher with a String messagae type
    //*Topic name
    //*Required queue size
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    //* Lambda Function declaration
    auto timer_callback =

    //* Camptures by refence of the current object, No params and return void 
      [this]() -> void {

        //Create a new message of type String, sets its data with the desired string and publishes it
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        
        //* ensures every published message is printed to the console
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };

      //* Timer initialization
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
//Private Constructor
private:
//* declaration of the timer, publisher, and counter fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

//* Node Execution
int main(int argc, char * argv[])
{
  //*Initializes ROS 2
  rclcpp::init(argc, argv);

  //*Processing data from the node, including the callbacks from the timer
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}