#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// create the node class "MinimalPublisher"
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) // names the node "minimal_publisher" and initialize to 0
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // initialize with the String message type
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this)); // cause the "timer_callback" function to be executed 1/2sec

    }

private:
  // "timer_callback" : where the message data is set and the messages are published
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // "RCLCPP_INFO" : ensure every published message is printed to the console
    publisher_->publish(message);
  }
  // declaration of the timer, publisher, and counter fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // initialize the ROS2
  rclcpp::spin(std::make_shared<MinimalPublisher>()); // start processing data from the node
  rclcpp::shutdown();
  return 0;
}
