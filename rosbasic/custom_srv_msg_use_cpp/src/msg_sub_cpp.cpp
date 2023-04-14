#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_srv_msg/msg/num.hpp"     // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<custom_srv_msg::msg::Num>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const custom_srv_msg::msg::Num::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
  }
  rclcpp::Subscription<custom_srv_msg::msg::Num>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}