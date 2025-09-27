#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class MyNode : public rclcpp::Node
{ 
public:
  MyNode() : Node("r1_state_machine_node")
  {
    // ジョイスティックの購読者を作成
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&MyNode::joy_callback, this, std::placeholders::_1));
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // ジョイスティックのデータを処理
    RCLCPP_INFO(this->get_logger(), "Received joystick data: axes[0]=%f, buttons[0]=%d", msg->axes[0], msg->buttons[0]);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}