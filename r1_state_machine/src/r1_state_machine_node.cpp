/*
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <map>

namespace MainState{
enum {
  IDLE,
  MANUAL,
  AUTO,
  EMERGENCY
};
std::map mp = {
  {IDLE, "IDLE"},
  {MANUAL, "MANUAL"},
  {AUTO, "AUTO"},
  {EMERGENCY, "EMERGENCY"};
};
}

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
    RCLCPP_INFO(
      this->get_logger(), "Received joystick data: axes[0]=%f, buttons[0]=%d", msg->axes[0],
      msg->buttons[0]);
  }

  int main_state_ = MainState::IDLE;
  int sub_state_ = 0;
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
*/

#include "rclcpp/rclcpp.hpp"
#include "magic_enum.hpp" // magic_enumのヘッダーをインクルード

namespace MainState {
    // enum classを定義するだけ！
    enum class State {
        IDLE,
        MANUAL,
        AUTO,
        EMERGENCY
    };
}

// --- 使用例 ---
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("state_example");
  
  MainState::State current_state = MainState::State::AUTO;

  // magic_enum::enum_name() を使って状態を文字列に変換
  auto state_name = magic_enum::enum_name(current_state);

  // state_nameはstd::string_view型なので、.data()や.c_str()でC言語文字列に変換
  RCLCPP_INFO(node->get_logger(), "Current state is: %s", state_name.data());

  // 文字列からenumへの変換も可能
  auto state_optional = magic_enum::enum_cast<MainState::State>("MANUAL");
  if (state_optional) {
    RCLCPP_INFO(node->get_logger(), "String 'MANUAL' converted back to enum.");
  }

  rclcpp::shutdown();
  return 0;
}