/**
 * @file r1_controller_gateway_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief r1_main_nodeとスマホアプリの仲介ノード
 * @version 0.1
 * @date 2026-04-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/*
[今後の実装に関するメモ]
R1InitParameter.msgは、R1の初期化に必要なパラメータをまとめたメッセージ。
スマホから初期化パラメータを受け取ったらPublishする。
joyはスマホ経由でコントローラの情報を受け取ったらその瞬間にPublishする。
ログはスマホに出力する文字列のSubscriptionを作り、タイマーで定期的にPublishする。

KFSは個別にも指定できるようにする。

[将来的にやりたいこと]
- 開発がしやすいようにトピックにしているが、r1_init_parameterはサービスにするかも。
- 
*/

/*
メモ（読んでもわからないと思うので無視でOK）
送信する内容
zone
R1 KFSの位置(MUST)
R2 KFSの位置(OPTION)
R2 FAKE KFSの位置(OPTION)
回収するKFSの番号とFRONT,REARの指定(OPTION)
  - 直接指定or auto
R2が回収するスピアの番号の指定(OPTION)

受け取る内容
現在の状態

ノードの構成
- r1_main_node
  - 状態遷移
  - メインの制御
- r1_ui/r1_controller_node
  - スマホとの通信
  - Joyの変換
  - r1_main_nodeとスマホとの架け渡し
  
以下トピック
初期化時にのみ送信
R1InitParameter.msg
```
# red or blue（必須）
string zone
# 要素数3（必須）
int32[] r1_kfs_value
# 要素数4（任意）
int32[] r2_kfs_value
# 要素数1（任意）
int32[] r2_fake_kfs_value
# どのKFSを回収するかを自動判別するか
bool enable_auto_select
# KFS回収時に足回りを自動制御するか
bool enable_kfs_auto_chassis
```

個別指定するとき
R1CollectKfs.msg
```
# 回収する（減速する）KFSの順番
int32[] forest_order
# 使用するKFS回収機構（front_kfsかrear_kfsのどちらを使うか）
# stringの値は"front_kfs"か"rear_kfs"のどちらか
string[] kfs_mechanism_type
```

通信内容
- joy関連は一定周期でスマホから送信する
*/

#include "magic_enum.hpp"
#include "r1_msgs/msg/r1_collect_kfs.hpp"
#include "r1_msgs/msg/r1_init_parameter.hpp"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class R1ControllerGatewayNode : public rclcpp::Node
{
public:
  R1ControllerGatewayNode() : Node("r1_controller_gateway_node")
  {
    // PublisherとSubscriptionの作成
    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    r1_init_parameter_publisher_ =
      this->create_publisher<r1_msgs::msg::R1InitParameter>("/r1_init_parameter", 10);
    r1_collect_kfs_publisher_ =
      this->create_publisher<r1_msgs::msg::R1CollectKfs>("/r1_collect_kfs", 10);
    log_message_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/r1_log_message", 10,
      std::bind(
        &R1ControllerGatewayNode::log_message_subscription_callback, this, std::placeholders::_1));
    operation_mode_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/r1_operation_mode", 10,
      std::bind(
        &R1ControllerGatewayNode::operation_mode_subscription_callback, this,
        std::placeholders::_1));
    // timerの作成
    this->declare_parameter<double>("log_publish_timer_rate", 10.0);
    this->get_parameter("log_publish_timer_rate", log_publish_timer_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / log_publish_timer_rate_),
      std::bind(&R1ControllerGatewayNode::timer_callback, this));

    // 各種パラメータ
    this->declare_parameter<bool>("enable_log_message", true);
    this->get_parameter("enable_log_message", enable_log_message_);
  }

  /**
   * @brief log_messageの受信コールバック
   * 
   * @param msg 
   */
  void log_message_subscription_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    log_message_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received log message: %s", log_message_.c_str());
  }

  void operation_mode_subscription_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    operation_mode_ = static_cast<OperationMode>(msg->data);
    std::string operation_mode_str = std::string(magic_enum::enum_name(operation_mode_));
    RCLCPP_INFO(this->get_logger(), "Received operation mode: %s", operation_mode_str.c_str());
  }

  /**
   * @brief タイマーコールバック
   * log_messageをスマホにPublishする。
   */
  void timer_callback()
  {
    // ここに実装を書く
  }

  // ========== publisherとsubscription ==========
  // スマホ経由で受け取ったコントローラ情報のPublisher
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<r1_msgs::msg::R1InitParameter>::SharedPtr r1_init_parameter_publisher_;
  rclcpp::Publisher<r1_msgs::msg::R1CollectKfs>::SharedPtr r1_collect_kfs_publisher_;
  // スマホに出力するログ（文字列）のSubscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_message_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr operation_mode_subscription_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // ========== R1状態保持用変数 ==========
  // R1初期化パラメータ
  r1_msgs::msg::R1InitParameter init_parameter_;
  // kfsの指令値
  r1_msgs::msg::R1CollectKfs kfs_ref_;
  OperationMode operation_mode_{OperationMode::MODE1_DETECT_ORIGIN};
  std::string log_message_;
  // ========= ROS 2パラメータ ==========
  // logをPublishするタイマーの周期
  double log_publish_timer_rate_;
  // ログ出力を有効にするか
  bool enable_log_message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1ControllerGatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
