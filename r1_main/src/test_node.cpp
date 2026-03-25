/**
 * @file test_node.cpp
 * @brief Joy から cmd_vel を出しつつ swerve 指令を Sabacan へ橋渡しする簡易ノード
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "r1_main/ps4.h"
#include "r1_msgs/msg/motor.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "r1_msgs/msg/swerve_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{

constexpr size_t MOTOR_COUNT = 4;
using StringArray = std::array<std::string, MOTOR_COUNT>;
using IntArray = std::array<int, MOTOR_COUNT>;

StringArray toStringArray(const std::vector<std::string> & values, const std::string & param_name)
{
  // wheel / steer を 4 輪固定で扱うので、配列長もここで揃えておく。
  if (values.size() != MOTOR_COUNT) {
    throw std::runtime_error(param_name + " must have exactly 4 elements");
  }

  StringArray result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

IntArray toIntArray(const std::vector<int64_t> & values, const std::string & param_name)
{
  // YAML から読む整数配列を、以降の処理で使いやすい固定長配列へ変換する。
  if (values.size() != MOTOR_COUNT) {
    throw std::runtime_error(param_name + " must have exactly 4 elements");
  }

  IntArray result{};
  std::transform(values.begin(), values.end(), result.begin(), [](int64_t value) {
    return static_cast<int>(value);
  });
  return result;
}

std::string normalizeControlType(const std::string & control_type)
{
  // launch / YAML / 上流ノードの表記揺れをここで吸収する。
  std::string normalized;
  normalized.reserve(control_type.size());

  for (unsigned char c : control_type) {
    normalized.push_back(static_cast<char>(std::toupper(c)));
  }

  if (normalized == "POS") {
    return "POSITION";
  }
  if (normalized == "VEL") {
    return "VELOCITY";
  }
  return normalized;
}

r1_msgs::msg::Motor toMotorStatus(const sabacan_msgs::msg::SabacanRobomasStatus & status)
{
  // Sabacan の生 status を、既存のデバッグ可視化で扱っている Motor 型へ詰め替える。
  r1_msgs::msg::Motor motor_msg;
  motor_msg.motor_type = status.motor_type;
  motor_msg.control_type = status.control_type;
  motor_msg.motor_state = status.motor_state;
  motor_msg.torque = status.torque;
  motor_msg.speed = status.speed;
  motor_msg.pos = status.pos;
  motor_msg.abs_pos = status.abs_pos;
  motor_msg.abs_speed = status.abs_speed;
  motor_msg.abs_turn_cnt = status.abs_turn_cnt;
  motor_msg.vesc_voltage = status.vesc_voltage;
  motor_msg.vesc_current = status.vesc_current;
  motor_msg.vesc_speed = status.vesc_speed;
  return motor_msg;
}

std::vector<std::string> defaultWheelMotorRefTopics()
{
  return {
    "/swerve_fr_wheel_motor_ref", "/swerve_fl_wheel_motor_ref", "/swerve_rl_wheel_motor_ref",
    "/swerve_rr_wheel_motor_ref"};
}

std::vector<std::string> defaultSteerMotorRefTopics()
{
  return {
    "/swerve_fr_steer_motor_ref", "/swerve_fl_steer_motor_ref", "/swerve_rl_steer_motor_ref",
    "/swerve_rr_steer_motor_ref"};
}

std::vector<std::string> defaultWheelStatusTopics()
{
  return {
    "/debug_swerve_fr_wheel_motor_status", "/debug_swerve_fl_wheel_motor_status",
    "/debug_swerve_rl_wheel_motor_status", "/debug_swerve_rr_wheel_motor_status"};
}

std::vector<std::string> defaultSteerStatusTopics()
{
  return {
    "/debug_swerve_fr_steer_motor_status", "/debug_swerve_fl_steer_motor_status",
    "/debug_swerve_rl_steer_motor_status", "/debug_swerve_rr_steer_motor_status"};
}

}  // namespace

class JoyToCmdVelNode : public rclcpp::Node
{
public:
  JoyToCmdVelNode() : Node("joy_to_cmd_vel_node")
  {
    try {
      loadParameters();
      validateParameters();
      createInterfaces();
      logConfiguration();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "%s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  struct MotorChannel
  {
    std::string label;
    std::string motor_ref_topic;
    std::string debug_status_topic;
    std::string expected_control_type;
    int board_id;
    int motor_number;
  };

  void loadParameters()
  {
    // PS4 は Joy callback と定周期 update を分けて使う前提なので、先に生成しておく。
    ps4_ = std::make_shared<PS4>(this->get_name());

    // test_node は Joy -> cmd_vel と、swerve -> Sabacan の橋渡しを両方持つ。
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    swerve_drive_ref_topic_ =
      this->declare_parameter<std::string>("swerve_drive_ref_topic", "/swerve_drive_ref");
    power_ref_topic_ =
      this->declare_parameter<std::string>("power_ref_topic", "/sabacan_power_ref0");
    robomas_reset_service_id1_ = this->declare_parameter<std::string>(
      "robomas_reset_service_id1", "/sabacan_robomas_reset_id1");
    robomas_reset_service_id2_ = this->declare_parameter<std::string>(
      "robomas_reset_service_id2", "/sabacan_robomas_reset_id2");

    timer_rate_ = this->declare_parameter<double>("timer_rate", 100.0);
    max_velocity_ = std::abs(this->declare_parameter<double>("max_velocity", 1.0));
    max_angular_velocity_ = std::abs(this->declare_parameter<double>("max_angular_velocity", 1.0));
    deadzone_ = this->declare_parameter<double>("deadzone", 0.1);
    sabacan_is_ems_ = this->declare_parameter<bool>("initial_is_ems", false);

    wheel_control_type_ =
      normalizeControlType(this->declare_parameter<std::string>("wheel_control_type", "VELOCITY"));
    steer_control_type_ =
      normalizeControlType(this->declare_parameter<std::string>("steer_control_type", "POSITION"));

    const auto wheel_motor_ref_topics = toStringArray(
      this->declare_parameter<std::vector<std::string>>(
        "wheel_motor_ref_topics", defaultWheelMotorRefTopics()),
      "wheel_motor_ref_topics");
    const auto steer_motor_ref_topics = toStringArray(
      this->declare_parameter<std::vector<std::string>>(
        "steer_motor_ref_topics", defaultSteerMotorRefTopics()),
      "steer_motor_ref_topics");
    const auto wheel_debug_status_topics = toStringArray(
      this->declare_parameter<std::vector<std::string>>(
        "wheel_debug_status_topics", defaultWheelStatusTopics()),
      "wheel_debug_status_topics");
    const auto steer_debug_status_topics = toStringArray(
      this->declare_parameter<std::vector<std::string>>(
        "steer_debug_status_topics", defaultSteerStatusTopics()),
      "steer_debug_status_topics");

    const auto wheel_board_ids = toIntArray(
      this->declare_parameter<std::vector<int64_t>>("wheel_board_ids", {1, 1, 2, 2}),
      "wheel_board_ids");
    const auto wheel_motor_numbers = toIntArray(
      this->declare_parameter<std::vector<int64_t>>("wheel_motor_numbers", {0, 1, 1, 0}),
      "wheel_motor_numbers");
    const auto steer_board_ids = toIntArray(
      this->declare_parameter<std::vector<int64_t>>("steer_board_ids", {1, 1, 2, 2}),
      "steer_board_ids");
    const auto steer_motor_numbers = toIntArray(
      this->declare_parameter<std::vector<int64_t>>("steer_motor_numbers", {2, 3, 3, 2}),
      "steer_motor_numbers");

    // 配列の並び順は fr, fl, rl, rr で統一する。
    constexpr std::array<const char *, MOTOR_COUNT> wheel_labels = {
      "fr_wheel", "fl_wheel", "rl_wheel", "rr_wheel"};
    constexpr std::array<const char *, MOTOR_COUNT> steer_labels = {
      "fr_steer", "fl_steer", "rl_steer", "rr_steer"};

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      wheel_channels_[i] =
        MotorChannel{wheel_labels[i],     wheel_motor_ref_topics[i], wheel_debug_status_topics[i],
                     wheel_control_type_, wheel_board_ids[i],        wheel_motor_numbers[i]};
      steer_channels_[i] =
        MotorChannel{steer_labels[i],     steer_motor_ref_topics[i], steer_debug_status_topics[i],
                     steer_control_type_, steer_board_ids[i],        steer_motor_numbers[i]};
    }
  }

  void validateParameters()
  {
    if (timer_rate_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "timer_rate must be positive. fallback to 100.0 Hz: %.3f", timer_rate_);
      timer_rate_ = 100.0;
    }

    if (deadzone_ < 0.0 || deadzone_ > 1.0) {
      const double clamped_deadzone = std::clamp(deadzone_, 0.0, 1.0);
      RCLCPP_WARN(
        this->get_logger(), "deadzone must be in [0.0, 1.0]. clamped from %.3f to %.3f", deadzone_,
        clamped_deadzone);
      deadzone_ = clamped_deadzone;
    }
    ps4_->set_deadzone(deadzone_);

    // 1 つの board / motor を wheel と steer の両方で重複使用しないように検査する。
    std::set<std::pair<int, int>> used_motors;
    auto validate_channel = [this, &used_motors](const MotorChannel & channel) {
      if (channel.board_id < 0) {
        throw std::runtime_error(channel.label + " board_id must be >= 0");
      }
      if (channel.motor_number < 0 || channel.motor_number > 3) {
        throw std::runtime_error(channel.label + " motor_number must be in [0, 3]");
      }
      const auto [_, inserted] = used_motors.emplace(channel.board_id, channel.motor_number);
      if (!inserted) {
        throw std::runtime_error(
          channel.label + " duplicates board/motor mapping: board=" +
          std::to_string(channel.board_id) + " motor=" + std::to_string(channel.motor_number));
      }
    };

    for (const auto & channel : wheel_channels_) {
      validate_channel(channel);
    }
    for (const auto & channel : steer_channels_) {
      validate_channel(channel);
    }
  }

  void createInterfaces()
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    sabacan_power_ref_pub_ =
      this->create_publisher<sabacan_msgs::msg::SabacanPowerRef>(power_ref_topic_, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JoyToCmdVelNode::joyCallback, this, std::placeholders::_1));
    swerve_drive_ref_sub_ = this->create_subscription<r1_msgs::msg::SwerveDrive>(
      swerve_drive_ref_topic_, 10,
      std::bind(&JoyToCmdVelNode::swerveDriveRefCallback, this, std::placeholders::_1));

    robomas_reset_client_id1_ =
      this->create_client<sabacan_msgs::srv::SabacanReset>(robomas_reset_service_id1_);
    robomas_reset_client_id2_ =
      this->create_client<sabacan_msgs::srv::SabacanReset>(robomas_reset_service_id2_);

    // test_node は single_ref の出力口も自前で持ち、swerve と Sabacan の間を直接つなぐ。
    createSabacanSingleRefPublishers();
    createMotorRefSubscriptions();
    createDebugStatusPublishers();
    createSabacanStatusSubscriptions();

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / timer_rate_)),
      std::bind(&JoyToCmdVelNode::publishCmdVel, this));
  }

  void createSabacanSingleRefPublishers()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      // board_id / motor_number の対応は launch / yaml 側から差し替え可能。
      wheel_single_ref_pubs_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          "/sabacan_robomas_ref" + std::to_string(wheel_channels_[i].board_id) + "/motor" +
            std::to_string(wheel_channels_[i].motor_number),
          10);
      steer_single_ref_pubs_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          "/sabacan_robomas_ref" + std::to_string(steer_channels_[i].board_id) + "/motor" +
            std::to_string(steer_channels_[i].motor_number),
          10);
    }
  }

  void createMotorRefSubscriptions()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      // swerve 経由だけでなく、各軸へ直接 MotorRef を流して試験できるようにしてある。
      wheel_motor_ref_subs_[i] = this->create_subscription<r1_msgs::msg::MotorRef>(
        wheel_channels_[i].motor_ref_topic, 10,
        [this, i](const r1_msgs::msg::MotorRef::SharedPtr msg) {
          routeMotorRef(wheel_channels_[i], wheel_single_ref_pubs_[i], *msg);
        });
      steer_motor_ref_subs_[i] = this->create_subscription<r1_msgs::msg::MotorRef>(
        steer_channels_[i].motor_ref_topic, 10,
        [this, i](const r1_msgs::msg::MotorRef::SharedPtr msg) {
          routeMotorRef(steer_channels_[i], steer_single_ref_pubs_[i], *msg);
        });
    }
  }

  void createDebugStatusPublishers()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      wheel_debug_status_pubs_[i] =
        this->create_publisher<r1_msgs::msg::Motor>(wheel_channels_[i].debug_status_topic, 10);
      steer_debug_status_pubs_[i] =
        this->create_publisher<r1_msgs::msg::Motor>(steer_channels_[i].debug_status_topic, 10);
    }
  }

  void createSabacanStatusSubscriptions()
  {
    std::set<int> board_ids;
    for (const auto & channel : wheel_channels_) {
      board_ids.insert(channel.board_id);
    }
    for (const auto & channel : steer_channels_) {
      board_ids.insert(channel.board_id);
    }

    for (int board_id : board_ids) {
      // 物理基板ごとに status topic は 1 本なので、必要な board_id だけ購読する。
      sabacan_status_subs_.push_back(
        this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
          "/sabacan_robomas_status" + std::to_string(board_id), 10,
          [this, board_id](const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg) {
            handleSabacanStatus(board_id, msg);
          }));
    }
  }

  void logConfiguration()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "subscribing %s and %s, publishing %s (max_velocity: %.2f, max_angular_velocity: %.2f, "
      "deadzone: %.2f, timer_rate: %.1f Hz, power_ref: %s)",
      joy_topic_.c_str(), swerve_drive_ref_topic_.c_str(), cmd_vel_topic_.c_str(), max_velocity_,
      max_angular_velocity_, deadzone_, timer_rate_, power_ref_topic_.c_str());

    for (const auto & channel : wheel_channels_) {
      RCLCPP_INFO(
        this->get_logger(), "%s: %s -> board %d motor %d, debug %s", channel.label.c_str(),
        channel.motor_ref_topic.c_str(), channel.board_id, channel.motor_number,
        channel.debug_status_topic.c_str());
    }
    for (const auto & channel : steer_channels_) {
      RCLCPP_INFO(
        this->get_logger(), "%s: %s -> board %d motor %d, debug %s", channel.label.c_str(),
        channel.motor_ref_topic.c_str(), channel.board_id, channel.motor_number,
        channel.debug_status_topic.c_str());
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) { ps4_->joy_callback(msg); }

  void swerveDriveRefCallback(const r1_msgs::msg::SwerveDrive::SharedPtr msg)
  {
    // r1_swerve_drive_node の出力を 4 輪の wheel / steer 用 MotorRef として扱う。
    const std::array<float, MOTOR_COUNT> wheel_refs = {
      msg->omega0, msg->omega1, msg->omega2, msg->omega3};
    const std::array<float, MOTOR_COUNT> steer_refs = {
      msg->theta0, msg->theta1, msg->theta2, msg->theta3};

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      r1_msgs::msg::MotorRef wheel_ref;
      wheel_ref.control_type = wheel_control_type_;
      wheel_ref.ref = wheel_refs[i];
      routeMotorRef(wheel_channels_[i], wheel_single_ref_pubs_[i], wheel_ref);

      r1_msgs::msg::MotorRef steer_ref;
      steer_ref.control_type = steer_control_type_;
      steer_ref.ref = steer_refs[i];
      routeMotorRef(steer_channels_[i], steer_single_ref_pubs_[i], steer_ref);
    }
  }

  void publishCmdVel()
  {
    ps4_->update();
    handleButtonEvents();

    geometry_msgs::msg::Twist cmd_vel;
    if (ps4_->is_connected()) {
      // PS4 クラス側で deadzone 適用済みのスティック値を速度指令へ変換する。
      cmd_vel.linear.x = -max_velocity_ * ps4_->data.left_stick_x;
      cmd_vel.linear.y = max_velocity_ * ps4_->data.left_stick_y;
      cmd_vel.angular.z = max_angular_velocity_ * ps4_->data.right_stick_x;
    }

    cmd_vel_pub_->publish(cmd_vel);
  }

  void handleButtonEvents()
  {
    // PS はロボマス基板の再初期化、Options は power_ref の EMS トグルに割り当てる。
    if (ps4_->is_pushed_ps()) {
      sendSabacanReset(robomas_reset_client_id1_, robomas_reset_service_id1_);
      sendSabacanReset(robomas_reset_client_id2_, robomas_reset_service_id2_);
    }

    if (ps4_->is_pushed_options()) {
      publishSabacanPowerRef(!sabacan_is_ems_);
    }
  }

  void routeMotorRef(
    const MotorChannel & channel,
    const rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr &
      publisher,
    const r1_msgs::msg::MotorRef & motor_ref)
  {
    const std::string actual_control_type = normalizeControlType(motor_ref.control_type);
    if (!actual_control_type.empty() && actual_control_type != channel.expected_control_type) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "%s control_type mismatch: expected %s, got %s", channel.label.c_str(),
        channel.expected_control_type.c_str(), motor_ref.control_type.c_str());
    }

    if (!publisher) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "sabacan single ref publisher is not configured for %s", channel.label.c_str());
      return;
    }

    // single_control_node 側で制御モード切替も扱うため、control_type ごと渡す。
    sabacan_single_control_msgs::msg::SabacanRobomasSingleRef single_ref;
    single_ref.control_type =
      actual_control_type.empty() ? channel.expected_control_type : actual_control_type;
    single_ref.ref = motor_ref.ref;
    publisher->publish(single_ref);
  }

  void handleSabacanStatus(
    int board_id, const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    const auto motor_status = toMotorStatus(*msg);

    // board_id / motor_number が一致したデバッグ用 topic にだけ再配信する。
    publishDebugStatus(
      wheel_channels_, wheel_debug_status_pubs_, board_id, static_cast<int>(msg->motor_number),
      motor_status);
    publishDebugStatus(
      steer_channels_, steer_debug_status_pubs_, board_id, static_cast<int>(msg->motor_number),
      motor_status);
  }

  template <typename ChannelArray, typename PublisherArray>
  void publishDebugStatus(
    const ChannelArray & channels, const PublisherArray & publishers, int board_id,
    int motor_number, const r1_msgs::msg::Motor & motor_status)
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if (channels[i].board_id == board_id && channels[i].motor_number == motor_number) {
        // 論理名ごとの topic に流し直しておくと、board 配線を意識せずに監視できる。
        publishers[i]->publish(motor_status);
      }
    }
  }

  void sendSabacanReset(
    const rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr & client,
    const std::string & service_name)
  {
    // ボタン連打時でも詰まらないよう、待ち時間 0 秒でサービス有無だけ確認する。
    if (!client->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_WARN(
        this->get_logger(), "sabacan reset service is not available: %s", service_name.c_str());
      return;
    }

    auto request = std::make_shared<sabacan_msgs::srv::SabacanReset::Request>();
    client->async_send_request(
      request,
      [this, service_name](rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "sabacan reset sent: %s", service_name.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "sabacan reset failed: %s (%s)", service_name.c_str(),
            response->message.c_str());
        }
      });
  }

  void publishSabacanPowerRef(bool is_ems)
  {
    sabacan_msgs::msg::SabacanPowerRef msg;
    msg.is_ems = is_ems;
    sabacan_is_ems_ = is_ems;
    sabacan_power_ref_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "sabacan power ref is_ems: %d", is_ems);
  }

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string swerve_drive_ref_topic_;
  std::string power_ref_topic_;
  std::string robomas_reset_service_id1_;
  std::string robomas_reset_service_id2_;
  double timer_rate_;
  double max_velocity_;
  double max_angular_velocity_;
  double deadzone_;
  bool sabacan_is_ems_;
  std::string wheel_control_type_;
  std::string steer_control_type_;

  std::shared_ptr<PS4> ps4_;
  std::array<MotorChannel, MOTOR_COUNT> wheel_channels_;
  std::array<MotorChannel, MOTOR_COUNT> steer_channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr sabacan_power_ref_pub_;
  std::array<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr,
    MOTOR_COUNT>
    wheel_single_ref_pubs_;
  std::array<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr,
    MOTOR_COUNT>
    steer_single_ref_pubs_;
  std::array<rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr, MOTOR_COUNT>
    wheel_debug_status_pubs_;
  std::array<rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr, MOTOR_COUNT>
    steer_debug_status_pubs_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<r1_msgs::msg::SwerveDrive>::SharedPtr swerve_drive_ref_sub_;
  std::array<rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr, MOTOR_COUNT>
    wheel_motor_ref_subs_;
  std::array<rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr, MOTOR_COUNT>
    steer_motor_ref_subs_;
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_status_subs_;

  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr robomas_reset_client_id1_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr robomas_reset_client_id2_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
