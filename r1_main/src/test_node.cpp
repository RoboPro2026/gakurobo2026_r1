/**
 * @file test_node.cpp
 * @brief Joy から cmd_vel を出しつつ swerve 指令を Sabacan へ橋渡しする簡易ノード
 */

// NOTE: test用のプログラムなので、AIに適当に作ってもらったやつです。

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
#include "sabacan_msgs/msg/sabacan_power_status.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

constexpr size_t MOTOR_COUNT = 4;
using StringArray = std::array<std::string, MOTOR_COUNT>;
using IntArray = std::array<int, MOTOR_COUNT>;
constexpr uint32_t EMS_BIT_MASK = 1u << 0;
constexpr uint32_t SOFT_EMS_BIT_MASK = 1u << 1;

/**
 * @brief 可変長の文字列配列を 4 輪固定長の配列へ変換する。
 *
 * @param values パラメータから読み込んだ文字列配列
 * @param param_name 例外メッセージ用のパラメータ名
 * @return StringArray 4 要素に揃えた配列
 */
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

/**
 * @brief 可変長の整数配列を 4 輪固定長の配列へ変換する。
 *
 * @param values パラメータから読み込んだ整数配列
 * @param param_name 例外メッセージ用のパラメータ名
 * @return IntArray 4 要素に揃えた配列
 */
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

/**
 * @brief ASCII の英字だけを大文字へ正規化する。
 *
 * @param value 正規化前の文字列
 * @return std::string 大文字化した文字列
 */
std::string toUpperAscii(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (unsigned char c : value) {
    normalized.push_back(static_cast<char>(std::toupper(c)));
  }
  return normalized;
}

/**
 * @brief 制御モード文字列の表記揺れを吸収して正規化する。
 *
 * @param control_type 上流ノードや設定ファイルから渡される制御モード
 * @return std::string POSITION / VELOCITY などの正規化後の文字列
 */
std::string normalizeControlType(const std::string & control_type)
{
  // launch / YAML / 上流ノードの表記揺れをここで吸収する。
  std::string normalized = toUpperAscii(control_type);

  if (normalized == "POS") {
    return "POSITION";
  }
  if (normalized == "VEL") {
    return "VELOCITY";
  }
  return normalized;
}

/**
 * @brief 1 チャンネルで受け入れる制御モード一覧を正規化して重複なく保持する。
 *
 * @param control_types 許可したい制御モード候補
 * @return std::vector<std::string> 正規化済みの許可リスト
 */
std::vector<std::string> makeAcceptedControlTypes(std::initializer_list<std::string> control_types)
{
  std::vector<std::string> accepted;
  accepted.reserve(control_types.size());

  for (const auto & control_type : control_types) {
    const std::string normalized = normalizeControlType(control_type);
    if (normalized.empty()) {
      continue;
    }
    if (std::find(accepted.begin(), accepted.end(), normalized) == accepted.end()) {
      accepted.push_back(normalized);
    }
  }

  return accepted;
}

/**
 * @brief Sabacan の生 status をデバッグ用の Motor メッセージへ詰め替える。
 *
 * @param status Sabacan から受け取った生 status
 * @return r1_msgs::msg::Motor デバッグ表示に使う統一フォーマット
 */
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
  /**
   * @brief ノードを初期化し、必要な設定・インタフェースをまとめて構築する。
   */
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
    std::string default_control_type;
    std::vector<std::string> accepted_control_types;
    int board_id;
    int motor_number;
  };

  struct MotorCommand
  {
    std::string control_type;
    float ref;
  };

  /**
   * @brief すべての ROS パラメータを読み込み、内部状態へ反映する。
   */
  void loadParameters()
  {
    // PS4 は Joy callback と定周期 update を分けて使う前提なので、先に生成しておく。
    ps4_ = std::make_shared<PS4>(this->get_name());

    // test_node は Joy -> cmd_vel と、swerve -> Sabacan の橋渡しを両方持つ。
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    swerve_drive_ref_topic_ =
      this->declare_parameter<std::string>("swerve_drive_ref_topic", "/swerve_drive_ref");
    manual_swerve_drive_ref_topic_ = this->declare_parameter<std::string>(
      "manual_swerve_drive_ref_topic", "/manual_swerve_drive_ref");
    power_status_topic_ =
      this->declare_parameter<std::string>("power_status_topic", "/sabacan_power_status0");
    power_ref_topic_ =
      this->declare_parameter<std::string>("power_ref_topic", "/sabacan_power_ref0");
    robomas_reset_service_id1_ = this->declare_parameter<std::string>(
      "robomas_reset_service_id1", "/sabacan_robomas_reset_id1");
    robomas_reset_service_id2_ = this->declare_parameter<std::string>(
      "robomas_reset_service_id2", "/sabacan_robomas_reset_id2");

    set_swerve_drive_yaw_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/set_swerve_drive_yaw", 10);

    manual_swerve_drive_ref_pub_ =
      this->create_publisher<r1_msgs::msg::SwerveDrive>(manual_swerve_drive_ref_topic_, 10);

    timer_rate_ = this->declare_parameter<double>("timer_rate", 100.0);
    max_velocity_ = std::abs(this->declare_parameter<double>("max_velocity", 1.0));
    max_angular_velocity_ = std::abs(this->declare_parameter<double>("max_angular_velocity", 1.0));
    deadzone_ = this->declare_parameter<double>("deadzone", 0.2);
    sabacan_is_ems_ = this->declare_parameter<bool>("initial_is_ems", false);

    wheel_control_type_ =
      normalizeControlType(this->declare_parameter<std::string>("wheel_control_type", "VELOCITY"));
    normal_steer_control_type_ =
      normalizeControlType(this->declare_parameter<std::string>("steer_control_type", "POSITION"));
    steer_emergency_control_type_ = normalizeControlType(
      this->declare_parameter<std::string>("steer_emergency_control_type", "TORQUE"));
    steer_vesc_emergency_control_type_ = normalizeControlType(
      this->declare_parameter<std::string>("steer_vesc_emergency_control_type", "CURRENT"));
    steer_emergency_ref_ =
      static_cast<float>(this->declare_parameter<double>("steer_emergency_ref", 0.0));
    steer_reinit_required_ = sabacan_is_ems_;
    emergency_release_confirmed_ = !sabacan_is_ems_;

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
      wheel_channels_[i] = MotorChannel{
        wheel_labels[i],
        wheel_motor_ref_topics[i],
        wheel_debug_status_topics[i],
        wheel_control_type_,
        makeAcceptedControlTypes({wheel_control_type_}),
        wheel_board_ids[i],
        wheel_motor_numbers[i]};
      steer_channels_[i] = MotorChannel{
        steer_labels[i],
        steer_motor_ref_topics[i],
        steer_debug_status_topics[i],
        normal_steer_control_type_,
        makeAcceptedControlTypes(
          {normal_steer_control_type_, steer_emergency_control_type_,
           steer_vesc_emergency_control_type_}),
        steer_board_ids[i],
        steer_motor_numbers[i]};
    }
  }

  /**
   * @brief 読み込んだパラメータの妥当性を検証し、必要なら補正する。
   */
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

  /**
   * @brief Publisher / Subscriber / Client / Timer を生成する。
   */
  void createInterfaces()
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    sabacan_power_ref_pub_ =
      this->create_publisher<sabacan_msgs::msg::SabacanPowerRef>(power_ref_topic_, 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JoyToCmdVelNode::joyCallback, this, std::placeholders::_1));
    sabacan_power_status_sub_ = this->create_subscription<sabacan_msgs::msg::SabacanPowerStatus>(
      power_status_topic_, 10,
      std::bind(&JoyToCmdVelNode::sabacanPowerStatusCallback, this, std::placeholders::_1));
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

  /**
   * @brief 各 wheel / steer 軸に対応する single_ref publisher を生成する。
   */
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

  /**
   * @brief 外部から直接 MotorRef を流して試験できるように各軸の購読口を作る。
   */
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

  /**
   * @brief 監視しやすい論理名 topic へ流し直すための debug publisher を生成する。
   */
  void createDebugStatusPublishers()
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      wheel_debug_status_pubs_[i] =
        this->create_publisher<r1_msgs::msg::Motor>(wheel_channels_[i].debug_status_topic, 10);
      steer_debug_status_pubs_[i] =
        this->create_publisher<r1_msgs::msg::Motor>(steer_channels_[i].debug_status_topic, 10);
    }
  }

  /**
   * @brief 使用中の board_id に対する Sabacan status 購読を作成する。
   */
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

  /**
   * @brief 起動時に主要設定と配線結果をログへ出力する。
   */
  void logConfiguration()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "subscribing %s, %s, %s and publishing %s (manual_swerve: %s, max_velocity: %.2f, "
      "max_angular_velocity: %.2f, deadzone: %.2f, timer_rate: %.1f Hz, power_ref: %s)",
      joy_topic_.c_str(), swerve_drive_ref_topic_.c_str(), power_status_topic_.c_str(),
      cmd_vel_topic_.c_str(), manual_swerve_drive_ref_topic_.c_str(), max_velocity_,
      max_angular_velocity_, deadzone_, timer_rate_, power_ref_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "power_status: %s, steer normal: %s, steer emergency: %s, steer vesc emergency: %s, "
      "steer emergency ref: %.3f",
      power_status_topic_.c_str(), normal_steer_control_type_.c_str(),
      steer_emergency_control_type_.c_str(), steer_vesc_emergency_control_type_.c_str(),
      steer_emergency_ref_);

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

  /**
   * @brief Joy メッセージを PS4 ラッパへ転送する。
   *
   * @param msg 受信した Joy メッセージ
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) { ps4_->joy_callback(msg); }

  /**
   * @brief 電源基板の状態から EMS 発報中かどうかを判定し、ステア停止ラッチを更新する。
   *
   * @param msg Sabacan 電源基板の状態メッセージ
   */
  void sabacanPowerStatusCallback(const sabacan_msgs::msg::SabacanPowerStatus::SharedPtr msg)
  {
    const auto pcu_state = static_cast<uint32_t>(std::lround(msg->pcu_state));
    const bool emergency_feedback_active =
      (pcu_state & EMS_BIT_MASK) != 0u || (pcu_state & SOFT_EMS_BIT_MASK) != 0u;

    if (emergency_feedback_active_ == emergency_feedback_active) {
      return;
    }

    emergency_feedback_active_ = emergency_feedback_active;
    if (emergency_feedback_active_) {
      steer_reinit_required_ = true;
      emergency_release_confirmed_ = false;
      RCLCPP_WARN(this->get_logger(), "sabacan power status entered emergency");
    } else {
      emergency_release_confirmed_ = true;
      RCLCPP_INFO(this->get_logger(), "sabacan power status cleared emergency");
    }
  }

  /**
   * @brief swerve 指令を 4 輪の wheel / steer 用 MotorRef へ分解して配信する。
   *
   * @param msg 4 輪分の swerve 指令
   */
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

      const auto steer_command = makeSteerCommand(i, steer_refs[i]);
      r1_msgs::msg::MotorRef steer_ref;
      steer_ref.control_type = steer_command.control_type;
      steer_ref.ref = steer_command.ref;
      routeMotorRef(steer_channels_[i], steer_single_ref_pubs_[i], steer_ref);
    }
  }

  /**
   * @brief 定周期で PS4 状態を更新し、速度指令を生成して publish する。
   */
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

  /**
   * @brief swerve ノードのヨー基準を指定値へ合わせる指令を publish する。
   *
   * @param yaw 新しいヨー基準 [rad]
   */
  void publishSetSwerveDriveYaw(double yaw)
  {
    std_msgs::msg::Float64 msg;
    msg.data = yaw;
    set_swerve_drive_yaw_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "set swerve drive yaw: %.3f", yaw);
  }

  /**
   * @brief manual_swerve_drive_ref を使って 4 輪のステア角を 0 度へ揃える。
   */
  void publishManualSwerveDriveZero()
  {
    if (!manual_swerve_drive_ref_pub_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "manual_swerve_drive_ref publisher is not configured");
      return;
    }

    r1_msgs::msg::SwerveDrive msg;
    msg.omega0 = 0.0;
    msg.omega1 = 0.0;
    msg.omega2 = 0.0;
    msg.omega3 = 0.0;
    msg.theta0 = 0.0;
    msg.theta1 = 0.0;
    msg.theta2 = 0.0;
    msg.theta3 = 0.0;
    manual_swerve_drive_ref_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "published manual_swerve_drive_ref to align steer to 0 deg");
  }

  /**
   * @brief PS4 のエッジ入力を処理し、初期化や EMS トグルを実行する。
   */
  void handleButtonEvents()
  {
    // PS はロボマス基板の再初期化、Options は power_ref の EMS トグルに割り当てる。
    if (ps4_->is_pushed_ps()) {
      handleInitializeCommand();
    }

    if (ps4_->is_pushed_options()) {
      publishSabacanPowerRef(!sabacan_is_ems_);
    }
  }

  /**
   * @brief MotorRef を single_control_node が受け取る単軸指令へ変換して publish する。
   *
   * @param channel 出力先チャンネル情報
   * @param publisher publish 先の single_ref publisher
   * @param motor_ref 上流から受け取った MotorRef
   */
  void routeMotorRef(
    const MotorChannel & channel,
    const rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr &
      publisher,
    const r1_msgs::msg::MotorRef & motor_ref)
  {
    const std::string actual_control_type = normalizeControlType(motor_ref.control_type);
    if (
      !actual_control_type.empty() &&
      std::find(
        channel.accepted_control_types.begin(), channel.accepted_control_types.end(),
        actual_control_type) == channel.accepted_control_types.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "%s control_type mismatch: default %s, got %s", channel.label.c_str(),
        channel.default_control_type.c_str(), motor_ref.control_type.c_str());
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
      actual_control_type.empty() ? channel.default_control_type : actual_control_type;
    single_ref.ref = motor_ref.ref;
    publisher->publish(single_ref);
  }

  /**
   * @brief 現在の安全状態に応じてステア軸の制御モードと目標値を決定する。
   *
   * @param index ステア軸のインデックス
   * @param target_theta 通常時に追従したいステア角 [rad]
   * @return MotorCommand 現在 publish すべき control_type と ref
   */
  MotorCommand makeSteerCommand(size_t index, float target_theta) const
  {
    if (!isSteerPositionControlEnabled()) {
      return {emergencySteerControlType(index), steer_emergency_ref_};
    }

    return {normal_steer_control_type_, target_theta};
  }

  /**
   * @brief ステア軸を位置制御モードで動かしてよい状態かを返す。
   *
   * @return true EMS が解除され、再初期化も完了している
   * @return false 停止モードを維持すべき
   */
  bool isSteerPositionControlEnabled() const
  {
    return !(sabacan_is_ems_ || emergency_feedback_active_ || steer_reinit_required_);
  }

  /**
   * @brief 非常停止中に使うステア軸の制御モードをモータ種別から決める。
   *
   * @param index ステア軸のインデックス
   * @return std::string VESC なら CURRENT、それ以外は TORQUE
   */
  std::string emergencySteerControlType(size_t index) const
  {
    if (index < MOTOR_COUNT && steer_status_received_[index]) {
      if (toUpperAscii(steer_latest_status_[index].motor_type) == "VESC") {
        return steer_vesc_emergency_control_type_;
      }
    }
    return steer_emergency_control_type_;
  }

  /**
   * @brief 初期化コマンドを処理し、条件を満たす場合だけステア位置制御を復帰させる。
   */
  void handleInitializeCommand()
  {
    const uint64_t request_id = ++initialize_request_id_;
    pending_initialize_reset_responses_ = 0;
    initialize_reset_failed_ = false;

    const bool any_request_sent =
      sendSabacanReset(robomas_reset_client_id1_, robomas_reset_service_id1_, request_id) |
      sendSabacanReset(robomas_reset_client_id2_, robomas_reset_service_id2_, request_id);

    publishSetSwerveDriveYaw(0.0);
    if (!any_request_sent) {
      initialize_reset_failed_ = true;
      RCLCPP_WARN(this->get_logger(), "initialize command could not send any reset request");
    }

    completeInitializeCommandIfReady(request_id);
  }

  /**
   * @brief Sabacan の status を受けて、ステア軸のモータ種別キャッシュと debug 出力を更新する。
   *
   * @param board_id status を受け取った board_id
   * @param msg Sabacan からの status
   */
  void handleSabacanStatus(
    int board_id, const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    const auto motor_status = toMotorStatus(*msg);

    cacheMotorStatus(
      steer_channels_, steer_latest_status_, steer_status_received_, board_id,
      static_cast<int>(msg->motor_number), motor_status);

    // board_id / motor_number が一致したデバッグ用 topic にだけ再配信する。
    publishDebugStatus(
      wheel_channels_, wheel_debug_status_pubs_, board_id, static_cast<int>(msg->motor_number),
      motor_status);
    publishDebugStatus(
      steer_channels_, steer_debug_status_pubs_, board_id, static_cast<int>(msg->motor_number),
      motor_status);
  }

  /**
   * @brief board_id / motor_number に一致する軸の最新 status をキャッシュする。
   *
   * @tparam ChannelArray チャンネル配列の型
   * @tparam StatusArray status キャッシュ配列の型
   * @tparam ReceivedArray 受信済みフラグ配列の型
   * @param channels 軸の配線情報
   * @param status_cache 最新 status の保存先
   * @param received 受信済みフラグの保存先
   * @param board_id status の board_id
   * @param motor_number status の motor_number
   * @param motor_status 保存したい status
   */
  template <typename ChannelArray, typename StatusArray, typename ReceivedArray>
  void cacheMotorStatus(
    const ChannelArray & channels, StatusArray & status_cache, ReceivedArray & received,
    int board_id, int motor_number, const r1_msgs::msg::Motor & motor_status)
  {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if (channels[i].board_id == board_id && channels[i].motor_number == motor_number) {
        status_cache[i] = motor_status;
        received[i] = true;
      }
    }
  }

  /**
   * @brief board_id / motor_number に一致する debug topic へ status を再配信する。
   *
   * @tparam ChannelArray チャンネル配列の型
   * @tparam PublisherArray publisher 配列の型
   * @param channels 軸の配線情報
   * @param publishers debug topic の publisher 群
   * @param board_id status の board_id
   * @param motor_number status の motor_number
   * @param motor_status 再配信する status
   */
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

  /**
   * @brief 指定した Sabacan reset service へ非同期リクエストを送る。
   *
   * @param client reset service client
   * @param service_name ログ表示用のサービス名
   * @param request_id 初期化シーケンスの世代番号
   * @return true リクエストを送信できた
   * @return false サービス未接続などで送信できなかった
   */
  bool sendSabacanReset(
    const rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr & client,
    const std::string & service_name, uint64_t request_id)
  {
    // ボタン連打時でも詰まらないよう、待ち時間 0 秒でサービス有無だけ確認する。
    if (!client->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_WARN(
        this->get_logger(), "sabacan reset service is not available: %s", service_name.c_str());
      initialize_reset_failed_ = true;
      return false;
    }

    ++pending_initialize_reset_responses_;
    auto request = std::make_shared<sabacan_msgs::srv::SabacanReset::Request>();
    client->async_send_request(
      request, [this, service_name,
                request_id](rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "sabacan reset sent: %s", service_name.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "sabacan reset failed: %s (%s)", service_name.c_str(),
            response->message.c_str());
        }
        handleInitializeResetResponse(request_id, response->success);
      });
    return true;
  }

  /**
   * @brief 初期化用 reset の非同期応答を集約し、全応答後に後処理を実行する。
   *
   * @param request_id 応答が属する初期化シーケンスの世代番号
   * @param success 対象 reset request の成否
   */
  void handleInitializeResetResponse(uint64_t request_id, bool success)
  {
    if (request_id != initialize_request_id_) {
      return;
    }

    if (pending_initialize_reset_responses_ > 0) {
      --pending_initialize_reset_responses_;
    }
    if (!success) {
      initialize_reset_failed_ = true;
    }

    completeInitializeCommandIfReady(request_id);
  }

  /**
   * @brief 初期化シーケンスが完了していれば、位置制御復帰と 0 度合わせを実行する。
   *
   * @param request_id 確認対象の初期化シーケンス世代番号
   */
  void completeInitializeCommandIfReady(uint64_t request_id)
  {
    if (request_id != initialize_request_id_) {
      return;
    }
    if (pending_initialize_reset_responses_ != 0) {
      return;
    }
    if (initialize_reset_failed_) {
      RCLCPP_WARN(
        this->get_logger(),
        "initialize command did not complete successfully. keep current steer state");
      return;
    }

    if (!sabacan_is_ems_ && emergency_release_confirmed_) {
      if (steer_reinit_required_) {
        RCLCPP_INFO(this->get_logger(), "steer position control restored after initialization");
      }
      steer_reinit_required_ = false;
      publishManualSwerveDriveZero();
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "initialize completed before emergency clear was confirmed. keep steer in stop mode");
    }
  }

  /**
   * @brief EMS の要求状態を publish し、ローカルの停止ラッチも更新する。
   *
   * @param is_ems true なら EMS 発報、false なら解除要求
   */
  void publishSabacanPowerRef(bool is_ems)
  {
    sabacan_msgs::msg::SabacanPowerRef msg;
    msg.is_ems = is_ems;
    sabacan_is_ems_ = is_ems;
    if (is_ems) {
      steer_reinit_required_ = true;
      emergency_release_confirmed_ = false;
    }
    sabacan_power_ref_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "sabacan power ref is_ems: %d", is_ems);
  }

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string swerve_drive_ref_topic_;
  std::string manual_swerve_drive_ref_topic_;
  std::string power_status_topic_;
  std::string power_ref_topic_;
  std::string robomas_reset_service_id1_;
  std::string robomas_reset_service_id2_;
  double timer_rate_;
  double max_velocity_;
  double max_angular_velocity_;
  double deadzone_;
  bool sabacan_is_ems_;
  bool emergency_feedback_active_ = false;
  bool emergency_release_confirmed_ = true;
  bool steer_reinit_required_ = false;
  uint64_t initialize_request_id_ = 0;
  size_t pending_initialize_reset_responses_ = 0;
  bool initialize_reset_failed_ = false;
  std::string wheel_control_type_;
  std::string normal_steer_control_type_;
  std::string steer_emergency_control_type_;
  std::string steer_vesc_emergency_control_type_;
  float steer_emergency_ref_;

  std::shared_ptr<PS4> ps4_;
  std::array<MotorChannel, MOTOR_COUNT> wheel_channels_;
  std::array<MotorChannel, MOTOR_COUNT> steer_channels_;
  std::array<r1_msgs::msg::Motor, MOTOR_COUNT> steer_latest_status_;
  std::array<bool, MOTOR_COUNT> steer_status_received_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<r1_msgs::msg::SwerveDrive>::SharedPtr manual_swerve_drive_ref_pub_;
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
  rclcpp::Subscription<sabacan_msgs::msg::SabacanPowerStatus>::SharedPtr sabacan_power_status_sub_;
  rclcpp::Subscription<r1_msgs::msg::SwerveDrive>::SharedPtr swerve_drive_ref_sub_;
  std::array<rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr, MOTOR_COUNT>
    wheel_motor_ref_subs_;
  std::array<rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr, MOTOR_COUNT>
    steer_motor_ref_subs_;
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_status_subs_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_swerve_drive_yaw_pub_;

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
