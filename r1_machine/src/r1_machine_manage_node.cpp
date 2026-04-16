/**
 * @file r1_machine_manage_node.cpp
 * @author Yamaguchi Yudai
 * @brief sabacan_msgs と r1_machine 間のメッセージ変換を行うノード。
 * @version 0.1
 * @date 2025-10-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "r1_msgs/msg/angle_motion.hpp"
#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/gpio_pwm_ref.hpp"
#include "r1_msgs/msg/gpio_servo_ref.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/mecanum.hpp"
#include "r1_msgs/msg/motor.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "r1_msgs/msg/odometry_encoder.hpp"
#include "r1_msgs/msg/swerve_drive.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_float.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_int.hpp"
#include "sabacan_msgs/msg/sabacan_led_ref.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_status.hpp"
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sabacan_msgs/msg/sabacan_power_status.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_msgs/msg/sabacan_robstride_ref.hpp"
#include "sabacan_msgs/msg/sabacan_robstride_status.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sabacan_msgs/srv/set_robomas_gains.hpp"
#include "sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

constexpr uint32_t kEmsBitMask = 1u << 0;
constexpr uint32_t kSoftEmsBitMask = 1u << 1;
constexpr const char * kSwerveWheelControlType = "VELOCITY";
constexpr const char * kSwerveSteerControlType = "POSITION";
constexpr const char * kRobstrideMotorType = "ROBSTRIDE";
constexpr const char * kRobstridePositionControlType = "PP";

// 使用するデバイスの数
// Robomas / GPIO は board_id 1..N を使用する。
// Power / LED はそれぞれ 1 台だけを単独で扱う。
constexpr int kSabacanRobomasNumber = 7;
constexpr int kSabacanGpioNumber = 3;
constexpr int kSabacanRobstrideNumber = 1;
constexpr int kSabacanPowerNumber = 1;
constexpr int kSabacanLedNumber = 1;
constexpr std::size_t kSabacanBoardSlotCount = static_cast<std::size_t>(
  std::max({kSabacanRobomasNumber, kSabacanGpioNumber, kSabacanRobstrideNumber}) + 1);
constexpr std::size_t kSabacanResetServiceNumber = static_cast<std::size_t>(
  kSabacanPowerNumber + kSabacanLedNumber + kSabacanRobomasNumber + kSabacanGpioNumber);

/**
 * @brief 足回りの制御方式を表す列挙型。
 */
enum class DriveMode
{
  Mecanum,
  Swerve,
};

/**
 * @brief 各チャネルが有効になる drive mode の範囲を表す。
 */
enum class ChannelAvailability
{
  Both,
  MecanumOnly,
  SwerveOnly,
};

/**
 * @brief モータチャネルが利用する Sabacan 側の制御 backend を表す。
 */
enum class MotorControllerType
{
  Robomas,
  Vesc,
  Robstride,
};

/**
 * @brief ASCII の英字だけを大文字へ正規化する。
 * @param value 正規化前文字列
 * @return std::string 大文字化した文字列
 */
static std::string to_upper_ascii(const std::string & value)
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
 * @param control_type 正規化前の制御モード文字列
 * @return std::string 正規化済み制御モード文字列
 */
static std::string normalize_control_type(const std::string & control_type)
{
  const std::string normalized = to_upper_ascii(control_type);
  if (normalized == "POS") {
    return "POSITION";
  }
  if (normalized == "VEL") {
    return "VELOCITY";
  }
  return normalized;
}

/**
 * @brief drive mode 文字列を enum に変換する。
 * @param drive_mode 文字列で指定された drive mode
 * @return DriveMode 変換結果
 */
static DriveMode parse_drive_mode(const std::string & drive_mode)
{
  const std::string normalized = to_upper_ascii(drive_mode);
  if (normalized == "MECANUM") {
    return DriveMode::Mecanum;
  }
  if (normalized == "SWERVE" || normalized == "STEER") {
    return DriveMode::Swerve;
  }
  throw std::invalid_argument("drive_mode must be 'mecanum' or 'swerve'");
}

/**
 * @brief drive mode をログ出力向けの文字列へ変換する。
 * @param drive_mode 変換対象の drive mode
 * @return const char * 文字列表現
 */
static const char * drive_mode_name(DriveMode drive_mode)
{
  return drive_mode == DriveMode::Mecanum ? "mecanum" : "swerve";
}

/**
 * @brief 指定した availability が現在の drive mode で有効かを返す。
 * @param availability チャネルの有効範囲
 * @param drive_mode 現在の drive mode
 * @return true この mode で有効
 * @return false この mode では無効
 */
static bool is_channel_available(ChannelAvailability availability, DriveMode drive_mode)
{
  switch (availability) {
    case ChannelAvailability::Both:
      return true;
    case ChannelAvailability::MecanumOnly:
      return drive_mode == DriveMode::Mecanum;
    case ChannelAvailability::SwerveOnly:
      return drive_mode == DriveMode::Swerve;
  }
  return false;
}

/**
 * @brief SabacanPowerStatus から非常停止発報中かどうかを判定する。
 * @param msg 受信した電源状態メッセージ
 * @return true EMS または SOFT EMS が有効
 * @return false 非常停止ではない
 */
static bool is_emergency_active(const sabacan_msgs::msg::SabacanPowerStatus & msg)
{
  const auto pcu_state = static_cast<uint32_t>(std::lround(msg.pcu_state));
  return (pcu_state & kEmsBitMask) != 0u || (pcu_state & kSoftEmsBitMask) != 0u;
}

/**
 * @brief モータ種別文字列が VESC かどうかを返す。
 * @param motor_type Sabacan status に含まれる motor_type
 * @return true VESC
 * @return false VESC 以外
 */
static bool is_vesc_motor_type(const std::string & motor_type)
{
  return to_upper_ascii(motor_type) == "VESC";
}

/**
 * @brief モータ種別文字列が Robstride かどうかを返す。
 * @param motor_type モータ種別文字列
 * @return true Robstride
 * @return false Robstride 以外
 */
static bool is_robstride_motor_type(const std::string & motor_type)
{
  return to_upper_ascii(motor_type) == kRobstrideMotorType;
}

/**
 * @brief 非常停止時に CURRENT 0.0 停止を使うモータ種別かを返す。
 * @param motor_type モータ種別文字列
 * @return true CURRENT 停止
 * @return false CURRENT 以外の停止方式
 */
static bool is_current_control_motor_type(const std::string & motor_type)
{
  return is_vesc_motor_type(motor_type) || is_robstride_motor_type(motor_type);
}

/**
 * @brief controller_type に応じて単軸 Sabacan 指令の control_type を正規化する。
 * @param controller_type チャネルが利用する backend 種別
 * @param control_type 入力 control_type
 * @return std::string 正規化後の control_type
 */
static std::string normalize_single_ref_control_type(
  MotorControllerType controller_type, const std::string & control_type)
{
  const std::string normalized = normalize_control_type(control_type);
  if (controller_type == MotorControllerType::Vesc && normalized == "TORQUE") {
    return "CURRENT";
  }
  return normalized;
}

/**
 * @brief Sabacan 上の board_id と motor / pin 番号の組を表す。
 */
struct BoardInfo
{
  int board_id{};
  int number{};

  bool operator==(const BoardInfo & other) const = default;
};

/**
 * @brief 非常停止時の制御モード判定用に、最新の motor_type を保持する。
 */
struct MotorTypeCacheEntry
{
  BoardInfo board_info;
  std::string motor_type;
};

template <typename MsgT>
using PublisherPtr = typename rclcpp::Publisher<MsgT>::SharedPtr;

template <typename MsgT>
using SubscriptionPtr = typename rclcpp::Subscription<MsgT>::SharedPtr;

using SingleRefPublisher = PublisherPtr<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>;
using RobstrideRefPublisher = PublisherPtr<sabacan_msgs::msg::SabacanRobstrideRef>;
using SetRobomasGainsClient = rclcpp::Client<sabacan_msgs::srv::SetRobomasGains>;
using SabacanResetClient = rclcpp::Client<sabacan_msgs::srv::SabacanReset>;

struct SabacanResetServiceEntry
{
  std::string service_name;
  SabacanResetClient::SharedPtr client;
};

/**
 * @brief メカナム足回り 1 輪分の配線情報を保持する。
 */
struct MecanumWheelChannel
{
  /**
   * @brief 空のメカナムチャネルを生成する。
   */
  MecanumWheelChannel() = default;

  /**
   * @brief メカナムチャネル定義を生成する。
   * @param info ボード情報
   * @param topic デバッグ出力 topic
   * @param controller_type 利用する motor controller 種別
   * @param availability 有効となる drive mode
   */
  MecanumWheelChannel(
    BoardInfo info, std::string topic,
    MotorControllerType controller_type = MotorControllerType::Robomas,
    ChannelAvailability availability = ChannelAvailability::MecanumOnly)
  : board_info(info),
    debug_topic(std::move(topic)),
    controller_type(controller_type),
    availability(availability)
  {
  }

  BoardInfo board_info;
  std::string debug_topic;
  MotorControllerType controller_type{MotorControllerType::Robomas};
  ChannelAvailability availability{ChannelAvailability::MecanumOnly};
  SingleRefPublisher ref_publisher;
  PublisherPtr<r1_msgs::msg::Motor> debug_publisher;
};

/**
 * @brief 共通のモータ指令チャネル情報を保持する。
 * @tparam StatusMsgT 定期 publish する状態メッセージ型
 */
template <typename StatusMsgT>
struct MotorCommandChannel
{
  /**
   * @brief 空のモータチャネルを生成する。
   */
  MotorCommandChannel() = default;

  /**
   * @brief モータチャネル定義を生成する。
   * @param info ボード情報
   * @param ref_topic_name 指令値 topic
   * @param status_topic_name 状態出力 topic
   * @param debug_topic_name デバッグ出力 topic
   */
  MotorCommandChannel(
    BoardInfo info, std::string ref_topic_name, std::string status_topic_name,
    std::string debug_topic_name, std::string initialize_topic_name = "",
    MotorControllerType controller_type = MotorControllerType::Robomas,
    std::string torque_limit_topic_name = "")
  : board_info(info),
    ref_topic(std::move(ref_topic_name)),
    status_topic(std::move(status_topic_name)),
    debug_topic(std::move(debug_topic_name)),
    initialize_topic(std::move(initialize_topic_name)),
    torque_limit_topic(std::move(torque_limit_topic_name)),
    controller_type(controller_type)
  {
  }

  BoardInfo board_info;
  std::string ref_topic;
  std::string status_topic;
  std::string debug_topic;
  std::string initialize_topic;
  std::string torque_limit_topic;
  MotorControllerType controller_type{MotorControllerType::Robomas};
  SingleRefPublisher ref_publisher;
  RobstrideRefPublisher robstride_ref_publisher;
  SubscriptionPtr<r1_msgs::msg::MotorRef> ref_subscription;
  SubscriptionPtr<std_msgs::msg::Float64> torque_limit_subscription;
  PublisherPtr<StatusMsgT> status_publisher;
  PublisherPtr<r1_msgs::msg::Motor> debug_publisher;
  PublisherPtr<std_msgs::msg::Empty> initialize_publisher;
  StatusMsgT value{};
};

using LinearMotionChannel = MotorCommandChannel<r1_msgs::msg::LinearMotion>;
using AngleMotionChannel = MotorCommandChannel<r1_msgs::msg::AngleMotion>;
using MotorStatusChannel = MotorCommandChannel<r1_msgs::msg::Motor>;

/**
 * @brief `/name_suffix` 形式の topic 名を生成する。
 * @param name ベース名
 * @param suffix 末尾 suffix
 * @return std::string 生成した topic 名
 */
static std::string make_topic(const std::string & name, const std::string & suffix)
{
  return "/" + name + suffix;
}

/**
 * @brief `/debug_name_motor_status` 形式の debug topic 名を生成する。
 * @param name ベース名
 * @return std::string 生成した debug topic 名
 */
static std::string make_debug_motor_status_topic(const std::string & name)
{
  return "/debug_" + name + "_motor_status";
}

/**
 * @brief メカナム wheel 用 channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param controller_type 利用する backend
 * @return MecanumWheelChannel 生成した channel
 */
static MecanumWheelChannel make_mecanum_wheel_channel(
  BoardInfo info, const std::string & name,
  MotorControllerType controller_type = MotorControllerType::Robomas)
{
  return MecanumWheelChannel{info, make_debug_motor_status_topic(name), controller_type};
}

/**
 * @brief 速度制御系 channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param controller_type 利用する backend
 * @return MotorStatusChannel 生成した channel
 */
static MotorStatusChannel make_velocity_control_channel(
  BoardInfo info, const std::string & name,
  MotorControllerType controller_type = MotorControllerType::Robomas)
{
  return MotorStatusChannel{
    info,
    make_topic(name, "_motor_ref"),
    make_topic(name, "_motor_status"),
    make_debug_motor_status_topic(name),
    "",
    controller_type};
}

/**
 * @brief LinearMotion channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param controller_type 利用する backend
 * @return LinearMotionChannel 生成した channel
 */
static LinearMotionChannel make_linear_motion_channel(
  BoardInfo info, const std::string & name,
  MotorControllerType controller_type = MotorControllerType::Robomas)
{
  const bool enable_torque_limit = controller_type == MotorControllerType::Robomas;
  return LinearMotionChannel{
    info,
    make_topic(name, "_motor_ref"),
    make_topic(name, "_linear_motion_status"),
    make_debug_motor_status_topic(name),
    make_topic(name, "_initialize"),
    controller_type,
    enable_torque_limit ? make_topic(name, "_torque_limit_ref") : ""};
}

/**
 * @brief AngleMotion channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param controller_type 利用する backend
 * @return AngleMotionChannel 生成した channel
 */
static AngleMotionChannel make_angle_motion_channel(
  BoardInfo info, const std::string & name,
  MotorControllerType controller_type = MotorControllerType::Robomas)
{
  const bool enable_torque_limit = controller_type == MotorControllerType::Robomas;
  return AngleMotionChannel{
    info,
    make_topic(name, "_motor_ref"),
    make_topic(name, "_angle_motion_status"),
    make_debug_motor_status_topic(name),
    make_topic(name, "_initialize"),
    controller_type,
    enable_torque_limit ? make_topic(name, "_torque_limit_ref") : ""};
}

/**
 * @brief 独ステの wheel / steer 軸 1 本分の配線情報を保持する。
 */
struct DriveMotorChannel
{
  /**
   * @brief 空の独ステチャネルを生成する。
   */
  DriveMotorChannel() = default;

  /**
   * @brief 独ステチャネル定義を生成する。
   * @param info ボード情報
   * @param ref_topic_name 軸ごとの MotorRef topic
   * @param debug_topic_name デバッグ出力 topic
   * @param controller_type 利用する motor controller 種別
   * @param channel_availability 有効となる drive mode
   */
  DriveMotorChannel(
    BoardInfo info, std::string ref_topic_name, std::string status_topic_name,
    std::string debug_topic_name,
    MotorControllerType controller_type = MotorControllerType::Robomas,
    ChannelAvailability channel_availability = ChannelAvailability::SwerveOnly)
  : board_info(info),
    ref_topic(std::move(ref_topic_name)),
    status_topic(std::move(status_topic_name)),
    debug_topic(std::move(debug_topic_name)),
    controller_type(controller_type),
    availability(channel_availability)
  {
  }

  BoardInfo board_info;
  std::string ref_topic;
  std::string status_topic;
  std::string debug_topic;
  MotorControllerType controller_type{MotorControllerType::Robomas};
  ChannelAvailability availability{ChannelAvailability::SwerveOnly};
  SingleRefPublisher ref_publisher;
  SubscriptionPtr<r1_msgs::msg::MotorRef> ref_subscription;
  PublisherPtr<r1_msgs::msg::Motor> status_publisher;
  PublisherPtr<r1_msgs::msg::Motor> debug_publisher;
  r1_msgs::msg::Motor value{};
};

/**
 * @brief 独ステの単軸モータ channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param controller_type 利用する backend
 * @return DriveMotorChannel 生成した channel
 */
static DriveMotorChannel make_drive_motor_channel(
  BoardInfo info, const std::string & name,
  MotorControllerType controller_type = MotorControllerType::Robomas)
{
  return DriveMotorChannel{
    info, make_topic(name, "_motor_ref"), make_topic(name, "_motor_status"),
    make_debug_motor_status_topic(name), controller_type};
}

/**
 * @brief PWM 出力 GPIO の配線情報を保持する。
 */
struct GpioFloatOutputChannel
{
  /**
   * @brief 空の PWM 出力チャネルを生成する。
   */
  GpioFloatOutputChannel() = default;

  /**
   * @brief PWM 出力チャネル定義を生成する。
   * @param info ボード情報
   * @param topic 指令値 topic
   * @param channel_availability 有効となる drive mode
   */
  GpioFloatOutputChannel(
    BoardInfo info, std::string topic,
    ChannelAvailability channel_availability = ChannelAvailability::Both)
  : board_info(info), ref_topic(std::move(topic)), availability(channel_availability)
  {
  }

  BoardInfo board_info;
  std::string ref_topic;
  ChannelAvailability availability{ChannelAvailability::Both};
  SubscriptionPtr<r1_msgs::msg::GpioPwmRef> ref_subscription;
};

/**
 * @brief サーボ出力 GPIO の配線情報を保持する。
 */
struct GpioIntOutputChannel
{
  /**
   * @brief 空のサーボ出力チャネルを生成する。
   */
  GpioIntOutputChannel() = default;

  /**
   * @brief サーボ出力チャネル定義を生成する。
   * @param info ボード情報
   * @param topic 指令値 topic
   * @param channel_availability 有効となる drive mode
   */
  GpioIntOutputChannel(
    BoardInfo info, std::string topic,
    ChannelAvailability channel_availability = ChannelAvailability::Both)
  : board_info(info), ref_topic(std::move(topic)), availability(channel_availability)
  {
  }

  BoardInfo board_info;
  std::string ref_topic;
  ChannelAvailability availability{ChannelAvailability::Both};
  SubscriptionPtr<r1_msgs::msg::GpioServoRef> ref_subscription;
};

/**
 * @brief GPIO 入力チャネルの配線情報を保持する。
 */
struct GpioInputChannel
{
  /**
   * @brief 空の GPIO 入力チャネルを生成する。
   */
  GpioInputChannel() = default;

  /**
   * @brief GPIO 入力チャネル定義を生成する。
   * @param info ボード情報
   * @param status_topic_name 状態出力 topic
   * @param debug_topic_name デバッグ出力 topic
   * @param channel_availability 有効となる drive mode
   */
  GpioInputChannel(
    BoardInfo info, std::string status_topic_name, std::string debug_topic_name,
    ChannelAvailability channel_availability = ChannelAvailability::Both)
  : board_info(info),
    status_topic(std::move(status_topic_name)),
    debug_topic(std::move(debug_topic_name)),
    availability(channel_availability)
  {
  }

  BoardInfo board_info;
  std::string status_topic;
  std::string debug_topic;
  ChannelAvailability availability{ChannelAvailability::Both};
  PublisherPtr<r1_msgs::msg::GpioInput> status_publisher;
  PublisherPtr<r1_msgs::msg::GpioInput> debug_publisher;
  r1_msgs::msg::GpioInput value{};
};

/**
 * @brief `/debug_name_status` 形式の GPIO debug topic 名を生成する。
 * @param name ベース名
 * @return std::string 生成した debug topic 名
 */
static std::string make_debug_gpio_status_topic(const std::string & name)
{
  return "/debug_" + name + "_status";
}

/**
 * @brief PWM 出力 GPIO channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param availability 有効となる drive mode
 * @return GpioFloatOutputChannel 生成した channel
 */
static GpioFloatOutputChannel make_gpio_float_output_channel(
  BoardInfo info, const std::string & name,
  ChannelAvailability availability = ChannelAvailability::Both)
{
  return GpioFloatOutputChannel{info, make_topic(name, "_gpio_pwm_ref"), availability};
}

/**
 * @brief サーボ出力 GPIO channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param availability 有効となる drive mode
 * @return GpioIntOutputChannel 生成した channel
 */
[[maybe_unused]] static GpioIntOutputChannel make_gpio_int_output_channel(
  BoardInfo info, const std::string & name,
  ChannelAvailability availability = ChannelAvailability::Both)
{
  return GpioIntOutputChannel{info, make_topic(name, "_gpio_servo_ref"), availability};
}

/**
 * @brief GPIO 入力 channel を生成する。
 * @param info ボード情報
 * @param name ベース名
 * @param availability 有効となる drive mode
 * @return GpioInputChannel 生成した channel
 */
static GpioInputChannel make_gpio_input_channel(
  BoardInfo info, const std::string & name,
  ChannelAvailability availability = ChannelAvailability::Both)
{
  return GpioInputChannel{
    info, make_topic(name, "_status"), make_debug_gpio_status_topic(name), availability};
}

class MachineManageNode : public rclcpp::Node
{
public:
  /**
   * @brief ノードを初期化し、各種 publisher / subscription / timer を生成する。
   */
  MachineManageNode() : Node("r1_machine_manage_node")
  {
    sabacan_reset_last_send_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    post_reset_initialize_sent_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    load_drive_configuration();
    load_safety_configuration();
    initialize_sabacan_gpio_publishers();
    initialize_sabacan_status_subscriptions();
    initialize_sabacan_service_clients();
    initialize_safety_interfaces();
    initialize_drive_interfaces();
    initialize_machine_interfaces();
    initialize_publish_timers();

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MachineManageNode::parameter_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "drive_mode = %s", drive_mode_name(drive_mode_));
  }

private:
  static constexpr std::size_t FL = 0;
  static constexpr std::size_t FR = 1;
  static constexpr std::size_t RL = 2;
  static constexpr std::size_t RR = 3;
  static constexpr std::size_t X = 0;
  static constexpr std::size_t Y = 1;

  enum TimerIndex : std::size_t
  {
    TIMER_CHASSIS = 0,
    TIMER_ODOMETRY,
    TIMER_LINEAR_MOTION,
    TIMER_ANGLE_MOTION,
    TIMER_VELOCITY_CONTROL,
    TIMER_GPIO,
    TIMER_COUNT,
  };

  enum class SabacanResetState
  {
    Idle,
    Requested,
    Sending,
    WaitingPostInitializeDelay,
  };

  /**
   * @brief 足回り関連のパラメータを読み込む。
   */
  void load_drive_configuration()
  {
    drive_mode_ = parse_drive_mode(this->declare_parameter<std::string>("drive_mode", "mecanum"));
  }

  /**
   * @brief 安全系の時系列制御に関するパラメータを読み込む。
   */
  void load_safety_configuration()
  {
    sabacan_reset_send_interval_sec_ =
      this->declare_parameter<double>("sabacan_reset_send_interval_sec", 1.0);
    post_reset_initialize_delay_sec_ =
      this->declare_parameter<double>("post_reset_initialize_delay_sec", 0.2);
  }

  /**
   * @brief パラメータ更新を受け取り、drive mode などの動作設定を切り替える。
   * @param parameters 更新要求されたパラメータ群
   * @return rcl_interfaces::msg::SetParametersResult 更新可否
   */
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();

      if (name == "drive_mode") {
        try {
          drive_mode_ = parse_drive_mode(parameter.as_string());
          RCLCPP_INFO(
            this->get_logger(), "Updated parameter: drive_mode = %s", drive_mode_name(drive_mode_));
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
      } else if (name == "sabacan_reset_send_interval_sec") {
        const double value = parameter.as_double();
        if (value < 0.0) {
          result.successful = false;
          result.reason = "sabacan_reset_send_interval_sec must be >= 0.0";
          return result;
        }
        sabacan_reset_send_interval_sec_ = value;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: sabacan_reset_send_interval_sec = %.3f", value);
      } else if (name == "post_reset_initialize_delay_sec") {
        const double value = parameter.as_double();
        if (value < 0.0) {
          result.successful = false;
          result.reason = "post_reset_initialize_delay_sec must be >= 0.0";
          return result;
        }
        post_reset_initialize_delay_sec_ = value;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: post_reset_initialize_delay_sec = %.3f", value);
      }
    }

    return result;
  }

  /**
   * @brief Sabacan GPIO 指令用 publisher 群をボードごとに生成する。
   */
  void initialize_sabacan_gpio_publishers()
  {
    sabacan_gpio_ref_int_publishers_.resize(kSabacanBoardSlotCount);
    sabacan_gpio_ref_float_publishers_.resize(kSabacanBoardSlotCount);
    for (std::size_t i = 0; i < sabacan_gpio_ref_int_publishers_.size(); ++i) {
      sabacan_gpio_ref_int_publishers_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefInt>(
          "/sabacan_gpio_ref_int" + std::to_string(i), 10);
      sabacan_gpio_ref_float_publishers_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefFloat>(
          "/sabacan_gpio_ref_float" + std::to_string(i), 10);
    }
  }

  /**
   * @brief Sabacan の robomas / GPIO status 購読をボードごとに生成する。
   */
  void initialize_sabacan_status_subscriptions()
  {
    sabacan_robomas_status_subscriptions_.clear();
    sabacan_robomas_status_subscriptions_.reserve(kSabacanRobomasNumber);
    for (int board_id = 1; board_id <= kSabacanRobomasNumber; ++board_id) {
      sabacan_robomas_status_subscriptions_.push_back(
        this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
          "/sabacan_robomas_status" + std::to_string(board_id), 10,
          [this, board_id](sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg) {
            this->sabacan_robomas_status_callback(board_id, msg);
          }));
    }

    sabacan_robstride_status_subscriptions_.clear();
    sabacan_robstride_status_subscriptions_.reserve(kSabacanRobstrideNumber);
    for (int board_id = 1; board_id <= kSabacanRobstrideNumber; ++board_id) {
      sabacan_robstride_status_subscriptions_.push_back(
        this->create_subscription<sabacan_msgs::msg::SabacanRobstrideStatus>(
          "/sabacan_robstride_status" + std::to_string(board_id), 10,
          [this, board_id](sabacan_msgs::msg::SabacanRobstrideStatus::SharedPtr msg) {
            this->sabacan_robstride_status_callback(board_id, msg);
          }));
    }

    sabacan_gpio_status_subscriptions_.clear();
    sabacan_gpio_status_subscriptions_.reserve(kSabacanGpioNumber);
    for (int board_id = 1; board_id <= kSabacanGpioNumber; ++board_id) {
      sabacan_gpio_status_subscriptions_.push_back(
        this->create_subscription<sabacan_msgs::msg::SabacanGPIOStatus>(
          "/sabacan_gpio_status" + std::to_string(board_id), 10,
          [this, board_id](sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg) {
            this->sabacan_gpio_status_callback(board_id, msg);
          }));
    }
  }

  /**
   * @brief Robomas のゲイン更新 service client をボードごとに生成する。
   */
  void initialize_sabacan_service_clients()
  {
    set_robomas_gains_clients_.resize(kSabacanBoardSlotCount);
    for (std::size_t i = 0; i < set_robomas_gains_clients_.size(); ++i) {
      set_robomas_gains_clients_[i] = this->create_client<sabacan_msgs::srv::SetRobomasGains>(
        "/set_robomas_gains_id" + std::to_string(i));
    }

    sabacan_reset_service_entries_.clear();
    sabacan_reset_service_entries_.reserve(kSabacanResetServiceNumber);

    auto add_reset_service = [this](const std::string & service_name) {
      sabacan_reset_service_entries_.push_back(
        SabacanResetServiceEntry{
          service_name, this->create_client<sabacan_msgs::srv::SabacanReset>(service_name)});
    };

    add_reset_service("/sabacan_power_reset");
    for (int board_id = 1; board_id <= kSabacanRobomasNumber; ++board_id) {
      add_reset_service("/sabacan_robomas_reset_id" + std::to_string(board_id));
    }
    for (int board_id = 1; board_id <= kSabacanGpioNumber; ++board_id) {
      add_reset_service("/sabacan_gpio_reset_id" + std::to_string(board_id));
    }
    add_reset_service("/sabacan_led_reset");
  }

  /**
   * @brief 非常停止監視と初期化信号の subscription を生成する。
   */
  void initialize_safety_interfaces()
  {
    sabacan_power_status_subscription_ =
      this->create_subscription<sabacan_msgs::msg::SabacanPowerStatus>(
        "/sabacan_power_status0", 10,
        std::bind(&MachineManageNode::sabacan_power_status_callback, this, std::placeholders::_1));
    sabacan_power_ref_publisher_ =
      this->create_publisher<sabacan_msgs::msg::SabacanPowerRef>("/sabacan_power_ref0", 10);
    sabacan_led_ref_publisher_ =
      this->create_publisher<sabacan_msgs::msg::SabacanLEDRef>("/sabacan_led_ref1", 10);
    initialize_done_publisher_ =
      this->create_publisher<std_msgs::msg::Empty>("/r1_machine_initialize_done", 10);
    initialize_signal_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/r1_machine_initialize", 10,
      std::bind(&MachineManageNode::initialize_signal_callback, this, std::placeholders::_1));
    sabacan_reset_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MachineManageNode::sabacan_reset_update, this));
  }

  /**
   * @brief 足回りの publisher / subscription をまとめて初期化する。
   */
  void initialize_drive_interfaces()
  {
    initialize_mecanum_channels();
    initialize_swerve_channels();
    swerve_drive_initialize_publisher_ =
      this->create_publisher<std_msgs::msg::Empty>("/swerve_drive_initialize", 10);
    chassis_velocity_control_initialize_publisher_ =
      this->create_publisher<std_msgs::msg::Empty>("/chassis_velocity_control_initialize", 10);

    mecanum_wheel_speeds_ref_subscription_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_ref", 10,
      std::bind(
        &MachineManageNode::mecanum_wheel_speeds_ref_callback, this, std::placeholders::_1));
    swerve_drive_ref_subscription_ = this->create_subscription<r1_msgs::msg::SwerveDrive>(
      "/swerve_drive_ref", 10,
      std::bind(&MachineManageNode::swerve_drive_ref_callback, this, std::placeholders::_1));

    mecanum_wheel_speeds_feedback_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_feedback", 10);
    odometry_encoder_publisher_ =
      this->create_publisher<r1_msgs::msg::OdometryEncoder>("/odometry_encoder", 10);
  }

  /**
   * @brief 既存の機構群の publisher / subscription をまとめて初期化する。
   */
  void initialize_machine_interfaces()
  {
    initialize_linear_motion_channels();
    initialize_angle_motion_channels();
    initialize_velocity_control_channels();

    initialize_gpio_float_output_channels(gpio_float_output_channels_);
    initialize_gpio_int_output_channels(gpio_int_output_channels_);
    initialize_gpio_input_channels(gpio_input_channels_);
  }

  /**
   * @brief メカナム足回りの publisher / debug publisher を初期化する。
   */
  void initialize_mecanum_channels()
  {
    for (auto & channel : mecanum_channels_) {
      channel.ref_publisher = create_single_ref_publisher(channel.board_info);
      channel.debug_publisher =
        this->create_publisher<r1_msgs::msg::Motor>(channel.debug_topic, 10);
    }
  }

  /**
   * @brief 独ステ足回りの publisher / subscription / debug publisher を初期化する。
   */
  void initialize_swerve_channels()
  {
    initialize_drive_motor_channels(swerve_wheel_channels_);
    initialize_drive_motor_channels(swerve_steer_channels_);
  }

  /**
   * @brief 独ステの軸ごとのチャネル群を初期化する。
   * @param channels 初期化対象のチャネル配列
   */
  void initialize_drive_motor_channels(std::vector<DriveMotorChannel> & channels)
  {
    for (auto & channel : channels) {
      channel.ref_publisher = create_single_ref_publisher(channel.board_info);
      channel.status_publisher =
        this->create_publisher<r1_msgs::msg::Motor>(channel.status_topic, 10);
      channel.debug_publisher =
        this->create_publisher<r1_msgs::msg::Motor>(channel.debug_topic, 10);

      auto * channel_ptr = &channel;
      channel.ref_subscription = this->create_subscription<r1_msgs::msg::MotorRef>(
        channel.ref_topic, 10, [this, channel_ptr](const r1_msgs::msg::MotorRef::SharedPtr msg) {
          if (!is_channel_enabled(channel_ptr->availability)) {
            return;
          }
          publish_motor_ref(
            channel_ptr->board_info, channel_ptr->ref_publisher, channel_ptr->controller_type,
            *msg);
        });
    }
  }

  /**
   * @brief LinearMotionChannel 系の購読と publisher を初期化する。
   */
  void initialize_linear_motion_channels()
  {
    initialize_motor_channels(linear_motion_channels_, []() { return true; });
  }

  /**
   * @brief AngleMotionChannel 系の購読と publisher を初期化する。
   */
  void initialize_angle_motion_channels()
  {
    initialize_motor_channels(angle_motion_channels_, []() { return true; });
  }

  /**
   * @brief 速度制御系チャネルの購読と publisher を初期化する。
   */
  void initialize_velocity_control_channels()
  {
    initialize_motor_channels(velocity_control_channels_, []() { return true; });
  }

  /**
   * @brief 同種のモータチャネル群を初期化する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @tparam EnabledFn そのチャネル群が現在有効かを返す関数型
   * @param channels 初期化対象のチャネル配列
   * @param is_enabled 現在の drive mode などに基づく有効判定
   */
  template <typename StatusMsgT, typename EnabledFn>
  void initialize_motor_channels(
    std::vector<MotorCommandChannel<StatusMsgT>> & channels, EnabledFn is_enabled)
  {
    for (auto & channel : channels) {
      initialize_motor_channel(channel, is_enabled);
    }
  }

  /**
   * @brief 単一のモータチャネルを初期化する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @tparam EnabledFn そのチャネルが現在有効かを返す関数型
   * @param channel 初期化対象のチャネル
   * @param is_enabled 現在の drive mode などに基づく有効判定
   */
  template <typename StatusMsgT, typename EnabledFn>
  void initialize_motor_channel(MotorCommandChannel<StatusMsgT> & channel, EnabledFn is_enabled)
  {
    if (channel.controller_type == MotorControllerType::Robstride) {
      channel.robstride_ref_publisher = create_robstride_ref_publisher(channel.board_info);
    } else {
      channel.ref_publisher = create_single_ref_publisher(channel.board_info);
    }
    channel.status_publisher = this->create_publisher<StatusMsgT>(channel.status_topic, 10);
    channel.debug_publisher = this->create_publisher<r1_msgs::msg::Motor>(channel.debug_topic, 10);
    if (!channel.initialize_topic.empty()) {
      channel.initialize_publisher =
        this->create_publisher<std_msgs::msg::Empty>(channel.initialize_topic, 10);
    }

    auto * channel_ptr = &channel;
    channel.ref_subscription = this->create_subscription<r1_msgs::msg::MotorRef>(
      channel.ref_topic, 10,
      [this, channel_ptr, is_enabled](const r1_msgs::msg::MotorRef::SharedPtr msg) {
        if (!is_enabled()) {
          return;
        }
        publish_motor_ref(*channel_ptr, *msg);
      });
    if (
      channel.controller_type == MotorControllerType::Robomas &&
      !channel.torque_limit_topic.empty()) {
      rclcpp::QoS torque_limit_qos(1);
      torque_limit_qos.reliable();
      torque_limit_qos.transient_local();
      channel.torque_limit_subscription = this->create_subscription<std_msgs::msg::Float64>(
        channel.torque_limit_topic, torque_limit_qos,
        [this, channel_ptr](const std_msgs::msg::Float64::SharedPtr msg) {
          apply_robomas_torque_limit(channel_ptr->board_info, msg->data);
        });
    }
  }

  /**
   * @brief Robomas モータの torque limit を service 経由で更新する。
   * @param board_info 対象モータの board_id / motor_number
   * @param torque_limit 反映したい torque limit [Nm]
   */
  void apply_robomas_torque_limit(const BoardInfo & board_info, double torque_limit)
  {
    if (board_info.board_id < 0 || board_info.number < 0) {
      return;
    }
    if (board_info.board_id >= static_cast<int>(set_robomas_gains_clients_.size())) {
      RCLCPP_WARN(
        this->get_logger(), "invalid board_id=%d for torque limit update", board_info.board_id);
      return;
    }

    const auto & client = set_robomas_gains_clients_[board_info.board_id];
    if (!client) {
      return;
    }
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(
        this->get_logger(), "set_robomas_gains is not ready for board_id=%d", board_info.board_id);
      return;
    }

    auto request = std::make_shared<sabacan_msgs::srv::SetRobomasGains::Request>();
    request->motor_number = static_cast<uint8_t>(board_info.number);
    request->torque_lim = static_cast<float>(torque_limit);
    request->set_torque_limit = true;
    client->async_send_request(request);
  }

  /**
   * @brief 機構 node の initialize topic へ空メッセージをまとめて publish する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channels publish 対象のチャネル配列
   */
  template <typename StatusMsgT>
  void publish_initialize_channels(const std::vector<MotorCommandChannel<StatusMsgT>> & channels)
  {
    std_msgs::msg::Empty msg;
    for (const auto & channel : channels) {
      if (!channel.initialize_publisher) {
        continue;
      }
      channel.initialize_publisher->publish(msg);
    }
  }

  /**
   * @brief motion node 群と swerve drive node へ initialize をまとめて publish する。
   */
  void publish_all_initialize_signals()
  {
    if (swerve_drive_initialize_publisher_) {
      std_msgs::msg::Empty msg;
      swerve_drive_initialize_publisher_->publish(msg);
    }
    if (chassis_velocity_control_initialize_publisher_) {
      std_msgs::msg::Empty msg;
      chassis_velocity_control_initialize_publisher_->publish(msg);
    }
    publish_initialize_channels(linear_motion_channels_);
    publish_initialize_channels(angle_motion_channels_);
  }

  void clear_all_sabacan_leds_once()
  {
    if (!sabacan_led_ref_publisher_) {
      return;
    }

    for (uint8_t pin_number = 0; pin_number < 3; ++pin_number) {
      sabacan_msgs::msg::SabacanLEDRef msg;
      msg.pin_number = pin_number;
      msg.start = 0;
      msg.length = 255;
      msg.r = 0;
      msg.g = 0;
      msg.b = 0;
      sabacan_led_ref_publisher_->publish(msg);
    }

    RCLCPP_INFO(this->get_logger(), "Cleared sabacan LED pins 0-2 after initialize sequence");
  }

  void publish_initialize_done()
  {
    if (!initialize_done_publisher_) {
      return;
    }

    std_msgs::msg::Empty msg;
    initialize_done_publisher_->publish(msg);
  }

  /**
   * @brief PWM 出力用 GPIO チャネル群の subscription を生成する。
   * @param channels 初期化対象のチャネル配列
   */
  void initialize_gpio_float_output_channels(std::vector<GpioFloatOutputChannel> & channels)
  {
    for (auto & channel : channels) {
      auto * channel_ptr = &channel;
      channel.ref_subscription = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
        channel.ref_topic, 10, [this, channel_ptr](const r1_msgs::msg::GpioPwmRef::SharedPtr msg) {
          if (!is_channel_enabled(channel_ptr->availability)) {
            return;
          }
          publish_gpio_float_ref(channel_ptr->board_info, msg->ref);
        });
    }
  }

  /**
   * @brief サーボ出力用 GPIO チャネル群の subscription を生成する。
   * @param channels 初期化対象のチャネル配列
   */
  void initialize_gpio_int_output_channels(std::vector<GpioIntOutputChannel> & channels)
  {
    for (auto & channel : channels) {
      auto * channel_ptr = &channel;
      channel.ref_subscription = this->create_subscription<r1_msgs::msg::GpioServoRef>(
        channel.ref_topic, 10,
        [this, channel_ptr](const r1_msgs::msg::GpioServoRef::SharedPtr msg) {
          if (!is_channel_enabled(channel_ptr->availability)) {
            return;
          }
          publish_gpio_int_ref(channel_ptr->board_info, msg->ref);
        });
    }
  }

  /**
   * @brief GPIO 入力チャネル群の publisher を生成する。
   * @param channels 初期化対象のチャネル配列
   */
  void initialize_gpio_input_channels(std::vector<GpioInputChannel> & channels)
  {
    for (auto & channel : channels) {
      channel.status_publisher =
        this->create_publisher<r1_msgs::msg::GpioInput>(channel.status_topic, 10);
      channel.debug_publisher =
        this->create_publisher<r1_msgs::msg::GpioInput>(channel.debug_topic, 10);
    }
  }

  /**
   * @brief 足回りと役割別 status publish 用 timer をパラメータから生成する。
   */
  void initialize_publish_timers()
  {
    publish_timers_[TIMER_CHASSIS] = create_publish_timer_from_parameter(
      "chassis_timer_rate", 100.0, std::bind(&MachineManageNode::publish_mecanum_feedback, this));
    publish_timers_[TIMER_ODOMETRY] = create_publish_timer_from_parameter(
      "odometry_encoder_timer_rate", 100.0,
      std::bind(&MachineManageNode::publish_odometry_encoder, this));
    publish_timers_[TIMER_LINEAR_MOTION] = create_publish_timer_from_parameter(
      "linear_motion_timer_rate", 100.0,
      std::bind(&MachineManageNode::publish_linear_motion_status, this));
    publish_timers_[TIMER_ANGLE_MOTION] = create_publish_timer_from_parameter(
      "angle_motion_timer_rate", 100.0,
      std::bind(&MachineManageNode::publish_angle_motion_status, this));
    publish_timers_[TIMER_VELOCITY_CONTROL] = create_publish_timer_from_parameter(
      "velocity_control_timer_rate", 100.0,
      std::bind(&MachineManageNode::publish_velocity_control_status, this));
    publish_timers_[TIMER_GPIO] = create_publish_timer_from_parameter(
      "gpio_timer_rate", 100.0, std::bind(&MachineManageNode::publish_gpio_status, this));
  }

  /**
   * @brief パラメータから timer 周波数を読み取り、publish timer を生成する。
   * @param parameter_name 周波数パラメータ名
   * @param default_rate パラメータ未指定時の既定周波数
   * @param callback timer 実行時の callback
   * @return 生成した timer。周波数が 0 以下の場合は nullptr
   */
  rclcpp::TimerBase::SharedPtr create_publish_timer_from_parameter(
    const std::string & parameter_name, double default_rate, const std::function<void()> & callback)
  {
    this->declare_parameter<double>(parameter_name, default_rate);
    return create_publish_timer(this->get_parameter(parameter_name).as_double(), callback);
  }

  /**
   * @brief 指定周波数で実行する wall timer を生成する。
   * @param timer_rate 実行周波数 [Hz]
   * @param callback timer 実行時の callback
   * @return 生成した timer。周波数が 0 以下の場合は nullptr
   */
  rclcpp::TimerBase::SharedPtr create_publish_timer(
    double timer_rate, const std::function<void()> & callback)
  {
    if (timer_rate <= 0.0) {
      return nullptr;
    }
    return this->create_wall_timer(std::chrono::duration<double>(1.0 / timer_rate), callback);
  }

  /**
   * @brief ボード情報に対応する Sabacan 単体制御 publisher を生成する。
   * @param board_info ボード ID とモータ番号
   * @return 生成した publisher
   */
  SingleRefPublisher create_single_ref_publisher(const BoardInfo & board_info)
  {
    return this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
      "/sabacan_robomas_ref" + std::to_string(board_info.board_id) + "/motor" +
        std::to_string(board_info.number),
      10);
  }

  /**
   * @brief ボード情報に対応する Robstride 指令 publisher を生成する。
   * @param board_info ボード ID
   * @return 生成した publisher
   */
  RobstrideRefPublisher create_robstride_ref_publisher(const BoardInfo & board_info)
  {
    return this->create_publisher<sabacan_msgs::msg::SabacanRobstrideRef>(
      "/sabacan_robstride_ref" + std::to_string(board_info.board_id), 10);
  }

  /**
   * @brief 現在の drive mode でチャネルが有効かを返す。
   * @param availability チャネルの有効範囲
   * @return true 現在の mode で有効
   * @return false 現在の mode では無効
   */
  bool is_channel_enabled(ChannelAvailability availability) const
  {
    return is_channel_available(availability, drive_mode_);
  }

  /**
   * @brief Sabacan のモータステータスを r1_msgs::msg::Motor に変換する。
   * @param msg 変換元メッセージ
   * @return r1_msgs::msg::Motor 変換後のモータステータス
   */
  static r1_msgs::msg::Motor to_motor_status(const sabacan_msgs::msg::SabacanRobomasStatus & msg)
  {
    r1_msgs::msg::Motor motor_status;
    motor_status.motor_type = msg.motor_type;
    motor_status.control_type = msg.control_type;
    motor_status.motor_state = msg.motor_state;
    motor_status.torque = msg.torque;
    motor_status.speed = msg.speed;
    motor_status.pos = msg.pos;
    motor_status.abs_pos = msg.abs_pos;
    motor_status.abs_speed = msg.abs_speed;
    motor_status.abs_turn_cnt = msg.abs_turn_cnt;
    motor_status.vesc_voltage = msg.vesc_voltage;
    motor_status.vesc_current = msg.vesc_current;
    motor_status.vesc_speed = msg.vesc_speed;
    return motor_status;
  }

  /**
   * @brief Robstride のモータステータスを r1_msgs::msg::Motor に変換する。
   * @param msg 変換元メッセージ
   * @return r1_msgs::msg::Motor 変換後のモータステータス
   */
  static r1_msgs::msg::Motor to_motor_status(const sabacan_msgs::msg::SabacanRobstrideStatus & msg)
  {
    r1_msgs::msg::Motor motor_status;
    motor_status.motor_type = kRobstrideMotorType;
    motor_status.control_type = msg.control_type;
    motor_status.motor_state = msg.motor_mode_status == "RUN";
    motor_status.torque = msg.torque;
    motor_status.speed = msg.speed;
    motor_status.pos = msg.pos;
    return motor_status;
  }

  /**
   * @brief LinearMotion 用の状態値を Sabacan ステータスから更新する。
   * @param value 更新先
   * @param msg 受信した Sabacan ステータス
   * @param unused Motor 型とのインターフェース統一用引数
   */
  static void update_status_value(
    r1_msgs::msg::LinearMotion & value, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor &)
  {
    value.torque = msg.torque;
    value.speed = msg.speed;
    value.pos = msg.pos;
  }

  /**
   * @brief LinearMotion 用の状態値を Robstride ステータスから更新する。
   * @param value 更新先
   * @param msg 受信した Robstride ステータス
   * @param unused Motor 型とのインターフェース統一用引数
   */
  static void update_status_value(
    r1_msgs::msg::LinearMotion & value, const sabacan_msgs::msg::SabacanRobstrideStatus & msg,
    const r1_msgs::msg::Motor &)
  {
    value.torque = msg.torque;
    value.speed = msg.speed;
    value.pos = msg.pos;
  }

  /**
   * @brief AngleMotion 用の状態値を Sabacan ステータスから更新する。
   * @param value 更新先
   * @param msg 受信した Sabacan ステータス
   * @param unused Motor 型とのインターフェース統一用引数
   */
  static void update_status_value(
    r1_msgs::msg::AngleMotion & value, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor &)
  {
    value.torque = msg.torque;
    value.speed = msg.speed;
    value.pos = msg.pos;
  }

  /**
   * @brief AngleMotion 用の状態値を Robstride ステータスから更新する。
   * @param value 更新先
   * @param msg 受信した Robstride ステータス
   * @param unused Motor 型とのインターフェース統一用引数
   */
  static void update_status_value(
    r1_msgs::msg::AngleMotion & value, const sabacan_msgs::msg::SabacanRobstrideStatus & msg,
    const r1_msgs::msg::Motor &)
  {
    value.torque = msg.torque;
    value.speed = msg.speed;
    value.pos = msg.pos;
  }

  /**
   * @brief Motor 用の状態値を更新する。
   * @param value 更新先
   * @param unused Sabacan ステータス。インターフェース統一用
   * @param motor_status 変換済みモータステータス
   */
  static void update_status_value(
    r1_msgs::msg::Motor & value, const sabacan_msgs::msg::SabacanRobomasStatus &,
    const r1_msgs::msg::Motor & motor_status)
  {
    value = motor_status;
  }

  /**
   * @brief Motor 用の状態値を Robstride ステータスから更新する。
   * @param value 更新先
   * @param unused Robstride ステータス。インターフェース統一用
   * @param motor_status 変換済みモータステータス
   */
  static void update_status_value(
    r1_msgs::msg::Motor & value, const sabacan_msgs::msg::SabacanRobstrideStatus &,
    const r1_msgs::msg::Motor & motor_status)
  {
    value = motor_status;
  }

  /**
   * @brief デバッグ用のモータステータスを publish する。
   * @param publisher 出力先 publisher
   * @param status publish するステータス
   */
  static void publish_debug_motor(
    const PublisherPtr<r1_msgs::msg::Motor> & publisher, const r1_msgs::msg::Motor & status)
  {
    if (publisher) {
      publisher->publish(status);
    }
  }

  /**
   * @brief 現在の安全状態を考慮した Sabacan 単体制御メッセージを生成する。
   * @param board_info 出力先モータのボード情報
   * @param control_type 通常時に使いたい制御モード
   * @param ref 通常時に使いたい指令値
   * @return sabacan_single_control_msgs::msg::SabacanRobomasSingleRef 実際に publish する単軸指令
   */
  sabacan_single_control_msgs::msg::SabacanRobomasSingleRef make_single_ref_message(
    const BoardInfo & board_info, MotorControllerType controller_type,
    const std::string & control_type, double ref) const
  {
    sabacan_single_control_msgs::msg::SabacanRobomasSingleRef ref_msg;
    if (should_force_open_loop()) {
      ref_msg.control_type = open_loop_control_type_for_motor(board_info, controller_type);
      ref_msg.ref = 0.0f;
      return ref_msg;
    }

    ref_msg.control_type = normalize_single_ref_control_type(controller_type, control_type);
    ref_msg.ref = static_cast<float>(ref);
    return ref_msg;
  }

  /**
   * @brief 現在の安全状態を考慮した Robstride 指令メッセージを生成する。
   * @param board_info 出力先モータのボード情報
   * @param control_type 通常時に使いたい制御モード
   * @param ref 通常時に使いたい指令値
   * @param out 生成した指令メッセージ
   * @return true 変換に成功
   * @return false control_type が Robstride 非対応
   */
  bool make_robstride_ref_message(
    const BoardInfo & board_info, const std::string & control_type, double ref,
    sabacan_msgs::msg::SabacanRobstrideRef & out) const
  {
    const std::string normalized =
      should_force_open_loop()
        ? open_loop_control_type_for_motor(board_info, MotorControllerType::Robstride)
        : normalize_control_type(control_type);

    out.control_type = normalized == "POSITION" ? kRobstridePositionControlType : normalized;
    if (should_force_open_loop()) {
      ref = 0.0;
    }

    if (out.control_type == "CURRENT") {
      out.current_ref = static_cast<float>(ref);
      return true;
    }
    if (out.control_type == "VELOCITY") {
      out.velocity_ref = static_cast<float>(ref);
      return true;
    }
    if (out.control_type == "PP") {
      out.pp_angle_ref = static_cast<float>(ref);
      return true;
    }
    if (out.control_type == "CSP") {
      out.csp_angle_ref = static_cast<float>(ref);
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(), "unsupported robstride control_type '%s' on board_id=%d",
      control_type.c_str(), board_info.board_id);
    return false;
  }

  /**
   * @brief MotorRef を Sabacan 単体制御メッセージへ変換して publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   * @param msg 入力された MotorRef
   */
  void publish_motor_ref(
    const BoardInfo & board_info, const SingleRefPublisher & publisher,
    MotorControllerType controller_type, const r1_msgs::msg::MotorRef & msg) const
  {
    if (!publisher) {
      return;
    }
    publisher->publish(
      make_single_ref_message(board_info, controller_type, msg.control_type, msg.ref));
  }

  /**
   * @brief MotorRef を Robstride 指令メッセージへ変換して publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   * @param msg 入力された MotorRef
   */
  void publish_motor_ref(
    const BoardInfo & board_info, const RobstrideRefPublisher & publisher,
    const r1_msgs::msg::MotorRef & msg) const
  {
    if (!publisher) {
      return;
    }

    sabacan_msgs::msg::SabacanRobstrideRef ref_msg;
    if (!make_robstride_ref_message(board_info, msg.control_type, msg.ref, ref_msg)) {
      return;
    }
    publisher->publish(ref_msg);
  }

  /**
   * @brief channel の backend に応じて MotorRef を Sabacan 指令へ変換して publish する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channel 出力先 channel
   * @param msg 入力された MotorRef
   */
  template <typename StatusMsgT>
  void publish_motor_ref(
    const MotorCommandChannel<StatusMsgT> & channel, const r1_msgs::msg::MotorRef & msg) const
  {
    if (
      channel.controller_type == MotorControllerType::Robomas ||
      channel.controller_type == MotorControllerType::Vesc) {
      publish_motor_ref(channel.board_info, channel.ref_publisher, channel.controller_type, msg);
      return;
    }
    publish_motor_ref(channel.board_info, channel.robstride_ref_publisher, msg);
  }

  /**
   * @brief 制御モードと指令値を指定して Sabacan 単体制御メッセージを publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   * @param control_type 制御モード
   * @param ref 指令値
   */
  void publish_motor_ref(
    const BoardInfo & board_info, const SingleRefPublisher & publisher,
    MotorControllerType controller_type, const std::string & control_type, double ref) const
  {
    if (!publisher) {
      return;
    }
    publisher->publish(make_single_ref_message(board_info, controller_type, control_type, ref));
  }

  /**
   * @brief 制御モードと指令値を指定して Robstride 指令メッセージを publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   * @param control_type 制御モード
   * @param ref 指令値
   */
  void publish_motor_ref(
    const BoardInfo & board_info, const RobstrideRefPublisher & publisher,
    const std::string & control_type, double ref) const
  {
    if (!publisher) {
      return;
    }

    sabacan_msgs::msg::SabacanRobstrideRef ref_msg;
    if (!make_robstride_ref_message(board_info, control_type, ref, ref_msg)) {
      return;
    }
    publisher->publish(ref_msg);
  }

  /**
   * @brief 非常停止ラッチ中で MotorRef をオープンループへ強制すべきかを返す。
   * @return true 非常停止中、または初期化待ち
   * @return false 通常制御を通してよい
   */
  bool should_force_open_loop() const
  {
    return emergency_feedback_active_ || emergency_reinit_required_ ||
           sabacan_reset_state_ != SabacanResetState::Idle;
  }

  /**
   * @brief Sabacan reset シーケンスを開始する。
   * @param request_name 起動契機の topic / 操作名
   */
  void start_sabacan_reset_sequence(const char * request_name)
  {
    if (sabacan_reset_state_ != SabacanResetState::Idle) {
      RCLCPP_WARN(
        this->get_logger(), "ignored %s because sabacan reset is already running", request_name);
      return;
    }

    sabacan_reset_state_ = SabacanResetState::Requested;
    sabacan_reset_step_ = 0;
    sabacan_reset_last_send_valid_ = false;
    post_reset_initialize_sent_valid_ = false;
    publish_open_loop_stop_to_enabled_motors();
    publish_all_initialize_signals();
    RCLCPP_INFO(
      this->get_logger(),
      "received %s. forwarded pre-reset initialize under open-loop stop and starting sabacan "
      "reset sequence",
      request_name);
  }

  /**
   * @brief reset 完了後に post-reset initialize を送信し、解除待ち状態へ移る。
   */
  void finish_initialize_sequence_after_reset()
  {
    if (emergency_feedback_active_) {
      sabacan_reset_state_ = SabacanResetState::Idle;
      post_reset_initialize_sent_valid_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "sabacan reset completed while emergency is active. keep open-loop stop and skip "
        "initialize forwarding");
      return;
    }

    publish_all_initialize_signals();
    post_reset_initialize_sent_time_ = this->get_clock()->now();
    post_reset_initialize_sent_valid_ = true;
    sabacan_reset_state_ = SabacanResetState::WaitingPostInitializeDelay;

    if (post_reset_initialize_delay_sec_ <= 0.0) {
      complete_initialize_sequence_after_delay();
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "sabacan reset completed. forwarded post-reset initialize and waiting %.3f sec before "
      "restoring normal motor control routing",
      post_reset_initialize_delay_sec_);
  }

  /**
   * @brief post-reset initialize 後の待機時間が満了したら通常制御へ戻す。
   */
  void complete_initialize_sequence_after_delay()
  {
    if (emergency_feedback_active_) {
      sabacan_reset_state_ = SabacanResetState::Idle;
      post_reset_initialize_sent_valid_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "skipped restoring normal motor control routing because emergency is active");
      return;
    }

    const bool was_reinit_required = emergency_reinit_required_;
    emergency_reinit_required_ = false;
    sabacan_reset_state_ = SabacanResetState::Idle;
    post_reset_initialize_sent_valid_ = false;
    clear_all_sabacan_leds_once();
    // 将来的に原点検出や初期位置指令などの後処理を追加する場合も、この publish を
    // sabacan 初期化完了の同期点として使えるようにしておく。
    publish_initialize_done();

    if (was_reinit_required) {
      RCLCPP_INFO(
        this->get_logger(),
        "post-reset initialize delay completed. restoring normal motor control routing");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "post-reset initialize delay completed");
  }

  /**
   * @brief Sabacan reset service 群へ順番に request を送り、完了後に initialize を転送する。
   */
  void sabacan_reset_update()
  {
    if (sabacan_reset_state_ == SabacanResetState::Idle) {
      return;
    }

    if (sabacan_reset_state_ == SabacanResetState::WaitingPostInitializeDelay) {
      if (emergency_feedback_active_) {
        sabacan_reset_state_ = SabacanResetState::Idle;
        post_reset_initialize_sent_valid_ = false;
        RCLCPP_WARN(
          this->get_logger(),
          "emergency became active during post-reset initialize delay. aborting restore to "
          "normal motor control");
        return;
      }

      if (!post_reset_initialize_sent_valid_) {
        complete_initialize_sequence_after_delay();
        return;
      }

      const auto now = this->get_clock()->now();
      const rclcpp::Duration post_initialize_delay =
        rclcpp::Duration::from_seconds(post_reset_initialize_delay_sec_);
      if ((now - post_reset_initialize_sent_time_) < post_initialize_delay) {
        return;
      }
      complete_initialize_sequence_after_delay();
      return;
    }

    if (sabacan_reset_state_ == SabacanResetState::Requested) {
      sabacan_reset_state_ = SabacanResetState::Sending;
      sabacan_reset_step_ = 0;
      sabacan_reset_last_send_valid_ = false;
    }

    const rclcpp::Duration send_interval =
      rclcpp::Duration::from_seconds(sabacan_reset_send_interval_sec_);
    const auto now = this->get_clock()->now();
    if (sabacan_reset_last_send_valid_ && (now - sabacan_reset_last_send_time_) < send_interval) {
      return;
    }

    if (sabacan_reset_step_ >= sabacan_reset_service_entries_.size()) {
      finish_initialize_sequence_after_reset();
      return;
    }

    const auto & entry = sabacan_reset_service_entries_[sabacan_reset_step_];
    if (!entry.client) {
      RCLCPP_WARN(
        this->get_logger(), "sabacan reset client is null: %s", entry.service_name.c_str());
    } else if (!entry.client->wait_for_service(std::chrono::milliseconds(0))) {
      RCLCPP_WARN(
        this->get_logger(), "sabacan reset service is not ready: %s", entry.service_name.c_str());
    } else {
      auto request = std::make_shared<sabacan_msgs::srv::SabacanReset::Request>();
      entry.client->async_send_request(
        request, [this, service_name = entry.service_name](
                   rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedFuture future) {
          (void)future.get();
          RCLCPP_INFO(this->get_logger(), "sabacan reset sent: %s", service_name.c_str());
        });
    }

    sabacan_reset_last_send_time_ = now;
    sabacan_reset_last_send_valid_ = true;
    ++sabacan_reset_step_;
  }

  /**
   * @brief 指定モータの最新 motor_type をキャッシュする。
   * @param board_info モータ識別子
   * @param motor_type Sabacan status に含まれる motor_type
   */
  void cache_motor_type(const BoardInfo & board_info, const std::string & motor_type)
  {
    const std::string normalized = to_upper_ascii(motor_type);
    for (auto & entry : motor_type_cache_) {
      if (entry.board_info == board_info) {
        entry.motor_type = normalized;
        return;
      }
    }

    motor_type_cache_.push_back(MotorTypeCacheEntry{board_info, normalized});
  }

  /**
   * @brief 非常停止時に使う open-loop control_type をモータ種別から返す。
   * @param board_info 出力先モータのボード情報
   * @return const char * VESC なら CURRENT、それ以外は TORQUE
   */
  const char * open_loop_control_type_for_motor(
    const BoardInfo & board_info, MotorControllerType controller_type) const
  {
    if (
      controller_type == MotorControllerType::Vesc ||
      controller_type == MotorControllerType::Robstride) {
      return "CURRENT";
    }

    for (const auto & entry : motor_type_cache_) {
      if (entry.board_info == board_info) {
        return is_current_control_motor_type(entry.motor_type) ? "CURRENT" : "TORQUE";
      }
    }
    return "TORQUE";
  }

  /**
   * @brief 指定モータへ非常停止用の open-loop 停止指令を publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   */
  void publish_open_loop_stop(
    const BoardInfo & board_info, const SingleRefPublisher & publisher,
    MotorControllerType controller_type) const
  {
    if (!publisher) {
      return;
    }

    sabacan_single_control_msgs::msg::SabacanRobomasSingleRef ref_msg;
    ref_msg.control_type = open_loop_control_type_for_motor(board_info, controller_type);
    ref_msg.ref = 0.0f;
    publisher->publish(ref_msg);
  }

  /**
   * @brief 指定 Robstride モータへ非常停止用の open-loop 停止指令を publish する。
   * @param board_info 出力先モータのボード情報
   * @param publisher 出力先 publisher
   */
  void publish_open_loop_stop(
    const BoardInfo & board_info, const RobstrideRefPublisher & publisher) const
  {
    if (!publisher) {
      return;
    }

    sabacan_msgs::msg::SabacanRobstrideRef ref_msg;
    if (!make_robstride_ref_message(
          board_info, open_loop_control_type_for_motor(board_info, MotorControllerType::Robstride),
          0.0, ref_msg)) {
      return;
    }
    publisher->publish(ref_msg);
  }

  /**
   * @brief チャネル配列に含まれる全モータへ open-loop 停止指令を publish する。
   * @tparam ChannelT board_info / ref_publisher を持つチャネル型
   * @param channels 停止指令を送りたいチャネル配列
   */
  template <typename ChannelT>
  void publish_open_loop_stop_channels(const std::vector<ChannelT> & channels) const
  {
    for (const auto & channel : channels) {
      publish_open_loop_stop(channel.board_info, channel.ref_publisher, channel.controller_type);
    }
  }

  /**
   * @brief 役割別モータチャネル群へ open-loop 停止指令を publish する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channels 停止指令を送りたいチャネル配列
   */
  template <typename StatusMsgT>
  void publish_open_loop_stop_channels(
    const std::vector<MotorCommandChannel<StatusMsgT>> & channels) const
  {
    for (const auto & channel : channels) {
      if (
        channel.controller_type == MotorControllerType::Robomas ||
        channel.controller_type == MotorControllerType::Vesc) {
        publish_open_loop_stop(channel.board_info, channel.ref_publisher, channel.controller_type);
        continue;
      }
      publish_open_loop_stop(channel.board_info, channel.robstride_ref_publisher);
    }
  }

  /**
   * @brief 現在有効な全モータチャネルへ open-loop 停止指令を即時送信する。
   */
  void publish_open_loop_stop_to_enabled_motors() const
  {
    if (drive_mode_ == DriveMode::Mecanum) {
      publish_open_loop_stop_channels(mecanum_channels_);
    } else {
      publish_open_loop_stop_channels(swerve_wheel_channels_);
      publish_open_loop_stop_channels(swerve_steer_channels_);
    }
    publish_open_loop_stop_channels(linear_motion_channels_);
    publish_open_loop_stop_channels(angle_motion_channels_);
    publish_open_loop_stop_channels(velocity_control_channels_);
  }

  /**
   * @brief PWM 値を Sabacan GPIO float 指令として publish する。
   * @param board_info 出力先 GPIO のボード情報
   * @param ref PWM 指令値
   */
  void publish_gpio_float_ref(const BoardInfo & board_info, double ref)
  {
    sabacan_msgs::msg::SabacanGPIORefFloat gpio_msg;
    gpio_msg.pin_number = board_info.number;
    gpio_msg.ref_float = static_cast<float>(ref);
    sabacan_gpio_ref_float_publishers_[board_info.board_id]->publish(gpio_msg);
  }

  /**
   * @brief サーボ値を Sabacan GPIO int 指令として publish する。
   * @param board_info 出力先 GPIO のボード情報
   * @param ref サーボ指令値
   */
  void publish_gpio_int_ref(const BoardInfo & board_info, int ref)
  {
    sabacan_msgs::msg::SabacanGPIORefInt gpio_msg;
    gpio_msg.pin_number = board_info.number;
    gpio_msg.ref_int = ref;
    sabacan_gpio_ref_int_publishers_[board_info.board_id]->publish(gpio_msg);
  }

  /**
   * @brief Sabacan のモータステータスを受け取り、各機構の状態キャッシュへ反映する。
   * @param board_id 受信元ボード ID
   * @param msg 受信した Sabacan モータステータス
   */
  void sabacan_robomas_status_callback(
    int board_id, const sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    const BoardInfo receive{board_id, msg->motor_number};
    cache_motor_type(receive, msg->motor_type);
    const auto motor_status = to_motor_status(*msg);

    update_drive_status(receive, *msg, motor_status);
    update_motor_channels(
      linear_motion_channels_, MotorControllerType::Robomas, receive, *msg, motor_status);
    update_motor_channels(
      angle_motion_channels_, MotorControllerType::Robomas, receive, *msg, motor_status);
    update_motor_channels(
      velocity_control_channels_, MotorControllerType::Robomas, receive, *msg, motor_status);
  }

  /**
   * @brief Robstride のモータステータスを受け取り、各機構の状態キャッシュへ反映する。
   * @param board_id 受信元ボード ID
   * @param msg 受信した Robstride モータステータス
   */
  void sabacan_robstride_status_callback(
    int board_id, const sabacan_msgs::msg::SabacanRobstrideStatus::SharedPtr msg)
  {
    const BoardInfo receive{board_id, -1};
    cache_motor_type(receive, kRobstrideMotorType);
    const auto motor_status = to_motor_status(*msg);

    update_motor_channels(
      linear_motion_channels_, MotorControllerType::Robstride, receive, *msg, motor_status);
    update_motor_channels(
      angle_motion_channels_, MotorControllerType::Robstride, receive, *msg, motor_status);
    update_motor_channels(
      velocity_control_channels_, MotorControllerType::Robstride, receive, *msg, motor_status);
  }

  /**
   * @brief SabacanPowerStatus を受け取り、非常停止ラッチ状態を更新する。
   * @param msg 受信した電源状態メッセージ
   */
  void sabacan_power_status_callback(const sabacan_msgs::msg::SabacanPowerStatus::SharedPtr msg)
  {
    const bool emergency_active = is_emergency_active(*msg);
    if (emergency_feedback_active_ == emergency_active) {
      return;
    }

    emergency_feedback_active_ = emergency_active;
    if (emergency_feedback_active_) {
      emergency_reinit_required_ = true;
      pending_initialize_ = false;
      publish_open_loop_stop_to_enabled_motors();
      RCLCPP_WARN(
        this->get_logger(),
        "sabacan power status entered emergency. forcing motor outputs to open-loop stop");
      return;
    }

    if (pending_initialize_) {
      pending_initialize_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "sabacan power status cleared emergency. auto-triggering pending /r1_machine_initialize");
      start_sabacan_reset_sequence("/r1_machine_initialize");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "sabacan power status cleared emergency. keep open-loop stop until /r1_machine_initialize");
  }

  /**
   * @brief 初期化信号を受け取り、非常停止解除後の open-loop ラッチを解除する。
   * @details まず SabacanPowerRef(is_ems=false) を送信してソフト EMS ラッチをクリアする。
   *          emergency_feedback_active_ がすでに false なら即座にシーケンスを開始する。
   *          まだ true の場合は pending_initialize_ を立てて待機し、
   *          sabacan_power_status_callback で EMS クリアが確認された時点で自動実行する。
   * @param msg 受信した初期化信号
   */
  void initialize_signal_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;

    sabacan_msgs::msg::SabacanPowerRef power_ref;
    power_ref.is_ems = false;
    sabacan_power_ref_publisher_->publish(power_ref);

    if (emergency_feedback_active_) {
      pending_initialize_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "received /r1_machine_initialize while emergency active. "
        "sent SabacanPowerRef(is_ems=false) and waiting for EMS to clear");
      return;
    }
    start_sabacan_reset_sequence("/r1_machine_initialize");
  }

  /**
   * @brief 現在の drive mode に応じて足回りステータスを更新する。
   * @param receive 受信元のボード情報
   * @param msg 受信した Sabacan モータステータス
   * @param motor_status 変換済みモータステータス
   */
  void update_drive_status(
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    update_odometry_feedback(receive, msg);

    if (drive_mode_ == DriveMode::Mecanum) {
      update_mecanum_feedback(receive, msg, motor_status);
      return;
    }

    update_drive_motor_channels(swerve_wheel_channels_, receive, motor_status);
    update_drive_motor_channels(swerve_steer_channels_, receive, motor_status);
  }

  /**
   * @brief メカナム足回りのフィードバックとデバッグ出力を更新する。
   * @param receive 受信元のボード情報
   * @param msg 受信した Sabacan モータステータス
   * @param motor_status 変換済みモータステータス
   */
  void update_mecanum_feedback(
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    for (std::size_t i = 0; i < mecanum_channels_.size(); ++i) {
      auto & channel = mecanum_channels_[i];
      if (!is_channel_enabled(channel.availability) || !(receive == channel.board_info)) {
        continue;
      }

      mecanum_wheel_speeds_feedback_[i] = msg.speed;
      publish_debug_motor(channel.debug_publisher, motor_status);
      break;
    }
  }

  /**
   * @brief オドメトリエンコーダの位置と速度を更新する。
   * @param receive 受信元のボード情報
   * @param msg 受信した Sabacan モータステータス
   */
  void update_odometry_feedback(
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobomasStatus & msg)
  {
    for (std::size_t i = 0; i < odometry_encoder_channels_.size(); ++i) {
      if (!(receive == odometry_encoder_channels_[i])) {
        continue;
      }

      odometry_encoder_pos_values_[i] = msg.abs_pos;
      odometry_encoder_speed_values_[i] = msg.abs_speed;
      break;
    }
  }

  /**
   * @brief 独ステの debug status publisher を board 情報に応じて更新する。
   * @param channels 更新対象のチャネル配列
   * @param receive 受信元のボード情報
   * @param motor_status 変換済みモータステータス
   */
  void update_drive_motor_channels(
    std::vector<DriveMotorChannel> & channels, const BoardInfo & receive,
    const r1_msgs::msg::Motor & motor_status)
  {
    for (auto & channel : channels) {
      if (!is_channel_enabled(channel.availability) || !(receive == channel.board_info)) {
        continue;
      }

      channel.value = motor_status;
      if (channel.status_publisher) {
        channel.status_publisher->publish(channel.value);
      }
      publish_debug_motor(channel.debug_publisher, motor_status);
      break;
    }
  }

  /**
   * @brief モータチャネル群のうち該当する 1 件を更新する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channels 更新対象のチャネル配列
   * @param receive 受信元のボード情報
   * @param msg 受信した Sabacan モータステータス
   * @param motor_status 変換済みのモータステータス
   */
  template <typename StatusMsgT>
  void update_motor_channels(
    std::vector<MotorCommandChannel<StatusMsgT>> & channels, MotorControllerType controller_type,
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    for (auto & channel : channels) {
      if (update_motor_channel(channel, controller_type, receive, msg, motor_status)) {
        break;
      }
    }
  }

  /**
   * @brief モータチャネル群のうち該当する 1 件を Robstride ステータスで更新する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channels 更新対象のチャネル配列
   * @param controller_type 更新対象 backend
   * @param receive 受信元のボード情報
   * @param msg 受信した Robstride モータステータス
   * @param motor_status 変換済みのモータステータス
   */
  template <typename StatusMsgT>
  void update_motor_channels(
    std::vector<MotorCommandChannel<StatusMsgT>> & channels, MotorControllerType controller_type,
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobstrideStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    for (auto & channel : channels) {
      if (update_motor_channel(channel, controller_type, receive, msg, motor_status)) {
        break;
      }
    }
  }

  /**
   * @brief 単一のモータチャネルを更新する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channel 更新対象のチャネル
   * @param receive 受信元のボード情報
   * @param msg 受信した Sabacan モータステータス
   * @param motor_status 変換済みのモータステータス
   * @return true このチャネルが受信データに一致した場合
   * @return false 一致しなかった場合
   */
  template <typename StatusMsgT>
  bool update_motor_channel(
    MotorCommandChannel<StatusMsgT> & channel, MotorControllerType controller_type,
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobomasStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    if (channel.controller_type != controller_type || !(receive == channel.board_info)) {
      return false;
    }

    update_status_value(channel.value, msg, motor_status);
    publish_debug_motor(channel.debug_publisher, motor_status);
    return true;
  }

  /**
   * @brief 単一のモータチャネルを Robstride ステータスで更新する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channel 更新対象のチャネル
   * @param controller_type 更新対象 backend
   * @param receive 受信元のボード情報
   * @param msg 受信した Robstride モータステータス
   * @param motor_status 変換済みのモータステータス
   * @return true このチャネルが受信データに一致した場合
   * @return false 一致しなかった場合
   */
  template <typename StatusMsgT>
  bool update_motor_channel(
    MotorCommandChannel<StatusMsgT> & channel, MotorControllerType controller_type,
    const BoardInfo & receive, const sabacan_msgs::msg::SabacanRobstrideStatus & msg,
    const r1_msgs::msg::Motor & motor_status)
  {
    if (channel.controller_type != controller_type || !(receive == channel.board_info)) {
      return false;
    }

    update_status_value(channel.value, msg, motor_status);
    publish_debug_motor(channel.debug_publisher, motor_status);
    return true;
  }

  /**
   * @brief Sabacan の GPIO 入力を受け取り、状態キャッシュとデバッグ出力を更新する。
   * @param board_id 受信元ボード ID
   * @param msg 受信した Sabacan GPIO ステータス
   */
  void sabacan_gpio_status_callback(
    int board_id, const sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg)
  {
    const BoardInfo receive{board_id, msg->pin_number};
    r1_msgs::msg::GpioInput gpio_status;
    gpio_status.status = msg->input;

    for (auto & channel : gpio_input_channels_) {
      if (!is_channel_enabled(channel.availability) || !(receive == channel.board_info)) {
        continue;
      }

      channel.value = gpio_status;
      if (channel.debug_publisher) {
        channel.debug_publisher->publish(gpio_status);
      }
      break;
    }
  }

  /**
   * @brief メカナムの各輪速度指令を Sabacan 単体指令へ変換して publish する。
   * @param msg メカナム速度指令
   */
  void mecanum_wheel_speeds_ref_callback(const r1_msgs::msg::Mecanum::ConstSharedPtr msg)
  {
    if (drive_mode_ != DriveMode::Mecanum) {
      return;
    }

    const std::vector<double> wheel_speeds = {
      msg->fl_wheel_speed,
      msg->fr_wheel_speed,
      msg->rl_wheel_speed,
      msg->rr_wheel_speed,
    };

    const std::size_t channel_count = mecanum_channels_.size() < wheel_speeds.size()
                                        ? mecanum_channels_.size()
                                        : wheel_speeds.size();
    for (std::size_t i = 0; i < channel_count; ++i) {
      publish_motor_ref(
        mecanum_channels_[i].board_info, mecanum_channels_[i].ref_publisher,
        mecanum_channels_[i].controller_type, "VELOCITY", wheel_speeds[i]);
    }
  }

  /**
   * @brief 独ステの 4 輪指令を wheel / steer の単軸指令へ分解して publish する。
   * @param msg 独ステ 4 輪分の指令
   */
  void swerve_drive_ref_callback(const r1_msgs::msg::SwerveDrive::ConstSharedPtr msg)
  {
    if (drive_mode_ != DriveMode::Swerve) {
      return;
    }

    const std::vector<double> wheel_refs = {
      msg->omega0,
      msg->omega1,
      msg->omega2,
      msg->omega3,
    };
    const std::vector<double> steer_refs = {
      msg->theta0,
      msg->theta1,
      msg->theta2,
      msg->theta3,
    };

    const std::size_t wheel_channel_count = swerve_wheel_channels_.size() < wheel_refs.size()
                                              ? swerve_wheel_channels_.size()
                                              : wheel_refs.size();
    const std::size_t steer_channel_count = swerve_steer_channels_.size() < steer_refs.size()
                                              ? swerve_steer_channels_.size()
                                              : steer_refs.size();
    const std::size_t channel_count =
      wheel_channel_count < steer_channel_count ? wheel_channel_count : steer_channel_count;
    for (std::size_t i = 0; i < channel_count; ++i) {
      publish_motor_ref(
        swerve_wheel_channels_[i].board_info, swerve_wheel_channels_[i].ref_publisher,
        swerve_wheel_channels_[i].controller_type, kSwerveWheelControlType, wheel_refs[i]);
      publish_motor_ref(
        swerve_steer_channels_[i].board_info, swerve_steer_channels_[i].ref_publisher,
        swerve_steer_channels_[i].controller_type, kSwerveSteerControlType, steer_refs[i]);
    }
  }

  /**
   * @brief キャッシュしたメカナム足回りフィードバックをまとめて publish する。
   */
  void publish_mecanum_feedback()
  {
    if (drive_mode_ != DriveMode::Mecanum) {
      return;
    }

    r1_msgs::msg::Mecanum msg_feedback;
    msg_feedback.fl_wheel_speed = mecanum_wheel_speeds_feedback_[FL];
    msg_feedback.fr_wheel_speed = mecanum_wheel_speeds_feedback_[FR];
    msg_feedback.rl_wheel_speed = mecanum_wheel_speeds_feedback_[RL];
    msg_feedback.rr_wheel_speed = mecanum_wheel_speeds_feedback_[RR];
    mecanum_wheel_speeds_feedback_publisher_->publish(msg_feedback);
  }

  /**
   * @brief キャッシュしたオドメトリエンコーダ値を publish する。
   */
  void publish_odometry_encoder()
  {
    r1_msgs::msg::OdometryEncoder odom_msg;
    odom_msg.encoder_pos_x = odometry_encoder_pos_values_[X];
    odom_msg.encoder_pos_y = odometry_encoder_pos_values_[Y];
    odom_msg.encoder_speed_x = odometry_encoder_speed_values_[X];
    odom_msg.encoder_speed_y = odometry_encoder_speed_values_[Y];
    odometry_encoder_publisher_->publish(odom_msg);
  }

  /**
   * @brief チャネル配列の状態値を順番に publish する。
   * @tparam StatusMsgT フィードバックに使うメッセージ型
   * @param channels publish 対象のチャネル配列
   */
  template <typename StatusMsgT>
  void publish_status_channels(const std::vector<MotorCommandChannel<StatusMsgT>> & channels) const
  {
    for (const auto & channel : channels) {
      channel.status_publisher->publish(channel.value);
    }
  }

  /**
   * @brief LinearMotionChannel 系の状態を publish する。
   */
  void publish_linear_motion_status() { publish_status_channels(linear_motion_channels_); }

  /**
   * @brief AngleMotionChannel 系の状態を publish する。
   */
  void publish_angle_motion_status() { publish_status_channels(angle_motion_channels_); }

  /**
   * @brief 速度制御系チャネルの状態を publish する。
   */
  void publish_velocity_control_status() { publish_status_channels(velocity_control_channels_); }

  /**
   * @brief GPIO 入力の状態を publish する。
   */
  void publish_gpio_status()
  {
    for (const auto & channel : gpio_input_channels_) {
      if (!is_channel_enabled(channel.availability)) {
        continue;
      }
      channel.status_publisher->publish(channel.value);
    }
  }

  // Sabacan 側の入出力をボード単位でまとめて保持する。
  std::vector<PublisherPtr<sabacan_msgs::msg::SabacanGPIORefInt>> sabacan_gpio_ref_int_publishers_;
  std::vector<PublisherPtr<sabacan_msgs::msg::SabacanGPIORefFloat>>
    sabacan_gpio_ref_float_publishers_;
  std::vector<SubscriptionPtr<sabacan_msgs::msg::SabacanRobomasStatus>>
    sabacan_robomas_status_subscriptions_;
  std::vector<SubscriptionPtr<sabacan_msgs::msg::SabacanRobstrideStatus>>
    sabacan_robstride_status_subscriptions_;
  std::vector<SubscriptionPtr<sabacan_msgs::msg::SabacanGPIOStatus>>
    sabacan_gpio_status_subscriptions_;
  std::vector<SetRobomasGainsClient::SharedPtr> set_robomas_gains_clients_;
  std::vector<SabacanResetServiceEntry> sabacan_reset_service_entries_;
  SubscriptionPtr<sabacan_msgs::msg::SabacanPowerStatus> sabacan_power_status_subscription_;
  rclcpp::Publisher<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr sabacan_power_ref_publisher_;
  SubscriptionPtr<std_msgs::msg::Empty> initialize_signal_subscription_;
  rclcpp::TimerBase::SharedPtr sabacan_reset_timer_;

  // drive mode と足回りの制御設定。
  DriveMode drive_mode_{DriveMode::Mecanum};
  bool emergency_feedback_active_{false};
  bool emergency_reinit_required_{false};
  bool pending_initialize_{false};
  SabacanResetState sabacan_reset_state_{SabacanResetState::Idle};
  rclcpp::Time sabacan_reset_last_send_time_{0LL, RCL_SYSTEM_TIME};
  bool sabacan_reset_last_send_valid_{false};
  rclcpp::Time post_reset_initialize_sent_time_{0LL, RCL_SYSTEM_TIME};
  bool post_reset_initialize_sent_valid_{false};
  std::size_t sabacan_reset_step_{0};
  double sabacan_reset_send_interval_sec_{1.0};
  double post_reset_initialize_delay_sec_{0.2};
  std::vector<MotorTypeCacheEntry> motor_type_cache_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;

  // 足回りのチャネル定義。
  std::vector<MecanumWheelChannel> mecanum_channels_{
    make_mecanum_wheel_channel({1, 0}, "mecanum_fl", MotorControllerType::Vesc),
    make_mecanum_wheel_channel({1, 1}, "mecanum_fr", MotorControllerType::Vesc),
    make_mecanum_wheel_channel({1, 2}, "mecanum_rl", MotorControllerType::Vesc),
    make_mecanum_wheel_channel({1, 3}, "mecanum_rr", MotorControllerType::Vesc),
  };
  std::vector<BoardInfo> odometry_encoder_channels_{
    BoardInfo{7, 2},
    BoardInfo{7, 3},
  };
  // 独ステの board_id / motor_number は頻繁に変えない前提でソース内に固定する。
  std::vector<DriveMotorChannel> swerve_wheel_channels_{
    make_drive_motor_channel({1, 0}, "swerve_fr_wheel", MotorControllerType::Vesc),
    make_drive_motor_channel({1, 1}, "swerve_fl_wheel", MotorControllerType::Vesc),
    make_drive_motor_channel({1, 2}, "swerve_rl_wheel", MotorControllerType::Vesc),
    make_drive_motor_channel({1, 3}, "swerve_rr_wheel", MotorControllerType::Vesc),
  };
  std::vector<DriveMotorChannel> swerve_steer_channels_{
    make_drive_motor_channel({2, 0}, "swerve_fr_steer"),
    make_drive_motor_channel({2, 1}, "swerve_fl_steer"),
    make_drive_motor_channel({2, 2}, "swerve_rl_steer"),
    make_drive_motor_channel({2, 3}, "swerve_rr_steer"),
  };

  // 足回り以外のモータチャネルは、機構別ではなく役割別の vector でまとめて保持する。
  std::vector<MotorStatusChannel> velocity_control_channels_{
    make_velocity_control_channel({3, 3}, "r2_flift"),
    make_velocity_control_channel({4, 3}, "r2_rlift"),
  };
  std::vector<LinearMotionChannel> linear_motion_channels_{
    make_linear_motion_channel({3, 0}, "kfs_fx"), make_linear_motion_channel({3, 1}, "kfs_fz"),
    make_linear_motion_channel({4, 0}, "kfs_rx"), make_linear_motion_channel({4, 1}, "kfs_rz"),
    make_linear_motion_channel({5, 0}, "spear1"), make_linear_motion_channel({5, 1}, "spear2"),
    // NOTE: 一旦spear3,4は使わない構成
    // make_linear_motion_channel({5, 2}, "spear3"),  make_linear_motion_channel({5, 3}, "spear4"),
    // make_linear_motion_channel({6, 0}, "spear_x"),
    make_linear_motion_channel({5, 2}, "spear_x"), make_linear_motion_channel({6, 1}, "spear_y")};
  std::vector<AngleMotionChannel> angle_motion_channels_{
    make_angle_motion_channel({3, 2}, "kfs_fyaw"), make_angle_motion_channel({4, 2}, "kfs_ryaw"),
    make_angle_motion_channel({6, 2}, "spear_pitch1"),
    make_angle_motion_channel({6, 3}, "spear_pitch2"),
    // spear_rollはrobstride
    make_angle_motion_channel({1, -1}, "spear_roll", MotorControllerType::Robstride)};

  std::vector<GpioFloatOutputChannel> gpio_float_output_channels_{
    make_gpio_float_output_channel({1, 0}, "kfs_front_pump"),
    make_gpio_float_output_channel({1, 1}, "kfs_rear_pump"),
    make_gpio_float_output_channel({1, 2}, "kfs_front_valve"),
    make_gpio_float_output_channel({1, 3}, "kfs_rear_valve"),
    make_gpio_float_output_channel({2, 0}, "spear_u1_valve"),
    make_gpio_float_output_channel({2, 1}, "spear_d1_valve"),
    make_gpio_float_output_channel({2, 2}, "spear_u2_valve"),
    make_gpio_float_output_channel({2, 3}, "spear_d2_valve"),
    // 電磁弁は何個つながるかわからないので一旦ここで止めとく
  };
  std::vector<GpioIntOutputChannel> gpio_int_output_channels_{
    // make_gpio_int_output_channel({3, 0}, "pole_servo1"),
    // make_gpio_int_output_channel({3, 1}, "pole_servo2"),
    // make_gpio_int_output_channel({3, 2}, "pole_servo3"),
    // make_gpio_int_output_channel({3, 3}, "pole_servo4"),
  };
  std::vector<GpioInputChannel> gpio_input_channels_{
    make_gpio_input_channel({1, 7}, "kfs_fz_low_switch"),
    make_gpio_input_channel({1, 8}, "kfs_rz_low_switch"),
  };

  // r1_msgs 側の足回り publisher / subscription。
  SubscriptionPtr<r1_msgs::msg::Mecanum> mecanum_wheel_speeds_ref_subscription_;
  SubscriptionPtr<r1_msgs::msg::SwerveDrive> swerve_drive_ref_subscription_;
  PublisherPtr<r1_msgs::msg::Mecanum> mecanum_wheel_speeds_feedback_publisher_;
  PublisherPtr<r1_msgs::msg::OdometryEncoder> odometry_encoder_publisher_;
  PublisherPtr<std_msgs::msg::Empty> swerve_drive_initialize_publisher_;
  PublisherPtr<std_msgs::msg::Empty> chassis_velocity_control_initialize_publisher_;
  PublisherPtr<std_msgs::msg::Empty> initialize_done_publisher_;
  PublisherPtr<sabacan_msgs::msg::SabacanLEDRef> sabacan_led_ref_publisher_;

  // 一定周期 publish のために保持する最新値キャッシュ。
  std::vector<double> mecanum_wheel_speeds_feedback_{0.0, 0.0, 0.0, 0.0};
  std::vector<double> odometry_encoder_pos_values_{0.0, 0.0};
  std::vector<double> odometry_encoder_speed_values_{0.0, 0.0};

  // publish timer は寿命管理のため配列で保持する。
  std::array<rclcpp::TimerBase::SharedPtr, TIMER_COUNT> publish_timers_{};
};

/**
 * @brief ノードを起動してスピンする。
 * @param argc コマンドライン引数数
 * @param argv コマンドライン引数
 * @return int 終了コード
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MachineManageNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
