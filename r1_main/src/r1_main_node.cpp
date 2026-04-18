/**
 * @file r1_main_node.cpp
 * @author Yamaguchi Yudai
 * @brief R1の状態遷移ノード
 * @version 0.1
 * @date 2025-09-27
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "r1_main/r1_main_node.h"

#include <cctype>
#include <chrono>
#include <optional>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace
{
std::optional<RobotState> parse_robot_control_mode_parameter(std::string_view value)
{
  std::string normalized(value);
  for (auto & c : normalized) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  if (normalized == "manual") {
    return RobotState{
      MainState::READY, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::MANUAL};
  }
  if (normalized == "auto") {
    return RobotState{
      MainState::READY, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::AUTO};
  }

  return std::nullopt;
}

std::string robot_control_mode_parameter_help() { return "Accepted values: manual, auto."; }

OperationMode next_operation_mode(OperationMode mode)
{
  switch (mode) {
    case OperationMode::MODE1_DETECT_ORIGIN:
      return OperationMode::MODE2_POLE;
    case OperationMode::MODE2_POLE:
      return OperationMode::MODE3_SPEAR;
    case OperationMode::MODE3_SPEAR:
      return OperationMode::MODE4_FKFS;
    case OperationMode::MODE4_FKFS:
      return OperationMode::MODE5_RKFS;
    case OperationMode::MODE5_RKFS:
      return OperationMode::MODE6_R2_LIFT;
    case OperationMode::MODE6_R2_LIFT:
      return OperationMode::MODE7_SPEAR_ATTACK;
    case OperationMode::MODE7_SPEAR_ATTACK:
      return OperationMode::MODE8_AUTO_COLLECT_KFS;
    case OperationMode::MODE8_AUTO_COLLECT_KFS:
      return OperationMode::MODE9_AUTO_CHASSIS;
    case OperationMode::MODE9_AUTO_CHASSIS:
    default:
      return OperationMode::MODE1_DETECT_ORIGIN;
  }
}

std::string kfs_auto_collect_status_name(R1MainNode::KfsAutoCollectStatus status)
{
  return std::string(magic_enum::enum_name(status));
}
}  // namespace

/**
 * @brief double 型パラメータを declare して取得する。
 * @param name パラメータ名。
 * @param value 取得結果の格納先。
 * @param default_value declare 時の初期値。
 */
void R1MainNode::declare_and_get_parameter(
  const std::string & name, double & value, double default_value)
{
  this->declare_parameter<double>(name, default_value);
  this->get_parameter(name, value);
}

/**
 * @brief bool 型パラメータを declare して取得する。
 * @param name パラメータ名。
 * @param value 取得結果の格納先。
 * @param default_value declare 時の初期値。
 */
void R1MainNode::declare_and_get_parameter(
  const std::string & name, bool & value, bool default_value)
{
  this->declare_parameter<bool>(name, default_value);
  this->get_parameter(name, value);
}

/**
 * @brief int 型パラメータを declare して取得する。
 * @param name パラメータ名。
 * @param value 取得結果の格納先。
 * @param default_value declare 時の初期値。
 */
void R1MainNode::declare_and_get_parameter(const std::string & name, int & value, int default_value)
{
  this->declare_parameter<int>(name, default_value);
  this->get_parameter(name, value);
}

/**
 * @brief std::string 型パラメータを declare して取得する。
 * @param name パラメータ名。
 * @param value 取得結果の格納先。
 * @param default_value declare 時の初期値。
 */
void R1MainNode::declare_and_get_parameter(
  const std::string & name, std::string & value, const std::string & default_value)
{
  this->declare_parameter<std::string>(name, default_value);
  this->get_parameter(name, value);
}

/**
 * @brief mode_status 受信時に PositionAxisInterface の状態を更新する callback を生成する。
 * @param axis 状態を更新する位置制御軸インターフェース。
 * @param actuator_name ログ出力に使用する軸名。
 * @return mode_status subscription 用 callback。
 */
std::function<void(const std_msgs::msg::Int32::SharedPtr)> R1MainNode::create_mode_status_callback(
  PositionAxisInterface * axis, const std::string & actuator_name)
{
  return [this, axis, actuator_name](const std_msgs::msg::Int32::SharedPtr msg) {
    const bool next_is_pos_mode = (msg->data == 0);
    if (next_is_pos_mode && !axis->is_pos_mode) {
      RCLCPP_INFO(this->get_logger(), "%s detected origin", actuator_name.c_str());
    }
    axis->is_pos_mode = next_is_pos_mode;
  };
}

/**
 * @brief GPIO 入力状態を保持する callback を生成する。
 * @param switch_status 受信結果を書き込む先。
 * @return GPIO input subscription 用 callback。
 */
std::function<void(const r1_msgs::msg::GpioInput::SharedPtr)>
R1MainNode::create_switch_status_callback(bool * switch_status)
{
  return
    [switch_status](const r1_msgs::msg::GpioInput::SharedPtr msg) { *switch_status = msg->status; };
}

/**
 * @brief 位置制御軸の publisher / subscription を登録する。
 * @param name 軸名。topic 名の prefix に使用する。
 * @param position_ref_alias 最新の指令値を同期する既存変数。不要なら nullptr。
 */
void R1MainNode::register_position_axis(
  const std::string & name, double * position_ref_alias, double * speed_ref_alias)
{
  auto [it, inserted] = position_axes_.try_emplace(name);
  (void)inserted;
  auto & axis = it->second;
  axis.position_ref_alias = position_ref_alias;
  axis.speed_ref_alias = speed_ref_alias;
  if (axis.position_ref_alias != nullptr) {
    *axis.position_ref_alias = axis.position_ref;
  }
  if (axis.speed_ref_alias != nullptr) {
    *axis.speed_ref_alias = axis.speed_ref;
  }

  axis.position_ref_publisher =
    this->create_publisher<std_msgs::msg::Float64>("/" + name + "_position_ref", 10);
  axis.speed_ref_publisher =
    this->create_publisher<std_msgs::msg::Float64>("/" + name + "_speed_ref", 10);
  axis.detect_origin_publisher =
    this->create_publisher<std_msgs::msg::Bool>("/" + name + "_detect_origin", 10);
  axis.speed_mode_stop_publisher =
    this->create_publisher<std_msgs::msg::Empty>("/" + name + "_speed_mode_stop", 10);
  axis.move_mech_lock_publisher =
    this->create_publisher<std_msgs::msg::Int32>("/" + name + "_move_mech_lock", 10);
  axis.mode_status_subscription = this->create_subscription<std_msgs::msg::Int32>(
    "/" + name + "_mode_status", 10, create_mode_status_callback(&axis, name));
}

/**
 * @brief 速度制御軸の publisher を登録する。
 * @param name 軸名。
 * @param topic_name MotorRef を publish する topic 名。
 * @param velocity_ref_alias 最新の指令値を同期する既存変数。不要なら nullptr。
 */
void R1MainNode::register_velocity_axis(
  const std::string & name, const std::string & topic_name, double * velocity_ref_alias)
{
  auto [it, inserted] = velocity_axes_.try_emplace(name);
  (void)inserted;
  it->second.velocity_ref_alias = velocity_ref_alias;
  if (it->second.velocity_ref_alias != nullptr) {
    *it->second.velocity_ref_alias = it->second.velocity_ref;
  }
  it->second.motor_ref_publisher = this->create_publisher<r1_msgs::msg::MotorRef>(topic_name, 10);
}

/**
 * @brief GPIO PWM 出力 topic の publisher を登録する。
 * @param name GPIO 名。topic 名の prefix に使用する。
 * @param double_ref_alias 最新の指令値を同期する double 変数。不要なら nullptr。
 * @param bool_ref_alias 最新の指令値を同期する bool 変数。不要なら nullptr。
 */
void R1MainNode::register_gpio_pwm_output(
  const std::string & name, double * double_ref_alias, bool * bool_ref_alias)
{
  auto [it, inserted] = gpio_pwm_outputs_.try_emplace(name);
  (void)inserted;
  it->second.double_ref_alias = double_ref_alias;
  it->second.bool_ref_alias = bool_ref_alias;
  if (it->second.double_ref_alias != nullptr) {
    *it->second.double_ref_alias = it->second.ref;
  }
  if (it->second.bool_ref_alias != nullptr) {
    *it->second.bool_ref_alias = (it->second.ref != 0.0);
  }
  it->second.publisher =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/" + name + "_gpio_pwm_ref", 10);
}

/**
 * @brief GPIO サーボ出力 topic の publisher を登録する。
 * @param name GPIO 名。topic 名の prefix に使用する。
 * @param ref_alias 最新の指令値を同期する既存変数。不要なら nullptr。
 */
void R1MainNode::register_gpio_servo_output(const std::string & name, int * ref_alias)
{
  auto [it, inserted] = gpio_servo_outputs_.try_emplace(name);
  (void)inserted;
  it->second.ref_alias = ref_alias;
  if (it->second.ref_alias != nullptr) {
    *it->second.ref_alias = it->second.ref;
  }
  it->second.publisher =
    this->create_publisher<r1_msgs::msg::GpioServoRef>("/" + name + "_gpio_servo_ref", 10);
}

/**
 * @brief GPIO 入力 topic の subscription を登録する。
 * @param name GPIO 名。topic 名の prefix に使用する。
 * @param switch_status 受信結果を書き込む先。
 */
void R1MainNode::register_gpio_input(const std::string & name, bool * switch_status)
{
  auto [it, inserted] = gpio_inputs_.try_emplace(name);
  (void)inserted;
  it->second.subscription = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/" + name + "_status", 10, create_switch_status_callback(switch_status));
}

/**
 * @brief 指定した位置制御軸へ位置指令を publish する。
 * @param name 軸名。
 * @param pos 指令位置。
 */
void R1MainNode::publish_position_axis(const std::string & name, double pos)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.position_ref_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s position axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = pos;
  it->second.position_ref_publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s pos %f", name.c_str(), pos);
  it->second.position_ref = pos;
  if (it->second.position_ref_alias != nullptr) {
    *it->second.position_ref_alias = pos;
  }
}

/**
 * @brief 指定した位置制御軸へ速度指令を publish する。
 * @param name 軸名。
 * @param speed 指令速度。
 */
void R1MainNode::publish_position_axis_speed_ref(const std::string & name, double speed)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.speed_ref_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s speed_ref axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = speed;
  it->second.speed_ref_publisher->publish(msg);
  it->second.speed_ref = speed;
  if (it->second.speed_ref_alias != nullptr) {
    *it->second.speed_ref_alias = speed;
  }
  RCLCPP_INFO(this->get_logger(), "%s speed_ref %f", name.c_str(), speed);
}

/**
 * @brief 指定した位置制御軸へ原点検出指令を publish する。
 * @param name 軸名。
 */
void R1MainNode::detect_origin_position_axis(const std::string & name)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.detect_origin_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s detect_origin axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  it->second.detect_origin_publisher->publish(msg);
}

/**
 * @brief 指定した位置制御軸へ速度モード停止指令を publish する。
 * @param name 軸名。
 */
void R1MainNode::stop_position_axis_speed_mode(const std::string & name)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.speed_mode_stop_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s speed_mode_stop axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Empty msg;
  it->second.speed_mode_stop_publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s speed_mode_stop", name.c_str());
}

/**
 * @brief 指定した位置制御軸へ mech lock 移動指令を publish する。
 * @param name 軸名。
 * @param direction 移動方向。正の値で正方向、負の値で負方向、0 で停止。
 */
void R1MainNode::move_mech_lock_position_axis(const std::string & name, int direction)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.move_mech_lock_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s move_mech_lock axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = direction;
  it->second.move_mech_lock_publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s move_mech_lock direction %d", name.c_str(), direction);
}

/**
 * @brief 指定した速度制御軸へ速度指令を publish する。
 * @param name 軸名。
 * @param vel 指令速度。
 */
void R1MainNode::publish_velocity_axis(const std::string & name, double vel)
{
  const auto it = velocity_axes_.find(name);
  if (it == velocity_axes_.end() || !it->second.motor_ref_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s velocity axis is not initialized", name.c_str());
    return;
  }

  r1_msgs::msg::MotorRef msg;
  msg.control_type = "VELOCITY";
  msg.ref = vel;
  it->second.motor_ref_publisher->publish(msg);
  it->second.velocity_ref = vel;
  if (it->second.velocity_ref_alias != nullptr) {
    *it->second.velocity_ref_alias = vel;
  }
  // 速度制御はログが見づらくなるので、ログ出力はしない
}

/**
 * @brief 指定した GPIO PWM 出力へ指令を publish する。
 * @param name GPIO 名。
 * @param ref 指令値。
 */
void R1MainNode::publish_gpio_pwm_output(const std::string & name, double ref)
{
  const auto it = gpio_pwm_outputs_.find(name);
  if (it == gpio_pwm_outputs_.end() || !it->second.publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s gpio pwm output is not initialized", name.c_str());
    return;
  }

  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = ref;
  it->second.publisher->publish(msg);
  it->second.ref = ref;
  if (it->second.double_ref_alias != nullptr) {
    *it->second.double_ref_alias = ref;
  }
  if (it->second.bool_ref_alias != nullptr) {
    *it->second.bool_ref_alias = (ref != 0.0);
  }
  RCLCPP_INFO(this->get_logger(), "%s gpio pwm ref %f", name.c_str(), ref);
}

/**
 * @brief 指定した GPIO サーボ出力へ指令を publish する。
 * @param name GPIO 名。
 * @param ref 指令値。
 */
void R1MainNode::publish_gpio_servo_output(const std::string & name, int ref)
{
  const auto it = gpio_servo_outputs_.find(name);
  if (it == gpio_servo_outputs_.end() || !it->second.publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s gpio servo output is not initialized", name.c_str());
    return;
  }

  r1_msgs::msg::GpioServoRef msg;
  msg.ref = ref;
  it->second.publisher->publish(msg);
  it->second.ref = ref;
  if (it->second.ref_alias != nullptr) {
    *it->second.ref_alias = ref;
  }

  RCLCPP_INFO(this->get_logger(), "%s gpio servo ref %d", name.c_str(), ref);
}

R1MainNode::R1MainNode() : Node("r1_main_node")
{
  declare_and_get_parameter("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
  // 足回りの速度指令
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  // joy
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&R1MainNode::joy_callback, this, std::placeholders::_1));
  // ========== 位置制御軸 ==========
  register_position_axis("kfs_fx", &kfs_fx_position_ref_);
  register_position_axis("kfs_fz", &kfs_fz_position_ref_);
  register_position_axis("kfs_fyaw", &kfs_fyaw_position_ref_);
  register_position_axis("kfs_rx", &kfs_rx_position_ref_);
  register_position_axis("kfs_rz", &kfs_rz_position_ref_);
  register_position_axis("kfs_ryaw", &kfs_ryaw_position_ref_);
  register_position_axis("r2_flift", &r2_flift_position_ref_);
  register_position_axis("r2_rlift", &r2_rlift_position_ref_);
  register_position_axis("spear1", &spear1_position_ref_);
  register_position_axis("spear2", &spear2_position_ref_);
  register_position_axis("spear3", &spear3_position_ref_);
  register_position_axis("spear4", &spear4_position_ref_);
  register_position_axis("spear_x", &spear_x_position_ref_);
  register_position_axis("spear_y", &spear_y_position_ref_);
  register_position_axis("spear_roll", &spear_roll_position_ref_);
  register_position_axis("spear_pitch1", &spear_pitch1_position_ref_);
  register_position_axis("spear_pitch2", &spear_pitch2_position_ref_);

  // // ========== R2昇降指令値 ==========
  // register_velocity_axis("r2_flift", "/r2_flift_motor_ref", &r2_flift_velocity_ref_);
  // register_velocity_axis("r2_rlift", "/r2_rlift_motor_ref", &r2_rlift_velocity_ref_);
  // ========== GPIO ==========
  // kfs
  register_gpio_pwm_output("kfs_front_pump", &kfs_front_pump_ref_, nullptr);
  register_gpio_pwm_output("kfs_rear_pump", &kfs_rear_pump_ref_, nullptr);
  register_gpio_pwm_output("kfs_front_valve", nullptr, &kfs_front_valve_ref_);
  register_gpio_pwm_output("kfs_rear_valve", nullptr, &kfs_rear_valve_ref_);
  // 槍電磁弁
  register_gpio_pwm_output("spear_u1_valve", nullptr, &spear_u1_valve_ref_);
  register_gpio_pwm_output("spear_d1_valve", nullptr, &spear_d1_valve_ref_);
  register_gpio_pwm_output("spear_u2_valve", nullptr, &spear_u2_valve_ref_);
  register_gpio_pwm_output("spear_d2_valve", nullptr, &spear_d2_valve_ref_);
  // センサー入力
  register_gpio_input("kfs_fz_low_switch", &kfs_fz_low_switch_status_);
  register_gpio_input("kfs_rz_low_switch", &kfs_rz_low_switch_status_);

  // ========== Sabacan ==========
  // 電源基板の指令値Publisher
  sabacan_power_ref_publisher_ =
    this->create_publisher<sabacan_msgs::msg::SabacanPowerRef>("/sabacan_power_ref0", 10);
  // LED基板の指令値Publisher
  sabacan_led_ref_publisher_ =
    this->create_publisher<sabacan_msgs::msg::SabacanLEDRef>("/sabacan_led_ref1", 10);

  // IMU
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/bno086/imu/data_raw", 10, std::bind(&R1MainNode::imu_callback, this, std::placeholders::_1));

  // set_mecanum_yawのPublisher
  set_mecanum_yaw_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/set_mecanum_yaw", 10);
  // set_swerve_drive_yawのPublisher
  set_swerve_drive_yaw_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/set_swerve_drive_yaw", 10);

  // set_odometryのPublisher
  set_odometry_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/set_odometry", 10);
  // オドメトリのSubscription
  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", 10, std::bind(&R1MainNode::odometry_callback, this, std::placeholders::_1));
  // initialposeのPublisher
  initialpose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  // chassis_actのPublisher
  chassis_act_ref_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/chassis_act_ref", 10);
  // chassis_actのSubscription
  chassis_act_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/chassis_act_status", 10,
    std::bind(&R1MainNode::chassis_act_status_callback, this, std::placeholders::_1));
  // robot_moveのPublisher
  robot_move_publisher_ = this->create_publisher<r1_msgs::msg::RobotMove>("/robot_move", 10);
  // r1_machine_manage_node の初期化要求
  r1_machine_initialize_publisher_ =
    this->create_publisher<std_msgs::msg::Empty>("/r1_machine_initialize", 10);
  // r1_machine_manage_node の初期化完了通知
  r1_machine_initialize_done_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
    "/r1_machine_initialize_done", 10,
    std::bind(&R1MainNode::r1_machine_initialize_done_callback, this, std::placeholders::_1));

  // tf関連
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ========== パラメータ ==========
  // ゾーン
  this->declare_parameter<std::string>("zone", "blue");
  this->get_parameter("zone", zone_);
  if (zone_ != "blue" && zone_ != "red") {
    RCLCPP_ERROR(this->get_logger(), "Invalid zone parameter: %s", zone_.c_str());
    rclcpp::shutdown();
  }
  // 足回り
  declare_and_get_parameter("timer_rate", timer_rate_, 100.0);
  declare_and_get_parameter("ps4_connection_timeout", ps4_connection_timeout_, 0.3);
  declare_and_get_parameter("share_long_press_sec", SHARE_LONG_PRESS_SEC, 1.0);
  declare_and_get_parameter("chassis_make_spear_velocity", CHASSIS_MAKE_SPEAR_VELOCITY);
  declare_and_get_parameter("chassis_make_spear_omega", CHASSIS_MAKE_SPEAR_OMEGA);
  declare_and_get_parameter("chassis_max_velocity", CHASSIS_MAX_VELOCITY);
  declare_and_get_parameter("chassis_max_omega", CHASSIS_MAX_OMEGA);

  // ========== KFS回収 ==========
  declare_and_get_parameter("use_kfs_mech_lock", USE_KFS_MECH_LOCK);
  // fx
  declare_and_get_parameter("kfs_fx_normal_pos", KFS_FX_NORMAL_POS);
  declare_and_get_parameter("kfs_fx_start_pos", KFS_FX_START_POS);
  declare_and_get_parameter("kfs_fx_put_pos", KFS_FX_PUT_POS);
  declare_and_get_parameter("kfs_fx_storage_pos", KFS_FX_STORAGE_POS);
  declare_and_get_parameter("kfs_fx_expand_pos", KFS_FX_EXPAND_POS);
  declare_and_get_parameter("kfs_fx_low_mech_lock_pos", KFS_FX_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fx_high_mech_lock_pos", KFS_FX_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fx_r2_lift_pos", KFS_FX_R2_LIFT_POS);
  // fz
  declare_and_get_parameter("kfs_fz_normal_pos", KFS_FZ_NORMAL_POS);
  declare_and_get_parameter("kfs_fz_low_pos", KFS_FZ_LOW_POS);
  declare_and_get_parameter("kfs_fz_middle_pos", KFS_FZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_fz_high_pos", KFS_FZ_HIGH_POS);
  declare_and_get_parameter("kfs_fz_put_pos", KFS_FZ_PUT_POS);
  declare_and_get_parameter("kfs_fz_storage_pos", KFS_FZ_STORAGE_POS);
  declare_and_get_parameter("kfs_fz_low_mech_lock_pos", KFS_FZ_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fz_high_mech_lock_pos", KFS_FZ_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fz_r2_lift_pos", KFS_FZ_R2_LIFT_POS);
  // fyaw
  declare_and_get_parameter("kfs_fyaw_normal_angle", KFS_FYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_fyaw_front_angle", KFS_FYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_fyaw_side_angle", KFS_FYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_fyaw_rear_angle", KFS_FYAW_REAR_ANGLE);
  declare_and_get_parameter("kfs_fyaw_low_mech_lock_angle", KFS_FYAW_LOW_MECH_LOCK_ANGLE);
  declare_and_get_parameter("kfs_fyaw_high_mech_lock_angle", KFS_FYAW_HIGH_MECH_LOCK_ANGLE);
  // rx
  declare_and_get_parameter("kfs_rx_normal_pos", KFS_RX_NORMAL_POS);
  declare_and_get_parameter("kfs_rx_start_pos", KFS_RX_START_POS);
  declare_and_get_parameter("kfs_rx_put_pos", KFS_RX_PUT_POS);
  declare_and_get_parameter("kfs_rx_storage_pos", KFS_RX_STORAGE_POS);
  declare_and_get_parameter("kfs_rx_expand_pos", KFS_RX_EXPAND_POS);
  declare_and_get_parameter("kfs_rx_low_mech_lock_pos", KFS_RX_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rx_high_mech_lock_pos", KFS_RX_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rx_r2_lift_pos", KFS_RX_R2_LIFT_POS);
  // rz
  declare_and_get_parameter("kfs_rz_normal_pos", KFS_RZ_NORMAL_POS);
  declare_and_get_parameter("kfs_rz_low_pos", KFS_RZ_LOW_POS);
  declare_and_get_parameter("kfs_rz_middle_pos", KFS_RZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_rz_high_pos", KFS_RZ_HIGH_POS);
  declare_and_get_parameter("kfs_rz_put_pos", KFS_RZ_PUT_POS);
  declare_and_get_parameter("kfs_rz_storage_pos", KFS_RZ_STORAGE_POS);
  declare_and_get_parameter("kfs_rz_low_mech_lock_pos", KFS_RZ_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rz_high_mech_lock_pos", KFS_RZ_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rz_r2_lift_pos", KFS_RZ_R2_LIFT_POS);
  // ryaw
  declare_and_get_parameter("kfs_ryaw_normal_angle", KFS_RYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_ryaw_front_angle", KFS_RYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_ryaw_side_angle", KFS_RYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_ryaw_rear_angle", KFS_RYAW_REAR_ANGLE);
  declare_and_get_parameter("kfs_ryaw_low_mech_lock_angle", KFS_RYAW_LOW_MECH_LOCK_ANGLE);
  declare_and_get_parameter("kfs_ryaw_high_mech_lock_angle", KFS_RYAW_HIGH_MECH_LOCK_ANGLE);

  // ========== 展開 ==========
  // R2昇降
  declare_and_get_parameter("r2_flift_normal_pos", R2_FLIFT_NORMAL_POS);
  declare_and_get_parameter("r2_flift_up_pos", R2_FLIFT_UP_POS);
  declare_and_get_parameter("r2_flift_down_pos", R2_FLIFT_DOWN_POS);
  declare_and_get_parameter("r2_rlift_normal_pos", R2_RLIFT_NORMAL_POS);
  declare_and_get_parameter("r2_rlift_up_pos", R2_RLIFT_UP_POS);
  declare_and_get_parameter("r2_rlift_down_pos", R2_RLIFT_DOWN_POS);
  // ========== やり ==========
  // spear1
  declare_and_get_parameter("spear1_normal_pos", SPEAR1_NORMAL_POS);
  declare_and_get_parameter("spear1_collect1_pos", SPEAR1_COLLECT1_POS);
  declare_and_get_parameter("spear1_collect2_pos", SPEAR1_COLLECT2_POS);
  declare_and_get_parameter("spear1_collect3_pos", SPEAR1_COLLECT3_POS);
  declare_and_get_parameter("spear1_make_spear_start_pos", SPEAR1_MAKE_SPEAR_START_POS);
  declare_and_get_parameter("spear1_kfs_collect_pos", SPEAR1_KFS_COLLECT_POS);
  declare_and_get_parameter("spear1_low_attack_pos", SPEAR1_LOW_ATTACK_POS);
  declare_and_get_parameter("spear1_middle_attack_pos", SPEAR1_MIDDLE_ATTACK_POS);
  declare_and_get_parameter("spear1_high_attack_pos", SPEAR1_HIGH_ATTACK_POS);
  declare_and_get_parameter("spear1_push_vel", SPEAR1_PUSH_VEL);
  // spear2
  declare_and_get_parameter("spear2_normal_pos", SPEAR2_NORMAL_POS);
  declare_and_get_parameter("spear2_collect1_pos", SPEAR2_COLLECT1_POS);
  declare_and_get_parameter("spear2_collect2_pos", SPEAR2_COLLECT2_POS);
  declare_and_get_parameter("spear2_collect3_pos", SPEAR2_COLLECT3_POS);
  declare_and_get_parameter("spear2_make_spear_start_pos", SPEAR2_MAKE_SPEAR_START_POS);
  declare_and_get_parameter("spear2_kfs_collect_pos", SPEAR2_KFS_COLLECT_POS);
  declare_and_get_parameter("spear2_low_attack_pos", SPEAR2_LOW_ATTACK_POS);
  declare_and_get_parameter("spear2_middle_attack_pos", SPEAR2_MIDDLE_ATTACK_POS);
  declare_and_get_parameter("spear2_high_attack_pos", SPEAR2_HIGH_ATTACK_POS);
  declare_and_get_parameter("spear2_push_vel", SPEAR2_PUSH_VEL);
  // spear3
  declare_and_get_parameter("spear3_normal_pos", SPEAR3_NORMAL_POS);
  declare_and_get_parameter("spear3_collect_pos", SPEAR3_COLLECT_POS);
  declare_and_get_parameter("spear3_make_spear_start_pos", SPEAR3_MAKE_SPEAR_START_POS);
  declare_and_get_parameter("spear3_kfs_collect_pos", SPEAR3_KFS_COLLECT_POS);
  declare_and_get_parameter("spear3_push_vel", SPEAR3_PUSH_VEL);
  // spear4
  declare_and_get_parameter("spear4_normal_pos", SPEAR4_NORMAL_POS);
  declare_and_get_parameter("spear4_collect_pos", SPEAR4_COLLECT_POS);
  declare_and_get_parameter("spear4_make_spear_start_pos", SPEAR4_MAKE_SPEAR_START_POS);
  declare_and_get_parameter("spear4_kfs_collect_pos", SPEAR4_KFS_COLLECT_POS);
  declare_and_get_parameter("spear4_push_vel", SPEAR4_PUSH_VEL);
  // spear_x
  declare_and_get_parameter("spear_x_normal_pos", SPEAR_X_NORMAL_POS);
  declare_and_get_parameter("spear_x_middle_pos", SPEAR_X_MIDDLE_POS);
  declare_and_get_parameter("spear_x_make_spear1_pos", SPEAR_X_MAKE_SPEAR1_POS);
  declare_and_get_parameter("spear_x_make_spear2_pos", SPEAR_X_MAKE_SPEAR2_POS);
  declare_and_get_parameter("spear_x_make_spear3_pos", SPEAR_X_MAKE_SPEAR3_POS);
  declare_and_get_parameter("spear_x_make_spear4_pos", SPEAR_X_MAKE_SPEAR4_POS);
  // spear_y
  declare_and_get_parameter("spear_y_normal_pos", SPEAR_Y_NORMAL_POS);
  declare_and_get_parameter("spear_y_expand_pos", SPEAR_Y_EXPAND_POS);
  // spear_roll
  declare_and_get_parameter("spear_roll_normal_angle", SPEAR_ROLL_NORMAL_ANGLE);
  declare_and_get_parameter("spear_roll_inv_normal_angle", SPEAR_ROLL_INV_NORMAL_ANGLE);
  declare_and_get_parameter("spear_roll_vertical_angle", SPEAR_ROLL_VERTICAL_ANGLE);
  declare_and_get_parameter("spear_roll_low_attack_angle", SPEAR_ROLL_LOW_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll_middle_attack_angle", SPEAR_ROLL_MIDDLE_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll_high_attack_angle", SPEAR_ROLL_HIGH_ATTACK_ANGLE);
  // spear_pitch1
  declare_and_get_parameter("spear_pitch1_normal_angle", SPEAR_PITCH1_NORMAL_ANGLE);
  declare_and_get_parameter("spear_pitch1_vertical_angle", SPEAR_PITCH1_VERTICAL_ANGLE);
  // spear_pitch2
  declare_and_get_parameter("spear_pitch2_normal_angle", SPEAR_PITCH2_NORMAL_ANGLE);
  declare_and_get_parameter("spear_pitch2_vertical_angle", SPEAR_PITCH2_VERTICAL_ANGLE);

  // FOREST関連
  this->declare_parameter<std::vector<int64_t>>("kfs_forest_number", std::vector<int64_t>{});
  std::vector<int64_t> kfs_forest_number;
  this->get_parameter("kfs_forest_number", kfs_forest_number);
  for (int i = 0; i < (int)kfs_forest_number.size(); i++) {
    if (kfs_forest_number[i] < 1 || kfs_forest_number[i] > 12) {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid kfs_forest_number[%d]: %ld. Must be between 1 and 12.", i,
        static_cast<long>(kfs_forest_number[i]));
      rclcpp::shutdown();
    } else {
      KFS_FOREST_NUMBER.push_back(kfs_forest_number[i]);
    }
  }

  for (int i = 0; i < 12; i++) {
    std::string inner_center_name = "inner_collect_kfs_center_pos." + std::to_string(i + 1);
    std::string outer_center_name = "outer_collect_kfs_center_pos." + std::to_string(i + 1);
    // 適当な初期値を代入
    this->declare_parameter<std::vector<double>>(inner_center_name, {100.0, 100.0, 0.0});
    this->declare_parameter<std::vector<double>>(outer_center_name, {100.0, 100.0, 0.0});
    std::vector<double> inner_center, outer_center;
    this->get_parameter(inner_center_name, inner_center);
    this->get_parameter(outer_center_name, outer_center);

    if (inner_center.size() != 3 || outer_center.size() != 3) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid parameter size for collect_kfs center positions. Each position must have 3 "
        "elements (x, y, yaw).");
      rclcpp::shutdown();
    }

    INNER_COLLECT_KFS_CENTER_POS.push_back(inner_center);
    OUTER_COLLECT_KFS_CENTER_POS.push_back(outer_center);
  }
  declare_and_get_parameter("collect_kfs_height", COLLECT_KFS_HEIGHT);
  declare_and_get_parameter("collect_kfs_width", COLLECT_KFS_WIDTH);
  declare_and_get_parameter("collect_kfs_offset", COLLECT_KFS_OFFSET);
  declare_and_get_parameter("kfs_yaw_delay_time", KFS_YAW_DELAY_TIME, 1.0);
  declare_and_get_parameter(
    "enable_auto_collect_kfs_actuator", ENABLE_AUTO_COLLECT_KFS_ACTUATOR, true);
  parameter_callback_handle_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto & parameter : parameters) {
        if (parameter.get_name() != "enable_auto_collect_kfs_actuator") {
          continue;
        }
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "enable_auto_collect_kfs_actuator must be bool";
          return result;
        }
        ENABLE_AUTO_COLLECT_KFS_ACTUATOR = parameter.as_bool();
      }
      return result;
    });

  std::string robot_control_mode_parameter;
  this->declare_parameter<std::string>("robot_control_mode", "manual");
  this->get_parameter("robot_control_mode", robot_control_mode_parameter);
  const auto resolved_initial_state =
    parse_robot_control_mode_parameter(robot_control_mode_parameter);
  if (!resolved_initial_state) {
    RCLCPP_FATAL(
      this->get_logger(), "Invalid robot_control_mode parameter: %s. %s",
      robot_control_mode_parameter.c_str(), robot_control_mode_parameter_help().c_str());
    rclcpp::shutdown();
    return;
  }
  initial_state_ = *resolved_initial_state;

  if (timer_rate_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "timer_rate must be positive. Fallback to 100.0 Hz.");
    timer_rate_ = 100.0;
  }

  // タイマー
  timer_publisher_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&R1MainNode::timer_callback, this));

  const double timer_dt = 1.0 / timer_rate_;
  simple_trapezoid_vx_ = SimpleTrapezoid(3.0, timer_dt);
  simple_trapezoid_vy_ = SimpleTrapezoid(3.0, timer_dt);
  simple_trapezoid_omega_ = SimpleTrapezoid(3.0, timer_dt);

  ps4_ = std::make_shared<PS4>("PS4");
  ps4_->set_connection_timeout(ps4_connection_timeout_);

  state_machine_ = std::make_shared<StateMachine>();
  state_machine_->set_next_state(initial_state_);
  state_machine_->print_state(initial_state_, "Configured initial state: ");
  // アクチュエータを初期化
  // init_actuator();
}

void R1MainNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // IMUの情報を更新
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
}

void R1MainNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_ = *msg;
}

void R1MainNode::chassis_act_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  chassis_act_status_ = static_cast<ChassisAct>(msg->data);
}

void R1MainNode::kfs_fx_detect_origin(void) { detect_origin_position_axis("kfs_fx"); }

void R1MainNode::kfs_fz_detect_origin(void) { detect_origin_position_axis("kfs_fz"); }

void R1MainNode::kfs_fyaw_detect_origin(void) { detect_origin_position_axis("kfs_fyaw"); }

void R1MainNode::kfs_rx_detect_origin(void) { detect_origin_position_axis("kfs_rx"); }

void R1MainNode::kfs_rz_detect_origin(void) { detect_origin_position_axis("kfs_rz"); }

void R1MainNode::kfs_ryaw_detect_origin(void) { detect_origin_position_axis("kfs_ryaw"); }

void R1MainNode::r2_flift_detect_origin(void) { detect_origin_position_axis("r2_flift"); }

void R1MainNode::r2_rlift_detect_origin(void) { detect_origin_position_axis("r2_rlift"); }

void R1MainNode::spear1_detect_origin(void) { detect_origin_position_axis("spear1"); }

void R1MainNode::spear2_detect_origin(void) { detect_origin_position_axis("spear2"); }

void R1MainNode::spear3_detect_origin(void) { detect_origin_position_axis("spear3"); }

void R1MainNode::spear4_detect_origin(void) { detect_origin_position_axis("spear4"); }

void R1MainNode::spear_x_detect_origin(void) { detect_origin_position_axis("spear_x"); }

void R1MainNode::spear_y_detect_origin(void) { detect_origin_position_axis("spear_y"); }

void R1MainNode::spear_roll_detect_origin(void) { detect_origin_position_axis("spear_roll"); }

void R1MainNode::spear_pitch1_detect_origin(void) { detect_origin_position_axis("spear_pitch1"); }

void R1MainNode::spear_pitch2_detect_origin(void) { detect_origin_position_axis("spear_pitch2"); }

void R1MainNode::kfs_fx_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_fx", direction);
}

void R1MainNode::kfs_fz_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_fz", direction);
}

void R1MainNode::kfs_fyaw_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_fyaw", direction);
}

void R1MainNode::kfs_rx_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_rx", direction);
}

void R1MainNode::kfs_rz_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_rz", direction);
}

void R1MainNode::kfs_ryaw_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("kfs_ryaw", direction);
}

void R1MainNode::r2_flift_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("r2_flift", direction);
}

void R1MainNode::r2_rlift_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("r2_rlift", direction);
}

void R1MainNode::spear1_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear1", direction);
}

void R1MainNode::spear2_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear2", direction);
}

void R1MainNode::spear3_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear3", direction);
}

void R1MainNode::spear4_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear4", direction);
}

void R1MainNode::spear_x_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_x", direction);
}

void R1MainNode::spear_y_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_y", direction);
}

void R1MainNode::spear_roll_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_roll", direction);
}

void R1MainNode::spear_pitch1_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_pitch1", direction);
}

void R1MainNode::spear_pitch2_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_pitch2", direction);
}

void R1MainNode::kfs_fyaw_move_front_mech_lock(void) { kfs_fyaw_move_mech_lock(1); }
void R1MainNode::kfs_fyaw_move_rear_mech_lock(void) { kfs_fyaw_move_mech_lock(-1); }
void R1MainNode::kfs_ryaw_move_front_mech_lock(void) { kfs_ryaw_move_mech_lock(1); }
void R1MainNode::kfs_ryaw_move_rear_mech_lock(void) { kfs_ryaw_move_mech_lock(-1); }

// --- コールバック関数 ---
void R1MainNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  ps4_->joy_callback(msg);
}

void R1MainNode::sabacan_power_ref(bool is_ems)
{
  sabacan_msgs::msg::SabacanPowerRef msg;
  msg.is_ems = is_ems;
  sabacan_is_ems_ = is_ems;
  sabacan_power_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "sabacan power ref is_ems: %d", is_ems);
}

void R1MainNode::sabacan_led_ref(int pin_number, uint8_t r, uint8_t g, uint8_t b)
{
  sabacan_msgs::msg::SabacanLEDRef msg;
  msg.pin_number = pin_number;
  msg.start = 0;
  msg.length = 255;
  msg.r = r;
  msg.g = g;
  msg.b = b;
  sabacan_led_ref_publisher_->publish(msg);
  // RCLCPP_INFO(this->get_logger(), "sabacan led ref r: %d, g: %d, b: %d", r, g, b);
}

void R1MainNode::set_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s)
{
  // status は AUTO 中の範囲判定のような、その周期だけ使う上書き表示。
  led_status_pattern_.enabled = true;
  led_status_pattern_.color = {r, g, b};
  led_status_pattern_.blink_period_s = (blink_period_s > 0.0) ? blink_period_s : 0.0;
}

void R1MainNode::set_fkfs_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s)
{
  led_fkfs_status_pattern_.enabled = true;
  led_fkfs_status_pattern_.color = {r, g, b};
  led_fkfs_status_pattern_.blink_period_s = (blink_period_s > 0.0) ? blink_period_s : 0.0;
}

void R1MainNode::set_rkfs_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s)
{
  led_rkfs_status_pattern_.enabled = true;
  led_rkfs_status_pattern_.color = {r, g, b};
  led_rkfs_status_pattern_.blink_period_s = (blink_period_s > 0.0) ? blink_period_s : 0.0;
}

void R1MainNode::clear_led_status(void)
{
  led_status_pattern_.enabled = false;
  led_status_pattern_.color = {};
  led_status_pattern_.blink_period_s = 0.0;
  led_fkfs_status_pattern_.enabled = false;
  led_fkfs_status_pattern_.color = {};
  led_fkfs_status_pattern_.blink_period_s = 0.0;
  led_rkfs_status_pattern_.enabled = false;
  led_rkfs_status_pattern_.color = {};
  led_rkfs_status_pattern_.blink_period_s = 0.0;
}

void R1MainNode::set_led_event(
  uint8_t r, uint8_t g, uint8_t b, double blink_period_s, double duration_sec)
{
  // event は reset 直後の通知のような、一時的に最優先で見せたい表示。
  led_event_pattern_.enabled = true;
  led_event_pattern_.color = {r, g, b};
  led_event_pattern_.blink_period_s = (blink_period_s > 0.0) ? blink_period_s : 0.0;
  led_event_expire_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(
                                                        (duration_sec > 0.0) ? duration_sec : 0.0);
}

R1MainNode::LedPattern R1MainNode::resolve_base_led_pattern(void)
{
  // base は通常時の表示。状態遷移先に合わせて色を決める。
  const auto state = state_machine_->get_next_state();
  auto apply_led_blink = [this, &state](LedPattern pattern) {
    if (state.chassis_control_mode == ChassisControlMode::AUTO && pattern.enabled) {
      pattern.blink_period_s = 0.25;
      return pattern;
    }
    if (chassis_rotate90 && pattern.enabled && pattern.blink_period_s <= 0.0) {
      pattern.blink_period_s = 0.5;
    }
    return pattern;
  };

  if (state.main == MainState::IDLE) {
    // 消灯
    return apply_led_blink(LedPattern{});
  }
  if (state.main == MainState::EMERGENCY) {
    // 赤点滅
    return apply_led_blink(LedPattern{true, {50, 0, 0}, 0.5});
  }
  if (state.main != MainState::READY) {
    // 想定外状態は消灯
    return apply_led_blink(LedPattern{});
  }

  if (state.operation_mode == OperationMode::MODE1_DETECT_ORIGIN) {
    return apply_led_blink(LedPattern{true, {0, 0, 50}, 0});
  }
  if (state.operation_mode == OperationMode::MODE2_POLE) {
    return apply_led_blink(LedPattern{true, {0, 50, 0}, 0});
  }
  if (state.operation_mode == OperationMode::MODE3_SPEAR) {
    return apply_led_blink(LedPattern{true, {0, 50, 50}, 0});
  }
  if (state.operation_mode == OperationMode::MODE4_FKFS) {
    return apply_led_blink(LedPattern{true, {50, 0, 0}, 0});
  }
  if (state.operation_mode == OperationMode::MODE5_RKFS) {
    return apply_led_blink(LedPattern{true, {50, 0, 50}, 0});
  }
  if (state.operation_mode == OperationMode::MODE6_R2_LIFT) {
    return apply_led_blink(LedPattern{true, {50, 50, 0}, 0});
  }
  if (state.operation_mode == OperationMode::MODE7_SPEAR_ATTACK) {
    return apply_led_blink(LedPattern{true, {50, 50, 50}, 0});
  }
  if (state.operation_mode == OperationMode::MODE8_AUTO_COLLECT_KFS) {
    return apply_led_blink(LedPattern{true, {50, 25, 0}, 0});
  }
  if (state.operation_mode == OperationMode::MODE9_AUTO_CHASSIS) {
    // TODO: 9の色は変える
    return apply_led_blink(LedPattern{true, {50, 50, 0}, 0});
  }

  // 未定義なら消灯
  return apply_led_blink(LedPattern{});
}

R1MainNode::LedColor R1MainNode::resolve_led_output_color(
  const LedPattern & pattern, const rclcpp::Time & now) const
{
  if (!pattern.enabled) {
    return {};
  }
  if (pattern.blink_period_s <= 0.0) {
    return pattern.color;
  }

  const int64_t period_ns = static_cast<int64_t>(pattern.blink_period_s * 1.0e9);
  if (period_ns <= 0) {
    return pattern.color;
  }

  // 周期の前半だけ点灯させる 50% デューティの単純な点滅。
  const int64_t phase_ns = now.nanoseconds() % period_ns;
  if (phase_ns < (period_ns / 2)) {
    return pattern.color;
  }
  return {};
}

void R1MainNode::publish_r1_machine_initialize(void)
{
  std_msgs::msg::Empty msg;
  r1_machine_initialize_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish /r1_machine_initialize");
}

void R1MainNode::invalidate_led_cache(void)
{
  has_last_led_color_ = false;
  has_last_led_fkfs_color_ = false;
  has_last_led_rkfs_color_ = false;
}

void R1MainNode::r1_machine_initialize_done_callback(const std_msgs::msg::Empty::SharedPtr)
{
  is_initialized_ = true;
  // initialize 完了直後に LED を再送できるよう、出力キャッシュを無効化する。
  // 将来的にここへ原点検出後処理や初期姿勢への指令を追加する場合も、この callback を
  // sabacan 初期化完了後の集約ポイントとして拡張する。
  invalidate_led_cache();
  RCLCPP_INFO(this->get_logger(), "received /r1_machine_initialize_done");
}

void R1MainNode::sabacan_led_update(void)
{
  const auto now = this->get_clock()->now();
  if (led_event_pattern_.enabled && now >= led_event_expire_time_) {
    led_event_pattern_.enabled = false;
  }

  // 優先順位は event > status > base。
  LedPattern pattern = resolve_base_led_pattern();
  if (led_status_pattern_.enabled) {
    pattern = led_status_pattern_;
  }
  if (led_event_pattern_.enabled) {
    pattern = led_event_pattern_;
  }

  const LedColor color = resolve_led_output_color(pattern, now);
  if (!has_last_led_color_ || color != last_led_color_) {
    sabacan_led_ref(LED_SYSTEM, color.r, color.g, color.b);
    last_led_color_ = color;
    has_last_led_color_ = true;
  }

  const LedColor fkfs_color = resolve_led_output_color(led_fkfs_status_pattern_, now);
  if (!has_last_led_fkfs_color_ || fkfs_color != last_led_fkfs_color_) {
    sabacan_led_ref(LED_FKFS, fkfs_color.r, fkfs_color.g, fkfs_color.b);
    last_led_fkfs_color_ = fkfs_color;
    has_last_led_fkfs_color_ = true;
  }

  const LedColor rkfs_color = resolve_led_output_color(led_rkfs_status_pattern_, now);
  if (!has_last_led_rkfs_color_ || rkfs_color != last_led_rkfs_color_) {
    sabacan_led_ref(LED_RKFS, rkfs_color.r, rkfs_color.g, rkfs_color.b);
    last_led_rkfs_color_ = rkfs_color;
    has_last_led_rkfs_color_ = true;
  }
}

void R1MainNode::set_mecanum_yaw(double yaw)
{
  std_msgs::msg::Float64 msg;
  msg.data = yaw;
  set_mecanum_yaw_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "set mecanum yaw: %f", yaw);
}

void R1MainNode::set_swerve_drive_yaw(double yaw)
{
  std_msgs::msg::Float64 msg;
  msg.data = yaw;
  set_swerve_drive_yaw_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "set swerve drive yaw: %f", yaw);
}

void R1MainNode::set_odometry(double x, double y, double yaw)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(x);
  msg.data.push_back(y);
  msg.data.push_back(yaw);
  set_odometry_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "set odometry x: %f, y: %f, yaw: %f", x, y, yaw);
}

void R1MainNode::set_initialpose(double x, double y, double yaw, double delay_sec)
{
  // setTimeout風でdelay_sec秒後にinitialposeをpublishする
  initialpose_publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(delay_sec), [this, x, y, yaw]() {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";
      msg.pose.pose.position.x = x;
      msg.pose.pose.position.y = y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      msg.pose.pose.orientation.x = q.x();
      msg.pose.pose.orientation.y = q.y();
      msg.pose.pose.orientation.z = q.z();
      msg.pose.pose.orientation.w = q.w();
      initialpose_publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "publish initialpose x: %f, y: %f, yaw: %f", x, y, yaw);
      // タイマーは1回だけ実行するので、初期化する
      initialpose_publish_timer_->cancel();
    });
}

void R1MainNode::publish_chassis_act_ref(ChassisAct ref)
{
  std_msgs::msg::Int32 msg;
  msg.data = static_cast<int>(ref);
  chassis_act_ref_publisher_->publish(msg);
  // RCLCPP_INFO(this->get_logger(), "chassis act ref: %d", ref);
}

void R1MainNode::publish_chassis_act_stop(void)
{
  ChassisAct stop_ref = ChassisAct::NONE;
  switch (chassis_act_status_) {
    case ChassisAct::ACT0_START:
    case ChassisAct::ACT0:
      stop_ref = ChassisAct::ACT0_FINISH;
      break;
    case ChassisAct::ACT1_START:
    case ChassisAct::ACT1:
      stop_ref = ChassisAct::ACT1_FINISH;
      break;
    case ChassisAct::ACT2_START:
    case ChassisAct::ACT2:
      stop_ref = ChassisAct::ACT2_FINISH;
      break;
    case ChassisAct::ACT3_START:
    case ChassisAct::ACT3:
      stop_ref = ChassisAct::ACT3_FINISH;
      break;
    case ChassisAct::ACT4_START:
    case ChassisAct::ACT4:
      stop_ref = ChassisAct::ACT4_FINISH;
      break;
    case ChassisAct::ACT0_FINISH:
    case ChassisAct::ACT1_FINISH:
    case ChassisAct::ACT2_FINISH:
    case ChassisAct::ACT3_FINISH:
    case ChassisAct::ACT4_FINISH:
    case ChassisAct::NONE:
    default:
      stop_ref = ChassisAct::NONE;
      break;
  }

  publish_chassis_act_ref(stop_ref);
}

void R1MainNode::publish_robot_move(
  ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type)
{
  r1_msgs::msg::RobotMove msg;
  msg.act = static_cast<int>(act);
  msg.forest_order = forest_order;
  msg.kfs_mechanism_type = kfs_mechanism_type;
  robot_move_publisher_->publish(msg);
  std::string forest_order_str = "[";
  for (size_t i = 0; i < forest_order.size(); i++) {
    forest_order_str += std::to_string(forest_order[i]);
    if (i + 1 < forest_order.size()) {
      forest_order_str += ", ";
    }
  }
  forest_order_str += "]";
  std::string mechanism_type_str = "[";
  for (size_t i = 0; i < kfs_mechanism_type.size(); i++) {
    mechanism_type_str += kfs_mechanism_type[i];
    if (i + 1 < kfs_mechanism_type.size()) {
      mechanism_type_str += ", ";
    }
  }
  mechanism_type_str += "]";
  RCLCPP_INFO(
    this->get_logger(), "publish robot move act: %d, forest_order: %s, mechanism_type: %s",
    static_cast<int>(act), forest_order_str.c_str(), mechanism_type_str.c_str());
  current_robot_move_ = msg;
}

bool R1MainNode::is_localization_ready(void)
{
  if (!tf_buffer_->_frameExists("map") || !tf_buffer_->_frameExists("base_link")) {
    return false;
  }
  std::string err;
  return tf_buffer_->canTransform(
    "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.0), &err);
}

void R1MainNode::request_auto_robot_move(
  ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type)
{
  if (is_localization_ready()) {
    publish_robot_move(act, forest_order, kfs_mechanism_type);
    pending_auto_robot_move_valid_ = false;
    return;
  }

  pending_auto_robot_move_.act = static_cast<int>(act);
  pending_auto_robot_move_.forest_order = forest_order;
  pending_auto_robot_move_.kfs_mechanism_type = kfs_mechanism_type;
  pending_auto_robot_move_valid_ = true;

  std::string act_name{magic_enum::enum_name(act)};
  RCLCPP_WARN(
    this->get_logger(),
    "Localization is not ready yet. Queued %s until map->base_link becomes available.",
    act_name.c_str());
}

void R1MainNode::publish_pending_auto_robot_move_if_ready(void)
{
  if (!pending_auto_robot_move_valid_) {
    return;
  }
  if (!is_localization_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for localization: map->base_link TF is not available yet.");
    return;
  }

  publish_robot_move(
    static_cast<ChassisAct>(pending_auto_robot_move_.act), pending_auto_robot_move_.forest_order,
    pending_auto_robot_move_.kfs_mechanism_type);
  pending_auto_robot_move_valid_ = false;
}

void R1MainNode::start_auto_chassis(
  ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type)
{
  auto_chassis_status_ = act;
  request_auto_robot_move(act, std::move(forest_order), std::move(kfs_mechanism_type));
}

void R1MainNode::clear_auto_chassis_state(bool stop_kfs_auto_collect)
{
  if (stop_kfs_auto_collect) {
    this->stop_kfs_auto_collect();
  }
  auto_chassis_status_ = ChassisAct::NONE;
  pending_auto_robot_move_valid_ = false;
  pending_auto_robot_move_ = r1_msgs::msg::RobotMove();
}

bool R1MainNode::build_inner_kfs_auto_collect_plan(
  std::vector<int> & forest_order, std::vector<std::string> & collect_kfs_type) const
{
  forest_order.clear();
  collect_kfs_type.clear();

  if (zone_ != "blue") {
    RCLCPP_ERROR(
      this->get_logger(), "inner kfs auto collect plan is not implemented for zone=%s",
      zone_.c_str());
    return false;
  }

  int assigned_count = 0;
  for (const int n : KFS_FOREST_NUMBER) {
    if (n != 1 && n != 2 && n != 4 && n != 7 && n != 10) {
      continue;
    }
    forest_order.push_back(n);
    if (assigned_count == 0) {
      collect_kfs_type.push_back("rear_kfs");
      assigned_count++;
    } else if (assigned_count == 1) {
      collect_kfs_type.push_back("front_kfs");
    } else {
      RCLCPP_ERROR(this->get_logger(), "inner kfs collect_kfs_type size error");
      return false;
    }
  }

  return !forest_order.empty() && forest_order.size() == collect_kfs_type.size();
}

bool R1MainNode::build_outer_kfs_auto_collect_plan(
  std::vector<int> & forest_order, std::vector<std::string> & collect_kfs_type) const
{
  forest_order.clear();
  collect_kfs_type.clear();

  if (zone_ != "blue") {
    RCLCPP_ERROR(
      this->get_logger(), "outer kfs auto collect plan is not implemented for zone=%s",
      zone_.c_str());
    return false;
  }

  int assigned_count = 0;
  for (const int n : KFS_FOREST_NUMBER) {
    if (n != 3 && n != 6 && n != 9 && n != 12 && n != 11 && n != 10) {
      continue;
    }
    forest_order.push_back(n);
    if (assigned_count == 0) {
      collect_kfs_type.push_back("front_kfs");
      assigned_count++;
    } else if (assigned_count == 1) {
      collect_kfs_type.push_back("rear_kfs");
    } else {
      RCLCPP_ERROR(this->get_logger(), "outer kfs collect_kfs_type size error");
      return false;
    }
  }

  return !forest_order.empty() && forest_order.size() == collect_kfs_type.size();
}

bool R1MainNode::set_mode3_kfs_auto_collect_status(KfsAutoCollectStatus & status)
{
  auto is_inner_number = [](int n) { return n == 1 || n == 2 || n == 4 || n == 7 || n == 10; };
  auto is_outer_number = [](int n) {
    return n == 3 || n == 6 || n == 9 || n == 10 || n == 11 || n == 12;
  };

  int inner_count = 0;
  int outer_count = 0;

  // 回収するKFSの個数を多数決で決定する
  for (const int n : KFS_FOREST_NUMBER) {
    if (is_inner_number(n)) inner_count++;
    if (is_outer_number(n)) outer_count++;
  }

  if (outer_count > inner_count) {
    status = KfsAutoCollectStatus::OUTER_ACTIVE;
  } else {
    // inner_count >= outer_count (同数の場合はINNERを優先)
    status = KfsAutoCollectStatus::INNER_ACTIVE;
  }

  RCLCPP_INFO(
    this->get_logger(), "Set MODE3_SPEAR status to %s (inner=%d, outer=%d)",
    kfs_auto_collect_status_name(status).c_str(), inner_count, outer_count);
  return true;
}

void R1MainNode::reset_kfs_auto_collect_tracking(void)
{
  if (auto_collect_front_storage_yaw_timer_) {
    auto_collect_front_storage_yaw_timer_->cancel();
  }
  if (auto_collect_rear_storage_yaw_timer_) {
    auto_collect_rear_storage_yaw_timer_->cancel();
  }
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < 2; j++) {
      kfs_auto_collect_within_[i][j] = false;
      kfs_auto_collect_prev_within_[i][j] = false;
    }
  }
}

void R1MainNode::start_kfs_auto_collect(
  KfsAutoCollectStatus status, std::vector<int> forest_order,
  std::vector<std::string> kfs_mechanism_type)
{
  if (status == KfsAutoCollectStatus::NONE) {
    stop_kfs_auto_collect();
    return;
  }

  if (forest_order.empty() || forest_order.size() != kfs_mechanism_type.size()) {
    RCLCPP_ERROR(
      this->get_logger(), "invalid kfs auto collect plan: forest_order=%zu, kfs_mechanism_type=%zu",
      forest_order.size(), kfs_mechanism_type.size());
    return;
  }

  reset_kfs_auto_collect_tracking();
  kfs_auto_collect_plan_.status = status;
  kfs_auto_collect_plan_.forest_order = std::move(forest_order);
  kfs_auto_collect_plan_.kfs_mechanism_type = std::move(kfs_mechanism_type);

  // if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
  kfs_init_pos();
  // }

  RCLCPP_INFO(
    this->get_logger(), "started kfs auto collect: %s",
    kfs_auto_collect_status_name(kfs_auto_collect_plan_.status).c_str());
}

void R1MainNode::stop_kfs_auto_collect(void)
{
  if (kfs_auto_collect_plan_.status == KfsAutoCollectStatus::NONE) {
    reset_kfs_auto_collect_tracking();
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "stopped kfs auto collect: %s",
    kfs_auto_collect_status_name(kfs_auto_collect_plan_.status).c_str());
  reset_kfs_auto_collect_tracking();
  kfs_auto_collect_plan_.status = KfsAutoCollectStatus::NONE;
  kfs_auto_collect_plan_.forest_order.clear();
  kfs_auto_collect_plan_.kfs_mechanism_type.clear();
}

geometry_msgs::msg::PoseStamped R1MainNode::get_map_pos()
{
  try {
    const auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped map_pos;
    map_pos.header = transform.header;
    map_pos.pose.position.x = transform.transform.translation.x;
    map_pos.pose.position.y = transform.transform.translation.y;
    map_pos.pose.position.z = transform.transform.translation.z;
    map_pos.pose.orientation = transform.transform.rotation;
    return map_pos;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get map position: %s", ex.what());
    geometry_msgs::msg::PoseStamped empty_pose;
    return empty_pose;
  }
}

void R1MainNode::timer_callback(void)
{
  ps4_->update();
  clear_led_status();
  // 状態を更新
  state_machine_->update();
  // タスクを実行
  // ps4_->print_data();
  main_task();
  sabacan_led_update();
}

void R1MainNode::chassis_move_vel(double vx, double vy, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = omega;
  cmd_vel_publisher_->publish(msg);
}

void R1MainNode::kfs_fx_pos_ref(double pos) { publish_position_axis("kfs_fx", pos); }

void R1MainNode::kfs_fz_pos_ref(double pos) { publish_position_axis("kfs_fz", pos); }

void R1MainNode::kfs_fyaw_pos_ref(double pos) { publish_position_axis("kfs_fyaw", pos); }

void R1MainNode::kfs_rx_pos_ref(double pos) { publish_position_axis("kfs_rx", pos); }

void R1MainNode::kfs_rz_pos_ref(double pos) { publish_position_axis("kfs_rz", pos); }

void R1MainNode::kfs_ryaw_pos_ref(double pos) { publish_position_axis("kfs_ryaw", pos); }

void R1MainNode::r2_flift_pos_ref(double pos) { publish_position_axis("r2_flift", pos); }

void R1MainNode::r2_rlift_pos_ref(double pos) { publish_position_axis("r2_rlift", pos); }

void R1MainNode::kfs_fx_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_fx", speed);
}

void R1MainNode::kfs_fz_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_fz", speed);
}

void R1MainNode::kfs_fyaw_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_fyaw", speed);
}

void R1MainNode::kfs_rx_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_rx", speed);
}

void R1MainNode::kfs_rz_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_rz", speed);
}

void R1MainNode::kfs_ryaw_speed_ref(double speed)
{
  publish_position_axis_speed_ref("kfs_ryaw", speed);
}

void R1MainNode::kfs_fx_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_fx"); }

void R1MainNode::kfs_fz_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_fz"); }

void R1MainNode::kfs_fyaw_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_fyaw"); }

void R1MainNode::kfs_rx_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_rx"); }

void R1MainNode::kfs_rz_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_rz"); }

void R1MainNode::kfs_ryaw_speed_mode_stop(void) { stop_position_axis_speed_mode("kfs_ryaw"); }

void R1MainNode::r2_flift_speed_ref(double speed)
{
  publish_position_axis_speed_ref("r2_flift", speed);
}

void R1MainNode::r2_rlift_speed_ref(double speed)
{
  publish_position_axis_speed_ref("r2_rlift", speed);
}

void R1MainNode::r2_flift_speed_mode_stop(void) { stop_position_axis_speed_mode("r2_flift"); }

void R1MainNode::r2_rlift_speed_mode_stop(void) { stop_position_axis_speed_mode("r2_rlift"); }

void R1MainNode::spear1_pos_ref(double pos) { publish_position_axis("spear1", pos); }

void R1MainNode::spear2_pos_ref(double pos) { publish_position_axis("spear2", pos); }

void R1MainNode::spear3_pos_ref(double pos) { publish_position_axis("spear3", pos); }

void R1MainNode::spear4_pos_ref(double pos) { publish_position_axis("spear4", pos); }

void R1MainNode::spear_x_pos_ref(double pos) { publish_position_axis("spear_x", pos); }

void R1MainNode::spear_y_pos_ref(double pos) { publish_position_axis("spear_y", pos); }

void R1MainNode::spear_roll_pos_ref(double angle) { publish_position_axis("spear_roll", angle); }

void R1MainNode::spear_pitch1_pos_ref(double angle)
{
  publish_position_axis("spear_pitch1", angle);
}

void R1MainNode::spear_pitch2_pos_ref(double angle)
{
  publish_position_axis("spear_pitch2", angle);
}

void R1MainNode::spear1_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear1", speed);
}

void R1MainNode::spear2_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear2", speed);
}

void R1MainNode::spear3_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear3", speed);
}

void R1MainNode::spear4_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear4", speed);
}

void R1MainNode::spear_x_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_x", speed);
}

void R1MainNode::spear_y_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_y", speed);
}

void R1MainNode::spear_roll_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_roll", speed);
}

void R1MainNode::spear_pitch1_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_pitch1", speed);
}

void R1MainNode::spear_pitch2_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_pitch2", speed);
}

void R1MainNode::spear1_speed_mode_stop(void) { stop_position_axis_speed_mode("spear1"); }

void R1MainNode::spear2_speed_mode_stop(void) { stop_position_axis_speed_mode("spear2"); }

void R1MainNode::spear3_speed_mode_stop(void) { stop_position_axis_speed_mode("spear3"); }

void R1MainNode::spear4_speed_mode_stop(void) { stop_position_axis_speed_mode("spear4"); }

void R1MainNode::spear_x_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_x"); }

void R1MainNode::spear_y_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_y"); }

void R1MainNode::spear_roll_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_roll"); }

void R1MainNode::spear_pitch1_speed_mode_stop(void)
{
  stop_position_axis_speed_mode("spear_pitch1");
}

void R1MainNode::spear_pitch2_speed_mode_stop(void)
{
  stop_position_axis_speed_mode("spear_pitch2");
}

void R1MainNode::kfs_front_pump(double pwm) { publish_gpio_pwm_output("kfs_front_pump", pwm); }

void R1MainNode::kfs_rear_pump(double pwm) { publish_gpio_pwm_output("kfs_rear_pump", pwm); }

void R1MainNode::kfs_front_valve(bool on)
{
  publish_gpio_pwm_output("kfs_front_valve", on ? 1.0 : 0.0);
}

void R1MainNode::kfs_rear_valve(bool on)
{
  publish_gpio_pwm_output("kfs_rear_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_u1_valve(bool on)
{
  publish_gpio_pwm_output("spear_u1_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_d1_valve(bool on)
{
  publish_gpio_pwm_output("spear_d1_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_u2_valve(bool on)
{
  publish_gpio_pwm_output("spear_u2_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_d2_valve(bool on)
{
  publish_gpio_pwm_output("spear_d2_valve", on ? 1.0 : 0.0);
}

void R1MainNode::kfs_init_pos(void)
{
  // spear_xを動かす
  spear_x_pos_ref(SPEAR_X_MIDDLE_POS);
  // まずrollを動かす
  spear_roll_pos_ref(SPEAR_ROLL_VERTICAL_ANGLE);
  spear1_pos_ref(SPEAR1_KFS_COLLECT_POS);
  spear2_pos_ref(SPEAR2_KFS_COLLECT_POS);
  // 一定時間経過後にKFS回収機構を動かす
  if (kfs_init_pos_roll_timer_) {
    kfs_init_pos_roll_timer_->cancel();
  }
  kfs_init_pos_roll_timer_ = this->create_wall_timer(1000ms, [this]() {
    // デバッグ用にKFS回収用アクチュエータを回収位置位置に移動
    kfs_fx_pos_ref(KFS_FX_START_POS);
    kfs_rx_pos_ref(KFS_RX_START_POS);
    kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
    kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
    if (zone_ == "blue" && chassis_act_status_ == ChassisAct::ACT2) {
      // front
      kfs_fyaw_move_front_mech_lock();
      kfs_ryaw_move_front_mech_lock();
    } else if (zone_ == "blue" && chassis_act_status_ == ChassisAct::ACT3) {
      // rear
      kfs_fyaw_move_rear_mech_lock();
      kfs_ryaw_move_rear_mech_lock();
    } else if (zone_ == "red" && chassis_act_status_ == ChassisAct::ACT2) {
      // rear
      kfs_fyaw_move_rear_mech_lock();
      kfs_ryaw_move_rear_mech_lock();
    } else if (zone_ == "red" && chassis_act_status_ == ChassisAct::ACT3) {
      // front
      kfs_fyaw_move_front_mech_lock();
      kfs_ryaw_move_front_mech_lock();
    }
    kfs_front_pump(0.0);
    kfs_rear_pump(0.0);
    if (kfs_init_pos_roll_timer_) {
      kfs_init_pos_roll_timer_->cancel();
    }
  });
}

void R1MainNode::stop_actuator(void)
{
  // 速度制御のモータ指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
  r2_flift_speed_ref(0.0);
  r2_rlift_speed_ref(0.0);
  r2_flift_speed_mode_stop();
  r2_rlift_speed_mode_stop();
  // 真空ポンプを止める
  kfs_front_pump(0.0);
  kfs_rear_pump(0.0);
  // KFS回収電磁弁を止める
  kfs_front_valve(false);
  kfs_rear_valve(false);
  // やりの電磁弁を止める
  spear_u1_valve(false);
  spear_d1_valve(false);
  spear_u2_valve(false);
  spear_d2_valve(false);
}

// --- 各状態のタスク ---
void R1MainNode::idle_task(void)
{
  // 速度指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
}

void R1MainNode::emergency_task(void) {}

void R1MainNode::manual_mode1_detect_origin(void)
{
  if (ps4_->is_pushed_up()) {
    kfs_fx_detect_origin();
  }

  if (ps4_->is_pushed_right()) {
    kfs_fz_detect_origin();
  }

  if (ps4_->is_pushed_down()) {
    kfs_fyaw_detect_origin();
  }

  // if (ps4_->is_pushed_left()) {
  //   front_expand_detect_origin();
  // }

  if (ps4_->is_pushed_triangle()) {
    kfs_rx_detect_origin();
  }

  if (ps4_->is_pushed_circle()) {
    kfs_rz_detect_origin();
  }

  if (ps4_->is_pushed_cross()) {
    kfs_ryaw_detect_origin();
  }

  // if (ps4_->is_pushed_square()) {
  //   rear_expand_detect_origin();
  // }

  if (ps4_->is_pushed_l1()) {
    spear1_detect_origin();
  }

  if (ps4_->is_pushed_r1()) {
    spear2_detect_origin();
  }

  if (ps4_->is_pushed_l2()) {
    r2_rlift_detect_origin();
  }

  if (ps4_->is_pushed_r2()) {
    r2_flift_detect_origin();
  }
}

void R1MainNode::manual_mode2_collect_pole_task(void)
{
  int & step = manual_mode2_collect_pole_task_step_;
  RCLCPP_INFO(this->get_logger(), "manual_mode2_collect_pole_task step: %d", step);
  if (step == 1) {
    spear_roll_pos_ref(SPEAR_ROLL_VERTICAL_ANGLE);
    spear_x_pos_ref(SPEAR_X_MIDDLE_POS);
    spear_pitch1_pos_ref(SPEAR_PITCH1_VERTICAL_ANGLE);
    spear_pitch2_pos_ref(SPEAR_PITCH2_VERTICAL_ANGLE);
    spear_y_pos_ref(SPEAR_Y_EXPAND_POS);
    spear_u1_valve(true);
    spear_d1_valve(true);
    spear_u2_valve(true);
    spear_d2_valve(true);
    spear1_pos_ref(SPEAR1_COLLECT1_POS);
    spear2_pos_ref(SPEAR2_COLLECT1_POS);
    step++;
  } else if (step == 2) {
    spear_d1_valve(false);
    spear_d2_valve(false);
    step++;
    // } else if (step == 3) {
    //   spear1_pos_ref(SPEAR1_COLLECT2_POS);
    //   spear2_pos_ref(SPEAR2_COLLECT2_POS);
    //   step++;
  } else if (step == 3) {
    spear1_pos_ref(SPEAR1_COLLECT3_POS);
    spear2_pos_ref(SPEAR2_COLLECT3_POS);
    step++;
  } else if (step == 4) {
    spear_u1_valve(false);
    spear_u2_valve(false);
    spear_y_pos_ref(SPEAR_Y_NORMAL_POS);
    step++;
  } else if (step == 5) {
    spear_roll_pos_ref(SPEAR_ROLL_NORMAL_ANGLE);
    step++;
  } else if (step == 6) {
    spear_d1_valve(true);
    spear_d2_valve(true);
    step++;
  } else if (step == 7) {
    spear1_pos_ref(SPEAR1_NORMAL_POS);
    spear2_pos_ref(SPEAR2_NORMAL_POS);
    spear_pitch1_pos_ref(SPEAR_PITCH1_NORMAL_ANGLE);
    spear_pitch2_pos_ref(SPEAR_PITCH2_NORMAL_ANGLE);
    step++;
  } else if (step == 8) {
    spear_d1_valve(false);
    spear_d2_valve(false);
    RCLCPP_INFO(this->get_logger(), "pole collect task completed");
    step = 1;
  }
}

void R1MainNode::manual_mode2_pole(void)
{
  if (ps4_->is_pushed_up()) {
    spear_roll_pos_ref(spear_roll_position_ref_ + 0.025);
  }

  if (ps4_->is_pushed_right()) {
    spear_y_pos_ref(spear_y_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_down()) {
    spear_roll_pos_ref(spear_roll_position_ref_ - 0.025);
  }

  if (ps4_->is_pushed_left()) {
    spear_y_pos_ref(spear_y_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_triangle()) {
    // ポール回収
    manual_mode2_collect_pole_task();
  }

  if (ps4_->is_pushed_circle()) {
    spear2_pos_ref(spear2_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_cross()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushed_square()) {
    spear2_pos_ref(spear2_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_l1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ + 0.05);
  }

  if (ps4_->is_pushed_l2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ + 0.05);
  }
}

void R1MainNode::manual_mode3_make_spear_task(int n)
{
  int & step = manual_mode3_make_spear_task_step_;
  RCLCPP_INFO(this->get_logger(), "manual_mode3_make_spear_task step: %d", step);
  if (step == 1) {
    if (n == 1) {
      spear_x_pos_ref(SPEAR_X_MAKE_SPEAR1_POS);
    } else if (n == 2) {
      spear_x_pos_ref(SPEAR_X_MAKE_SPEAR2_POS);
    } else if (n == 3) {
      spear_x_pos_ref(SPEAR_X_MAKE_SPEAR3_POS);
    } else if (n == 4) {
      spear_x_pos_ref(SPEAR_X_MAKE_SPEAR4_POS);
    }
    // rollを逆横向きにする
    // TODO: ゾーンによって向きを変える
    // spear_roll_pos_ref(SPEAR_ROLL_NORMAL_ANGLE);
    spear_roll_pos_ref(SPEAR_ROLL_INV_NORMAL_ANGLE);
    spear_pitch1_pos_ref(SPEAR_PITCH1_VERTICAL_ANGLE);
    spear_pitch2_pos_ref(SPEAR_PITCH2_VERTICAL_ANGLE);
    step++;
  } else if (step == 2) {
    if (n == 1) {
      spear1_pos_ref(SPEAR1_MAKE_SPEAR_START_POS);
    } else if (n == 2) {
      spear2_pos_ref(SPEAR2_MAKE_SPEAR_START_POS);
    } else if (n == 3) {
      spear3_pos_ref(SPEAR3_MAKE_SPEAR_START_POS);
    } else if (n == 4) {
      spear4_pos_ref(SPEAR4_MAKE_SPEAR_START_POS);
    }
    step++;
  } else if (step == 3) {
    // 押し込む
    if (n == 1) {
      spear1_speed_ref(SPEAR1_PUSH_VEL);
    } else if (n == 2) {
      spear2_speed_ref(SPEAR2_PUSH_VEL);
    } else if (n == 3) {
      spear3_speed_ref(SPEAR3_PUSH_VEL);
    } else if (n == 4) {
      spear4_speed_ref(SPEAR4_PUSH_VEL);
    }
    step++;
  } else if (step == 4) {
    // 現在位置で停止
    if (n == 1) {
      spear1_speed_mode_stop();
    } else if (n == 2) {
      spear2_speed_mode_stop();
    } else if (n == 3) {
      spear3_speed_mode_stop();
    } else if (n == 4) {
      spear4_speed_mode_stop();
    }
    step++;
  } else if (step == 5) {
    // 位置を戻す
    if (n == 1) {
      spear1_pos_ref(SPEAR1_NORMAL_POS);
    } else if (n == 2) {
      spear2_pos_ref(SPEAR2_NORMAL_POS);
    } else if (n == 3) {
      spear3_pos_ref(SPEAR3_NORMAL_POS);
    } else if (n == 4) {
      spear4_pos_ref(SPEAR4_NORMAL_POS);
    }
    step++;
  } else if (step == 6) {
    spear_roll_pos_ref(SPEAR_ROLL_VERTICAL_ANGLE);
    spear_y_pos_ref(SPEAR_Y_NORMAL_POS);
    RCLCPP_INFO(this->get_logger(), "make spear task completed");
    step = 1;
  }
}

void R1MainNode::manual_mode3_spear(void)
{
  if (ps4_->is_pushed_up()) {
    spear_roll_pos_ref(spear_roll_position_ref_ + 0.025);
  }

  if (ps4_->is_pushed_right()) {
    spear_x_pos_ref(spear_x_position_ref_ + 0.0025);
  }

  if (ps4_->is_pushed_down()) {
    spear_roll_pos_ref(spear_roll_position_ref_ - 0.025);
  }

  if (ps4_->is_pushed_left()) {
    spear_x_pos_ref(spear_x_position_ref_ - 0.0025);
  }

  if (ps4_->is_pushed_triangle()) {
    // やり組み立て
    manual_mode3_make_spear_task(2);
  }

  if (ps4_->is_pushed_circle()) {
    spear2_pos_ref(spear2_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_cross()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushed_square()) {
    spear2_pos_ref(spear2_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_l1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ + 0.05);
  }

  if (ps4_->is_pushed_l2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ + 0.05);
  }
}

void R1MainNode::manual_mode4_fkfs(void)
{
  int & fx_step = manual_mode4_fx_step_;
  int & fz_step = manual_mode4_fz_step_;
  int & fyaw_step = manual_mode4_fyaw_step_;
  int & front_pump_step = manual_mode4_front_pump_step_;

  if (ps4_->is_pushed_up()) {
    // 1段上のkfs_fz位置へ移動
    fz_step++;
    if (fz_step > 4) {
      fz_step = 4;
    }
    RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
    if (fz_step == 1) {
      kfs_fz_pos_ref(KFS_FZ_LOW_POS);
    } else if (fz_step == 2) {
      kfs_fz_pos_ref(KFS_FZ_MIDDLE_POS);
    } else if (fz_step == 3) {
      kfs_fz_pos_ref(KFS_FZ_HIGH_POS);
    } else if (fz_step == 4) {
      kfs_fz_pos_ref(KFS_FZ_PUT_POS);
    }
  }

  if (ps4_->is_pushed_right()) {
    kfs_fx_pos_ref(KFS_FX_PUT_POS);
    kfs_fz_pos_ref(KFS_FZ_PUT_POS);
    kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
    kfs_rz_pos_ref(KFS_RZ_PUT_POS);
    RCLCPP_INFO(this->get_logger(), "moved to front_kfs put position");
  }

  if (ps4_->is_pushed_down()) {
    // 1段下のkfs_fz位置へ移動
    fz_step--;
    if (fz_step < 1) {
      fz_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
    if (fz_step == 1) {
      kfs_fz_pos_ref(KFS_FZ_LOW_POS);
    } else if (fz_step == 2) {
      kfs_fz_pos_ref(KFS_FZ_MIDDLE_POS);
    } else if (fz_step == 3) {
      kfs_fz_pos_ref(KFS_FZ_HIGH_POS);
    }
  }

  if (ps4_->is_pushed_left()) {
    // front_pumpを動かす。止めるときは電磁弁も一緒に動く
    if (front_pump_step == 1) {
      kfs_front_pump(1.0);
      kfs_front_valve(false);
      front_pump_step = 2;
    } else {
      kfs_front_pump(0.0);
      kfs_front_valve(true);
      // setTimeout風で電磁弁をOFFにする。
      manual_mode4_front_valve_timer_ = this->create_wall_timer(250ms, [this]() {
        kfs_front_valve(false);
        manual_mode4_front_valve_timer_->cancel();
      });
      front_pump_step = 1;
    }
  }

  if (ps4_->is_pushed_r1()) {
    manual_mode4_r1_long_press_triger_ = true;
  }

  if (ps4_->is_pushing_r1() && ps4_->is_pushed_triangle()) {
    // R1+△: kfs_fyawの微調整（指令値を増加）
    manual_mode4_r1_long_press_triger_ = false;
    kfs_fyaw_pos_ref(kfs_fyaw_position_ref_ + 0.05);
  } else if (ps4_->is_pushed_triangle()) {
    // △単押し: kfs_fyawを90度進める
    fyaw_step++;
    if (fyaw_step > 3) {
      fyaw_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
    if (fyaw_step == 1) {
      kfs_fyaw_move_rear_mech_lock();
    } else if (fyaw_step == 2) {
      kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
    } else if (fyaw_step == 3) {
      kfs_fyaw_move_front_mech_lock();
    }
  }

  if (ps4_->is_pushed_circle()) {
    fx_step++;
    if (fx_step > 4) {
      fx_step = 4;
    }
    RCLCPP_INFO(this->get_logger(), "fx_step: %d", fx_step);
    if (fx_step == 1) {
      kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
    } else if (fx_step == 2) {
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
    } else if (fx_step == 3) {
      kfs_fx_pos_ref(KFS_FX_PUT_POS);
    } else if (fx_step == 4) {
      kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
    }
  }

  if (ps4_->is_pushing_r1() && ps4_->is_pushed_cross()) {
    // R1+×: kfs_fyawの微調整（指令値を減少）
    manual_mode4_r1_long_press_triger_ = false;
    kfs_fyaw_pos_ref(kfs_fyaw_position_ref_ - 0.05);
  } else if (ps4_->is_pushed_cross()) {
    // ×単押し: kfs_fyawを90度戻す
    fyaw_step--;
    if (fyaw_step < 1) {
      fyaw_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
    if (fyaw_step == 1) {
      kfs_fyaw_move_rear_mech_lock();
    } else if (fyaw_step == 2) {
      kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
    } else if (fyaw_step == 3) {
      kfs_fyaw_move_front_mech_lock();
    }
  }

  if (ps4_->is_pushed_square()) {
    fx_step--;
    if (fx_step < 1) {
      fx_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "fx_step: %d", fx_step);
    if (fx_step == 1) {
      kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
    } else if (fx_step == 2) {
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
    } else if (fx_step == 3) {
      kfs_fx_pos_ref(KFS_FX_PUT_POS);
    } else if (fx_step == 4) {
      kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
    }
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_fxの微調整（指令値を減少）
    // メカロックが有効かつ次の指令値がメカロックしきい値を下回る場合は、メカロックにぶつける。
    double next_ref = kfs_fx_position_ref_ - 0.01;
    if (USE_KFS_MECH_LOCK && next_ref < KFS_FX_LOW_MECH_LOCK_POS) {
      kfs_fx_position_ref_ = KFS_FX_LOW_MECH_LOCK_POS;
      kfs_fx_move_mech_lock(-1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_fx low mech lock");
    } else {
      kfs_fx_pos_ref(kfs_fx_position_ref_ - 0.01);
    }
  }

  if (ps4_->is_released_r1() && manual_mode4_r1_long_press_triger_) {
    // R1単押し（コンボなし確定）: kfs_fxの微調整（指令値を増加）
    manual_mode4_r1_long_press_triger_ = false;
    double next_ref = kfs_fx_position_ref_ + 0.01;
    if (USE_KFS_MECH_LOCK && next_ref > KFS_FX_HIGH_MECH_LOCK_POS) {
      kfs_fx_position_ref_ = KFS_FX_HIGH_MECH_LOCK_POS;
      kfs_fx_move_mech_lock(1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_fx high mech lock");
    } else {
      kfs_fx_pos_ref(kfs_fx_position_ref_ + 0.01);
    }
  }

  if (ps4_->is_pushed_l2()) {
    // kfs_fzの微調整（指令値を減少）
    // メカロックが有効かつ次の指令値がメカロックしきい値を下回る場合は、メカロックにぶつける。
    double next_ref = kfs_fz_position_ref_ - 0.01;
    if (USE_KFS_MECH_LOCK && next_ref < KFS_FZ_LOW_MECH_LOCK_POS) {
      kfs_fz_position_ref_ = KFS_FZ_LOW_MECH_LOCK_POS;
      kfs_fz_move_mech_lock(-1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_fz low mech lock");
    } else {
      kfs_fz_pos_ref(kfs_fz_position_ref_ - 0.01);
    }
  }

  if (ps4_->is_pushed_r2()) {
    // kfs_fzの微調整（指令値を増加）
    // メカロックが有効かつ次の指令値がメカロックしきい値を上回る場合は、メカロックにぶつける。
    double next_ref = kfs_fz_position_ref_ + 0.01;
    if (USE_KFS_MECH_LOCK && next_ref > KFS_FZ_HIGH_MECH_LOCK_POS) {
      kfs_fz_position_ref_ = KFS_FZ_HIGH_MECH_LOCK_POS;
      kfs_fz_move_mech_lock(1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_fz high mech lock");
    } else {
      kfs_fz_pos_ref(kfs_fz_position_ref_ + 0.01);
    }
  }
}

void R1MainNode::manual_mode5_rkfs(void)
{
  int & rx_step = manual_mode5_rx_step_;
  int & rz_step = manual_mode5_rz_step_;
  int & ryaw_step = manual_mode5_ryaw_step_;
  int & rear_pump_step = manual_mode5_rear_pump_step_;

  if (ps4_->is_pushed_up()) {
    // 1段上のkfs_rz位置へ移動
    rz_step++;
    if (rz_step > 4) {
      rz_step = 4;
    }
    RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
    if (rz_step == 1) {
      kfs_rz_pos_ref(KFS_RZ_LOW_POS);
    } else if (rz_step == 2) {
      kfs_rz_pos_ref(KFS_RZ_MIDDLE_POS);
    } else if (rz_step == 3) {
      kfs_rz_pos_ref(KFS_RZ_HIGH_POS);
    } else if (rz_step == 4) {
      kfs_rz_pos_ref(KFS_RZ_PUT_POS);
    }
  }

  if (ps4_->is_pushed_right()) {
    RCLCPP_INFO(this->get_logger(), "moved to rear_kfs put position");
    kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
    kfs_fz_pos_ref(KFS_FZ_PUT_POS);
    kfs_rx_pos_ref(KFS_RX_PUT_POS);
    kfs_rz_pos_ref(KFS_RZ_PUT_POS);
  }

  if (ps4_->is_pushed_down()) {
    // 1段下のkfs_rz位置へ移動
    rz_step--;
    if (rz_step < 1) {
      rz_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
    if (rz_step == 1) {
      kfs_rz_pos_ref(KFS_RZ_LOW_POS);
    } else if (rz_step == 2) {
      kfs_rz_pos_ref(KFS_RZ_MIDDLE_POS);
    } else if (rz_step == 3) {
      kfs_rz_pos_ref(KFS_RZ_HIGH_POS);
    }
  }

  if (ps4_->is_pushed_left()) {
    // rear_pumpを動かす。止めるときは電磁弁も一緒に動く
    if (rear_pump_step == 1) {
      kfs_rear_pump(1.0);
      kfs_rear_valve(false);
      rear_pump_step = 2;
    } else {
      kfs_rear_pump(0.0);
      kfs_rear_valve(true);
      // setTimeout風で電磁弁をOFFにする。
      manual_mode5_rear_valve_timer_ = this->create_wall_timer(250ms, [this]() {
        kfs_rear_valve(false);
        manual_mode5_rear_valve_timer_->cancel();
      });
      rear_pump_step = 1;
    }
  }

  if (ps4_->is_pushed_r1()) {
    manual_mode5_r1_long_press_triger_ = true;
  }

  if (ps4_->is_pushing_r1() && ps4_->is_pushed_triangle()) {
    // R1+△: kfs_ryawの微調整（指令値を増加）
    manual_mode5_r1_long_press_triger_ = false;
    kfs_ryaw_pos_ref(kfs_ryaw_position_ref_ + 0.05);
  } else if (ps4_->is_pushed_triangle()) {
    // △単押し: kfs_ryawを90度進める
    ryaw_step++;
    if (ryaw_step > 3) {
      ryaw_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
    if (ryaw_step == 1) {
      kfs_ryaw_move_rear_mech_lock();
    } else if (ryaw_step == 2) {
      kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
    } else if (ryaw_step == 3) {
      kfs_ryaw_move_front_mech_lock();
    }
  }

  if (ps4_->is_pushed_circle()) {
    rx_step++;
    if (rx_step > 4) {
      rx_step = 4;
    }
    RCLCPP_INFO(this->get_logger(), "rx_step: %d", rx_step);
    if (rx_step == 1) {
      kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
    } else if (rx_step == 2) {
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
    } else if (rx_step == 3) {
      kfs_rx_pos_ref(KFS_RX_PUT_POS);
    } else if (rx_step == 4) {
      kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
    }
  }

  if (ps4_->is_pushing_r1() && ps4_->is_pushed_cross()) {
    // R1+×: kfs_ryawの微調整（指令値を減少）
    manual_mode5_r1_long_press_triger_ = false;
    kfs_ryaw_pos_ref(kfs_ryaw_position_ref_ - 0.05);
  } else if (ps4_->is_pushed_cross()) {
    // ×単押し: kfs_ryawを90度戻す
    ryaw_step--;
    if (ryaw_step < 1) {
      ryaw_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
    if (ryaw_step == 1) {
      kfs_ryaw_move_rear_mech_lock();
    } else if (ryaw_step == 2) {
      kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
    } else if (ryaw_step == 3) {
      kfs_ryaw_move_front_mech_lock();
    }
  }

  if (ps4_->is_pushed_square()) {
    rx_step--;
    if (rx_step < 1) {
      rx_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "rx_step: %d", rx_step);
    if (rx_step == 1) {
      kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
    } else if (rx_step == 2) {
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
    } else if (rx_step == 3) {
      kfs_rx_pos_ref(KFS_RX_PUT_POS);
    } else if (rx_step == 4) {
      kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
    }
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_rxの微調整（指令値を減少）
    // メカロックが有効かつ次の指令値がメカロックしきい値を下回る場合は、メカロックにぶつける。
    double next_ref = kfs_rx_position_ref_ - 0.01;
    if (USE_KFS_MECH_LOCK && next_ref < KFS_RX_LOW_MECH_LOCK_POS) {
      kfs_rx_position_ref_ = KFS_RX_LOW_MECH_LOCK_POS;
      kfs_rx_move_mech_lock(-1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_rx low mech lock");
    } else {
      kfs_rx_pos_ref(kfs_rx_position_ref_ - 0.01);
    }
  }

  if (ps4_->is_released_r1() && manual_mode5_r1_long_press_triger_) {
    // R1単押し（コンボなし確定）: kfs_rxの微調整（指令値を増加）
    manual_mode5_r1_long_press_triger_ = false;
    double next_ref = kfs_rx_position_ref_ + 0.01;
    if (USE_KFS_MECH_LOCK && next_ref > KFS_RX_HIGH_MECH_LOCK_POS) {
      kfs_rx_position_ref_ = KFS_RX_HIGH_MECH_LOCK_POS;
      kfs_rx_move_mech_lock(1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_rx high mech lock");
    } else {
      kfs_rx_pos_ref(kfs_rx_position_ref_ + 0.01);
    }
  }

  if (ps4_->is_pushed_l2()) {
    // kfs_rzの微調整（指令値を減少）
    // メカロックが有効かつ次の指令値がメカロックしきい値を下回る場合は、メカロックにぶつける。
    double next_ref = kfs_rz_position_ref_ - 0.01;
    if (USE_KFS_MECH_LOCK && next_ref < KFS_RZ_LOW_MECH_LOCK_POS) {
      kfs_rz_position_ref_ = KFS_RZ_LOW_MECH_LOCK_POS;
      kfs_rz_move_mech_lock(-1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_rz mech lock");
    } else {
      kfs_rz_pos_ref(kfs_rz_position_ref_ - 0.01);
    }
  }

  if (ps4_->is_pushed_r2()) {
    // kfs_rzの微調整（指令値を増加）
    // メカロックが有効かつ次の指令値がメカロックしきい値を上回る場合は、メカロックにぶつける。
    double next_ref = kfs_rz_position_ref_ + 0.01;
    if (USE_KFS_MECH_LOCK && next_ref > KFS_RZ_HIGH_MECH_LOCK_POS) {
      kfs_rz_position_ref_ = KFS_RZ_HIGH_MECH_LOCK_POS;
      kfs_rz_move_mech_lock(1);
      RCLCPP_INFO(this->get_logger(), "moved kfs_rz mech lock");
    } else {
      kfs_rz_pos_ref(kfs_rz_position_ref_ + 0.01);
    }
  }
}

void R1MainNode::manual_mode6_r2_lift(void)
{
  if (ps4_->is_pushed_up()) {
  }

  if (ps4_->is_pushed_right()) {
  }

  if (ps4_->is_pushed_down()) {
  }

  if (ps4_->is_pushed_left()) {
  }

  if (ps4_->is_pushed_triangle()) {
    r2_flift_pos_ref(R2_FLIFT_UP_POS);
    r2_rlift_pos_ref(R2_RLIFT_UP_POS);
  }

  if (ps4_->is_pushed_circle()) {
    kfs_fx_pos_ref(KFS_FX_R2_LIFT_POS);
    kfs_fz_pos_ref(KFS_FZ_R2_LIFT_POS);
    kfs_rx_pos_ref(KFS_RX_R2_LIFT_POS);
    kfs_rz_pos_ref(KFS_RZ_R2_LIFT_POS);
    RCLCPP_INFO(this->get_logger(), "moved to r2_lift position");
    manual_mode6_r2_lift_timer_ = this->create_wall_timer(500ms, [this]() {
      r2_flift_move_mech_lock(-1);
      r2_rlift_move_mech_lock(-1);
      manual_mode6_r2_lift_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "r2 lift stop (timer)");
    });
  }

  if (ps4_->is_pushed_cross()) {
    r2_flift_pos_ref(R2_FLIFT_DOWN_POS);
    r2_rlift_pos_ref(R2_RLIFT_DOWN_POS);
  }

  if (ps4_->is_pushed_l1()) {
    r2_flift_pos_ref(r2_flift_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    r2_flift_pos_ref(r2_flift_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    r2_rlift_pos_ref(r2_rlift_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r2()) {
    r2_rlift_pos_ref(r2_rlift_position_ref_ + 0.01);
  }
}

/**
 * @brief
 *
 * @param n 何個目の機構を動かすか。
 * @param m どの高さを狙うか。m==1で下段、m==2で中段、m==3で上段を狙う
 */
void R1MainNode::manual_mode7_spear_attack_task(int n, int m)
{
  (void)n;
  int & step = manual_mode7_spear_attack_task_step_;
  RCLCPP_INFO(this->get_logger(), "manual_mode7_spear_attack_task step: %d", step);
  if (step == 1) {
    kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
    kfs_fz_pos_ref(KFS_FZ_NORMAL_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_NORMAL_ANGLE);
    kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
    kfs_rz_pos_ref(KFS_RZ_NORMAL_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_NORMAL_ANGLE);
    step++;
  } else if (step == 2) {
    spear_x_pos_ref(SPEAR_X_NORMAL_POS);
    spear_y_pos_ref(SPEAR_Y_NORMAL_POS);
    spear1_pos_ref(SPEAR1_NORMAL_POS);
    spear2_pos_ref(SPEAR2_NORMAL_POS);
    if (m == 1) {
      // 下段を狙う
      spear_roll_pos_ref(SPEAR_ROLL_LOW_ATTACK_ANGLE);
    } else if (m == 2) {
      // 中段を狙う
      spear_roll_pos_ref(SPEAR_ROLL_MIDDLE_ATTACK_ANGLE);
    } else if (m == 3) {
      // 上段を狙う
      spear_roll_pos_ref(SPEAR_ROLL_HIGH_ATTACK_ANGLE);
    }
    step++;
  } else if (step == 3) {
    if (n == 1) {
      if (m == 1) {
        // 下段を狙う
        spear1_pos_ref(SPEAR1_LOW_ATTACK_POS);
      } else if (m == 2) {
        // 中段を狙う
        spear1_pos_ref(SPEAR1_MIDDLE_ATTACK_POS);
      } else if (m == 3) {
        // 上段を狙う
        spear1_pos_ref(SPEAR1_HIGH_ATTACK_POS);
      }
    } else if (n == 2) {
      if (m == 1) {
        // 下段を狙う
        spear2_pos_ref(SPEAR2_LOW_ATTACK_POS);
      } else if (m == 2) {
        // 中段を狙う
        spear2_pos_ref(SPEAR2_MIDDLE_ATTACK_POS);
      } else if (m == 3) {
        // 上段を狙う
        spear2_pos_ref(SPEAR2_HIGH_ATTACK_POS);
      }
    } else if (n == 3) {
    } else if (n == 4) {
    }
    step++;
  } else if (step == 4) {
    if (n == 1) {
      spear1_pos_ref(SPEAR1_NORMAL_POS);
    } else if (n == 2) {
      spear2_pos_ref(SPEAR2_NORMAL_POS);
    } else if (n == 3) {
      spear3_pos_ref(SPEAR3_NORMAL_POS);
    } else if (n == 4) {
      spear4_pos_ref(SPEAR4_NORMAL_POS);
    }
    spear_roll_pos_ref(SPEAR_ROLL_NORMAL_ANGLE);
    step++;
  } else if (step == 5) {
    if (n == 1) {
      spear_u1_valve(true);
      spear_d1_valve(true);
    } else if (n == 2) {
      spear_u2_valve(true);
      spear_d2_valve(true);
    } else if (n == 3) {
    } else if (n == 4) {
    }
    step++;
  } else if (step == 6) {
    if (n == 1) {
      spear_u1_valve(false);
      spear_d1_valve(false);
    } else if (n == 2) {
      spear_u2_valve(false);
      spear_d2_valve(false);
    } else if (n == 3) {
    } else if (n == 4) {
    }
    spear_roll_pos_ref(SPEAR_ROLL_VERTICAL_ANGLE);
    step = 1;
    RCLCPP_INFO(this->get_logger(), "spear attack task completed");
  }
}

void R1MainNode::manual_mode7_spear_attack(void)
{
  if (ps4_->is_pushed_up()) {
    spear_roll_pos_ref(spear_roll_position_ref_ + 0.05);
  }

  if (ps4_->is_pushed_right()) {
    manual_mode7_spear_attack_task(2, 2);
  }

  if (ps4_->is_pushed_down()) {
    spear_roll_pos_ref(spear_roll_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_left()) {
    manual_mode7_spear_attack_task(2, 3);
  }

  if (ps4_->is_pushed_triangle()) {
    manual_mode7_spear_attack_task(2, 1);
  }

  if (ps4_->is_pushed_circle()) {
    spear2_pos_ref(spear2_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_cross()) {
  }

  if (ps4_->is_pushed_square()) {
    spear2_pos_ref(spear2_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_l1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r1()) {
    spear_pitch1_pos_ref(spear_pitch1_position_ref_ + 0.05);
  }

  if (ps4_->is_pushed_l2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ - 0.05);
  }

  if (ps4_->is_pushed_r2()) {
    spear_pitch2_pos_ref(spear_pitch2_position_ref_ + 0.05);
  }
}

void R1MainNode::auto_collect_kfs_task(void)
{
  constexpr int FKFS = 0;
  constexpr int RKFS = 1;

  if (kfs_auto_collect_plan_.status == KfsAutoCollectStatus::NONE) {
    return;
  }
  if (!is_localization_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for localization before running kfs auto collect.");
    return;
  }

  const bool is_inner = (kfs_auto_collect_plan_.status == KfsAutoCollectStatus::INNER_ACTIVE);
  if (
    kfs_auto_collect_plan_.forest_order.size() !=
    kfs_auto_collect_plan_.kfs_mechanism_type.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "kfs auto collect plan size mismatch during %s: forest_order=%zu, kfs_mechanism_type=%zu",
      kfs_auto_collect_status_name(kfs_auto_collect_plan_.status).c_str(),
      kfs_auto_collect_plan_.forest_order.size(), kfs_auto_collect_plan_.kfs_mechanism_type.size());
    return;
  }
  // TODO: 進行方向と使用する回収機構の順番に応じて、OFFSETをいい感じに適応する
  geometry_msgs::msg::PoseStamped map_pos = get_map_pos();
  int n = kfs_auto_collect_plan_.forest_order.size();
  bool front_kfs_assigned = false;
  bool rear_kfs_assigned = false;
  bool front_kfs_within = false;
  bool rear_kfs_within = false;
  for (int i = 0; i < n; i++) {
    int target_forest_number = kfs_auto_collect_plan_.forest_order[i];
    if (target_forest_number < 1 || target_forest_number > 12) {
      RCLCPP_ERROR(
        this->get_logger(), "invalid forest number in robot_move: %d", target_forest_number);
      continue;
    }
    const std::string & mechanism_type = kfs_auto_collect_plan_.kfs_mechanism_type[i];
    if (mechanism_type != "front_kfs" && mechanism_type != "rear_kfs") {
      RCLCPP_ERROR(
        this->get_logger(), "invalid kfs_mechanism_type in robot_move: %s", mechanism_type.c_str());
      continue;
    }
    double map_x = map_pos.pose.position.x;
    double map_y = map_pos.pose.position.y;
    double center_x = 0.0, center_y = 0.0, rect_yaw = 0.0, offset_x = 0.0, offset_y = 0.0;

    // within関連はメンバー変数。名前が長いので、参照として短い名前で扱う。
    int within_index = (mechanism_type == "front_kfs") ? FKFS : RKFS;
    front_kfs_assigned |= (within_index == FKFS);
    rear_kfs_assigned |= (within_index == RKFS);
    std::vector<bool>::reference within =
      kfs_auto_collect_within_[target_forest_number - 1][within_index];
    std::vector<bool>::reference prev_within =
      kfs_auto_collect_prev_within_[target_forest_number - 1][within_index];
    // within はこの周期の判定結果なので、毎周期いったん false に戻して再評価する。
    within = false;

    if (is_inner) {
      center_x = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][0];
      center_y = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][1];
      rect_yaw = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][2];
    } else {
      center_x = OUTER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][0];
      center_y = OUTER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][1];
      rect_yaw = OUTER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][2];
    }
    // 青ゾーンのときは角度を反転させる
    if (zone_ == "blue") {
      center_x *= -1.0;
      rect_yaw = angle_normalize(M_PI - rect_yaw);
    }
    // TODO: ココらへんの処理はかなり怪しいので、赤ゾーンに対応するときに見直す。おそらく角度の扱いが怪しい
    // yは進行方向と同じ向きに対してオフセットを適用する
    if (is_inner && mechanism_type == "rear_kfs") {
      offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
      offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
    } else if (!is_inner && mechanism_type == "front_kfs") {
      offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
      offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
    }

    // center_xとcenter_yにオフセットを適用する
    if (zone_ == "red") {
      center_x += offset_x;
      center_y += offset_y;
    } else {
      // 本当はcenter_xはプラスではなくマイナスのはずだが、何故か動かないので一旦プラス
      center_x += offset_x;
      center_y += offset_y;
    }
    if (
      is_within_rotated_rectangle(
        map_x, map_y, center_x, center_y, rect_yaw, COLLECT_KFS_WIDTH, COLLECT_KFS_HEIGHT)) {
      within = true;
    }
    if (within_index == FKFS) {
      front_kfs_within = front_kfs_within || within;
    } else {
      rear_kfs_within = rear_kfs_within || within;
    }

    if (within == false) {
      // trueからfalseに変わったら、収納動作を行う。
      if (prev_within == true) {
        if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
          // 収納位置に移動
          if (within_index == FKFS) {
            kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
            kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
            if (auto_collect_front_storage_yaw_timer_) {
              auto_collect_front_storage_yaw_timer_->cancel();
            }
            auto_collect_front_storage_yaw_timer_ =
              this->create_wall_timer(std::chrono::duration<double>(KFS_YAW_DELAY_TIME), [this]() {
                kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
                auto_collect_front_storage_yaw_timer_->cancel();
              });
          } else {
            kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
            kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
            if (auto_collect_rear_storage_yaw_timer_) {
              auto_collect_rear_storage_yaw_timer_->cancel();
            }
            auto_collect_rear_storage_yaw_timer_ =
              this->create_wall_timer(std::chrono::duration<double>(KFS_YAW_DELAY_TIME), [this]() {
                kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
                auto_collect_rear_storage_yaw_timer_->cancel();
              });
          }
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "%d forest %s kfs storage skipped because enable_auto_collect_kfs_actuator=false",
            target_forest_number, kfs_auto_collect_plan_.kfs_mechanism_type[i].c_str());
        }
        if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
          RCLCPP_INFO(
            this->get_logger(), "%d forest %s kfs storage", target_forest_number,
            kfs_auto_collect_plan_.kfs_mechanism_type[i].c_str());
        }
      }
    } else {
      // falseからtrueに変わったら、回収動作を行う。
      if (prev_within == false) {
        if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
          // 回収位置に移動
          if (within_index == FKFS) {
            // 回収機構を動かす
            kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
            if (zone_ == "blue" && is_inner) {
              kfs_fyaw_move_front_mech_lock();
            } else if (zone_ == "blue" && !is_inner) {
              kfs_fyaw_move_rear_mech_lock();
            } else if (zone_ == "red" && is_inner) {
              kfs_fyaw_move_rear_mech_lock();
            } else if (zone_ == "red" && !is_inner) {
              kfs_fyaw_move_front_mech_lock();
            }
            kfs_front_pump(1.0);
            kfs_front_valve(false);
            if (
              target_forest_number == 2 || target_forest_number == 4 ||
              target_forest_number == 10 || target_forest_number == 12) {
              kfs_fz_pos_ref(KFS_FZ_LOW_POS);
            } else if (
              target_forest_number == 1 || target_forest_number == 3 || target_forest_number == 7 ||
              target_forest_number == 9 || target_forest_number == 11) {
              kfs_fz_pos_ref(KFS_FZ_MIDDLE_POS);
            } else if (target_forest_number == 6) {
              kfs_fz_pos_ref(KFS_FZ_HIGH_POS);
            }
          } else {
            // 回収機構を動かす
            kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
            if (zone_ == "blue" && is_inner) {
              kfs_ryaw_move_front_mech_lock();
            } else if (zone_ == "blue" && !is_inner) {
              kfs_ryaw_move_rear_mech_lock();
            } else if (zone_ == "red" && is_inner) {
              kfs_ryaw_move_rear_mech_lock();
            } else if (zone_ == "red" && !is_inner) {
              kfs_ryaw_move_front_mech_lock();
            }
            kfs_rear_pump(1.0);
            kfs_rear_valve(false);
            if (
              target_forest_number == 2 || target_forest_number == 4 ||
              target_forest_number == 10 || target_forest_number == 12) {
              kfs_rz_pos_ref(KFS_RZ_LOW_POS);
            } else if (
              target_forest_number == 1 || target_forest_number == 3 || target_forest_number == 7 ||
              target_forest_number == 9 || target_forest_number == 11) {
              kfs_rz_pos_ref(KFS_RZ_MIDDLE_POS);
            } else if (target_forest_number == 6) {
              kfs_rz_pos_ref(KFS_RZ_HIGH_POS);
            }
          }
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "%d forest %s kfs collect skipped because enable_auto_collect_kfs_actuator=false",
            target_forest_number, kfs_auto_collect_plan_.kfs_mechanism_type[i].c_str());
        }
        if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
          RCLCPP_INFO(
            this->get_logger(), "%d forest %s kfs collect", target_forest_number,
            kfs_auto_collect_plan_.kfs_mechanism_type[i].c_str());
        }
      }
    }
    // 最後に前回値を更新する
    prev_within = within;
  }

  if (front_kfs_assigned) {
    if (front_kfs_within) {
      // FKFS が担当範囲内に入ったら緑固定
      set_fkfs_led_status(0, 50, 0, 0.0);
    } else {
      // FKFS が担当中だが範囲外なら赤固定
      set_fkfs_led_status(50, 0, 0, 0.0);
    }
  }

  if (rear_kfs_assigned) {
    if (rear_kfs_within) {
      // RKFS が担当範囲内に入ったら緑固定
      set_rkfs_led_status(0, 50, 0, 0.0);
    } else {
      // RKFS が担当中だが範囲外なら赤固定
      set_rkfs_led_status(50, 0, 0, 0.0);
    }
  }
}

void R1MainNode::manual_mode8_auto_collect_kfs(void)
{
  if (ps4_->is_pushed_triangle()) {
    std::vector<int> forest_order;
    std::vector<std::string> collect_kfs_type;
    if (build_inner_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
      start_kfs_auto_collect(
        KfsAutoCollectStatus::INNER_ACTIVE, std::move(forest_order), std::move(collect_kfs_type));
    }
  }

  if (ps4_->is_pushed_cross()) {
    std::vector<int> forest_order;
    std::vector<std::string> collect_kfs_type;
    if (build_outer_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
      start_kfs_auto_collect(
        KfsAutoCollectStatus::OUTER_ACTIVE, std::move(forest_order), std::move(collect_kfs_type));
    }
  }

  if (ps4_->is_pushed_circle()) {
    reset_position(true);
    stop_kfs_auto_collect();
  }

  if (ps4_->is_pushed_up()) {
    spear_x_pos_ref(spear_x_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_down()) {
    spear_x_pos_ref(spear_x_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_left()) {
    kfs_front_pump(0.0);
    kfs_front_valve(true);
    // setTimeout風で電磁弁をOFFにする。
    manual_mode7_front_valve_timer_ = this->create_wall_timer(250ms, [this]() {
      kfs_front_valve(false);
      manual_mode7_front_valve_timer_->cancel();
    });
    kfs_rear_pump(0.0);
    kfs_rear_valve(true);
    // setTimeout風で電磁弁をOFFにする。
    manual_mode7_rear_valve_timer_ = this->create_wall_timer(250ms, [this]() {
      kfs_rear_valve(false);
      manual_mode7_rear_valve_timer_->cancel();
    });
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_fxの微調整（指令値を減少）
    kfs_fx_pos_ref(kfs_fx_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    // kfs_fxの微調整（指令値を増加）
    kfs_fx_pos_ref(kfs_fx_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    // kfs_fzの微調整（指令値を減少）
    kfs_fz_pos_ref(kfs_fz_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r2()) {
    // kfs_fzの微調整（指令値を増加）
    kfs_fz_pos_ref(kfs_fz_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_square()) {
    stop_kfs_auto_collect();
  }
}

void R1MainNode::manual_mode9_auto_chassis(void)
{
  const bool chassis_idle =
    (chassis_act_status_ == ChassisAct::NONE) && !pending_auto_robot_move_valid_;

  if (!chassis_idle) {
    return;
  }

  if (ps4_->is_pushed_triangle()) {
    spear_x_pos_ref(SPEAR_X_MIDDLE_POS);
    spear_roll_pos_ref(SPEAR_ROLL_VERTICAL_ANGLE);
    start_auto_chassis(ChassisAct::ACT0_START, std::vector<int>{}, std::vector<std::string>{});
  } else if (ps4_->is_pushed_circle()) {
    set_mecanum_yaw(0.0);
    set_odometry(-5.5, 0.5, 0.0);
    set_initialpose(-5.5, 0.5, 0.0);
  } else if (ps4_->is_pushed_cross()) {
    std::vector<int> forest_order;
    std::vector<std::string> collect_kfs_type;
    if (build_inner_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
      start_auto_chassis(ChassisAct::ACT2_START, forest_order, collect_kfs_type);
      start_kfs_auto_collect(
        KfsAutoCollectStatus::INNER_ACTIVE, std::move(forest_order), std::move(collect_kfs_type));
    }
  } else if (ps4_->is_pushed_square()) {
    std::vector<int> forest_order;
    std::vector<std::string> collect_kfs_type;
    if (build_outer_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
      start_auto_chassis(ChassisAct::ACT3_START, forest_order, collect_kfs_type);
      start_kfs_auto_collect(
        KfsAutoCollectStatus::OUTER_ACTIVE, std::move(forest_order), std::move(collect_kfs_type));
    }
  } else if (ps4_->is_pushed_down()) {
    start_auto_chassis(ChassisAct::ACT1_START, std::vector<int>{}, std::vector<std::string>{});
  }
  if (ps4_->is_pushed_right()) {
    kfs_fx_detect_origin();
    kfs_fz_detect_origin();
    kfs_fyaw_detect_origin();
    kfs_rx_detect_origin();
    kfs_rz_detect_origin();
    kfs_ryaw_detect_origin();
  }
}

void R1MainNode::update_auto_chassis_task(void)
{
  publish_pending_auto_robot_move_if_ready();

  const ChassisAct step = chassis_act_status_;
  if (step != ChassisAct::NONE) {
    auto_chassis_status_ = step;
  } else if (!pending_auto_robot_move_valid_) {
    auto_chassis_status_ = ChassisAct::NONE;
  }

  if (
    step == ChassisAct::ACT0_FINISH || step == ChassisAct::ACT1_FINISH ||
    step == ChassisAct::ACT2_FINISH || step == ChassisAct::ACT3_FINISH ||
    step == ChassisAct::ACT4_FINISH) {
    if (step == ChassisAct::ACT2_FINISH || step == ChassisAct::ACT3_FINISH) {
      stop_kfs_auto_collect();
    }
    publish_chassis_act_ref(ChassisAct::NONE);
    clear_auto_chassis_state(false);
    // FINISH 状態を 1 周期で消費しないと、manual mode の KFS 自動回収まで
    // 毎周期 stop されてしまう。
    chassis_act_status_ = ChassisAct::NONE;
    return;
  }

  if (ps4_->is_pushed_r2()) {
    if (step == ChassisAct::ACT0) {
      publish_robot_move(ChassisAct::ACT0_FINISH, std::vector<int>{}, std::vector<std::string>{});
    } else if (step == ChassisAct::ACT1) {
      publish_robot_move(ChassisAct::ACT1_FINISH, std::vector<int>{}, std::vector<std::string>{});
    } else if (step == ChassisAct::ACT2) {
      publish_robot_move(ChassisAct::ACT2_FINISH, std::vector<int>{}, std::vector<std::string>{});
      stop_kfs_auto_collect();
    } else if (step == ChassisAct::ACT3) {
      publish_robot_move(ChassisAct::ACT3_FINISH, std::vector<int>{}, std::vector<std::string>{});
      stop_kfs_auto_collect();
    } else if (step == ChassisAct::ACT4) {
      publish_robot_move(ChassisAct::ACT4_FINISH, std::vector<int>{}, std::vector<std::string>{});
    } else if (pending_auto_robot_move_valid_) {
      clear_auto_chassis_state(true);
    }
  }
}

void R1MainNode::reset_step(void)
{
  // 各手順のステップをリセット
  manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
  manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  manual_mode3_brake_valve_step_ = DEFAULT_STEP;
  manual_mode4_fx_step_ = DEFAULT_STEP;
  manual_mode4_fz_step_ = DEFAULT_STEP;
  manual_mode4_fyaw_step_ = DEFAULT_STEP;
  manual_mode4_front_pump_step_ = DEFAULT_STEP;
  manual_mode4_r1_long_press_triger_ = false;
  manual_mode5_rx_step_ = DEFAULT_STEP;
  manual_mode5_rz_step_ = DEFAULT_STEP;
  manual_mode5_ryaw_step_ = DEFAULT_STEP;
  manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  manual_mode5_r1_long_press_triger_ = false;
  manual_mode6_front_expand_step_ = DEFAULT_STEP;
  manual_mode6_rear_expand_step_ = DEFAULT_STEP;
  manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  manual_mode7_spear_hand_valve1_step_ = DEFAULT_STEP;
  chassis_rotate90 = false;
  stop_kfs_auto_collect();
  clear_auto_chassis_state(false);
  publish_chassis_act_stop();
}

void R1MainNode::reset_position(bool is_start_zone)
{
  if (zone_ != "blue" && zone_ != "red") {
    RCLCPP_WARN(
      this->get_logger(), "Invalid reset zone: %s. Fallback to current zone: %s", zone_.c_str(),
      zone_.c_str());
    return;
  }

  double start_x = 0.0;
  double start_y = 0.5;
  double start_yaw = 0.0;
  if (zone_ == "blue") {
    start_x = -5.5;
    start_yaw = 0.0;
  } else {
    start_x = 5.5;
    start_yaw = 0.0;
  }
  if (!is_start_zone) {
    // TODO: start zone 以外の初期位置が確定したらここで切り替える。
  }

  // 現在の角度が start_yaw となるようなオフセットを設定する。
  set_mecanum_yaw(start_yaw);
  set_swerve_drive_yaw(start_yaw);
  set_odometry(start_x, start_y, start_yaw);
  set_initialpose(start_x, start_y, start_yaw);
}

void R1MainNode::reset_robot(bool is_start_zone)
{
  // stepをリセットする
  reset_step();
  stop_actuator();
  // 位置をリセットする
  reset_position(is_start_zone);
  // initial_stateにする
  state_machine_->set_next_state(initial_state_);
  state_machine_->print_state(initial_state_, "Reset to initial state: ");
  is_initialized_ = true;
  set_led_event(0, 0, 50, 0.2, 1.0);
}

void R1MainNode::manual_task(void)
{
  auto current_state = state_machine_->get_current_state();
  // スピア作成時だけロボットの速度を落とす
  double vx_max = CHASSIS_MAX_VELOCITY;
  double vy_max = CHASSIS_MAX_VELOCITY;
  double vz_max = CHASSIS_MAX_OMEGA;
  bool on_mode2_low_speed = (current_state.operation_mode == OperationMode::MODE2_POLE) &&
                            ps4_->is_pushing_cross() == false;
  bool on_mode3_low_speed = (current_state.operation_mode == OperationMode::MODE3_SPEAR) &&
                            ps4_->is_pushing_cross() == false;
  if (on_mode2_low_speed || on_mode3_low_speed) {
    vx_max = CHASSIS_MAKE_SPEAR_VELOCITY;
    vy_max = CHASSIS_MAKE_SPEAR_VELOCITY;
    vz_max = CHASSIS_MAKE_SPEAR_OMEGA;
  }

  if (current_state.chassis_control_mode == ChassisControlMode::MANUAL) {
    double vx_ref = vx_max * (-1) * ps4_->get_left_stick_x();
    double vy_ref = vy_max * ps4_->get_left_stick_y();
    double vz_ref = vz_max * ps4_->get_right_stick_x();
    if (ps4_->is_pushed_left_stick()) {
      chassis_rotate90 = !chassis_rotate90;
    }
    if (chassis_rotate90) {
      //L2を押している間は赤ゾーンの場合はロボットを90度回転、青ゾーンの場合はロボットを-90度回転させる
      double rotated_vx_ref, rotated_vy_ref;
      if (zone_ == "red") {
        // 赤ゾーンのとき
        rotated_vx_ref = -vy_ref;
        rotated_vy_ref = vx_ref;
      } else {
        // 青ゾーンのとき
        rotated_vx_ref = vy_ref;
        rotated_vy_ref = -vx_ref;
      }
      vx_ref = rotated_vx_ref;
      vy_ref = rotated_vy_ref;
    }
    chassis_move_vel(vx_ref, vy_ref, vz_ref);
  }

  if (current_state.operation_mode == OperationMode::MODE1_DETECT_ORIGIN) {
    manual_mode1_detect_origin();
  } else if (current_state.operation_mode == OperationMode::MODE2_POLE) {
    manual_mode2_pole();
  } else if (current_state.operation_mode == OperationMode::MODE3_SPEAR) {
    manual_mode3_spear();
  } else if (current_state.operation_mode == OperationMode::MODE4_FKFS) {
    manual_mode4_fkfs();
  } else if (current_state.operation_mode == OperationMode::MODE5_RKFS) {
    manual_mode5_rkfs();
  } else if (current_state.operation_mode == OperationMode::MODE6_R2_LIFT) {
    manual_mode6_r2_lift();
  } else if (current_state.operation_mode == OperationMode::MODE7_SPEAR_ATTACK) {
    manual_mode7_spear_attack();
  } else if (current_state.operation_mode == OperationMode::MODE8_AUTO_COLLECT_KFS) {
    manual_mode8_auto_collect_kfs();
  } else if (current_state.operation_mode == OperationMode::MODE9_AUTO_CHASSIS) {
    manual_mode9_auto_chassis();
  }
}

void R1MainNode::main_task(void)
{
  static bool stop_actuator_flag = false;
  auto current_state = state_machine_->get_current_state();
  if (current_state.main == MainState::IDLE) {
    idle_task();
    return;
  }
  if (current_state.main == MainState::EMERGENCY) {
    emergency_task();
    return;
  }
  if (current_state.main != MainState::READY) {
    return;
  }

  if (ps4_->is_connected() == false) {
    if (stop_actuator_flag == false) {
      stop_actuator();
      stop_actuator_flag = true;
    }
    auto next_state = current_state;
    next_state.chassis_control_mode = ChassisControlMode::HOLD;
    state_machine_->set_next_state(next_state);
    return;
  }

  stop_actuator_flag = false;

  auto next_state = current_state;
  if (ps4_->is_pushed_options()) {
    sabacan_power_ref(!sabacan_is_ems_);
  }
  if (ps4_->is_pushed_ps()) {
    is_initialized_ = false;
    reset_robot(true);
    publish_r1_machine_initialize();
    return;
  }
  if (ps4_->is_pushed_right_stick()) {
    const bool auto_running =
      (chassis_act_status_ != ChassisAct::NONE) || pending_auto_robot_move_valid_;
    if (auto_running) {
      publish_chassis_act_stop();
      clear_auto_chassis_state(true);
      chassis_act_status_ = ChassisAct::NONE;
      next_state.chassis_control_mode = ChassisControlMode::MANUAL;
    } else if (is_initialized_) {
      bool started_auto = false;
      if (current_state.operation_mode == OperationMode::MODE1_DETECT_ORIGIN) {
        // MODE1のときはACT0_STARTを開始する
        start_auto_chassis(ChassisAct::ACT0_START, std::vector<int>{}, std::vector<std::string>{});
        started_auto = true;
        // MODE2のステップをリセットする
        manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
        // 最初のタスクを実行する
        manual_mode2_collect_pole_task();
      } else if (current_state.operation_mode == OperationMode::MODE2_POLE) {
        // MODE2のときはACT1_STARTを開始する
        start_auto_chassis(ChassisAct::ACT1_START, std::vector<int>{}, std::vector<std::string>{});
        started_auto = true;
        // MODE3のステップをリセットする
        manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
        // 最初のタスクを実行する
        manual_mode3_make_spear_task(2);

      } else if (current_state.operation_mode == OperationMode::MODE3_SPEAR) {
        // MODE3のときはOUTER_ACTIVEのときはACT3_STARTを、INNER_ACTIVEのときはACT2_STARTを開始する
        std::vector<int> forest_order;
        std::vector<std::string> collect_kfs_type;
        KfsAutoCollectStatus mode3_collect_status = kfs_auto_collect_plan_.status;
        if (mode3_collect_status == KfsAutoCollectStatus::NONE) {
          if (!set_mode3_kfs_auto_collect_status(mode3_collect_status)) {
            RCLCPP_WARN(
              this->get_logger(),
              "Failed to infer MODE3_SPEAR inner/outer placement for right stick auto advance.");
          }
        }
        if (mode3_collect_status == KfsAutoCollectStatus::OUTER_ACTIVE) {
          if (build_outer_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
            start_auto_chassis(ChassisAct::ACT3_START, forest_order, collect_kfs_type);
            start_kfs_auto_collect(
              KfsAutoCollectStatus::OUTER_ACTIVE, std::move(forest_order),
              std::move(collect_kfs_type));
            started_auto = true;
          }
        } else if (mode3_collect_status == KfsAutoCollectStatus::INNER_ACTIVE) {
          if (build_inner_kfs_auto_collect_plan(forest_order, collect_kfs_type)) {
            start_auto_chassis(ChassisAct::ACT2_START, forest_order, collect_kfs_type);
            start_kfs_auto_collect(
              KfsAutoCollectStatus::INNER_ACTIVE, std::move(forest_order),
              std::move(collect_kfs_type));
            started_auto = true;
          }
        }
      }

      if (started_auto) {
        next_state.operation_mode = next_operation_mode(current_state.operation_mode);
        next_state.chassis_control_mode = ChassisControlMode::AUTO;
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Right stick auto advance is not supported in operation_mode=%s",
          std::string(magic_enum::enum_name(current_state.operation_mode)).c_str());
      }
    }
  }
  // shareボタン: 短押し→次のモードへ、長押し（1秒以上）→一つ前のモードへ戻る
  if (ps4_->is_pushed_share()) {
    share_press_start_time_ = this->now();
    share_long_press_triggered_ = false;
  }
  if (ps4_->is_pushing_share() && !share_long_press_triggered_) {
    if ((this->now() - share_press_start_time_).seconds() >= SHARE_LONG_PRESS_SEC) {
      next_state.operation_mode = state_machine_->get_prev_state().operation_mode;
      share_long_press_triggered_ = true;
    }
  }
  if (ps4_->is_released_share() && !share_long_press_triggered_) {
    next_state.operation_mode = next_operation_mode(current_state.operation_mode);
  }

  const bool chassis_auto_requested = (chassis_act_status_ != ChassisAct::NONE) ||
                                      pending_auto_robot_move_valid_ ||
                                      (auto_chassis_status_ != ChassisAct::NONE);
  next_state.chassis_control_mode =
    chassis_auto_requested ? ChassisControlMode::AUTO : ChassisControlMode::MANUAL;
  state_machine_->set_next_state(next_state);

  if (is_initialized_ == false) {
    return;
  }

  manual_task();
  update_auto_chassis_task();
  auto_collect_kfs_task();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1MainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
