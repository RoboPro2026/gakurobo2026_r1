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

#include <algorithm>
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

const std::vector<OperationMode> kOperationModeOrder = {
  OperationMode::MODE1_DETECT_ORIGIN, OperationMode::MODE2_POLE, OperationMode::MODE3_SPEAR,
  OperationMode::MODE4_FKFS,          OperationMode::MODE5_RKFS, OperationMode::MODE6_R2_LIFT,
  OperationMode::MODE7_SPEAR_ATTACK,
};

OperationMode cycle_operation_mode(OperationMode mode, int step)
{
  auto it = std::find(kOperationModeOrder.begin(), kOperationModeOrder.end(), mode);
  if (it == kOperationModeOrder.end()) return OperationMode::MODE1_DETECT_ORIGIN;

  const int n = static_cast<int>(kOperationModeOrder.size());
  const int current_idx = static_cast<int>(std::distance(kOperationModeOrder.begin(), it));
  const int next_idx = (current_idx + step + n) % n;
  return kOperationModeOrder[next_idx];
}

OperationMode next_operation_mode(OperationMode mode) { return cycle_operation_mode(mode, +1); }
OperationMode prev_operation_mode(OperationMode mode) { return cycle_operation_mode(mode, -1); }

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
  const std::string & name, double * position_ref_alias, double * speed_ref_alias,
  bool use_set_angle_topic, double * current_pos_alias)
{
  auto [it, inserted] = position_axes_.try_emplace(name);
  (void)inserted;
  auto & axis = it->second;
  axis.position_ref_alias = position_ref_alias;
  axis.speed_ref_alias = speed_ref_alias;
  axis.current_pos_alias = current_pos_alias;
  if (axis.position_ref_alias != nullptr) {
    *axis.position_ref_alias = axis.position_ref;
  }
  if (axis.speed_ref_alias != nullptr) {
    *axis.speed_ref_alias = axis.speed_ref;
  }
  if (axis.current_pos_alias != nullptr) {
    *axis.current_pos_alias = axis.current_pos;
  }

  axis.position_ref_publisher =
    this->create_publisher<std_msgs::msg::Float64>("/" + name + "_position_ref", 10);
  axis.set_position_publisher = this->create_publisher<std_msgs::msg::Float64>(
    "/" + name + (use_set_angle_topic ? "_set_angle" : "_set_pos"), 10);
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
  const std::string current_pos_suffix = use_set_angle_topic ? "_current_angle" : "_current_pos";
  PositionAxisInterface * axis_ptr = &axis;
  axis.current_pos_subscription = this->create_subscription<std_msgs::msg::Float64>(
    "/" + name + current_pos_suffix, 10, [axis_ptr](const std_msgs::msg::Float64::SharedPtr msg) {
      axis_ptr->current_pos = msg->data;
      if (axis_ptr->current_pos_alias != nullptr) {
        *axis_ptr->current_pos_alias = msg->data;
      }
    });
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
 * @brief 指定した位置制御軸へ set_pos / set_angle 指令を publish する。
 * @param name 軸名。
 * @param pos 設定したい論理位置・角度。
 */
void R1MainNode::set_position_axis(const std::string & name, double pos)
{
  const auto it = position_axes_.find(name);
  if (it == position_axes_.end() || !it->second.set_position_publisher) {
    RCLCPP_ERROR(this->get_logger(), "%s set position axis is not initialized", name.c_str());
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = pos;
  it->second.set_position_publisher->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s set_pos %f", name.c_str(), pos);
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
  register_position_axis("kfs_fx", &kfs_fx_position_ref_, nullptr, false, &kfs_fx_current_pos_);
  register_position_axis("kfs_fz", &kfs_fz_position_ref_, nullptr, false, &kfs_fz_current_pos_);
  register_position_axis(
    "kfs_fyaw", &kfs_fyaw_position_ref_, nullptr, true, &kfs_fyaw_current_pos_);
  register_position_axis("kfs_rx", &kfs_rx_position_ref_, nullptr, false, &kfs_rx_current_pos_);
  register_position_axis("kfs_rz", &kfs_rz_position_ref_, nullptr, false, &kfs_rz_current_pos_);
  register_position_axis(
    "kfs_ryaw", &kfs_ryaw_position_ref_, nullptr, true, &kfs_ryaw_current_pos_);
  register_position_axis(
    "r2_flift", &r2_flift_position_ref_, nullptr, false, &r2_flift_current_pos_);
  register_position_axis(
    "r2_rlift", &r2_rlift_position_ref_, nullptr, false, &r2_rlift_current_pos_);
  register_position_axis("spear_y", &spear_y_position_ref_, nullptr, false, &spear_y_current_pos_);
  register_position_axis(
    "spear_roll1", &spear_roll1_position_ref_, nullptr, true, &spear_roll1_current_pos_);
  register_position_axis(
    "spear_roll2", &spear_roll2_position_ref_, nullptr, true, &spear_roll2_current_pos_);
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
  register_gpio_pwm_output("spear_hand1_valve", nullptr, &spear_hand1_valve_ref_);
  register_gpio_pwm_output("spear_hand2_valve", nullptr, &spear_hand2_valve_ref_);
  register_gpio_pwm_output("spear_hand_push_valve", nullptr, &spear_hand_push_valve_ref_);
  // センサー入力
  register_gpio_input("kfs_fz_low_switch", &kfs_fz_low_switch_status_);
  register_gpio_input("kfs_rz_low_switch", &kfs_rz_low_switch_status_);
  register_gpio_input("front_pressure_switch", &front_pressure_switch_status_);
  register_gpio_input("rear_pressure_switch", &rear_pressure_switch_status_);
  register_gpio_input("r2_flift_low_switch", &r2_flift_low_switch_status_);
  register_gpio_input("r2_flift_high_switch", &r2_flift_high_switch_status_);
  register_gpio_input("r2_rlift_low_switch", &r2_rlift_low_switch_status_);
  register_gpio_input("r2_rlift_high_switch", &r2_rlift_high_switch_status_);

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
  // Scan (YDLidar) の登録
  register_scan("/scan_fh", scan_fh_subscription_, scan_fh_data_);
  register_scan("/scan_fm", scan_fm_subscription_, scan_fm_data_);
  register_scan("/scan_fl", scan_fl_subscription_, scan_fl_data_);
  register_scan("/scan_rh", scan_rh_subscription_, scan_rh_data_);
  register_scan("/scan_rm", scan_rm_subscription_, scan_rm_data_);
  register_scan("/scan_rl", scan_rl_subscription_, scan_rl_data_);

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
  // 接線方向PID補正ON/OFFのPublisher
  chassis_tangent_pid_enable_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/chassis_tangent_pid_enable", 10);
  // robot_moveのPublisher
  robot_move_publisher_ = this->create_publisher<r1_msgs::msg::RobotMove>("/robot_move", 10);
  // r1_machine_manage_node の初期化要求
  r1_machine_initialize_publisher_ =
    this->create_publisher<std_msgs::msg::Empty>("/r1_machine_initialize", 10);
  // r1_machine_manage_node の初期化完了通知
  r1_machine_initialize_done_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
    "/r1_machine_initialize_done", 10,
    std::bind(&R1MainNode::r1_machine_initialize_done_callback, this, std::placeholders::_1));

  // arucoマーカ
  spear_red_aruco_marker_id_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>("/spear_red_aruco_marker_id", 10);
  spear_blue_aruco_marker_id_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>("/spear_blue_aruco_marker_id", 10);
  r2_lift_lower_aruco_marker_id_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>("/r2_lift_lower_aruco_marker_id", 10);
  r2_lift_upper_aruco_marker_id_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>("/r2_lift_upper_aruco_marker_id", 10);

  // スマホ通信
  r1_operation_mode_publisher_ =
    this->create_publisher<std_msgs::msg::Int32>("/r1_operation_mode", 10);
  // r1_log_message_publisher_ = this->create_publisher<std_msgs::msg::String>("/r1_log_message", 10);

  r1_log_message_info_publisher_ =
    this->create_publisher<std_msgs::msg::String>("/r1_log_message_info", 10);
  r1_log_message_warn_publisher_ =
    this->create_publisher<std_msgs::msg::String>("/r1_log_message_warn", 10);
  r1_log_message_error_publisher_ =
    this->create_publisher<std_msgs::msg::String>("/r1_log_message_error", 10);

  r1_init_parameter_subscription_ = this->create_subscription<r1_msgs::msg::R1InitParameter>(
    "/r1_init_parameter", 10,
    std::bind(&R1MainNode::r1_init_parameter_callback, this, std::placeholders::_1));
  r1_collect_kfs_subscription_ = this->create_subscription<r1_msgs::msg::R1CollectKfs>(
    "/r1_collect_kfs", 10,
    std::bind(&R1MainNode::r1_collect_kfs_callback, this, std::placeholders::_1));
  r1_kfs_mechanism_ref_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/r1_kfs_mechanism_ref", 10,
    std::bind(&R1MainNode::r1_kfs_mechanism_ref_callback, this, std::placeholders::_1));
  r1_retry_collect_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/r1_retry_collect", 10,
    std::bind(&R1MainNode::r1_retry_collect_callback, this, std::placeholders::_1));
  r1_collect_3rd_kfs_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/r1_collect_3rd_kfs", 10,
    std::bind(&R1MainNode::r1_collect_3rd_kfs_callback, this, std::placeholders::_1));
  r1_initialize_all_actuator_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/r1_initialize_all_actuator", 10,
    std::bind(&R1MainNode::r1_initialize_all_actuator_callback, this, std::placeholders::_1));
  r1_aruco_marker_id_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/r1_aruco_marker_id", 10,
    std::bind(&R1MainNode::r1_aruco_marker_id_callback, this, std::placeholders::_1));

  // tf関連
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  initialize_lidar_lifecycle_clients();

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
  declare_and_get_parameter("activate_lidar_on_ps", activate_lidar_on_ps_, true);
  declare_and_get_parameter("share_long_press_sec", SHARE_LONG_PRESS_SEC, 1.0);
  declare_and_get_parameter("ps_long_press_sec", PS_LONG_PRESS_SEC, 2.0);
  declare_and_get_parameter("enable_right_stick_pause", enable_right_stick_pause_, false);
  declare_and_get_parameter("initialpose_tf_log_delay_sec", initialpose_tf_log_delay_sec_, 1.0);
  declare_and_get_parameter("initialpose_retry1_delay_sec", initialpose_retry1_delay_sec_, 1.0);
  declare_and_get_parameter("initialpose_retry2_delay_sec", initialpose_retry2_delay_sec_, 3.0);
  declare_and_get_parameter("enable_r2_analog_speed_control", ENABLE_R2_ANALOG_SPEED_CONTROL);
  declare_and_get_parameter("chassis_low_velocity", CHASSIS_LOW_VELOCITY);
  declare_and_get_parameter("chassis_normal_velocity", CHASSIS_NORMAL_VELOCITY);
  declare_and_get_parameter("chassis_high_velocity", CHASSIS_HIGH_VELOCITY);
  declare_and_get_parameter("chassis_low_omega", CHASSIS_LOW_OMEGA);
  declare_and_get_parameter("chassis_normal_omega", CHASSIS_NORMAL_OMEGA);
  declare_and_get_parameter("chassis_high_omega", CHASSIS_HIGH_OMEGA);
  declare_and_get_parameter("chassis_make_spear_velocity", CHASSIS_MAKE_SPEAR_VELOCITY);
  declare_and_get_parameter("chassis_make_spear_omega", CHASSIS_MAKE_SPEAR_OMEGA);

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
  declare_and_get_parameter("kfs_fx_ground_pos", KFS_FX_GROUND_POS);
  // fz
  declare_and_get_parameter("kfs_fz_normal_pos", KFS_FZ_NORMAL_POS);
  declare_and_get_parameter("kfs_fz_low_pos", KFS_FZ_LOW_POS);
  declare_and_get_parameter("kfs_fz_middle_pos", KFS_FZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_fz_high_pos", KFS_FZ_HIGH_POS);
  declare_and_get_parameter("kfs_fz_put_pos", KFS_FZ_PUT_POS);
  declare_and_get_parameter("kfs_fz_storage_pos", KFS_FZ_STORAGE_POS);
  declare_and_get_parameter("kfs_fz_start_pos", KFS_FZ_START_POS);
  declare_and_get_parameter("kfs_fz_low_mech_lock_pos", KFS_FZ_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fz_high_mech_lock_pos", KFS_FZ_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_fz_r2_lift_pos", KFS_FZ_R2_LIFT_POS);
  declare_and_get_parameter("kfs_fz_ground_pos", KFS_FZ_GROUND_POS);
  // fyaw
  declare_and_get_parameter("kfs_fyaw_normal_angle", KFS_FYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_fyaw_front_angle", KFS_FYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_fyaw_side_angle", KFS_FYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_fyaw_rear_angle", KFS_FYAW_REAR_ANGLE);
  declare_and_get_parameter("kfs_fyaw_start_angle", KFS_FYAW_START_ANGLE);
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
  declare_and_get_parameter("kfs_rx_ground_pos", KFS_RX_GROUND_POS);
  // rz
  declare_and_get_parameter("kfs_rz_normal_pos", KFS_RZ_NORMAL_POS);
  declare_and_get_parameter("kfs_rz_low_pos", KFS_RZ_LOW_POS);
  declare_and_get_parameter("kfs_rz_middle_pos", KFS_RZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_rz_high_pos", KFS_RZ_HIGH_POS);
  declare_and_get_parameter("kfs_rz_put_pos", KFS_RZ_PUT_POS);
  declare_and_get_parameter("kfs_rz_storage_pos", KFS_RZ_STORAGE_POS);
  declare_and_get_parameter("kfs_rz_start_pos", KFS_RZ_START_POS);
  declare_and_get_parameter("kfs_rz_low_mech_lock_pos", KFS_RZ_LOW_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rz_high_mech_lock_pos", KFS_RZ_HIGH_MECH_LOCK_POS);
  declare_and_get_parameter("kfs_rz_r2_lift_pos", KFS_RZ_R2_LIFT_POS);
  declare_and_get_parameter("kfs_rz_ground_pos", KFS_RZ_GROUND_POS);
  // ryaw
  declare_and_get_parameter("kfs_ryaw_normal_angle", KFS_RYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_ryaw_front_angle", KFS_RYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_ryaw_side_angle", KFS_RYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_ryaw_rear_angle", KFS_RYAW_REAR_ANGLE);
  declare_and_get_parameter("kfs_ryaw_start_angle", KFS_RYAW_START_ANGLE);
  declare_and_get_parameter("kfs_ryaw_low_mech_lock_angle", KFS_RYAW_LOW_MECH_LOCK_ANGLE);
  declare_and_get_parameter("kfs_ryaw_high_mech_lock_angle", KFS_RYAW_HIGH_MECH_LOCK_ANGLE);
  // 真空用電磁弁の遅延時間[s]
  declare_and_get_parameter("kfs_valve_delay_time", KFS_VALVE_DELAY_TIME);
  // arucoマーカの指令を受け取ってからもとのarucoマーカの表示に戻す時間 [s]
  declare_and_get_parameter("aruco_marker_reset_time", ARUCO_MARKER_RESET_TIME);

  // ========== 展開 ==========
  // R2昇降
  declare_and_get_parameter("r2_flift_normal_pos", R2_FLIFT_NORMAL_POS);
  declare_and_get_parameter("r2_flift_up_pos", R2_FLIFT_UP_POS);
  declare_and_get_parameter("r2_flift_down_pos", R2_FLIFT_DOWN_POS);
  declare_and_get_parameter("r2_rlift_normal_pos", R2_RLIFT_NORMAL_POS);
  declare_and_get_parameter("r2_rlift_up_pos", R2_RLIFT_UP_POS);
  declare_and_get_parameter("r2_rlift_down_pos", R2_RLIFT_DOWN_POS);
  // ========== やり ==========
  // spear_y
  declare_and_get_parameter("spear_y_collect1_pos", SPEAR_Y_COLLECT1_POS);
  declare_and_get_parameter("spear_y_collect2_pos", SPEAR_Y_COLLECT2_POS);
  declare_and_get_parameter("spear_y_make_spear_pos", SPEAR_Y_MAKE_SPEAR_POS);
  declare_and_get_parameter("spear_y_collect_kfs_pos", SPEAR_Y_COLLECT_KFS_POS);
  declare_and_get_parameter("spear_y_low_attack_pos", SPEAR_Y_LOW_ATTACK_POS);
  declare_and_get_parameter("spear_y_middle_attack_pos", SPEAR_Y_MIDDLE_ATTACK_POS);
  declare_and_get_parameter("spear_y_high_attack_pos", SPEAR_Y_HIGH_ATTACK_POS);
  declare_and_get_parameter("spear_y_throw_away_pos", SPEAR_Y_THROW_AWAY_POS);
  // spear_roll1
  declare_and_get_parameter("spear_roll1_normal_angle", SPEAR_ROLL1_NORMAL_ANGLE);
  declare_and_get_parameter("spear_roll1_vertical_angle", SPEAR_ROLL1_VERTICAL_ANGLE);
  declare_and_get_parameter("spear_roll1_horizontal_angle", SPEAR_ROLL1_HORIZONTAL_ANGLE);
  declare_and_get_parameter("spear_roll1_inv_horizontal_angle", SPEAR_ROLL1_INV_HORIZONTAL_ANGLE);
  declare_and_get_parameter("spear_roll1_red_low_attack_angle", SPEAR_ROLL1_RED_LOW_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll1_red_middle_attack_angle", SPEAR_ROLL1_RED_MIDDLE_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll1_red_high_attack_angle", SPEAR_ROLL1_RED_HIGH_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll1_blue_low_attack_angle", SPEAR_ROLL1_BLUE_LOW_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll1_blue_middle_attack_angle", SPEAR_ROLL1_BLUE_MIDDLE_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll1_blue_high_attack_angle", SPEAR_ROLL1_BLUE_HIGH_ATTACK_ANGLE);
  // spear_roll2
  declare_and_get_parameter("spear_roll2_normal_angle", SPEAR_ROLL2_NORMAL_ANGLE);
  declare_and_get_parameter("spear_roll2_vertical_angle", SPEAR_ROLL2_VERTICAL_ANGLE);
  declare_and_get_parameter("spear_roll2_horizontal_angle", SPEAR_ROLL2_HORIZONTAL_ANGLE);
  declare_and_get_parameter("spear_roll2_inv_horizontal_angle", SPEAR_ROLL2_INV_HORIZONTAL_ANGLE);
  declare_and_get_parameter("spear_roll2_red_low_attack_angle", SPEAR_ROLL2_RED_LOW_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll2_red_middle_attack_angle", SPEAR_ROLL2_RED_MIDDLE_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll2_red_high_attack_angle", SPEAR_ROLL2_RED_HIGH_ATTACK_ANGLE);
  declare_and_get_parameter("spear_roll2_blue_low_attack_angle", SPEAR_ROLL2_BLUE_LOW_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll2_blue_middle_attack_angle", SPEAR_ROLL2_BLUE_MIDDLE_ATTACK_ANGLE);
  declare_and_get_parameter(
    "spear_roll2_blue_high_attack_angle", SPEAR_ROLL2_BLUE_HIGH_ATTACK_ANGLE);
  // spear_roll微調整用パラメータ
  declare_and_get_parameter("spear_roll_adjust_angle", SPEAR_ROLL_ADJUST_ANGLE);

  for (int i = 0; i < 12; i++) {
    const std::string idx = std::to_string(i + 1);
    std::string blue_inner_name = "blue_inner_collect_kfs_center_pos." + idx;
    std::string blue_outer_name = "blue_outer_collect_kfs_center_pos." + idx;
    std::string red_inner_name = "red_inner_collect_kfs_center_pos." + idx;
    std::string red_outer_name = "red_outer_collect_kfs_center_pos." + idx;
    this->declare_parameter<std::vector<double>>(blue_inner_name, {100.0, 100.0, 0.0});
    this->declare_parameter<std::vector<double>>(blue_outer_name, {100.0, 100.0, 0.0});
    this->declare_parameter<std::vector<double>>(red_inner_name, {-100.0, 100.0, 0.0});
    this->declare_parameter<std::vector<double>>(red_outer_name, {-100.0, 100.0, 0.0});
    std::vector<double> inner_center, outer_center;
    if (zone_ == "blue") {
      this->get_parameter(blue_inner_name, inner_center);
      this->get_parameter(blue_outer_name, outer_center);
    } else {
      this->get_parameter(red_inner_name, inner_center);
      this->get_parameter(red_outer_name, outer_center);
    }

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
  declare_and_get_parameter("kfs_yaw_delay_time", KFS_YAW_DELAY_TIME);
  declare_and_get_parameter("enable_auto_collect_kfs_actuator", ENABLE_AUTO_COLLECT_KFS_ACTUATOR);
  declare_and_get_parameter("enalbe_stop_before_collect_kfs", ENABLE_STOP_BEFORE_COLLECT_KFS);
  declare_and_get_parameter("enable_wall_sensor", ENABLE_WALL_SENSOR);
  declare_and_get_parameter("enable_pressure_sensor", ENABLE_PRESSURE_SENSOR);
  declare_and_get_parameter(
    "enable_r1_kfs_mechanism_ref_pressure_sensor", ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR);
  declare_and_get_parameter("wall_sensor_distance_threshold", WALL_SENSOR_DISTANCE_THRESHOLD);
  declare_and_get_parameter("wall_sensor_time_threshold", WALL_SENSOR_TIME_THRESHOLD);
  declare_and_get_parameter("move_distance_after_wall_detect", MOVE_DISTANCE_AFTER_WALL_DETECT);
  declare_and_get_parameter("wall_sensor_detect_height", WALL_SENSOR_DETECT_HEIGHT);
  declare_and_get_parameter("wall_sensor_detect_width", WALL_SENSOR_DETECT_WIDTH);
  declare_and_get_parameter("wall_sensor_delay_offset_distance", WALL_SENSOR_DELAY_OFFSET_DISTANCE);
  declare_and_get_parameter("pressure_sensor_time_threshold", PRESSURE_SENSOR_TIME_THRESHOLD);
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

  // this->now() と同じクロック型で時刻ベクターを初期化する
  // ヘッダのデフォルト値 rclcpp::Time(0) は RCL_ROS_TIME (type=1) になるため、
  // use_sim_time=false 環境では this->now() (type=2) と型が合わずクラッシュする
  for (int i = 0; i < 12; i++) {
    wall_sensor_detect_start_time_[i] = this->now();
  }

  // ログのテスト出力
  r1_log_info("r1_main_node起動");
  r1_log_info("★テスト r1_log_info★");
  r1_log_warn("★テスト r1_log_warn★");
  r1_log_error("★テスト r1_log_error★");
}

void R1MainNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // IMUの情報を更新
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
}

void R1MainNode::register_scan(
  const std::string & topic_name,
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr & subscription, double & data)
{
  subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_name, rclcpp::SensorDataQoS(), [&data](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      data = static_cast<double>(msg->ranges[4]);
    });
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

void R1MainNode::spear_y_detect_origin(void) { detect_origin_position_axis("spear_y"); }

void R1MainNode::spear_roll1_detect_origin(void) { detect_origin_position_axis("spear_roll1"); }

void R1MainNode::spear_roll2_detect_origin(void) { detect_origin_position_axis("spear_roll2"); }

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

void R1MainNode::spear_y_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_y", direction);
}

void R1MainNode::spear_roll1_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_roll1", direction);
}

void R1MainNode::spear_roll2_move_mech_lock(int direction)
{
  move_mech_lock_position_axis("spear_roll2", direction);
}

void R1MainNode::r2_flift_move_down_mech_lock(void)
{
  r2_flift_move_mech_lock(-1);
  r2_flift_position_ref_ = R2_FLIFT_DOWN_POS;
}

void R1MainNode::r2_flift_move_up_mech_lock(void)
{
  r2_flift_move_mech_lock(1);
  r2_flift_position_ref_ = R2_FLIFT_UP_POS;
}

void R1MainNode::r2_rlift_move_down_mech_lock(void)
{
  r2_rlift_move_mech_lock(-1);
  r2_rlift_position_ref_ = R2_RLIFT_DOWN_POS;
}

void R1MainNode::r2_rlift_move_up_mech_lock(void)
{
  r2_rlift_move_mech_lock(1);
  r2_rlift_position_ref_ = R2_RLIFT_UP_POS;
}

void R1MainNode::kfs_fyaw_move_front_mech_lock(void)
{
  kfs_fyaw_move_mech_lock(1);
  kfs_fyaw_position_ref_ = KFS_FYAW_FRONT_ANGLE;
}
void R1MainNode::kfs_fyaw_move_rear_mech_lock(void)
{
  kfs_fyaw_move_mech_lock(-1);
  kfs_fyaw_position_ref_ = KFS_FYAW_REAR_ANGLE;
}
void R1MainNode::kfs_ryaw_move_front_mech_lock(void)
{
  kfs_ryaw_move_mech_lock(1);
  kfs_ryaw_position_ref_ = KFS_RYAW_FRONT_ANGLE;
}
void R1MainNode::kfs_ryaw_move_rear_mech_lock(void)
{
  kfs_ryaw_move_mech_lock(-1);
  kfs_ryaw_position_ref_ = KFS_RYAW_REAR_ANGLE;
}

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

void R1MainNode::update_kfs_led_status(void)
{
  constexpr int FKFS = 0;
  constexpr int RKFS = 1;
  constexpr double KFS_PUMP_ACTIVE_BLINK_PERIOD_S = 0.25;

  bool front_kfs_assigned = false;
  bool rear_kfs_assigned = false;
  if (kfs_auto_collect_plan_.status != KfsAutoCollectStatus::NONE) {
    for (const auto & mechanism_type : kfs_auto_collect_plan_.kfs_mechanism_type) {
      front_kfs_assigned |= (mechanism_type == "front_kfs");
      rear_kfs_assigned |= (mechanism_type == "rear_kfs");
    }
  }

  bool front_kfs_within = false;
  bool rear_kfs_within = false;
  for (const auto & within : kfs_auto_collect_within_) {
    front_kfs_within |= within[FKFS];
    rear_kfs_within |= within[RKFS];
  }

  const bool front_kfs_pump_active = (kfs_front_pump_ref_ > 0.0);
  const bool rear_kfs_pump_active = (kfs_rear_pump_ref_ > 0.0);
  const double front_kfs_blink_period =
    front_kfs_pump_active ? KFS_PUMP_ACTIVE_BLINK_PERIOD_S : 0.0;
  const double rear_kfs_blink_period = rear_kfs_pump_active ? KFS_PUMP_ACTIVE_BLINK_PERIOD_S : 0.0;

  if (kfs_manual_front_pressure_wait_ && kfs_manual_front_pressure_detected_) {
    // 手動指令後の圧力センサ待機中に反応あり → 青。ポンプ動作中は点滅。
    set_fkfs_led_status(0, 0, 50, front_kfs_blink_period);
  } else if (front_kfs_assigned) {
    if (front_kfs_within) {
      // FKFS が担当範囲内に入ったら緑。ポンプ動作中は点滅。
      set_fkfs_led_status(0, 50, 0, front_kfs_blink_period);
    } else {
      // FKFS が担当中だが範囲外なら赤。ポンプ動作中は点滅。
      set_fkfs_led_status(50, 0, 0, front_kfs_blink_period);
    }
  } else {
    if (front_kfs_pump_active) {
      set_fkfs_led_status(50, 0, 0, front_kfs_blink_period);
    } else {
      set_fkfs_led_status(0, 0, 0, 0.0);
    }
  }

  if (kfs_manual_rear_pressure_wait_ && kfs_manual_rear_pressure_detected_) {
    // 手動指令後の圧力センサ待機中に反応あり → 青。ポンプ動作中は点滅。
    set_rkfs_led_status(0, 0, 50, rear_kfs_blink_period);
  } else if (rear_kfs_assigned) {
    if (rear_kfs_within) {
      // RKFS が担当範囲内に入ったら緑。ポンプ動作中は点滅。
      set_rkfs_led_status(0, 50, 0, rear_kfs_blink_period);
    } else {
      // RKFS が担当中だが範囲外なら赤。ポンプ動作中は点滅。
      set_rkfs_led_status(50, 0, 0, rear_kfs_blink_period);
    }
  } else {
    if (rear_kfs_pump_active) {
      set_rkfs_led_status(50, 0, 0, rear_kfs_blink_period);
    } else {
      set_rkfs_led_status(0, 0, 0, 0.0);
    }
  }
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

void R1MainNode::r1_init_parameter_callback(const r1_msgs::msg::R1InitParameter::SharedPtr msg)
{
  r1_init_parameter_ = *msg;
  received_r1_init_parameter_ = true;
  // データを受信したらKFS自動走行関連のパラメータもリセットする
  reset_kfs_auto_collect_tracking();
  std::string s;
  s += "zone: " + r1_init_parameter_.zone + "\n";
  for (int i = 0; i < (int)r1_init_parameter_.r1_kfs_value.size(); i++) {
    s += "r1_kfs_value[" + std::to_string(i) +
         "]: " + std::to_string(r1_init_parameter_.r1_kfs_value[i]) + "\n";
  }
  for (int i = 0; i < (int)r1_init_parameter_.r2_kfs_value.size(); i++) {
    s += "r2_kfs_value[" + std::to_string(i) +
         "]: " + std::to_string(r1_init_parameter_.r2_kfs_value[i]) + "\n";
  }
  for (int i = 0; i < (int)r1_init_parameter_.r2_fake_kfs_value.size(); i++) {
    s += "r2_fake_kfs_value[" + std::to_string(i) +
         "]: " + std::to_string(r1_init_parameter_.r2_fake_kfs_value[i]) + "\n";
  }
  s += std::string("enable_auto_select: ") +
       (r1_init_parameter_.enable_auto_select ? "true" : "false") + "\n";
  s += std::string("enable_kfs_auto_chassis: ") +
       (r1_init_parameter_.enable_kfs_auto_chassis ? "true" : "false") + "\n";
  RCLCPP_INFO(this->get_logger(), "received /r1_init_parameter:\n%s", s.c_str());
  r1_log_info("Initialize received: zone=%s", r1_init_parameter_.zone.c_str());
  if (r1_init_parameter_.zone != zone_) {
    r1_log_error("ゾーンエラー");
    r1_log_error(
      "Received zone '%s' does not match expected zone '%s'", r1_init_parameter_.zone.c_str(),
      zone_.c_str());
  }
}

void R1MainNode::r1_collect_kfs_callback(const r1_msgs::msg::R1CollectKfs::SharedPtr msg)
{
  std::string s;
  if (msg->kfs_mechanism_type.size() != msg->forest_order.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Size mismatch in /r1_collect_kfs: kfs_mechanism_type has size %zu, but "
      "forest_order has size %zu.",
      msg->kfs_mechanism_type.size(), msg->forest_order.size());
    return;
  }
  r1_collect_kfs_ = *msg;
  received_r1_collect_kfs_ = true;
  for (int i = 0; i < (int)r1_collect_kfs_.kfs_mechanism_type.size(); i++) {
    s += r1_collect_kfs_.kfs_mechanism_type[i] + ": " +
         std::to_string(r1_collect_kfs_.forest_order[i]) + "\n";
    int forest = r1_collect_kfs_.forest_order[i];
    if (forest >= 1 && forest <= 12) {
      kfs_already_collected_[forest - 1] = false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "received /r1_collect_kfs:\n%s", s.c_str());
  r1_log_info("Collect KFS order received");
}

void R1MainNode::apply_r1_kfs_mechanism_ref(R1KfsMechanismRef ref)
{
  // 回収プランがアクティブな場合、step4と同様にzone/is_innerに基づいてYAWを回転させる
  const bool is_collect_active = kfs_auto_collect_plan_.status != KfsAutoCollectStatus::NONE;
  const bool is_inner = kfs_auto_collect_plan_.status == KfsAutoCollectStatus::INNER_ACTIVE;

  auto apply_fyaw = [&]() {
    if (!is_collect_active) return;
    if (zone_ == "blue" && is_inner)
      kfs_fyaw_move_front_mech_lock();
    else if (zone_ == "blue" && !is_inner)
      kfs_fyaw_move_rear_mech_lock();
    else if (zone_ == "red" && is_inner)
      kfs_fyaw_move_rear_mech_lock();
    else if (zone_ == "red" && !is_inner)
      kfs_fyaw_move_front_mech_lock();
  };

  auto apply_ryaw = [&]() {
    if (!is_collect_active) return;
    if (zone_ == "blue" && is_inner)
      kfs_ryaw_move_front_mech_lock();
    else if (zone_ == "blue" && !is_inner)
      kfs_ryaw_move_rear_mech_lock();
    else if (zone_ == "red" && is_inner)
      kfs_ryaw_move_rear_mech_lock();
    else if (zone_ == "red" && !is_inner)
      kfs_ryaw_move_front_mech_lock();
  };

  // アクチュエータを動かす
  // FKFS
  if (ref == R1KfsMechanismRef::FKFS_RACK) {
    kfs_fx_pos_ref(KFS_FX_PUT_POS);
    kfs_fz_pos_ref(KFS_FZ_PUT_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_HIGH) {
    kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
    kfs_fz_pos_ref(KFS_FZ_HIGH_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_fyaw();
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    kfs_manual_front_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_front_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_MIDDLE) {
    kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
    kfs_fz_pos_ref(KFS_FZ_MIDDLE_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_fyaw();
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    kfs_manual_front_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_front_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_LOW) {
    kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
    kfs_fz_pos_ref(KFS_FZ_LOW_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_fyaw();
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    kfs_manual_front_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_front_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_GROUND) {
    kfs_fx_pos_ref(KFS_FX_GROUND_POS);
    kfs_fz_pos_ref(KFS_FZ_GROUND_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_STORAGE) {
    kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
    kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::FKFS_COLLECT_START_POS) {
    kfs_fx_pos_ref(KFS_FX_START_POS);
    kfs_fz_pos_ref(KFS_FZ_START_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
    kfs_front_pump(1.0);
    kfs_front_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  }
  // RKFS
  else if (ref == R1KfsMechanismRef::RKFS_RACK) {
    kfs_rx_pos_ref(KFS_RX_PUT_POS);
    kfs_rz_pos_ref(KFS_RZ_PUT_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_HIGH) {
    kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
    kfs_rz_pos_ref(KFS_RZ_HIGH_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_ryaw();
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    kfs_manual_rear_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_rear_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_MIDDLE) {
    kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
    kfs_rz_pos_ref(KFS_RZ_MIDDLE_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_ryaw();
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    kfs_manual_rear_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_rear_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_LOW) {
    kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
    kfs_rz_pos_ref(KFS_RZ_LOW_POS);
    // 回収プランに基づいてYAWを回転させる
    apply_ryaw();
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    kfs_manual_rear_pressure_wait_ = ENABLE_R1_KFS_MECHANISM_REF_PRESSURE_SENSOR;
    kfs_manual_rear_pressure_detected_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_GROUND) {
    kfs_rx_pos_ref(KFS_RX_GROUND_POS);
    kfs_rz_pos_ref(KFS_RZ_GROUND_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_STORAGE) {
    kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
    kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else if (ref == R1KfsMechanismRef::RKFS_COLLECT_START_POS) {
    kfs_rx_pos_ref(KFS_RX_START_POS);
    kfs_rz_pos_ref(KFS_RZ_START_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
    kfs_rear_pump(1.0);
    kfs_rear_valve(false);
    // 圧力センサ待機を無効化。
    kfs_manual_front_pressure_wait_ = false;
    kfs_manual_rear_pressure_wait_ = false;
  } else {
    // このときはエラー
    RCLCPP_ERROR(this->get_logger(), "Unknown R1KfsMechanismRef value: %d", static_cast<int>(ref));
    return;
  }
  // ログを出力
  auto enum_name = magic_enum::enum_name(ref);
  std::string s{enum_name};
  RCLCPP_INFO(
    this->get_logger(), "Applied R1KfsMechanismRef = %d(%s)", static_cast<int>(ref), s.c_str());
}

void R1MainNode::r1_kfs_mechanism_ref_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  r1_kfs_mechanism_ref_ = msg->data;
  R1KfsMechanismRef ref = static_cast<R1KfsMechanismRef>(r1_kfs_mechanism_ref_);
  auto enum_name = magic_enum::enum_name(ref);
  if (enum_name.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "received /r1_kfs_mechanism_ref = %d (unknown enum value)",
      r1_kfs_mechanism_ref_);
    return;
  }
  std::string s{enum_name};
  RCLCPP_INFO(
    this->get_logger(), "received /r1_kfs_mechanism_ref = %d(%s)", r1_kfs_mechanism_ref_,
    s.c_str());

  apply_r1_kfs_mechanism_ref(ref);
}

void R1MainNode::update_kfs_manual_pressure_sensor(void)
{
  auto check = [&](
                 bool & wait, bool & detected, rclcpp::Time & start_time, bool raw_switch,
                 R1KfsMechanismRef storage_ref) {
    if (!wait) return;
    // 吸着時にセンサがOFFになるため論理を反転
    const bool current = !raw_switch;
    if (current && !detected) {
      start_time = this->now();
    }
    detected = current;
    if (detected && (this->now() - start_time).seconds() > PRESSURE_SENSOR_TIME_THRESHOLD) {
      RCLCPP_INFO(
        this->get_logger(), "manual kfs pressure detected, moving to storage (%d)",
        static_cast<int>(storage_ref));
      apply_r1_kfs_mechanism_ref(storage_ref);
      // apply_r1_kfs_mechanism_ref内でwait=falseになるが、明示的にリセットする
      wait = false;
    }
  };

  check(
    kfs_manual_front_pressure_wait_, kfs_manual_front_pressure_detected_,
    kfs_manual_front_pressure_start_time_, front_pressure_switch_status_,
    R1KfsMechanismRef::FKFS_STORAGE);
  check(
    kfs_manual_rear_pressure_wait_, kfs_manual_rear_pressure_detected_,
    kfs_manual_rear_pressure_start_time_, rear_pressure_switch_status_,
    R1KfsMechanismRef::RKFS_STORAGE);
}

void R1MainNode::r1_retry_collect_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "received /r1_retry_collect");
  // KFS回収関連のパラメータをリセットする
  reset_kfs_auto_collect_tracking();
  // KFS回収機構を回収初期位置に移動
  // 初期化動作を行うときは、ポンプを止めると既に保持しているKFSを落としてしまう。
  // そのため引数をtrueにすることで、ポンプは動かしたままにする。
  // またリトライのときはrollは回転しないので、push_valveは操作しない
  kfs_collect_start_act(true, false);
}

void R1MainNode::r1_collect_3rd_kfs_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  r1_collect_3rd_kfs_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "received /r1_collect_3rd_kfs = %d", r1_collect_3rd_kfs_);
  // とりあえず、KFS回収機構を回収初期位置に移動
  // ポンプは一旦止める
  // rollは回転しないので、push_valveは操作しない
  kfs_collect_start_act(false, false);
  // TODO: 今後3つ目回収に必要な処理をかくこと
}

void R1MainNode::r1_initialize_all_actuator_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(this->get_logger(), "received /r1_initialize_all_actuator");
  detect_origin_all_actuator();
}

void R1MainNode::r1_aruco_marker_id_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  // arucoマーカの数が増えて管理が面倒なので、送られてきた数字をPublishする実装に変更
  if (msg->data == DEFAULT_ARUCO_MARKER_ID) {
    // タイマーが存在していた場合はリセット
    if (aruco_marker_timer_) {
      aruco_marker_timer_->cancel();
    }

    publish_all_aruco_marker_id(msg->data);
  } else if (msg->data == SPEAR_COMBINE_ARUCO_MARKER_ID) {
    // タイマーが存在していた場合はリセット
    if (aruco_marker_timer_) {
      aruco_marker_timer_->cancel();
    }

    // やり合体終了の合図はタイマーでDEFAULT_ARUCO_MARKER_IDに戻さない
    // 理由はarucoマーカーが読めなかったときに表示し続けたいため
    publish_all_aruco_marker_id(msg->data);
  } else {
    // それ以外のときはarucoマーカを表示し、一定時間後にDEFAULT_ARUCO_MARKER_IDに戻す
    if (aruco_marker_timer_) {
      aruco_marker_timer_->cancel();
    }

    publish_all_aruco_marker_id(msg->data);

    aruco_marker_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(ARUCO_MARKER_RESET_TIME), [this]() {
        publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
        r1_log_info("aruco デフォ(リセット)");
        aruco_marker_timer_->cancel();
      });
  }
}

void R1MainNode::publish_r1_machine_initialize(void)
{
  std_msgs::msg::Empty msg;
  r1_machine_initialize_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish /r1_machine_initialize");
}

void R1MainNode::initialize_lidar_lifecycle_clients(void)
{
  const auto node_names = this->declare_parameter<std::vector<std::string>>(
    "lidar_lifecycle_node_names", {"urg_node2_1", "urg_node2_2"});

  for (auto node_name : node_names) {
    if (node_name.empty()) {
      continue;
    }
    if (node_name.front() != '/') {
      node_name = "/" + node_name;
    }

    LifecycleClientInterface client;
    client.node_name = node_name;
    client.change_state_client =
      this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
    lidar_lifecycle_clients_.push_back(client);
  }
}

void R1MainNode::request_lidar_lifecycle_activation(void)
{
  for (const auto & client : lidar_lifecycle_clients_) {
    if (!client.change_state_client->service_is_ready()) {
      RCLCPP_WARN(
        this->get_logger(), "LiDAR lifecycle service is not ready: %s/change_state",
        client.node_name.c_str());
      continue;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    client.change_state_client->async_send_request(
      request, [this, node_name = client.node_name](
                 rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture result) {
        handle_lidar_activate_response(node_name, result);
      });
    RCLCPP_INFO(this->get_logger(), "Sent LiDAR activate request to %s", client.node_name.c_str());
  }
}

void R1MainNode::handle_lidar_activate_response(
  const std::string & node_name,
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
{
  if (future.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Requested LiDAR activate for %s", node_name.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "LiDAR activate request was rejected by %s", node_name.c_str());
  }
}

void R1MainNode::invalidate_led_cache(void)
{
  has_last_led_color_ = false;
  has_last_led_fkfs_color_ = false;
  has_last_led_rkfs_color_ = false;
}

void R1MainNode::r1_machine_initialize_done_callback(const std_msgs::msg::Empty::SharedPtr)
{
  initialize_done_time_ = this->now();
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
  // 前回のタイマーをすべてキャンセル
  if (initialpose_publish_timer_) initialpose_publish_timer_->cancel();
  if (initialpose_retry1_timer_) initialpose_retry1_timer_->cancel();
  if (initialpose_retry2_timer_) initialpose_retry2_timer_->cancel();

  // initialposeをpublishする共通処理
  auto publish_pose = [this, x, y, yaw]() {
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
  };

  // 1回目（delay_sec後）: TFログもスケジュール
  initialpose_publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(delay_sec), [this, publish_pose]() {
      publish_pose();
      schedule_initialpose_tf_log();
      if (initialpose_publish_timer_) initialpose_publish_timer_->cancel();
    });

  // 2回目（LiDAR起動完了タイミングに合わせた再送）
  initialpose_retry1_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(initialpose_retry1_delay_sec_), [this, publish_pose]() {
      publish_pose();
      if (initialpose_retry1_timer_) initialpose_retry1_timer_->cancel();
    });

  // 3回目（より確実に届けるための再送）
  initialpose_retry2_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(initialpose_retry2_delay_sec_), [this, publish_pose]() {
      publish_pose();
      if (initialpose_retry2_timer_) initialpose_retry2_timer_->cancel();
    });
}

void R1MainNode::schedule_initialpose_tf_log(void)
{
  if (initialpose_tf_log_timer_) {
    initialpose_tf_log_timer_->cancel();
  }

  const double delay_sec = std::max(0.0, initialpose_tf_log_delay_sec_);
  initialpose_tf_log_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(delay_sec), [this]() {
      log_initialpose_tf_once();
      if (initialpose_tf_log_timer_) {
        initialpose_tf_log_timer_->cancel();
      }
    });
}

void R1MainNode::log_initialpose_tf_once(void)
{
  const double elapsed_sec = initialize_done_time_.nanoseconds() > 0
                               ? (this->now() - initialize_done_time_).seconds()
                               : -1.0;
  RCLCPP_INFO(this->get_logger(), "Logging TF %.3f sec after r1_initialize_done", elapsed_sec);
  log_transform_once("map", "odom");
  log_transform_once("map", "base_link");
}

void R1MainNode::log_transform_once(
  const std::string & target_frame, const std::string & source_frame)
{
  try {
    const auto transform =
      tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    const auto & translation = transform.transform.translation;
    const double yaw = tf2::getYaw(transform.transform.rotation);
    constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
    RCLCPP_INFO(
      this->get_logger(),
      "Current TF %s->%s: stamp=%d.%09u, x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad (%.2f deg)",
      target_frame.c_str(), source_frame.c_str(), transform.header.stamp.sec,
      transform.header.stamp.nanosec, translation.x, translation.y, translation.z, yaw,
      yaw * rad_to_deg);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not log TF %s->%s after initialpose: %s", target_frame.c_str(),
      source_frame.c_str(), ex.what());
  }
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
    case ChassisAct::ACT5_START:
    case ChassisAct::ACT5:
      stop_ref = ChassisAct::ACT5_FINISH;
      break;
    case ChassisAct::ACT0_FINISH:
    case ChassisAct::ACT1_FINISH:
    case ChassisAct::ACT2_FINISH:
    case ChassisAct::ACT3_FINISH:
    case ChassisAct::ACT4_FINISH:
    case ChassisAct::ACT5_FINISH:
    case ChassisAct::NONE:
    default:
      stop_ref = ChassisAct::NONE;
      break;
  }

  publish_chassis_act_ref(stop_ref);
}

void R1MainNode::publish_chassis_act_pause(void)
{
  publish_chassis_act_ref(ChassisAct::ACT_PAUSE);
  RCLCPP_INFO(this->get_logger(), "chassis act pause");
}

void R1MainNode::publish_chassis_act_resume(void)
{
  publish_chassis_act_ref(ChassisAct::ACT_RESUME);
  RCLCPP_INFO(this->get_logger(), "chassis act resume");
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

void R1MainNode::publish_all_aruco_marker_id(int id)
{
  publish_spear_red_aruco_marker_id(id);
  publish_spear_blue_aruco_marker_id(id);
  publish_r2_lift_lower_aruco_marker_id(id);
  publish_r2_lift_upper_aruco_marker_id(id);
  RCLCPP_INFO(this->get_logger(), "publish all aruco marker id: %d", id);
}

void R1MainNode::publish_spear_red_aruco_marker_id(int id)
{
  std_msgs::msg::Int32 msg;
  msg.data = id;
  spear_red_aruco_marker_id_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish spear red aruco marker id: %d", id);
}

void R1MainNode::publish_spear_blue_aruco_marker_id(int id)
{
  std_msgs::msg::Int32 msg;
  msg.data = id;
  spear_blue_aruco_marker_id_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish spear blue aruco marker id: %d", id);
}

void R1MainNode::publish_r2_lift_lower_aruco_marker_id(int id)
{
  std_msgs::msg::Int32 msg;
  msg.data = id;
  r2_lift_lower_aruco_marker_id_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish r2 lift lower aruco marker id: %d", id);
}

void R1MainNode::publish_r2_lift_upper_aruco_marker_id(int id)
{
  std_msgs::msg::Int32 msg;
  msg.data = id;
  r2_lift_upper_aruco_marker_id_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "publish r2 lift upper aruco marker id: %d", id);
}

// void R1MainNode::publish_r1_log(const std::string & message)
// {
//   std_msgs::msg::String msg;
//   msg.data = message;
//   r1_log_message_publisher_->publish(msg);
// }

void R1MainNode::r1_log_info(const char * fmt, ...)
{
  // printf と同じ書式でフォーマットしてバッファに展開
  va_list args;
  va_start(args, fmt);
  char buf[512];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  // ROS ログと /r1_log_message_info トピックの両方に出力
  RCLCPP_INFO(this->get_logger(), "%s", buf);
  std_msgs::msg::String msg;
  msg.data = buf;
  r1_log_message_info_publisher_->publish(msg);
}

void R1MainNode::r1_log_warn(const char * fmt, ...)
{
  // printf と同じ書式でフォーマットしてバッファに展開
  va_list args;
  va_start(args, fmt);
  char buf[512];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  // ROS ログと /r1_log_message_warn トピックの両方に出力
  RCLCPP_WARN(this->get_logger(), "%s", buf);
  std_msgs::msg::String msg;
  msg.data = buf;
  r1_log_message_warn_publisher_->publish(msg);
}

void R1MainNode::r1_log_error(const char * fmt, ...)
{
  // printf と同じ書式でフォーマットしてバッファに展開
  va_list args;
  va_start(args, fmt);
  char buf[512];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  // ROS ログと /r1_log_message_error トピックの両方に出力
  RCLCPP_ERROR(this->get_logger(), "%s", buf);
  std_msgs::msg::String msg;
  msg.data = buf;
  r1_log_message_error_publisher_->publish(msg);
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
    auto now = this->now();
    if ((now - last_tf_warn_time_).seconds() >= 1.0) {
      last_tf_warn_time_ = now;
      // r1_log_warn("map->base_link 利用不可");
      RCLCPP_WARN(
        this->get_logger(),
        "map->base_link is not available yet. Cannot publish pending robot move.");
    }
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
  is_act_paused_ = false;
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
  for (int i = 0; i < 12; i++) {
    wall_sensor_detect_start_time_[i] = this->now();
    wall_sensor_detected_[i] = false;
    // odom座標系
    wall_detect_pos_[i] = nav_msgs::msg::Odometry();
  }
  for (int i = 0; i < 2; i++) {
    pressure_sensor_detect_start_time_[i] = this->now();
    pressure_sensor_detected_[i] = false;
  }
  auto_collect_kfs_fkfs_step_ = DEFAULT_STEP;
  auto_collect_kfs_rkfs_step_ = DEFAULT_STEP;
  for (int i = 0; i < 12; i++) {
    kfs_already_collected_[i] = false;
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
  kfs_collect_start_act();
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
  // operation_modeの変化をスマホにpublish
  const auto current_op_mode = state_machine_->get_current_state().operation_mode;
  if (current_op_mode != last_published_operation_mode_) {
    std_msgs::msg::Int32 op_mode_msg;
    op_mode_msg.data = static_cast<int32_t>(current_op_mode);
    r1_operation_mode_publisher_->publish(op_mode_msg);
    last_published_operation_mode_ = current_op_mode;
  }
  // map->base_link TF が来ていない間は一定間隔で警告を出す
  if (!is_localization_ready()) {
    auto now = this->now();
    if ((now - last_tf_warn_time_).seconds() >= 1.0) {
      last_tf_warn_time_ = now;
      r1_log_warn("map->base_link 利用不可");
    }
  }
  // タスクを実行
  // ps4_->print_data();
  main_task();
  update_kfs_manual_pressure_sensor();
  update_kfs_led_status();
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

void R1MainNode::kfs_fx_set_pos(double pos) { set_position_axis("kfs_fx", pos); }

void R1MainNode::kfs_fz_set_pos(double pos) { set_position_axis("kfs_fz", pos); }

void R1MainNode::kfs_fyaw_set_angle(double angle) { set_position_axis("kfs_fyaw", angle); }

void R1MainNode::kfs_rx_set_pos(double pos) { set_position_axis("kfs_rx", pos); }

void R1MainNode::kfs_rz_set_pos(double pos) { set_position_axis("kfs_rz", pos); }

void R1MainNode::kfs_ryaw_set_angle(double angle) { set_position_axis("kfs_ryaw", angle); }

void R1MainNode::r2_flift_pos_ref(double pos) { publish_position_axis("r2_flift", pos); }

void R1MainNode::r2_rlift_pos_ref(double pos) { publish_position_axis("r2_rlift", pos); }

void R1MainNode::r2_flift_set_pos(double pos) { set_position_axis("r2_flift", pos); }

void R1MainNode::r2_rlift_set_pos(double pos) { set_position_axis("r2_rlift", pos); }

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

void R1MainNode::spear_y_pos_ref(double pos) { publish_position_axis("spear_y", pos); }

void R1MainNode::spear_roll1_pos_ref(double angle) { publish_position_axis("spear_roll1", angle); }

void R1MainNode::spear_roll2_pos_ref(double angle) { publish_position_axis("spear_roll2", angle); }

void R1MainNode::spear_y_set_pos(double pos) { set_position_axis("spear_y", pos); }

void R1MainNode::spear_roll1_set_angle(double angle) { set_position_axis("spear_roll1", angle); }

void R1MainNode::spear_roll2_set_angle(double angle) { set_position_axis("spear_roll2", angle); }

void R1MainNode::spear_y_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_y", speed);
}

void R1MainNode::spear_roll1_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_roll1", speed);
}

void R1MainNode::spear_roll2_speed_ref(double speed)
{
  publish_position_axis_speed_ref("spear_roll2", speed);
}

void R1MainNode::spear_y_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_y"); }
void R1MainNode::spear_roll1_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_roll1"); }
void R1MainNode::spear_roll2_speed_mode_stop(void) { stop_position_axis_speed_mode("spear_roll2"); }

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

// 大槻機構
void R1MainNode::spear_hand1_valve(bool on)
{
  publish_gpio_pwm_output("spear_hand1_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_hand2_valve(bool on)
{
  publish_gpio_pwm_output("spear_hand2_valve", on ? 1.0 : 0.0);
}

void R1MainNode::spear_hand_push_valve(bool on)
{
  publish_gpio_pwm_output("spear_hand_push_valve", on ? 1.0 : 0.0);
}

void R1MainNode::kfs_robot_start_act(bool is_start_zone)
{
  kfs_fx_pos_ref(KFS_FX_START_POS);
  kfs_rx_pos_ref(KFS_RX_START_POS);
  kfs_fz_pos_ref(KFS_FZ_START_POS);
  kfs_rz_pos_ref(KFS_RZ_START_POS);
  if (is_start_zone) {
    // スタートゾーンのときは展開制限範囲内に収めるためにメカロックにぶつける
    kfs_fyaw_move_rear_mech_lock();
    kfs_ryaw_move_rear_mech_lock();
  } else {
    // スタートゾーンから出たら、yawは90度にする
    kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
    kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
  }
}

void R1MainNode::kfs_collect_start_act(bool enable_pump, bool enable_push_valve)
{
  auto ROLL_DELAY = 300ms;
  auto PUSH_VALVE_DELAY = 1000ms;

  if (kfs_collect_start_act_roll_timer_) {
    kfs_collect_start_act_roll_timer_->cancel();
  }

  if (kfs_collect_start_act_push_valve_timer_) {
    kfs_collect_start_act_push_valve_timer_->cancel();
  }

  // spear_yを移動。push_valveをtrueにし、槍回収機構をKFS回収時用の高さに移動する
  spear_y_pos_ref(SPEAR_Y_COLLECT_KFS_POS);
  // arucoマーカをもとに戻す
  publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
  r1_log_info("aruco デフォ");
  if (enable_push_valve) {
    // hand_push_valveをtrueにし、やり回収機構を押し出す
    spear_hand_push_valve(true);
  }

  // 2. 少ししたら、rollを垂直にする
  kfs_collect_start_act_roll_timer_ = this->create_wall_timer(ROLL_DELAY, [this]() {
    spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
    spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
    if (kfs_collect_start_act_roll_timer_) {
      kfs_collect_start_act_roll_timer_->cancel();
    }
  });

  kfs_collect_start_act_push_valve_timer_ = this->create_wall_timer(
    ROLL_DELAY + PUSH_VALVE_DELAY, [this, enable_pump, enable_push_valve]() {
      // KFS回収用アクチュエータを回収位置位置に移動
      // STORAGE_POSとSTART_POSのどっちにすればいいかよくわからないので、適当にSTART_POSにしている
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
      kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
      kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
      bool is_inner =
        (chassis_act_status_ == ChassisAct::ACT2 || chassis_act_status_ == ChassisAct::ACT3);
      bool is_outer = chassis_act_status_ == ChassisAct::ACT4;
      if (zone_ == "blue" && is_inner) {
        // front
        kfs_fyaw_move_front_mech_lock();
        kfs_ryaw_move_front_mech_lock();
      } else if (zone_ == "blue" && is_outer) {
        // rear
        kfs_fyaw_move_rear_mech_lock();
        kfs_ryaw_move_rear_mech_lock();
      } else if (zone_ == "red" && is_inner) {
        // rear
        kfs_fyaw_move_rear_mech_lock();
        kfs_ryaw_move_rear_mech_lock();
      } else if (zone_ == "red" && is_outer) {
        // front
        kfs_fyaw_move_front_mech_lock();
        kfs_ryaw_move_front_mech_lock();
      }
      if (enable_pump) {
        kfs_front_pump(1.0);
        kfs_rear_pump(1.0);
      } else {
        kfs_front_pump(0.0);
        kfs_rear_pump(0.0);
      }
      if (enable_push_valve) {
        // hand_push_valveをoffにし、やり回収機構を引っ込める
        spear_hand_push_valve(false);
      }
      if (kfs_collect_start_act_push_valve_timer_) {
        kfs_collect_start_act_push_valve_timer_->cancel();
      }
    });
}

void R1MainNode::stop_actuator(void)
{
  // 速度制御のモータ指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
  // r2_flift_speed_ref(0.0);
  // r2_rlift_speed_ref(0.0);
  // r2_flift_speed_mode_stop();
  // r2_rlift_speed_mode_stop();
  // 真空ポンプを止める
  kfs_front_pump(0.0);
  kfs_rear_pump(0.0);
  // KFS回収電磁弁を止める
  kfs_front_valve(false);
  kfs_rear_valve(false);
  // やりの電磁弁を止める
  spear_hand1_valve(false);
  spear_hand2_valve(false);
  spear_hand_push_valve(false);
}

// --- 各状態のタスク ---
void R1MainNode::idle_task(void)
{
  // 速度指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
}

void R1MainNode::emergency_task(void) {}

void R1MainNode::detect_origin_all_actuator(void)
{
  // 電磁弁などの各種アクチュエータを停止
  stop_actuator();
  // 原点検出処理
  kfs_fx_detect_origin();
  kfs_fz_detect_origin();
  kfs_fyaw_detect_origin();
  kfs_rx_detect_origin();
  kfs_rz_detect_origin();
  kfs_ryaw_detect_origin();
  r2_flift_detect_origin();
  r2_rlift_detect_origin();
  spear_y_detect_origin();
  spear_roll1_set_angle(0.0);
  spear_roll2_set_angle(0.0);
}

void R1MainNode::manual_mode1_detect_origin(void)
{
  auto & hand_valve_step = manual_mode2_hand_valve_step_;

  if (ps4_->is_pushed_up()) {
    kfs_rx_detect_origin();
  }

  if (ps4_->is_pushed_right()) {
    kfs_rz_detect_origin();
  }

  if (ps4_->is_pushed_down()) {
    kfs_ryaw_detect_origin();
  }

  if (ps4_->is_pushed_left()) {
    spear_y_detect_origin();
    // spear_roll1_set_angle(0.0);
    // spear_roll2_set_angle(0.0);
  }

  if (ps4_->is_pushed_triangle()) {
    kfs_fx_detect_origin();
  }

  if (ps4_->is_pushed_circle()) {
    kfs_fz_detect_origin();
  }

  if (ps4_->is_pushed_cross()) {
    kfs_fyaw_detect_origin();
  }

  if (ps4_->is_pushed_square()) {
    spear_roll1_set_angle(1.5707963267948966);
    spear_roll2_set_angle(1.5707963267948966);
    // spear_roll1_detect_origin();
    // spear_roll2_detect_origin();
  }

  if (ps4_->is_pushed_l1()) {
    kfs_robot_start_act(true);
    spear_y_pos_ref(SPEAR_Y_COLLECT1_POS);
    spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
    spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
    spear_hand1_valve(true);
    spear_hand2_valve(true);
    // hand_push_valveはfalseにする。理由は展開制限に引っかかるから
    spear_hand_push_valve(false);
    r1_log_info("初期化動作完了");
  }

  if (ps4_->is_pushed_r1()) {
    if (hand_valve_step == 1) {
      spear_hand1_valve(true);
      spear_hand2_valve(true);
      hand_valve_step = 2;
    } else if (hand_valve_step == 2) {
      spear_hand1_valve(false);
      spear_hand2_valve(false);
      hand_valve_step = 1;
    }
    // spear_y_detect_origin();
    // spear_roll1_set_angle(0.0);
    // spear_roll2_set_angle(0.0);
  }

  if (ps4_->is_pushed_l2()) {
    // r2_rlift_detect_origin();
    r2_rlift_set_pos(0.0);
  }

  if (ps4_->is_pushed_r2()) {
    // r2_flift_detect_origin();
    r2_flift_set_pos(0.0);
  }
}

void R1MainNode::manual_mode2_collect_pole_task(void)
{
  int & step = manual_mode2_collect_pole_task_step_;
  // RCLCPP_INFO(this->get_logger(), "manual_mode2_collect_pole_task step: %d", step);

  r1_log_info("mode2 ポール step%d", step);
  if (step == 1) {
    kfs_robot_start_act();
    spear_y_pos_ref(SPEAR_Y_COLLECT1_POS);
    spear_hand1_valve(true);
    spear_hand2_valve(true);
    spear_hand_push_valve(true);
    // spear_rollは角度調整が行われていた場合は指令値を送らないようにする
    // この処理はmode2_collect_pole_taskのこのときにしか行われない特例措置
    bool roll1_adjusted =
      (spear_roll1_position_ref_ - SPEAR_ROLL_ADJUST_ANGLE) < SPEAR_ROLL1_VERTICAL_ANGLE &&
      (SPEAR_ROLL1_VERTICAL_ANGLE < spear_roll1_position_ref_ + SPEAR_ROLL_ADJUST_ANGLE);
    bool roll2_adjusted =
      (spear_roll2_position_ref_ - SPEAR_ROLL_ADJUST_ANGLE) < SPEAR_ROLL2_VERTICAL_ANGLE &&
      (SPEAR_ROLL2_VERTICAL_ANGLE < spear_roll2_position_ref_ + SPEAR_ROLL_ADJUST_ANGLE);
    // どちらかが微調整されていなければ指令値を送信
    if (!roll1_adjusted || !roll2_adjusted) {
      spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
      spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
    }
    step++;
  } else if (step == 2) {
    spear_hand1_valve(false);
    spear_hand2_valve(false);
    step++;
  } else if (step == 3) {
    spear_y_pos_ref(SPEAR_Y_COLLECT2_POS);
    step++;
  } else if (step == 4) {
    spear_hand_push_valve(false);
    r1_log_info("mode2 ポール 完了");
    step = 1;
  }
}

void R1MainNode::manual_mode2_pole(void)
{
  auto & hand_valve_step = manual_mode2_hand_valve_step_;
  auto & push_valve_step = manual_mode2_push_valve_step_;
  if (ps4_->is_pushed_up()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_y_pos_ref(spear_y_position_ref_ + 0.002);
    } else {
      // 通常調整
      spear_y_pos_ref(spear_y_position_ref_ + 0.01);
    }
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ + 0.01);
      spear_roll2_pos_ref(spear_roll2_position_ref_ + 0.01);
    } else {
      // 通常調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ + 0.03);
      spear_roll2_pos_ref(spear_roll2_position_ref_ + 0.03);
    }
  }

  if (ps4_->is_pushed_down()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_y_pos_ref(spear_y_position_ref_ - 0.002);
    } else {
      // 通常調整
      spear_y_pos_ref(spear_y_position_ref_ - 0.01);
    }
  }

  if (ps4_->is_pushed_left()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ - 0.01);
      spear_roll2_pos_ref(spear_roll2_position_ref_ - 0.01);
    } else {
      // 通常調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ - 0.03);
      spear_roll2_pos_ref(spear_roll2_position_ref_ - 0.03);
    }
  }

  if (ps4_->is_pushed_triangle()) {
    // ポール回収
    manual_mode2_collect_pole_task();
  }

  if (ps4_->is_pushed_circle()) {
  }

  if (ps4_->is_pushed_cross()) {
  }

  if (ps4_->is_pushed_square()) {
  }

  if (ps4_->is_pushed_l1()) {
    if (push_valve_step == 1) {
      spear_hand_push_valve(true);
      push_valve_step++;
    } else if (push_valve_step == 2) {
      spear_hand_push_valve(false);
      push_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_r1()) {
    if (hand_valve_step == 1) {
      spear_hand1_valve(true);
      spear_hand2_valve(true);
      hand_valve_step++;
    } else if (hand_valve_step == 2) {
      spear_hand1_valve(false);
      spear_hand2_valve(false);
      hand_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_l2()) {
    // 微調整トリガー
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }
}

void R1MainNode::manual_mode3_init_move_task(int n)
{
  (void)n;
  // 一旦何もしない
}

void R1MainNode::manual_mode3_make_spear_task(int n)
{
  // 千田機構だったときの名残で現在は使用していない
  (void)n;

  int & step = manual_mode3_make_spear_task_step_;

  auto ROLL_DELAY = 300ms;
  auto PUSH_VALVE_DELAY = 600ms;
  // RCLCPP_INFO(this->get_logger(), "manual_mode3_make_spear_task step: %d", step);
  r1_log_info("mode3 やり step%d", step);
  if (step == 1) {
    if (manual_mode3_roll_timer_) {
      manual_mode3_roll_timer_->cancel();
    }
    if (manual_mode3_push_valve_timer_) {
      manual_mode3_push_valve_timer_->cancel();
    }

    // 1. spear_yを動かす。
    // hand_push_valveを動かし、機構を前に出す。
    // arucoマーカーは0をpublishする
    spear_y_pos_ref(SPEAR_Y_MAKE_SPEAR_POS);
    spear_hand_push_valve(true);
    publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
    r1_log_info("aruco デフォ");

    // 2. 少し遅延を入れて、rollを横向きにする
    manual_mode3_roll_timer_ = this->create_wall_timer(ROLL_DELAY, [this]() {
      if (zone_ == "red") {
        spear_roll1_pos_ref(SPEAR_ROLL1_INV_HORIZONTAL_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_INV_HORIZONTAL_ANGLE);
      } else {
        spear_roll1_pos_ref(SPEAR_ROLL1_HORIZONTAL_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_HORIZONTAL_ANGLE);
      }

      if (manual_mode3_roll_timer_) {
        manual_mode3_roll_timer_->cancel();
      }
    });

    // 3. さらに遅延を入れて、hand_push_valveをoffにし、機構を引っ込める
    manual_mode3_push_valve_timer_ =
      this->create_wall_timer(ROLL_DELAY + PUSH_VALVE_DELAY, [this]() {
        spear_hand_push_valve(false);
        if (manual_mode3_push_valve_timer_) {
          manual_mode3_push_valve_timer_->cancel();
        }
      });
    step++;
  } else if (step == 2) {
    publish_all_aruco_marker_id(SPEAR_COMBINE_ARUCO_MARKER_ID);
    r1_log_info("aruco やり合体");
    step++;
  } else if (step == 3) {
    if (manual_mode3_roll_timer_) {
      manual_mode3_roll_timer_->cancel();
    }

    if (manual_mode3_push_valve_timer_) {
      manual_mode3_push_valve_timer_->cancel();
    }

    // 1. push_valveをonにし、機構を前に出す。
    spear_hand_push_valve(true);

    // 2. 少し遅延を入れて、rollを縦向きにする
    manual_mode3_roll_timer_ = this->create_wall_timer(ROLL_DELAY, [this]() {
      spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
      spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
      if (manual_mode3_roll_timer_) {
        manual_mode3_roll_timer_->cancel();
      }
    });

    // 3. さらに遅延を入れて、push_valveをoffにし、機構を引っ込める
    manual_mode3_push_valve_timer_ =
      this->create_wall_timer(ROLL_DELAY + PUSH_VALVE_DELAY, [this]() {
        spear_hand_push_valve(false);
        if (manual_mode3_push_valve_timer_) {
          manual_mode3_push_valve_timer_->cancel();
        }
      });
    step = 1;
    // RCLCPP_INFO(this->get_logger(), "make spear task completed");
    r1_log_info("mode3 やり 完了");
  }
}

void R1MainNode::manual_mode3_spear(void)
{
  auto & hand_valve_step = manual_mode3_hand_valve_step_;
  auto & push_valve_step = manual_mode3_push_valve_step_;

  if (ps4_->is_pushed_up()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      // spear_y_pos_ref(spear_y_position_ref_ + 0.002);
      spear_y_pos_ref(spear_y_position_ref_ + 0.001);
    } else {
      // 通常調整
      // spear_y_pos_ref(spear_y_position_ref_ + 0.01);
      spear_y_pos_ref(spear_y_position_ref_ + 0.005);
    }
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ + 0.01);
      spear_roll2_pos_ref(spear_roll2_position_ref_ + 0.01);
    } else {
      // 通常調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ + 0.03);
      spear_roll2_pos_ref(spear_roll2_position_ref_ + 0.03);
    }
  }

  if (ps4_->is_pushed_down()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      // spear_y_pos_ref(spear_y_position_ref_ - 0.002);
      spear_y_pos_ref(spear_y_position_ref_ + 0.001);
    } else {
      // 通常調整
      spear_y_pos_ref(spear_y_position_ref_ - 0.01);
      spear_y_pos_ref(spear_y_position_ref_ - 0.005);
    }
  }

  if (ps4_->is_pushed_left()) {
    if (ps4_->is_pushing_l2()) {
      // 微調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ - 0.01);
      spear_roll2_pos_ref(spear_roll2_position_ref_ - 0.01);
    } else {
      // 通常調整
      spear_roll1_pos_ref(spear_roll1_position_ref_ - 0.03);
      spear_roll2_pos_ref(spear_roll2_position_ref_ - 0.03);
    }
  }

  if (ps4_->is_pushed_triangle()) {
    // やり組み立て
    manual_mode3_make_spear_task(2);
  }

  if (ps4_->is_pushed_circle()) {
  }

  if (ps4_->is_pushed_cross()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushed_square()) {
  }

  if (ps4_->is_pushed_l1()) {
    if (push_valve_step == 1) {
      spear_hand_push_valve(true);
      push_valve_step++;
    } else if (push_valve_step == 2) {
      spear_hand_push_valve(false);
      push_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_r1()) {
    if (hand_valve_step == 1) {
      spear_hand1_valve(true);
      spear_hand2_valve(true);
      hand_valve_step++;
    } else if (hand_valve_step == 2) {
      spear_hand1_valve(false);
      spear_hand2_valve(false);
      hand_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_l2()) {
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }
}

void R1MainNode::manual_mode4_fkfs(void)
{
  int & fx_step = manual_mode4_fx_step_;
  int & fz_step = manual_mode4_fz_step_;
  int & fyaw_step = manual_mode4_fyaw_step_;
  int & front_pump_step = manual_mode4_front_pump_step_;
  int & l2_r2_trigger_step = manual_mode4_l2_r2_trigger_step_;

  if (ps4_->is_pushed_up()) {
    if (ps4_->is_pushing_l2()) {
      // 上段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::FKFS_HIGH;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
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
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      // 地面に置かれたものを回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::FKFS_GROUND;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
      // put動作
      kfs_fx_pos_ref(KFS_FX_PUT_POS);
      kfs_fz_pos_ref(KFS_FZ_PUT_POS);
      kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
      kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
      kfs_rz_pos_ref(KFS_RZ_PUT_POS);
      kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
      RCLCPP_INFO(this->get_logger(), "moved to front_kfs put position");
    }
  }

  if (ps4_->is_pushed_down()) {
    if (ps4_->is_pushing_l2()) {
      // 下段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::FKFS_LOW;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
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
  }

  if (ps4_->is_pushed_left()) {
    if (ps4_->is_pushing_l2()) {
      // 中段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::FKFS_MIDDLE;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
      // front_pumpを動かす。止めるときは電磁弁も一緒に動く
      if (front_pump_step == 1) {
        kfs_front_pump(1.0);
        kfs_front_valve(false);
        front_pump_step = 2;
      } else {
        kfs_front_pump(0.0);
        kfs_front_valve(true);
        // setTimeout風で電磁弁をOFFにする。
        if (manual_mode4_front_valve_timer_) {
          manual_mode4_front_valve_timer_->cancel();
        }
        manual_mode4_front_valve_timer_ =
          this->create_wall_timer(std::chrono::duration<double>(KFS_VALVE_DELAY_TIME), [this]() {
            kfs_front_valve(false);
            if (manual_mode4_front_valve_timer_) {
              manual_mode4_front_valve_timer_->cancel();
            }
          });
        front_pump_step = 1;
      }
    }
  }

  if (ps4_->is_pushed_triangle()) {
    if (ps4_->is_pushing_l2()) {
      kfs_fyaw_pos_ref(kfs_fyaw_position_ref_ + 0.05);
    } else {
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
  }

  if (ps4_->is_pushed_circle()) {
    if (ps4_->is_pushing_l2()) {
      // kfs_fxの微調整（指令値を増加）
      kfs_fx_pos_ref(kfs_fx_position_ref_ + 0.01);
    } else {
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
  }

  if (ps4_->is_pushed_cross()) {
    if (ps4_->is_pushing_l2()) {
      kfs_fyaw_pos_ref(kfs_fyaw_position_ref_ - 0.05);
    } else {
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
  }

  if (ps4_->is_pushed_square()) {
    if (ps4_->is_pushing_l2()) {
      // kfs_fxの微調整（指令値を減少）
      kfs_fx_pos_ref(kfs_fx_position_ref_ - 0.01);
    } else {
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
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_fzの微調整（指令値を減少）
    kfs_fz_pos_ref(kfs_fz_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    // kfs_fzの微調整（指令値を増加）
    kfs_fz_pos_ref(kfs_fz_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    // 微調整トリガーとして使用
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushing_l2() && ps4_->is_pushing_r2()) {
    if (l2_r2_trigger_step == DEFAULT_STEP) {
      // ストレージ位置に戻す
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
      kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
      kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
      kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
      kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
      // 真空ポンプ関連の指令値は、操縦ミスでKFSを落とすのを防止するため、操作しない。
      l2_r2_trigger_step = 2;
    }
  } else {
    l2_r2_trigger_step = DEFAULT_STEP;
  }
}

void R1MainNode::manual_mode5_rkfs(void)
{
  int & rx_step = manual_mode5_rx_step_;
  int & rz_step = manual_mode5_rz_step_;
  int & ryaw_step = manual_mode5_ryaw_step_;
  int & rear_pump_step = manual_mode5_rear_pump_step_;
  int & l2_r2_trigger_step = manual_mode5_l2_r2_trigger_step_;

  if (ps4_->is_pushed_up()) {
    if (ps4_->is_pushing_l2()) {
      // 上段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::RKFS_HIGH;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
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
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      // 地面に置かれたものを回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::RKFS_GROUND;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
      // put動作
      kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
      kfs_fz_pos_ref(KFS_FZ_PUT_POS);
      kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
      kfs_rx_pos_ref(KFS_RX_PUT_POS);
      kfs_rz_pos_ref(KFS_RZ_PUT_POS);
      kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
      RCLCPP_INFO(this->get_logger(), "moved to rear_kfs put position");
    }
  }

  if (ps4_->is_pushed_down()) {
    if (ps4_->is_pushing_l2()) {
      // 下段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::RKFS_LOW;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
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
  }

  if (ps4_->is_pushed_left()) {
    if (ps4_->is_pushing_l2()) {
      // 中段回収
      R1KfsMechanismRef ref = R1KfsMechanismRef::RKFS_MIDDLE;
      apply_r1_kfs_mechanism_ref(ref);
    } else {
      // rear_pumpを動かす。止めるときは電磁弁も一緒に動く
      if (rear_pump_step == 1) {
        kfs_rear_pump(1.0);
        kfs_rear_valve(false);
        rear_pump_step = 2;
      } else {
        kfs_rear_pump(0.0);
        kfs_rear_valve(true);
        // setTimeout風で電磁弁をOFFにする。
        if (manual_mode5_rear_valve_timer_) {
          manual_mode5_rear_valve_timer_->cancel();
        }
        manual_mode5_rear_valve_timer_ =
          this->create_wall_timer(std::chrono::duration<double>(KFS_VALVE_DELAY_TIME), [this]() {
            kfs_rear_valve(false);
            if (manual_mode5_rear_valve_timer_) {
              manual_mode5_rear_valve_timer_->cancel();
            }
          });
        rear_pump_step = 1;
      }
    }
  }

  if (ps4_->is_pushed_triangle()) {
    if (ps4_->is_pushing_l2()) {
      kfs_ryaw_pos_ref(kfs_ryaw_position_ref_ + 0.05);
    } else {
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
  }

  if (ps4_->is_pushed_circle()) {
    if (ps4_->is_pushing_l2()) {
      // kfs_rxの微調整（指令値を増加）
      kfs_rx_pos_ref(kfs_rx_position_ref_ + 0.01);
    } else {
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
  }

  if (ps4_->is_pushed_cross()) {
    if (ps4_->is_pushing_l2()) {
      kfs_ryaw_pos_ref(kfs_ryaw_position_ref_ - 0.05);
    } else {
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
  }

  if (ps4_->is_pushed_square()) {
    if (ps4_->is_pushing_l2()) {
      // kfs_rxの微調整（指令値を減少）
      kfs_rx_pos_ref(kfs_rx_position_ref_ - 0.01);
    } else {
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
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_rzの微調整（指令値を減少）
    kfs_rz_pos_ref(kfs_rz_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    // kfs_rzの微調整（指令値を増加）
    kfs_rz_pos_ref(kfs_rz_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    // 微調整トリガーとして使用
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushing_l2() && ps4_->is_pushing_r2()) {
    if (l2_r2_trigger_step == DEFAULT_STEP) {
      // 回収初期値に戻す
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
      kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
      kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
      kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
      kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
      // 真空ポンプ関連の指令値は、操縦ミスでKFSを落とすのを防止するため、操作しない。
      l2_r2_trigger_step = 2;
    }
  } else {
    l2_r2_trigger_step = DEFAULT_STEP;
  }
}

void R1MainNode::manual_mode6_r2_lift(void)
{
  // static bool is_speed_mode = false;

  if (ps4_->is_pushed_up()) {
    if (ps4_->is_pushing_l2()) {
      // r2_rliftの微調整（指令値を増加）
      r2_rlift_pos_ref(r2_rlift_position_ref_ + 0.01);
      // 微調整は他とは異なり、現在位置に対して行う
      // r2_rlift_pos_ref(r2_rlift_current_pos_ + 0.01);
    } else {
      publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
      r1_log_info("aruco デフォ");
      r2_flift_pos_ref(R2_FLIFT_UP_POS);
      r2_rlift_pos_ref(R2_RLIFT_UP_POS);
      // r2_flift_move_mech_lock(1);
      // r2_rlift_move_mech_lock(1);
    }
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      // arucoマーカを初期状態にする
      publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
      r1_log_info("aruco デフォ");
    } else {
      // 間違えてボタン押したときのために、aroco_marker_id=0に設定
      publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
      r1_log_info("aruco デフォ");
      kfs_fx_pos_ref(KFS_FX_R2_LIFT_POS);
      kfs_fz_pos_ref(KFS_FZ_R2_LIFT_POS);
      kfs_rx_pos_ref(KFS_RX_R2_LIFT_POS);
      kfs_rz_pos_ref(KFS_RZ_R2_LIFT_POS);
      RCLCPP_INFO(this->get_logger(), "moved to r2_lift position");
      if (manual_mode6_r2_lift_timer_ != nullptr) {
        manual_mode6_r2_lift_timer_->cancel();
      }
      manual_mode6_r2_lift_timer_ = this->create_wall_timer(500ms, [this]() {
        // r2_flift_move_mech_lock(-1);
        // r2_rlift_move_mech_lock(-1);
        r2_flift_pos_ref(R2_FLIFT_NORMAL_POS);
        r2_rlift_pos_ref(R2_RLIFT_NORMAL_POS);
        if (manual_mode6_r2_lift_timer_ != nullptr) {
          manual_mode6_r2_lift_timer_->cancel();
        }
        RCLCPP_INFO(this->get_logger(), "r2 lift stop (timer)");
      });
    }
  }

  if (ps4_->is_pushed_down()) {
    if (ps4_->is_pushing_l2()) {
      // r2_rliftの微調整（指令値を減少）
      r2_rlift_pos_ref(r2_rlift_position_ref_ - 0.01);
    } else {
      // まず最初にR2昇降機構をもとの位置に移動する
      // 理由は微調整後の輪ゴムの伸びの差による誤動作を防止するため
      r2_flift_pos_ref(R2_FLIFT_UP_POS);
      r2_rlift_pos_ref(R2_RLIFT_UP_POS);
      // 次に少ししたら下降する
      if (manual_mode6_r2_lift_timer_ != nullptr) {
        manual_mode6_r2_lift_timer_->cancel();
      }
      manual_mode6_r2_lift_timer_ = this->create_wall_timer(700ms, [this]() {
        r2_flift_pos_ref(R2_FLIFT_DOWN_POS);
        r2_rlift_pos_ref(R2_RLIFT_DOWN_POS);
        if (manual_mode6_r2_lift_timer_ != nullptr) {
          manual_mode6_r2_lift_timer_->cancel();
        }
      });
    }
  }

  // 下ろすときは速度制御でおろしてみる
  // if (ps4_->is_pushed_down() && ps4_->is_pushing_l2()) {
  //   // L2を押しながら単押し: r2_rliftの微調整（指令値を減少）
  //   // 微調整は他とは異なり、現在位置に対して行う
  //   r2_rlift_pos_ref(r2_rlift_current_pos_ - 0.01);
  //   // r2_rlift_pos_ref(r2_rlift_position_ref_ - 0.01);
  //   is_speed_mode = false;
  // } else if (ps4_->is_pushing_down() && !ps4_->is_pushing_l2()) {
  //   // 下のみが押されているときは速度制御で下ろす
  //   r2_flift_speed_ref(-3.0);
  //   r2_rlift_speed_ref(-3.0);
  //   is_speed_mode = true;
  // } else {
  //   if (is_speed_mode) {
  //     is_speed_mode = false;
  //     // 速度制御停止（L2+下の微調整後も含む）
  //     r2_flift_speed_mode_stop();
  //     r2_rlift_speed_mode_stop();
  //   }
  // }

  if (ps4_->is_pushed_left()) {
    // 間違えてボタン押したときのために、aroco_marker_id=0に設定
    publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
    r1_log_info("aruco デフォ");
    r2_flift_pos_ref(R2_FLIFT_NORMAL_POS);
    r2_rlift_pos_ref(R2_RLIFT_NORMAL_POS);
    if (manual_mode6_r2_lift_timer_ != nullptr) {
      manual_mode6_r2_lift_timer_->cancel();
    }
    manual_mode6_r2_lift_timer_ = this->create_wall_timer(500ms, [this]() {
      kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
      kfs_fz_pos_ref(KFS_FZ_NORMAL_POS);
      kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
      kfs_rz_pos_ref(KFS_RZ_NORMAL_POS);
      if (manual_mode6_r2_lift_timer_ != nullptr) {
        manual_mode6_r2_lift_timer_->cancel();
      }
      RCLCPP_INFO(this->get_logger(), "r2 lift lock (timer)");
    });
  }

  if (ps4_->is_pushed_triangle()) {
    if (ps4_->is_pushing_l2()) {
      // r2_fliftの微調整（指令値を増加）
      r2_flift_pos_ref(r2_flift_position_ref_ + 0.01);
      // 微調整は他とは異なり、現在位置に対して行う
      // r2_flift_pos_ref(r2_flift_current_pos_ + 0.01);
    } else {
      publish_all_aruco_marker_id(SECOND_KFS_ARUCO_MARKER_ID);
      r1_log_info("aruco KFS2つ目");
      if (aruco_marker_timer_) {
        aruco_marker_timer_->cancel();
      }
      aruco_marker_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(ARUCO_MARKER_RESET_TIME), [this]() {
          publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
          r1_log_info("aruco デフォ(リセット)");
          aruco_marker_timer_->cancel();
        });
    }
  }

  if (ps4_->is_pushed_circle()) {
    if (aruco_marker_timer_) {
      aruco_marker_timer_->cancel();
    }
    publish_all_aruco_marker_id(FIRST_KFS_ARUCO_MARKER_ID);
    r1_log_info("aruco KFS1つ目");
    aruco_marker_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(ARUCO_MARKER_RESET_TIME), [this]() {
        publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
        r1_log_info("aruco デフォ(リセット)");
        aruco_marker_timer_->cancel();
      });
  }

  if (ps4_->is_pushed_cross()) {
    if (ps4_->is_pushing_l2()) {
      // r2_fliftの微調整（指令値を減少）
      r2_flift_pos_ref(r2_flift_position_ref_ - 0.01);
      // 微調整は他とは異なり、現在位置に対して行う
      r2_flift_pos_ref(r2_flift_current_pos_ - 0.01);
    } else {
      if (aruco_marker_timer_) {
        aruco_marker_timer_->cancel();
      }
      publish_all_aruco_marker_id(PUT_KFS_ARUCO_MARKER_ID);
      r1_log_info("aruco put_kfs");
      aruco_marker_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(ARUCO_MARKER_RESET_TIME), [this]() {
          publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
          r1_log_info("aruco デフォ(リセット)");
          aruco_marker_timer_->cancel();
        });
    }
  }

  if (ps4_->is_pushed_square()) {
    if (aruco_marker_timer_) {
      aruco_marker_timer_->cancel();
    }
    publish_all_aruco_marker_id(THIRD_KFS_ARUCO_MARKER_ID);
    r1_log_info("aruco KFS3つ目");
    aruco_marker_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(ARUCO_MARKER_RESET_TIME), [this]() {
        publish_all_aruco_marker_id(DEFAULT_ARUCO_MARKER_ID);
        r1_log_info("aruco デフォ(リセット)");
        aruco_marker_timer_->cancel();
      });
  }

  if (ps4_->is_pushed_l1()) {
    // r2_fliftの微調整（指令値を減少）
    r2_flift_pos_ref(r2_flift_position_ref_ - 0.01);
    r2_rlift_pos_ref(r2_rlift_position_ref_ - 0.01);
    // 微調整は他とは異なり、現在位置に対して行う
    // r2_flift_pos_ref(r2_flift_current_pos_ - 0.01);
    // r2_rlift_pos_ref(r2_rlift_current_pos_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    // r2_fliftの微調整（指令値を増加）
    r2_flift_pos_ref(r2_flift_position_ref_ + 0.01);
    r2_rlift_pos_ref(r2_rlift_position_ref_ + 0.01);
    // 微調整は他とは異なり、現在位置に対して行う
    // r2_flift_pos_ref(r2_flift_current_pos_ + 0.01);
    // r2_rlift_pos_ref(r2_rlift_current_pos_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    // 微調整トリガーとして使用
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }
}

/**
 * @brief
 *
 * @param n 何個目の機構を動かすか。
 * @param m どの高さを狙うか。m==1で下段、m==2で中段、m==3で上段を狙う
 */
void R1MainNode::manual_mode7_spear_attack_task(int n, int m, bool _reverse_trigger)
{
  // 千田機構だったときの名残で引数にnがあるが、現在は使用していない
  (void)n;

  static bool reverse_trigger = false;

  int & step = manual_mode7_spear_attack_task_step_;
  // RCLCPP_INFO(this->get_logger(), "manual_mode7_spear_attack_task step: %d", step);
  r1_log_info("mode7 やり攻撃 step%d", step);
  if (step == 1) {
    // reverse_triggerを更新
    reverse_trigger = _reverse_trigger;
    // KFS回収機構を当たらない位置に移動
    kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
    kfs_fz_pos_ref(KFS_FZ_NORMAL_POS);
    kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
    kfs_rz_pos_ref(KFS_RZ_NORMAL_POS);
    // 中段と上段を狙うときのみ、yawは内向きにする
    if (m == 2 || m == 3) {
      kfs_fyaw_move_rear_mech_lock();
      kfs_ryaw_move_front_mech_lock();
    }
    // 念の為push_valveはfalseにしておく
    spear_hand_push_valve(false);
    if (m == 1) {
      // 下段を狙う
      spear_y_pos_ref(SPEAR_Y_LOW_ATTACK_POS);
    } else if (m == 2) {
      // 中段を狙う
      spear_y_pos_ref(SPEAR_Y_MIDDLE_ATTACK_POS);
    } else if (m == 3) {
      // 上段を狙う
      spear_y_pos_ref(SPEAR_Y_HIGH_ATTACK_POS);
    }
    step++;
  } else if (step == 2) {
    // reverse_triggerがfalseのときのみもう一度更新
    // こうすることでstep1とstep2のどちらかのみでも、reverse_triggerの値を更新できる
    if (!reverse_trigger) {
      reverse_trigger = _reverse_trigger;
    }
    bool is_red = (zone_ == "red");
    // reverse_triggerがtrueのときは、is_redを反転させる
    if (reverse_trigger) {
      is_red = !is_red;
      RCLCPP_INFO(
        this->get_logger(), "spear attack task: reverse_trigger is true, is_red is reversed to %s",
        is_red ? "true" : "false");
    }
    if (m == 1) {
      // 下段を狙う
      if (is_red) {
        spear_roll1_pos_ref(SPEAR_ROLL1_RED_LOW_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_RED_LOW_ATTACK_ANGLE);
      } else {
        spear_roll1_pos_ref(SPEAR_ROLL1_BLUE_LOW_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_BLUE_LOW_ATTACK_ANGLE);
      }
    } else if (m == 2) {
      // 中段を狙う
      if (is_red) {
        spear_roll1_pos_ref(SPEAR_ROLL1_RED_MIDDLE_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_RED_MIDDLE_ATTACK_ANGLE);
      } else {
        spear_roll1_pos_ref(SPEAR_ROLL1_BLUE_MIDDLE_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_BLUE_MIDDLE_ATTACK_ANGLE);
      }
    } else if (m == 3) {
      // 上段を狙う
      if (is_red) {
        spear_roll1_pos_ref(SPEAR_ROLL1_RED_HIGH_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_RED_HIGH_ATTACK_ANGLE);
      } else {
        spear_roll1_pos_ref(SPEAR_ROLL1_BLUE_HIGH_ATTACK_ANGLE);
        spear_roll2_pos_ref(SPEAR_ROLL2_BLUE_HIGH_ATTACK_ANGLE);
      }
    }
    if (n == 2 || n == 3) {
      // spear_hand_push_valve(true);
    }
    step++;
  } else if (step == 3) {
    spear_hand_push_valve(false);
    spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
    spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
    // spear_yは攻撃動作前の位置に戻す
    spear_y_pos_ref(SPEAR_Y_MAKE_SPEAR_POS);
    step = 1;
    // RCLCPP_INFO(this->get_logger(), "spear attack task completed");
    r1_log_info("mode7 やり攻撃 完了");
  }
}

void R1MainNode::manual_mode7_spear_throw_away_task(int n)
{
  // 千田機構だったときの名残で引数にnがあるが、現在は使用していない
  (void)n;

  int & step = manual_mode7_spear_throw_away_task_step_;
  r1_log_info("mode7 やり廃棄 step%d", step);

  if (step == 1) {
    // KFS回収機構はスタート時の高い位置に移動させる
    kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
    kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
    kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
    kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
    kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
    kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
    step++;
  } else if (step == 2) {
    spear_y_pos_ref(SPEAR_Y_THROW_AWAY_POS);
    spear_hand_push_valve(true);
    if (zone_ == "red") {
      spear_roll1_pos_ref(SPEAR_ROLL1_INV_HORIZONTAL_ANGLE);
      spear_roll2_pos_ref(SPEAR_ROLL2_INV_HORIZONTAL_ANGLE);
    } else {
      spear_roll1_pos_ref(SPEAR_ROLL1_HORIZONTAL_ANGLE);
      spear_roll2_pos_ref(SPEAR_ROLL2_HORIZONTAL_ANGLE);
    }
    step++;
  } else if (step == 3) {
    spear_hand_push_valve(false);
    step++;
  } else if (step == 4) {
    spear_hand_push_valve(true);
    spear_hand1_valve(true);
    spear_hand2_valve(true);
    step++;
  } else if (step == 5) {
    spear_hand1_valve(false);
    spear_hand2_valve(false);
    spear_roll1_pos_ref(SPEAR_ROLL1_VERTICAL_ANGLE);
    spear_roll2_pos_ref(SPEAR_ROLL2_VERTICAL_ANGLE);
    // spear_yは攻撃動作前の位置に戻す
    spear_y_pos_ref(SPEAR_Y_MAKE_SPEAR_POS);
    step = 1;
    r1_log_info("mode7 やり廃棄 完了");
    // RCLCPP_INFO(this->get_logger(), "spear throw away task completed");
  }
}

void R1MainNode::manual_mode7_spear_attack(void)
{
  constexpr int FKFS = 0;
  constexpr int RKFS = 1;
  auto & hand_valve_step = manual_mode7_hand_valve_step_;
  auto & push_valve_step = manual_mode7_push_valve_step_;
  int & l2_r2_trigger_step = manual_mode7_l2_r2_trigger_step_;

  bool reverse_trigger = ps4_->is_pushing_l2();

  bool front_pressure_detected = !front_pressure_switch_status_;
  bool rear_pressure_detected = !rear_pressure_switch_status_;

  // 適当な初期値を代入
  static int kfs_select = FKFS;

  if (ps4_->is_pushed_up()) {
    spear_y_pos_ref(spear_y_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_right()) {
    if (ps4_->is_pushing_l2()) {
      if (ENABLE_PRESSURE_SENSOR) {
        // やり攻撃の最後のステップを実行し、アクチュエータをspear_yとspear_rollを移動させる
        manual_mode7_spear_attack_task_step_ = 3;
        manual_mode7_spear_attack_task(2, 1, reverse_trigger);

        if (manual_mode7_put_timer_) {
          manual_mode7_put_timer_->cancel();
        }

        manual_mode7_put_timer_ =
          this->create_wall_timer(500ms, [&, front_pressure_detected, rear_pressure_detected] {
            // 圧力センサが反応している方のput動作を行う
            RCLCPP_INFO(
              this->get_logger(), "front_pressure_detected: %s, rear_pressure_detected: %s",
              front_pressure_detected ? "true" : "false",
              rear_pressure_detected ? "true" : "false");
            if (front_pressure_detected) {
              // put動作
              kfs_fx_pos_ref(KFS_FX_PUT_POS);
              kfs_fz_pos_ref(KFS_FZ_PUT_POS);
              kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
              kfs_rx_pos_ref(KFS_RX_NORMAL_POS);
              kfs_rz_pos_ref(KFS_RZ_PUT_POS);
              kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
              kfs_select = FKFS;
              RCLCPP_INFO(this->get_logger(), "moved to front_kfs put position");
            } else if (rear_pressure_detected) {
              // put動作
              kfs_fx_pos_ref(KFS_FX_NORMAL_POS);
              kfs_fz_pos_ref(KFS_FZ_PUT_POS);
              kfs_fyaw_pos_ref(KFS_FYAW_SIDE_ANGLE);
              kfs_rx_pos_ref(KFS_RX_PUT_POS);
              kfs_rz_pos_ref(KFS_RZ_PUT_POS);
              kfs_ryaw_pos_ref(KFS_RYAW_SIDE_ANGLE);
              kfs_select = RKFS;
              RCLCPP_INFO(this->get_logger(), "moved to rear_kfs put position");
            }

            manual_mode7_put_timer_->cancel();
          });
      }
    } else {
      spear_roll1_pos_ref(spear_roll1_position_ref_ + 0.05);
      spear_roll2_pos_ref(spear_roll2_position_ref_ + 0.05);
    }
  }

  if (ps4_->is_pushed_down()) {
    spear_y_pos_ref(spear_y_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_left()) {
    if (ps4_->is_pushing_l2()) {
      if (ENABLE_PRESSURE_SENSOR) {
        // 置くときは箱を持ち上げている状態で、そのとき圧力が下がりきっていない可能性があるので
        // 最後に右ボタンを押されたときに動かしたKFS回収機構の方を動かすようにする
        if (kfs_select == FKFS) {
          kfs_front_pump(0.0);
          kfs_front_valve(true);
          // setTimeout風で電磁弁をOFFにする。
          if (manual_mode7_front_valve_timer_) {
            manual_mode7_front_valve_timer_->cancel();
          }
          manual_mode7_front_valve_timer_ =
            this->create_wall_timer(std::chrono::duration<double>(KFS_VALVE_DELAY_TIME), [this]() {
              kfs_front_valve(false);
              if (manual_mode7_front_valve_timer_) {
                manual_mode7_front_valve_timer_->cancel();
              }
            });
        } else if (kfs_select == RKFS) {
          kfs_rear_pump(0.0);
          kfs_rear_valve(true);
          // setTimeout風で電磁弁をOFFにする。
          if (manual_mode7_rear_valve_timer_) {
            manual_mode7_rear_valve_timer_->cancel();
          }
          manual_mode7_rear_valve_timer_ =
            this->create_wall_timer(std::chrono::duration<double>(KFS_VALVE_DELAY_TIME), [this]() {
              kfs_rear_valve(false);
              if (manual_mode7_rear_valve_timer_) {
                manual_mode7_rear_valve_timer_->cancel();
              }
            });
        }
      }
    } else {
      spear_roll1_pos_ref(spear_roll1_position_ref_ - 0.05);
      spear_roll2_pos_ref(spear_roll2_position_ref_ - 0.05);
    }
  }

  if (ps4_->is_pushed_triangle()) {
    // 番号は千田機構だったときの名残で2を指定。現在、この指定には意味はない。
    manual_mode7_spear_attack_task(2, 2, reverse_trigger);
  }

  if (ps4_->is_pushed_circle()) {
    // 番号は千田機構だったときの名残で2を指定。現在、この指定には意味はない。
    manual_mode7_spear_attack_task(2, 1, reverse_trigger);
  }

  if (ps4_->is_pushed_cross()) {
    // 番号は千田機構だったときの名残で2を指定。現在、この指定には意味はない。
    manual_mode7_spear_throw_away_task(2);
  }

  if (ps4_->is_pushed_square()) {
    // 番号は千田機構だったときの名残で2を指定。現在、この指定には意味はない。
    manual_mode7_spear_attack_task(2, 3, reverse_trigger);
  }

  if (ps4_->is_pushed_l1()) {
    if (push_valve_step == 1) {
      spear_hand_push_valve(true);
      push_valve_step = 2;
    } else {
      spear_hand_push_valve(false);
      push_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_r1()) {
    if (hand_valve_step == 1) {
      spear_hand1_valve(true);
      spear_hand2_valve(true);
      hand_valve_step = 2;
    } else {
      spear_hand1_valve(false);
      spear_hand2_valve(false);
      hand_valve_step = 1;
    }
  }

  if (ps4_->is_pushed_l2()) {
    // spear_attack_task内で、reverse_triggerとして使用
  }

  if (ps4_->is_pushed_r2()) {
    // manual_task内で、速度トリガーとして使用
  }

  if (ps4_->is_pushing_l2() && ps4_->is_pushing_r2()) {
    if (l2_r2_trigger_step == DEFAULT_STEP) {
      // 回収初期値に戻す
      kfs_fx_pos_ref(KFS_FX_STORAGE_POS);
      kfs_fz_pos_ref(KFS_FZ_STORAGE_POS);
      kfs_fyaw_pos_ref(KFS_FYAW_START_ANGLE);
      kfs_rx_pos_ref(KFS_RX_STORAGE_POS);
      kfs_rz_pos_ref(KFS_RZ_STORAGE_POS);
      kfs_ryaw_pos_ref(KFS_RYAW_START_ANGLE);
      // 真空ポンプ関連の指令値は、操縦ミスでKFSを落とすのを防止するため、操作しない。
      l2_r2_trigger_step = 2;
    }
  } else {
    l2_r2_trigger_step = DEFAULT_STEP;
  }
}

void R1MainNode::auto_collect_kfs_task(void)
{
  constexpr int FKFS = 0;
  constexpr int RKFS = 1;
  constexpr int HEIGHT_LOW = 1;
  constexpr int HEIGHT_MIDDLE = 2;
  constexpr int HEIGHT_HIGH = 3;

  static int prev_fkfs_step = DEFAULT_STEP;
  static int prev_rkfs_step = DEFAULT_STEP;

  auto calc_height = [&](int n) {
    if (n == 2 || n == 4 || n == 10 || n == 12) {
      return HEIGHT_LOW;  // 下段
    } else if (n == 1 || n == 3 || n == 7 || n == 9 || n == 11) {
      return HEIGHT_MIDDLE;  // 中段
    } else if (n == 6) {
      return HEIGHT_HIGH;  // 上段
    } else {
      RCLCPP_ERROR(this->get_logger(), "invalid forest number: %d", n);
      return -1;
    }
  };

  auto update_wall_sensor_status = [&](int target_forest, int within_index) {
    int kfs_height = calc_height(target_forest);
    bool wall_detected = false;
    double sensor_value_low = 0.0;
    double sensor_value_middle = 0.0;
    if (within_index == FKFS) {
      sensor_value_low = scan_fl_data_;
      sensor_value_middle = scan_fm_data_;
    } else if (within_index == RKFS) {
      sensor_value_low = scan_rl_data_;
      sensor_value_middle = scan_rm_data_;
    }

    // センサーが反応しているかを確認
    if (kfs_height == HEIGHT_LOW) {
      // 下段の検出：lセンサがしきい値より遠いかつmセンサがしきい値より遠いとき：センサ反応
      wall_detected =
        (sensor_value_low > WALL_SENSOR_DISTANCE_THRESHOLD &&
         sensor_value_middle > WALL_SENSOR_DISTANCE_THRESHOLD);
    } else if (kfs_height == HEIGHT_MIDDLE) {
      // 中段の検出：lセンサがしきい値より近いかつmセンサがしきい値より近いとき：センサ反応
      wall_detected =
        (sensor_value_low < WALL_SENSOR_DISTANCE_THRESHOLD &&
         sensor_value_middle > WALL_SENSOR_DISTANCE_THRESHOLD);
    } else if (kfs_height == HEIGHT_HIGH) {
      // 上段の検出：lセンサがしきい値より近いかつmセンサがしきい値より近いとき：センサ反応
      wall_detected = (sensor_value_low < WALL_SENSOR_DISTANCE_THRESHOLD) &&
                      (sensor_value_middle < WALL_SENSOR_DISTANCE_THRESHOLD);
    }
    // センサーが初回の反応だった場合は開始時刻を更新
    if (wall_detected && wall_sensor_detected_[target_forest - 1] == false) {
      wall_sensor_detect_start_time_[target_forest - 1] = this->now();
    }
    wall_sensor_detected_[target_forest - 1] = wall_detected;
  };

  auto is_detect_wall = [&](int target_forest) {
    // 壁センサーが一定時間以上反応している場合は壁があると判断する
    if (wall_sensor_detected_[target_forest - 1]) {
      rclcpp::Duration detected_duration =
        this->now() - wall_sensor_detect_start_time_[target_forest - 1];
      if (detected_duration.seconds() > WALL_SENSOR_TIME_THRESHOLD) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  };

  auto update_pressure_sensor_status = [&](int kfs_mechanism) {
    bool current_pressure_sensor_status = false;
    // 吸着すると、センサーの出力がOFFになるため、論理を反転させる。
    if (kfs_mechanism == FKFS) {
      current_pressure_sensor_status = !front_pressure_switch_status_;
    } else if (kfs_mechanism == RKFS) {
      current_pressure_sensor_status = !rear_pressure_switch_status_;
    }
    // センサーが初回の反応だった場合は開始時刻を更新
    if (current_pressure_sensor_status && !pressure_sensor_detected_[kfs_mechanism]) {
      pressure_sensor_detect_start_time_[kfs_mechanism] = this->now();
    }
    pressure_sensor_detected_[kfs_mechanism] = current_pressure_sensor_status;
  };

  auto is_detect_pressure = [&](int kfs_mechanism) {
    // 圧力センサーが一定時間以上反応している場合は物体を掴んでいると判断する
    if (pressure_sensor_detected_[kfs_mechanism]) {
      rclcpp::Duration detected_duration =
        this->now() - pressure_sensor_detect_start_time_[kfs_mechanism];
      if (detected_duration.seconds() > PRESSURE_SENSOR_TIME_THRESHOLD) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  };

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

  geometry_msgs::msg::PoseStamped map_pos = get_map_pos();
  int n = kfs_auto_collect_plan_.forest_order.size();
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
    double odom_x = odometry_.pose.pose.position.x;
    double odom_y = odometry_.pose.pose.position.y;
    double center_x = 0.0;
    double center_y = 0.0;
    double rect_yaw = 0.0;
    double offset_x = 0.0;
    double offset_y = 0.0;
    // 壁センサー反応時のオフセット
    double wall_offset_x = 0.0;
    double wall_offset_y = 0.0;
    // 壁検出から終了までの移動距離
    double wall_move_dist_x = 0.0;
    double wall_move_dist_y = 0.0;

    // within関連はメンバー変数。名前が長いので、参照として短い名前で扱う。
    int within_index = (mechanism_type == "front_kfs") ? FKFS : RKFS;
    // 短い別名をつける。よくわからないけど、bool型はstd::vector<bool>::reference型で参照を取る必要があるらしい
    std::vector<bool>::reference within =
      kfs_auto_collect_within_[target_forest_number - 1][within_index];
    // std::vector<bool>::reference prev_within =
    //   kfs_auto_collect_prev_within_[target_forest_number - 1][within_index];
    // within はこの周期の判定結果なので、毎周期いったん false に戻して再評価する。
    within = false;

    auto & step = within_index == FKFS ? auto_collect_kfs_fkfs_step_ : auto_collect_kfs_rkfs_step_;

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
      // center_x *= -1.0;
      // rect_yaw = angle_normalize(M_PI - rect_yaw);
    }
    // 足回りオフセットと壁センサーオフセットを計算する
    if (zone_ == "red") {
      if (is_inner && mechanism_type == "front_kfs") {
        offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
        offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
      } else if (!is_inner && mechanism_type == "rear_kfs") {
        offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
        offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
      }
    } else if (zone_ == "blue") {
      if (is_inner && mechanism_type == "rear_kfs") {
        offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
        offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
      } else if (!is_inner && mechanism_type == "front_kfs") {
        offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
        offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
      }
    }

    // 壁センサーは進行方向に対して、回収機構よりも先に壁が反応するときにのみ適用する
    if (zone_ == "red") {
      wall_move_dist_x = MOVE_DISTANCE_AFTER_WALL_DETECT * std::cos(rect_yaw);
      wall_move_dist_y = MOVE_DISTANCE_AFTER_WALL_DETECT * std::sin(rect_yaw);
      if (is_inner && mechanism_type == "rear_kfs") {
        wall_offset_x = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::cos(rect_yaw);
        wall_offset_y = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::sin(rect_yaw);
      } else if (!is_inner && mechanism_type == "front_kfs") {
        wall_offset_x = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::cos(rect_yaw);
        wall_offset_y = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::sin(rect_yaw);
      }
    } else if (zone_ == "blue") {
      wall_move_dist_x = MOVE_DISTANCE_AFTER_WALL_DETECT * std::cos(rect_yaw);
      wall_move_dist_y = MOVE_DISTANCE_AFTER_WALL_DETECT * std::sin(rect_yaw);
      if (is_inner && mechanism_type == "front_kfs") {
        wall_offset_x = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::cos(rect_yaw);
        wall_offset_y = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::sin(rect_yaw);
      } else if (!is_inner && mechanism_type == "rear_kfs") {
        wall_offset_x = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::cos(rect_yaw);
        wall_offset_y = WALL_SENSOR_DELAY_OFFSET_DISTANCE * std::sin(rect_yaw);
      }
    }

    // center_xとcenter_yにオフセットを適用する
    if (zone_ == "red") {
      center_x += offset_x;
      center_y += offset_y;
    } else {
      // 本当はcenter_xはプラスではなくマイナスのはずだが、何故か動かないので一旦プラス
      // おそらく、プラスでいいのはrect_yawが角度を反転させてるからだと思う。
      center_x += offset_x;
      center_y += offset_y;
    }

    int kfs_height = calc_height(target_forest_number);

    if (step == 1) {
      // 既に回収完了済みの場合はスキップ
      if (kfs_already_collected_[target_forest_number - 1]) {
        continue;
      }
      // ENABLE_WALL_SENSOR == trueのとき、壁センサーの値を更新し、壁を検出する
      // ENABLE_WALL_SENSOR == falseのとき、座標ベースで回収動作を開始するかどうかの判定を行う
      if (ENABLE_WALL_SENSOR == true) {
        // 壁センサーのステータスを更新
        update_wall_sensor_status(target_forest_number, within_index);

        // まずは壁検出機能が有効となる範囲内にロボットがいるかを確認する
        if (
          is_within_rotated_rectangle(
            map_x, map_y, center_x, center_y, rect_yaw, WALL_SENSOR_DETECT_WIDTH,
            WALL_SENSOR_DETECT_HEIGHT)) {
          double log_sensor_low = (within_index == FKFS) ? scan_fl_data_ : scan_rl_data_;
          double log_sensor_middle = (within_index == FKFS) ? scan_fm_data_ : scan_rm_data_;
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 250,
            "Step = %d, wall sensor status for forest %d %s kfs: map_x=%.2f, map_y=%.2f, "
            "odom_x=%.2f, odom_y=%.2f, sensor_value_low=%.2f, sensor_value_middle=%.2f, "
            "wall_detected=%d, offset_x=%.2f, offset_y=%.2f, center_x=%.2f, center_y=%.2f",
            step, target_forest_number, mechanism_type.c_str(), map_x, map_y, odom_x, odom_y,
            log_sensor_low, log_sensor_middle, (int)wall_sensor_detected_[target_forest_number - 1],
            offset_x, offset_y, center_x, center_y);
          if (is_detect_wall(target_forest_number)) {
            // 壁検出位置の座標を更新（odom座標系）
            wall_detect_pos_[target_forest_number - 1] = odometry_;
            RCLCPP_INFO(
              this->get_logger(),
              "Step = %d, wall detected for forest %d %s kfs: map_x=%.2f, map_y=%.2f, "
              "odom_x=%.2f, "
              "odom_y=%.2f, wall_offset_x=%.2f, wall_offset_y=%.2f",
              step, target_forest_number, mechanism_type.c_str(), map_x, map_y, odom_x, odom_y,
              wall_offset_x, wall_offset_y);
            step++;
          }
        }
      } else if (ENABLE_WALL_SENSOR == false) {
        // 座標ベースの判定
        if (
          is_within_rotated_rectangle(
            map_x, map_y, center_x, center_y, rect_yaw, COLLECT_KFS_WIDTH, COLLECT_KFS_HEIGHT)) {
          // 範囲内に入ったら次のステップに移動する
          step++;
        }
      }
    } else if (step == 2) {
      // 壁センサーでオフセットを適用する場合、少し進む
      // 壁センサーを使用しない場合、このステップはスキップする
      if (ENABLE_WALL_SENSOR == true) {
        double sx = wall_detect_pos_[target_forest_number - 1].pose.pose.position.x;
        double sy = wall_detect_pos_[target_forest_number - 1].pose.pose.position.y;
        double gx = sx + wall_offset_x;
        double gy = sy + wall_offset_y;
        double cx = odom_x;
        double cy = odom_y;
        if (is_passed_goal_by_dot(sx, sy, gx, gy, cx, cy)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Step = %d, passed wall detect offset position for forest %d %s kfs: sx=%.2f, "
            "sy=%.2f, "
            "gx=%.2f, "
            "gy=%.2f, cx=%.2f, cy=%.2f",
            step, target_forest_number, mechanism_type.c_str(), sx, sy, gx, gy, cx, cy);
          step++;
        }
      } else {
        step++;
      }
    } else if (step == 3) {
      // 足回りが自動制御モードのときは進行方向の位置制御はOFFにする
      if (r1_init_parameter_.enable_kfs_auto_chassis == true) {
        std_msgs::msg::Bool tangent_msg;
        tangent_msg.data = false;
        chassis_tangent_pid_enable_publisher_->publish(tangent_msg);
        if (ENABLE_STOP_BEFORE_COLLECT_KFS) {
          publish_chassis_act_pause();
          is_act_paused_ = true;
        }
      }
      // withinをtrueにし、KFS回収機構のLEDの色を変化させる。
      within = true;
      step++;
    } else if (step == 4) {
      // 機構を展開
      if (ENABLE_AUTO_COLLECT_KFS_ACTUATOR) {
        auto choose_use_front_mech_lock = [&]() -> bool {
          if (zone_ == "blue")
            return is_inner;
          else
            return !is_inner;
        };

        if (within_index == FKFS) {
          // 収納yawタイマーが残っている場合はキャンセルする
          if (auto_collect_front_storage_yaw_timer_) {
            auto_collect_front_storage_yaw_timer_->cancel();
          }
          // 回収機構を動かす
          kfs_fx_pos_ref(KFS_FX_EXPAND_POS);
          if (choose_use_front_mech_lock()) {
            kfs_fyaw_move_front_mech_lock();
          } else {
            kfs_fyaw_move_rear_mech_lock();
          }
          kfs_front_pump(1.0);
          kfs_front_valve(false);
          if (kfs_height == HEIGHT_LOW) {
            kfs_fz_pos_ref(KFS_FZ_LOW_POS);
          } else if (kfs_height == HEIGHT_MIDDLE) {
            kfs_fz_pos_ref(KFS_FZ_MIDDLE_POS);
          } else if (kfs_height == HEIGHT_HIGH) {
            kfs_fz_pos_ref(KFS_FZ_HIGH_POS);
          }
        } else {
          // 収納yawタイマーが残っている場合はキャンセルする
          if (auto_collect_rear_storage_yaw_timer_) {
            auto_collect_rear_storage_yaw_timer_->cancel();
          }
          // 回収機構を動かす
          kfs_rx_pos_ref(KFS_RX_EXPAND_POS);
          if (choose_use_front_mech_lock()) {
            kfs_ryaw_move_front_mech_lock();
          } else {
            kfs_ryaw_move_rear_mech_lock();
          }
          kfs_rear_pump(1.0);
          kfs_rear_valve(false);
          if (kfs_height == HEIGHT_LOW) {
            kfs_rz_pos_ref(KFS_RZ_LOW_POS);
          } else if (kfs_height == HEIGHT_MIDDLE) {
            kfs_rz_pos_ref(KFS_RZ_MIDDLE_POS);
          } else if (kfs_height == HEIGHT_HIGH) {
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
      // 最後まで終わったら次のステップに進む
      step++;
    } else if (step == 5) {
      // ポーズ中は回収動作を行わない（右スティックでresumeされるまで待機）
      if (r1_init_parameter_.enable_kfs_auto_chassis == true) {
        if (is_act_paused_) {
          return;
        }
      }
      // 最後まで終わったら次のステップに進む
      step++;
    } else if (step == 6) {
      if (ENABLE_PRESSURE_SENSOR) {
        // 圧力センサーの値を更新
        update_pressure_sensor_status(within_index);
        // 圧力センサーが反応しているかを確認し、反応していれば回収完了とみなす
        if (is_detect_pressure(within_index)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Step = %d, pressure detected for forest %d %s kfs, considering collect completed",
            step, target_forest_number, mechanism_type.c_str());
          step++;
        }
      } else if (ENABLE_WALL_SENSOR) {
        // 一定距離進むまで待機
        double sx = wall_detect_pos_[target_forest_number - 1].pose.pose.position.x;
        double sy = wall_detect_pos_[target_forest_number - 1].pose.pose.position.y;
        double gx = sx + wall_offset_x + wall_move_dist_x;
        double gy = sy + wall_offset_y + wall_move_dist_y;
        double cx = odom_x;
        double cy = odom_y;
        if (is_passed_goal_by_dot(sx, sy, gx, gy, cx, cy)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Step = %d, passed wall detect offset + move distance position for forest %d %s kfs: "
            "sx=%.2f, "
            "sy=%.2f, "
            "gx=%.2f, gy=%.2f, cx=%.2f, cy=%.2f",
            step, target_forest_number, mechanism_type.c_str(), sx, sy, gx, gy, cx, cy);
          step++;
        }
      } else {
        // TODO: 座標ベースの終了判定も本当は終点のときはodom座標系のほうがいいかも
        // 現在は過去のプログラムをそのまま引き継いでいるのでmap座標系で判別している
        if (
          is_within_rotated_rectangle(
            map_x, map_y, center_x, center_y, rect_yaw, COLLECT_KFS_WIDTH, COLLECT_KFS_HEIGHT) ==
          false) {
          step++;
        }
      }
    } else if (step == 7) {
      // 回収動作終了
      // 足回りの進行方向の位置制御をONにする
      if (r1_init_parameter_.enable_kfs_auto_chassis == true) {
        std_msgs::msg::Bool tangent_msg;
        tangent_msg.data = false;
        chassis_tangent_pid_enable_publisher_->publish(tangent_msg);
      }
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
              if (auto_collect_front_storage_yaw_timer_) {
                auto_collect_front_storage_yaw_timer_->cancel();
              }
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
              if (auto_collect_rear_storage_yaw_timer_) {
                auto_collect_rear_storage_yaw_timer_->cancel();
              }
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
      RCLCPP_INFO(
        this->get_logger(), "%d forest %s kfs auto collect completed", target_forest_number,
        kfs_auto_collect_plan_.kfs_mechanism_type[i].c_str());
      // ステップをリセット
      step = 1;
      // 壁センサーの状態をリセット
      wall_sensor_detected_[target_forest_number - 1] = false;
      wall_sensor_detect_start_time_[target_forest_number - 1] = this->now();
      // 圧力センサーの状態をリセット
      pressure_sensor_detected_[within_index] = false;
      pressure_sensor_detect_start_time_[within_index] = this->now();
      // 回収完了済みフラグをセット
      kfs_already_collected_[target_forest_number - 1] = true;
      within = false;
    }

    // stepが変化した場合はログ出力
    if (
      prev_fkfs_step != auto_collect_kfs_fkfs_step_ ||
      prev_rkfs_step != auto_collect_kfs_rkfs_step_) {
      r1_log_info(
        "%d forest %s kfs auto collect step: %d -> %d", target_forest_number,
        mechanism_type.c_str(), (within_index == FKFS) ? prev_fkfs_step : prev_rkfs_step, step);
      prev_fkfs_step = auto_collect_kfs_fkfs_step_;
      prev_rkfs_step = auto_collect_kfs_rkfs_step_;
    }
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
    step == ChassisAct::ACT2_FINISH || step == ChassisAct::ACT4_FINISH ||
    step == ChassisAct::ACT3_FINISH || step == ChassisAct::ACT5_FINISH) {
    // KFS回収動作のACTの場合は、回収動作も止める。
    if (
      step == ChassisAct::ACT2_FINISH || step == ChassisAct::ACT3_FINISH ||
      step == ChassisAct::ACT4_FINISH) {
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
      stop_kfs_auto_collect();
    } else if (step == ChassisAct::ACT5) {
      publish_robot_move(ChassisAct::ACT5_FINISH, std::vector<int>{}, std::vector<std::string>{});
    } else if (pending_auto_robot_move_valid_) {
      clear_auto_chassis_state(true);
    }
  }
}

void R1MainNode::reset_step(void)
{
  // 各手順のステップをリセット
  manual_mode1_hand_valve_step_ = DEFAULT_STEP;
  manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
  manual_mode2_hand_valve_step_ = DEFAULT_STEP;
  manual_mode2_push_valve_step_ = DEFAULT_STEP;
  manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  manual_mode3_hand_valve_step_ = DEFAULT_STEP;
  manual_mode3_push_valve_step_ = DEFAULT_STEP;
  manual_mode4_fx_step_ = DEFAULT_STEP;
  manual_mode4_fz_step_ = DEFAULT_STEP;
  manual_mode4_fyaw_step_ = DEFAULT_STEP;
  manual_mode4_front_pump_step_ = DEFAULT_STEP;
  manual_mode4_l2_r2_trigger_step_ = DEFAULT_STEP;
  manual_mode5_rx_step_ = DEFAULT_STEP;
  manual_mode5_rz_step_ = DEFAULT_STEP;
  manual_mode5_ryaw_step_ = DEFAULT_STEP;
  manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  manual_mode5_l2_r2_trigger_step_ = DEFAULT_STEP;
  manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  manual_mode7_spear_throw_away_task_step_ = DEFAULT_STEP;
  manual_mode7_l2_r2_trigger_step_ = DEFAULT_STEP;
  manual_mode7_hand_valve_step_ = DEFAULT_STEP;
  manual_mode7_push_valve_step_ = DEFAULT_STEP;
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
  double start_y = 0.0;
  double start_yaw = 0.0;
  if (is_start_zone) {
    if (zone_ == "blue") {
      start_x = -5.5;
      start_y = 0.5;
      start_yaw = -M_PI / 2.0;
    } else {
      start_x = 5.5;
      start_y = 0.5;
      start_yaw = -M_PI / 2.0;
    }
  } else {
    if (zone_ == "blue") {
      start_x = -5.5;
      start_y = 11.5;
      start_yaw = -M_PI / 2.0;
    } else {
      start_x = 5.5;
      start_y = 11.5;
      start_yaw = -M_PI / 2.0;
    }
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
  set_led_event(0, 0, 50, 0.2, 1.0);
  // arucoマーカをリセットする
  publish_all_aruco_marker_id(0);
  received_r1_collect_kfs_ = false;
  std_msgs::msg::Bool msg;
  msg.data = true;
  chassis_tangent_pid_enable_publisher_->publish(msg);
}

void R1MainNode::manual_task(void)
{
  auto current_state = state_machine_->get_current_state();
  double vx_max = CHASSIS_NORMAL_VELOCITY;
  double vy_max = CHASSIS_NORMAL_VELOCITY;
  double vz_max = CHASSIS_NORMAL_OMEGA;
  double slope = ENABLE_R2_ANALOG_SPEED_CONTROL == true ? ps4_->get_r2_analog() : 1.0;

  if (
    (current_state.operation_mode == OperationMode::MODE2_POLE) ||
    (current_state.operation_mode == OperationMode::MODE6_R2_LIFT)) {
    // 最大速度と最大角速度をCHASSIS_LOW_VELOCITY / CHASSIS_LOW_OMEGAからCHASSIS_NORMAL_VELOCITY / CHASSIS_NORMAL_OMEGAまで線形に変化させる
    vx_max = CHASSIS_LOW_VELOCITY + (CHASSIS_NORMAL_VELOCITY - CHASSIS_LOW_VELOCITY) * slope;
    vy_max = CHASSIS_LOW_VELOCITY + (CHASSIS_NORMAL_VELOCITY - CHASSIS_LOW_VELOCITY) * slope;
    vz_max = CHASSIS_LOW_OMEGA + (CHASSIS_NORMAL_OMEGA - CHASSIS_LOW_OMEGA) * slope;
  } else if (current_state.operation_mode == OperationMode::MODE3_SPEAR) {
    // 最大速度と最大角速度をCHASSIS_LOW_VELOCITY / CHASSIS_LOW_OMEGAからCHASSIS_MAKE_SPEAR_VELOCITY / CHASSIS_MAKE_SPEAR_OMEGAまで線形に変化させる
    vx_max = CHASSIS_LOW_VELOCITY + (CHASSIS_MAKE_SPEAR_VELOCITY - CHASSIS_LOW_VELOCITY) * slope;
    vy_max = CHASSIS_LOW_VELOCITY + (CHASSIS_MAKE_SPEAR_VELOCITY - CHASSIS_LOW_VELOCITY) * slope;
    vz_max = CHASSIS_LOW_OMEGA + (CHASSIS_MAKE_SPEAR_OMEGA - CHASSIS_LOW_OMEGA) * slope;
  } else if (
    (current_state.operation_mode == OperationMode::MODE4_FKFS) ||
    (current_state.operation_mode == OperationMode::MODE5_RKFS) ||
    (current_state.operation_mode == OperationMode::MODE7_SPEAR_ATTACK)) {
    // 最大速度と最大角速度をCHASSIS_NORMAL_VELOVITY / CHASSIS_NORMAL_OMEGAからCHASSIS_HIGH_VELOCITY / CHASSIS_HIGH_OMEGAまで線形に変化させる
    vx_max = CHASSIS_NORMAL_VELOCITY + (CHASSIS_HIGH_VELOCITY - CHASSIS_NORMAL_VELOCITY) * slope;
    vy_max = CHASSIS_NORMAL_VELOCITY + (CHASSIS_HIGH_VELOCITY - CHASSIS_NORMAL_VELOCITY) * slope;
    vz_max = CHASSIS_NORMAL_OMEGA + (CHASSIS_HIGH_OMEGA - CHASSIS_NORMAL_OMEGA) * slope;
  }

  if (current_state.chassis_control_mode == ChassisControlMode::MANUAL) {
    double vx_ref = vx_max * (-1) * ps4_->get_left_stick_x();
    double vy_ref = vy_max * ps4_->get_left_stick_y();
    double vz_ref = vz_max * ps4_->get_right_stick_x();
    if (ps4_->is_pushed_left_stick()) {
      chassis_rotate90 = !chassis_rotate90;
    }
    if (chassis_rotate90) {
      // 赤ゾーンの場合はロボットを90度回転、青ゾーンの場合はロボットを-90度回転させる
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
  if (current_state.main != MainState::READY && current_state.main != MainState::EMERGENCY) {
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
  // PSボタン: 短押し→reset_robot(true)、長押し→reset_robot(false) + 緑点滅
  if (ps4_->is_pushed_ps()) {
    ps_press_start_time_ = this->now();
    ps_long_press_triggered_ = false;
  }
  if (ps4_->is_pushing_ps() && !ps_long_press_triggered_) {
    if ((this->now() - ps_press_start_time_).seconds() >= PS_LONG_PRESS_SEC) {
      ps_long_press_triggered_ = true;
      is_act_paused_ = false;
      if (activate_lidar_on_ps_) {
        request_lidar_lifecycle_activation();
      }
      reset_robot(false);
      set_led_event(0, 50, 0, 0.2, 1.0);  // 緑点滅（reset_robot内の青を上書き）
      publish_r1_machine_initialize();
      return;
    }
  }
  if (ps4_->is_released_ps() && !ps_long_press_triggered_) {
    is_act_paused_ = false;
    if (activate_lidar_on_ps_) {
      request_lidar_lifecycle_activation();
    }
    reset_robot(true);
    publish_r1_machine_initialize();
    return;
  }
  // 右スティック押下: 長押し判定の開始時刻をリセット
  if (ps4_->is_pushed_right_stick()) {
    bool right_stick_handled = false;
    if (is_act_paused_) {
      // ポーズ中 → レジューム
      publish_chassis_act_resume();
      is_act_paused_ = false;
      right_stick_handled = true;
    } else {
      const bool auto_running =
        (chassis_act_status_ != ChassisAct::NONE && chassis_act_status_ != ChassisAct::ACT_PAUSE) ||
        pending_auto_robot_move_valid_;
      if (auto_running) {
        right_stick_handled = true;
        const bool is_pausable =
          enable_right_stick_pause_ &&
          (chassis_act_status_ == ChassisAct::ACT2 || chassis_act_status_ == ChassisAct::ACT3 ||
           chassis_act_status_ == ChassisAct::ACT4 || chassis_act_status_ == ChassisAct::ACT5);
        if (is_pausable) {
          // ACT2以降かつパラメータ有効 → ポーズ
          publish_chassis_act_pause();
          is_act_paused_ = true;
        } else {
          // それ以外 → 停止（既存の挙動）
          publish_chassis_act_stop();
          clear_auto_chassis_state(true);
          chassis_act_status_ = ChassisAct::NONE;
          next_state.chassis_control_mode = ChassisControlMode::MANUAL;
        }
      }
    }
    if (!right_stick_handled) {
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
        manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
        // 番号の指定は千田機構だったときの名残で2を指定
        manual_mode3_make_spear_task(2);

      } else if (current_state.operation_mode == OperationMode::MODE3_SPEAR) {
        // MODE3のときはOUTER_ACTIVEのときはACT4_STARTを、INNER_ACTIVEのときはACT2_STARTを開始する
        // enable_auto_select=true  → r1_init_parameter_.r1_kfs_value（森番号3つ）から経路を決定
        // enable_auto_select=false → r1_collect_kfsの内容をそのまま経路として使用
        std::vector<int> forest_order;
        std::vector<std::string> collect_kfs_type;
        KfsAutoCollectStatus mode3_collect_status = KfsAutoCollectStatus::NONE;

        auto is_inner_forest = [](int n) {
          return n == 1 || n == 2 || n == 4 || n == 7 || n == 10;
        };
        auto is_outer_forest = [](int n) {
          return n == 3 || n == 6 || n == 9 || n == 10 || n == 11 || n == 12;
        };
        auto is_valid_forest = [](int n) { return n >= 1 && n <= 12 && n != 5 && n != 8; };
        auto determine_status = [&](const std::vector<int> & forests) {
          int inner_count = 0, outer_count = 0;
          for (int n : forests) {
            if (is_inner_forest(n)) inner_count++;
            if (is_outer_forest(n)) outer_count++;
          }
          RCLCPP_INFO(
            this->get_logger(), "MODE3 INNER/OUTER vote: inner=%d, outer=%d", inner_count,
            outer_count);
          return (outer_count > inner_count) ? KfsAutoCollectStatus::OUTER_ACTIVE
                                             : KfsAutoCollectStatus::INNER_ACTIVE;
        };

        if (!received_r1_init_parameter_) {
          RCLCPP_WARN(
            this->get_logger(),
            "MODE3: /r1_init_parameter not yet received. Cannot start auto advance.");
        } else if (r1_init_parameter_.enable_auto_select) {
          // --- enable_auto_select=true: r1_kfs_value（森番号3つ）から経路を生成 ---
          const auto & kfs_val = r1_init_parameter_.r1_kfs_value;
          if (kfs_val.size() != 3) {
            RCLCPP_WARN(
              this->get_logger(),
              "MODE3 enable_auto_select=true: r1_kfs_value must have 3 elements (got %zu).",
              kfs_val.size());
          } else {
            // 無効な値はWARNを出して読み飛ばし、有効な森番号のみ収集
            std::vector<int> all_forests;
            for (int n : kfs_val) {
              if (!is_valid_forest(n)) {
                RCLCPP_WARN(
                  this->get_logger(),
                  "MODE3 enable_auto_select=true: r1_kfs_value contains invalid forest number "
                  "%d.",
                  n);
                continue;
              }
              all_forests.push_back(n);
            }
            if (!all_forests.empty()) {
              // 有効な値で多数決してINNER/OUTERを決定する
              mode3_collect_status = determine_status(all_forests);
              // 優先順位リストに従って森を抽出（最大2つ = front_kfs + rear_kfs）
              // INNER: 2,1,4,7,10 の順、OUTER: 3,6,9,12,11,10 の順
              const std::vector<int> priority_list =
                (mode3_collect_status == KfsAutoCollectStatus::INNER_ACTIVE)
                  ? std::vector<int>{2, 1, 4, 7, 10}
                  : std::vector<int>{3, 6, 9, 12, 11, 10};
              for (int prio : priority_list) {
                if (forest_order.size() >= 2) break;
                for (int v : all_forests) {
                  if (v == prio) {
                    forest_order.push_back(prio);
                    break;
                  }
                }
              }
              if (forest_order.empty()) {
                RCLCPP_WARN(
                  this->get_logger(),
                  "MODE3 enable_auto_select=true: no forest in r1_kfs_value matches %s.",
                  kfs_auto_collect_status_name(mode3_collect_status).c_str());
                mode3_collect_status = KfsAutoCollectStatus::NONE;
              } else {
                // 青ゾーン: INNER→[rear, front], OUTER→[front, rear]
                // 赤ゾーン: INNER→[front, rear], OUTER→[rear, front]（逆）
                const bool is_blue = (zone_ == "blue");
                const bool is_inner = (mode3_collect_status == KfsAutoCollectStatus::INNER_ACTIVE);
                const std::string first_type = (is_blue == is_inner) ? "rear_kfs" : "front_kfs";
                const std::string rest_type = (is_blue == is_inner) ? "front_kfs" : "rear_kfs";
                for (size_t i = 0; i < forest_order.size(); i++) {
                  collect_kfs_type.push_back(i == 0 ? first_type : rest_type);
                }
                RCLCPP_INFO(
                  this->get_logger(),
                  "MODE3 enable_auto_select=true: status=%s zone=%s forests=[%d,%d,%d] -> "
                  "selected "
                  "%zu first=%s",
                  kfs_auto_collect_status_name(mode3_collect_status).c_str(), zone_.c_str(),
                  kfs_val[0], kfs_val[1], kfs_val[2], forest_order.size(), first_type.c_str());
              }
            }
          }
        } else {
          // --- enable_auto_select=false: r1_collect_kfsの内容をそのまま使用 ---
          if (!received_r1_collect_kfs_) {
            RCLCPP_WARN(
              this->get_logger(),
              "MODE3 enable_auto_select=false: /r1_collect_kfs not yet received.");
          } else if (r1_collect_kfs_.forest_order.empty()) {
            RCLCPP_WARN(
              this->get_logger(),
              "MODE3 enable_auto_select=false: r1_collect_kfs.forest_order is empty.");
          } else {
            bool valid = true;
            for (int n : r1_collect_kfs_.forest_order) {
              if (!is_valid_forest(n)) {
                RCLCPP_WARN(
                  this->get_logger(),
                  "MODE3 enable_auto_select=false: r1_collect_kfs.forest_order contains invalid "
                  "forest number %d.",
                  n);
                valid = false;
                break;
              }
            }
            if (valid) {
              forest_order.assign(
                r1_collect_kfs_.forest_order.begin(), r1_collect_kfs_.forest_order.end());
              collect_kfs_type = r1_collect_kfs_.kfs_mechanism_type;
              mode3_collect_status = determine_status(forest_order);
              RCLCPP_INFO(
                this->get_logger(),
                "MODE3 enable_auto_select=false: using r1_collect_kfs directly, status=%s",
                kfs_auto_collect_status_name(mode3_collect_status).c_str());
            }
          }
        }

        if (mode3_collect_status == KfsAutoCollectStatus::OUTER_ACTIVE && !forest_order.empty()) {
          start_auto_chassis(ChassisAct::ACT4_START, forest_order, collect_kfs_type);
          start_kfs_auto_collect(
            KfsAutoCollectStatus::OUTER_ACTIVE, std::move(forest_order),
            std::move(collect_kfs_type));
          started_auto = true;
        } else if (
          mode3_collect_status == KfsAutoCollectStatus::INNER_ACTIVE && !forest_order.empty()) {
          bool is_contain_1_or_2 = false;
          for (int fn : forest_order) {
            if (fn == 1 || fn == 2) {
              is_contain_1_or_2 = true;
              break;
            }
          }
          if (is_contain_1_or_2) {
            start_auto_chassis(ChassisAct::ACT2_START, forest_order, collect_kfs_type);
          } else {
            // ACT3_STARTは調整中のためACT2_STARTで代用
            start_auto_chassis(ChassisAct::ACT2_START, forest_order, collect_kfs_type);
            // start_auto_chassis(ChassisAct::ACT3_START, forest_order, collect_kfs_type);
          }
          start_kfs_auto_collect(
            KfsAutoCollectStatus::INNER_ACTIVE, std::move(forest_order),
            std::move(collect_kfs_type));
          started_auto = true;
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
      next_state.operation_mode = prev_operation_mode(current_state.operation_mode);
      share_long_press_triggered_ = true;
    }
  }
  if (ps4_->is_released_share() && !share_long_press_triggered_) {
    next_state.operation_mode = next_operation_mode(current_state.operation_mode);
  }

  const bool chassis_auto_requested =
    (chassis_act_status_ != ChassisAct::NONE && chassis_act_status_ != ChassisAct::ACT_PAUSE) ||
    pending_auto_robot_move_valid_ ||
    (auto_chassis_status_ != ChassisAct::NONE && auto_chassis_status_ != ChassisAct::ACT_PAUSE);
  next_state.chassis_control_mode =
    chassis_auto_requested ? ChassisControlMode::AUTO : ChassisControlMode::MANUAL;
  state_machine_->set_next_state(next_state);

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
