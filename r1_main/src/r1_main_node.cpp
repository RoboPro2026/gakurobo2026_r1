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

R1MainNode::R1MainNode() : Node("r1_main_node")
{
  // 足回りの速度指令
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  // joy
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&R1MainNode::joy_callback, this, std::placeholders::_1));
  // ========== KFS回収 ==========
  // 指令値Publisher
  kfs_fx_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_fx_position_ref", 10);
  kfs_fz_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_fz_position_ref", 10);
  kfs_fyaw_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_fyaw_position_ref", 10);
  kfs_rx_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_rx_position_ref", 10);
  kfs_rz_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_rz_position_ref", 10);
  kfs_ryaw_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/kfs_ryaw_position_ref", 10);
  // 原点検出Publisher
  kfs_fx_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_fx_detect_origin", 10);
  kfs_fz_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_fz_detect_origin", 10);
  kfs_fyaw_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_fyaw_detect_origin", 10);
  kfs_rx_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_rx_detect_origin", 10);
  kfs_rz_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_rz_detect_origin", 10);
  kfs_ryaw_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/kfs_ryaw_detect_origin", 10);
  // ========== 展開 ==========
  // 指令値Publisher
  front_expand_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/front_expand_position_ref", 10);
  rear_expand_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/rear_expand_position_ref", 10);
  // 原点検出Publisher
  front_expand_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/front_expand_detect_origin", 10);
  rear_expand_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/rear_expand_detect_origin", 10);
  // ========== R2昇降指令値 ==========
  r2_lift_motor_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::MotorRef>("/r2_lift_motor_ref", 10);
  // ========== ポール回収 ==========
  // 指令値Publisher
  pole_x_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/pole_x_position_ref", 10);
  pole_y_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/pole_y_position_ref", 10);
  pole_roger_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/pole_roger_position_ref", 10);
  // 原点検出Publisher
  pole_x_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/pole_x_detect_origin", 10);
  pole_y_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/pole_y_detect_origin", 10);
  pole_roger_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/pole_roger_detect_origin", 10);
  // ========== やり ==========
  // 指令値Publisher
  spear_roger1_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/spear_roger1_position_ref", 10);
  spear_roger2_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/spear_roger2_position_ref", 10);
  spear_move_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/spear_move_position_ref", 10);
  spear_rotate_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/spear_rotate_position_ref", 10);
  // 原点検出Publisher
  spear_roger1_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/spear_roger1_detect_origin", 10);
  spear_roger2_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/spear_roger2_detect_origin", 10);
  spear_move_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/spear_move_detect_origin", 10);
  spear_rotate_detect_origin_publisher_ =
    this->create_publisher<std_msgs::msg::Bool>("/spear_rotate_detect_origin", 10);

  // modeのSubscription
  kfs_fx_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_fx_mode_status", 10,
    std::bind(&R1MainNode::kfs_fx_mode_status_callback, this, std::placeholders::_1));
  kfs_fz_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_fz_mode_status", 10,
    std::bind(&R1MainNode::kfs_fz_mode_status_callback, this, std::placeholders::_1));
  kfs_fyaw_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_fyaw_mode_status", 10,
    std::bind(&R1MainNode::kfs_fyaw_mode_status_callback, this, std::placeholders::_1));
  kfs_rx_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_rx_mode_status", 10,
    std::bind(&R1MainNode::kfs_rx_mode_status_callback, this, std::placeholders::_1));
  kfs_rz_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_rz_mode_status", 10,
    std::bind(&R1MainNode::kfs_rz_mode_status_callback, this, std::placeholders::_1));
  kfs_ryaw_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/kfs_ryaw_mode_status", 10,
    std::bind(&R1MainNode::kfs_ryaw_mode_status_callback, this, std::placeholders::_1));
  // 展開のmode Subscription
  front_expand_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/front_expand_mode_status", 10,
    std::bind(&R1MainNode::front_expand_mode_status_callback, this, std::placeholders::_1));
  rear_expand_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/rear_expand_mode_status", 10,
    std::bind(&R1MainNode::rear_expand_mode_status_callback, this, std::placeholders::_1));
  pole_x_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/pole_x_mode_status", 10,
    std::bind(&R1MainNode::pole_x_mode_status_callback, this, std::placeholders::_1));
  pole_y_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/pole_y_mode_status", 10,
    std::bind(&R1MainNode::pole_y_mode_status_callback, this, std::placeholders::_1));
  pole_roger_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/pole_roger_mode_status", 10,
    std::bind(&R1MainNode::pole_roger_mode_status_callback, this, std::placeholders::_1));
  spear_roger1_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/spear_roger1_mode_status", 10,
    std::bind(&R1MainNode::spear_roger1_mode_status_callback, this, std::placeholders::_1));
  spear_roger2_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/spear_roger2_mode_status", 10,
    std::bind(&R1MainNode::spear_roger2_mode_status_callback, this, std::placeholders::_1));
  spear_move_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/spear_move_mode_status", 10,
    std::bind(&R1MainNode::spear_move_mode_status_callback, this, std::placeholders::_1));
  spear_rotate_mode_status_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/spear_rotate_mode_status", 10,
    std::bind(&R1MainNode::spear_rotate_mode_status_callback, this, std::placeholders::_1));

  // KFS回収用真空ポンプ
  kfs_front_pump_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_front_pump_gpio_pwm_ref", 10);
  kfs_rear_pump_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_rear_pump_gpio_pwm_ref", 10);
  // KFS真空電磁弁
  kfs_front_valve_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_front_valve_gpio_pwm_ref", 10);
  kfs_rear_valve_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_rear_valve_gpio_pwm_ref", 10);
  // KFSリミットスイッチ
  kfs_front_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/kfs_front_switch_status", 10,
    std::bind(&R1MainNode::kfs_front_switch_status_callback, this, std::placeholders::_1));
  kfs_rear_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/kfs_rear_switch_status", 10,
    std::bind(&R1MainNode::kfs_rear_switch_status_callback, this, std::placeholders::_1));
  // ポール回収サーボ
  pole_servo1_gpio_servo_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioServoRef>("/pole_servo1_gpio_servo_ref", 10);
  pole_servo2_gpio_servo_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioServoRef>("/pole_servo2_gpio_servo_ref", 10);
  pole_servo3_gpio_servo_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioServoRef>("/pole_servo3_gpio_servo_ref", 10);
  pole_servo4_gpio_servo_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioServoRef>("/pole_servo4_gpio_servo_ref", 10);
  //  ポール回収電磁弁
  pole_valve1_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/pole_valve1_gpio_pwm_ref", 10);
  pole_valve2_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/pole_valve2_gpio_pwm_ref", 10);
  pole_valve3_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/pole_valve3_gpio_pwm_ref", 10);
  pole_valve4_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/pole_valve4_gpio_pwm_ref", 10);
  // やりハンド電磁弁
  spear_hand_valve_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/spear_hand_valve_gpio_pwm_ref", 10);
  // やりリミットスイッチ
  spear_move_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/spear_move_switch_status", 10,
    std::bind(&R1MainNode::spear_move_switch_status_callback, this, std::placeholders::_1));
  spear_rotate_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/spear_rotate_switch_status", 10,
    std::bind(&R1MainNode::spear_rotate_switch_status_callback, this, std::placeholders::_1));

  timer_publisher_ = this->create_wall_timer(10ms, std::bind(&R1MainNode::timer_callback, this));

  simple_trapezoid_vx_ = SimpleTrapezoid(3.0, 0.01);  // 加速度1.0m/s^2、制御周期10ms
  simple_trapezoid_vy_ = SimpleTrapezoid(3.0, 0.01);
  simple_trapezoid_omega_ = SimpleTrapezoid(3.0, 0.01);

  ps4_ = std::make_shared<PS4>("PS4");

  state_machine_ = std::make_shared<StateMachine>();
  state_machine_->set_next_state({MainState::MANUAL, ManualSubState::NONE});
}

void R1MainNode::kfs_fx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_fx_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs fx detected origin");
  }
  is_kfs_fx_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_fz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_fz_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs fz detected origin");
  }
  is_kfs_fz_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_fyaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_fyaw_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs fyaw detected origin");
  }
  is_kfs_fyaw_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_rx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_rx_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs rx detected origin");
  }
  is_kfs_rx_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_rz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_rz_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs rz detected origin");
  }
  is_kfs_rz_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_ryaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_kfs_ryaw_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "kfs ryaw detected origin");
  }
  is_kfs_ryaw_pos_mode_ = _is_pos_mode;
}

void R1MainNode::front_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_front_expand_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "front expand assist detected origin");
  }
  is_front_expand_pos_mode_ = _is_pos_mode;
}

void R1MainNode::rear_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_rear_expand_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "rear expand assist detected origin");
  }
  is_rear_expand_pos_mode_ = _is_pos_mode;
}

void R1MainNode::pole_x_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_pole_x_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "pole x detected origin");
  }
  is_pole_x_pos_mode_ = _is_pos_mode;
}

void R1MainNode::pole_y_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_pole_y_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "pole y detected origin");
  }
  is_pole_y_pos_mode_ = _is_pos_mode;
}

void R1MainNode::pole_roger_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_pole_roger_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "pole roger detected origin");
  }
  is_pole_roger_pos_mode_ = _is_pos_mode;
}

void R1MainNode::spear_roger1_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_spear_roger1_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "spear roger1 detected origin");
  }
  is_spear_roger1_pos_mode_ = _is_pos_mode;
}

void R1MainNode::spear_roger2_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_spear_roger2_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "spear roger2 detected origin");
  }
  is_spear_roger2_pos_mode_ = _is_pos_mode;
}

void R1MainNode::spear_move_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_spear_move_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "spear move detected origin");
  }
  is_spear_move_pos_mode_ = _is_pos_mode;
}

void R1MainNode::spear_rotate_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  auto mode = msg->data;
  bool _is_pos_mode = (mode == 0);
  if (_is_pos_mode == true && is_spear_rotate_pos_mode_ == false) {
    RCLCPP_INFO(this->get_logger(), "spear rotate detected origin");
  }
  is_spear_rotate_pos_mode_ = _is_pos_mode;
}

void R1MainNode::kfs_front_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  kfs_front_switch_status_ = msg->status;
}

void R1MainNode::kfs_rear_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  kfs_rear_switch_status_ = msg->status;
}

void R1MainNode::spear_move_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  spear_move_switch_status_ = msg->status;
}

void R1MainNode::spear_rotate_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  spear_rotate_switch_status_ = msg->status;
}

void R1MainNode::kfs_fx_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_fx_detect_origin_publisher_->publish(msg);
}

void R1MainNode::kfs_fz_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_fz_detect_origin_publisher_->publish(msg);
}

void R1MainNode::kfs_fyaw_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_fyaw_detect_origin_publisher_->publish(msg);
}

void R1MainNode::kfs_rx_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_rx_detect_origin_publisher_->publish(msg);
}

void R1MainNode::kfs_rz_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_rz_detect_origin_publisher_->publish(msg);
}

void R1MainNode::kfs_ryaw_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  kfs_ryaw_detect_origin_publisher_->publish(msg);
}

void R1MainNode::front_expand_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  front_expand_detect_origin_publisher_->publish(msg);
}

void R1MainNode::rear_expand_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  rear_expand_detect_origin_publisher_->publish(msg);
}

void R1MainNode::pole_x_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pole_x_detect_origin_publisher_->publish(msg);
}

void R1MainNode::pole_y_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pole_y_detect_origin_publisher_->publish(msg);
}

void R1MainNode::pole_roger_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pole_roger_detect_origin_publisher_->publish(msg);
}

void R1MainNode::spear_roger1_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  spear_roger1_detect_origin_publisher_->publish(msg);
}

void R1MainNode::spear_roger2_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  spear_roger2_detect_origin_publisher_->publish(msg);
}

void R1MainNode::spear_move_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  spear_move_detect_origin_publisher_->publish(msg);
}

void R1MainNode::spear_rotate_detect_origin(void)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  spear_rotate_detect_origin_publisher_->publish(msg);
}

// --- コールバック関数 ---
void R1MainNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  ps4_->joy_callback(msg);
}

void R1MainNode::timer_callback(void)
{
  ps4_->update();
  // 状態を更新
  state_machine_->update();
  // タスクを実行
  // ps4_->print_data();
  main_task();
}

void R1MainNode::kfs_fx(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fx_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fx pos %f", pos);
}

void R1MainNode::kfs_fz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fz_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fz pos %f", pos);
}

void R1MainNode::kfs_fyaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fyaw_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fyaw pos %f", pos);
}

void R1MainNode::kfs_rx(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rx_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rx pos %f", pos);
}

void R1MainNode::kfs_rz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rz_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rz pos %f", pos);
}

void R1MainNode::kfs_ryaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_ryaw_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs ryaw pos %f", pos);
}

void R1MainNode::front_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  front_expand_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "front expand pos %f", pos);
}

void R1MainNode::rear_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  rear_expand_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "rear expand pos %f", pos);
}

void R1MainNode::r2_lift(double vel)
{
  r1_msgs::msg::MotorRef msg;
  msg.control_type = "VELOCITY";
  msg.ref = vel;
  r2_lift_motor_ref_publisher_->publish(msg);
}

void R1MainNode::pole_x(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_x_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole x pos %f", pos);
}

void R1MainNode::pole_y(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_y_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole y pos %f", pos);
}

void R1MainNode::pole_roger(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_roger_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole roger pos %f", pos);
}

void R1MainNode::spear_roger1(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_roger1_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear roger1 pos %f", pos);
}

void R1MainNode::spear_roger2(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_roger2_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear roger2 pos %f", pos);
}

void R1MainNode::spear_move(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_move_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear move pos %f", pos);
}

void R1MainNode::spear_rotate(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_rotate_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear rotate pos %f", pos);
}

void R1MainNode::kfs_front_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_front_pump_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs front pump pwm %f", pwm);
}

void R1MainNode::kfs_rear_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_rear_pump_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rear pump pwm %f", pwm);
}

void R1MainNode::kfs_front_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_front_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs front valve %d", on);
}

void R1MainNode::kfs_rear_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_rear_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rear valve %d", on);
}

void R1MainNode::pole_servo1(int angle)
{
  r1_msgs::msg::GpioServoRef msg;
  msg.ref = angle;
  pole_servo1_gpio_servo_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole servo1 angle %d", angle);
}

void R1MainNode::pole_servo2(int angle)
{
  r1_msgs::msg::GpioServoRef msg;
  msg.ref = angle;
  pole_servo2_gpio_servo_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole servo2 angle %d", angle);
}

void R1MainNode::pole_servo3(int angle)
{
  r1_msgs::msg::GpioServoRef msg;
  msg.ref = angle;
  pole_servo3_gpio_servo_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole servo3 angle %d", angle);
}

void R1MainNode::pole_servo4(int angle)
{
  r1_msgs::msg::GpioServoRef msg;
  msg.ref = angle;
  pole_servo4_gpio_servo_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole servo4 angle %d", angle);
}

void R1MainNode::pole_valve1(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  pole_valve1_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole valve1 %d", on);
}

void R1MainNode::pole_valve2(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  pole_valve2_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole valve2 %d", on);
}

void R1MainNode::pole_valve3(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  pole_valve3_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole valve3 %d", on);
}

void R1MainNode::pole_valve4(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  pole_valve4_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole valve4 %d", on);
}

void R1MainNode::spear_hand_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  spear_hand_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear hand valve %d", on);
}

// --- 各状態のタスク ---
void R1MainNode::idle_task(void)
{
  // 速度指令値を0にする
  target_vel_.linear.x = 0.0;
  target_vel_.linear.y = 0.0;
  target_vel_.angular.z = 0.0;
  cmd_vel_publisher_->publish(target_vel_);
}

void R1MainNode::emergency_task(void)
{
  // 速度指令値を0にする
  target_vel_.linear.x = 0.0;
  target_vel_.linear.y = 0.0;
  target_vel_.angular.z = 0.0;
  cmd_vel_publisher_->publish(target_vel_);
}

void R1MainNode::manual_task(void)
{
  if (ps4_->is_connected()) {
    // スティックの状態に応じて、足回りを動かす
    // TODO: 必要に応じて、符号の反転や係数をかける。
    double stick_max_velocity = 1.5;
    double vx_ref = stick_max_velocity * ps4_->data.left_stick_x;
    double vy_ref = stick_max_velocity * ps4_->data.left_stick_y;
    double vz_ref = ps4_->data.right_stick_x;
    // 台形制御で速度を滑らかに変化させる
    target_vel_.linear.x = simple_trapezoid_vx_.update(vx_ref);
    target_vel_.linear.y = simple_trapezoid_vy_.update(vy_ref);
    target_vel_.angular.z = simple_trapezoid_omega_.update(vz_ref);

    test_front_kfs();
    // test_spear();
    // test_pole();
    // test_r2_lift();
  } else {
    target_vel_.linear.x = 0.0;
    target_vel_.linear.y = 0.0;
    target_vel_.angular.z = 0.0;
  }
  cmd_vel_publisher_->publish(target_vel_);
}

void R1MainNode::auto_task(void) {}

void R1MainNode::main_task(void)
{
  auto current_state = state_machine_->get_current_state();
  if (current_state.main == MainState::IDLE) {
    idle_task();
  } else if (current_state.main == MainState::EMERGENCY) {
    emergency_task();
  } else if (current_state.main == MainState::MANUAL) {
    manual_task();
  } else if (current_state.main == MainState::AUTO) {
    auto_task();
  }
}

void R1MainNode::test_front_kfs(void)
{
  static bool kfs_fx_pushed = false;
  static bool kfs_fz_pushed = false;
  static bool kfs_fyaw_pushed = false;
  static bool kfs_fpump_pushed = false;
  static bool kfs_fvalve_pushed = false;

  if (ps4_->is_pushed_triangle()) {
    if (kfs_fx_pushed) {
      kfs_fx(0.0);
    } else {
      kfs_fx(0.5);
    }
    kfs_fx_pushed = !kfs_fx_pushed;
  }

  if (ps4_->is_pushed_circle()) {
    if (kfs_fz_pushed) {
      kfs_fz(0.0);
    } else {
      kfs_fz(0.3);
    }
    kfs_fz_pushed = !kfs_fz_pushed;
  }

  if (ps4_->is_pushed_cross()) {
    if (kfs_fyaw_pushed) {
      kfs_fyaw(0.0);
    } else {
      kfs_fyaw(1.57);
    }
    kfs_fyaw_pushed = !kfs_fyaw_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (kfs_fpump_pushed) {
      kfs_front_pump(0.0);
    } else {
      kfs_front_pump(1.0);
    }
    kfs_fpump_pushed = !kfs_fpump_pushed;
  }

  if (ps4_->is_pushed_up()) {
    kfs_fx_detect_origin();
  }

  if (ps4_->is_pushed_right()) {
    kfs_fz_detect_origin();
  }

  if (ps4_->is_pushed_down()) {
    kfs_fyaw_detect_origin();
  }

  if (ps4_->is_pushed_left()) {
    if (kfs_fvalve_pushed) {
      kfs_front_valve(false);
    } else {
      kfs_front_valve(true);
    }
    kfs_fvalve_pushed = !kfs_fvalve_pushed;
  }
}

void R1MainNode::test_rear_kfs(void)
{
  static bool kfs_rx_pushed = false;
  static bool kfs_rz_pushed = false;
  static bool kfs_rpump_pushed = false;
  static bool kfs_ryaw_pushed = false;
  static bool kfs_rvalve_pushed = false;

  // ========== KFS回収後 ==========
  if (ps4_->is_pushed_triangle()) {
    if (kfs_rx_pushed) {
      kfs_rx(0.0);
    } else {
      kfs_rx(0.5);
    }
    kfs_rx_pushed = !kfs_rx_pushed;
  }

  if (ps4_->is_pushed_circle()) {
    if (kfs_rz_pushed) {
      kfs_rz(0.0);
    } else {
      kfs_rz(0.3);
    }
    kfs_rz_pushed = !kfs_rz_pushed;
  }

  if (ps4_->is_pushed_cross()) {
    if (kfs_ryaw_pushed) {
      kfs_ryaw(0.0);
    } else {
      kfs_ryaw(1.57);
    }
    kfs_ryaw_pushed = !kfs_ryaw_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (kfs_rpump_pushed) {
      kfs_rear_pump(0.0);
    } else {
      kfs_rear_pump(1.0);
    }
    kfs_rpump_pushed = !kfs_rpump_pushed;
  }

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
    if (kfs_rvalve_pushed) {
      kfs_rear_valve(false);
    } else {
      kfs_rear_valve(true);
    }
    kfs_rvalve_pushed = !kfs_rvalve_pushed;
  }
}

void R1MainNode::test_pole(void)
{
  static bool pole_x_pushed = false;
  static bool pole_y_pushed = false;
  static bool pole_roger_pushed = false;
  static bool pole_valve1_pushed = false;
  static bool pole_valve2_pushed = false;
  static bool pole_valve3_pushed = false;
  static bool pole_valve4_pushed = false;
  static bool pole_servo1_pushed = false;
  static bool pole_servo2_pushed = false;
  static bool pole_servo3_pushed = false;
  static bool pole_servo4_pushed = false;

  // サーボは約90度から約180度に回転させる。
  if (ps4_->is_pushed_triangle()) {
    if (pole_servo1_pushed) {
      pole_servo1(80);
    } else {
      pole_servo1(180);
    }
    pole_servo1_pushed = !pole_servo1_pushed;
  }
  if (ps4_->is_pushed_circle()) {
    if (pole_servo2_pushed) {
      pole_servo2(70);
    } else {
      pole_servo2(170);
    }
    pole_servo2_pushed = !pole_servo2_pushed;
  }
  if (ps4_->is_pushed_cross()) {
    if (pole_servo3_pushed) {
      pole_servo3(70);
    } else {
      pole_servo3(180);
    }
    pole_servo3_pushed = !pole_servo3_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (pole_servo4_pushed) {
      pole_servo4(70);
    } else {
      pole_servo4(185);
    }
    pole_servo4_pushed = !pole_servo4_pushed;
  }

  if (ps4_->is_pushed_up()) {
    if (pole_valve1_pushed) {
      pole_valve1(false);
    } else {
      pole_valve1(true);
    }
    pole_valve1_pushed = !pole_valve1_pushed;
  }

  if (ps4_->is_pushed_right()) {
    if (pole_valve2_pushed) {
      pole_valve2(false);
    } else {
      pole_valve2(true);
    }
    pole_valve2_pushed = !pole_valve2_pushed;
  }

  if (ps4_->is_pushed_down()) {
    if (pole_valve3_pushed) {
      pole_valve3(false);
    } else {
      pole_valve3(true);
    }
    pole_valve3_pushed = !pole_valve3_pushed;
  }

  if (ps4_->is_pushed_left()) {
    if (pole_valve4_pushed) {
      pole_valve4(false);
    } else {
      pole_valve4(true);
    }
    pole_valve4_pushed = !pole_valve4_pushed;
  }

  if (ps4_->is_pushed_r1()) {
    if (pole_x_pushed) {
      pole_x(0.0);
    } else {
      pole_x(0.1);
    }
    pole_x_pushed = !pole_x_pushed;
  }

  if (ps4_->is_pushed_r2()) {
    if (pole_y_pushed) {
      pole_y(0.0);
    } else {
      pole_y(0.3);
    }
    pole_y_pushed = !pole_y_pushed;
  }

  if (ps4_->is_pushed_options()) {
    if (pole_roger_pushed) {
      pole_roger(0.0);
    } else {
      pole_roger(0.45);
    }
    pole_roger_pushed = !pole_roger_pushed;
  }

  if (ps4_->is_pushed_l1()) {
    pole_x_detect_origin();
  }

  if (ps4_->is_pushed_l2()) {
    pole_y_detect_origin();
  }

  if (ps4_->is_pushed_share()) {
    pole_roger_detect_origin();
  }
}

void R1MainNode::test_spear(void)
{
  static bool spear_roger1_pushed = false;
  static bool spear_roger2_pushed = false;
  static bool spear_move_pushed = false;
  static bool spear_rotate_pushed = false;
  static bool spear_hand_valve_pushed = false;
  if (ps4_->is_pushed_triangle()) {
    if (spear_roger1_pushed) {
      spear_roger1(0.0);
    } else {
      spear_roger1(0.2);
    }
    spear_roger1_pushed = !spear_roger1_pushed;
  }

  if (ps4_->is_pushed_circle()) {
    if (spear_roger2_pushed) {
      spear_roger2(0.0);
    } else {
      spear_roger2(0.2);
    }
    spear_roger2_pushed = !spear_roger2_pushed;
  }

  if (ps4_->is_pushed_cross()) {
    if (spear_move_pushed) {
      spear_move(0.0);
    } else {
      spear_move(0.3);
    }
    spear_move_pushed = !spear_move_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (spear_rotate_pushed) {
      spear_rotate(0.0);
    } else {
      spear_rotate(1.57);
    }
    spear_rotate_pushed = !spear_rotate_pushed;
  }

  if (ps4_->is_pushed_r1()) {
    if (spear_hand_valve_pushed) {
      spear_hand_valve(false);
    } else {
      spear_hand_valve(true);
    }
    spear_hand_valve_pushed = !spear_hand_valve_pushed;
  }

  if (ps4_->is_pushed_up()) {
    spear_roger1_detect_origin();
  }

  if (ps4_->is_pushed_right()) {
    spear_roger2_detect_origin();
  }

  if (ps4_->is_pushed_down()) {
    spear_move_detect_origin();
  }

  if (ps4_->is_pushed_left()) {
    spear_rotate_detect_origin();
  }
}

void R1MainNode::test_r2_lift(void)
{
  static bool front_expand_pushed = false;
  static bool rear_expand_pushed = false;
  // 三角を押している間モータが正回転、バツを押している間モータが逆回転する
  if (ps4_->data.triangle) {
    r2_lift(15);
    RCLCPP_INFO(this->get_logger(), "r2 lift up");
  } else if (ps4_->data.cross) {
    r2_lift(-15);
    RCLCPP_INFO(this->get_logger(), "r2 lift down");
  } else {
    r2_lift(0);
  }

  if (ps4_->is_pushed_square()) {
    if (front_expand_pushed) {
      front_expand(0.0);
    } else {
      front_expand(0.1);
    }
    front_expand_pushed = !front_expand_pushed;
  }

  if (ps4_->is_pushed_circle()) {
    if (rear_expand_pushed) {
      rear_expand(0.0);
    } else {
      rear_expand(0.1);
    }
    rear_expand_pushed = !rear_expand_pushed;
  }

  if (ps4_->is_pushed_right()) {
    front_expand_detect_origin();
  }

  if (ps4_->is_pushed_left()) {
    rear_expand_detect_origin();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1MainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}