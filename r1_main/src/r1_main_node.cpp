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
  // KFS回収
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
  // 展開
  front_expand_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/front_expand_position_ref", 10);
  rear_expand_position_ref_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/rear_expand_position_ref", 10);
  // R2昇降
  r2_lift_motor_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::MotorRef>("/r2_lift_motor_ref", 10);
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

  // リミットスイッチ
  kfs_front_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/kfs_front_switch_status", 10,
    std::bind(&R1MainNode::kfs_front_switch_status_callback, this, std::placeholders::_1));
  kfs_rear_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
    "/kfs_rear_switch_status", 10,
    std::bind(&R1MainNode::kfs_rear_switch_status_callback, this, std::placeholders::_1));

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

void R1MainNode::kfs_front_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  kfs_front_switch_status_ = msg->status;
}

void R1MainNode::kfs_rear_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
{
  kfs_rear_switch_status_ = msg->status;
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
}

void R1MainNode::kfs_fz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fz_position_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_fyaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fyaw_position_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_rx(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rx_position_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_rz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rz_position_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_ryaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_ryaw_position_ref_publisher_->publish(msg);
}

void R1MainNode::front_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  front_expand_position_ref_publisher_->publish(msg);
}

void R1MainNode::rear_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  rear_expand_position_ref_publisher_->publish(msg);
}

void R1MainNode::r2_lift(double vel)
{
  r1_msgs::msg::MotorRef msg;
  msg.control_type = "VELOCITY";
  msg.ref = vel;
  r2_lift_motor_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_front_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_front_pump_gpio_pwm_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_rear_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_rear_pump_gpio_pwm_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_front_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_front_valve_gpio_pwm_ref_publisher_->publish(msg);
}

void R1MainNode::kfs_rear_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_rear_valve_gpio_pwm_ref_publisher_->publish(msg);
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
    double VEL = 1.0;
    if (ps4_->data.right) {
      target_vel_.linear.x = -VEL;
      target_vel_.linear.y = 0;
      target_vel_.angular.z = 0;
    } else if (ps4_->data.left) {
      target_vel_.linear.x = VEL;
      target_vel_.linear.y = 0;
      target_vel_.angular.z = 0;
    } else if (ps4_->data.up) {
      target_vel_.linear.x = 0;
      target_vel_.linear.y = VEL;
      target_vel_.angular.z = 0;
    } else if (ps4_->data.down) {
      target_vel_.linear.x = 0;
      target_vel_.linear.y = -VEL;
      target_vel_.angular.z = 0;
    } else {
      // TODO: 必要に応じて、符号の反転や係数をかける。
      double stick_max_velocity = 3.0;
      double vx_ref = stick_max_velocity * ps4_->data.left_stick_x;
      double vy_ref = stick_max_velocity * ps4_->data.left_stick_y;
      double vz_ref = ps4_->data.right_stick_x;
      // 台形制御で速度を滑らかに変化させる
      target_vel_.linear.x = simple_trapezoid_vx_.update(vx_ref);
      target_vel_.linear.y = simple_trapezoid_vy_.update(vy_ref);
      target_vel_.angular.z = simple_trapezoid_omega_.update(vz_ref);
      // RCLCPP_INFO(this->get_logger(), "vx: %.2f, vx_ref: %.2f", target_vel_.linear.x, vx_ref);
      // target_vel_.linear.x = stick_max_velocity * ps4_->data.left_stick_x;
      // target_vel_.linear.y = stick_max_velocity * ps4_->data.left_stick_y;
      // target_vel_.angular.z = ps4_->data.right_stick_x;
    }
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1MainNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}