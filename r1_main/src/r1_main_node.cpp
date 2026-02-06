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

  // ポンプ
  kfs_front_pump_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_front_pump_gpio_pwm_ref", 10);
  kfs_rear_pump_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_rear_pump_gpio_pwm_ref", 10);
  // 電磁弁
  kfs_front_valve_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_front_valve_gpio_pwm_ref", 10);
  kfs_rear_valve_gpio_pwm_ref_publisher_ =
    this->create_publisher<r1_msgs::msg::GpioPwmRef>("/kfs_rear_valve_gpio_pwm_ref", 10);
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
    // TODO: 必要に応じて、符号の反転や係数をかける。
    double stick_max_velocity = 1.5;
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
  } else {
    target_vel_.linear.x = 0.0;
    target_vel_.linear.y = 0.0;
    target_vel_.angular.z = 0.0;
  }
  // デバッグ処理

  static bool kfs_fx_pushed = false;
  static bool kfs_fz_pushed = false;
  static bool kfs_fyaw_pushed = false;
  static bool kfs_fpump_pushed = false;
  static bool kfs_fvalve_pushed = false;
  static bool kfs_rx_pushed = false;
  static bool kfs_rz_pushed = false;
  static bool kfs_rpump_pushed = false;
  static bool kfs_ryaw_pushed = false;
  static bool kfs_rvalve_pushed = false;
  static bool front_expand_pushed = false;
  static bool rear_expand_pushed = false;

  // // ========== KFS回収前 ==========
  // if (ps4_->is_pushed_triangle()) {
  //   if (kfs_fx_pushed) {
  //     kfs_fx(0.0);
  //     RCLCPP_INFO(this->get_logger(), "kfs fx pos 0.0");
  //   } else {
  //     kfs_fx(0.5);
  //     RCLCPP_INFO(this->get_logger(), "kfs fx pos 0.3");
  //   }
  //   kfs_fx_pushed = !kfs_fx_pushed;
  // }

  // if (ps4_->is_pushed_circle()) {
  //   if (kfs_fz_pushed) {
  //     kfs_fz(0.0);
  //     RCLCPP_INFO(this->get_logger(), "kfs fz pos 0.0");
  //   } else {
  //     kfs_fz(0.3);
  //     RCLCPP_INFO(this->get_logger(), "kfs fz pos 0.4");
  //   }
  //   kfs_fz_pushed = !kfs_fz_pushed;
  // }

  // if (ps4_->is_pushed_cross()) {
  //   if (kfs_fyaw_pushed) {
  //     kfs_fyaw(0.0);
  //     RCLCPP_INFO(this->get_logger(), "kfs fyaw pos 0.0");
  //   } else {
  //     kfs_fyaw(1.57);
  //     RCLCPP_INFO(this->get_logger(), "kfs fyaw pos 1.57");
  //   }
  //   kfs_fyaw_pushed = !kfs_fyaw_pushed;
  // }

  // if (ps4_->is_pushed_square()) {
  //   if (kfs_fpump_pushed) {
  //     kfs_front_pump(0.0);
  //     RCLCPP_INFO(this->get_logger(), "kfs front pump off");
  //   } else {
  //     kfs_front_pump(1.0);
  //     RCLCPP_INFO(this->get_logger(), "kfs front pump on");
  //   }
  //   kfs_fpump_pushed = !kfs_fpump_pushed;
  // }

  // if (ps4_->is_pushed_up()) {
  //   kfs_fx_detect_origin();
  // }

  // if (ps4_->is_pushed_right()) {
  //   kfs_fz_detect_origin();
  // }

  // if (ps4_->is_pushed_down()) {
  //   kfs_fyaw_detect_origin();
  // }

  // if (ps4_->is_pushed_left()) {
  //   if (kfs_fvalve_pushed) {
  //     kfs_front_valve(false);
  //     RCLCPP_INFO(this->get_logger(), "kfs front valve off");
  //   } else {
  //     kfs_front_valve(true);
  //     RCLCPP_INFO(this->get_logger(), "kfs front valve on");
  //   }
  //   kfs_fvalve_pushed = !kfs_fvalve_pushed;
  // }

  // ========== KFS回収後 ==========
  if (ps4_->is_pushed_triangle()) {
    if (kfs_rx_pushed) {
      kfs_rx(0.0);
      RCLCPP_INFO(this->get_logger(), "kfs rx pos 0.0");
    } else {
      kfs_rx(0.3);
      RCLCPP_INFO(this->get_logger(), "kfs rx pos 0.3");
    }
    kfs_rx_pushed = !kfs_rx_pushed;
  }

  if (ps4_->is_pushed_circle()) {
    if (kfs_rz_pushed) {
      kfs_rz(0.0);
      RCLCPP_INFO(this->get_logger(), "kfs rz pos 0.0");
    } else {
      kfs_rz(0.3);
      RCLCPP_INFO(this->get_logger(), "kfs rz pos 0.4");
    }
    kfs_rz_pushed = !kfs_rz_pushed;
  }

  if (ps4_->is_pushed_cross()) {
    if (kfs_ryaw_pushed) {
      kfs_ryaw(0.0);
      RCLCPP_INFO(this->get_logger(), "kfs ryaw pos 0.0");
    } else {
      kfs_ryaw(1.57);
      RCLCPP_INFO(this->get_logger(), "kfs ryaw pos 1.57");
    }
    kfs_ryaw_pushed = !kfs_ryaw_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (kfs_rpump_pushed) {
      kfs_rear_pump(0.0);
      RCLCPP_INFO(this->get_logger(), "kfs rear pump off");
    } else {
      kfs_rear_pump(1.0);
      RCLCPP_INFO(this->get_logger(), "kfs rear pump on");
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
      RCLCPP_INFO(this->get_logger(), "kfs rear valve off");
    } else {
      kfs_rear_valve(true);
      RCLCPP_INFO(this->get_logger(), "kfs rear valve on");
    }
    kfs_rvalve_pushed = !kfs_rvalve_pushed;
  }

  // if (ps4_->is_pushed_square()) {
  //   if (kfs_rx_pushed) {
  //     kfs_rx(0.0);
  //   } else {
  //     kfs_rx(0.1);
  //   }
  //   kfs_rx_pushed = !kfs_rx_pushed;
  // }

  // if (ps4_->is_pushed_up()) {
  //   if (kfs_rz_pushed) {
  //     kfs_rz(0.0);
  //   } else {
  //     kfs_rz(0.1);
  //   }
  //   kfs_rz_pushed = !kfs_rz_pushed;
  // }

  // if (ps4_->is_pushed_right()) {
  //   if (kfs_ryaw_pushed) {
  //     kfs_ryaw(0.0);
  //   } else {
  //     kfs_ryaw(0.1);
  //   }
  //   kfs_ryaw_pushed = !kfs_ryaw_pushed;
  // }

  // if (ps4_->is_pushed_down()) {
  //   if (front_expand_pushed) {
  //     front_expand(0.0);
  //   } else {
  //     front_expand(0.1);
  //   }
  //   front_expand_pushed = !front_expand_pushed;
  // }

  // if (ps4_->is_pushed_left()) {
  //   if (rear_expand_pushed) {
  //     rear_expand(0.0);
  //   } else {
  //     rear_expand(0.1);
  //   }
  //   rear_expand_pushed = !rear_expand_pushed;
  // }

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