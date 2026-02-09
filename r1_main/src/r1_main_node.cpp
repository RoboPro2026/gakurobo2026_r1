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

void R1MainNode::declare_and_get_parameter(
  const std::string & name, double & value, double default_value)
{
  this->declare_parameter<double>(name, default_value);
  this->get_parameter(name, value);
}

void R1MainNode::declare_and_get_parameter(const std::string & name, int & value, int default_value)
{
  this->declare_parameter<int>(name, default_value);
  this->get_parameter(name, value);
}

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

  // ========== Sabacan ==========
  // 電源基板の指令値Publisher
  sabacan_power_ref_publisher_ =
    this->create_publisher<sabacan_msgs::msg::SabacanPowerRef>("/sabacan_power_ref0", 10);
  // LED基板の指令値Publisher
  sabacan_led_ref_publisher_ =
    this->create_publisher<sabacan_msgs::msg::SabacanLEDRef>("/sabacan_led_ref1", 10);
  // 電源基板のリセットサービスClient
  sabacan_power_reset_client_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_power_reset");
  sabacan_robomas_reset_client_id1_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id1");
  sabacan_robomas_reset_client_id2_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id2");
  sabacan_robomas_reset_client_id3_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id3");
  sabacan_robomas_reset_client_id4_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id4");
  sabacan_robomas_reset_client_id5_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id5");
  sabacan_gpio_reset_client_id1_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id1");
  sabacan_gpio_reset_client_id2_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id2");
  sabacan_gpio_reset_client_id3_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id3");
  sabacan_led_reset_client_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_led_reset");

  // ========== パラメータ ==========
  // 足回り
  declare_and_get_parameter("chasssis_max_velocity", CHASSIS_MAX_VELOCITY);
  declare_and_get_parameter("chassis_max_omega", CHASSIS_MAX_OMEGA);

  // ========== KFS回収 ==========
  // fx
  declare_and_get_parameter("kfs_fx_normal_pos", KFS_FX_NORMAL_POS);
  declare_and_get_parameter("kfs_fx_expand_pos", KFS_FX_EXPAND_POS);
  // fz
  declare_and_get_parameter("kfs_fz_normal_pos", KFS_FZ_NORMAL_POS);
  declare_and_get_parameter("kfs_fz_low_pos", KFS_FZ_LOW_POS);
  declare_and_get_parameter("kfs_fz_middle_pos", KFS_FZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_fz_high_pos", KFS_FZ_HIGH_POS);
  // fyaw
  declare_and_get_parameter("kfs_fyaw_normal_angle", KFS_FYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_fyaw_front_angle", KFS_FYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_fyaw_side_angle", KFS_FYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_fyaw_rear_angle", KFS_FYAW_REAR_ANGLE);
  // rx
  declare_and_get_parameter("kfs_rx_normal_pos", KFS_RX_NORMAL_POS);
  declare_and_get_parameter("kfs_rx_expand_pos", KFS_RX_EXPAND_POS);
  // rz
  declare_and_get_parameter("kfs_rz_normal_pos", KFS_RZ_NORMAL_POS);
  declare_and_get_parameter("kfs_rz_low_pos", KFS_RZ_LOW_POS);
  declare_and_get_parameter("kfs_rz_middle_pos", KFS_RZ_MIDDLE_POS);
  declare_and_get_parameter("kfs_rz_high_pos", KFS_RZ_HIGH_POS);
  // ryaw
  declare_and_get_parameter("kfs_ryaw_normal_angle", KFS_RYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_ryaw_front_angle", KFS_RYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_ryaw_side_angle", KFS_RYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_ryaw_rear_angle", KFS_RYAW_REAR_ANGLE);

  // ========== 展開 ==========
  // R2昇降
  declare_and_get_parameter("r2_lift_max_velocity", R2_LIFT_MAX_VELOCITY);
  // front_expand
  declare_and_get_parameter("front_expand_normal_pos", FRONT_EXPAND_NORMAL_POS);
  declare_and_get_parameter("front_expand_expand_pos", FRONT_EXPAND_EXPAND_POS);
  // rear_expand
  declare_and_get_parameter("rear_expand_normal_pos", REAR_EXPAND_NORMAL_POS);
  declare_and_get_parameter("rear_expand_expand_pos", REAR_EXPAND_EXPAND_POS);

  // ========== ポール回収 ==========
  // pole_x
  declare_and_get_parameter("pole_x_normal_pos", POLE_X_NORMAL_POS);
  declare_and_get_parameter("pole_x_expand_pos", POLE_X_EXPAND_POS);
  // pole_y
  declare_and_get_parameter("pole_y_normal_pos", POLE_Y_NORMAL_POS);
  declare_and_get_parameter("pole_y_collect_pos", POLE_Y_COLLECT_POS);
  declare_and_get_parameter("pole_y_transfer1_pos", POLE_Y_TRANSFER1_POS);
  declare_and_get_parameter("pole_y_transfer2_pos", POLE_Y_TRANSFER2_POS);
  declare_and_get_parameter("pole_y_transfer3_pos", POLE_Y_TRANSFER3_POS);
  declare_and_get_parameter("pole_y_transfer4_pos", POLE_Y_TRANSFER4_POS);
  // pole_roger
  declare_and_get_parameter("pole_roger_normal_pos", POLE_ROGER_NORMAL_POS);
  declare_and_get_parameter("pole_roger_expand_pos", POLE_ROGER_EXPAND_POS);
  // servo、servoのみint型
  declare_and_get_parameter("pole_servo1_normal_angle", POLE_SERVO1_NORMAL_ANGLE);
  declare_and_get_parameter("pole_servo1_horizontal_angle", POLE_SERVO1_HORIZONTAL_ANGLE);
  declare_and_get_parameter("pole_servo2_normal_angle", POLE_SERVO2_NORMAL_ANGLE);
  declare_and_get_parameter("pole_servo2_horizontal_angle", POLE_SERVO2_HORIZONTAL_ANGLE);
  declare_and_get_parameter("pole_servo3_normal_angle", POLE_SERVO3_NORMAL_ANGLE);
  declare_and_get_parameter("pole_servo3_horizontal_angle", POLE_SERVO3_HORIZONTAL_ANGLE);
  declare_and_get_parameter("pole_servo4_normal_angle", POLE_SERVO4_NORMAL_ANGLE);
  declare_and_get_parameter("pole_servo4_horizontal_angle", POLE_SERVO4_HORIZONTAL_ANGLE);

  // ========== やり ==========
  // spear_roger1
  declare_and_get_parameter("spear_roger1_normal_pos", SPEAR_ROGER1_NORMAL_POS);
  declare_and_get_parameter("spear_roger1_combine_pos", SPEAR_ROGER1_COMBINE_POS);
  declare_and_get_parameter("spear_roger1_transfer_pos", SPEAR_ROGER1_TRANSFER_POS);
  // spear_roger2
  declare_and_get_parameter("spear_roger2_normal_pos", SPEAR_ROGER2_NORMAL_POS);
  declare_and_get_parameter("spear_roger2_combine_pos", SPEAR_ROGER2_COMBINE_POS);
  declare_and_get_parameter("spear_roger2_transfer_pos", SPEAR_ROGER2_TRANSFER_POS);
  // spear_move
  declare_and_get_parameter("spear_move_normal_pos", SPEAR_MOVE_NORMAL_POS);
  declare_and_get_parameter("spear_move_combine_pos", SPEAR_MOVE_COMBINE_POS);
  declare_and_get_parameter("spear_move_transfer_pos", SPEAR_MOVE_TRANSFER_POS);
  // spear_rotate
  declare_and_get_parameter("spear_rotate_normal_pos", SPEAR_ROTATE_NORMAL_POS);

  // タイマー
  timer_publisher_ = this->create_wall_timer(10ms, std::bind(&R1MainNode::timer_callback, this));

  simple_trapezoid_vx_ = SimpleTrapezoid(3.0, 0.01);  // 加速度1.0m/s^2、制御周期10ms
  simple_trapezoid_vy_ = SimpleTrapezoid(3.0, 0.01);
  simple_trapezoid_omega_ = SimpleTrapezoid(3.0, 0.01);

  ps4_ = std::make_shared<PS4>("PS4");

  state_machine_ = std::make_shared<StateMachine>();
  // state_machine_->set_next_state({MainState::MANUAL, ManualSubState::TEST});
  state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE2_SPEAR_AND_POLE});
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

void R1MainNode::sabacan_reset_update(void)
{
  // resetクライアントのやることがないときはreturn
  if (sabacan_reset_status_ == SABACAN_AVAILABLE) {
    return;
  }

  if (sabacan_reset_status_ == SABACAN_RESET_NOW) {
    sabacan_reset_status_ = SABACAN_RESET_SENDING;
    sabacan_reset_step_ = 0;
    sabacan_reset_last_send_valid_ = false;
  }

  const rclcpp::Duration send_interval = rclcpp::Duration::from_seconds(1.0);
  const auto now = this->get_clock()->now();
  if (sabacan_reset_last_send_valid_ && (now - sabacan_reset_last_send_time_) < send_interval) {
    return;
  }

  auto try_send = [this](
                    const rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr & client,
                    const char * service_name) -> void {
    auto request = std::make_shared<sabacan_msgs::srv::SabacanReset::Request>();
    client->async_send_request(
      request,
      [this, service_name](rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedFuture future) {
        (void)future.get();
        RCLCPP_INFO(this->get_logger(), "sabacan reset sent: %s", service_name);
      });
  };

  switch (sabacan_reset_step_) {
    case 0:
      try_send(sabacan_power_reset_client_, "/sabacan_power_reset");
      break;
    case 1:
      try_send(sabacan_robomas_reset_client_id1_, "/sabacan_robomas_reset_id1");
      break;
    case 2:
      try_send(sabacan_robomas_reset_client_id2_, "/sabacan_robomas_reset_id2");
      break;
    case 3:
      try_send(sabacan_robomas_reset_client_id3_, "/sabacan_robomas_reset_id3");
      break;
    case 4:
      try_send(sabacan_robomas_reset_client_id4_, "/sabacan_robomas_reset_id4");
      break;
    case 5:
      try_send(sabacan_robomas_reset_client_id5_, "/sabacan_robomas_reset_id5");
      break;
    case 6:
      try_send(sabacan_gpio_reset_client_id1_, "/sabacan_gpio_reset_id1");
      break;
    case 7:
      try_send(sabacan_gpio_reset_client_id2_, "/sabacan_gpio_reset_id2");
      break;
    case 8:
      try_send(sabacan_gpio_reset_client_id3_, "/sabacan_gpio_reset_id3");
      break;
    case 9:
      try_send(sabacan_led_reset_client_, "/sabacan_led_reset");
      break;
  }

  sabacan_reset_last_send_time_ = now;
  sabacan_reset_last_send_valid_ = true;
  sabacan_reset_step_++;
  // stepは最後の処理が終わるのにかかる時間も考慮し、1つ多く設定する
  if (sabacan_reset_step_ >= 11) {
    sabacan_reset_status_ = SABACAN_AVAILABLE;
    RCLCPP_INFO(this->get_logger(), "sabacan reset completed");
  }
}

void R1MainNode::sabacan_reset(void)
{
  if (sabacan_reset_status_ != SABACAN_AVAILABLE) {
    return;
  }
  sabacan_reset_status_ = SABACAN_RESET_NOW;
}

void R1MainNode::sabacan_power_ref(bool is_ems)
{
  sabacan_msgs::msg::SabacanPowerRef msg;
  msg.is_ems = is_ems;
  sabacan_is_ems_ = is_ems;
  sabacan_power_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "sabacan power ref is_ems: %d", is_ems);
}

void R1MainNode::sabacan_led_ref(uint8_t r, uint8_t g, uint8_t b)
{
  sabacan_msgs::msg::SabacanLEDRef msg;
  msg.pin_number = 0;
  msg.start = 0;
  msg.length = 255;
  msg.r = r;
  msg.g = g;
  msg.b = b;
  sabacan_led_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "sabacan led ref r: %d, g: %d, b: %d", r, g, b);
}

void R1MainNode::sabacan_led_update(void)
{
  // TODO: 暇なときに実装する
}

void R1MainNode::timer_callback(void)
{
  ps4_->update();
  sabacan_reset_update();
  sabacan_led_update();
  actuator_update();
  // 状態を更新
  state_machine_->update();
  // タスクを実行
  // ps4_->print_data();
  main_task();
}

void R1MainNode::chassis_move_vel(double vx, double vy, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = omega;
  cmd_vel_publisher_->publish(msg);
}

void R1MainNode::kfs_fx(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fx_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fx pos %f", pos);
  kfs_fx_position_ref_ = pos;
}

void R1MainNode::kfs_fz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fz_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fz pos %f", pos);
  kfs_fz_position_ref_ = pos;
}

void R1MainNode::kfs_fyaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_fyaw_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs fyaw pos %f", pos);
  kfs_fyaw_position_ref_ = pos;
}

void R1MainNode::kfs_rx(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rx_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rx pos %f", pos);
  kfs_rx_position_ref_ = pos;
}

void R1MainNode::kfs_rz(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_rz_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rz pos %f", pos);
  kfs_rz_position_ref_ = pos;
}

void R1MainNode::kfs_ryaw(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  kfs_ryaw_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs ryaw pos %f", pos);
  kfs_ryaw_position_ref_ = pos;
}

void R1MainNode::front_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  front_expand_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "front expand pos %f", pos);
  front_expand_position_ref_ = pos;
}

void R1MainNode::rear_expand(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  rear_expand_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "rear expand pos %f", pos);
  rear_expand_position_ref_ = pos;
}

void R1MainNode::r2_lift(double vel)
{
  r1_msgs::msg::MotorRef msg;
  msg.control_type = "VELOCITY";
  msg.ref = vel;
  r2_lift_motor_ref_publisher_->publish(msg);
  r2_lift_velocity_ref_ = vel;
}

void R1MainNode::pole_x(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_x_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole x pos %f", pos);
  pole_x_position_ref_ = pos;
}

void R1MainNode::pole_y(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_y_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole y pos %f", pos);
  pole_y_position_ref_ = pos;
}

void R1MainNode::pole_roger(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  pole_roger_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "pole roger pos %f", pos);
  pole_roger_position_ref_ = pos;
}

void R1MainNode::spear_roger1(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_roger1_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear roger1 pos %f", pos);
  spear_roger1_position_ref_ = pos;
}

void R1MainNode::spear_roger2(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_roger2_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear roger2 pos %f", pos);
  spear_roger2_position_ref_ = pos;
}

void R1MainNode::spear_move(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_move_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear move pos %f", pos);
  spear_move_position_ref_ = pos;
}

void R1MainNode::spear_rotate(double pos)
{
  std_msgs::msg::Float64 msg;
  msg.data = pos;
  spear_rotate_position_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear rotate pos %f", pos);
  spear_rotate_position_ref_ = pos;
}

void R1MainNode::kfs_front_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_front_pump_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs front pump pwm %f", pwm);
  kfs_front_pump_ref_ = pwm;
}

void R1MainNode::kfs_rear_pump(double pwm)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = pwm;
  kfs_rear_pump_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rear pump pwm %f", pwm);
  kfs_rear_pump_ref_ = pwm;
}

void R1MainNode::kfs_front_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_front_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs front valve %d", on);
  kfs_front_valve_ref_ = on;
}

void R1MainNode::kfs_rear_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  kfs_rear_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "kfs rear valve %d", on);
  kfs_rear_valve_ref_ = on;
}

void R1MainNode::pole_servo(int n, int angle)
{
  r1_msgs::msg::GpioServoRef msg;
  msg.ref = angle;
  if (n == 1) {
    pole_servo1_gpio_servo_ref_publisher_->publish(msg);
    pole_servo1_angle_ref_ = angle;
  } else if (n == 2) {
    pole_servo2_gpio_servo_ref_publisher_->publish(msg);
    pole_servo2_angle_ref_ = angle;
  } else if (n == 3) {
    pole_servo3_gpio_servo_ref_publisher_->publish(msg);
    pole_servo3_angle_ref_ = angle;
  } else if (n == 4) {
    pole_servo4_gpio_servo_ref_publisher_->publish(msg);
    pole_servo4_angle_ref_ = angle;
  }

  if (1 <= n && n <= 4) {
    RCLCPP_INFO(this->get_logger(), "pole servo%d angle %d", n, angle);
  } else {
    RCLCPP_ERROR(this->get_logger(), "pole servo%d angle %d: invalid servo number", n, angle);
  }
}

void R1MainNode::pole_valve(int n, bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  if (n == 1) {
    pole_valve1_gpio_pwm_ref_publisher_->publish(msg);
    pole_valve1_ref_ = on;
  } else if (n == 2) {
    pole_valve2_gpio_pwm_ref_publisher_->publish(msg);
    pole_valve2_ref_ = on;
  } else if (n == 3) {
    pole_valve3_gpio_pwm_ref_publisher_->publish(msg);
    pole_valve3_ref_ = on;
  } else if (n == 4) {
    pole_valve4_gpio_pwm_ref_publisher_->publish(msg);
    pole_valve4_ref_ = on;
  }

  if (1 <= n && n <= 4) {
    RCLCPP_INFO(this->get_logger(), "pole valve%d = %d", n, on);
  } else {
    RCLCPP_ERROR(this->get_logger(), "pole valve%d = %d: invalid valve number", n, on);
  }
}

void R1MainNode::spear_hand_valve(bool on)
{
  r1_msgs::msg::GpioPwmRef msg;
  msg.ref = on ? 1.0 : 0.0;
  spear_hand_valve_gpio_pwm_ref_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "spear hand valve %d", on);
  spear_hand_valve_ref_ = on;
}

void R1MainNode::stop_actuator(void)
{
  // 速度制御のモータ指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
  r2_lift(0.0);
  // 真空ポンプを止める
  kfs_front_pump(0.0);
  kfs_rear_pump(0.0);
  // KFS回収電磁弁を止める
  kfs_front_valve(false);
  kfs_rear_valve(false);
  // ポール回収用電磁弁を止める
  pole_valve(1, false);
  pole_valve(2, false);
  pole_valve(3, false);
  pole_valve(4, false);
  // やり電磁弁
  spear_hand_valve(false);
}

void R1MainNode::init_actuator(void) { actuator_status_ = ACTUATOR_INITIALIZING; }

void R1MainNode::actuator_update(void)
{
  // sabacanのresetが完了したら、actuatorを初期化する
  if (sabacan_reset_status_ == SABACAN_AVAILABLE && actuator_status_ == ACTUATOR_INITIALIZING) {
    // 位置制御系のアクチュエータを初期位置に移動
    // TODO: 将来的にはこの関数で原点検出も行う
    kfs_fx(KFS_FX_NORMAL_POS);
    kfs_fz(KFS_FZ_NORMAL_POS);
    kfs_fyaw(KFS_FYAW_NORMAL_ANGLE);
    kfs_rx(KFS_RX_NORMAL_POS);
    kfs_rz(KFS_RZ_NORMAL_POS);
    kfs_ryaw(KFS_RYAW_NORMAL_ANGLE);
    front_expand(FRONT_EXPAND_NORMAL_POS);
    rear_expand(REAR_EXPAND_NORMAL_POS);
    pole_x(POLE_X_NORMAL_POS);
    pole_y(POLE_Y_NORMAL_POS);
    pole_roger(POLE_ROGER_NORMAL_POS);
    spear_roger1(SPEAR_ROGER1_NORMAL_POS);
    spear_roger2(SPEAR_ROGER2_NORMAL_POS);
    spear_move(SPEAR_MOVE_NORMAL_POS);
    spear_rotate(SPEAR_ROTATE_NORMAL_POS);
    pole_servo(1, POLE_SERVO1_NORMAL_ANGLE);
    pole_servo(2, POLE_SERVO2_NORMAL_ANGLE);
    pole_servo(3, POLE_SERVO3_NORMAL_ANGLE);
    pole_servo(4, POLE_SERVO4_NORMAL_ANGLE);
    // 位置制御系以外のアクチュエータは停止状態にする
    stop_actuator();
    actuator_status_ = ACTUATOR_AVAILABLE;
    RCLCPP_INFO(this->get_logger(), "robot initialized");
  }
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

  if (ps4_->is_pushed_left()) {
    front_expand_detect_origin();
  }

  if (ps4_->is_pushed_triangle()) {
    kfs_rx_detect_origin();
  }

  if (ps4_->is_pushed_circle()) {
    kfs_rz_detect_origin();
  }

  if (ps4_->is_pushed_cross()) {
    kfs_ryaw_detect_origin();
  }

  if (ps4_->is_pushed_square()) {
    rear_expand_detect_origin();
  }

  if (ps4_->is_pushed_l1()) {
    spear_roger1_detect_origin();
  }

  if (ps4_->is_pushed_r1()) {
    spear_roger2_detect_origin();
  }

  if (ps4_->is_pushed_l2()) {
    spear_move_detect_origin();
  }

  if (ps4_->is_pushed_r2()) {
    spear_rotate_detect_origin();
  }
}

void R1MainNode::manual_mode2_make_spear_task(int n)
{
  static int step = 1;
  RCLCPP_INFO(this->get_logger(), "manual_mode2_make_spear_task step: %d", step);
  if (step == 1) {
    // pole_yを受け渡し位置に移動
    if (n == 1) {
      pole_y(POLE_Y_TRANSFER1_POS);
    } else if (n == 2) {
      pole_y(POLE_Y_TRANSFER2_POS);
    } else if (n == 3) {
      pole_y(POLE_Y_TRANSFER3_POS);
    } else if (n == 4) {
      pole_y(POLE_Y_TRANSFER4_POS);
    }
    spear_roger1(SPEAR_ROGER1_TRANSFER_POS);
    spear_roger2(SPEAR_ROGER2_TRANSFER_POS);
    spear_move(SPEAR_MOVE_TRANSFER_POS);
    pole_roger(POLE_ROGER_EXPAND_POS);
    step++;
  } else if (step == 2) {
    // サーボモータを回転させ、ポールを水平にする
    if (n == 1) {
      pole_servo(1, POLE_SERVO1_HORIZONTAL_ANGLE);
    } else if (n == 2) {
      pole_servo(2, POLE_SERVO2_HORIZONTAL_ANGLE);
    } else if (n == 3) {
      pole_servo(3, POLE_SERVO3_HORIZONTAL_ANGLE);
    } else if (n == 4) {
      pole_servo(4, POLE_SERVO4_HORIZONTAL_ANGLE);
    }
    // やりハンドの電磁弁をONにし、ハンドを開放する。
    spear_hand_valve(true);
    step++;
  } else if (step == 3) {
    // やりハンドの電磁弁をOFFにし、ハンドを閉じる。
    spear_hand_valve(false);
    // ポールハンドの電磁弁をONにし、やり機構にポールを受け渡す。
    if (n == 1) {
      pole_valve(1, true);
    } else if (n == 2) {
      pole_valve(2, true);
    } else if (n == 3) {
      pole_valve(3, true);
    } else if (n == 4) {
      pole_valve(4, true);
    }
    step++;
  } else if (step == 4) {
    // R2とのやり合体位置に、やりハンドを移動する。
    spear_move(SPEAR_MOVE_COMBINE_POS);
    step++;
  } else if (step == 5) {
    // ポールハンドの電磁弁をOFFにする。
    if (n == 1) {
      pole_valve(1, false);
    } else if (n == 2) {
      pole_valve(2, false);
    } else if (n == 3) {
      pole_valve(3, false);
    } else if (n == 4) {
      pole_valve(4, false);
    }
    step++;
  } else if (step == 6) {
    // 受け渡し位置にやりハンドを移動する。
    // ポールハンドの電磁弁をONにし、ハンドを開放する。
    spear_move(SPEAR_MOVE_TRANSFER_POS);
    if (n == 1) {
      pole_valve(1, true);
    } else if (n == 2) {
      pole_valve(2, true);
    } else if (n == 3) {
      pole_valve(3, true);
    } else if (n == 4) {
      pole_valve(4, true);
    }
    step++;
  } else if (step == 7) {
    // ポールハンドの電磁弁をOFFにし、ハンドを閉じる。
    // やりハンドの電磁弁をONにし、やりハンドを開放する。
    if (n == 1) {
      pole_valve(1, false);
    } else if (n == 2) {
      pole_valve(2, false);
    } else if (n == 3) {
      pole_valve(3, false);
    } else if (n == 4) {
      pole_valve(4, false);
    }
    spear_hand_valve(true);
    step++;
  } else if (step == 8) {
    // やりハンドをぶつからない位置に移動する。
    spear_move(SPEAR_MOVE_NORMAL_POS);
    step++;
  } else if (step == 9) {
    // やりハンドの電磁弁をOFFにする。
    spear_hand_valve(false);
    step++;
  } else if (step == 10) {
    // サーボモータを回転させ、ポールを垂直にする。
    if (n == 1) {
      pole_servo(1, POLE_SERVO1_NORMAL_ANGLE);
    } else if (n == 2) {
      pole_servo(2, POLE_SERVO2_NORMAL_ANGLE);
    } else if (n == 3) {
      pole_servo(3, POLE_SERVO3_NORMAL_ANGLE);
    } else if (n == 4) {
      pole_servo(4, POLE_SERVO4_NORMAL_ANGLE);
    }
    step++;
  } else if (step == 11) {
    pole_roger(POLE_ROGER_NORMAL_POS);
    pole_y(POLE_Y_NORMAL_POS);
    RCLCPP_INFO(this->get_logger(), "spear make task completed");
    step = 1;
  }
}

void R1MainNode::manual_mode2_collect_pole_task(void)
{
  static int step = 1;
  RCLCPP_INFO(this->get_logger(), "manual_mode2_collect_pole_task step: %d", step);
  if (step == 1) {
    // pole_x(POLE_X_EXPAND_POS);
    pole_y(POLE_Y_COLLECT_POS);
    step++;
  } else if (step == 2) {
    pole_roger(POLE_ROGER_EXPAND_POS);
    step++;
  } else if (step == 3) {
    pole_valve(1, true);
    pole_valve(2, true);
    pole_valve(3, true);
    pole_valve(4, true);
    step++;
  } else if (step == 4) {
    pole_valve(1, false);
    pole_valve(2, false);
    pole_valve(3, false);
    pole_valve(4, false);
    step++;
  } else if (step == 5) {
    // pole_x(POLE_X_NORMAL_POS);
    step++;
  } else if (step == 6) {
    pole_roger(POLE_ROGER_NORMAL_POS);
    RCLCPP_INFO(this->get_logger(), "pole collect task completed");
    step = 1;
  }
}

void R1MainNode::manual_mode2_spear_and_pole(void)
{
  if (ps4_->is_pushed_up()) {
    // やり組み立て
    manual_mode2_make_spear_task(2);
  }

  if (ps4_->is_pushed_right()) {
    pole_roger(pole_roger_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_down()) {
    pole_x(pole_x_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_left()) {
    pole_roger(pole_roger_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_triangle()) {
    // ポール回収
    manual_mode2_collect_pole_task();
  }

  if (ps4_->is_pushed_circle()) {
    pole_y(pole_y_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_cross()) {
    pole_x(pole_x_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_square()) {
    pole_y(pole_y_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_l1()) {
    spear_roger1(spear_roger1_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r1()) {
    spear_roger1(spear_roger1_position_ref_ + 0.01);
  }

  if (ps4_->is_pushed_l2()) {
    spear_roger2(spear_roger2_position_ref_ - 0.01);
  }

  if (ps4_->is_pushed_r2()) {
    spear_roger2(spear_roger2_position_ref_ + 0.01);
  }
}

void R1MainNode::manual_mode3_kfs(void)
{
  static int fx_step = 1;
  static int fz_step = 1;
  static int fyaw_step = 1;
  static int rx_step = 1;
  static int rz_step = 1;
  static int ryaw_step = 1;
  static int front_pump_step = 1;
  static int rear_pump_step = 1;

  if (ps4_->is_pushed_up()) {
    // 1段上のkfs_fz位置へ移動
    fz_step++;
    if (fz_step > 3) {
      fz_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
    if (fz_step == 1) {
      kfs_fz(KFS_FZ_LOW_POS);
    } else if (fz_step == 2) {
      kfs_fz(KFS_FZ_MIDDLE_POS);
    } else if (fz_step == 3) {
      kfs_fz(KFS_FZ_HIGH_POS);
    }
  }

  if (ps4_->is_pushed_right()) {
    // kfs_fxを動かす
    if (fx_step == 1) {
      kfs_fx(KFS_FX_EXPAND_POS);
      fx_step = 2;
    } else {
      kfs_fx(KFS_FX_NORMAL_POS);
      fx_step = 1;
    }
  }

  if (ps4_->is_pushed_down()) {
    // 1段下のkfs_fz位置へ移動
    fz_step--;
    if (fz_step < 1) {
      fz_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
    if (fz_step == 1) {
      kfs_fz(KFS_FZ_LOW_POS);
    } else if (fz_step == 2) {
      kfs_fz(KFS_FZ_MIDDLE_POS);
    } else if (fz_step == 3) {
      kfs_fz(KFS_FZ_HIGH_POS);
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
      manual_mode3_front_valve_timer_ = this->create_wall_timer(250ms, [this]() {
        kfs_front_valve(false);
        manual_mode3_front_valve_timer_->cancel();
      });
      front_pump_step = 1;
    }
  }

  if (ps4_->is_pushed_triangle()) {
    // 1段上のkfs_rz位置へ移動
    rz_step++;
    if (rz_step > 3) {
      rz_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
    if (rz_step == 1) {
      kfs_rz(KFS_RZ_LOW_POS);
    } else if (rz_step == 2) {
      kfs_rz(KFS_RZ_MIDDLE_POS);
    } else if (rz_step == 3) {
      kfs_rz(KFS_RZ_HIGH_POS);
    }
  }

  if (ps4_->is_pushed_circle()) {
    // kfs_rxを動かす
    if (rx_step == 1) {
      kfs_rx(KFS_RX_EXPAND_POS);
      rx_step = 2;
    } else {
      kfs_rx(KFS_RX_NORMAL_POS);
      rx_step = 1;
    }
  }

  if (ps4_->is_pushed_cross()) {
    // 1段下のkfs_rz位置へ移動
    rz_step--;
    if (rz_step < 1) {
      rz_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
    if (rz_step == 1) {
      kfs_rz(KFS_RZ_LOW_POS);
    } else if (rz_step == 2) {
      kfs_rz(KFS_RZ_MIDDLE_POS);
    } else if (rz_step == 3) {
      kfs_rz(KFS_RZ_HIGH_POS);
    }
  }

  if (ps4_->is_pushed_square()) {
    // rear_pumpを動かす。止めるときは電磁弁も一緒に動く
    if (rear_pump_step == 1) {
      kfs_rear_pump(1.0);
      kfs_rear_valve(false);
      rear_pump_step = 2;
    } else {
      kfs_rear_pump(0.0);
      kfs_rear_valve(true);
      // setTimeout風で電磁弁をOFFにする。
      manual_mode3_rear_valve_timer_ = this->create_wall_timer(250ms, [this]() {
        kfs_rear_valve(false);
        manual_mode3_rear_valve_timer_->cancel();
      });
      rear_pump_step = 1;
    }
  }

  if (ps4_->is_pushed_l1()) {
    // kfs_fyawを90度戻す
    fyaw_step--;
    if (fyaw_step < 1) {
      fyaw_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
    if (fyaw_step == 1) {
      kfs_fyaw(KFS_FYAW_FRONT_ANGLE);
    } else if (fyaw_step == 2) {
      kfs_fyaw(KFS_FYAW_SIDE_ANGLE);
    } else if (fyaw_step == 3) {
      kfs_fyaw(KFS_FYAW_REAR_ANGLE);
    }
  }

  if (ps4_->is_pushed_r1()) {
    // kfs_fyawを90度進める
    fyaw_step++;
    if (fyaw_step > 3) {
      fyaw_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
    if (fyaw_step == 1) {
      kfs_fyaw(KFS_FYAW_FRONT_ANGLE);
    } else if (fyaw_step == 2) {
      kfs_fyaw(KFS_FYAW_SIDE_ANGLE);
    } else if (fyaw_step == 3) {
      kfs_fyaw(KFS_FYAW_REAR_ANGLE);
    }
  }

  if (ps4_->is_pushed_l2()) {
    // kfs_ryawを90度戻す
    ryaw_step--;
    if (ryaw_step < 1) {
      ryaw_step = 1;
    }
    RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
    if (ryaw_step == 1) {
      kfs_ryaw(KFS_RYAW_FRONT_ANGLE);
    } else if (ryaw_step == 2) {
      kfs_ryaw(KFS_RYAW_SIDE_ANGLE);
    } else if (ryaw_step == 3) {
      kfs_ryaw(KFS_RYAW_REAR_ANGLE);
    }
  }

  if (ps4_->is_pushed_r2()) {
    // kfs_ryawを90度進める
    ryaw_step++;
    if (ryaw_step > 3) {
      ryaw_step = 3;
    }
    RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
    if (ryaw_step == 1) {
      kfs_ryaw(KFS_RYAW_FRONT_ANGLE);
    } else if (ryaw_step == 2) {
      kfs_ryaw(KFS_RYAW_SIDE_ANGLE);
    } else if (ryaw_step == 3) {
      kfs_ryaw(KFS_RYAW_REAR_ANGLE);
    }
  }
}

void R1MainNode::manual_mode4_r2_lift(void)
{
  static int front_expand_step = 1;
  static int rear_expand_step = 1;
  static int r2_lift_step = 0;

  if (ps4_->data.triangle) {
    if (r2_lift_step != 1) {
      r2_lift(R2_LIFT_MAX_VELOCITY);
      RCLCPP_INFO(this->get_logger(), "r2 lift up");
      r2_lift_step = 1;
    }
  } else if (ps4_->data.cross) {
    if (r2_lift_step != -1) {
      r2_lift(-R2_LIFT_MAX_VELOCITY);
      RCLCPP_INFO(this->get_logger(), "r2 lift down");
      r2_lift_step = -1;
    }
  } else {
    if (r2_lift_step != 0) {
      r2_lift(0.0);
      RCLCPP_INFO(this->get_logger(), "r2 lift stop");
      r2_lift_step = 0;
    }
  }

  if (ps4_->is_pushed_up()) {
  }

  if (ps4_->is_pushed_right()) {
  }

  if (ps4_->is_pushed_down()) {
  }

  if (ps4_->is_pushed_left()) {
  }

  if (ps4_->is_pushed_triangle()) {
  }

  if (ps4_->is_pushed_circle()) {
    if (rear_expand_step == 1) {
      rear_expand(REAR_EXPAND_EXPAND_POS);
      rear_expand_step = 2;
    } else {
      rear_expand(REAR_EXPAND_NORMAL_POS);
      rear_expand_step = 1;
    }
  }

  if (ps4_->is_pushed_cross()) {
  }

  if (ps4_->is_pushed_square()) {
    if (front_expand_step == 1) {
      front_expand(FRONT_EXPAND_EXPAND_POS);
      front_expand_step = 2;
    } else {
      front_expand(FRONT_EXPAND_NORMAL_POS);
      front_expand_step = 1;
    }
  }

  if (ps4_->is_pushed_l1()) {
  }

  if (ps4_->is_pushed_r1()) {
  }

  if (ps4_->is_pushed_l2()) {
  }

  if (ps4_->is_pushed_r2()) {
  }
}

void R1MainNode::manual_task(void)
{
  static bool stop_actuator_flag = false;
  auto current_state = state_machine_->get_current_state();
  if (ps4_->is_connected() == false) {
    // 未接続のときはアクチュエータ停止
    if (stop_actuator_flag == false) {
      stop_actuator();
      stop_actuator_flag = true;
    }

  } else {
    stop_actuator_flag = false;
    // 状態に応じて、各タスクを実行
    if (const auto * manual_sub = std::get_if<ManualSubState>(&current_state.sub)) {
      if (*manual_sub == ManualSubState::MODE1_DETECT_ORIGIN) {
        manual_mode1_detect_origin();
      } else if (*manual_sub == ManualSubState::MODE2_SPEAR_AND_POLE) {
        manual_mode2_spear_and_pole();
      } else if (*manual_sub == ManualSubState::MODE3_KFS) {
        manual_mode3_kfs();
      } else if (*manual_sub == ManualSubState::MODE4_R2_LIFT) {
        manual_mode4_r2_lift();
      } else if (*manual_sub == ManualSubState::TEST) {
        // test_front_kfs();
        test_spear();
      }
    }
    // 共通タスク
    double stick_max_velocity = 1.5;
    double vx_ref = stick_max_velocity * ps4_->data.left_stick_x;
    double vy_ref = stick_max_velocity * ps4_->data.left_stick_y;
    double vz_ref = ps4_->data.right_stick_x;
    // 台形制御で速度を滑らかに変化させる
    double vx = simple_trapezoid_vx_.update(vx_ref);
    double vy = simple_trapezoid_vy_.update(vy_ref);
    double vz = simple_trapezoid_omega_.update(vz_ref);
    chassis_move_vel(vx, vy, vz);

    // optionsが押されたときは電源をOFFにする
    if (ps4_->is_pushed_options()) {
      sabacan_power_ref(!sabacan_is_ems_);
    }
    // psボタンが押されたときはsabacan resetを行う
    if (ps4_->is_pushed_ps()) {
      sabacan_reset();
      init_actuator();
    }
    // shareボタンが押されたときはモードを切り替える
    if (ps4_->is_pushed_share()) {
      if (const auto * manual_sub = std::get_if<ManualSubState>(&current_state.sub)) {
        if (*manual_sub == ManualSubState::MODE1_DETECT_ORIGIN) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE2_SPEAR_AND_POLE});
        } else if (*manual_sub == ManualSubState::MODE2_SPEAR_AND_POLE) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE3_KFS});
        } else if (*manual_sub == ManualSubState::MODE3_KFS) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE4_R2_LIFT});
        } else if (*manual_sub == ManualSubState::MODE4_R2_LIFT) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE1_DETECT_ORIGIN});
        }
      }
    }
  }
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
      pole_servo(1, 80);
    } else {
      pole_servo(1, 180);
    }
    pole_servo1_pushed = !pole_servo1_pushed;
  }
  if (ps4_->is_pushed_circle()) {
    if (pole_servo2_pushed) {
      pole_servo(2, 70);
    } else {
      pole_servo(2, 170);
    }
    pole_servo2_pushed = !pole_servo2_pushed;
  }
  if (ps4_->is_pushed_cross()) {
    if (pole_servo3_pushed) {
      pole_servo(3, 70);
    } else {
      pole_servo(3, 180);
    }
    pole_servo3_pushed = !pole_servo3_pushed;
  }

  if (ps4_->is_pushed_square()) {
    if (pole_servo4_pushed) {
      pole_servo(4, 70);
    } else {
      pole_servo(4, 185);
    }
    pole_servo4_pushed = !pole_servo4_pushed;
  }

  if (ps4_->is_pushed_up()) {
    if (pole_valve1_pushed) {
      pole_valve(1, false);
    } else {
      pole_valve(1, true);
    }
    pole_valve1_pushed = !pole_valve1_pushed;
  }

  if (ps4_->is_pushed_right()) {
    if (pole_valve2_pushed) {
      pole_valve(2, false);
    } else {
      pole_valve(2, true);
    }
    pole_valve2_pushed = !pole_valve2_pushed;
  }

  if (ps4_->is_pushed_down()) {
    if (pole_valve3_pushed) {
      pole_valve(3, false);
    } else {
      pole_valve(3, true);
    }
    pole_valve3_pushed = !pole_valve3_pushed;
  }

  if (ps4_->is_pushed_left()) {
    if (pole_valve4_pushed) {
      pole_valve(4, false);
    } else {
      pole_valve(4, true);
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
