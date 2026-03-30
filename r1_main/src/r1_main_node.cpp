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
void R1MainNode::register_position_axis(const std::string & name, double * position_ref_alias)
{
  auto [it, inserted] = position_axes_.try_emplace(name);
  (void)inserted;
  auto & axis = it->second;
  axis.position_ref_alias = position_ref_alias;
  if (axis.position_ref_alias != nullptr) {
    *axis.position_ref_alias = axis.position_ref;
  }

  axis.position_ref_publisher =
    this->create_publisher<std_msgs::msg::Float64>("/" + name + "_position_ref", 10);
  axis.detect_origin_publisher =
    this->create_publisher<std_msgs::msg::Bool>("/" + name + "_detect_origin", 10);
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
}

R1MainNode::R1MainNode() : Node("r1_main_node")
{
  // 足回りの速度指令
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
  register_position_axis("spear1", &spear1_position_ref_);
  register_position_axis("spear2", &spear2_position_ref_);
  register_position_axis("spear3", &spear3_position_ref_);
  register_position_axis("spear4", &spear4_position_ref_);
  register_position_axis("spear_x", &spear_x_position_ref_);
  register_position_axis("spear_y", &spear_y_position_ref_);
  register_position_axis("spear_roll", &spear_roll_position_ref_);
  register_position_axis("spear_pitch1", &spear_pitch1_position_ref_);
  register_position_axis("spear_pitch2", &spear_pitch2_position_ref_);

  // ========== R2昇降指令値 ==========
  register_velocity_axis("r2_flift", "/r2_flift_motor_ref", &r2_flift_velocity_ref_);
  register_velocity_axis("r2_rlift", "/r2_rlift_motor_ref", &r2_rlift_velocity_ref_);
  // ========== GPIO ==========
  register_gpio_pwm_output("kfs_front_pump", &kfs_front_pump_ref_, nullptr);
  register_gpio_pwm_output("kfs_rear_pump", &kfs_rear_pump_ref_, nullptr);
  register_gpio_pwm_output("kfs_front_valve", nullptr, &kfs_front_valve_ref_);
  register_gpio_pwm_output("kfs_rear_valve", nullptr, &kfs_rear_valve_ref_);
  register_gpio_input("kfs_fz_low_switch", &kfs_fz_switch_status_);
  register_gpio_input("kfs_rz_low_switch", &kfs_rz_switch_status_);

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
  // ロボマス制御基板のリセットサービスClient
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
  sabacan_robomas_reset_client_id6_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_robomas_reset_id6");
  // GPIO基板のリセットサービスClient
  sabacan_gpio_reset_client_id1_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id1");
  sabacan_gpio_reset_client_id2_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id2");
  sabacan_gpio_reset_client_id3_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_gpio_reset_id3");
  sabacan_led_reset_client_ =
    this->create_client<sabacan_msgs::srv::SabacanReset>("/sabacan_led_reset");

  // IMU
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/bno086/imu/data_raw", 10, std::bind(&R1MainNode::imu_callback, this, std::placeholders::_1));

  // set_mecanum_yawのPublisher
  set_mecanum_yaw_publisher_ =
    this->create_publisher<std_msgs::msg::Float64>("/set_mecanum_yaw", 10);

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
  declare_and_get_parameter("chassis_max_velocity", CHASSIS_MAX_VELOCITY);
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
  declare_and_get_parameter("kfs_fz_book_pos", KFS_FZ_BOOK_POS);
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
  declare_and_get_parameter("kfs_rz_book_pos", KFS_RZ_BOOK_POS);
  // ryaw
  declare_and_get_parameter("kfs_ryaw_normal_angle", KFS_RYAW_NORMAL_ANGLE);
  declare_and_get_parameter("kfs_ryaw_front_angle", KFS_RYAW_FRONT_ANGLE);
  declare_and_get_parameter("kfs_ryaw_side_angle", KFS_RYAW_SIDE_ANGLE);
  declare_and_get_parameter("kfs_ryaw_rear_angle", KFS_RYAW_REAR_ANGLE);

  // ========== 展開 ==========
  // R2昇降
  declare_and_get_parameter("r2_lift_max_velocity", R2_LIFT_MAX_VELOCITY);

  // ========== やり ==========
  // spear1

  // spear2

  // spear3

  // spear4

  // spear_x

  // spear_y

  // spear_roll

  // spear_pitch1

  // spear_pitch2

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

  state_machine_ = std::make_shared<StateMachine>();
  // state_machine_->set_next_state({MainState::MANUAL, ManualSubState::TEST});
  // state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE2_POLE});
  state_machine_->set_next_state({MainState::AUTO, AutoSubState::ACT0});
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

void R1MainNode::spear1_detect_origin(void) { detect_origin_position_axis("spear1"); }

void R1MainNode::spear2_detect_origin(void) { detect_origin_position_axis("spear2"); }

void R1MainNode::spear3_detect_origin(void) { detect_origin_position_axis("spear3"); }

void R1MainNode::spear4_detect_origin(void) { detect_origin_position_axis("spear4"); }

void R1MainNode::spear_x_detect_origin(void) { detect_origin_position_axis("spear_x"); }

void R1MainNode::spear_y_detect_origin(void) { detect_origin_position_axis("spear_y"); }

void R1MainNode::spear_roll_detect_origin(void) { detect_origin_position_axis("spear_roll"); }

void R1MainNode::spear_pitch1_detect_origin(void) { detect_origin_position_axis("spear_pitch1"); }

void R1MainNode::spear_pitch2_detect_origin(void) { detect_origin_position_axis("spear_pitch2"); }

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
      try_send(sabacan_robomas_reset_client_id6_, "/sabacan_robomas_reset_id6");
      break;
    case 7:
      try_send(sabacan_gpio_reset_client_id1_, "/sabacan_gpio_reset_id1");
      break;
    case 8:
      try_send(sabacan_gpio_reset_client_id2_, "/sabacan_gpio_reset_id2");
      break;
    case 9:
      try_send(sabacan_gpio_reset_client_id3_, "/sabacan_gpio_reset_id3");
      break;
    case 10:
      try_send(sabacan_led_reset_client_, "/sabacan_led_reset");
      break;
  }

  sabacan_reset_last_send_time_ = now;
  sabacan_reset_last_send_valid_ = true;
  sabacan_reset_step_++;
  // stepは最後の処理が終わるのにかかる時間も考慮し、1つ多く設定する
  if (sabacan_reset_step_ >= 12) {
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
  // RCLCPP_INFO(this->get_logger(), "sabacan led ref r: %d, g: %d, b: %d", r, g, b);
}

void R1MainNode::sabacan_led_update(void)
{
  // TODO: 暇なときに実装する
}

void R1MainNode::set_mecanum_yaw(double yaw)
{
  std_msgs::msg::Float64 msg;
  msg.data = yaw;
  set_mecanum_yaw_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "set mecanum yaw: %f", yaw);
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

void R1MainNode::kfs_fx(double pos) { publish_position_axis("kfs_fx", pos); }

void R1MainNode::kfs_fz(double pos) { publish_position_axis("kfs_fz", pos); }

void R1MainNode::kfs_fyaw(double pos) { publish_position_axis("kfs_fyaw", pos); }

void R1MainNode::kfs_rx(double pos) { publish_position_axis("kfs_rx", pos); }

void R1MainNode::kfs_rz(double pos) { publish_position_axis("kfs_rz", pos); }

void R1MainNode::kfs_ryaw(double pos) { publish_position_axis("kfs_ryaw", pos); }

void R1MainNode::r2_flift(double vel) { publish_velocity_axis("r2_flift", vel); }

void R1MainNode::r2_rlift(double vel) { publish_velocity_axis("r2_rlift", vel); }

void R1MainNode::spear1(double pos) { publish_position_axis("spear1", pos); }

void R1MainNode::spear2(double pos) { publish_position_axis("spear2", pos); }

void R1MainNode::spear3(double pos) { publish_position_axis("spear3", pos); }

void R1MainNode::spear4(double pos) { publish_position_axis("spear4", pos); }

void R1MainNode::spear_x(double pos) { publish_position_axis("spear_x", pos); }

void R1MainNode::spear_y(double pos) { publish_position_axis("spear_y", pos); }

void R1MainNode::spear_roll(double angle) { publish_position_axis("spear_roll", angle); }

void R1MainNode::spear_pitch1(double angle) { publish_position_axis("spear_pitch1", angle); }

void R1MainNode::spear_pitch2(double angle) { publish_position_axis("spear_pitch2", angle); }

void R1MainNode::kfs_front_pump(double pwm)
{
  publish_gpio_pwm_output("kfs_front_pump", pwm);
  RCLCPP_INFO(this->get_logger(), "kfs front pump pwm %f", pwm);
}

void R1MainNode::kfs_rear_pump(double pwm)
{
  publish_gpio_pwm_output("kfs_rear_pump", pwm);
  RCLCPP_INFO(this->get_logger(), "kfs rear pump pwm %f", pwm);
}

void R1MainNode::kfs_front_valve(bool on)
{
  publish_gpio_pwm_output("kfs_front_valve", on ? 1.0 : 0.0);
  RCLCPP_INFO(this->get_logger(), "kfs front valve %d", on);
}

void R1MainNode::kfs_rear_valve(bool on)
{
  publish_gpio_pwm_output("kfs_rear_valve", on ? 1.0 : 0.0);
  RCLCPP_INFO(this->get_logger(), "kfs rear valve %d", on);
}

void R1MainNode::stop_actuator(void)
{
  // 速度制御のモータ指令値を0にする
  chassis_move_vel(0.0, 0.0, 0.0);
  r2_flift(0.0);
  r2_rlift(0.0);
  // 真空ポンプを止める
  kfs_front_pump(0.0);
  kfs_rear_pump(0.0);
  // KFS回収電磁弁を止める
  kfs_front_valve(false);
  kfs_rear_valve(false);
}

void R1MainNode::init_actuator(void) { actuator_status_ = ACTUATOR_INITIALIZING; }

void R1MainNode::actuator_update(void)
{
  // sabacanのresetが完了したら、actuatorを初期化する
  if (sabacan_reset_status_ == SABACAN_AVAILABLE && actuator_status_ == ACTUATOR_INITIALIZING) {
    // 位置制御系のアクチュエータを初期位置に移動
    // TODO: 将来的にはこの関数で原点検出も行う
    // TODO: 現在リファクタリング中のため、一旦コメントアウト
    // kfs_fx(KFS_FX_NORMAL_POS);
    // kfs_fz(KFS_FZ_NORMAL_POS);
    // kfs_fyaw(KFS_FYAW_NORMAL_ANGLE);
    // kfs_rx(KFS_RX_NORMAL_POS);
    // kfs_rz(KFS_RZ_NORMAL_POS);
    // kfs_ryaw(KFS_RYAW_NORMAL_ANGLE);
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
  //   front_expand_detect_origin();
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   kfs_rx_detect_origin();
  // }

  // if (ps4_->is_pushed_circle()) {
  //   kfs_rz_detect_origin();
  // }

  // if (ps4_->is_pushed_cross()) {
  //   kfs_ryaw_detect_origin();
  // }

  // if (ps4_->is_pushed_square()) {
  //   rear_expand_detect_origin();
  // }

  // if (ps4_->is_pushed_l1()) {
  //   spear_roger1_detect_origin();
  // }

  // if (ps4_->is_pushed_r1()) {
  //   spear_roger2_detect_origin();
  // }

  // if (ps4_->is_pushed_l2()) {
  //   spear_move_detect_origin();
  // }

  // if (ps4_->is_pushed_r2()) {
  //   spear_rotate_detect_origin();
  // }
}

void R1MainNode::manual_mode2_collect_pole_task(void)
{
  // int & step = manual_mode2_collect_pole_task_step_;
  // RCLCPP_INFO(this->get_logger(), "manual_mode2_collect_pole_task step: %d", step);
  // if (step == 1) {
  //   pole_x1(POLE_X1_EXPAND_POS);
  //   pole_x2(POLE_X2_EXPAND_POS);
  //   pole_y(POLE_Y_COLLECT_POS);
  //   step++;
  // } else if (step == 2) {
  //   pole_roger(POLE_ROGER_EXPAND_POS);
  //   step++;
  // } else if (step == 3) {
  //   pole_valve(1, true);
  //   pole_valve(2, true);
  //   pole_valve(3, true);
  //   pole_valve(4, true);
  //   step++;
  // } else if (step == 4) {
  //   pole_valve(1, false);
  //   pole_valve(2, false);
  //   pole_valve(3, false);
  //   pole_valve(4, false);
  //   step++;
  // } else if (step == 5) {
  //   pole_x1(POLE_X1_NORMAL_POS);
  //   pole_x2(POLE_X2_NORMAL_POS);
  //   step++;
  // } else if (step == 6) {
  //   pole_roger(POLE_ROGER_NORMAL_POS);
  //   RCLCPP_INFO(this->get_logger(), "pole collect task completed");
  //   step = 1;
  // }
}

void R1MainNode::manual_mode3_make_spear_task(int n)
{
  (void)n;
  // int & step = manual_mode3_make_spear_task_step_;
  // RCLCPP_INFO(this->get_logger(), "manual_mode2_make_spear_task step: %d", step);
  // if (step == 1) {
  //   // pole_yを受け渡し位置に移動
  //   if (n == 1) {
  //     pole_y(POLE_Y_TRANSFER1_POS);
  //   } else if (n == 2) {
  //     pole_y(POLE_Y_TRANSFER2_POS);
  //   } else if (n == 3) {
  //     pole_y(POLE_Y_TRANSFER3_POS);
  //   } else if (n == 4) {
  //     pole_y(POLE_Y_TRANSFER4_POS);
  //   }
  //   spear_roger1(SPEAR_ROGER1_TRANSFER_POS);
  //   spear_roger2(SPEAR_ROGER2_TRANSFER_POS);
  //   spear_move(SPEAR_MOVE_TRANSFER_POS);
  //   spear_hand_valve2(true);
  //   pole_roger(POLE_ROGER_EXPAND_POS);
  //   step++;
  // } else if (step == 2) {
  //   // サーボモータを回転させ、ポールを水平にする
  //   if (n == 1) {
  //     pole_servo(1, POLE_SERVO1_HORIZONTAL_ANGLE);
  //   } else if (n == 2) {
  //     pole_servo(2, POLE_SERVO2_HORIZONTAL_ANGLE);
  //   } else if (n == 3) {
  //     pole_servo(3, POLE_SERVO3_HORIZONTAL_ANGLE);
  //   } else if (n == 4) {
  //     pole_servo(4, POLE_SERVO4_HORIZONTAL_ANGLE);
  //   }
  //   // やりハンドの電磁弁をONにし、ハンドを開放する。
  //   spear_hand_valve1(true);
  //   step++;
  // } else if (step == 3) {
  //   // やりハンドの電磁弁をOFFにし、ハンドを閉じる。
  //   spear_hand_valve1(false);
  //   // ポールハンドの電磁弁をONにし、やり機構にポールを受け渡す。
  //   if (n == 1) {
  //     pole_valve(1, true);
  //   } else if (n == 2) {
  //     pole_valve(2, true);
  //   } else if (n == 3) {
  //     pole_valve(3, true);
  //   } else if (n == 4) {
  //     pole_valve(4, true);
  //   }
  //   step++;
  // } else if (step == 4) {
  //   // R2とのやり合体位置に、やりハンドを移動する。
  //   spear_move(SPEAR_MOVE_COMBINE_POS);
  //   step++;
  // } else if (step == 5) {
  //   // サーボモータを回転させ、ポールを垂直にする。
  //   if (n == 1) {
  //     pole_servo(1, POLE_SERVO1_NORMAL_ANGLE);
  //   } else if (n == 2) {
  //     pole_servo(2, POLE_SERVO2_NORMAL_ANGLE);
  //   } else if (n == 3) {
  //     pole_servo(3, POLE_SERVO3_NORMAL_ANGLE);
  //   } else if (n == 4) {
  //     pole_servo(4, POLE_SERVO4_NORMAL_ANGLE);
  //   }
  //   step++;
  // } else if (step == 6) {
  //   // ポールハンドの電磁弁をOFFにする。
  //   if (n == 1) {
  //     pole_valve(1, false);
  //   } else if (n == 2) {
  //     pole_valve(2, false);
  //   } else if (n == 3) {
  //     pole_valve(3, false);
  //   } else if (n == 4) {
  //     pole_valve(4, false);
  //   }
  //   step++;
  // } else if (step == 7) {
  //   spear_rotate(SPEAR_ROTATE_COMBINE_ANGLE);  // 180度回転
  //   step++;
  // } else if (step == 8) {
  //   pole_roger(POLE_ROGER_NORMAL_POS);
  //   pole_y(POLE_Y_NORMAL_POS);
  //   step++;
  // } else if (step == 9) {
  //   // spear_moveを受け渡し位置に移動
  //   spear_move(SPEAR_MOVE_TRANSFER_POS);
  //   // spear_rogerを初期位置に移動
  //   spear_roger1(SPEAR_ROGER1_NORMAL_POS);
  //   spear_roger2(SPEAR_ROGER2_NORMAL_POS);
  //   step++;
  // } else if (step == 10) {
  //   // やりハンド1の電磁弁をON。やりハンド2の電磁弁をOFF。
  //   spear_hand_valve1(true);
  //   spear_hand_valve2(false);
  //   step++;
  // } else if (step == 11) {
  //   // spear_moveを後ろに下げる。
  //   spear_move(SPEAR_MOVE_VALVE1_POS);
  //   step++;
  // } else if (step == 12) {
  //   // やりハンド1の電磁弁をOFF。
  //   spear_hand_valve1(false);
  //   step++;
  // } else if (step == 13) {
  //   // やりハンド2の電磁弁をON。
  //   spear_hand_valve2(true);
  //   step++;
  // } else if (step == 14) {
  //   // spear_moveを少し前に出す。
  //   spear_move(SPEAR_MOVE_VALVE2_POS);
  //   step++;
  // } else if (step == 15) {
  //   // やりハンド2の電磁弁をOFF。
  //   spear_hand_valve2(false);
  //   RCLCPP_INFO(this->get_logger(), "spear make task completed");
  //   step = 1;
  // }
}

void R1MainNode::manual_mode2_pole(void)
{
  // if (ps4_->is_pushed_up()) {
  // }

  // if (ps4_->is_pushed_right()) {
  //   pole_roger(pole_roger_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_down()) {
  //   pole_servo(1, pole_servo1_angle_ref_ - 1);
  // }

  // if (ps4_->is_pushed_left()) {
  //   pole_roger(pole_roger_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   // ポール回収
  //   manual_mode2_collect_pole_task();
  // }

  // if (ps4_->is_pushed_circle()) {
  //   pole_y(pole_y_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_cross()) {
  //   pole_servo(1, pole_servo1_angle_ref_ + 1);
  // }

  // if (ps4_->is_pushed_square()) {
  //   pole_y(pole_y_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_l1()) {
  //   pole_x1(pole_x1_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r1()) {
  //   pole_x1(pole_x1_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_l2()) {
  //   pole_x2(pole_x2_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r2()) {
  //   pole_x2(pole_x2_position_ref_ + 0.01);
  // }
}

void R1MainNode::manual_mode3_spear(void)
{
  // int & brake_valve_step = manual_mode3_brake_valve_step_;
  // int & spear_hand_valve1_step = manual_mode3_spear_hand_valve1_step_;
  // int & spear_hand_valve2_step = manual_mode3_spear_hand_valve2_step_;

  // if (ps4_->is_pushed_up()) {
  //   manual_mode3_make_spear_task(1);
  // }

  // if (ps4_->is_pushed_right()) {
  //   spear_rotate(spear_rotate_position_ref_ + 0.1);
  // }

  // if (ps4_->is_pushed_down()) {
  //   spear_move(spear_move_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_left()) {
  //   spear_rotate(spear_rotate_position_ref_ - 0.1);
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   if (brake_valve_step == 1) {
  //     brake_valve(true);
  //     brake_valve_step = 2;
  //   } else {
  //     brake_valve(false);
  //     brake_valve_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_circle()) {
  //   // if (spear_hand_valve2_step == 1) {
  //   //   spear_hand_valve2(true);
  //   //   spear_hand_valve2_step = 2;
  //   // } else {
  //   //   spear_hand_valve2(false);
  //   //   spear_hand_valve2_step = 1;
  //   // }
  //   pole_y(pole_y_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_cross()) {
  //   spear_move(spear_move_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_square()) {
  //   // if (spear_hand_valve1_step == 1) {
  //   //   spear_hand_valve1(true);
  //   //   spear_hand_valve1_step = 2;
  //   // } else {
  //   //   spear_hand_valve1(false);
  //   //   spear_hand_valve1_step = 1;
  //   // }
  //   pole_y(pole_y_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_l1()) {
  //   spear_roger1(spear_roger1_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r1()) {
  //   spear_roger1(spear_roger1_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_l2()) {
  //   spear_roger2(spear_roger2_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r2()) {
  //   spear_roger2(spear_roger2_position_ref_ + 0.01);
  // }
}

void R1MainNode::manual_mode4_fkfs(void)
{
  // int & fx_step = manual_mode4_fx_step_;
  // int & fz_step = manual_mode4_fz_step_;
  // int & fyaw_step = manual_mode4_fyaw_step_;
  // int & front_pump_step = manual_mode4_front_pump_step_;

  // if (ps4_->is_pushed_up()) {
  //   // 1段上のkfs_fz位置へ移動
  //   fz_step++;
  //   if (fz_step > 4) {
  //     fz_step = 4;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
  //   if (fz_step == 1) {
  //     kfs_fz(KFS_FZ_LOW_POS);
  //   } else if (fz_step == 2) {
  //     kfs_fz(KFS_FZ_MIDDLE_POS);
  //   } else if (fz_step == 3) {
  //     kfs_fz(KFS_FZ_HIGH_POS);
  //   } else if (fz_step == 4) {
  //     kfs_fz(KFS_FZ_BOOK_POS);
  //   }
  // }

  // if (ps4_->is_pushed_right()) {
  //   // kfs_fxを動かす
  //   if (fx_step == 1) {
  //     kfs_fx(KFS_FX_EXPAND_POS);
  //     fx_step = 2;
  //   } else {
  //     kfs_fx(KFS_FX_NORMAL_POS);
  //     fx_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_down()) {
  //   // 1段下のkfs_fz位置へ移動
  //   fz_step--;
  //   if (fz_step < 1) {
  //     fz_step = 1;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "fz_step: %d", fz_step);
  //   if (fz_step == 1) {
  //     kfs_fz(KFS_FZ_LOW_POS);
  //   } else if (fz_step == 2) {
  //     kfs_fz(KFS_FZ_MIDDLE_POS);
  //   } else if (fz_step == 3) {
  //     kfs_fz(KFS_FZ_HIGH_POS);
  //   }
  // }

  // if (ps4_->is_pushed_left()) {
  //   // front_pumpを動かす。止めるときは電磁弁も一緒に動く
  //   if (front_pump_step == 1) {
  //     kfs_front_pump(1.0);
  //     kfs_front_valve(false);
  //     front_pump_step = 2;
  //   } else {
  //     kfs_front_pump(0.0);
  //     kfs_front_valve(true);
  //     // setTimeout風で電磁弁をOFFにする。
  //     manual_mode4_front_valve_timer_ = this->create_wall_timer(250ms, [this]() {
  //       kfs_front_valve(false);
  //       manual_mode4_front_valve_timer_->cancel();
  //     });
  //     front_pump_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   // kfs_fyawを90度進める
  //   fyaw_step++;
  //   if (fyaw_step > 3) {
  //     fyaw_step = 3;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
  //   if (fyaw_step == 1) {
  //     kfs_fyaw(KFS_FYAW_FRONT_ANGLE);
  //   } else if (fyaw_step == 2) {
  //     kfs_fyaw(KFS_FYAW_SIDE_ANGLE);
  //   } else if (fyaw_step == 3) {
  //     kfs_fyaw(KFS_FYAW_REAR_ANGLE);
  //   }
  // }

  // if (ps4_->is_pushed_circle()) {
  //   // kfs_fyawを微調整（指令値を増加）
  //   kfs_fyaw(kfs_fyaw_position_ref_ + 0.1);
  // }

  // if (ps4_->is_pushed_cross()) {
  //   // kfs_fyawを90度戻す
  //   fyaw_step--;
  //   if (fyaw_step < 1) {
  //     fyaw_step = 1;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "fyaw_step: %d", fyaw_step);
  //   if (fyaw_step == 1) {
  //     kfs_fyaw(KFS_FYAW_FRONT_ANGLE);
  //   } else if (fyaw_step == 2) {
  //     kfs_fyaw(KFS_FYAW_SIDE_ANGLE);
  //   } else if (fyaw_step == 3) {
  //     kfs_fyaw(KFS_FYAW_REAR_ANGLE);
  //   }
  // }

  // if (ps4_->is_pushed_square()) {
  //   // kfs_fyawを微調整（指令値を減少）
  //   kfs_fyaw(kfs_fyaw_position_ref_ - 0.1);
  // }

  // if (ps4_->is_pushed_l1()) {
  //   // kfs_fxの微調整（指令値を減少）
  //   kfs_fx(kfs_fx_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r1()) {
  //   // kfs_fxの微調整（指令値を増加）
  //   kfs_fx(kfs_fx_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_l2()) {
  //   // kfs_fzの微調整（指令値を減少）
  //   kfs_fz(kfs_fz_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r2()) {
  //   // kfs_fzの微調整（指令値を増加）
  //   kfs_fz(kfs_fz_position_ref_ + 0.01);
  // }
}

void R1MainNode::manual_mode5_rkfs(void)
{
  // int & rx_step = manual_mode5_rx_step_;
  // int & rz_step = manual_mode5_rz_step_;
  // int & ryaw_step = manual_mode5_ryaw_step_;
  // int & rear_pump_step = manual_mode5_rear_pump_step_;

  // if (ps4_->is_pushed_up()) {
  //   // 1段上のkfs_rz位置へ移動
  //   rz_step++;
  //   if (rz_step > 4) {
  //     rz_step = 4;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
  //   if (rz_step == 1) {
  //     kfs_rz(KFS_RZ_LOW_POS);
  //   } else if (rz_step == 2) {
  //     kfs_rz(KFS_RZ_MIDDLE_POS);
  //   } else if (rz_step == 3) {
  //     kfs_rz(KFS_RZ_HIGH_POS);
  //   } else if (rz_step == 4) {
  //     kfs_rz(KFS_RZ_BOOK_POS);
  //   }
  // }

  // if (ps4_->is_pushed_right()) {
  //   // kfs_rxを動かす
  //   if (rx_step == 1) {
  //     kfs_rx(KFS_RX_EXPAND_POS);
  //     rx_step = 2;
  //   } else {
  //     kfs_rx(KFS_RX_NORMAL_POS);
  //     rx_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_down()) {
  //   // 1段下のkfs_rz位置へ移動
  //   rz_step--;
  //   if (rz_step < 1) {
  //     rz_step = 1;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "rz_step: %d", rz_step);
  //   if (rz_step == 1) {
  //     kfs_rz(KFS_RZ_LOW_POS);
  //   } else if (rz_step == 2) {
  //     kfs_rz(KFS_RZ_MIDDLE_POS);
  //   } else if (rz_step == 3) {
  //     kfs_rz(KFS_RZ_HIGH_POS);
  //   }
  // }

  // if (ps4_->is_pushed_left()) {
  //   // rear_pumpを動かす。止めるときは電磁弁も一緒に動く
  //   if (rear_pump_step == 1) {
  //     kfs_rear_pump(1.0);
  //     kfs_rear_valve(false);
  //     rear_pump_step = 2;
  //   } else {
  //     kfs_rear_pump(0.0);
  //     kfs_rear_valve(true);
  //     // setTimeout風で電磁弁をOFFにする。
  //     manual_mode5_rear_valve_timer_ = this->create_wall_timer(250ms, [this]() {
  //       kfs_rear_valve(false);
  //       manual_mode5_rear_valve_timer_->cancel();
  //     });
  //     rear_pump_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   // kfs_ryawを90度進める
  //   ryaw_step++;
  //   if (ryaw_step > 3) {
  //     ryaw_step = 3;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
  //   if (ryaw_step == 1) {
  //     kfs_ryaw(KFS_RYAW_FRONT_ANGLE);
  //   } else if (ryaw_step == 2) {
  //     kfs_ryaw(KFS_RYAW_SIDE_ANGLE);
  //   } else if (ryaw_step == 3) {
  //     kfs_ryaw(KFS_RYAW_REAR_ANGLE);
  //   }
  // }

  // if (ps4_->is_pushed_circle()) {
  //   // kfs_ryawを微調整（指令値を増加）
  //   kfs_ryaw(kfs_ryaw_position_ref_ + 0.1);
  // }

  // if (ps4_->is_pushed_cross()) {
  //   // kfs_ryawを90度戻す
  //   ryaw_step--;
  //   if (ryaw_step < 1) {
  //     ryaw_step = 1;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "ryaw_step: %d", ryaw_step);
  //   if (ryaw_step == 1) {
  //     kfs_fyaw(KFS_RYAW_FRONT_ANGLE);
  //   } else if (ryaw_step == 2) {
  //     kfs_fyaw(KFS_RYAW_SIDE_ANGLE);
  //   } else if (ryaw_step == 3) {
  //     kfs_fyaw(KFS_RYAW_REAR_ANGLE);
  //   }
  // }

  // if (ps4_->is_pushed_square()) {
  //   // kfs_ryawを微調整（指令値を減少）
  //   kfs_ryaw(kfs_ryaw_position_ref_ - 0.1);
  // }

  // if (ps4_->is_pushed_l1()) {
  //   // kfs_rxの微調整（指令値を減少）
  //   kfs_rx(kfs_rx_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r1()) {
  //   // kfs_rxの微調整（指令値を増加）
  //   kfs_rx(kfs_rx_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_l2()) {
  //   // kfs_rzの微調整（指令値を減少）
  //   kfs_rz(kfs_rz_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r2()) {
  //   // kfs_rzの微調整（指令値を増加）
  //   kfs_rz(kfs_rz_position_ref_ + 0.01);
  // }
}

void R1MainNode::manual_mode6_r2_lift(void)
{
  // int & front_expand_step = manual_mode6_front_expand_step_;
  // int & rear_expand_step = manual_mode6_rear_expand_step_;
  // int & r2_lift_step = manual_mode6_r2_lift_step_;

  // if (ps4_->data.triangle) {
  //   if (r2_lift_step != 2) {
  //     r2_lift(R2_LIFT_MAX_VELOCITY);
  //     RCLCPP_INFO(this->get_logger(), "r2 lift up");
  //     r2_lift_step = 2;
  //   }
  // } else if (ps4_->data.cross) {
  //   if (r2_lift_step != 3) {
  //     r2_lift(-R2_LIFT_MAX_VELOCITY);
  //     RCLCPP_INFO(this->get_logger(), "r2 lift down");
  //     r2_lift_step = 3;
  //   }
  // } else {
  //   if (r2_lift_step != 1) {
  //     r2_lift(0.0);
  //     RCLCPP_INFO(this->get_logger(), "r2 lift stop");
  //     r2_lift_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_up()) {
  // }

  // if (ps4_->is_pushed_right()) {
  // }

  // if (ps4_->is_pushed_down()) {
  // }

  // if (ps4_->is_pushed_left()) {
  // }

  // if (ps4_->is_pushed_triangle()) {
  // }

  // if (ps4_->is_pushed_circle()) {
  //   if (rear_expand_step == 1) {
  //     rear_expand(REAR_EXPAND_EXPAND_POS);
  //     rear_expand_step = 2;
  //   } else {
  //     rear_expand(REAR_EXPAND_NORMAL_POS);
  //     rear_expand_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_cross()) {
  // }

  // if (ps4_->is_pushed_square()) {
  //   if (front_expand_step == 1) {
  //     front_expand(FRONT_EXPAND_EXPAND_POS);
  //     front_expand_step = 2;
  //   } else {
  //     front_expand(FRONT_EXPAND_NORMAL_POS);
  //     front_expand_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_l1()) {
  // }

  // if (ps4_->is_pushed_r1()) {
  // }

  // if (ps4_->is_pushed_l2()) {
  // }

  // if (ps4_->is_pushed_r2()) {
  // }
}

void R1MainNode::manual_mode7_spear_attack_task(int n)
{
  (void)n;
  // int & step = manual_mode7_spear_attack_task_step_;
  // RCLCPP_INFO(this->get_logger(), "manual_mode7_spear_attack_task step: %d", step);

  // if (step == 1) {
  //   // spear_roger1とspear_roger2を攻撃の高さに合わせる。
  //   // spear_moveとspear_rotateを初期位置に移動する。
  //   if (n == 1) {
  //     spear_roger1(SPEAR_ROGER1_LOW_ATTACK_POS);
  //     spear_roger2(SPEAR_ROGER2_LOW_ATTACK_POS);
  //   } else if (n == 2) {
  //     spear_roger1(SPEAR_ROGER1_MIDDLE_ATTACK_POS);
  //     spear_roger2(SPEAR_ROGER2_MIDDLE_ATTACK_POS);
  //   } else if (n == 3) {
  //     spear_roger1(SPEAR_ROGER1_HIGH_ATTACK_POS);
  //     spear_roger2(SPEAR_ROGER2_HIGH_ATTACK_POS);
  //   }
  //   spear_move(SPEAR_MOVE_NORMAL_POS);
  //   spear_rotate(SPEAR_ROTATE_NORMAL_POS);
  //   step++;
  // } else if (step == 2) {
  //   // spear_moveを動かして、やりを押し出す。
  //   spear_move(SPEAR_MOVE_ATTACK_POS);
  //   step++;
  // } else if (step == 3) {
  //   // spear_moveを動かして、やりを戻す。
  //   spear_move(SPEAR_MOVE_NORMAL_POS);
  //   step++;
  // } else if (step == 4) {
  //   // spear_roger1とspear_roger2をもとの高さに戻す。
  //   spear_roger1(SPEAR_ROGER1_NORMAL_POS);
  //   spear_roger2(SPEAR_ROGER2_NORMAL_POS);
  //   RCLCPP_INFO(this->get_logger(), "spear attack task completed");
  //   step = 1;
  // }
}

void R1MainNode::manual_mode7_spear_attack(void)
{
  // int & spear_hand_valve1_step = manual_mode7_spear_hand_valve1_step_;

  // if (ps4_->is_pushed_up()) {
  //   if (spear_hand_valve1_step == 1) {
  //     spear_hand_valve1(true);
  //     spear_hand_valve1_step = 2;
  //   } else {
  //     spear_hand_valve1(false);
  //     spear_hand_valve1_step = 1;
  //   }
  // }

  // if (ps4_->is_pushed_right()) {
  //   spear_rotate(spear_rotate_position_ref_ + 0.1);
  // }

  // if (ps4_->is_pushed_down()) {
  //   spear_move(spear_move_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_left()) {
  //   spear_rotate(spear_rotate_position_ref_ - 0.1);
  // }

  // if (ps4_->is_pushed_triangle()) {
  //   manual_mode7_spear_attack_task(3);
  // }

  // if (ps4_->is_pushed_circle()) {
  //   manual_mode7_spear_attack_task(2);
  // }

  // if (ps4_->is_pushed_cross()) {
  //   spear_move(spear_move_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_square()) {
  //   manual_mode7_spear_attack_task(1);
  // }

  // if (ps4_->is_pushed_l1()) {
  //   spear_roger1(spear_roger1_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r1()) {
  //   spear_roger1(spear_roger1_position_ref_ + 0.01);
  // }

  // if (ps4_->is_pushed_l2()) {
  //   spear_roger2(spear_roger2_position_ref_ - 0.01);
  // }

  // if (ps4_->is_pushed_r2()) {
  //   spear_roger2(spear_roger2_position_ref_ + 0.01);
  // }
}

void R1MainNode::auto_collect_kfs_task(void)
{
  ChassisAct & step = chassis_act_status_;

  if (step == ChassisAct::ACT1 || step == ChassisAct::ACT2) {
    // TODO: 進行方向と使用する回収機構の順番に応じて、OFFSETをいい感じに適応する
    geometry_msgs::msg::PoseStamped map_pos = get_map_pos();
    int n = current_robot_move_.forest_order.size();
    bool within = false;
    for (int i = 0; i < n; i++) {
      int target_forest_number = current_robot_move_.forest_order[i];
      double map_x = map_pos.pose.position.x;
      double map_y = map_pos.pose.position.y;
      double center_x = 0.0, center_y = 0.0, rect_yaw = 0.0, offset_x = 0.0, offset_y = 0.0;
      if (step == ChassisAct::ACT1) {
        center_x = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][0];
        center_y = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][1];
        rect_yaw = INNER_COLLECT_KFS_CENTER_POS[target_forest_number - 1][2];
      } else if (step == ChassisAct::ACT2) {
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
      if (step == ChassisAct::ACT1 && current_robot_move_.kfs_mechanism_type[i] == "front_kfs") {
        offset_x = COLLECT_KFS_OFFSET * std::cos(rect_yaw);
        offset_y = COLLECT_KFS_OFFSET * std::sin(rect_yaw);
      } else if (
        step == ChassisAct::ACT2 && current_robot_move_.kfs_mechanism_type[i] == "rear_kfs") {
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
        // LEDを設定、緑色
        sabacan_led_ref(0, 50, 0);
        break;
      }
    }
    // witinがfalseのときはLEDを赤色にする
    if (within == false) {
      sabacan_led_ref(50, 0, 0);
    }
  } else {
    // LEDを設定、白色
    sabacan_led_ref(50, 50, 50);
  }
}

void R1MainNode::auto_act0(void)
{
  ChassisAct & step = chassis_act_status_;

  if (step == ChassisAct::NONE) {
    sabacan_led_ref(50, 50, 0);
    if (ps4_->is_pushed_triangle()) {
      // 位置制御のプログラム実行
      // publish_chassis_act_ref(ChassisAct::ACT0_START);
      publish_robot_move(ChassisAct::ACT0_START, std::vector<int>{}, std::vector<std::string>{});
    }
    if (ps4_->is_pushed_circle()) {
      // 青のスタートゾーン
      set_mecanum_yaw(0.0);
      set_odometry(-5.5, 0.5, 0.0);
      set_initialpose(-5.5, 0.5, 0.0);
    }
    if (ps4_->is_pushed_cross()) {
      // 位置制御のプログラム実行
      // publish_chassis_act_ref(ChassisAct::ACT1_START);
      std::vector<int> forest_order;
      std::vector<std::string> collect_kfs_type;
      int j = 0;
      for (int i = 0; i < (int)KFS_FOREST_NUMBER.size(); i++) {
        int n = KFS_FOREST_NUMBER[i];
        if (n == 1 || n == 2 || n == 4 || n == 7 || n == 10) {
          forest_order.push_back(n);
          if (j == 0) {
            if (zone_ == "blue") {
              collect_kfs_type.push_back("rear_kfs");
            } else {
              // 今は何もしない
            }
            j++;
          } else if (j == 1) {
            if (zone_ == "blue") {
              collect_kfs_type.push_back("front_kfs");
            } else {
              // 今は何もしない
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "collect_kfs_type size error");
          }
        }
      }
      publish_robot_move(ChassisAct::ACT1_START, forest_order, collect_kfs_type);
    }
    if (ps4_->is_pushed_square()) {
      // 位置制御のプログラム実行
      // publish_chassis_act_ref(ChassisAct::ACT2_START);
      std::vector<int> forest_order;
      std::vector<std::string> collect_kfs_type;
      int j = 0;
      for (int i = 0; i < (int)KFS_FOREST_NUMBER.size(); i++) {
        int n = KFS_FOREST_NUMBER[i];
        if (n == 3 || n == 6 || n == 9 || n == 12 || n == 11 || n == 10) {
          forest_order.push_back(n);
          if (j == 0) {
            if (zone_ == "blue") {
              collect_kfs_type.push_back("front_kfs");
            } else {
              // 今は何もしない
            }
            j++;
          } else if (j == 1) {
            if (zone_ == "blue") {
              collect_kfs_type.push_back("rear_kfs");
            } else {
              // 今は何もしない
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "collect_kfs_type size error");
          }
        }
      }
      publish_robot_move(ChassisAct::ACT2_START, forest_order, collect_kfs_type);
    }
    if (ps4_->is_pushed_down()) {
      // 位置制御のプログラム実行
      // publish_chassis_act_ref(ChassisAct::ACT3_START);
      publish_robot_move(ChassisAct::ACT3_START, std::vector<int>{}, std::vector<std::string>{});
    }
    double vx_ref = CHASSIS_MAX_VELOCITY * (-1) * ps4_->data.left_stick_x;
    double vy_ref = CHASSIS_MAX_VELOCITY * ps4_->data.left_stick_y;
    double vz_ref = CHASSIS_MAX_OMEGA * ps4_->data.right_stick_x;
    chassis_move_vel(vx_ref, vy_ref, vz_ref);
  } else if (step == ChassisAct::ACT0_START) {
    // 何もしない
  } else if (step == ChassisAct::ACT0) {
    // 何もしない
  } else if (step == ChassisAct::ACT0_FINISH) {
    publish_chassis_act_ref(ChassisAct::NONE);
  } else if (step == ChassisAct::ACT1) {
    auto_collect_kfs_task();
  } else if (step == ChassisAct::ACT1_FINISH) {
    publish_chassis_act_ref(ChassisAct::NONE);
  } else if (step == ChassisAct::ACT2) {
    auto_collect_kfs_task();
  } else if (step == ChassisAct::ACT2_FINISH) {
    publish_chassis_act_ref(ChassisAct::NONE);
  } else if (step == ChassisAct::ACT3) {
    // 何もしない
  } else if (step == ChassisAct::ACT3_FINISH) {
    publish_chassis_act_ref(ChassisAct::NONE);
  }
}

void R1MainNode::reset_step(void)
{
  // 各手順のステップをリセット
  manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
  manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  manual_mode3_brake_valve_step_ = DEFAULT_STEP;
  manual_mode3_spear_hand_valve1_step_ = DEFAULT_STEP;
  manual_mode3_spear_hand_valve2_step_ = DEFAULT_STEP;
  manual_mode4_fx_step_ = DEFAULT_STEP;
  manual_mode4_fz_step_ = DEFAULT_STEP;
  manual_mode4_fyaw_step_ = DEFAULT_STEP;
  manual_mode4_front_pump_step_ = DEFAULT_STEP;
  manual_mode5_rx_step_ = DEFAULT_STEP;
  manual_mode5_rz_step_ = DEFAULT_STEP;
  manual_mode5_ryaw_step_ = DEFAULT_STEP;
  manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  manual_mode6_front_expand_step_ = DEFAULT_STEP;
  manual_mode6_rear_expand_step_ = DEFAULT_STEP;
  manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  manual_mode7_spear_hand_valve1_step_ = DEFAULT_STEP;
  publish_chassis_act_ref(ChassisAct::NONE);
}

void R1MainNode::reset_robot(void)
{
  // sabacanにリセット信号を送信する
  sabacan_reset();
  // stepをリセットする
  reset_step();
  // 現在の角度が0度となるようなオフセットを設定する。
  set_mecanum_yaw(0.0);
  // 位置は適当
  set_odometry(0.0, 0.0, 0.0);
  init_actuator();
  is_initialized_ = true;
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
    // 共通タスク
    double vx_ref = CHASSIS_MAX_VELOCITY * (-1) * ps4_->data.left_stick_x;
    double vy_ref = CHASSIS_MAX_VELOCITY * ps4_->data.left_stick_y;
    double vz_ref = CHASSIS_MAX_OMEGA * ps4_->data.right_stick_x;
    // 台形制御で速度を滑らかに変化させる
    // double vx = simple_trapezoid_vx_.update(vx_ref);
    // double vy = simple_trapezoid_vy_.update(vy_ref);
    // double vz = simple_trapezoid_omega_.update(vz_ref);
    chassis_move_vel(vx_ref, vy_ref, vz_ref);

    // optionsが押されたときは電源をOFFにする
    if (ps4_->is_pushed_options()) {
      sabacan_power_ref(!sabacan_is_ems_);
    }
    // psボタンが押されたときはsabacan resetを行う
    if (ps4_->is_pushed_ps()) {
      reset_robot();
    }
    // shareボタンが押されたときはモードを切り替える
    if (ps4_->is_pushed_share()) {
      if (const auto * manual_sub = std::get_if<ManualSubState>(&current_state.sub)) {
        if (*manual_sub == ManualSubState::MODE1_DETECT_ORIGIN) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE2_POLE});
        } else if (*manual_sub == ManualSubState::MODE2_POLE) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE3_SPEAR});
        } else if (*manual_sub == ManualSubState::MODE3_SPEAR) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE4_FKFS});
        } else if (*manual_sub == ManualSubState::MODE4_FKFS) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE5_RKFS});
        } else if (*manual_sub == ManualSubState::MODE5_RKFS) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE6_R2_LIFT});
        } else if (*manual_sub == ManualSubState::MODE6_R2_LIFT) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE7_SPEAR_ATTACK});
        } else if (*manual_sub == ManualSubState::MODE7_SPEAR_ATTACK) {
          state_machine_->set_next_state({MainState::MANUAL, ManualSubState::MODE1_DETECT_ORIGIN});
        }
      }
    }

    // 初期化が行われていない場合はこれ以降の処理は実行しない
    // TODO: 初期化の待ち時間が面倒だったら、この処理はなくす
    if (is_initialized_ == false) {
      return;
    }

    // 状態に応じて、各タスクを実行
    if (const auto * manual_sub = std::get_if<ManualSubState>(&current_state.sub)) {
      if (*manual_sub == ManualSubState::MODE1_DETECT_ORIGIN) {
        manual_mode1_detect_origin();
      } else if (*manual_sub == ManualSubState::MODE2_POLE) {
        manual_mode2_pole();
      } else if (*manual_sub == ManualSubState::MODE3_SPEAR) {
        manual_mode3_spear();
      } else if (*manual_sub == ManualSubState::MODE4_FKFS) {
        manual_mode4_fkfs();
      } else if (*manual_sub == ManualSubState::MODE5_RKFS) {
        manual_mode5_rkfs();
      } else if (*manual_sub == ManualSubState::MODE6_R2_LIFT) {
        manual_mode6_r2_lift();
      } else if (*manual_sub == ManualSubState::MODE7_SPEAR_ATTACK) {
        manual_mode7_spear_attack();
      }
    }
  }
}

void R1MainNode::auto_task(void)
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
    // 共通タスク

    // optionsが押されたときは電源をOFFにする
    if (ps4_->is_pushed_options()) {
      sabacan_power_ref(!sabacan_is_ems_);
    }
    // psボタンが押されたときはsabacan resetを行う
    if (ps4_->is_pushed_ps()) {
      reset_robot();
    }
    // shareボタンが押されたときはモードを切り替える
    if (ps4_->is_pushed_share()) {
      if (const auto * auto_sub = std::get_if<AutoSubState>(&current_state.sub)) {
        if (*auto_sub == AutoSubState::ACT0) {
          state_machine_->set_next_state({MainState::AUTO, AutoSubState::ACT0});
        }
      }
    }

    // 初期化が行われていない場合はこれ以降の処理は実行しない
    // TODO: 初期化の待ち時間が面倒だったら、この処理はなくす
    if (is_initialized_ == false) {
      return;
    }

    // 状態に応じて、各タスクを実行
    if (const auto * auto_sub = std::get_if<AutoSubState>(&current_state.sub)) {
      if (*auto_sub == AutoSubState::ACT0) {
        auto_act0();
      }
    }
  }
}

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

void R1MainNode::test_spear(void) {}

void R1MainNode::test_r2_lift(void)
{
  // 三角を押している間モータが正回転、バツを押している間モータが逆回転する
  if (ps4_->data.triangle) {
    // r2_lift(15);
    RCLCPP_INFO(this->get_logger(), "r2 lift up");
  } else if (ps4_->data.cross) {
    // r2_lift(-15);
    RCLCPP_INFO(this->get_logger(), "r2 lift down");
  } else {
    // r2_lift(0);
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
