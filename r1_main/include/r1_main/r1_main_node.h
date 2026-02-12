/**
 * @file r1_main_node.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "magic_enum.hpp"
#include "r1_main/ps4.h"
#include "r1_main/simple_trapezoid.h"
#include "r1_main/state_machine.h"
#include "r1_msgs/msg/angle_motion.hpp"
#include "r1_msgs/msg/gpio_esc_ref.hpp"
#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/gpio_pwm_ref.hpp"
#include "r1_msgs/msg/gpio_servo_ref.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_led_ref.hpp"
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sabacan_msgs/srv/sabacan_reset.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class R1MainNode : public rclcpp::Node
{
public:
  std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<PS4> ps4_;
  SimpleTrapezoid simple_trapezoid_vx_;
  SimpleTrapezoid simple_trapezoid_vy_;
  SimpleTrapezoid simple_trapezoid_omega_;

  // publisherとsubscriber
  // 足回りの速度指令
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // joyの受信
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // ========== KFS回収 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_fyaw_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rx_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_rz_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr kfs_ryaw_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fx_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fz_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_fyaw_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_rx_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_rz_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kfs_ryaw_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fx_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fz_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_fyaw_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_rx_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_rz_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr kfs_ryaw_mode_status_subscription_;
  // ========== 展開 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_expand_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_expand_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_expand_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_expand_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr front_expand_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_expand_mode_status_subscription_;
  // ========== R2昇降 ==========
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr r2_lift_motor_ref_publisher_;
  // ========== ポール回収 ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_x1_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_x2_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_y_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pole_roger_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_x1_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_x2_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_y_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pole_roger_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_x1_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_x2_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_y_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pole_roger_mode_status_subscription_;
  // ========== やり ==========
  // 指令値Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_roger1_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_roger2_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_move_position_ref_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spear_rotate_position_ref_publisher_;
  // 原点検出Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_roger1_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_roger2_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_move_detect_origin_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spear_rotate_detect_origin_publisher_;
  // mode Subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_roger1_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_roger2_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_move_mode_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr spear_rotate_mode_status_subscription_;

  // KFS真空ポンプ
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_pump_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_pump_gpio_pwm_ref_publisher_;
  // 真空電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_front_valve_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr kfs_rear_valve_gpio_pwm_ref_publisher_;
  // KFSリミットスイッチ
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_front_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr kfs_rear_switch_status_subscription_;
  // ポール回収サーボ
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo1_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo2_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo3_gpio_servo_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr pole_servo4_gpio_servo_ref_publisher_;
  //  ポール回収電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve1_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve2_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve3_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve4_gpio_pwm_ref_publisher_;
  // やりハンド電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr spear_hand_valve1_gpio_pwm_ref_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr spear_hand_valve2_gpio_pwm_ref_publisher_;
  // やりリミットスイッチ
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr spear_move_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr spear_rotate_switch_status_subscription_;
  // ブレーキ用電磁弁
  rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr brake_valve_gpio_pwm_ref_publisher_;
  // ========== Sabacan ==========
  // 電源基板の指令値Publisher
  rclcpp::Publisher<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr sabacan_power_ref_publisher_;
  // LED基板の指令値Publisher
  rclcpp::Publisher<sabacan_msgs::msg::SabacanLEDRef>::SharedPtr sabacan_led_ref_publisher_;
  // リセットクライアント
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_power_reset_client_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id1_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id2_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id3_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id4_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id5_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_robomas_reset_client_id6_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_gpio_reset_client_id1_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_gpio_reset_client_id2_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_gpio_reset_client_id3_;
  rclcpp::Client<sabacan_msgs::srv::SabacanReset>::SharedPtr sabacan_led_reset_client_;
  rclcpp::Time sabacan_reset_last_send_time_ = rclcpp::Time(0LL, RCL_SYSTEM_TIME);
  bool sabacan_reset_last_send_valid_ = false;
  size_t sabacan_reset_step_ = 0;
  // IMUのSubscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
  double roll_ = 0.0;
  // メカナムのyaw_offset
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_offset_publisher_;
  // タイマー
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;

  // スイッチの状態
  bool kfs_front_switch_status_ = false;
  bool kfs_rear_switch_status_ = false;
  bool spear_move_switch_status_ = false;
  bool spear_rotate_switch_status_ = false;
  // 各モードの状態
  bool is_kfs_fx_pos_mode_ = false;
  bool is_kfs_fz_pos_mode_ = false;
  bool is_kfs_fyaw_pos_mode_ = false;
  bool is_kfs_rx_pos_mode_ = false;
  bool is_kfs_rz_pos_mode_ = false;
  bool is_kfs_ryaw_pos_mode_ = false;
  bool is_front_expand_pos_mode_ = false;
  bool is_rear_expand_pos_mode_ = false;
  bool is_pole_x1_pos_mode_ = false;
  bool is_pole_x2_pos_mode_ = false;
  bool is_pole_y_pos_mode_ = false;
  bool is_pole_roger_pos_mode_ = false;
  bool is_spear_roger1_pos_mode_ = false;
  bool is_spear_roger2_pos_mode_ = false;
  bool is_spear_move_pos_mode_ = false;
  bool is_spear_rotate_pos_mode_ = false;
  // 指令値
  double kfs_fx_position_ref_ = 0.0;
  double kfs_fz_position_ref_ = 0.0;
  double kfs_fyaw_position_ref_ = 0.0;
  double kfs_rx_position_ref_ = 0.0;
  double kfs_rz_position_ref_ = 0.0;
  double kfs_ryaw_position_ref_ = 0.0;
  double front_expand_position_ref_ = 0.0;
  double rear_expand_position_ref_ = 0.0;
  double r2_lift_velocity_ref_ = 0.0;
  double pole_x1_position_ref_ = 0.0;
  double pole_x2_position_ref_ = 0.0;
  double pole_y_position_ref_ = 0.0;
  double pole_roger_position_ref_ = 0.0;
  double spear_roger1_position_ref_ = 0.0;
  double spear_roger2_position_ref_ = 0.0;
  double spear_move_position_ref_ = 0.0;
  double spear_rotate_position_ref_ = 0.0;
  double kfs_front_pump_ref_ = 0.0;
  double kfs_rear_pump_ref_ = 0.0;
  bool kfs_front_valve_ref_ = 0.0;
  bool kfs_rear_valve_ref_ = 0.0;
  int pole_servo1_angle_ref_ = 0;
  int pole_servo2_angle_ref_ = 0;
  int pole_servo3_angle_ref_ = 0;
  int pole_servo4_angle_ref_ = 0;
  bool pole_valve1_ref_ = false;
  bool pole_valve2_ref_ = false;
  bool pole_valve3_ref_ = false;
  bool pole_valve4_ref_ = false;
  bool spear_hand_valve1_ref_ = false;
  bool spear_hand_valve2_ref_ = false;
  double brake_valve_ref_ = 0.0;

  // sabacan
  bool sabacan_is_ems_ = false;
  static constexpr int SABACAN_AVAILABLE = 0;
  static constexpr int SABACAN_RESET_NOW = 1;
  static constexpr int SABACAN_RESET_SENDING = 2;
  int sabacan_reset_status_ = SABACAN_AVAILABLE;
  // actuator
  static constexpr int ACTUATOR_AVAILABLE = 0;
  static constexpr int ACTUATOR_INITIALIZING = 1;
  int actuator_status_ = ACTUATOR_AVAILABLE;

  bool is_initialized_ = false;

  // 指令値関係
  // ========== 足回り ==========
  double CHASSIS_MAX_VELOCITY = 0.0;
  double CHASSIS_MAX_OMEGA = 0.0;
  // ========== KFS回収 ==========
  // fx
  double KFS_FX_NORMAL_POS = 0.0;
  double KFS_FX_EXPAND_POS = 0.0;
  // fz
  double KFS_FZ_NORMAL_POS = 0.0;
  double KFS_FZ_LOW_POS = 0.0;
  double KFS_FZ_MIDDLE_POS = 0.0;
  double KFS_FZ_HIGH_POS = 0.0;
  double KFS_FZ_BOOK_POS = 0.0;
  // fyaw
  double KFS_FYAW_NORMAL_ANGLE = 0.0;
  double KFS_FYAW_FRONT_ANGLE = 0.0;
  double KFS_FYAW_SIDE_ANGLE = 0.0;
  double KFS_FYAW_REAR_ANGLE = 0.0;
  // rx
  double KFS_RX_NORMAL_POS = 0.0;
  double KFS_RX_EXPAND_POS = 0.0;
  // rz
  double KFS_RZ_NORMAL_POS = 0.0;
  double KFS_RZ_LOW_POS = 0.0;
  double KFS_RZ_MIDDLE_POS = 0.0;
  double KFS_RZ_HIGH_POS = 0.0;
  double KFS_RZ_BOOK_POS = 0.0;
  // ryaw
  double KFS_RYAW_NORMAL_ANGLE = 0.0;
  double KFS_RYAW_FRONT_ANGLE = 0.0;
  double KFS_RYAW_SIDE_ANGLE = 0.0;
  double KFS_RYAW_REAR_ANGLE = 0.0;
  // ========== 展開 ==========
  // R2昇降
  double R2_LIFT_MAX_VELOCITY = 0.0;
  // front_expand
  double FRONT_EXPAND_NORMAL_POS = 0.0;
  double FRONT_EXPAND_EXPAND_POS = 0.0;
  // rear_expand
  double REAR_EXPAND_NORMAL_POS = 0.0;
  double REAR_EXPAND_EXPAND_POS = 0.5;
  // ========== ポール回収 ==========
  // pole_x1
  double POLE_X1_NORMAL_POS = 0.0;
  double POLE_X1_EXPAND_POS = 0.0;
  // pole_x2
  double POLE_X2_NORMAL_POS = 0.0;
  double POLE_X2_EXPAND_POS = 0.0;
  // pole_y
  double POLE_Y_NORMAL_POS = 0.0;
  double POLE_Y_COLLECT_POS = 0.0;
  double POLE_Y_TRANSFER1_POS = 0.0;
  double POLE_Y_TRANSFER2_POS = 0.0;
  double POLE_Y_TRANSFER3_POS = 0.0;
  double POLE_Y_TRANSFER4_POS = 0.0;
  // pole_roger
  double POLE_ROGER_NORMAL_POS = 0.0;
  double POLE_ROGER_EXPAND_POS = 0.0;
  // servo1
  int POLE_SERVO1_NORMAL_ANGLE = 0;
  int POLE_SERVO1_HORIZONTAL_ANGLE = 0;
  // servo2
  int POLE_SERVO2_NORMAL_ANGLE = 0;
  int POLE_SERVO2_HORIZONTAL_ANGLE = 0;
  // servo3
  int POLE_SERVO3_NORMAL_ANGLE = 0;
  int POLE_SERVO3_HORIZONTAL_ANGLE = 0;
  // servo4
  int POLE_SERVO4_NORMAL_ANGLE = 0;
  int POLE_SERVO4_HORIZONTAL_ANGLE = 0;
  // ========== やり ==========
  // spear_roger1
  double SPEAR_ROGER1_NORMAL_POS = 0.0;
  double SPEAR_ROGER1_COMBINE_POS = 0.0;
  double SPEAR_ROGER1_TRANSFER_POS = 0.0;
  double SPEAR_ROGER1_LOW_ATTACK_POS = 0.0;
  double SPEAR_ROGER1_MIDDLE_ATTACK_POS = 0.0;
  double SPEAR_ROGER1_HIGH_ATTACK_POS = 0.0;
  // spear_roger2
  double SPEAR_ROGER2_NORMAL_POS = 0.0;
  double SPEAR_ROGER2_COMBINE_POS = 0.0;
  double SPEAR_ROGER2_TRANSFER_POS = 0.0;
  double SPEAR_ROGER2_LOW_ATTACK_POS = 0.0;
  double SPEAR_ROGER2_MIDDLE_ATTACK_POS = 0.0;
  double SPEAR_ROGER2_HIGH_ATTACK_POS = 0.0;
  // spear_move
  double SPEAR_MOVE_NORMAL_POS = 0.0;
  double SPEAR_MOVE_COMBINE_POS = 0.0;
  double SPEAR_MOVE_TRANSFER_POS = 0.0;
  double SPEAR_MOVE_VALVE1_POS = 0.0;
  double SPEAR_MOVE_VALVE2_POS = 0.0;
  double SPEAR_MOVE_ATTACK_POS = 0.0;
  // spear_rotate
  double SPEAR_ROTATE_NORMAL_POS = 0.0;
  double SPEAR_ROTATE_COMBINE_ANGLE = 0.0;

  // コンストラクタ
  R1MainNode();

  // ========== コールバック関数 =========
  // modeのcalalback
  void kfs_fx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_fz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_fyaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_rx_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_rz_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void kfs_ryaw_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void front_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void rear_expand_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_x1_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_x2_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_y_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pole_roger_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_roger1_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_roger2_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_move_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void spear_rotate_mode_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // スイッチのcallback
  void kfs_front_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void kfs_rear_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void spear_move_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  void spear_rotate_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg);
  // joyのcallback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback(void);
  void declare_and_get_parameter(
    const std::string & name, double & value, double default_value = 0.0);
  void declare_and_get_parameter(const std::string & name, int & value, int default_value = 0);
  // sabacan
  void sabacan_reset_update(void);
  void sabacan_reset(void);
  void sabacan_power_ref(bool is_ems);
  void sabacan_led_ref(uint8_t r, uint8_t g, uint8_t b);
  // 現在の状態に応じて、LEDを光らせる。
  void sabacan_led_update(void);
  // IMU
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publish_yaw_offset(double offset);
  // ========== 各動作の関数 ==========
  // 足回り
  void chassis_move_vel(double vx, double vy, double omega);
  // KFS回収
  void kfs_fx(double pos);
  void kfs_fz(double pos);
  void kfs_fyaw(double pos);
  void kfs_rx(double pos);
  void kfs_rz(double pos);
  void kfs_ryaw(double pos);
  // 展開
  void front_expand(double pos);
  void rear_expand(double pos);
  // R2昇降
  void r2_lift(double vel);
  // ポール回収
  void pole_x1(double pos);
  void pole_x2(double pos);
  void pole_y(double pos);
  void pole_roger(double pos);
  // やり
  void spear_roger1(double pos);
  void spear_roger2(double pos);
  void spear_move(double pos);
  void spear_rotate(double pos);
  // KFS真空ポンプ・電磁弁
  void kfs_front_pump(double pwm);
  void kfs_rear_pump(double pwm);
  void kfs_front_valve(bool on);
  void kfs_rear_valve(bool on);
  // ポールサーボ・電磁弁
  void pole_servo(int n, int angle);
  void pole_valve(int n, bool on);
  // やり電磁弁
  void spear_hand_valve1(bool on);
  void spear_hand_valve2(bool on);
  // ブレーキ電磁弁
  void brake_valve(bool on);
  // 動いていたら危険なアクチュエータは停止する
  // 位置制御は止められないので、そのまま
  // TODO: 位置制御系も止められるようにする
  void stop_actuator(void);
  // 位置制御系のアクチュエータを初期位置に移動する
  void init_actuator(void);
  void actuator_update(void);
  void set_emergency(bool enable);
  // ========== 原点検出関数 ==========
  // KFS回収
  void kfs_fx_detect_origin(void);
  void kfs_fz_detect_origin(void);
  void kfs_fyaw_detect_origin(void);
  void kfs_rx_detect_origin(void);
  void kfs_rz_detect_origin(void);
  void kfs_ryaw_detect_origin(void);
  // 展開
  void front_expand_detect_origin(void);
  void rear_expand_detect_origin(void);
  // ポール回収
  void pole_x1_detect_origin(void);
  void pole_x2_detect_origin(void);
  void pole_y_detect_origin(void);
  void pole_roger_detect_origin(void);
  // やり
  void spear_roger1_detect_origin(void);
  void spear_roger2_detect_origin(void);
  void spear_move_detect_origin(void);
  void spear_rotate_detect_origin(void);
  // ========== センサーの取得 ==========
  bool get_kfs_front_switch_status(void) { return kfs_front_switch_status_; }
  bool get_kfs_rear_switch_status(void) { return kfs_rear_switch_status_; }
  bool get_spear_move_switch_status(void) { return spear_move_switch_status_; }
  bool get_spear_rotate_switch_status(void) { return spear_rotate_switch_status_; }
  // ========== 各状態のタスク ==========
  void idle_task(void);
  void emergency_task(void);
  void manual_task(void);
  void auto_task(void);
  void main_task(void);
  // ========== テスト関数 ==========
  void test_front_kfs(void);
  void test_rear_kfs(void);
  void test_pole(void);
  void test_spear(void);
  void test_r2_lift(void);
  // ========== マニュアルモード ==========
  void manual_mode1_detect_origin(void);
  void manual_mode2_pole(void);
  void manual_mode2_collect_pole_task(void);
  void manual_mode3_spear(void);
  void manual_mode3_make_spear_task(int n);
  void manual_mode4_fkfs(void);
  void manual_mode5_rkfs(void);
  void manual_mode6_r2_lift(void);
  void manual_mode7_spear_attack(void);
  void manual_mode7_spear_attack_task(int n);
  static constexpr int DEFAULT_STEP = 1;
  int manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
  int manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  int manual_mode3_brake_valve_step_ = DEFAULT_STEP;
  int manual_mode3_spear_hand_valve1_step_ = DEFAULT_STEP;
  int manual_mode3_spear_hand_valve2_step_ = DEFAULT_STEP;
  int manual_mode4_fx_step_ = DEFAULT_STEP;
  int manual_mode4_fz_step_ = DEFAULT_STEP;
  int manual_mode4_fyaw_step_ = DEFAULT_STEP;
  int manual_mode4_front_pump_step_ = DEFAULT_STEP;
  int manual_mode5_rx_step_ = DEFAULT_STEP;
  int manual_mode5_rz_step_ = DEFAULT_STEP;
  int manual_mode5_ryaw_step_ = DEFAULT_STEP;
  int manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  int manual_mode6_front_expand_step_ = DEFAULT_STEP;
  int manual_mode6_rear_expand_step_ = DEFAULT_STEP;
  int manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  int manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  int manual_mode7_spear_hand_valve1_step_ = DEFAULT_STEP;
  void reset_step(void);
  rclcpp::TimerBase::SharedPtr manual_mode4_front_valve_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode5_rear_valve_timer_;
};
