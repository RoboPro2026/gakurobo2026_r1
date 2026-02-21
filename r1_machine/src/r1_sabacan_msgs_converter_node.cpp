/**
 * @file r1_sabacan_msgs_converter_node.cpp
 * @author Yamaguchi Yudai
 * @brief sabacan_msgsとr1_machineのノード間でのメッセージ変換を行うノード。
 * @version 0.1
 * @date 2025-10-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>
#include <limits>
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
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_float.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_ref_int.hpp"
#include "sabacan_msgs/msg/sabacan_gpio_status.hpp"
#include "sabacan_msgs/msg/sabacan_robomas_status.hpp"
#include "sabacan_single_control_msgs/msg/sabacan_robomas_single_ref.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

struct BoardInfo
{
  int board_id;
  int number;

  bool operator==(const BoardInfo & other) const
  {
    return (board_id == other.board_id) && (number == other.number);
  }

  bool operator!=(const BoardInfo & other) const { return !this->operator==(other); }
};

struct DebugMotorInfo
{
  BoardInfo board_info;
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr motor_status_publisher;

  bool operator==(const DebugMotorInfo & other) const { return board_info == other.board_info; }

  bool operator!=(const DebugMotorInfo & other) const { return !this->operator==(other); }
};

struct DebugGPIOInputInfo
{
  BoardInfo board_info;
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr gpio_input_status_publisher;

  bool operator==(const DebugGPIOInputInfo & other) const { return board_info == other.board_info; }

  bool operator!=(const DebugGPIOInputInfo & other) const { return !this->operator==(other); }
};

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_sabacan_msgs_converter_node")
  {
    // ========== Publisherとsubscriptionの作成 ==========
    // sabacan_gpio_ref_publisher_の作成
    sabacan_gpio_ref_int_publisher_.resize(10);
    sabacan_gpio_ref_float_publisher_.resize(10);
    for (size_t i = 0; i < 10; i++) {
      sabacan_gpio_ref_int_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefInt>(
          "/sabacan_gpio_ref_int" + std::to_string(i), 10);
      sabacan_gpio_ref_float_publisher_[i] =
        this->create_publisher<sabacan_msgs::msg::SabacanGPIORefFloat>(
          "/sabacan_gpio_ref_float" + std::to_string(i), 10);
    }

    // sabacan_robomas_status_subscription_の作成
    sabacan_robomas_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_robomas_status_subscription_.size(); i++) {
      sabacan_robomas_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanRobomasStatus>(
          "/sabacan_robomas_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg) {
            this->sabacan_robomas_status_callback(i, msg);
          });
    }

    // sabacan_gpio_status_subscription_の作成
    sabacan_gpio_status_subscription_.resize(10);
    for (size_t i = 0; i < sabacan_gpio_status_subscription_.size(); i++) {
      sabacan_gpio_status_subscription_[i] =
        this->create_subscription<sabacan_msgs::msg::SabacanGPIOStatus>(
          "/sabacan_gpio_status" + std::to_string(i), 10,
          [this, i](sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg) {
            this->sabacan_gpio_status_callback(i, msg);
          });
    }
    // メカナム
    mecanum_wheel_speeds_ref_subscription_ = this->create_subscription<r1_msgs::msg::Mecanum>(
      "/mecanum_wheel_speeds_ref", 10,
      std::bind(&MyNode::mecanum_wheel_speeds_ref_callback, this, std::placeholders::_1));
    mecanum_wheel_speeds_feedback_publisher_ =
      this->create_publisher<r1_msgs::msg::Mecanum>("/mecanum_wheel_speeds_feedback", 10);

    // オドメトリ
    odometry_encoder_publisher_ =
      this->create_publisher<r1_msgs::msg::OdometryEncoder>("/odometry_encoder", 10);

    // KFS回収機構の指令値
    kfs_fx_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fx_motor_ref", 10,
      std::bind(&MyNode::kfs_fx_motor_ref_callback, this, std::placeholders::_1));
    kfs_fz_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fz_motor_ref", 10,
      std::bind(&MyNode::kfs_fz_motor_ref_callback, this, std::placeholders::_1));
    kfs_fyaw_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_fyaw_motor_ref", 10,
      std::bind(&MyNode::kfs_fyaw_motor_ref_callback, this, std::placeholders::_1));
    kfs_rx_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_rx_motor_ref", 10,
      std::bind(&MyNode::kfs_rx_motor_ref_callback, this, std::placeholders::_1));
    kfs_rz_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_rz_motor_ref", 10,
      std::bind(&MyNode::kfs_rz_motor_ref_callback, this, std::placeholders::_1));
    kfs_ryaw_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/kfs_ryaw_motor_ref", 10,
      std::bind(&MyNode::kfs_ryaw_motor_ref_callback, this, std::placeholders::_1));
    // KFS回収機構のフィードバック
    kfs_fx_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_fx_linear_motion_status", 10);
    kfs_fz_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_fz_linear_motion_status", 10);
    kfs_fyaw_angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/kfs_fyaw_angle_motion_status", 10);
    kfs_rx_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_rx_linear_motion_status", 10);
    kfs_rz_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/kfs_rz_linear_motion_status", 10);
    kfs_ryaw_angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/kfs_ryaw_angle_motion_status", 10);

    // 展開の指令値
    front_expand_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/front_expand_motor_ref", 10,
      std::bind(&MyNode::front_expand_motor_ref_callback, this, std::placeholders::_1));
    rear_expand_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/rear_expand_motor_ref", 10,
      std::bind(&MyNode::rear_expand_motor_ref_callback, this, std::placeholders::_1));
    // 展開のフィードバック
    front_expand_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/front_expand_linear_motion_status", 10);
    rear_expand_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/rear_expand_linear_motion_status", 10);

    // R2昇降の指令値
    r2_lift_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/r2_lift_motor_ref", 10,
      std::bind(&MyNode::r2_lift_motor_ref_callback, this, std::placeholders::_1));
    // R2昇降のフィードバック
    r2_lift_motor_status_publisher_ =
      this->create_publisher<r1_msgs::msg::Motor>("/r2_lift_motor_status", 10);

    // ポール回収の指令値
    pole_x1_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/pole_x1_motor_ref", 10,
      std::bind(&MyNode::pole_x1_motor_ref_callback, this, std::placeholders::_1));
    pole_x2_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/pole_x2_motor_ref", 10,
      std::bind(&MyNode::pole_x2_motor_ref_callback, this, std::placeholders::_1));
    pole_y_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/pole_y_motor_ref", 10,
      std::bind(&MyNode::pole_y_motor_ref_callback, this, std::placeholders::_1));
    pole_roger_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/pole_roger_motor_ref", 10,
      std::bind(&MyNode::pole_roger_motor_ref_callback, this, std::placeholders::_1));
    // ポール回収のフィードバック
    pole_x1_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/pole_x1_linear_motion_status", 10);
    pole_x2_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/pole_x2_linear_motion_status", 10);
    pole_y_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/pole_y_linear_motion_status", 10);
    pole_roger_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/pole_roger_linear_motion_status", 10);

    // やりの指令値
    spear_roger1_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/spear_roger1_motor_ref", 10,
      std::bind(&MyNode::spear_roger1_motor_ref_callback, this, std::placeholders::_1));
    spear_roger2_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/spear_roger2_motor_ref", 10,
      std::bind(&MyNode::spear_roger2_motor_ref_callback, this, std::placeholders::_1));
    spear_move_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/spear_move_motor_ref", 10,
      std::bind(&MyNode::spear_move_motor_ref_callback, this, std::placeholders::_1));
    spear_rotate_motor_ref_subscription_ = this->create_subscription<r1_msgs::msg::MotorRef>(
      "/spear_rotate_motor_ref", 10,
      std::bind(&MyNode::spear_rotate_motor_ref_callback, this, std::placeholders::_1));
    // やりのフィードバック
    spear_roger1_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/spear_roger1_linear_motion_status", 10);
    spear_roger2_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/spear_roger2_linear_motion_status", 10);
    spear_move_linear_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::LinearMotion>("/spear_move_linear_motion_status", 10);
    spear_rotate_angle_motion_status_publisher_ =
      this->create_publisher<r1_msgs::msg::AngleMotion>("/spear_rotate_angle_motion_status", 10);

    //KFS真空ポンプ
    kfs_front_pump_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_front_pump_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_front_pump_gpio_pwm_ref_callback, this, std::placeholders::_1));
    kfs_rear_pump_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_rear_pump_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_rear_pump_gpio_pwm_ref_callback, this, std::placeholders::_1));

    // 真空電磁弁
    kfs_front_valve_gpio_pwm_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioPwmRef>(
        "/kfs_front_valve_gpio_pwm_ref", 10,
        std::bind(&MyNode::kfs_front_valve_gpio_pwm_ref_callback, this, std::placeholders::_1));
    kfs_rear_valve_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/kfs_rear_valve_gpio_pwm_ref", 10,
      std::bind(&MyNode::kfs_rear_valve_gpio_pwm_ref_callback, this, std::placeholders::_1));

    // KFSリミットスイッチ
    kfs_front_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/kfs_front_switch_status", 10);
    kfs_rear_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/kfs_rear_switch_status", 10);

    // ポール回収サーボ
    pole_servo1_gpio_servo_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioServoRef>(
        "/pole_servo1_gpio_servo_ref", 10,
        std::bind(&MyNode::pole_servo1_gpio_servo_ref_callback, this, std::placeholders::_1));
    pole_servo2_gpio_servo_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioServoRef>(
        "/pole_servo2_gpio_servo_ref", 10,
        std::bind(&MyNode::pole_servo2_gpio_servo_ref_callback, this, std::placeholders::_1));
    pole_servo3_gpio_servo_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioServoRef>(
        "/pole_servo3_gpio_servo_ref", 10,
        std::bind(&MyNode::pole_servo3_gpio_servo_ref_callback, this, std::placeholders::_1));
    pole_servo4_gpio_servo_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioServoRef>(
        "/pole_servo4_gpio_servo_ref", 10,
        std::bind(&MyNode::pole_servo4_gpio_servo_ref_callback, this, std::placeholders::_1));
    // ポール回収電磁弁
    pole_valve1_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/pole_valve1_gpio_pwm_ref", 10,
      std::bind(&MyNode::pole_valve1_gpio_pwm_ref_callback, this, std::placeholders::_1));
    pole_valve2_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/pole_valve2_gpio_pwm_ref", 10,
      std::bind(&MyNode::pole_valve2_gpio_pwm_ref_callback, this, std::placeholders::_1));
    pole_valve3_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/pole_valve3_gpio_pwm_ref", 10,
      std::bind(&MyNode::pole_valve3_gpio_pwm_ref_callback, this, std::placeholders::_1));
    pole_valve4_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/pole_valve4_gpio_pwm_ref", 10,
      std::bind(&MyNode::pole_valve4_gpio_pwm_ref_callback, this, std::placeholders::_1));
    // やり電磁弁
    spear_hand_valve1_gpio_pwm_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioPwmRef>(
        "/spear_hand_valve1_gpio_pwm_ref", 10,
        std::bind(&MyNode::spear_hand_valve1_gpio_pwm_ref_callback, this, std::placeholders::_1));
    spear_hand_valve2_gpio_pwm_ref_subscription_ =
      this->create_subscription<r1_msgs::msg::GpioPwmRef>(
        "/spear_hand_valve2_gpio_pwm_ref", 10,
        std::bind(&MyNode::spear_hand_valve2_gpio_pwm_ref_callback, this, std::placeholders::_1));
    // やりリミットスイッチ
    spear_move_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/spear_move_switch_status", 10);
    spear_rotate_switch_status_publisher_ =
      this->create_publisher<r1_msgs::msg::GpioInput>("/spear_rotate_switch_status", 10);
    // ブレーキ用電磁弁
    brake_valve_gpio_pwm_ref_subscription_ = this->create_subscription<r1_msgs::msg::GpioPwmRef>(
      "/brake_valve_gpio_pwm_ref", 10,
      std::bind(&MyNode::brake_valve_gpio_pwm_ref_callback, this, std::placeholders::_1));

    // ========== sabacan_single_control_msgsのpublisherを作成 =========
    // メカナム
    for (int i = 0; i < 4; ++i) {
      const auto & info = mecanum_[i];
      const std::string topic = "/sabacan_robomas_ref" + std::to_string(info.board_id) + "/motor" +
                                std::to_string(info.number);
      mecanum_single_ref_publisher_[i] =
        this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
          topic, 10);
    }
    // KFS回収機構
    kfs_fx_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "sabacan_robomas_ref" + std::to_string(kfs_fx_.board_id) + "/motor" +
          std::to_string(kfs_fx_.number),
        10);
    kfs_fz_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_fz_.board_id) + "/motor" +
          std::to_string(kfs_fz_.number),
        10);
    kfs_fyaw_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_fyaw_.board_id) + "/motor" +
          std::to_string(kfs_fyaw_.number),
        10);
    kfs_rx_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_rx_.board_id) + "/motor" +
          std::to_string(kfs_rx_.number),
        10);
    kfs_rz_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_rz_.board_id) + "/motor" +
          std::to_string(kfs_rz_.number),
        10);
    kfs_ryaw_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(kfs_ryaw_.board_id) + "/motor" +
          std::to_string(kfs_ryaw_.number),
        10);
    // 展開
    front_expand_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(front_expand_.board_id) + "/motor" +
          std::to_string(front_expand_.number),
        10);
    rear_expand_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(rear_expand_.board_id) + "/motor" +
          std::to_string(rear_expand_.number),
        10);
    // R2昇降
    r2_lift_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(r2_lift_.board_id) + "/motor" +
          std::to_string(r2_lift_.number),
        10);
    // ポール
    pole_x1_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(pole_x1_.board_id) + "/motor" +
          std::to_string(pole_x1_.number),
        10);
    pole_x2_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(pole_x2_.board_id) + "/motor" +
          std::to_string(pole_x2_.number),
        10);
    pole_y_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(pole_y_.board_id) + "/motor" +
          std::to_string(pole_y_.number),
        10);
    pole_roger_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(pole_roger_.board_id) + "/motor" +
          std::to_string(pole_roger_.number),
        10);
    // やり
    spear_roger1_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(spear_roger1_.board_id) + "/motor" +
          std::to_string(spear_roger1_.number),
        10);
    spear_roger2_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(spear_roger2_.board_id) + "/motor" +
          std::to_string(spear_roger2_.number),
        10);
    spear_move_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(spear_move_.board_id) + "/motor" +
          std::to_string(spear_move_.number),
        10);
    spear_rotate_single_ref_publisher_ =
      this->create_publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>(
        "/sabacan_robomas_ref" + std::to_string(spear_rotate_.board_id) + "/motor" +
          std::to_string(spear_rotate_.number),
        10);
    // デバッグ用のpublisherを追加
    // ---------- ロボマス制御 ----------
    // 足回り
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FL],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[FR],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_fr_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RL],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rl_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        mecanum_[RR],
        this->create_publisher<r1_msgs::msg::Motor>("/debug_mecanum_rr_motor_status", 10)});
    // KFS回収機構
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fx_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fx_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fz_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fz_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_fyaw_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_fyaw_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_rx_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_rx_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_rz_, this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_rz_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        kfs_ryaw_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_kfs_ryaw_motor_status", 10)});
    // R2昇降
    debug_motor_.push_back(
      DebugMotorInfo{
        r2_lift_, this->create_publisher<r1_msgs::msg::Motor>("/debug_r2_lift_motor_status", 10)});
    // ポール
    debug_motor_.push_back(
      DebugMotorInfo{
        pole_x1_, this->create_publisher<r1_msgs::msg::Motor>("/debug_pole_x1_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        pole_x2_, this->create_publisher<r1_msgs::msg::Motor>("/debug_pole_x2_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        pole_y_, this->create_publisher<r1_msgs::msg::Motor>("/debug_pole_y_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        pole_roger_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_pole_roger_motor_status", 10)});
    // やり
    debug_motor_.push_back(
      DebugMotorInfo{
        spear_roger1_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_spear_roger1_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        spear_roger2_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_spear_roger2_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        spear_move_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_spear_move_motor_status", 10)});
    debug_motor_.push_back(
      DebugMotorInfo{
        spear_rotate_,
        this->create_publisher<r1_msgs::msg::Motor>("/debug_spear_rotate_motor_status", 10)});

    // ---------- GPIO ----------
    // KFSリミットスイッチ
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        kfs_front_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_kfs_front_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        kfs_rear_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_kfs_rear_switch_status", 10)});
    // やりリミットスイッチ
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        spear_move_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_spear_move_switch_status", 10)});
    debug_gpio_input_.push_back(
      DebugGPIOInputInfo{
        spear_rotate_switch_,
        this->create_publisher<r1_msgs::msg::GpioInput>("/debug_spear_rotate_switch_status", 10)});

    // 100Hzのタイマーを作成
    timer_ = this->create_wall_timer(10ms, std::bind(&MyNode::timer_callback, this));
  }

  bool board_info_match(BoardInfo a, BoardInfo b)
  {
    return a.board_id == b.board_id && a.number == b.number;
  }

  void sabacan_robomas_status_callback(
    int baord_id, sabacan_msgs::msg::SabacanRobomasStatus::SharedPtr msg)
  {
    auto msg_status = r1_msgs::msg::Motor();
    msg_status.motor_type = msg->motor_type;
    msg_status.control_type = msg->control_type;
    msg_status.motor_state = msg->motor_state;
    msg_status.torque = msg->torque;
    msg_status.speed = msg->speed;
    msg_status.pos = msg->pos;
    msg_status.abs_pos = msg->abs_pos;
    msg_status.abs_speed = msg->abs_speed;
    msg_status.abs_turn_cnt = msg->abs_turn_cnt;
    msg_status.vesc_voltage = msg->vesc_voltage;
    msg_status.vesc_current = msg->vesc_current;
    msg_status.vesc_speed = msg->vesc_speed;

    BoardInfo receive{baord_id, msg->motor_number};

    if (receive == mecanum_[FL]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[FR]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[RL]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == mecanum_[RR]) {
      mecanum_wheel_speeds_feedback_[msg->motor_number] = msg->speed;
    }
    if (receive == odometry_encoder_[X]) {
      odometry_encoder_pos_values_[X] = msg->abs_pos;
      odometry_encoder_speed_values_[X] = msg->abs_speed;
    }
    if (receive == odometry_encoder_[Y]) {
      odometry_encoder_pos_values_[Y] = msg->abs_pos;
      odometry_encoder_speed_values_[Y] = msg->abs_speed;
    }
    if (receive == kfs_fx_) {
      kfs_fx_linear_motion_value_.torque = msg->torque;
      kfs_fx_linear_motion_value_.speed = msg->speed;
      kfs_fx_linear_motion_value_.pos = msg->pos;
    }
    if (receive == kfs_fz_) {
      kfs_fz_linear_motion_value_.torque = msg->torque;
      kfs_fz_linear_motion_value_.speed = msg->speed;
      kfs_fz_linear_motion_value_.pos = msg->pos;
    }
    if (receive == kfs_fyaw_) {
      kfs_fyaw_angle_motion_value_.torque = msg->torque;
      kfs_fyaw_angle_motion_value_.speed = msg->speed;
      kfs_fyaw_angle_motion_value_.pos = msg->pos;
    }
    if (receive == kfs_rx_) {
      kfs_rx_linear_motion_value_.torque = msg->torque;
      kfs_rx_linear_motion_value_.speed = msg->speed;
      kfs_rx_linear_motion_value_.pos = msg->pos;
    }
    if (receive == kfs_rz_) {
      kfs_rz_linear_motion_value_.torque = msg->torque;
      kfs_rz_linear_motion_value_.speed = msg->speed;
      kfs_rz_linear_motion_value_.pos = msg->pos;
    }
    if (receive == kfs_ryaw_) {
      kfs_ryaw_angle_motion_value_.torque = msg->torque;
      kfs_ryaw_angle_motion_value_.speed = msg->speed;
      kfs_ryaw_angle_motion_value_.pos = msg->pos;
    }
    if (receive == front_expand_) {
      front_expand_linear_motion_value_.torque = msg->torque;
      front_expand_linear_motion_value_.speed = msg->speed;
      front_expand_linear_motion_value_.pos = msg->pos;
    }
    if (receive == rear_expand_) {
      rear_expand_linear_motion_value_.torque = msg->torque;
      rear_expand_linear_motion_value_.speed = msg->speed;
      rear_expand_linear_motion_value_.pos = msg->pos;
    }
    if (receive == r2_lift_) {
      r2_lift_motor_value_ = msg_status;
    }
    if (receive == pole_x1_) {
      pole_x1_linear_motion_value_.torque = msg->torque;
      pole_x1_linear_motion_value_.speed = msg->speed;
      pole_x1_linear_motion_value_.pos = msg->pos;
    }
    if (receive == pole_x2_) {
      pole_x2_linear_motion_value_.torque = msg->torque;
      pole_x2_linear_motion_value_.speed = msg->speed;
      pole_x2_linear_motion_value_.pos = msg->pos;
    }
    if (receive == pole_y_) {
      pole_y_linear_motion_value_.torque = msg->torque;
      pole_y_linear_motion_value_.speed = msg->speed;
      pole_y_linear_motion_value_.pos = msg->pos;
    }
    if (receive == pole_roger_) {
      pole_roger_linear_motion_value_.torque = msg->torque;
      pole_roger_linear_motion_value_.speed = msg->speed;
      pole_roger_linear_motion_value_.pos = msg->pos;
    }
    if (receive == spear_roger1_) {
      spear_roger1_linear_motion_value_.torque = msg->torque;
      spear_roger1_linear_motion_value_.speed = msg->speed;
      spear_roger1_linear_motion_value_.pos = msg->pos;
    }
    if (receive == spear_roger2_) {
      spear_roger2_linear_motion_value_.torque = msg->torque;
      spear_roger2_linear_motion_value_.speed = msg->speed;
      spear_roger2_linear_motion_value_.pos = msg->pos;
    }
    if (receive == spear_move_) {
      spear_move_linear_motion_value_.torque = msg->torque;
      spear_move_linear_motion_value_.speed = msg->speed;
      spear_move_linear_motion_value_.pos = msg->pos;
    }
    if (receive == spear_rotate_) {
      spear_rotate_angle_motion_value_.torque = msg->torque;
      spear_rotate_angle_motion_value_.speed = msg->speed;
      spear_rotate_angle_motion_value_.pos = msg->pos;
    }
    // デバッグ用パブリッシュ（マッチするものに配信）
    // デバッグ用データは受信callback内で送信する
    for (size_t i = 0; i < debug_motor_.size(); ++i) {
      const auto & dbg = debug_motor_[i];
      if (receive == dbg.board_info && dbg.motor_status_publisher) {
        dbg.motor_status_publisher->publish(msg_status);
      }
    }
  }

  void sabacan_gpio_status_callback(
    int board_id, sabacan_msgs::msg::SabacanGPIOStatus::SharedPtr msg)
  {
    BoardInfo receive{board_id, msg->pin_number};

    if (receive == kfs_front_switch_) {
      kfs_front_switch_value_.status = msg->input;
    }
    if (receive == kfs_rear_switch_) {
      kfs_rear_switch_value_.status = msg->input;
    }
    if (receive == spear_move_switch_) {
      spear_move_switch_value_.status = msg->input;
    }
    if (receive == spear_rotate_switch_) {
      spear_rotate_switch_value_.status = msg->input;
    }

    // デバッグ用パブリッシュ（GPIO入力）
    r1_msgs::msg::GpioInput gpio_msg;
    gpio_msg.status = msg->input;
    for (size_t i = 0; i < debug_gpio_input_.size(); ++i) {
      const auto & dbg = debug_gpio_input_[i];
      if (receive == dbg.board_info && dbg.gpio_input_status_publisher) {
        dbg.gpio_input_status_publisher->publish(gpio_msg);
      }
    }
  }

  void mecanum_wheel_speeds_ref_callback(const r1_msgs::msg::Mecanum::ConstSharedPtr msg)
  {
    // RCLCPP_INFO(
    //   this->get_logger(), "wheel_speeds_ref: FL=%f, FR=%f, RL=%f, RR=%f", msg->fl_wheel_speed,
    //   msg->fr_wheel_speed, msg->rl_wheel_speed, msg->rr_wheel_speed);
    for (int i = 0; i < MECANUM_NUM; i++) {
      auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
      msg_ref.control_type = "VELOCITY";

      if (mecanum_[i] == mecanum_[FL]) {
        msg_ref.ref = msg->fl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[FR]) {
        msg_ref.ref = msg->fr_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RL]) {
        msg_ref.ref = msg->rl_wheel_speed;
      } else if (mecanum_[i] == mecanum_[RR]) {
        msg_ref.ref = msg->rr_wheel_speed;
      }

      mecanum_single_ref_publisher_[i]->publish(msg_ref);
    }
  }

  // TODO: callbackのところはcallback関数をいじって、処理を共通化する

  void kfs_fx_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fx_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_fz_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fz_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_fyaw_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_fyaw_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_rx_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_rx_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_rz_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_rz_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_ryaw_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    kfs_ryaw_single_ref_publisher_->publish(msg_ref);
  }

  void front_expand_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    front_expand_single_ref_publisher_->publish(msg_ref);
  }

  void rear_expand_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    rear_expand_single_ref_publisher_->publish(msg_ref);
  }

  void r2_lift_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    r2_lift_single_ref_publisher_->publish(msg_ref);
  }

  void pole_x1_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    pole_x1_single_ref_publisher_->publish(msg_ref);
  }

  void pole_x2_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    pole_x2_single_ref_publisher_->publish(msg_ref);
  }

  void pole_y_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    pole_y_single_ref_publisher_->publish(msg_ref);
  }

  void pole_roger_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    pole_roger_single_ref_publisher_->publish(msg_ref);
  }

  void spear_roger1_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    spear_roger1_single_ref_publisher_->publish(msg_ref);
  }

  void spear_roger2_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    spear_roger2_single_ref_publisher_->publish(msg_ref);
  }

  void spear_move_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    spear_move_single_ref_publisher_->publish(msg_ref);
  }

  void spear_rotate_motor_ref_callback(const r1_msgs::msg::MotorRef::SharedPtr msg)
  {
    auto msg_ref = sabacan_single_control_msgs::msg::SabacanRobomasSingleRef();
    msg_ref.control_type = msg->control_type;
    msg_ref.ref = msg->ref;
    spear_rotate_single_ref_publisher_->publish(msg_ref);
  }

  void kfs_front_pump_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_front_pump_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_rear_pump_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_rear_pump_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_front_valve_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_front_valve_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void kfs_rear_valve_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = kfs_rear_valve_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_servo1_gpio_servo_ref_callback(const r1_msgs::msg::GpioServoRef::SharedPtr msg)
  {
    auto info = pole_servo1_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefInt();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_int = msg->ref;
    sabacan_gpio_ref_int_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_servo2_gpio_servo_ref_callback(const r1_msgs::msg::GpioServoRef::SharedPtr msg)
  {
    auto info = pole_servo2_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefInt();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_int = msg->ref;
    sabacan_gpio_ref_int_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_servo3_gpio_servo_ref_callback(const r1_msgs::msg::GpioServoRef::SharedPtr msg)
  {
    auto info = pole_servo3_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefInt();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_int = msg->ref;
    sabacan_gpio_ref_int_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_servo4_gpio_servo_ref_callback(const r1_msgs::msg::GpioServoRef::SharedPtr msg)
  {
    auto info = pole_servo4_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefInt();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_int = msg->ref;
    sabacan_gpio_ref_int_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_valve1_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = pole_valve1_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_valve2_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = pole_valve2_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_valve3_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = pole_valve3_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void pole_valve4_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = pole_valve4_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void spear_hand_valve1_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = spear_hand_valve1_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void spear_hand_valve2_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = spear_hand_valve2_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void brake_valve_gpio_pwm_ref_callback(const r1_msgs::msg::GpioPwmRef::SharedPtr msg)
  {
    auto info = brake_valve_;
    auto sabacan_msg = sabacan_msgs::msg::SabacanGPIORefFloat();
    sabacan_msg.pin_number = info.number;
    sabacan_msg.ref_float = msg->ref;
    sabacan_gpio_ref_float_publisher_[info.board_id]->publish(sabacan_msg);
  }

  void timer_callback()
  {
    // ロボマス制御
    // 足回り
    auto msg_feedback = r1_msgs::msg::Mecanum();
    msg_feedback.fl_wheel_speed = mecanum_wheel_speeds_feedback_[FL];
    msg_feedback.fr_wheel_speed = mecanum_wheel_speeds_feedback_[FR];
    msg_feedback.rl_wheel_speed = mecanum_wheel_speeds_feedback_[RL];
    msg_feedback.rr_wheel_speed = mecanum_wheel_speeds_feedback_[RR];
    mecanum_wheel_speeds_feedback_publisher_->publish(msg_feedback);
    // オドメトリ
    auto odom_msg = r1_msgs::msg::OdometryEncoder();
    odom_msg.encoder_pos_x = odometry_encoder_pos_values_[X];
    odom_msg.encoder_pos_y = odometry_encoder_pos_values_[Y];
    odom_msg.encoder_speed_x = odometry_encoder_speed_values_[X];
    odom_msg.encoder_speed_y = odometry_encoder_speed_values_[Y];
    odometry_encoder_publisher_->publish(odom_msg);
    // KFS回収機構
    kfs_fx_linear_motion_status_publisher_->publish(kfs_fx_linear_motion_value_);
    kfs_fz_linear_motion_status_publisher_->publish(kfs_fz_linear_motion_value_);
    kfs_fyaw_angle_motion_status_publisher_->publish(kfs_fyaw_angle_motion_value_);
    kfs_rx_linear_motion_status_publisher_->publish(kfs_rx_linear_motion_value_);
    kfs_rz_linear_motion_status_publisher_->publish(kfs_rz_linear_motion_value_);
    kfs_ryaw_angle_motion_status_publisher_->publish(kfs_ryaw_angle_motion_value_);
    // 展開
    front_expand_linear_motion_status_publisher_->publish(front_expand_linear_motion_value_);
    rear_expand_linear_motion_status_publisher_->publish(rear_expand_linear_motion_value_);
    // R2昇降
    r2_lift_motor_status_publisher_->publish(r2_lift_motor_value_);
    // ポール
    pole_x1_linear_motion_status_publisher_->publish(pole_x1_linear_motion_value_);
    pole_x2_linear_motion_status_publisher_->publish(pole_x2_linear_motion_value_);
    pole_y_linear_motion_status_publisher_->publish(pole_y_linear_motion_value_);
    pole_roger_linear_motion_status_publisher_->publish(pole_roger_linear_motion_value_);
    // やり
    spear_roger1_linear_motion_status_publisher_->publish(spear_roger1_linear_motion_value_);
    spear_roger2_linear_motion_status_publisher_->publish(spear_roger2_linear_motion_value_);
    spear_move_linear_motion_status_publisher_->publish(spear_move_linear_motion_value_);
    spear_rotate_angle_motion_status_publisher_->publish(spear_rotate_angle_motion_value_);
    // GPIO
    kfs_front_switch_status_publisher_->publish(kfs_front_switch_value_);
    kfs_rear_switch_status_publisher_->publish(kfs_rear_switch_value_);
    spear_move_switch_status_publisher_->publish(spear_move_switch_value_);
    spear_rotate_switch_status_publisher_->publish(spear_rotate_switch_value_);
  }

  // ======== Sabacan Publisher and Subscription =========
  // gpio_refのpublisherを宣言
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIORefInt>::SharedPtr>
    sabacan_gpio_ref_int_publisher_;
  std::vector<rclcpp::Publisher<sabacan_msgs::msg::SabacanGPIORefFloat>::SharedPtr>
    sabacan_gpio_ref_float_publisher_;
  // robomas_statusのsubscriptionを宣言
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanRobomasStatus>::SharedPtr>
    sabacan_robomas_status_subscription_;
  // gpio_statusのsubscriptionを宣言
  std::vector<rclcpp::Subscription<sabacan_msgs::msg::SabacanGPIOStatus>::SharedPtr>
    sabacan_gpio_status_subscription_;
  // ---------- sabacan_single_control_msgsのpublisherを宣言 ----------
  // メカナム
  std::vector<
    rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr>
    mecanum_single_ref_publisher_{MECANUM_NUM};
  // KFS回収機構
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fx_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fz_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_fyaw_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_rx_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_rz_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    kfs_ryaw_single_ref_publisher_;
  // 展開
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    front_expand_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    rear_expand_single_ref_publisher_;
  // R2昇降
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    r2_lift_single_ref_publisher_;
  // ポール
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    pole_x1_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    pole_x2_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    pole_y_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    pole_roger_single_ref_publisher_;
  // やり
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    spear_roger1_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    spear_roger2_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    spear_move_single_ref_publisher_;
  rclcpp::Publisher<sabacan_single_control_msgs::msg::SabacanRobomasSingleRef>::SharedPtr
    spear_rotate_single_ref_publisher_;

  // ======== R1 Machine Publisher and Subscription ========
  // -------- ロボマス ----------
  // メカナムの指令値
  rclcpp::Subscription<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_ref_subscription_;
  // メカナムホイールの回転速度のフィードバック
  rclcpp::Publisher<r1_msgs::msg::Mecanum>::SharedPtr mecanum_wheel_speeds_feedback_publisher_;
  // オドメトリのフィードバック
  rclcpp::Publisher<r1_msgs::msg::OdometryEncoder>::SharedPtr odometry_encoder_publisher_;
  // KFS回収機構の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fx_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fz_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_fyaw_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_rx_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_rz_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr kfs_ryaw_motor_ref_subscription_;
  // KFS回収機構のフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_fx_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_fz_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr kfs_fyaw_angle_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_rx_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr kfs_rz_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr kfs_ryaw_angle_motion_status_publisher_;
  // 展開の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr front_expand_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr rear_expand_motor_ref_subscription_;
  // 展開のフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    front_expand_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    rear_expand_linear_motion_status_publisher_;
  // R2昇降の指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr r2_lift_motor_ref_subscription_;
  // R2昇降のフィードバック
  rclcpp::Publisher<r1_msgs::msg::Motor>::SharedPtr r2_lift_motor_status_publisher_;
  // ポールの指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr pole_x1_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr pole_x2_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr pole_y_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr pole_roger_motor_ref_subscription_;
  // ポールのフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr pole_x1_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr pole_x2_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr pole_y_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    pole_roger_linear_motion_status_publisher_;
  // やりの指令値
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr spear_roger1_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr spear_roger2_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr spear_move_motor_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::MotorRef>::SharedPtr spear_rotate_motor_ref_subscription_;
  // やりのフィードバック
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    spear_roger1_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    spear_roger2_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::LinearMotion>::SharedPtr
    spear_move_linear_motion_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::AngleMotion>::SharedPtr
    spear_rotate_angle_motion_status_publisher_;
  // --------- GPIO ----------
  // KFS真空ポンプ
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_front_pump_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_rear_pump_gpio_pwm_ref_subscription_;
  // 真空電磁弁
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_front_valve_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    kfs_rear_valve_gpio_pwm_ref_subscription_;
  // KFSリミットスイッチ
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr kfs_front_switch_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr kfs_rear_switch_status_publisher_;
  // ポール回収サーボモータ
  rclcpp::Subscription<r1_msgs::msg::GpioServoRef>::SharedPtr
    pole_servo1_gpio_servo_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioServoRef>::SharedPtr
    pole_servo2_gpio_servo_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioServoRef>::SharedPtr
    pole_servo3_gpio_servo_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioServoRef>::SharedPtr
    pole_servo4_gpio_servo_ref_subscription_;
  // ポール回収電磁弁
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve1_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve2_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve3_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr pole_valve4_gpio_pwm_ref_subscription_;
  // やり電磁弁
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    spear_hand_valve1_gpio_pwm_ref_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr
    spear_hand_valve2_gpio_pwm_ref_subscription_;
  // やりリミットスイッチ
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr spear_move_switch_status_publisher_;
  rclcpp::Publisher<r1_msgs::msg::GpioInput>::SharedPtr spear_rotate_switch_status_publisher_;
  // ブレーキ用電磁弁
  rclcpp::Subscription<r1_msgs::msg::GpioPwmRef>::SharedPtr brake_valve_gpio_pwm_ref_subscription_;
  // ========= Board id ==========
  // ---------- ロボマス制御 ----------
  // 足回り
  std::vector<BoardInfo> mecanum_ = {
    {.board_id = 1, .number = 0},
    {.board_id = 1, .number = 1},
    {.board_id = 1, .number = 2},
    {.board_id = 1, .number = 3}};
  // オドメトリエンコーダ
  std::vector<BoardInfo> odometry_encoder_ = {
    {.board_id = 1, .number = 0}, {.board_id = 1, .number = 1}};
  // KFS回収機構
  BoardInfo kfs_fx_ = {.board_id = 2, .number = 0};
  BoardInfo kfs_fz_ = {.board_id = 2, .number = 1};
  BoardInfo kfs_fyaw_ = {.board_id = 2, .number = 2};
  BoardInfo front_expand_ = {.board_id = 2, .number = 3};
  BoardInfo kfs_rx_ = {.board_id = 3, .number = 0};
  BoardInfo kfs_rz_ = {.board_id = 3, .number = 1};
  BoardInfo kfs_ryaw_ = {.board_id = 3, .number = 2};
  BoardInfo rear_expand_ = {.board_id = 3, .number = 3};
  // R2昇降
  BoardInfo r2_lift_ = {.board_id = 6, .number = 0};
  // ポール回収
  BoardInfo pole_x1_ = {.board_id = 4, .number = 0};
  BoardInfo pole_x2_ = {.board_id = 4, .number = 1};
  BoardInfo pole_y_ = {.board_id = 4, .number = 2};
  BoardInfo pole_roger_ = {.board_id = 4, .number = 3};
  // やり
  BoardInfo spear_roger1_ = {.board_id = 5, .number = 0};
  BoardInfo spear_roger2_ = {.board_id = 5, .number = 1};
  BoardInfo spear_move_ = {.board_id = 5, .number = 2};
  BoardInfo spear_rotate_ = {.board_id = 5, .number = 3};
  // ---------- GPIO ----------
  // KFS回収
  BoardInfo kfs_front_pump_ = {.board_id = 1, .number = 0};
  BoardInfo kfs_rear_pump_ = {.board_id = 1, .number = 1};
  BoardInfo kfs_front_valve_ = {.board_id = 1, .number = 2};
  BoardInfo kfs_rear_valve_ = {.board_id = 1, .number = 3};
  BoardInfo kfs_front_switch_ = {.board_id = 2, .number = 0};
  BoardInfo kfs_rear_switch_ = {.board_id = 2, .number = 1};
  // ポール回収
  BoardInfo pole_servo1_ = {.board_id = 3, .number = 0};
  BoardInfo pole_servo2_ = {.board_id = 3, .number = 1};
  BoardInfo pole_servo3_ = {.board_id = 3, .number = 2};
  BoardInfo pole_servo4_ = {.board_id = 3, .number = 3};
  BoardInfo pole_valve1_ = {.board_id = 3, .number = 4};
  BoardInfo pole_valve2_ = {.board_id = 3, .number = 5};
  BoardInfo pole_valve3_ = {.board_id = 3, .number = 6};
  BoardInfo pole_valve4_ = {.board_id = 3, .number = 7};
  // やり
  BoardInfo spear_hand_valve1_ = {.board_id = 1, .number = 8};
  BoardInfo spear_hand_valve2_ = {.board_id = 2, .number = 7};
  BoardInfo spear_move_switch_ = {.board_id = 2, .number = 2};
  BoardInfo spear_rotate_switch_ = {.board_id = 2, .number = 3};
  // ブレーキ用電磁弁
  BoardInfo brake_valve_ = {.board_id = 2, .number = 8};

  // ========== 各種ステータス。タイマーで一定周期で送信 =========
  // ---------- ロボマス制御 ----------
  // 足回り
  std::vector<double> mecanum_wheel_speeds_feedback_ = std::vector<double>(MECANUM_NUM);
  // オドメトリ用エンコーダ
  std::vector<double> odometry_encoder_pos_values_ = std::vector<double>(ODOMETRY_NUM);
  std::vector<double> odometry_encoder_speed_values_ = std::vector<double>(ODOMETRY_NUM);
  // KFS回収機構
  r1_msgs::msg::LinearMotion kfs_fx_linear_motion_value_;
  r1_msgs::msg::LinearMotion kfs_fz_linear_motion_value_;
  r1_msgs::msg::AngleMotion kfs_fyaw_angle_motion_value_;
  r1_msgs::msg::LinearMotion kfs_rx_linear_motion_value_;
  r1_msgs::msg::LinearMotion kfs_rz_linear_motion_value_;
  r1_msgs::msg::AngleMotion kfs_ryaw_angle_motion_value_;
  // 展開
  r1_msgs::msg::LinearMotion front_expand_linear_motion_value_;
  r1_msgs::msg::LinearMotion rear_expand_linear_motion_value_;
  // R2昇降
  r1_msgs::msg::Motor r2_lift_motor_value_;
  // ポール回収
  r1_msgs::msg::LinearMotion pole_x1_linear_motion_value_;
  r1_msgs::msg::LinearMotion pole_x2_linear_motion_value_;
  r1_msgs::msg::LinearMotion pole_y_linear_motion_value_;
  r1_msgs::msg::LinearMotion pole_roger_linear_motion_value_;
  // やり
  r1_msgs::msg::LinearMotion spear_roger1_linear_motion_value_;
  r1_msgs::msg::LinearMotion spear_roger2_linear_motion_value_;
  r1_msgs::msg::LinearMotion spear_move_linear_motion_value_;
  r1_msgs::msg::AngleMotion spear_rotate_angle_motion_value_;
  // ---------- GPIO ----------
  r1_msgs::msg::GpioInput kfs_front_switch_value_;
  r1_msgs::msg::GpioInput kfs_rear_switch_value_;
  r1_msgs::msg::GpioInput spear_move_switch_value_;
  r1_msgs::msg::GpioInput spear_rotate_switch_value_;

  static constexpr int FL = 0;
  static constexpr int FR = 1;
  static constexpr int RL = 2;
  static constexpr int RR = 3;
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int MECANUM_NUM = 4;
  static constexpr int ODOMETRY_NUM = 2;
  // デバッグ用
  std::vector<DebugMotorInfo> debug_motor_;
  std::vector<DebugGPIOInputInfo> debug_gpio_input_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
