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

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "magic_enum.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
#include "r1_msgs/msg/robot_move.hpp"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_led_ref.hpp"
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class R1MainNode : public rclcpp::Node
{
public:
  struct PositionAxisInterface
  {
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_ref_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_position_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_ref_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detect_origin_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr speed_mode_stop_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr move_mech_lock_publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_status_subscription;
    bool is_pos_mode = false;
    double position_ref = 0.0;
    double speed_ref = 0.0;
    double * position_ref_alias = nullptr;
    double * speed_ref_alias = nullptr;
  };

  struct VelocityAxisInterface
  {
    rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr motor_ref_publisher;
    double velocity_ref = 0.0;
    double * velocity_ref_alias = nullptr;
  };

  struct GpioPwmOutputInterface
  {
    rclcpp::Publisher<r1_msgs::msg::GpioPwmRef>::SharedPtr publisher;
    double ref = 0.0;
    double * double_ref_alias = nullptr;
    bool * bool_ref_alias = nullptr;
  };

  struct GpioServoOutputInterface
  {
    rclcpp::Publisher<r1_msgs::msg::GpioServoRef>::SharedPtr publisher;
    int ref = 0;
    int * ref_alias = nullptr;
  };

  struct GpioInputInterface
  {
    rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr subscription;
  };

  struct LifecycleClientInterface
  {
    std::string node_name;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
  };

  struct LedColor
  {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    bool operator==(const LedColor & other) const
    {
      return r == other.r && g == other.g && b == other.b;
    }

    bool operator!=(const LedColor & other) const { return !(*this == other); }
  };

  struct LedPattern
  {
    bool enabled = false;
    LedColor color{};
    // 0.0 のときは常時点灯、正の値のときはその周期[s]で点滅する。
    double blink_period_s = 0.0;
  };

  enum class KfsAutoCollectStatus
  {
    NONE,
    INNER_ACTIVE,            // 1回目: INNERが多い側から回収
    OUTER_ACTIVE,            // 1回目: OUTERが多い側から回収
    SECONDARY_INNER_ACTIVE,  // 2回目: 1回目の回収後に残ったINNER側を回収
    SECONDARY_OUTER_ACTIVE,  // 2回目: 1回目の回収後に残ったOUTER側を回収
  };

  struct KfsAutoCollectPlan
  {
    KfsAutoCollectStatus status = KfsAutoCollectStatus::NONE;
    std::vector<int> forest_order;
    std::vector<std::string> kfs_mechanism_type;
  };

  using AutoChassisStatus = ChassisAct;

  std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<PS4> ps4_;
  SimpleTrapezoid simple_trapezoid_vx_;
  SimpleTrapezoid simple_trapezoid_vy_;
  SimpleTrapezoid simple_trapezoid_omega_;
  std::map<std::string, PositionAxisInterface> position_axes_;
  std::map<std::string, VelocityAxisInterface> velocity_axes_;
  std::map<std::string, GpioPwmOutputInterface> gpio_pwm_outputs_;
  std::map<std::string, GpioServoOutputInterface> gpio_servo_outputs_;
  std::map<std::string, GpioInputInterface> gpio_inputs_;

  // publisherとsubscriber
  // 足回りの速度指令
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // joyの受信
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // ========== Sabacan ==========
  // 電源基板の指令値Publisher
  rclcpp::Publisher<sabacan_msgs::msg::SabacanPowerRef>::SharedPtr sabacan_power_ref_publisher_;
  // LED基板の指令値Publisher
  rclcpp::Publisher<sabacan_msgs::msg::SabacanLEDRef>::SharedPtr sabacan_led_ref_publisher_;
  // IMUのSubscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
  double roll_ = 0.0;
  // set_mecanum_yawのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_mecanum_yaw_publisher_;
  // set_swerve_drive_yawのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_swerve_drive_yaw_publisher_;
  // set_odometryのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr set_odometry_publisher_;
  // オドメトリのSubscription
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  // initialposeのPublisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_publisher_;
  // initialposeをPublish時に、遅延させる用のtimer
  rclcpp::TimerBase::SharedPtr initialpose_publish_timer_;
  rclcpp::TimerBase::SharedPtr initialpose_tf_log_timer_;
  // chassis_actのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr chassis_act_ref_publisher_;
  // chassis_actのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_act_status_subscription_;
  // robot_moveのPublisher
  rclcpp::Publisher<r1_msgs::msg::RobotMove>::SharedPtr robot_move_publisher_;
  // r1_machine_manage_node の初期化要求
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr r1_machine_initialize_publisher_;
  // r1_machine_manage_node の初期化完了通知
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr r1_machine_initialize_done_subscription_;
  // タイマー
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  double timer_rate_ = 100.0;
  double initialpose_tf_log_delay_sec_ = 1.0;
  rclcpp::Time initialize_done_time_;
  // tf関連
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<LifecycleClientInterface> lidar_lifecycle_clients_;
  // 速度指令値
  geometry_msgs::msg::Twist target_vel_;
  std::string cmd_vel_topic_ = "/cmd_vel";
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  // chassis_act
  ChassisAct chassis_act_status_ = ChassisAct::NONE;
  // arucoマーカ
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aruco_marker_id_publisher_;

  // zone
  std::string zone_;
  // スイッチの状態
  bool kfs_fz_low_switch_status_ = false;
  bool kfs_rz_low_switch_status_ = false;
  // 指令値
  double kfs_fx_position_ref_ = 0.0;
  double kfs_fz_position_ref_ = 0.0;
  double kfs_fyaw_position_ref_ = 0.0;
  double kfs_rx_position_ref_ = 0.0;
  double kfs_rz_position_ref_ = 0.0;
  double kfs_ryaw_position_ref_ = 0.0;
  double spear1_position_ref_ = 0.0;
  double spear2_position_ref_ = 0.0;
  double spear3_position_ref_ = 0.0;
  double spear4_position_ref_ = 0.0;
  double spear_x_position_ref_ = 0.0;
  double spear_y_position_ref_ = 0.0;
  double spear_roll_position_ref_ = 0.0;
  double spear_pitch1_position_ref_ = 0.0;
  double spear_pitch2_position_ref_ = 0.0;
  double r2_flift_position_ref_ = 0.0;
  double r2_rlift_position_ref_ = 0.0;
  double kfs_front_pump_ref_ = 0.0;
  double kfs_rear_pump_ref_ = 0.0;
  bool kfs_front_valve_ref_ = false;
  bool kfs_rear_valve_ref_ = false;
  // 千田機構
  bool spear_u1_valve_ref_ = false;
  bool spear_d1_valve_ref_ = false;
  bool spear_u2_valve_ref_ = false;
  bool spear_d2_valve_ref_ = false;
  // 大槻機構
  bool spear_hand1_valve_ref_ = false;
  bool spear_hand2_valve_ref_ = false;
  bool spear_hand_push_valve_ref_ = false;

  // sabacan
  bool sabacan_is_ems_ = false;

  static constexpr int LED_FKFS = 0;
  static constexpr int LED_RKFS = 1;
  static constexpr int LED_SYSTEM = 2;

  // status はその周期だけ有効な上書き表示、event は一定時間だけ優先される一時表示。
  LedPattern led_status_pattern_;
  LedPattern led_fkfs_status_pattern_;
  LedPattern led_rkfs_status_pattern_;
  LedPattern led_event_pattern_;
  rclcpp::Time led_event_expire_time_;
  LedColor last_led_color_{};
  bool has_last_led_color_ = false;
  LedColor last_led_fkfs_color_{};
  LedColor last_led_rkfs_color_{};
  bool has_last_led_fkfs_color_ = false;
  bool has_last_led_rkfs_color_ = false;

  double ps4_connection_timeout_ = 0.3;
  bool activate_lidar_on_ps_ = true;

  // shareボタン長押し判定
  rclcpp::Time share_press_start_time_;
  bool share_long_press_triggered_ = false;
  double SHARE_LONG_PRESS_SEC = 0.5;

  // robot_move
  r1_msgs::msg::RobotMove current_robot_move_;

  bool is_initialized_ = false;
  RobotState initial_state_{
    MainState::READY, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::MANUAL};

  // 指令値関係
  // ========== 足回り ==========
  double CHASSIS_MAKE_SPEAR_VELOCITY = 0.0;
  double CHASSIS_MAKE_SPEAR_OMEGA = 0.0;
  double CHASSIS_MAX_VELOCITY = 0.0;
  double CHASSIS_MAX_OMEGA = 0.0;
  // ========== KFS回収 ==========
  bool USE_KFS_MECH_LOCK = false;
  // fx
  double KFS_FX_NORMAL_POS = 0.0;
  double KFS_FX_STORAGE_POS = 0.0;
  double KFS_FX_START_POS = 0.0;
  double KFS_FX_PUT_POS = 0.0;
  double KFS_FX_EXPAND_POS = 0.0;
  double KFS_FX_LOW_MECH_LOCK_POS = 0.0;
  double KFS_FX_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_FX_R2_LIFT_POS = 0.0;
  // fz
  double KFS_FZ_NORMAL_POS = 0.0;
  double KFS_FZ_LOW_POS = 0.0;
  double KFS_FZ_MIDDLE_POS = 0.0;
  double KFS_FZ_HIGH_POS = 0.0;
  double KFS_FZ_PUT_POS = 0.0;
  double KFS_FZ_STORAGE_POS = 0.0;
  double KFS_FZ_LOW_MECH_LOCK_POS = 0.0;
  double KFS_FZ_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_FZ_R2_LIFT_POS = 0.0;
  // fyaw
  double KFS_FYAW_NORMAL_ANGLE = 0.0;
  double KFS_FYAW_FRONT_ANGLE = 0.0;
  double KFS_FYAW_SIDE_ANGLE = 0.0;
  double KFS_FYAW_REAR_ANGLE = 0.0;
  double KFS_FYAW_LOW_MECH_LOCK_ANGLE = 0.0;
  double KFS_FYAW_HIGH_MECH_LOCK_ANGLE = 0.0;
  // rx
  double KFS_RX_NORMAL_POS = 0.0;
  double KFS_RX_STORAGE_POS = 0.0;
  double KFS_RX_START_POS = 0.0;
  double KFS_RX_PUT_POS = 0.0;
  double KFS_RX_EXPAND_POS = 0.0;
  double KFS_RX_LOW_MECH_LOCK_POS = 0.0;
  double KFS_RX_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_RX_R2_LIFT_POS = 0.0;
  // rz
  double KFS_RZ_NORMAL_POS = 0.0;
  double KFS_RZ_LOW_POS = 0.0;
  double KFS_RZ_MIDDLE_POS = 0.0;
  double KFS_RZ_HIGH_POS = 0.0;
  double KFS_RZ_PUT_POS = 0.0;
  double KFS_RZ_STORAGE_POS = 0.0;
  double KFS_RZ_LOW_MECH_LOCK_POS = 0.0;
  double KFS_RZ_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_RZ_R2_LIFT_POS = 0.0;
  // ryaw
  double KFS_RYAW_NORMAL_ANGLE = 0.0;
  double KFS_RYAW_FRONT_ANGLE = 0.0;
  double KFS_RYAW_SIDE_ANGLE = 0.0;
  double KFS_RYAW_REAR_ANGLE = 0.0;
  double KFS_RYAW_LOW_MECH_LOCK_ANGLE = 0.0;
  double KFS_RYAW_HIGH_MECH_LOCK_ANGLE = 0.0;
  // R2昇降
  double R2_FLIFT_NORMAL_POS = 0.0;
  double R2_FLIFT_UP_POS = 0.0;
  double R2_FLIFT_DOWN_POS = 0.0;
  double R2_RLIFT_NORMAL_POS = 0.0;
  double R2_RLIFT_UP_POS = 0.0;
  double R2_RLIFT_DOWN_POS = 0.0;

  // ========== やり ==========
  // spear1
  double SPEAR1_NORMAL_POS = 0.0;
  double SPEAR1_COLLECT1_POS = 0.0;
  double SPEAR1_COLLECT2_POS = 0.0;
  double SPEAR1_COLLECT3_POS = 0.0;
  double SPEAR1_MAKE_SPEAR_START_POS = 0.0;
  double SPEAR1_KFS_COLLECT_POS = 0.0;
  double SPEAR1_LOW_ATTACK_POS = 0.0;
  double SPEAR1_MIDDLE_ATTACK_POS = 0.0;
  double SPEAR1_HIGH_ATTACK_POS = 0.0;
  double SPEAR1_PUSH_VEL = 0.0;
  // spear2
  double SPEAR2_NORMAL_POS = 0.0;
  double SPEAR2_COLLECT1_POS = 0.0;
  double SPEAR2_COLLECT2_POS = 0.0;
  double SPEAR2_COLLECT3_POS = 0.0;
  double SPEAR2_MAKE_SPEAR_START_POS = 0.0;
  double SPEAR2_KFS_COLLECT_POS = 0.0;
  double SPEAR2_LOW_ATTACK_POS = 0.0;
  double SPEAR2_MIDDLE_ATTACK_POS = 0.0;
  double SPEAR2_HIGH_ATTACK_POS = 0.0;
  double SPEAR2_PUSH_VEL = 0.0;
  // spear3
  double SPEAR3_NORMAL_POS = 0.0;
  double SPEAR3_COLLECT_POS = 0.0;
  double SPEAR3_MAKE_SPEAR_START_POS = 0.0;
  double SPEAR3_KFS_COLLECT_POS = 0.0;
  double SPEAR3_PUSH_VEL = 0.0;
  // spear4
  double SPEAR4_NORMAL_POS = 0.0;
  double SPEAR4_COLLECT_POS = 0.0;
  double SPEAR4_MAKE_SPEAR_START_POS = 0.0;
  double SPEAR4_KFS_COLLECT_POS = 0.0;
  double SPEAR4_PUSH_VEL = 0.0;
  // spear_x
  double SPEAR_X_NORMAL_POS = 0.0;
  double SPEAR_X_MIDDLE_POS = 0.0;
  double SPEAR_X_MAKE_SPEAR1_POS = 0.0;
  double SPEAR_X_MAKE_SPEAR2_POS = 0.0;
  double SPEAR_X_MAKE_SPEAR3_POS = 0.0;
  double SPEAR_X_MAKE_SPEAR4_POS = 0.0;
  // spear_y
  double SPEAR_Y_NORMAL_POS = 0.0;
  double SPEAR_Y_EXPAND_POS = 0.0;
  // spear_roll
  double SPEAR_ROLL_NORMAL_ANGLE = 0.0;
  double SPEAR_ROLL_INV_NORMAL_ANGLE = 0.0;
  double SPEAR_ROLL_VERTICAL_ANGLE = 0.0;
  double SPEAR_ROLL_LOW_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL_MIDDLE_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL_HIGH_ATTACK_ANGLE = 0.0;
  // spear_pitch1
  double SPEAR_PITCH1_NORMAL_ANGLE = 0.0;
  double SPEAR_PITCH1_VERTICAL_ANGLE = 0.0;
  // spear_pitch2
  double SPEAR_PITCH2_NORMAL_ANGLE = 0.0;
  double SPEAR_PITCH2_VERTICAL_ANGLE = 0.0;

  // KFS回収の森林の順番
  std::vector<int> KFS_FOREST_NUMBER;
  // 内回り/外回りでKFS回収の判定に使う長方形中心の座標 [x, y, yaw]
  // yaw=0 のときは map 座標系に平行で、yaw を与えるとその分だけ長方形が回転する
  std::vector<std::vector<double>> INNER_COLLECT_KFS_CENTER_POS;
  std::vector<std::vector<double>> OUTER_COLLECT_KFS_CENTER_POS;
  // KFS回収判定用長方形のサイズ
  // width は長方形ローカル x 方向、height は長方形ローカル y 方向の長さ
  double COLLECT_KFS_HEIGHT = 1.2;
  double COLLECT_KFS_WIDTH = 1.2;
  // KFS回収時のオフセット（front_kfsかrear_kfsのうち、遠い方に適応する）
  double COLLECT_KFS_OFFSET = 0.0;
  // 範囲外へ出た後、収納用 yaw を送るまでの遅延時間 [s]
  double KFS_YAW_DELAY_TIME = 1.0;
  // auto_collect_kfs_task 内で実際にアクチュエータ指令を出すか
  bool ENABLE_AUTO_COLLECT_KFS_ACTUATOR = true;
  // コンストラクタ
  R1MainNode();

  // ========== コールバック関数・ヘルパー関数 =========
  std::function<void(const std_msgs::msg::Int32::SharedPtr)> create_mode_status_callback(
    PositionAxisInterface * axis, const std::string & actuator_name);
  std::function<void(const r1_msgs::msg::GpioInput::SharedPtr)> create_switch_status_callback(
    bool * switch_status);
  void register_position_axis(
    const std::string & name, double * position_ref_alias = nullptr,
    double * speed_ref_alias = nullptr, bool use_set_angle_topic = false);
  void register_velocity_axis(
    const std::string & name, const std::string & topic_name,
    double * velocity_ref_alias = nullptr);
  void register_gpio_pwm_output(
    const std::string & name, double * double_ref_alias = nullptr, bool * bool_ref_alias = nullptr);
  void register_gpio_servo_output(const std::string & name, int * ref_alias = nullptr);
  void register_gpio_input(const std::string & name, bool * switch_status);
  void publish_position_axis(const std::string & name, double pos);
  void set_position_axis(const std::string & name, double pos);
  void publish_position_axis_speed_ref(const std::string & name, double speed);
  void detect_origin_position_axis(const std::string & name);
  void stop_position_axis_speed_mode(const std::string & name);
  void move_mech_lock_position_axis(const std::string & name, int direction);
  void publish_velocity_axis(const std::string & name, double vel);
  void publish_gpio_pwm_output(const std::string & name, double ref);
  void publish_gpio_servo_output(const std::string & name, int ref);
  // joyのcallback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback(void);
  void declare_and_get_parameter(
    const std::string & name, double & value, double default_value = 0.0);
  void declare_and_get_parameter(
    const std::string & name, bool & value, bool default_value = false);
  void declare_and_get_parameter(const std::string & name, int & value, int default_value = 0);
  void declare_and_get_parameter(
    const std::string & name, std::string & value, const std::string & default_value = "");
  // sabacan
  void sabacan_power_ref(bool is_ems);
  void sabacan_led_ref(int pin_number, uint8_t r, uint8_t g, uint8_t b);
  void set_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s = 0.0);
  void set_fkfs_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s = 0.0);
  void set_rkfs_led_status(uint8_t r, uint8_t g, uint8_t b, double blink_period_s = 0.0);
  void clear_led_status(void);
  void update_kfs_led_status(void);
  void set_led_event(uint8_t r, uint8_t g, uint8_t b, double blink_period_s, double duration_sec);
  LedPattern resolve_base_led_pattern(void);
  LedColor resolve_led_output_color(const LedPattern & pattern, const rclcpp::Time & now) const;
  void publish_r1_machine_initialize(void);
  void r1_machine_initialize_done_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void invalidate_led_cache(void);
  void initialize_lidar_lifecycle_clients(void);
  void request_lidar_lifecycle_activation(void);
  void handle_lidar_activate_response(
    const std::string & node_name,
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);
  // 現在の状態に応じて、LEDを光らせる。
  void sabacan_led_update(void);
  // IMU
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void set_mecanum_yaw(double yaw);
  void set_swerve_drive_yaw(double yaw);
  void publish_chassis_act_stop(void);
  // オドメトリ
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void set_odometry(double x, double y, double yaw);
  void set_initialpose(double x, double y, double yaw, double delay_sec = 0.2);
  void schedule_initialpose_tf_log(void);
  void log_initialpose_tf_once(void);
  void log_transform_once(const std::string & target_frame, const std::string & source_frame);
  // chassis_act
  void chassis_act_status_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void publish_chassis_act_ref(ChassisAct ref);
  // robot_move
  void publish_robot_move(
    ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type);
  // arucoマーカ
  void publish_aruco_marker_id(int id);
  bool is_localization_ready(void);
  void request_auto_robot_move(
    ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type);
  void publish_pending_auto_robot_move_if_ready(void);
  void start_auto_chassis(
    ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type);
  void clear_auto_chassis_state(bool stop_kfs_auto_collect = false);
  geometry_msgs::msg::PoseStamped get_map_pos(void);
  bool build_inner_kfs_auto_collect_plan(
    std::vector<int> & forest_order, std::vector<std::string> & collect_kfs_type) const;
  bool build_outer_kfs_auto_collect_plan(
    std::vector<int> & forest_order, std::vector<std::string> & collect_kfs_type) const;
  bool set_mode3_kfs_auto_collect_status(KfsAutoCollectStatus & status);
  void start_kfs_auto_collect(
    KfsAutoCollectStatus status, std::vector<int> forest_order,
    std::vector<std::string> kfs_mechanism_type);
  void stop_kfs_auto_collect(void);
  void reset_kfs_auto_collect_tracking(void);
  // ========== 各アクチュエータ単体の動作関数 ==========
  bool chassis_rotate90 = false;
  // 足回り
  void chassis_move_vel(double vx, double vy, double omega);
  // KFS回収
  // 位置指令
  void kfs_fx_pos_ref(double pos);
  void kfs_fz_pos_ref(double pos);
  void kfs_fyaw_pos_ref(double pos);
  void kfs_rx_pos_ref(double pos);
  void kfs_rz_pos_ref(double pos);
  void kfs_ryaw_pos_ref(double pos);
  void kfs_fx_set_pos(double pos);
  void kfs_fz_set_pos(double pos);
  void kfs_fyaw_set_angle(double angle);
  void kfs_rx_set_pos(double pos);
  void kfs_rz_set_pos(double pos);
  void kfs_ryaw_set_angle(double angle);
  // 速度指令
  void kfs_fx_speed_ref(double speed);
  void kfs_fz_speed_ref(double speed);
  void kfs_fyaw_speed_ref(double speed);
  void kfs_rx_speed_ref(double speed);
  void kfs_rz_speed_ref(double speed);
  void kfs_ryaw_speed_ref(double speed);
  // 速度指令停止
  void kfs_fx_speed_mode_stop(void);
  void kfs_fz_speed_mode_stop(void);
  void kfs_fyaw_speed_mode_stop(void);
  void kfs_rx_speed_mode_stop(void);
  void kfs_rz_speed_mode_stop(void);
  void kfs_ryaw_speed_mode_stop(void);
  // R2昇降
  // 位置指令
  void r2_flift_pos_ref(double pos);
  void r2_rlift_pos_ref(double pos);
  void r2_flift_set_pos(double pos);
  void r2_rlift_set_pos(double pos);
  // 速度指令
  void r2_flift_speed_ref(double speed);
  void r2_rlift_speed_ref(double speed);
  // 速度指令停止
  void r2_flift_speed_mode_stop(void);
  void r2_rlift_speed_mode_stop(void);
  // やり
  // 位置指令
  void spear1_pos_ref(double pos);
  void spear2_pos_ref(double pos);
  void spear3_pos_ref(double pos);
  void spear4_pos_ref(double pos);
  void spear_x_pos_ref(double pos);
  void spear_y_pos_ref(double pos);
  void spear_roll_pos_ref(double angle);
  void spear_pitch1_pos_ref(double angle);
  void spear_pitch2_pos_ref(double angle);
  void spear1_set_pos(double pos);
  void spear2_set_pos(double pos);
  void spear3_set_pos(double pos);
  void spear4_set_pos(double pos);
  void spear_x_set_pos(double pos);
  void spear_y_set_pos(double pos);
  void spear_roll_set_angle(double angle);
  void spear_pitch1_set_angle(double angle);
  void spear_pitch2_set_angle(double angle);
  // 速度指令
  void spear1_speed_ref(double speed);
  void spear2_speed_ref(double speed);
  void spear3_speed_ref(double speed);
  void spear4_speed_ref(double speed);
  void spear_x_speed_ref(double speed);
  void spear_y_speed_ref(double speed);
  void spear_roll_speed_ref(double speed);
  void spear_pitch1_speed_ref(double speed);
  void spear_pitch2_speed_ref(double speed);
  // 速度指令停止
  void spear1_speed_mode_stop(void);
  void spear2_speed_mode_stop(void);
  void spear3_speed_mode_stop(void);
  void spear4_speed_mode_stop(void);
  void spear_x_speed_mode_stop(void);
  void spear_y_speed_mode_stop(void);
  void spear_roll_speed_mode_stop(void);
  void spear_pitch1_speed_mode_stop(void);
  void spear_pitch2_speed_mode_stop(void);
  // ========== move_mech_lock関数 ==========
  // KFS回収
  void kfs_fx_move_mech_lock(int direction);
  void kfs_fz_move_mech_lock(int direction);
  void kfs_fyaw_move_mech_lock(int direction);
  void kfs_rx_move_mech_lock(int direction);
  void kfs_rz_move_mech_lock(int direction);
  void kfs_ryaw_move_mech_lock(int direction);
  // fyawとryawはmech_lockの方向がわからなくなることが多いので関数を作る
  void kfs_fyaw_move_front_mech_lock(void);
  void kfs_fyaw_move_rear_mech_lock(void);
  void kfs_ryaw_move_front_mech_lock(void);
  void kfs_ryaw_move_rear_mech_lock(void);
  // R2昇降
  void r2_flift_move_mech_lock(int direction);
  void r2_rlift_move_mech_lock(int direction);
  void r2_flift_move_down_mech_lock(void);
  void r2_flift_move_up_mech_lock(void);
  void r2_rlift_move_down_mech_lock(void);
  void r2_rlift_move_up_mech_lock(void);
  // やり
  void spear1_move_mech_lock(int direction);
  void spear2_move_mech_lock(int direction);
  void spear3_move_mech_lock(int direction);
  void spear4_move_mech_lock(int direction);
  void spear_x_move_mech_lock(int direction);
  void spear_y_move_mech_lock(int direction);
  void spear_roll_move_mech_lock(int direction);
  void spear_pitch1_move_mech_lock(int direction);
  void spear_pitch2_move_mech_lock(int direction);
  // KFS真空ポンプ・電磁弁
  void kfs_front_pump(double pwm);
  void kfs_rear_pump(double pwm);
  void kfs_front_valve(bool on);
  void kfs_rear_valve(bool on);
  // やり電磁弁
  // 大槻機構
  void spear_hand1_valve(bool on);
  void spear_hand2_valve(bool on);
  void spear_hand_push_valve(bool on);
  // 千田機構
  void spear_u1_valve(bool on);
  void spear_d1_valve(bool on);
  void spear_u2_valve(bool on);
  void spear_d2_valve(bool on);

  // ========== 各動作の関数 ==========

  rclcpp::TimerBase::SharedPtr kfs_init_pos_roll_timer_;
  void kfs_init_pos(void);

  // 動いていたら危険なアクチュエータは停止する
  // 位置制御は止められないので、そのまま
  // TODO: 位置制御系も止められるようにする
  void stop_actuator(void);
  // ========== 原点検出関数 ==========
  // KFS回収
  void kfs_fx_detect_origin(void);
  void kfs_fz_detect_origin(void);
  void kfs_fyaw_detect_origin(void);
  void kfs_rx_detect_origin(void);
  void kfs_rz_detect_origin(void);
  void kfs_ryaw_detect_origin(void);
  // r2昇降
  void r2_flift_detect_origin(void);
  void r2_rlift_detect_origin(void);
  // やり
  void spear1_detect_origin(void);
  void spear2_detect_origin(void);
  void spear3_detect_origin(void);
  void spear4_detect_origin(void);
  void spear_x_detect_origin(void);
  void spear_y_detect_origin(void);
  void spear_roll_detect_origin(void);
  void spear_pitch1_detect_origin(void);
  void spear_pitch2_detect_origin(void);
  // ========== センサーの取得 ==========
  bool get_kfs_fz_low_switch_status(void) { return kfs_fz_low_switch_status_; }
  bool get_kfs_rz_low_switch_status(void) { return kfs_rz_low_switch_status_; }
  // ========== 各状態のタスク ==========
  void idle_task(void);
  void emergency_task(void);
  void manual_task(void);
  void update_auto_chassis_task(void);
  void main_task(void);
  // ========== テスト関数 ==========
  // テスト関数はここに追加する
  // ========== マニュアルモード ==========
  void manual_mode1_detect_origin(void);
  void manual_mode2_pole(void);
  void manual_mode2_collect_pole_task(void);
  void manual_mode3_spear(void);
  void manual_mode3_init_move_task(int n);
  void manual_mode3_make_spear_task(int n);
  void manual_mode4_fkfs(void);
  void manual_mode5_rkfs(void);
  void manual_mode6_r2_lift(void);
  void manual_mode7_spear_attack(void);
  void manual_mode7_spear_attack_task(int n, int m);
  void manual_mode7_spear_throw_away_task(int n);
  void manual_mode8_auto_collect_kfs(void);
  void manual_mode9_auto_chassis(void);
  static constexpr int DEFAULT_STEP = 1;
  int manual_mode2_collect_pole_task_step_ = DEFAULT_STEP;
  int manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  int manual_mode4_fx_step_ = DEFAULT_STEP;
  int manual_mode4_fz_step_ = DEFAULT_STEP;
  int manual_mode4_fyaw_step_ = DEFAULT_STEP;
  int manual_mode4_front_pump_step_ = DEFAULT_STEP;
  bool manual_mode4_r1_long_press_triger_ = false;
  int manual_mode5_rx_step_ = DEFAULT_STEP;
  int manual_mode5_rz_step_ = DEFAULT_STEP;
  int manual_mode5_ryaw_step_ = DEFAULT_STEP;
  int manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  bool manual_mode5_r1_long_press_triger_ = false;
  int manual_mode6_aruco_marker_step_ = DEFAULT_STEP;
  int manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  int manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  int manual_mode7_spear_throw_away_task_step_ = DEFAULT_STEP;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer1_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer2_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer3_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer4_;
  rclcpp::TimerBase::SharedPtr manual_mode4_front_valve_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode5_rear_valve_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode6_r2_lift_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode7_front_valve_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode7_rear_valve_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode8_roll_timer_;
  // ========== 自動シャーシ ==========
  void auto_collect_kfs_task(void);
  rclcpp::TimerBase::SharedPtr auto_collect_front_storage_yaw_timer_;
  rclcpp::TimerBase::SharedPtr auto_collect_rear_storage_yaw_timer_;
  KfsAutoCollectPlan kfs_auto_collect_plan_;
  AutoChassisStatus auto_chassis_status_ = ChassisAct::NONE;
  // [12][2]の2次元配列
  std::vector<std::vector<bool>> kfs_auto_collect_within_ =
    std::vector<std::vector<bool>>(12, std::vector<bool>(2, false));
  std::vector<std::vector<bool>> kfs_auto_collect_prev_within_ =
    std::vector<std::vector<bool>>(12, std::vector<bool>(2, false));
  bool pending_auto_robot_move_valid_ = false;
  r1_msgs::msg::RobotMove pending_auto_robot_move_;
  // ========== リセット ==========
  void reset_step(void);
  void reset_robot(bool is_start_zone);
  void reset_position(bool is_start_zone);
};
