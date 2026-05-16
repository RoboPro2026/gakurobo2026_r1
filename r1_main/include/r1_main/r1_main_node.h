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

#include <cstdarg>
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
#include "r1_msgs/msg/r1_collect_kfs.hpp"
#include "r1_msgs/msg/r1_init_parameter.hpp"
#include "r1_msgs/msg/robot_move.hpp"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sabacan_msgs/msg/sabacan_led_ref.hpp"
#include "sabacan_msgs/msg/sabacan_power_ref.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
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

  enum class R1KfsMechanismRef : int32_t
  {
    NONE = -1,
    FKFS_RACK = 0,
    FKFS_HIGH = 1,
    FKFS_MIDDLE = 2,
    FKFS_LOW = 3,
    FKFS_GROUND = 4,
    FKFS_STORAGE = 5,
    FKFS_COLLECT_START_POS = 6,
    RKFS_RACK = 10,
    RKFS_HIGH = 11,
    RKFS_MIDDLE = 12,
    RKFS_LOW = 13,
    RKFS_GROUND = 14,
    RKFS_STORAGE = 15,
    RKFS_COLLECT_START_POS = 16
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

  // ========== Scan (YDLidar) ==========
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_fh_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_fm_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_fl_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_rh_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_rm_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_rl_subscription_;
  double scan_fh_data_ = 0.0;
  double scan_fm_data_ = 0.0;
  double scan_fl_data_ = 0.0;
  double scan_rh_data_ = 0.0;
  double scan_rm_data_ = 0.0;
  double scan_rl_data_ = 0.0;
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
  // LiDAR起動完了後にも確実に届くよう、遅延を変えて再送するタイマー
  rclcpp::TimerBase::SharedPtr initialpose_retry1_timer_;
  rclcpp::TimerBase::SharedPtr initialpose_retry2_timer_;
  rclcpp::TimerBase::SharedPtr initialpose_tf_log_timer_;
  // chassis_actのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr chassis_act_ref_publisher_;
  // chassis_actのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_act_status_subscription_;
  // 接線方向PID補正ON/OFFのPublisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chassis_tangent_pid_enable_publisher_;
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
  double initialpose_retry1_delay_sec_ = 1.0;
  double initialpose_retry2_delay_sec_ = 3.0;
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
  bool is_act_paused_ = false;
  bool enable_right_stick_pause_ = false;
  // arucoマーカ
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aruco_marker_id_publisher_;
  // ========== スマホ関連 ==========
  // スマホから送られてくる初期化パラメータ
  rclcpp::Subscription<r1_msgs::msg::R1InitParameter>::SharedPtr r1_init_parameter_subscription_;
  r1_msgs::msg::R1InitParameter r1_init_parameter_;
  bool received_r1_init_parameter_ = false;
  // KFS回収の個別指定
  rclcpp::Subscription<r1_msgs::msg::R1CollectKfs>::SharedPtr r1_collect_kfs_subscription_;
  r1_msgs::msg::R1CollectKfs r1_collect_kfs_;
  bool received_r1_collect_kfs_ = false;
  // 回収機構を指定の位置に動かす
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_kfs_mechanism_ref_subscription_;
  int r1_kfs_mechanism_ref_ = static_cast<int>(R1KfsMechanismRef::NONE);
  // 3つ目のKFSを回収する
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_retry_collect_subscription_;
  // KFSの回収に失敗したので再回収する
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_collect_3rd_kfs_subscription_;
  int r1_collect_3rd_kfs_ = -1;
  // ロボットの全アクチュエータを初期化する
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r1_initialize_all_actuator_subscription_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr r1_operation_mode_publisher_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_log_message_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_log_message_info_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_log_message_warn_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r1_log_message_error_publisher_;
  OperationMode last_published_operation_mode_{OperationMode::MODE1_DETECT_ORIGIN};

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
  double r2_flift_position_ref_ = 0.0;
  double r2_rlift_position_ref_ = 0.0;

  double kfs_front_pump_ref_ = 0.0;
  double kfs_rear_pump_ref_ = 0.0;
  bool kfs_front_valve_ref_ = false;
  bool kfs_rear_valve_ref_ = false;

  // 大槻機構
  double spear_y_position_ref_ = 0.0;
  double spear_roll1_position_ref_ = 0.0;
  double spear_roll2_position_ref_ = 0.0;
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
  bool ENABLE_R2_ANALOG_SPEED_CONTROL = false;
  double CHASSIS_LOW_VELOCITY = 0.0;
  double CHASSIS_NORMAL_VELOCITY = 0.0;
  double CHASSIS_HIGH_VELOCITY = 0.0;
  double CHASSIS_LOW_OMEGA = 0.0;
  double CHASSIS_NORMAL_OMEGA = 0.0;
  double CHASSIS_HIGH_OMEGA = 0.0;
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
  double KFS_FX_GROUND_POS = 0.0;
  // fz
  double KFS_FZ_NORMAL_POS = 0.0;
  double KFS_FZ_LOW_POS = 0.0;
  double KFS_FZ_MIDDLE_POS = 0.0;
  double KFS_FZ_HIGH_POS = 0.0;
  double KFS_FZ_PUT_POS = 0.0;
  double KFS_FZ_STORAGE_POS = 0.0;
  double KFS_FZ_START_POS = 0.0;
  double KFS_FZ_LOW_MECH_LOCK_POS = 0.0;
  double KFS_FZ_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_FZ_R2_LIFT_POS = 0.0;
  double KFS_FZ_GROUND_POS = 0.0;
  // fyaw
  double KFS_FYAW_NORMAL_ANGLE = 0.0;
  double KFS_FYAW_FRONT_ANGLE = 0.0;
  double KFS_FYAW_SIDE_ANGLE = 0.0;
  double KFS_FYAW_REAR_ANGLE = 0.0;
  double KFS_FYAW_START_ANGLE = 0.0;
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
  double KFS_RX_GROUND_POS = 0.0;
  // rz
  double KFS_RZ_NORMAL_POS = 0.0;
  double KFS_RZ_LOW_POS = 0.0;
  double KFS_RZ_MIDDLE_POS = 0.0;
  double KFS_RZ_HIGH_POS = 0.0;
  double KFS_RZ_PUT_POS = 0.0;
  double KFS_RZ_STORAGE_POS = 0.0;
  double KFS_RZ_START_POS = 0.0;
  double KFS_RZ_LOW_MECH_LOCK_POS = 0.0;
  double KFS_RZ_HIGH_MECH_LOCK_POS = 0.0;
  double KFS_RZ_R2_LIFT_POS = 0.0;
  double KFS_RZ_GROUND_POS = 0.0;
  // ryaw
  double KFS_RYAW_NORMAL_ANGLE = 0.0;
  double KFS_RYAW_FRONT_ANGLE = 0.0;
  double KFS_RYAW_SIDE_ANGLE = 0.0;
  double KFS_RYAW_REAR_ANGLE = 0.0;
  double KFS_RYAW_START_ANGLE = 0.0;
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
  // spear y
  double SPEAR_Y_NORMAL_POS = 0.0;
  double SPEAR_Y_COLLECT1_POS = 0.0;
  double SPEAR_Y_COLLECT2_POS = 0.0;
  double SPEAR_Y_MAKE_SPEAR_POS = 0.0;
  double SPEAR_Y_LOW_ATTACK_POS = 0.0;
  double SPEAR_Y_MIDDLE_ATTACK_POS = 0.0;
  double SPEAR_Y_HIGH_ATTACK_POS = 0.0;
  double SPEAR_Y_THROW_AWAY_POS = 0.0;
  // spear roll1
  double SPEAR_ROLL1_NORMAL_ANGLE = 0.0;
  double SPEAR_ROLL1_VERTICAL_ANGLE = 0.0;
  double SPEAR_ROLL1_HORIZONTAL_ANGLE = 0.0;
  double SPEAR_ROLL1_INV_HORIZONTAL_ANGLE = 0.0;
  double SPEAR_ROLL1_LOW_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL1_MIDDLE_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL1_HIGH_ATTACK_ANGLE = 0.0;
  // spear roll2
  double SPEAR_ROLL2_NORMAL_ANGLE = 0.0;
  double SPEAR_ROLL2_VERTICAL_ANGLE = 0.0;
  double SPEAR_ROLL2_HORIZONTAL_ANGLE = 0.0;
  double SPEAR_ROLL2_INV_HORIZONTAL_ANGLE = 0.0;
  double SPEAR_ROLL2_LOW_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL2_MIDDLE_ATTACK_ANGLE = 0.0;
  double SPEAR_ROLL2_HIGH_ATTACK_ANGLE = 0.0;

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
  // KFS回収時に一度停止するか
  bool ENABLE_STOP_BEFORE_COLLECT_KFS = true;
  // auto_collect_kfs_task内で単眼Lidarを用いた回収を行うか。
  // trueのときは単眼Lidarを用いた回収、falseのときは座標による回収となる
  bool ENABLE_WALL_SENSOR = true;
  // 壁検出センサーの距離閾値 [m]
  double WALL_SENSOR_DISTANCE_THRESHOLD = 0.5;
  // 壁検出センサーの反応時間閾値 [s]
  double WALL_SENSOR_TIME_THRESHOLD = 0.15;
  // 壁検出後に回収動作をしながら移動する距離 [m]
  // その間は進行方向に進み続ける。
  double MOVE_DISTANCE_AFTER_WALL_DETECT = 0.3;
  // 壁の検出範囲のサイズ [m]
  double WALL_SENSOR_DETECT_HEIGHT = 0.5;
  double WALL_SENSOR_DETECT_WIDTH = 1.0;
  // 壁検出の遅延距離オフセット [m]
  double WALL_SENSOR_DELAY_OFFSET_DISTANCE = 0.25;
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
  // Scan (YDLidar)
  void register_scan(
    const std::string & topic_name,
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr & subscription, double & data);
  void set_mecanum_yaw(double yaw);
  void set_swerve_drive_yaw(double yaw);
  void publish_chassis_act_stop(void);
  void publish_chassis_act_pause(void);
  void publish_chassis_act_resume(void);
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
  // スマホ関連
  void r1_init_parameter_callback(const r1_msgs::msg::R1InitParameter::SharedPtr msg);
  void r1_collect_kfs_callback(const r1_msgs::msg::R1CollectKfs::SharedPtr msg);
  void r1_kfs_mechanism_ref_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void r1_retry_collect_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void r1_collect_3rd_kfs_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void r1_initialize_all_actuator_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // void publish_r1_log(const std::string & message);
  // スマホにログ出力する関数
  // よくわからないけど、__attribute__((format(printf, 2, 3)));をつけると、型ミスをしていたときにコンパイラが警告を出してくれるらしい
  void r1_log_info(const char * fmt, ...) __attribute__((format(printf, 2, 3)));
  void r1_log_warn(const char * fmt, ...) __attribute__((format(printf, 2, 3)));
  void r1_log_error(const char * fmt, ...) __attribute__((format(printf, 2, 3)));

  bool is_localization_ready(void);
  void request_auto_robot_move(
    ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type);
  void publish_pending_auto_robot_move_if_ready(void);
  void start_auto_chassis(
    ChassisAct act, std::vector<int> forest_order, std::vector<std::string> kfs_mechanism_type);
  void clear_auto_chassis_state(bool stop_kfs_auto_collect = false);
  geometry_msgs::msg::PoseStamped get_map_pos(void);
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
  void spear_y_pos_ref(double pos);
  void spear_roll1_pos_ref(double angle);
  void spear_roll2_pos_ref(double angle);
  // 速度指令
  void spear_y_speed_ref(double speed);
  void spear_roll1_speed_ref(double speed);
  void spear_roll2_speed_ref(double speed);
  // 速度指令停止
  void spear_y_speed_mode_stop(void);
  void spear_roll1_speed_mode_stop(void);
  void spear_roll2_speed_mode_stop(void);
  // 角度指定
  void spear_y_set_pos(double pos);
  void spear_roll1_set_angle(double angle);
  void spear_roll2_set_angle(double angle);

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
  void spear_y_move_mech_lock(int direction);
  void spear_roll1_move_mech_lock(int direction);
  void spear_roll2_move_mech_lock(int direction);
  // KFS真空ポンプ・電磁弁
  void kfs_front_pump(double pwm);
  void kfs_rear_pump(double pwm);
  void kfs_front_valve(bool on);
  void kfs_rear_valve(bool on);
  // やり電磁弁
  void spear_hand1_valve(bool on);
  void spear_hand2_valve(bool on);
  void spear_hand_push_valve(bool on);
  // ========== 各動作の関数 ==========

  void kfs_robot_start_act(void);
  rclcpp::TimerBase::SharedPtr kfs_collect_start_act_roll_timer_;
  rclcpp::TimerBase::SharedPtr kfs_collect_start_act_push_valve_timer_;
  void kfs_collect_start_act(bool enable_pump = false);

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
  void spear_y_detect_origin(void);
  void spear_roll1_detect_origin(void);
  void spear_roll2_detect_origin(void);

  void detect_origin_all_actuator(void);
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
  int manual_mode2_hand_valve_step_ = DEFAULT_STEP;
  int manual_mode2_push_valve_step_ = DEFAULT_STEP;
  int manual_mode3_make_spear_task_step_ = DEFAULT_STEP;
  int manual_mode3_hand_valve_step_ = DEFAULT_STEP;
  int manual_mode3_push_valve_step_ = DEFAULT_STEP;
  int manual_mode4_fx_step_ = DEFAULT_STEP;
  int manual_mode4_fz_step_ = DEFAULT_STEP;
  int manual_mode4_fyaw_step_ = DEFAULT_STEP;
  int manual_mode4_front_pump_step_ = DEFAULT_STEP;
  int manual_mode5_rx_step_ = DEFAULT_STEP;
  int manual_mode5_rz_step_ = DEFAULT_STEP;
  int manual_mode5_ryaw_step_ = DEFAULT_STEP;
  int manual_mode5_rear_pump_step_ = DEFAULT_STEP;
  int manual_mode6_aruco_marker_step_ = DEFAULT_STEP;
  int manual_mode6_r2_lift_step_ = DEFAULT_STEP;
  int manual_mode7_spear_attack_task_step_ = DEFAULT_STEP;
  int manual_mode7_spear_throw_away_task_step_ = DEFAULT_STEP;
  int manual_mode7_hand_valve_step_ = DEFAULT_STEP;
  int manual_mode7_push_valve_step_ = DEFAULT_STEP;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer1_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer2_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer3_;
  rclcpp::TimerBase::SharedPtr manual_mode3_timer4_;
  rclcpp::TimerBase::SharedPtr manual_mode3_roll_timer_;
  rclcpp::TimerBase::SharedPtr manual_mode3_push_valve_timer_;
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
  // 要素数12の配列
  std::vector<rclcpp::Time> wall_sensor_detect_start_time_ =
    std::vector<rclcpp::Time>(12, rclcpp::Time(0));
  std::vector<bool> wall_sensor_detected_ = std::vector<bool>(12, false);
  // 壁検出の距離は近距離なので、map座標系ではなく、odom座標系で処理を行う
  // ただし、おおよその壁の位置の探索のみmap座標系で行う
  std::vector<nav_msgs::msg::Odometry> wall_detect_pos_ = std::vector<nav_msgs::msg::Odometry>(12);
  // 最後に自動回収を行った時刻
  std::vector<rclcpp::Time> last_auto_collect_kfs_time_ =
    std::vector<rclcpp::Time>(2, rclcpp::Time(0));
  // 回収完了済みフラグ（チャタリング防止）: forest番号-1 をインデックスとする
  std::vector<bool> kfs_already_collected_ = std::vector<bool>(12, false);
  int auto_collect_kfs_fkfs_step_ = DEFAULT_STEP;
  int auto_collect_kfs_rkfs_step_ = DEFAULT_STEP;

  bool pending_auto_robot_move_valid_ = false;
  r1_msgs::msg::RobotMove pending_auto_robot_move_;
  // ========== リセット ==========
  void reset_step(void);
  void reset_robot(bool is_start_zone);
  void reset_position(bool is_start_zone);
};