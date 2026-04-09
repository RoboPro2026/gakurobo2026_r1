/**
 * @file r1_linear_motion_node.cpp
 * @author Yamaguchi Yudai
 * @brief 
 * @version 0.1
 * @date 2025-11-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>
#include <cmath>

#include "r1_msgs/msg/gpio_input.hpp"
#include "r1_msgs/msg/linear_motion.hpp"
#include "r1_msgs/msg/motor_ref.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_linear_motion_node")
  {
    linear_motion_status_subscription_ = this->create_subscription<r1_msgs::msg::LinearMotion>(
      "/linear_motion_status", 10,
      std::bind(&MyNode::linear_motion_status_callback, this, std::placeholders::_1));

    low_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
      "/low_switch_status", 10,
      std::bind(&MyNode::low_switch_status_callback, this, std::placeholders::_1));

    high_switch_status_subscription_ = this->create_subscription<r1_msgs::msg::GpioInput>(
      "/high_switch_status", 10,
      std::bind(&MyNode::high_switch_status_callback, this, std::placeholders::_1));

    linear_motion_ref_publisher_ =
      this->create_publisher<r1_msgs::msg::MotorRef>("/linear_motion_motor_ref", 10);

    position_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/linear_motion_position_ref", 10,
      std::bind(&MyNode::positon_ref_callback, this, std::placeholders::_1));

    speed_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/linear_motion_speed_ref", 10,
      std::bind(&MyNode::speed_ref_callback, this, std::placeholders::_1));

    stop_speed_mode_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/linear_motion_speed_mode_stop", 10,
      std::bind(&MyNode::stop_speed_mode_callback, this, std::placeholders::_1));

    detect_origin_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/linear_motion_detect_origin", 10,
      std::bind(&MyNode::detect_origin_callback, this, std::placeholders::_1));

    move_mech_lock_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/linear_motion_move_mech_lock", 10,
      std::bind(&MyNode::move_mech_lock_callback, this, std::placeholders::_1));

    initialize_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/linear_motion_initialize", 10,
      std::bind(&MyNode::initialize_callback, this, std::placeholders::_1));

    mode_status_publisher_ =
      this->create_publisher<std_msgs::msg::Int32>("/linear_motion_mode_status", 10);
    rclcpp::QoS torque_limit_qos(1);
    torque_limit_qos.reliable();
    torque_limit_qos.transient_local();
    torque_limit_ref_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/linear_motion_torque_limit_ref", torque_limit_qos);

    parameter_callback_handler_ = this->add_on_set_parameters_callback(
      std::bind(&MyNode::parameter_callback, this, std::placeholders::_1));

    this->declare_parameter("timer_rate", 100.0);
    this->declare_parameter("use_low_switch", true);
    this->declare_parameter("use_high_switch", true);
    this->declare_parameter("torque_threshold", 1.0);              // Nm
    this->declare_parameter("normal_torque_limit", 1.0);           // Nm
    this->declare_parameter("contact_torque_limit", 1.0);          // Nm
    this->declare_parameter("origin_detect_threshold_time", 0.1);  // s
    // 原点検出時の速度は負の符号でも可、原点としたい方向に回転させる
    this->declare_parameter("origin_detect_speed", -3.14);  // rad/s
    this->declare_parameter("move_mech_lock_speed", 3.14);  // rad/s
    this->declare_parameter("pos_min", 0.0);                // m
    this->declare_parameter("pos_max", 1.0);                // m
    this->declare_parameter("normal_pos", 0.05);
    this->declare_parameter("radius", 0.05);  // m
    this->declare_parameter("inverse_motor", false);
    this->declare_parameter("inverse_low_switch_logic", false);
    this->declare_parameter("inverse_high_switch_logic", false);

    this->get_parameter("timer_rate", timer_rate_);
    this->get_parameter("use_low_switch", use_low_switch_);
    this->get_parameter("use_high_switch", use_high_switch_);
    this->get_parameter("torque_threshold", torque_threshold_);
    this->get_parameter("normal_torque_limit", normal_torque_limit_);
    this->get_parameter("contact_torque_limit", contact_torque_limit_);
    this->get_parameter("origin_detect_threshold_time", origin_detect_threshold_time_);
    this->get_parameter("origin_detect_speed", origin_detect_speed_);
    this->get_parameter("move_mech_lock_speed", move_mech_lock_speed_);
    this->get_parameter("pos_min", pos_min_);
    this->get_parameter("pos_max", pos_max_);
    this->get_parameter("normal_pos", normal_pos_);
    this->get_parameter("radius", radius_);
    bool inverse_motor;
    this->get_parameter("inverse_motor", inverse_motor);
    motor_dir_ = inverse_motor ? -1.0 : 1.0;
    this->get_parameter("inverse_low_switch_logic", inverse_low_switch_logic_);
    this->get_parameter("inverse_high_switch_logic", inverse_high_switch_logic_);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&MyNode::timer_callback, this));
    publish_active_torque_limit();
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "timer_rate") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "timer_rate must be greater than 0.0";
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          continue;
        }
        timer_rate_ = parameter.as_double();
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / timer_rate_),
          std::bind(&MyNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Updated parameter: timer_rate = %.3f", timer_rate_);
      } else if (name == "use_low_switch") {
        use_low_switch_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_low_switch = %s",
          use_low_switch_ ? "true" : "false");
      } else if (name == "use_high_switch") {
        use_high_switch_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: use_high_switch = %s",
          use_high_switch_ ? "true" : "false");
      } else if (name == "torque_threshold") {
        torque_threshold_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: torque_threshold = %.3f", torque_threshold_);
      } else if (name == "normal_torque_limit") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "normal_torque_limit must be greater than or equal to 0.0";
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          continue;
        }
        normal_torque_limit_ = parameter.as_double();
        publish_active_torque_limit();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: normal_torque_limit = %.3f",
          normal_torque_limit_);
      } else if (name == "contact_torque_limit") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "contact_torque_limit must be greater than or equal to 0.0";
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          continue;
        }
        contact_torque_limit_ = parameter.as_double();
        publish_active_torque_limit();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: contact_torque_limit = %.3f",
          contact_torque_limit_);
      } else if (name == "origin_detect_threshold_time") {
        origin_detect_threshold_time_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: origin_detect_threshold_time = %.3f",
          origin_detect_threshold_time_);
      } else if (name == "origin_detect_speed") {
        origin_detect_speed_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: origin_detect_speed = %.3f",
          origin_detect_speed_);
      } else if (name == "move_mech_lock_speed") {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = "move_mech_lock_speed must be greater than or equal to 0.0";
          RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          continue;
        }
        move_mech_lock_speed_ = parameter.as_double();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: move_mech_lock_speed = %.3f",
          move_mech_lock_speed_);
      } else if (name == "pos_min") {
        pos_min_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_min = %.3f", pos_min_);
      } else if (name == "pos_max") {
        pos_max_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: pos_max = %.3f", pos_max_);
      } else if (name == "normal_pos") {
        normal_pos_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: normal_pos = %.3f", normal_pos_);
      } else if (name == "radius") {
        radius_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated parameter: radius = %.3f", radius_);
      } else if (name == "inverse_motor") {
        bool inverse_motor = parameter.as_bool();
        motor_dir_ = inverse_motor ? -1.0 : 1.0;
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_motor = %s",
          inverse_motor ? "true" : "false");
      } else if (name == "inverse_low_switch_logic") {
        inverse_low_switch_logic_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_low_switch_logic = %s",
          inverse_low_switch_logic_ ? "true" : "false");
      } else if (name == "inverse_high_switch_logic") {
        inverse_high_switch_logic_ = parameter.as_bool();
        RCLCPP_INFO(
          this->get_logger(), "Updated parameter: inverse_high_switch_logic = %s",
          inverse_high_switch_logic_ ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Invalid parameter name: " + name;
        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
      }
    }

    return result;
  }

  void linear_motion_status_callback(const r1_msgs::msg::LinearMotion::SharedPtr msg)
  {
    current_torque_ = msg->torque;
    current_speed_ = msg->speed;
    current_pos_ = msg->pos;
  }

  void low_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
  {
    low_switch_ = msg->status ^ inverse_low_switch_logic_;
  }

  void high_switch_status_callback(const r1_msgs::msg::GpioInput::SharedPtr msg)
  {
    high_switch_ = msg->status ^ inverse_high_switch_logic_;
  }

  void positon_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double target_pos;

    if (mode_ == MODE_SPEED) {
      RCLCPP_ERROR(this->get_logger(), "Currently in speed mode, position ref ignored.");
      return;
    }

    // 範囲内に収める
    if (msg->data < pos_min_) {
      target_pos = pos_min_;
      RCLCPP_WARN(
        this->get_logger(), "Target position below minimum. Clamping to %.3f", target_pos);
    } else if (msg->data > pos_max_) {
      target_pos = pos_max_;
      RCLCPP_WARN(
        this->get_logger(), "Target position above maximum. Clamping to %.3f", target_pos);
    } else {
      target_pos = msg->data;
    }

    auto motor_msg = r1_msgs::msg::MotorRef();
    motor_msg.control_type = "POSITION";
    motor_msg.ref = (motor_dir_ * target_pos + pos_offset_) / radius_;
    linear_motion_ref_publisher_->publish(motor_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing motor position ref: %.3f", motor_msg.ref);
  }

  void detect_origin_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      mode_ = MODE_SPEED;
      speed_mode_reason_ = SPEED_MODE_ORIGIN_DETECTION;
      reset_speed_mode_detection_timestamps();
      publish_active_torque_limit();
      RCLCPP_INFO(this->get_logger(), "Switched to speed control mode.");
    } else {
      stop_and_hold_current_position("Origin detection canceled. Holding current position.");
    }
  }

  void move_mech_lock_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 0) {
      stop_and_hold_current_position("Stopped mech lock move and holding current position.");
      return;
    }

    mode_ = MODE_SPEED;
    speed_mode_reason_ = SPEED_MODE_MECH_LOCK;
    mech_lock_direction_ = (msg->data > 0) ? 1.0 : -1.0;
    reset_speed_mode_detection_timestamps();
    publish_active_torque_limit();
    RCLCPP_INFO(
      this->get_logger(), "Switched to speed control mode for mech lock move. direction = %.0f",
      mech_lock_direction_);
  }

  void speed_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    mode_ = MODE_SPEED;
    speed_mode_reason_ = SPEED_MODE_USER_COMMAND;
    user_speed_ref_ = msg->data;
    reset_speed_mode_detection_timestamps();
    publish_active_torque_limit();
    RCLCPP_INFO(
      this->get_logger(), "Switched to speed control mode for user command. speed = %.3f",
      user_speed_ref_);
  }

  void stop_speed_mode_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (mode_ != MODE_SPEED) {
      RCLCPP_INFO(this->get_logger(), "Speed mode stop requested while already in position mode.");
      return;
    }

    stop_and_hold_current_position("Stopped speed mode and holding current position.");
  }

  void initialize_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    mode_ = MODE_POSITON;
    speed_mode_reason_ = SPEED_MODE_NONE;
    pos_offset_ = radius_ * current_pos_;
    publish_active_torque_limit();
    publish_hold_current_position();
    RCLCPP_INFO(
      this->get_logger(),
      "Received initialize signal. Updated position offset to %.6f so current position becomes zero.",
      pos_offset_);
  }

  double active_torque_limit() const
  {
    return speed_mode_reason_ == SPEED_MODE_ORIGIN_DETECTION ? contact_torque_limit_
                                                             : normal_torque_limit_;
  }

  void publish_active_torque_limit()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = active_torque_limit();
    torque_limit_ref_publisher_->publish(msg);
  }

  void reset_speed_mode_detection_timestamps()
  {
    last_normal_torque_time_ = this->now();
    last_low_switch_not_detect_time_ = this->now();
    last_high_switch_not_detect_time_ = this->now();
  }

  void publish_hold_current_position()
  {
    auto motor_ref_msg = r1_msgs::msg::MotorRef();
    motor_ref_msg.control_type = "POSITION";
    motor_ref_msg.ref = current_pos_;
    linear_motion_ref_publisher_->publish(motor_ref_msg);
  }

  void stop_and_hold_current_position(const char * log_message)
  {
    mode_ = MODE_POSITON;
    speed_mode_reason_ = SPEED_MODE_NONE;
    publish_active_torque_limit();
    publish_hold_current_position();
    RCLCPP_INFO(this->get_logger(), "%s", log_message);
  }

  bool is_contact_detection_enabled_in_speed_mode() const
  {
    return speed_mode_reason_ != SPEED_MODE_USER_COMMAND;
  }

  void timer_callback()
  {
    if (mode_ == MODE_SPEED) {
      bool detect_stop = false;
      if (is_contact_detection_enabled_in_speed_mode()) {
        // 現在のトルクがしきい値以下のとき
        if (std::abs(current_torque_) <= torque_threshold_) {
          // 最後に通常のトルクを検出した時刻を更新
          last_normal_torque_time_ = this->now();
        }
        // リミットスイッチが反応していないとき
        if (use_low_switch_ && low_switch_ == false) {
          // 最後にリミットスイッチが反応していない時刻を更新
          last_low_switch_not_detect_time_ = this->now();
        }
        if (use_high_switch_ && high_switch_ == false) {
          // 最後にリミットスイッチが反応していない時刻を更新
          last_high_switch_not_detect_time_ = this->now();
        }

        // 一定時間トルクのしきい値を超えた場合、原点検出とみなす
        detect_stop |=
          ((this->now() - last_normal_torque_time_).seconds() > origin_detect_threshold_time_);
        // 一定時間リミットスイッチが反応した場合、原点検出とみなす
        detect_stop |=
          (use_low_switch_ && (this->now() - last_low_switch_not_detect_time_).seconds() >
                                origin_detect_threshold_time_);
        detect_stop |=
          (use_high_switch_ && (this->now() - last_high_switch_not_detect_time_).seconds() >
                                 origin_detect_threshold_time_);
      }

      auto motor_ref_msg = r1_msgs::msg::MotorRef();
      if (detect_stop) {
        mode_ = MODE_POSITON;
        if (speed_mode_reason_ == SPEED_MODE_ORIGIN_DETECTION) {
          pos_offset_ = radius_ * current_pos_;
          RCLCPP_INFO(this->get_logger(), "Origin detected at position: %.3f", pos_offset_);
          motor_ref_msg.control_type = "POSITION";
          motor_ref_msg.ref = (motor_dir_ * normal_pos_ + pos_offset_) / radius_;
          RCLCPP_INFO(this->get_logger(), "Moving to normal position: %.3f", normal_pos_);
        } else {
          motor_ref_msg.control_type = "POSITION";
          motor_ref_msg.ref = current_pos_;
          if (speed_mode_reason_ == SPEED_MODE_MECH_LOCK) {
            RCLCPP_INFO(
              this->get_logger(), "Detected mechanical lock. Holding current position: %.3f",
              motor_ref_msg.ref);
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "User speed mode stopped by torque or limit switch detection. Holding current position: %.3f",
              motor_ref_msg.ref);
          }
        }
        speed_mode_reason_ = SPEED_MODE_NONE;
        publish_active_torque_limit();
      } else {
        motor_ref_msg.control_type = "VELOCITY";
        if (speed_mode_reason_ == SPEED_MODE_ORIGIN_DETECTION) {
          motor_ref_msg.ref = motor_dir_ * origin_detect_speed_;
        } else if (speed_mode_reason_ == SPEED_MODE_MECH_LOCK) {
          motor_ref_msg.ref = motor_dir_ * mech_lock_direction_ * move_mech_lock_speed_;
        } else {
          motor_ref_msg.ref = motor_dir_ * user_speed_ref_;
        }
      }
      linear_motion_ref_publisher_->publish(motor_ref_msg);
      // RCLCPP_INFO(
      //   this->get_logger(), "Publishing %s ref: %.3f", motor_ref_msg.control_type.c_str(),
      //   motor_ref_msg.ref);
    }
    // モードをPublish
    auto mode_msg = std_msgs::msg::Int32();
    mode_msg.data = mode_;
    mode_status_publisher_->publish(mode_msg);
  }

private:
  rclcpp::Subscription<r1_msgs::msg::LinearMotion>::SharedPtr linear_motion_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr low_switch_status_subscription_;
  rclcpp::Subscription<r1_msgs::msg::GpioInput>::SharedPtr high_switch_status_subscription_;
  rclcpp::Publisher<r1_msgs::msg::MotorRef>::SharedPtr linear_motion_ref_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_ref_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_ref_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_speed_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detect_origin_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr move_mech_lock_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr initialize_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_limit_ref_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handler_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_normal_torque_time_ = rclcpp::Time(0);
  rclcpp::Time last_low_switch_not_detect_time_ = rclcpp::Time(0);
  rclcpp::Time last_high_switch_not_detect_time_ = rclcpp::Time(0);
  bool use_low_switch_ = true;
  bool use_high_switch_ = true;
  double origin_detect_speed_ = 0.0;
  double move_mech_lock_speed_ = 0.0;
  double torque_threshold_ = 0.0;
  double normal_torque_limit_ = 1.0;
  double contact_torque_limit_ = 1.0;
  double origin_detect_threshold_time_ = 0.0;
  double motor_dir_ = 1.0;
  double mech_lock_direction_ = 1.0;
  double pos_min_ = 0.0;
  double pos_max_ = 1.0;
  double normal_pos_ = 0.0;
  // オフセット補正用
  double pos_offset_ = 0.0;
  bool low_switch_ = false;
  bool high_switch_ = false;
  bool inverse_low_switch_logic_ = false;
  bool inverse_high_switch_logic_ = false;
  double timer_rate_ = 100.0;
  double current_torque_ = 0.0;
  double current_speed_ = 0.0;
  double current_pos_ = 0.0;
  double user_speed_ref_ = 0.0;
  double radius_ = 0.05;  // m、値は適当
  static constexpr int MODE_POSITON = 0;
  static constexpr int MODE_SPEED = 1;
  static constexpr int SPEED_MODE_NONE = 0;
  static constexpr int SPEED_MODE_ORIGIN_DETECTION = 1;
  static constexpr int SPEED_MODE_MECH_LOCK = 2;
  static constexpr int SPEED_MODE_USER_COMMAND = 3;
  int mode_ = MODE_POSITON;
  int speed_mode_reason_ = SPEED_MODE_NONE;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
