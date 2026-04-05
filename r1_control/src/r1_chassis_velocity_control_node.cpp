/**
 * @file r1_chassis_velocity_control_node.cpp
 * @brief /input_cmd_vel と /odometry から /output_cmd_vel を生成する車体速度 PID ノード
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

class R1ChassisVelocityControllerNode : public rclcpp::Node
{
public:
  static constexpr std::size_t AXIS_VX = 0;
  static constexpr std::size_t AXIS_VY = 1;
  static constexpr std::size_t AXIS_WZ = 2;
  static constexpr std::size_t AXIS_COUNT = 3;

  struct AxisConfig
  {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 0.5;
    double output_limit = 2.5;
  };

  struct AxisState
  {
    double integral = 0.0;
    double prev_error = 0.0;
  };

  R1ChassisVelocityControllerNode() : Node("r1_chassis_velocity_control_node")
  {
    declare_parameters();
    load_parameters();

    input_cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic_, 10,
      std::bind(&R1ChassisVelocityControllerNode::input_cmd_vel_callback, this, std::placeholders::_1));
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10,
      std::bind(&R1ChassisVelocityControllerNode::odometry_callback, this, std::placeholders::_1));
    initialize_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "/chassis_velocity_control_initialize", 10,
      std::bind(&R1ChassisVelocityControllerNode::initialize_callback, this, std::placeholders::_1));
    output_cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&R1ChassisVelocityControllerNode::parameter_callback, this, std::placeholders::_1));

    recreate_control_timer();
    reset_controller_state();
  }

private:
  void declare_parameters()
  {
    this->declare_parameter("enable_velocity_pid", false);
    this->declare_parameter("control_rate", 100.0);
    this->declare_parameter("input_cmd_vel_topic", std::string("/cmd_vel_target"));
    this->declare_parameter("output_cmd_vel_topic", std::string("/cmd_vel"));

    declare_axis_parameters("vx", axis_config_[AXIS_VX], 2.5);
    declare_axis_parameters("vy", axis_config_[AXIS_VY], 2.5);
    declare_axis_parameters("omega", axis_config_[AXIS_WZ], 2.5);
  }

  void declare_axis_parameters(
    const std::string & suffix, const AxisConfig & defaults, double default_output_limit)
  {
    this->declare_parameter("kp_" + suffix, defaults.kp);
    this->declare_parameter("ki_" + suffix, defaults.ki);
    this->declare_parameter("kd_" + suffix, defaults.kd);
    this->declare_parameter("integral_limit_" + suffix, defaults.integral_limit);
    this->declare_parameter("output_limit_" + suffix, default_output_limit);
  }

  void load_parameters()
  {
    this->get_parameter("enable_velocity_pid", enable_velocity_pid_);
    this->get_parameter("control_rate", control_rate_);
    this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
    this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);

    load_axis_parameters("vx", axis_config_[AXIS_VX]);
    load_axis_parameters("vy", axis_config_[AXIS_VY]);
    load_axis_parameters("omega", axis_config_[AXIS_WZ]);
  }

  void load_axis_parameters(const std::string & suffix, AxisConfig & config)
  {
    this->get_parameter("kp_" + suffix, config.kp);
    this->get_parameter("ki_" + suffix, config.ki);
    this->get_parameter("kd_" + suffix, config.kd);
    this->get_parameter("integral_limit_" + suffix, config.integral_limit);
    this->get_parameter("output_limit_" + suffix, config.output_limit);
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto new_enable_velocity_pid = enable_velocity_pid_;
    auto new_control_rate = control_rate_;
    auto new_axis_config = axis_config_;
    bool need_recreate_timer = false;

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "enable_velocity_pid") {
        new_enable_velocity_pid = parameter.as_bool();
      } else if (name == "control_rate") {
        if (parameter.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "control_rate must be greater than 0.0";
          return result;
        }
        new_control_rate = parameter.as_double();
        need_recreate_timer = true;
      } else if (name == "input_cmd_vel_topic" || name == "output_cmd_vel_topic") {
        result.successful = false;
        result.reason = name + " cannot be changed at runtime";
        return result;
      } else if (!update_axis_parameter(name, parameter, new_axis_config, result)) {
        return result;
      }
    }

    enable_velocity_pid_ = new_enable_velocity_pid;
    control_rate_ = new_control_rate;
    axis_config_ = new_axis_config;

    if (!enable_velocity_pid_) {
      reset_controller_state();
    }
    if (need_recreate_timer) {
      recreate_control_timer();
    }
    return result;
  }

  bool update_axis_parameter(
    const std::string & name, const rclcpp::Parameter & parameter,
    std::array<AxisConfig, AXIS_COUNT> & axis_config,
    rcl_interfaces::msg::SetParametersResult & result)
  {
    const auto set_axis_value = [&](std::size_t axis_index, double AxisConfig::* member) {
        axis_config[axis_index].*member = parameter.as_double();
        return true;
      };
    const auto set_non_negative_axis_value = [&](std::size_t axis_index, double AxisConfig::* member,
                                                 const std::string & message) {
        if (parameter.as_double() < 0.0) {
          result.successful = false;
          result.reason = message;
          return false;
        }
        axis_config[axis_index].*member = parameter.as_double();
        return true;
      };

    if (name == "kp_vx") return set_axis_value(AXIS_VX, &AxisConfig::kp);
    if (name == "ki_vx") return set_axis_value(AXIS_VX, &AxisConfig::ki);
    if (name == "kd_vx") return set_axis_value(AXIS_VX, &AxisConfig::kd);
    if (name == "integral_limit_vx") {
      return set_non_negative_axis_value(
        AXIS_VX, &AxisConfig::integral_limit, "integral_limit_vx must be non-negative");
    }
    if (name == "output_limit_vx") {
      return set_non_negative_axis_value(
        AXIS_VX, &AxisConfig::output_limit, "output_limit_vx must be non-negative");
    }

    if (name == "kp_vy") return set_axis_value(AXIS_VY, &AxisConfig::kp);
    if (name == "ki_vy") return set_axis_value(AXIS_VY, &AxisConfig::ki);
    if (name == "kd_vy") return set_axis_value(AXIS_VY, &AxisConfig::kd);
    if (name == "integral_limit_vy") {
      return set_non_negative_axis_value(
        AXIS_VY, &AxisConfig::integral_limit, "integral_limit_vy must be non-negative");
    }
    if (name == "output_limit_vy") {
      return set_non_negative_axis_value(
        AXIS_VY, &AxisConfig::output_limit, "output_limit_vy must be non-negative");
    }

    if (name == "kp_omega") return set_axis_value(AXIS_WZ, &AxisConfig::kp);
    if (name == "ki_omega") return set_axis_value(AXIS_WZ, &AxisConfig::ki);
    if (name == "kd_omega") return set_axis_value(AXIS_WZ, &AxisConfig::kd);
    if (name == "integral_limit_omega") {
      return set_non_negative_axis_value(
        AXIS_WZ, &AxisConfig::integral_limit, "integral_limit_omega must be non-negative");
    }
    if (name == "output_limit_omega") {
      return set_non_negative_axis_value(
        AXIS_WZ, &AxisConfig::output_limit, "output_limit_omega must be non-negative");
    }

    result.successful = false;
    result.reason = "Invalid parameter name: " + name;
    return false;
  }

  void input_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_cmd_vel_ = *msg;
    has_target_cmd_vel_ = true;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    measured_cmd_vel_ = msg->twist.twist;
    has_odometry_ = true;
  }

  void initialize_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
    reset_controller_state();
  }

  void control_timer_callback()
  {
    const auto now = this->now();
    const double dt = compute_dt(now);

    if (!has_target_cmd_vel_) {
      return;
    }

    if (!enable_velocity_pid_) {
      reset_controller_state();
      output_cmd_vel_publisher_->publish(target_cmd_vel_);
      return;
    }

    if (!has_odometry_) {
      reset_controller_state();
      output_cmd_vel_publisher_->publish(target_cmd_vel_);
      return;
    }

    geometry_msgs::msg::Twist corrected_cmd_vel = target_cmd_vel_;
    corrected_cmd_vel.linear.x =
      compute_control_output(AXIS_VX, target_cmd_vel_.linear.x, measured_cmd_vel_.linear.x, dt);
    corrected_cmd_vel.linear.y =
      compute_control_output(AXIS_VY, target_cmd_vel_.linear.y, measured_cmd_vel_.linear.y, dt);
    corrected_cmd_vel.angular.z =
      compute_control_output(AXIS_WZ, target_cmd_vel_.angular.z, measured_cmd_vel_.angular.z, dt);

    output_cmd_vel_publisher_->publish(corrected_cmd_vel);
  }

  double compute_dt(const rclcpp::Time & now)
  {
    const double default_dt = 1.0 / control_rate_;
    if (last_control_time_.nanoseconds() == 0) {
      last_control_time_ = now;
      return default_dt;
    }

    const double dt = (now - last_control_time_).seconds();
    last_control_time_ = now;
    if (dt <= 0.0) {
      return default_dt;
    }
    return dt;
  }

  double compute_control_output(std::size_t axis, double reference, double measured, double dt)
  {
    const auto & config = axis_config_[axis];
    auto & state = axis_state_[axis];

    const double error = reference - measured;
    state.integral += error * dt;
    state.integral = std::clamp(state.integral, -config.integral_limit, config.integral_limit);

    double derivative = 0.0;
    if (dt > 0.0) {
      derivative = (error - state.prev_error) / dt;
    }
    state.prev_error = error;

    double output =
      reference + config.kp * error + config.ki * state.integral + config.kd * derivative;
    output = std::clamp(output, -config.output_limit, config.output_limit);
    return output;
  }

  void reset_controller_state()
  {
    for (auto & axis_state : axis_state_) {
      axis_state.integral = 0.0;
      axis_state.prev_error = 0.0;
    }
    last_control_time_ = this->now();
  }

  void recreate_control_timer()
  {
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&R1ChassisVelocityControllerNode::control_timer_callback, this));
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_cmd_vel_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr initialize_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  geometry_msgs::msg::Twist target_cmd_vel_;
  geometry_msgs::msg::Twist measured_cmd_vel_;
  bool has_target_cmd_vel_ = false;
  bool has_odometry_ = false;

  bool enable_velocity_pid_ = false;
  double control_rate_ = 100.0;
  std::string input_cmd_vel_topic_ = "/cmd_vel_target";
  std::string output_cmd_vel_topic_ = "/cmd_vel";

  std::array<AxisConfig, AXIS_COUNT> axis_config_;
  std::array<AxisState, AXIS_COUNT> axis_state_;

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R1ChassisVelocityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
