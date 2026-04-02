/**
 * @file r1_dummy_odometry_node.cpp
 * @author Yudai Yamaguchi
 * @brief 指令値に追従するオドメトリのダミーノード
 * @version 0.1
 * @date 2026-02-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_dummy_odometry_node")
  {
    this->declare_parameter<double>("timer_rate", 100.0);
    this->declare_parameter<double>("tau", 0.15);
    this->declare_parameter<double>("mass", 30.0);
    this->declare_parameter<double>("yaw_inertia", 1.8);
    this->declare_parameter<double>("linear_velocity_response_gain", 6.0);
    this->declare_parameter<double>("angular_velocity_response_gain", 8.0);
    this->declare_parameter<double>("max_force_x", 120.0);
    this->declare_parameter<double>("max_force_y", 120.0);
    this->declare_parameter<double>("max_torque_z", 6.0);
    this->declare_parameter<double>("max_jerk_x", 40.0);
    this->declare_parameter<double>("max_jerk_y", 40.0);
    this->declare_parameter<double>("max_jerk_z", 30.0);
    this->declare_parameter<bool>("use_path_tracking", true);
    this->declare_parameter<bool>("enable_target_pose_correction", true);
    this->declare_parameter<double>("target_pose_tracking_tau", 0.08);
    this->declare_parameter<double>("target_pose_correction_period", 0.1);
    this->declare_parameter<double>("target_pose_position_snap_threshold", 0.03);
    this->declare_parameter<double>("target_pose_yaw_snap_threshold", 3.0 * M_PI / 180.0);

    timer_rate_ = this->get_parameter("timer_rate").as_double();
    tau_ = this->get_parameter("tau").as_double();
    mass_ = this->get_parameter("mass").as_double();
    yaw_inertia_ = this->get_parameter("yaw_inertia").as_double();
    linear_velocity_response_gain_ =
      this->get_parameter("linear_velocity_response_gain").as_double();
    angular_velocity_response_gain_ =
      this->get_parameter("angular_velocity_response_gain").as_double();
    max_force_x_ = this->get_parameter("max_force_x").as_double();
    max_force_y_ = this->get_parameter("max_force_y").as_double();
    max_torque_z_ = this->get_parameter("max_torque_z").as_double();
    max_jerk_x_ = this->get_parameter("max_jerk_x").as_double();
    max_jerk_y_ = this->get_parameter("max_jerk_y").as_double();
    max_jerk_z_ = this->get_parameter("max_jerk_z").as_double();
    use_path_tracking_ = this->get_parameter("use_path_tracking").as_bool();
    enable_target_pose_correction_ = this->get_parameter("enable_target_pose_correction").as_bool();
    target_pose_tracking_tau_ = this->get_parameter("target_pose_tracking_tau").as_double();
    target_pose_correction_period_ =
      this->get_parameter("target_pose_correction_period").as_double();
    target_pose_position_snap_threshold_ =
      this->get_parameter("target_pose_position_snap_threshold").as_double();
    target_pose_yaw_snap_threshold_ =
      this->get_parameter("target_pose_yaw_snap_threshold").as_double();

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&MyNode::target_pose_callback, this, std::placeholders::_1));
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/waypoints", 10, std::bind(&MyNode::path_callback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MyNode::cmd_vel_callback, this, std::placeholders::_1));
    set_odometry_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/set_odometry", 10, std::bind(&MyNode::set_odometry_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&MyNode::timer_callback, this));
  }

private:
  double angle_normalize(double angle) const
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
  double angle_lpf(double target, double current, double tau, double dt) const
  {
    if (tau <= 1e-9) {
      return angle_normalize(target);
    }
    const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
    return angle_normalize(current + alpha * angle_normalize(target - current));
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_target_pose_source_ = *msg;
    has_target_pose_source_ = true;
    refresh_target_pose_to_odom();
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_source_ = *msg;
    has_path_source_ = !msg->poses.empty();
    if (!refresh_path_to_odom(true)) {
      return;
    }
  }

  void reset_pose(double x, double y, double yaw, const char * source)
  {
    pos_x_ = x;
    pos_y_ = y;
    yaw_ = angle_normalize(yaw);
    filtered_cmd_vel_.linear.x = 0.0;
    filtered_cmd_vel_.linear.y = 0.0;
    filtered_cmd_vel_.angular.z = 0.0;
    vel_x_body_ = 0.0;
    vel_y_body_ = 0.0;
    omega_z_ = 0.0;
    acc_x_ = 0.0;
    acc_y_ = 0.0;
    alpha_z_ = 0.0;
    RCLCPP_INFO(
      this->get_logger(), "Reset dummy odometry from %s to x=%.3f, y=%.3f, yaw=%.3f", source,
      pos_x_, pos_y_, yaw_);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { target_cmd_vel_ = *msg; }

  bool transform_pose_to_odom(
    const geometry_msgs::msg::PoseStamped & pose_in, geometry_msgs::msg::PoseStamped & pose_out)
  {
    geometry_msgs::msg::PoseStamped normalized_pose = pose_in;
    if (normalized_pose.header.frame_id.empty()) {
      normalized_pose.header.frame_id = "odom";
    }

    if (normalized_pose.header.frame_id == "odom") {
      pose_out = normalized_pose;
      return true;
    }

    // target_pose / waypoints は保持したメッセージを周期的に再解釈するため、
    // 元の stamp を使うと古い時刻で map->odom を引きに行ってしまう。
    // ここでは stamp=0 にして、常に最新の TF で odom へ変換する。
    normalized_pose.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    try {
      pose_out = tf_buffer_->transform(normalized_pose, "odom", tf2::durationFromSec(0.01));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to transform pose from %s to odom: %s",
        normalized_pose.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  bool transform_path_to_odom(const nav_msgs::msg::Path & path_in, nav_msgs::msg::Path & path_out)
  {
    path_out.header = path_in.header;
    path_out.header.frame_id = "odom";
    path_out.poses.clear();
    path_out.poses.reserve(path_in.poses.size());

    for (const auto & pose : path_in.poses) {
      geometry_msgs::msg::PoseStamped normalized_pose = pose;
      if (normalized_pose.header.frame_id.empty()) {
        normalized_pose.header.frame_id =
          path_in.header.frame_id.empty() ? "odom" : path_in.header.frame_id;
      }

      geometry_msgs::msg::PoseStamped transformed_pose;
      if (!transform_pose_to_odom(normalized_pose, transformed_pose)) {
        return false;
      }
      path_out.poses.push_back(transformed_pose);
    }
    return true;
  }

  bool refresh_target_pose_to_odom()
  {
    if (!has_target_pose_source_) {
      has_target_pose_ = false;
      return true;
    }

    geometry_msgs::msg::PoseStamped transformed_target_pose;
    if (!transform_pose_to_odom(latest_target_pose_source_, transformed_target_pose)) {
      return false;
    }
    latest_target_pose_ = transformed_target_pose;
    has_target_pose_ = true;
    return true;
  }

  bool refresh_path_to_odom(bool reset_progress_to_nearest)
  {
    if (!has_path_source_) {
      latest_path_.poses.clear();
      path_arc_length_.clear();
      has_path_ = false;
      current_path_distance_ = 0.0;
      return true;
    }

    nav_msgs::msg::Path transformed_path;
    if (!transform_path_to_odom(latest_path_source_, transformed_path)) {
      return false;
    }
    latest_path_ = transformed_path;
    path_arc_length_.clear();
    path_arc_length_.reserve(latest_path_.poses.size());
    path_arc_length_.push_back(0.0);

    for (size_t i = 1; i < latest_path_.poses.size(); ++i) {
      const auto & prev = latest_path_.poses[i - 1].pose.position;
      const auto & curr = latest_path_.poses[i].pose.position;
      const double ds = std::hypot(curr.x - prev.x, curr.y - prev.y);
      path_arc_length_.push_back(path_arc_length_.back() + ds);
    }

    has_path_ = !latest_path_.poses.empty();
    if (!has_path_) {
      current_path_distance_ = 0.0;
      return true;
    }

    if (reset_progress_to_nearest) {
      // 現在位置に一番近い点から再開して、path 更新時の不連続を減らす。
      size_t nearest_index = 0;
      double nearest_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < latest_path_.poses.size(); ++i) {
        const auto & p = latest_path_.poses[i].pose.position;
        const double dist = std::hypot(p.x - pos_x_, p.y - pos_y_);
        if (dist < nearest_dist) {
          nearest_dist = dist;
          nearest_index = i;
        }
      }
      current_path_distance_ = path_arc_length_[nearest_index];
    } else {
      current_path_distance_ = std::clamp(current_path_distance_, 0.0, path_arc_length_.back());
    }

    return true;
  }

  void set_odometry_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Set odometry message must contain at least 3 elements");
      return;
    }
    reset_pose(msg->data[0], msg->data[1], msg->data[2], "/set_odometry");
  }

  geometry_msgs::msg::PoseStamped sample_path_pose(double distance) const
  {
    geometry_msgs::msg::PoseStamped pose;
    if (latest_path_.poses.empty()) {
      return pose;
    }
    if (latest_path_.poses.size() == 1 || distance <= 0.0) {
      return latest_path_.poses.front();
    }

    const double total_distance = path_arc_length_.empty() ? 0.0 : path_arc_length_.back();
    if (distance >= total_distance) {
      return latest_path_.poses.back();
    }

    for (size_t i = 1; i < path_arc_length_.size(); ++i) {
      if (distance > path_arc_length_[i]) {
        continue;
      }

      const double segment_length = std::max(path_arc_length_[i] - path_arc_length_[i - 1], 1e-9);
      const double ratio = (distance - path_arc_length_[i - 1]) / segment_length;
      const auto & p0 = latest_path_.poses[i - 1];
      const auto & p1 = latest_path_.poses[i];
      const double yaw0 = tf2::getYaw(p0.pose.orientation);
      const double yaw1 = tf2::getYaw(p1.pose.orientation);

      pose.header = p0.header;
      pose.pose.position.x =
        p0.pose.position.x + ratio * (p1.pose.position.x - p0.pose.position.x);
      pose.pose.position.y =
        p0.pose.position.y + ratio * (p1.pose.position.y - p0.pose.position.y);
      pose.pose.position.z =
        p0.pose.position.z + ratio * (p1.pose.position.z - p0.pose.position.z);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, angle_normalize(yaw0 + ratio * angle_normalize(yaw1 - yaw0)));
      q.normalize();
      pose.pose.orientation = tf2::toMsg(q);
      return pose;
    }

    return latest_path_.poses.back();
  }

  void timer_callback()
  {
    const double dt = 1.0 / timer_rate_;
    if (has_target_pose_source_) {
      refresh_target_pose_to_odom();
    }
    if (has_path_source_) {
      refresh_path_to_odom(false);
    }
    // 指令値をそのまま使うと角が立ちやすいので、まずは速度指令自体をなめらかにする。
    filtered_cmd_vel_.linear.x =
      lpf(target_cmd_vel_.linear.x, filtered_cmd_vel_.linear.x, tau_, dt);
    filtered_cmd_vel_.linear.y =
      lpf(target_cmd_vel_.linear.y, filtered_cmd_vel_.linear.y, tau_, dt);
    filtered_cmd_vel_.angular.z =
      lpf(target_cmd_vel_.angular.z, filtered_cmd_vel_.angular.z, tau_, dt);

    double vx_world = 0.0;
    double vy_world = 0.0;
    omega_z_ = filtered_cmd_vel_.angular.z;

    if (use_path_tracking_ && has_path_) {
      // Path 上の距離だけを cmd_vel の並進速度で進める。位置は軌道に拘束し、速度は cmd_vel を使う。
      const double speed = std::hypot(filtered_cmd_vel_.linear.x, filtered_cmd_vel_.linear.y);
      const double total_distance = path_arc_length_.empty() ? 0.0 : path_arc_length_.back();
      current_path_distance_ = std::min(current_path_distance_ + speed * dt, total_distance);
      const auto sampled_pose = sample_path_pose(current_path_distance_);
      pos_x_ = sampled_pose.pose.position.x;
      pos_y_ = sampled_pose.pose.position.y;
      yaw_ = angle_normalize(tf2::getYaw(sampled_pose.pose.orientation));
      vel_x_body_ = filtered_cmd_vel_.linear.x;
      vel_y_body_ = filtered_cmd_vel_.linear.y;
      vx_world = vel_x_body_ * std::cos(yaw_) - vel_y_body_ * std::sin(yaw_);
      vy_world = vel_x_body_ * std::sin(yaw_) + vel_y_body_ * std::cos(yaw_);
    } else {
      // Path が無いときだけ、従来の簡易ダイナミクスで自己位置を更新する。
      const double max_acc_x = max_force_x_ / std::max(mass_, 1e-6);
      const double max_acc_y = max_force_y_ / std::max(mass_, 1e-6);
      const double max_alpha_z = max_torque_z_ / std::max(yaw_inertia_, 1e-6);

      const double desired_acc_x = std::clamp(
        linear_velocity_response_gain_ * (filtered_cmd_vel_.linear.x - vel_x_body_), -max_acc_x,
        max_acc_x);
      const double desired_acc_y = std::clamp(
        linear_velocity_response_gain_ * (filtered_cmd_vel_.linear.y - vel_y_body_), -max_acc_y,
        max_acc_y);
      const double desired_alpha_z = std::clamp(
        angular_velocity_response_gain_ * (filtered_cmd_vel_.angular.z - omega_z_), -max_alpha_z,
        max_alpha_z);

      acc_x_ = std::clamp(
        desired_acc_x, acc_x_ - max_jerk_x_ * dt, acc_x_ + max_jerk_x_ * dt);
      acc_y_ = std::clamp(
        desired_acc_y, acc_y_ - max_jerk_y_ * dt, acc_y_ + max_jerk_y_ * dt);
      alpha_z_ = std::clamp(
        desired_alpha_z, alpha_z_ - max_jerk_z_ * dt, alpha_z_ + max_jerk_z_ * dt);

      vel_x_body_ += acc_x_ * dt;
      vel_y_body_ += acc_y_ * dt;
      omega_z_ += alpha_z_ * dt;

      vx_world = vel_x_body_ * std::cos(yaw_) - vel_y_body_ * std::sin(yaw_);
      vy_world = vel_x_body_ * std::sin(yaw_) + vel_y_body_ * std::cos(yaw_);

      pos_x_ += vx_world * dt;
      pos_y_ += vy_world * dt;
      yaw_ = angle_normalize(yaw_ + omega_z_ * dt);
    }

    if (!use_path_tracking_ && enable_target_pose_correction_ && has_target_pose_) {
      const double target_x = latest_target_pose_.pose.position.x;
      const double target_y = latest_target_pose_.pose.position.y;
      const double target_yaw = tf2::getYaw(latest_target_pose_.pose.orientation);
      const double position_error = std::hypot(target_x - pos_x_, target_y - pos_y_);
      const double yaw_error = std::abs(angle_normalize(target_yaw - yaw_));

      // 通常時は毎周期なめらかに目標姿勢へ寄せて、発散を防ぐ。
      pos_x_ = lpf(target_x, pos_x_, target_pose_tracking_tau_, dt);
      pos_y_ = lpf(target_y, pos_y_, target_pose_tracking_tau_, dt);
      yaw_ = angle_lpf(target_yaw, yaw_, target_pose_tracking_tau_, dt);

      // 大きく外れたときだけ一定周期でスナップして、復帰を早める。
      target_pose_correction_elapsed_ += dt;
      if (
        target_pose_correction_elapsed_ >= target_pose_correction_period_ &&
        (position_error > target_pose_position_snap_threshold_ ||
        yaw_error > target_pose_yaw_snap_threshold_)) {
        pos_x_ = target_x;
        pos_y_ = target_y;
        yaw_ = angle_normalize(target_yaw);
        target_pose_correction_elapsed_ = 0.0;
      } else if (target_pose_correction_elapsed_ >= target_pose_correction_period_) {
        target_pose_correction_elapsed_ = 0.0;
      }
    } else {
      target_pose_correction_elapsed_ = 0.0;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pos_x_;
    odom.pose.pose.position.y = pos_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = vx_world;
    odom.twist.twist.linear.y = vy_world;
    odom.twist.twist.angular.z = omega_z_;
    odometry_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Published dummy odometry: x=%.3f, y=%.3f, yaw=%.3f, vx=%.3f, vy=%.3f, wz=%.3f", pos_x_,
      pos_y_, yaw_, vx_world, vy_world, omega_z_);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::PoseStamped latest_target_pose_source_;
  nav_msgs::msg::Path latest_path_;
  nav_msgs::msg::Path latest_path_source_;
  geometry_msgs::msg::Twist target_cmd_vel_;
  geometry_msgs::msg::Twist filtered_cmd_vel_;
  std::vector<double> path_arc_length_;

  double timer_rate_ = 100.0;
  double tau_ = 0.15;
  double mass_ = 30.0;
  double yaw_inertia_ = 1.8;
  double linear_velocity_response_gain_ = 6.0;
  double angular_velocity_response_gain_ = 8.0;
  double max_force_x_ = 120.0;
  double max_force_y_ = 120.0;
  double max_torque_z_ = 6.0;
  double max_jerk_x_ = 40.0;
  double max_jerk_y_ = 40.0;
  double max_jerk_z_ = 30.0;
  bool use_path_tracking_ = true;
  bool enable_target_pose_correction_ = true;
  double target_pose_tracking_tau_ = 0.08;
  double target_pose_correction_period_ = 0.1;
  double target_pose_position_snap_threshold_ = 0.03;
  double target_pose_yaw_snap_threshold_ = 3.0 * M_PI / 180.0;
  double target_pose_correction_elapsed_ = 0.0;
  double current_path_distance_ = 0.0;
  bool has_target_pose_ = false;
  bool has_target_pose_source_ = false;
  bool has_path_ = false;
  bool has_path_source_ = false;
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double yaw_ = 0.0;
  double vel_x_body_ = 0.0;
  double vel_y_body_ = 0.0;
  double omega_z_ = 0.0;
  double acc_x_ = 0.0;
  double acc_y_ = 0.0;
  double alpha_z_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
