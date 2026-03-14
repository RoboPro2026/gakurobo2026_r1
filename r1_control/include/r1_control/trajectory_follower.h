/**
 * @file trajectory_follower.h
 * @author Yudai Yamaguchi
 * @brief 起動追従するクラス
 * @version 0.1
 * @date 2026-02-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r1_control/trajectory_planner.h"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct WayPoint
{
  double x;
  double y;
  double theta;
  double v_trans;
  double a_trans;
  double j_trans;
  double omega;
  double curvature;
};

class TrajectoryFollower
{
public:
  TrajectoryFollower(
    std::shared_ptr<TrajectoryPlanner> traj_planner, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener)
  : logger_(rclcpp::get_logger("trajectory_follower"))
  {
    traj_planner_ = traj_planner;
    tf_buffer_ = tf_buffer;
    tf_listener_ = tf_listener;
    finish_ = 0;
    last_out_of_range_time_ = rclcpp::Clock().now();
  }

  void set_param(
    double kp_pos_normal, double ki_pos_normal, double kd_pos_normal, double kp_pos_goal,
    double ki_pos_goal, double kd_pos_goal, double vel_i_limit, double kp_angle_normal,
    double ki_angle_normal, double kd_angle_normal, double kp_angle_goal, double ki_angle_goal,
    double kd_angle_goal, double omega_i_limit, double dt, double search_radius,
    double goal_pos_range, double goal_angle_range, double finish_time_threshold)
  {
    kp_pos_normal_ = kp_pos_normal;
    ki_pos_normal_ = ki_pos_normal;
    kd_pos_normal_ = kd_pos_normal;
    kp_pos_goal_ = kp_pos_goal;
    ki_pos_goal_ = ki_pos_goal;
    kd_pos_goal_ = kd_pos_goal;
    vel_i_limit_ = vel_i_limit;
    kp_angle_normal_ = kp_angle_normal;
    ki_angle_normal_ = ki_angle_normal;
    kd_angle_normal_ = kd_angle_normal;
    kp_angle_goal_ = kp_angle_goal;
    ki_angle_goal_ = ki_angle_goal;
    kd_angle_goal_ = kd_angle_goal;
    omega_i_limit_ = omega_i_limit;
    dt_ = dt;
    search_radius_ = search_radius;
    goal_pos_range_ = goal_pos_range;
    goal_angle_range_ = goal_angle_range;
    finish_time_threshold_ = finish_time_threshold;
    last_out_of_range_time_ = rclcpp::Clock().now();
  }

  void reset()
  {
    idx_ = 0;
    is_last_point_ = false;
    finish_ = 0;
    last_out_of_range_time_ = rclcpp::Clock().now();
    // 積分項をリセット
    integral_error_ = {0.0, 0.0, 0.0};
  }

  /**
   * @brief PID制御を行う（位置制御）
   * 
   * @param x_ref 目標位置(x_ref, y_ref, theta_ref)
   * @param x 現在の位置(x, y, theta)
   * @param v_ref 目標速度 (vx_refm vy_ref omega_ref)
   * @param v 現在の速度 (vx, vy, omega)
   * @return std::vector<double> 速度ベクトル指令値(vx, vy, omega)
   */
  std::vector<double> control(
    std::vector<double> x_ref, std::vector<double> x, std::vector<double> v_ref,
    std::vector<double> v)
  {
    double kp_pos, ki_pos, kd_pos, kp_angle, ki_angle, kd_angle;
    // 最後のwaypointのときはゴール用ゲインを使用する。そうでなければ、通常時のゲインを使用する。
    if (is_last_point_) {
      kp_pos = kp_pos_goal_;
      ki_pos = ki_pos_goal_;
      kd_pos = kd_pos_goal_;
      kp_angle = kp_angle_goal_;
      ki_angle = ki_angle_goal_;
      kd_angle = kd_angle_goal_;
    } else {
      kp_pos = kp_pos_normal_;
      ki_pos = ki_pos_normal_;
      kd_pos = kd_pos_normal_;
      kp_angle = kp_angle_normal_;
      ki_angle = ki_angle_normal_;
      kd_angle = kd_angle_normal_;
    }
    std::vector<double> ret(3);
    std::vector<double> error(3);
    error[0] = x_ref[0] - x[0];
    error[1] = x_ref[1] - x[1];
    error[2] = angle_diff(x_ref[2], x[2]);
    integral_error_[0] += error[0] * dt_;
    integral_error_[1] += error[1] * dt_;
    integral_error_[2] += error[2] * dt_;
    // 積分器のリミッター。kiが0でないことを確認してから実行する。
    if (std::abs(ki_pos) >= 1e-100) {
      double limit = vel_i_limit_ / ki_pos;
      integral_error_[0] = std::clamp(integral_error_[0], -limit, limit);
      integral_error_[1] = std::clamp(integral_error_[1], -limit, limit);
    }
    if (std::abs(ki_angle) >= 1e-100) {
      double limit = omega_i_limit_ / ki_angle;
      integral_error_[2] = std::clamp(integral_error_[2], -limit, limit);
    }
    // PID制御(DはFFとFB)
    ret[0] = kp_pos * error[0] + ki_pos * integral_error_[0] + kd_pos * (v_ref[0] - v[0]);
    ret[1] = kp_pos * error[1] + ki_pos * integral_error_[1] + kd_pos * (v_ref[1] - v[1]);
    ret[2] = kp_angle * error[2] + ki_angle * integral_error_[2] + kd_angle * (v_ref[2] - v[2]);

    return ret;
  }

  /**
   * @brief 現在位置から次のwaypointを探索し、軌道追従を行う。WayPointと速度ベクトルを返す。
   * 
   * @param _x 現在の位置(x, y, theta)
   * @return std::pair<WayPoint, geometry_msgs::msg::Twist> WayPointは次のwaypoint、geometry_msgs::msg::Twistは速度ベクトル(vx, vy, omega)
   */
  std::pair<WayPoint, geometry_msgs::msg::Twist> update(nav_msgs::msg::Odometry odometry)
  {
    // odomの位置を取得
    double x = odometry.pose.pose.position.x;
    double y = odometry.pose.pose.position.y;
    double theta = tf2::getYaw(odometry.pose.pose.orientation);
    double dx = 0.0, dy = 0.0, dist = 0.0;
    // 現在位置から次のwaypointを探索
    while (idx_ < traj_planner_->array_size_) {
      // 目標位置を取得
      // 目標位置はmap座標系のものをodom座標系に変換して使用する
      // odom座標系にて制御を行う理由は、map座標系で制御を行うと、map->odomのtfが変化したときに現在位置が急激にずれるのを防ぐため
      double x_map = traj_planner_->x_[idx_];
      double y_map = traj_planner_->y_[idx_];
      double theta_map = traj_planner_->theta_[idx_];
      geometry_msgs::msg::PoseStamped pose_map;
      pose_map.header.frame_id = "map";
      pose_map.pose.position.x = x_map;
      pose_map.pose.position.y = y_map;
      pose_map.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta_map);
      pose_map.pose.orientation = tf2::toMsg(q);
      geometry_msgs::msg::PoseStamped pose_odom;
      try {
        tf_buffer_->transform(pose_map, pose_odom, "odom", tf2::durationFromSec(0.01));
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(logger_, "Could not transform pose from map to odom: %s", ex.what());
        break;
      }
      // 目標位置と現在位置の距離を計算
      dx = traj_planner_->x_[idx_] - x;
      dy = traj_planner_->y_[idx_] - y;
      dist = std::sqrt(dx * dx + dy * dy);
      if (dist > search_radius_) {
        break;
      }
      idx_++;
    }

    if (idx_ >= traj_planner_->array_size_ - 1) {
      idx_ = traj_planner_->array_size_ - 1;
    }

    // 次のwaypointを更新
    WayPoint wp;
    wp.x = traj_planner_->x_[idx_];
    wp.y = traj_planner_->y_[idx_];
    wp.theta = traj_planner_->theta_[idx_];
    wp.v_trans = traj_planner_->v_trans_[idx_];
    wp.a_trans = traj_planner_->a_trans_[idx_];
    wp.j_trans = traj_planner_->j_trans_[idx_];
    wp.omega = traj_planner_->omega_[idx_];
    wp.curvature = traj_planner_->curvature_[idx_];

    // 現在の位置のnext_waypointから、角度を計算
    double arg = std::atan2(wp.y - y, wp.x - x);
    double vx = wp.v_trans * std::cos(arg);
    double vy = wp.v_trans * std::sin(arg);

    std::vector<double> x_ref = {wp.x, wp.y, wp.theta};
    std::vector<double> _x = {x, y, theta};
    std::vector<double> v_ref = {vx, vy, wp.omega};
    std::vector<double> _v = {
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.angular.z};

    // 終了判定
    is_last_point_ = (idx_ == traj_planner_->array_size_ - 1);
    bool is_pos_goal = (dist < goal_pos_range_);
    bool is_angle_goal = (std::abs(angle_diff(theta, wp.theta)) < goal_angle_range_);
    // 範囲外のときは、収束判定用変数を更新
    if (is_last_point_ == false || is_pos_goal == false || is_angle_goal == false) {
      last_out_of_range_time_ = rclcpp::Clock().now();
    }
    // 収束したかの終了判定
    bool is_time_ok =
      (rclcpp::Clock().now() - last_out_of_range_time_).seconds() > finish_time_threshold_;

    if (is_last_point_ && is_pos_goal && is_angle_goal && is_time_ok) {
      finish_ = 1;
    }

    // 足回りの速度ベクトルを計算
    std::vector<double> v_chassis;
    if (finish_ == 0) {
      v_chassis = control(x_ref, _x, v_ref, _v);
    } else {
      v_chassis = {0.0, 0.0, 0.0};
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_chassis[0];
    cmd_vel.linear.y = v_chassis[1];
    cmd_vel.angular.z = v_chassis[2];

    return std::make_pair(wp, cmd_vel);
  }

  int is_finished() { return finish_; }

private:
  std::shared_ptr<TrajectoryPlanner> traj_planner_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // 探索半径[m]
  double search_radius_ = 0.0;
  // 通常時のPID位置ゲイン
  double kp_pos_normal_ = 0.0;
  double ki_pos_normal_ = 0.0;
  double kd_pos_normal_ = 0.0;
  // ゴール時のPID位置ゲイン
  double kp_pos_goal_ = 0.0;
  double ki_pos_goal_ = 0.0;
  double kd_pos_goal_ = 0.0;
  // 積分器のリミッター
  double vel_i_limit_ = 0.0;
  // 通常時のPID角度ゲイン
  double kp_angle_normal_ = 0.0;
  double ki_angle_normal_ = 0.0;
  double kd_angle_normal_ = 0.0;
  // ゴール時のPID角度ゲイン
  double kp_angle_goal_ = 0.0;
  double ki_angle_goal_ = 0.0;
  double kd_angle_goal_ = 0.0;
  // 積分器のリミッター
  double omega_i_limit_ = 0.0;

  double goal_pos_range_ = 0.01;        // ゴールとみなす距離の閾値
  double goal_angle_range_ = 0.01;      // ゴールとみなす位置の閾値
  double finish_time_threshold_ = 0.3;  // 収束時間の判定用しきい値
  std::vector<double> integral_error_{0.0, 0.0, 0.0};
  double dt_ = 0.0;
  int idx_ = 0;
  bool is_last_point_ = false;
  int finish_ = 0;
  rclcpp::Time last_out_of_range_time_ = rclcpp::Clock().now();
};
