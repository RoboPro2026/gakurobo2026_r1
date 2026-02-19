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
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  TrajectoryFollower(TrajectoryPlanner * traj_planner)
  : logger_(rclcpp::get_logger("trajectory_follower"))
  {
    traj_planner_ = traj_planner;
    finish_ = 0;
  }

  void set_param(
    double kp, double ki, double kd, double kff, double dt, double search_radius, double eps)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    kff_ = kff;
    search_radius_ = search_radius;
    goal_range_ = eps;
  }

  void reset()
  {
    idx_ = 0;
    finish_ = 0;
  }

  /**
   * @brief P制御+軌道FFを行う。（位置制御）
   * 
   * @param x_ref 目標位置(x_ref, y_ref, theta_ref)
   * @param x 現在の位置(x, y, theta)
   * @param v_ref 目標速度 (vx_refm vy_ref omega_ref)
   * @return std::vector<double> 速度ベクトル(vx, vy, omega)
   */
  std::vector<double> control(
    std::vector<double> x_ref, std::vector<double> x, std::vector<double> v_ref)
  {
    // PIDの実装になっているが、実際に使用しているのはPのみ。
    std::vector<double> ret(3);
    std::vector<double> error(3);
    for (int i = 0; i < 3; i++) {
      error[i] = x_ref[i] - x[i];
      integral_error_[i] += error[i] * dt_;
      [[maybe_unused]] const double derivative = (error[i] - prev_error_[i]) / dt_;
      prev_error_[i] = error[i];
      // p制御+軌道FF
      ret[i] = kp_ * error[i] + kff_ * v_ref[i];
      // ret[i] = kp_ * error[i] + ki_ * integral_error_[i] + kd_ * derivative + kff_ * v_ref[i];
    }
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
    double x = odometry.pose.pose.position.x;
    double y = odometry.pose.pose.position.y;
    double theta = tf2::getYaw(odometry.pose.pose.orientation);
    double dx, dy, dist;
    // 現在位置から次のwaypointを探索
    while (idx_ < traj_planner_->array_size_) {
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

    // 終了判定
    bool is_last_point = (idx_ == traj_planner_->array_size_ - 1);
    bool is_dist_goal = (dist < goal_range_);
    bool is_theta_goal = (std::abs(theta - wp.theta) < goal_range_);
    if (is_last_point && is_dist_goal && is_theta_goal) {
      finish_ = 1;
    }
    // 足回りの速度ベクトルを計算
    std::vector<double> v_chassis;
    if (finish_ == 0) {
      v_chassis = control(x_ref, _x, v_ref);
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
  TrajectoryPlanner * traj_planner_;
  rclcpp::Logger logger_;
  // 探索半径[m]
  double search_radius_ = 0.0;
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kff_ = 0.0;
  double goal_range_ = 0.01;  // ゴールとみなす距離の閾値
  std::vector<double> prev_error_{0.0, 0.0, 0.0};
  std::vector<double> integral_error_{0.0, 0.0, 0.0};
  double dt_ = 0.0;
  int idx_ = 0;
  int finish_ = 0;
};
