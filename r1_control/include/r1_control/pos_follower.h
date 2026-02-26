/**
 * @file pos_follower.h
 * @author Yudai Yamaguchi
 * @brief 位置制御をするクラス
 * @version 0.1
 * @date 2026-02-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r1_control/trajectory_follower.h"
#include "r1_control/trajectory_planner.h"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
class PosFollower
{
public:
  PosFollower(void) : logger_(rclcpp::get_logger("PosFollower"))
  {
    finish_ = 0;
    last_out_of_range_time_ = rclcpp::Clock().now();
  }

  void set_param(
    double kp_pos, double ki_pos, double kd_pos, double kp_angle, double ki_angle, double kd_angle,
    double dt, double goal_pos_range, double goal_angle_range, double finish_time_threshold)
  {
    kp_pos_ = kp_pos;
    ki_pos_ = ki_pos;
    kd_pos_ = kd_pos;
    kp_angle_ = kp_angle;
    ki_angle_ = ki_angle;
    kd_angle_ = kd_angle;
    dt_ = dt;
    goal_pos_range_ = goal_pos_range;
    goal_angle_range_ = goal_angle_range;
    finish_time_threshold_ = finish_time_threshold;
  }

  void reset()
  {
    finish_ = 0;
    last_out_of_range_time_ = rclcpp::Clock().now();
  }

  std::vector<double> control(std::vector<double> x_ref, std::vector<double> x)
  {
    std::vector<double> ret(3);
    std::vector<double> error(3);
    error[0] = x_ref[0] - x[0];
    error[1] = x_ref[1] - x[1];
    error[2] = angle_diff(x_ref[2], x[2]);
    // p制御
    ret[0] = 3.0 * kp_pos_ * error[0];
    ret[1] = 3.0 * kp_pos_ * error[1];
    ret[2] = 3.0 * kp_angle_ * error[2];
    return ret;
  }

  std::pair<WayPoint, geometry_msgs::msg::Twist> update(
    nav_msgs::msg::Odometry odometry, std::vector<double> target)
  {
    double x = odometry.pose.pose.position.x;
    double y = odometry.pose.pose.position.y;
    double theta = tf2::getYaw(odometry.pose.pose.orientation);

    bool is_pos_in_range = std::hypot(target[0] - x, target[1] - y) < goal_pos_range_;
    bool is_angle_in_range = std::abs(angle_diff(target[2], theta)) < goal_angle_range_;
    // 範囲外のときは、収束判定用変数を更新
    if (is_pos_in_range == false || is_angle_in_range == false) {
      last_out_of_range_time_ = rclcpp::Clock().now();
    }
    // 収束したかの終了判定
    bool is_time_ok =
      (rclcpp::Clock().now() - last_out_of_range_time_).seconds() > finish_time_threshold_;

    if (is_pos_in_range && is_angle_in_range && is_time_ok) {
      finish_ = 1;
    }

    // 足回りの速度ベクトルを計算
    std::vector<double> v_chassis;
    if (finish_ == 0) {
      v_chassis = control(target, {x, y, theta});
    } else {
      v_chassis = {0.0, 0.0, 0.0};
    }

    WayPoint wp;
    wp.x = x;
    wp.y = y;
    wp.theta = theta;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_chassis[0];
    cmd_vel.linear.y = v_chassis[1];
    cmd_vel.angular.z = v_chassis[2];

    return std::make_pair(wp, cmd_vel);
  }

  int is_finished() { return finish_; }

private:
  rclcpp::Logger logger_;
  int finish_ = 0;
  double kp_pos_ = 0.0;
  double ki_pos_ = 0.0;
  double kd_pos_ = 0.0;
  double kp_angle_ = 0.0;
  double ki_angle_ = 0.0;
  double kd_angle_ = 0.0;
  double dt_ = 0.0;
  double goal_pos_range_ = 0.05;
  double goal_angle_range_ = 0.05;
  double finish_time_threshold_ = 0.3;
  rclcpp::Time last_out_of_range_time_ = rclcpp::Clock().now();
};
