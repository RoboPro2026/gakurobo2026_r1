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

#include <algorithm>
#include <cmath>
#include <limits>

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
    bool use_map, double kp_pos_normal, double ki_pos_normal, double kd_pos_normal,
    double kp_pos_goal, double ki_pos_goal, double kd_pos_goal, double vel_i_limit,
    double vel_output_limit, double kp_angle_normal, double ki_angle_normal, double kd_angle_normal,
    double kp_angle_goal, double ki_angle_goal, double kd_angle_goal, double omega_i_limit,
    double omega_output_limit, double dt, double search_radius, double lookahead_time,
    double goal_pos_range, double goal_angle_range, double finish_time_threshold)
  {
    use_map_ = use_map;
    kp_pos_normal_ = kp_pos_normal;
    ki_pos_normal_ = ki_pos_normal;
    kd_pos_normal_ = kd_pos_normal;
    kp_pos_goal_ = kp_pos_goal;
    ki_pos_goal_ = ki_pos_goal;
    kd_pos_goal_ = kd_pos_goal;
    vel_i_limit_ = vel_i_limit;
    vel_output_limit_ = vel_output_limit;
    kp_angle_normal_ = kp_angle_normal;
    ki_angle_normal_ = ki_angle_normal;
    kd_angle_normal_ = kd_angle_normal;
    kp_angle_goal_ = kp_angle_goal;
    ki_angle_goal_ = ki_angle_goal;
    kd_angle_goal_ = kd_angle_goal;
    omega_i_limit_ = omega_i_limit;
    omega_output_limit_ = omega_output_limit;
    dt_ = dt;
    search_radius_ = search_radius;
    lookahead_time_ = lookahead_time;
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
    // 偏差をリセット
    error_ = {0.0, 0.0, 0.0};
    prev_error_ = {0.0, 0.0, 0.0};
    // 積分項をリセット
    integral_error_ = {0.0, 0.0, 0.0};
    // 微分項をリセット
    derivative_error_ = {0.0, 0.0, 0.0};
  }

  // 軌道配列上の index から map 座標系の姿勢を組み立てる。
  geometry_msgs::msg::PoseStamped make_map_pose(int index) const
  {
    geometry_msgs::msg::PoseStamped pose_map;
    pose_map.header.frame_id = "map";
    pose_map.pose.position.x = traj_planner_->x_[index];
    pose_map.pose.position.y = traj_planner_->y_[index];
    pose_map.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, traj_planner_->theta_[index]);
    q.normalize();
    pose_map.pose.orientation = tf2::toMsg(q);
    return pose_map;
  }

  // 制御は odom 座標系で行うため、必要に応じて map -> odom 変換を行う。
  bool to_odom_pose(
    const geometry_msgs::msg::PoseStamped & pose_map, geometry_msgs::msg::PoseStamped & pose_odom)
  {
    if (!use_map_) {
      pose_odom = pose_map;
      return true;
    }
    try {
      tf_buffer_->transform(pose_map, pose_odom, "odom", tf2::durationFromSec(0.01));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Could not transform pose from map to odom: %s", ex.what());
      return false;
    }
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
    error_[0] = x_ref[0] - x[0];
    error_[1] = x_ref[1] - x[1];
    error_[2] = angle_diff(x_ref[2], x[2]);
    integral_error_[0] += error_[0] * dt_;
    integral_error_[1] += error_[1] * dt_;
    integral_error_[2] += error_[2] * dt_;
    derivative_error_[0] = (error_[0] - prev_error_[0]) / dt_;
    derivative_error_[1] = (error_[1] - prev_error_[1]) / dt_;
    derivative_error_[2] = (error_[2] - prev_error_[2]) / dt_;
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
    // PID制御+速度FF
    ret[0] =
      kp_pos * error_[0] + ki_pos * integral_error_[0] + kd_pos * derivative_error_[0] + v_ref[0];
    ret[1] =
      kp_pos * error_[1] + ki_pos * integral_error_[1] + kd_pos * derivative_error_[1] + v_ref[1];
    ret[2] = kp_angle * error_[2] + ki_angle * integral_error_[2] +
             kd_angle * derivative_error_[2] + v_ref[2];

    // PID制御の出力制限
    // 本当はやりたくないが、出力制限をしないと危ない挙動をするときがあるため
    ret[0] = std::clamp(ret[0], -vel_output_limit_, vel_output_limit_);
    ret[1] = std::clamp(ret[1], -vel_output_limit_, vel_output_limit_);
    ret[2] = std::clamp(ret[2], -omega_output_limit_, omega_output_limit_);

    // 前回値の更新
    prev_error_[0] = error_[0];
    prev_error_[1] = error_[1];
    prev_error_[2] = error_[2];
    return ret;
  }

  /**
   * @brief 現在位置から次のwaypointを探索し、軌道追従を行う。map座標系のWayPointと速度ベクトルを返す。
   * 
   * @param _x odom座標系の現在位置(x, y, theta)
   * @return std::pair<WayPoint, geometry_msgs::msg::Twist> WayPointは次のwaypoint、geometry_msgs::msg::Twistは速度ベクトル(vx, vy, omega)
   */
  std::pair<WayPoint, geometry_msgs::msg::Twist> update(nav_msgs::msg::Odometry odometry)
  {
    geometry_msgs::msg::PoseStamped current_pose_odom;
    current_pose_odom.header = odometry.header;
    current_pose_odom.pose = odometry.pose.pose;

    double x = odometry.pose.pose.position.x;
    double y = odometry.pose.pose.position.y;
    double theta = tf2::getYaw(odometry.pose.pose.orientation);
    double search_x = x;
    double search_y = y;
    if (use_map_) {
      // 最寄り点探索だけは軌道と同じ map 座標系で行い、TF のずれで探索が不安定になるのを避ける。
      geometry_msgs::msg::PoseStamped current_pose_map;
      try {
        // map->odomに座標変換
        current_pose_map =
          tf_buffer_->transform(current_pose_odom, "map", tf2::durationFromSec(0.01));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(logger_, "Could not transform pose from odom to map: %s", ex.what());
        // 失敗したときは適当なwaypointとcmd_velを返す。
        geometry_msgs::msg::Twist cmd_vel;
        WayPoint wp_map{};
        if (traj_planner_->array_size_ > 0) {
          wp_map.x = traj_planner_->x_[idx_];
          wp_map.y = traj_planner_->y_[idx_];
          wp_map.theta = traj_planner_->theta_[idx_];
          wp_map.v_trans = traj_planner_->v_trans_[idx_];
          wp_map.a_trans = traj_planner_->a_trans_[idx_];
          wp_map.j_trans = traj_planner_->j_trans_[idx_];
          wp_map.omega = traj_planner_->omega_[idx_];
          wp_map.curvature = traj_planner_->curvature_[idx_];
        }
        // 目標点が座標変換できない周期は無理に追従せず、ゼロ指令で抜ける。
        return std::make_pair(wp_map, cmd_vel);
      }
      search_x = current_pose_map.pose.position.x;
      search_y = current_pose_map.pose.position.y;
    }

    // 直前の index 以降だけを対象に最寄り点を探し、軌道上を後戻りしないようにする。
    int nearest_index = idx_;
    double nearest_dist = 1e9;
    for (int i = idx_; i < traj_planner_->array_size_; ++i) {
      double dx = traj_planner_->x_[i] - search_x;
      double dy = traj_planner_->y_[i] - search_y;
      double dist = dx * dx + dy * dy;
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest_index = i;
      }
    }
    idx_ = nearest_index;

    // 最寄り点から少し先を追うことで、高速時に古い waypoint へ引き戻される挙動を抑える。
    // search_radius_ は最低保証の先読み距離として使い、低速でも目標点が近すぎないようにする。
    // 速度が高いときは v_trans * lookahead_time_ を使って先読み距離を伸ばす。
    double lookahead_dist =
      std::max(search_radius_, std::abs(traj_planner_->v_trans_[nearest_index]) * lookahead_time_);
    int target_index = nearest_index;
    double target_distance = traj_planner_->distance_[nearest_index] + lookahead_dist;
    // 軌道距離 distance_ を基準にして目標点を進めることで、曲線でも一定の先読み量を保ちやすくする。
    while (target_index + 1 < traj_planner_->array_size_ &&
           traj_planner_->distance_[target_index] < target_distance) {
      target_index++;
    }

    geometry_msgs::msg::PoseStamped target_map = make_map_pose(target_index);
    geometry_msgs::msg::PoseStamped target_odom;
    if (!to_odom_pose(target_map, target_odom)) {
      // 目標点の座標変換に失敗したときは、適当な値を返す。
      geometry_msgs::msg::Twist cmd_vel;
      WayPoint wp_map;
      wp_map.x = traj_planner_->x_[target_index];
      wp_map.y = traj_planner_->y_[target_index];
      wp_map.theta = traj_planner_->theta_[target_index];
      wp_map.v_trans = traj_planner_->v_trans_[target_index];
      wp_map.a_trans = traj_planner_->a_trans_[target_index];
      wp_map.j_trans = traj_planner_->j_trans_[target_index];
      wp_map.omega = traj_planner_->omega_[target_index];
      wp_map.curvature = traj_planner_->curvature_[target_index];
      return std::make_pair(wp_map, cmd_vel);
    }

    double dx = target_odom.pose.position.x - x;
    double dy = target_odom.pose.position.y - y;
    double dist = std::sqrt(dx * dx + dy * dy);

    WayPoint wp_odom;
    wp_odom.x = target_odom.pose.position.x;
    wp_odom.y = target_odom.pose.position.y;
    wp_odom.theta = tf2::getYaw(target_odom.pose.orientation);
    wp_odom.v_trans = traj_planner_->v_trans_[target_index];
    wp_odom.a_trans = traj_planner_->a_trans_[target_index];
    wp_odom.j_trans = traj_planner_->j_trans_[target_index];
    wp_odom.omega = traj_planner_->omega_[target_index];
    wp_odom.curvature = traj_planner_->curvature_[target_index];

    // 並進FF軌道の接線方向に合わせる。
    // 接戦方向を軌道から取る理由は、現在位置とwaypointから取るとうまく行かなかったから。
    int tangent_from_index = target_index;
    int tangent_to_index = std::min(target_index + 1, traj_planner_->array_size_ - 1);
    if (tangent_to_index == tangent_from_index) {
      tangent_from_index = std::max(target_index - 1, 0);
    }
    geometry_msgs::msg::PoseStamped tangent_from_odom;
    geometry_msgs::msg::PoseStamped tangent_to_odom;
    bool tangent_ok = to_odom_pose(make_map_pose(tangent_from_index), tangent_from_odom) &&
                      to_odom_pose(make_map_pose(tangent_to_index), tangent_to_odom);

    // 接線がうまく取れないときは軌道に設定された姿勢角を使う。
    double tangent_heading = wp_odom.theta;
    if (tangent_ok) {
      double tangent_dx = tangent_to_odom.pose.position.x - tangent_from_odom.pose.position.x;
      double tangent_dy = tangent_to_odom.pose.position.y - tangent_from_odom.pose.position.y;
      if (std::hypot(tangent_dx, tangent_dy) > 1e-6) {
        tangent_heading = std::atan2(tangent_dy, tangent_dx);
      }
    }

    double vx = wp_odom.v_trans * std::cos(tangent_heading);
    double vy = wp_odom.v_trans * std::sin(tangent_heading);

    std::vector<double> x_ref = {wp_odom.x, wp_odom.y, wp_odom.theta};
    std::vector<double> _x = {x, y, theta};
    std::vector<double> v_ref = {vx, vy, wp_odom.omega};
    std::vector<double> _v = {
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.angular.z};

    // 終了判定
    is_last_point_ = (target_index == traj_planner_->array_size_ - 1);
    bool is_pos_goal = (dist < goal_pos_range_);
    bool is_angle_goal = (std::abs(angle_diff(theta, wp_odom.theta)) < goal_angle_range_);
    // 範囲外のときは、収束判定用変数を更新
    if (is_last_point_ == false || is_pos_goal == false || is_angle_goal == false) {
      last_out_of_range_time_ = rclcpp::Clock().now();
    }
    // 収束したかの終了判定
    // 一瞬だけ閾値内に入った場合ではなく、一定時間収束し続けたときだけ完了とみなす。
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

    // 速度指令値
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_chassis[0];
    cmd_vel.linear.y = v_chassis[1];
    cmd_vel.angular.z = v_chassis[2];
    // map座標系のwaypoint
    WayPoint wp_map;
    wp_map.x = traj_planner_->x_[idx_];
    wp_map.y = traj_planner_->y_[idx_];
    wp_map.theta = traj_planner_->theta_[idx_];
    wp_map.v_trans = traj_planner_->v_trans_[idx_];
    wp_map.a_trans = traj_planner_->a_trans_[idx_];
    wp_map.j_trans = traj_planner_->j_trans_[idx_];
    wp_map.omega = traj_planner_->omega_[idx_];
    wp_map.curvature = traj_planner_->curvature_[idx_];

    return std::make_pair(wp_map, cmd_vel);
  }

  int is_finished() { return finish_; }

  std::vector<double> get_error() { return error_; }
  std::vector<double> get_integral_error() { return integral_error_; }
  std::vector<double> get_derivative_error() { return derivative_error_; }

private:
  std::shared_ptr<TrajectoryPlanner> traj_planner_;
  rclcpp::Logger logger_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // mapを使用するか（Lidarを使用するか）
  bool use_map_;
  // 探索半径[m]
  double search_radius_ = 0.0;
  // 速度に応じて先読み距離を伸ばすための時間[s]
  double lookahead_time_ = 0.3;
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
  // PID制御の出力リミッター
  double vel_output_limit_ = 0.0;
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
  // PID制御の出力リミッター
  double omega_output_limit_ = 0.0;

  double goal_pos_range_ = 0.01;        // ゴールとみなす距離の閾値
  double goal_angle_range_ = 0.01;      // ゴールとみなす位置の閾値
  double finish_time_threshold_ = 0.3;  // 収束時間の判定用しきい値
  std::vector<double> error_{0.0, 0.0, 0.0};
  std::vector<double> prev_error_{0.0, 0.0, 0.0};
  std::vector<double> integral_error_{0.0, 0.0, 0.0};
  std::vector<double> derivative_error_{0.0, 0.0, 0.0};
  double dt_ = 0.0;
  int idx_ = 0;
  bool is_last_point_ = false;
  int finish_ = 0;
  rclcpp::Time last_out_of_range_time_ = rclcpp::Clock().now();
};
