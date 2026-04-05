/**
 * @file r1_chassis_control_node.cpp
 * @author Yamaguchi Yudai(yudai.yy0804@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-02-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "magic_enum.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "r1_control/pos_follower.h"
#include "r1_control/trajectory_follower.h"
#include "r1_control/trajectory_planner.h"
#include "r1_msgs/msg/robot_move.hpp"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class R1ChassisControlNode : public rclcpp::Node
{
public:
  void declare_and_get_parameter(const std::string & name, double & value, double default_value)
  {
    this->declare_parameter<double>(name, default_value);
    this->get_parameter(name, value);
  }

  void declare_and_get_parameter(const std::string & name, int & value, int default_value)
  {
    this->declare_parameter<int>(name, default_value);
    this->get_parameter(name, value);
  }

  void declare_and_get_parameter(const std::string & name, bool & value, bool default_value)
  {
    this->declare_parameter<bool>(name, default_value);
    this->get_parameter(name, value);
  }

  void declare_and_get_parameter(
    const std::string & name, std::string & value, const std::string & default_value)
  {
    this->declare_parameter<std::string>(name, default_value);
    this->get_parameter(name, value);
  }

  R1ChassisControlNode() : Node("r1_chassis_control_node")
  {
    {
      std::vector<std::vector<double>> waypoints;
      for (int i = 0; i < 3; i++) {
        std::string param_name = "waypoints." + std::to_string(i);
        std::vector<double> wp;
        this->declare_parameter<std::vector<double>>(param_name, {0.0, 0.0});
        this->get_parameter(param_name, wp);
        if (wp.size() != 2) {
          RCLCPP_ERROR(this->get_logger(), "Waypoint %d must have exactly 2 elements (x, y)", i);
          rclcpp::shutdown();
          return;
        }
        waypoints.push_back({wp[0], wp[1]});
        RCLCPP_INFO(this->get_logger(), "Loaded waypoint %d: x=%f, y=%f", i, wp[0], wp[1]);
      }
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declare_and_get_parameter("timer_rate", timer_rate_, 100.0);
    declare_and_get_parameter("visualize_timer_rate", visualize_timer_rate_, 10.0);
    declare_and_get_parameter("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    control_dt_ = 1.0 / timer_rate_;

    // 足回り速度指令値
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    // 自己位置
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10,
      std::bind(&R1ChassisControlNode::odometry_callback, this, std::placeholders::_1));
    // waypointsのPublisher
    waypoints_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
    // target_poseのPublisher
    target_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
    // cmd_vel_arrowのPublisher
    cmd_vel_arrow_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/cmd_vel_arrow", 10);
    // robot_markerのPublisher
    robot_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
    // robot_trajectoryのPublisher
    robot_trajectory_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("/robot_trajectory", 10);
    // ACTのPublisher
    act_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/chassis_act_status", 10);
    // ACTのSubscription
    act_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/chassis_act_ref", 10,
      std::bind(&R1ChassisControlNode::act_callback, this, std::placeholders::_1));
    // robot_moveのSubscription
    robot_move_subscription_ = this->create_subscription<r1_msgs::msg::RobotMove>(
      "/robot_move", 10,
      std::bind(&R1ChassisControlNode::robot_move_callback, this, std::placeholders::_1));

    // chassis_error_tangentのPublisher
    chassis_error_tangent_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/chassis_error_tangent", 10);
    // chassis_error_lateralのPublisher
    chassis_error_lateral_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/chassis_error_lateral", 10);
    // chassis_error_thetaのPublisher
    chassis_error_theta_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/chassis_error_theta", 10);

    // パラメータの宣言と取得
    declare_and_get_parameter("act_filebase", act_filebase_, "");
    declare_and_get_parameter("zone", zone_, "red");

    // 経路生成のパラメータ
    for (int i = 0; i < 12; i++) {
      std::string inner_decel_center_pos_name = "inner_decel_center_pos." + std::to_string(i + 1);
      std::string outer_decel_center_pos_name = "outer_decel_center_pos." + std::to_string(i + 1);
      // 適当な初期値を代入
      this->declare_parameter<std::vector<double>>(
        inner_decel_center_pos_name, {100.0, 100.0, 0.0});
      this->declare_parameter<std::vector<double>>(
        outer_decel_center_pos_name, {100.0, 100.0, 0.0});
      std::vector<double> inner_decel_center_pos, outer_decel_center_pos;
      this->get_parameter(inner_decel_center_pos_name, inner_decel_center_pos);
      this->get_parameter(outer_decel_center_pos_name, outer_decel_center_pos);
      if (inner_decel_center_pos.size() != 3) {
        RCLCPP_FATAL(
          this->get_logger(), "inner_decel_center_pos.%d must have exactly 3 elements (x, y, yaw)",
          i);
        rclcpp::shutdown();
        return;
      }
      if (outer_decel_center_pos.size() != 3) {
        RCLCPP_FATAL(
          this->get_logger(), "outer_decel_center_pos.%d must have exactly 3 elements (x, y, yaw)",
          i);
        rclcpp::shutdown();
        return;
      }
      inner_decel_center_pos_.push_back(inner_decel_center_pos);
      outer_decel_center_pos_.push_back(outer_decel_center_pos);
    }
    declare_and_get_parameter("decel_height", decel_height_, 1.2);
    declare_and_get_parameter("decel_width", decel_width_, 1.2);
    declare_and_get_parameter("decel_speed", decel_speed_, 0.0);
    declare_and_get_parameter("collect_kfs_offset", collect_kfs_offset_, 0.0);
    // 経路追従のパラメータ
    declare_and_get_parameter("use_map", use_map_, true);
    declare_and_get_parameter("search_radius", search_radius_, 0.0);
    declare_and_get_parameter("lookahead_time", lookahead_time_, 0.3);
    declare_and_get_parameter("kp_pos_tangent_usual", kp_pos_tangent_usual_, 0.0);
    declare_and_get_parameter("ki_pos_tangent_usual", ki_pos_tangent_usual_, 0.0);
    declare_and_get_parameter("kd_pos_tangent_usual", kd_pos_tangent_usual_, 0.0);
    declare_and_get_parameter("kp_pos_tangent_goal", kp_pos_tangent_goal_, 0.0);
    declare_and_get_parameter("ki_pos_tangent_goal", ki_pos_tangent_goal_, 0.0);
    declare_and_get_parameter("kd_pos_tangent_goal", kd_pos_tangent_goal_, 0.0);
    declare_and_get_parameter("kp_pos_normal_usual", kp_pos_normal_usual_, 0.0);
    declare_and_get_parameter("ki_pos_normal_usual", ki_pos_normal_usual_, 0.0);
    declare_and_get_parameter("kd_pos_normal_usual", kd_pos_normal_usual_, 0.0);
    declare_and_get_parameter("kp_pos_normal_goal", kp_pos_normal_goal_, 0.0);
    declare_and_get_parameter("ki_pos_normal_goal", ki_pos_normal_goal_, 0.0);
    declare_and_get_parameter("kd_pos_normal_goal", kd_pos_normal_goal_, 0.0);
    declare_and_get_parameter("vel_i_limit", vel_i_limit_, 0.0);
    declare_and_get_parameter("vel_output_limit", vel_output_limit_, 0.0);
    declare_and_get_parameter("kp_angle_usual", kp_angle_usual_, 0.0);
    declare_and_get_parameter("ki_angle_usual", ki_angle_usual_, 0.0);
    declare_and_get_parameter("kd_angle_usual", kd_angle_usual_, 0.0);
    declare_and_get_parameter("kp_angle_goal", kp_angle_goal_, 0.0);
    declare_and_get_parameter("ki_angle_goal", ki_angle_goal_, 0.0);
    declare_and_get_parameter("kd_angle_goal", kd_angle_goal_, 0.0);
    declare_and_get_parameter("omega_i_limit", omega_i_limit_, 0.0);
    declare_and_get_parameter("omega_output_limit", omega_output_limit_, 0.0);
    declare_and_get_parameter("goal_pos_range", goal_pos_range_, 0.0);
    declare_and_get_parameter("goal_angle_range", goal_angle_range_, 0.0);
    declare_and_get_parameter("finish_time_threshold", finish_time_threshold_, 0.0);
    declare_and_get_parameter(
      "publish_robot_trajectory_dist_threshold", publish_robot_trajectory_dist_threshold_, 0.1);
    declare_and_get_parameter(
      "publish_robot_trajectory_angle_threshold", publish_robot_trajectory_angle_threshold_,
      5.0 * M_PI / 180.0);
    declare_and_get_parameter("enable_visualization", enable_visualization_, true);
    declare_and_get_parameter("arrow_scale", arrow_scale_, 0.2);

    // 経路生成、軌道追従関連のstd::vectorをresize
    traj_dt_.resize(ACT_N);
    traj_v_max_.resize(ACT_N);
    traj_a_max_.resize(ACT_N);
    traj_j_max_.resize(ACT_N);
    traj_omega_max_.resize(ACT_N);
    traj_x_wp_.resize(ACT_N);
    traj_y_wp_.resize(ACT_N);
    traj_theta_wp_.resize(ACT_N);
    traj_v_trans_wp_.resize(ACT_N);
    traj_planner_.resize(ACT_N);
    traj_follower_.resize(ACT_N);
    // TrajectoryPlannerのインスタンスを生成
    for (int i = 0; i < ACT_N; i++) {
      traj_planner_[i] = std::make_shared<TrajectoryPlanner>();
    }

    // 軌道を生成
    try {
      for (int i = 0; i < ACT_N; i++) {
        // csvを読み込み
        if (load_trajectory_csv(i) != 0) {
          throw std::runtime_error("Failed to load trajectory CSV");
        }
        // trajectoryを生成
        if (generate_trajectory(i) != 0) {
          throw std::runtime_error("Failed to generate trajectory");
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        this->get_logger(), "Exception caught during trajectory generation: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    for (int i = 0; i < ACT_N; i++) {
      traj_follower_[i] =
        std::make_shared<TrajectoryFollower>(traj_planner_[i], tf_buffer_, tf_listener_);
      traj_follower_[i]->set_param(
        use_map_, kp_pos_tangent_usual_, ki_pos_tangent_usual_, kd_pos_tangent_usual_,
        kp_pos_tangent_goal_, ki_pos_tangent_goal_, kd_pos_tangent_goal_, kp_pos_normal_usual_,
        ki_pos_normal_usual_, kd_pos_normal_usual_, kp_pos_normal_goal_, ki_pos_normal_goal_,
        kd_pos_normal_goal_, vel_i_limit_, vel_output_limit_, kp_angle_usual_, ki_angle_usual_,
        kd_angle_usual_, kp_angle_goal_, ki_angle_goal_, kd_angle_goal_, omega_i_limit_,
        omega_output_limit_, control_dt_, search_radius_, lookahead_time_, goal_pos_range_,
        goal_angle_range_, finish_time_threshold_);
    }

    pos_follower_ = std::make_shared<PosFollower>();
    // TODO: 現状traj_followerのPID制御とpos_followerのPID制御の内容・パラメータが一致していない可能性があるので確認する
    // pos_follower_->set_param(
    //   kp_pos_normal_, ki_pos_, kd_pos_, vel_i_limit_, kp_angle_, ki_angle_, kd_angle_, omega_i_limit_,
    //   control_dt_, goal_pos_range_, goal_angle_range_, finish_time_threshold_);

    act_step_ = ChassisAct::NONE;
    prev_act_step_ = ChassisAct::NONE;

    // timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_),
      std::bind(&R1ChassisControlNode::timer_callback, this));
    visualize_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / visualize_timer_rate_),
      std::bind(&R1ChassisControlNode::visualize_timer_callback, this));
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odometry_ = *msg;
    // RCLCPP_INFO(
    //   this->get_logger(), "Received odometry: x=%f, y=%f, theta=%f", odometry_.pose.pose.position.x,
    //   odometry_.pose.pose.position.y, tf2::getYaw(odometry_.pose.pose.orientation));
  }

  void act_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    act_step_ = static_cast<ChassisAct>(msg->data);
    std::string act_name{magic_enum::enum_name(act_step_)};
    // RCLCPP_INFO(this->get_logger(), "Received act step: %s", act_name.c_str());
  }

  int act_to_trajectory_index(ChassisAct act) const
  {
    switch (act) {
      case ChassisAct::ACT0_START:
      case ChassisAct::ACT0:
      case ChassisAct::ACT0_FINISH:
        return 0;
      case ChassisAct::ACT1_START:
      case ChassisAct::ACT1:
      case ChassisAct::ACT1_FINISH:
        return 1;
      case ChassisAct::ACT2_START:
      case ChassisAct::ACT2:
      case ChassisAct::ACT2_FINISH:
        return 2;
      case ChassisAct::ACT3_START:
      case ChassisAct::ACT3:
      case ChassisAct::ACT3_FINISH:
        return 3;
      case ChassisAct::NONE:
      default:
        return -1;
    }
  }

  void robot_move_callback(const r1_msgs::msg::RobotMove::SharedPtr msg)
  {
    ChassisAct act = static_cast<ChassisAct>(msg->act);
    std::string act_name{magic_enum::enum_name(act)};
    const int index = act_to_trajectory_index(act);
    if (index < 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid act in RobotMove: %d", msg->act);
      return;
    }
    bool is_inner = index == 1;
    bool is_outer = index == 2;
    std::vector<int> forest_order(msg->forest_order.begin(), msg->forest_order.end());
    // パラメータを読み込み
    if (load_trajectory_csv(index) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory CSV for %s", act_name.c_str());
      return;
    }

    std::vector<double> & x_wp = traj_x_wp_[index];
    std::vector<double> & y_wp = traj_y_wp_[index];
    std::vector<double> & v_trans_wp = traj_v_trans_wp_[index];

    // パラメータの書き換え
    for (int i = 0; i < (int)forest_order.size(); i++) {
      int forest = forest_order[i];
      // forest_orderが適切な値かの確認
      bool out_of_range = forest <= 0 || forest >= 13;
      bool invalid_value = forest == 5 || forest == 8;
      if (out_of_range || invalid_value) {
        RCLCPP_ERROR(this->get_logger(), "Invalid forest order: %d", forest_order[i]);
        return;
      }
      // zoneを考慮して判定する範囲の取得
      double center_x = 0.0, center_y = 0.0, rect_yaw = 0.0, offset_x = 0.0, offset_y = 0.0;
      if (is_inner) {
        center_x = inner_decel_center_pos_[forest - 1][0];
        center_y = inner_decel_center_pos_[forest - 1][1];
        rect_yaw = inner_decel_center_pos_[forest - 1][2];
      } else if (is_outer) {
        center_x = outer_decel_center_pos_[forest - 1][0];
        center_y = outer_decel_center_pos_[forest - 1][1];
        rect_yaw = outer_decel_center_pos_[forest - 1][2];
      }
      if (zone_ == "blue") {
        // zoneがblueのときはy軸を反転させる（yawも反転させる）
        center_x *= -1.0;
        rect_yaw = angle_normalize(M_PI - rect_yaw);
      }
      // TODO: ココらへんの処理はかなり怪しいので、赤ゾーンに対応するときに見直す。おそらく角度の扱いが怪しい
      // yは進行方向と同じ向きに対してオフセットを適用する
      if (is_inner && msg->kfs_mechanism_type[i] == "front_kfs") {
        offset_x = collect_kfs_offset_ * std::cos(rect_yaw);
        offset_y = collect_kfs_offset_ * std::sin(rect_yaw);
      } else if (is_outer && msg->kfs_mechanism_type[i] == "rear_kfs") {
        offset_x = collect_kfs_offset_ * std::cos(rect_yaw);
        offset_y = collect_kfs_offset_ * std::sin(rect_yaw);
      }
      // center_xとcenter_yにオフセットを適用する
      if (zone_ == "red") {
        center_x += offset_x;
        center_y += offset_y;
      } else {
        // 本当はcenter_xはプラスではなくマイナスのはずだが、何故か動かないので一旦プラス
        center_x += offset_x;
        center_y += offset_y;
      }

      RCLCPP_INFO(
        this->get_logger(), "Decel zone for forest %d in %s: center_x=%f, center_y=%f, rect_yaw=%f",
        forest, act_name.c_str(), center_x, center_y, rect_yaw);

      for (int j = 0; j < (int)v_trans_wp.size(); j++) {
        // 範囲内かどうか判定
        // 範囲内だった場合は減速する
        if (is_within_rotated_rectangle(
              x_wp[j], y_wp[j], center_x, center_y, rect_yaw, decel_width_, decel_height_)) {
          if (std::isfinite(v_trans_wp[j])) {
            v_trans_wp[j] = std::min(v_trans_wp[j], decel_speed_);
          } else {
            // 有限の値でなかったときは直接代入
            v_trans_wp[j] = decel_speed_;
          }
          // RCLCPP_INFO(
          //   this->get_logger(),
          //   "Decelerating at waypoint %d for forest %d in %s: x=%f, y=%f, v_trans_wp = %f", j,
          //   forest, act_name.c_str(), x_wp[j], y_wp[j], v_trans_wp[j]);
        }
      }
    }
    // 経路の生成
    if (generate_trajectory(index) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate trajectory for %s", act_name.c_str());
      return;
    }
    // act_step_を更新
    act_step_ = act;
    // ログを出力
    std::string log_msg = "Received RobotMove: act=" + act_name + ", forest_order=[";
    for (size_t i = 0; i < forest_order.size(); i++) {
      log_msg += std::to_string(forest_order[i]);
      if (i < forest_order.size() - 1) {
        log_msg += ", ";
      }
    }
    log_msg += "]";
    RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
  }

  void publish_path(int n)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    int inc = 20;  // 表示間隔。pathはデバッグ用に使用するので、点の数を間引く

    for (int i = 0;; i += inc) {
      if (i >= traj_planner_[n]->array_size_) {
        i = traj_planner_[n]->array_size_ - 1;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = traj_planner_[n]->x_[i];
      pose.pose.position.y = traj_planner_[n]->y_[i];
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, traj_planner_[n]->theta_[i]);  // roll, pitch, yaw
      q.normalize();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      path.poses.push_back(pose);

      if (i >= traj_planner_[n]->array_size_ - 1) {
        break;
      }
    }
    waypoints_publisher_->publish(path);
  }

  void update_target_pose(const WayPoint & waypoint)
  {
    latest_target_pose_.header.stamp = this->get_clock()->now();
    latest_target_pose_.header.frame_id = "map";
    latest_target_pose_.pose.position.x = waypoint.x;
    latest_target_pose_.pose.position.y = waypoint.y;
    latest_target_pose_.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, waypoint.theta);
    q.normalize();
    latest_target_pose_.pose.orientation.x = q.x();
    latest_target_pose_.pose.orientation.y = q.y();
    latest_target_pose_.pose.orientation.z = q.z();
    latest_target_pose_.pose.orientation.w = q.w();
    has_target_pose_ = true;
  }

  void publish_cmd_vel(geometry_msgs::msg::Twist _cmd_vel)
  {
    cmd_vel_ = _cmd_vel;
    cmd_vel_publisher_->publish(cmd_vel_);
  }

  void publish_error(std::vector<double> error)
  {
    std_msgs::msg::Float64 error_msg;

    error_msg.data = error[0];
    chassis_error_tangent_publisher_->publish(error_msg);

    error_msg.data = error[1];
    chassis_error_lateral_publisher_->publish(error_msg);

    error_msg.data = error[2];
    chassis_error_theta_publisher_->publish(error_msg);
  }

  void publish_robot_marker(void)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = "base_link";
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = marker.scale.z / 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 0.5;  // 不透明
    marker.color.r = 0.0;
    marker.color.g = 1.0;  // 緑色
    marker.color.b = 0.0;
    robot_marker_publisher_->publish(marker);
  }

  void publish_cmd_vel_arrow()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = "base_link";
    marker.ns = "arrow";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = marker.scale.z / 2.0;
    double marker_length = arrow_scale_ * std::hypot(cmd_vel_.linear.x, cmd_vel_.linear.y);
    marker.scale.x = marker_length;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    double cmd_vel_yaw = std::atan2(cmd_vel_.linear.y, cmd_vel_.linear.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, cmd_vel_yaw);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.color.a = 1.0;  // 不透明
    marker.color.r = 0.0;
    marker.color.g = 1.0;  // 緑色
    marker.color.b = 0.0;
    cmd_vel_arrow_publisher_->publish(marker);
  }

  void reset_robot_trajectory()
  {
    robot_trajectory_.header.stamp = this->get_clock()->now();
    robot_trajectory_.header.frame_id = "map";
    robot_trajectory_.poses.clear();
  }

  /**
   * @brief ロボットの軌道（map座標系）をpublishする。
   * 
   */
  void publish_robot_trajectory()
  {
    if (act_step_ == ChassisAct::NONE) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose_map;
    geometry_msgs::msg::PoseStamped pose_odom;
    pose_odom.header = odometry_.header;
    pose_odom.pose = odometry_.pose.pose;
    try {
      // odomからmapへのtf変換を行う。
      pose_map = tf_buffer_->transform(pose_odom, "map", tf2::durationFromSec(0.01));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Failed to transform odometry pose: %s",
        ex.what());
      return;
    }
    // robot_trajectory_.posesの要素数が1以上のときは、前回値と比較して、
    // 距離または角度のしきい値を超えている場合にのみ追加する

    if (!robot_trajectory_.poses.empty()) {
      const auto & last_pose_map = robot_trajectory_.poses.back();
      double dx = pose_map.pose.position.x - last_pose_map.pose.position.x;
      double dy = pose_map.pose.position.y - last_pose_map.pose.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double current_yaw = tf2::getYaw(pose_map.pose.orientation);
      double prev_yaw = tf2::getYaw(last_pose_map.pose.orientation);
      double yaw_diff = angle_diff(current_yaw, prev_yaw);
      bool is_distance_out_of_threshold = distance >= publish_robot_trajectory_dist_threshold_;
      bool is_angle_out_of_threshold =
        std::abs(yaw_diff) >= publish_robot_trajectory_angle_threshold_;
      if (is_distance_out_of_threshold == false && is_angle_out_of_threshold == false) {
        return;
      }
    }

    robot_trajectory_.poses.push_back(pose_map);
    robot_trajectory_publisher_->publish(robot_trajectory_);
  }

  void visualize_timer_callback()
  {
    if (!enable_visualization_) {
      return;
    }
    if (has_target_pose_) {
      latest_target_pose_.header.stamp = this->get_clock()->now();
      // RCLCPP_INFO(
      //   this->get_logger(), "Publishing target pose: x=%f, y=%f, theta=%f",
      //   latest_target_pose_.pose.position.x, latest_target_pose_.pose.position.y,
      //   tf2::getYaw(latest_target_pose_.pose.orientation));
      target_pose_publisher_->publish(latest_target_pose_);
    }
    publish_robot_marker();
    publish_cmd_vel_arrow();
    publish_robot_trajectory();
  }

  void timer_callback()
  {
    if (act_step_ == ChassisAct::ACT0_START) {
      act_step_ = ChassisAct::ACT0;
      traj_follower_[0]->reset();
      pos_follower_->reset();
      reset_robot_trajectory();
      // act0のpathをpublishする
      publish_path(0);
    } else if (act_step_ == ChassisAct::ACT0) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = traj_follower_[0]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      publish_error(traj_follower_[0]->get_error());
      // goal_range_以内に到達したらFINISHに遷移する
      if (traj_follower_[0]->is_finished()) {
        act_step_ = ChassisAct::ACT0_FINISH;
      }
    } else if (act_step_ == ChassisAct::ACT0_FINISH) {
    } else if (act_step_ == ChassisAct::ACT1_START) {
      act_step_ = ChassisAct::ACT1;
      traj_follower_[1]->reset();
      reset_robot_trajectory();
      // act1のpathをpublishする
      publish_path(1);
    } else if (act_step_ == ChassisAct::ACT1) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = traj_follower_[1]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      publish_error(traj_follower_[1]->get_error());
      // goal_range_以内に到達したらROTATEに遷移する
      if (traj_follower_[1]->is_finished()) {
        act_step_ = ChassisAct::ACT1_FINISH;
      }
    } else if (act_step_ == ChassisAct::ACT1_FINISH) {
    } else if (act_step_ == ChassisAct::ACT2_START) {
      act_step_ = ChassisAct::ACT2;
      traj_follower_[2]->reset();
      reset_robot_trajectory();
      // act2のpathをpublishする
      publish_path(2);
    } else if (act_step_ == ChassisAct::ACT2) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = traj_follower_[2]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      publish_error(traj_follower_[2]->get_error());
      // goal_range_以内に到達したらROTATEに遷移する
      if (traj_follower_[2]->is_finished()) {
        act_step_ = ChassisAct::ACT2_FINISH;
      }
    } else if (act_step_ == ChassisAct::ACT2_FINISH) {
    } else if (act_step_ == ChassisAct::ACT3_START) {
      act_step_ = ChassisAct::ACT3;
      traj_follower_[3]->reset();
      reset_robot_trajectory();
      // act3のpathをpublishする
      publish_path(3);
    } else if (act_step_ == ChassisAct::ACT3) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = traj_follower_[3]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      publish_error(traj_follower_[3]->get_error());
      // goal_range_以内に到達したらROTATEに遷移する
      if (traj_follower_[3]->is_finished()) {
        act_step_ = ChassisAct::ACT3_FINISH;
      }
    } else if (act_step_ == ChassisAct::ACT3_FINISH) {
    }
    // 現在のact_step_をpublishする
    std_msgs::msg::Int32 act_status_msg;
    act_status_msg.data = static_cast<int>(act_step_);
    act_publisher_->publish(act_status_msg);
    // ログを出力する
    if (act_step_ != prev_act_step_) {
      std::string act_name{magic_enum::enum_name(act_step_)};
      RCLCPP_INFO(this->get_logger(), "Current act step: %s", act_name.c_str());
      // act_stepにFINISHという文字が含まれていた場合は現在の位置も出力する
      if (act_name.find("FINISH") != std::string::npos) {
        geometry_msgs::msg::PoseStamped pose_map;
        geometry_msgs::msg::PoseStamped pose_odom;
        pose_odom.header = odometry_.header;
        pose_odom.pose = odometry_.pose.pose;
        try {
          // odomからmapへのtf変換を行う。
          pose_map = tf_buffer_->transform(pose_odom, "map", tf2::durationFromSec(0.01));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, "Failed to transform odometry pose: %s",
            ex.what());
          return;
        }
        // 現在の位置をログに出力する
        RCLCPP_INFO(
          this->get_logger(), "Current map: x = %.3f, y = %.3f, yaw = %.3f",
          pose_map.pose.position.x, pose_map.pose.position.y,
          tf2::getYaw(pose_map.pose.orientation));
        RCLCPP_INFO(
          this->get_logger(), "Current odom: x = %.3f, y = %.3f, yaw = %.3f",
          odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
          tf2::getYaw(odometry_.pose.pose.orientation));
      }
      // 前回の値を更新
      prev_act_step_ = act_step_;
    }
  }

  int load_trajectory_csv(int n)
  {
    // paramの読み込み
    double & dt = traj_dt_[n];
    double & v_max = traj_v_max_[n];
    double & a_max = traj_a_max_[n];
    double & j_max = traj_j_max_[n];
    double & omega_max = traj_omega_max_[n];
    char line[256], buff[256];
    // ファイル名が与えられていなかったらエラーにする
    if (act_filebase_ == "") {
      RCLCPP_FATAL(this->get_logger(), "actfilebase is not set");
      return 1;
    }
    // ファイルを開く
    std::string param_name = act_filebase_ + std::to_string(n) + "_robot_parameter.csv";
    FILE * fp = fopen(param_name.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", param_name.c_str());
      return 1;
    }

    // 最初の1行目はzoneなので無視
    if (fscanf(fp, "%255s", line) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read zone from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 2行目はdt
    // データを読むときはなぜか最初は空白にしないと読めない
    if (fscanf(fp, " dt,%lf", &dt) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read dt from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 3行目はv_max
    if (fscanf(fp, " v_max,%lf", &v_max) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read v_max from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 4行目はa_max
    if (fscanf(fp, " a_max,%lf", &a_max) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read a_max from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 5行目はj_max
    if (fscanf(fp, " j_max,%lf", &j_max) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read j_max from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 6行目はomega_max
    if (fscanf(fp, " omega_max,%lf", &omega_max) != 1) {
      RCLCPP_FATAL(this->get_logger(), "Failed to read omega_max from %s", param_name.c_str());
      fclose(fp);
      return 1;
    }
    // 7行目以降は無視
    fclose(fp);

    // 取得したデータをログに出力する
    // RCLCPP_INFO(
    //   this->get_logger(), "Parameters for ACT%d: dt=%f, v_max=%f, a_max=%f, j_max=%f, omega_max=%f",
    //   n, dt, v_max, a_max, j_max, omega_max);

    // waypointの読み込み
    std::vector<double> & x_wp = traj_x_wp_[n];
    std::vector<double> & y_wp = traj_y_wp_[n];
    std::vector<double> & theta_wp = traj_theta_wp_[n];
    std::vector<double> & v_trans_wp = traj_v_trans_wp_[n];
    x_wp.clear();
    y_wp.clear();
    theta_wp.clear();
    v_trans_wp.clear();
    // ファイル名が与えられていなかったらエラーにする
    if (act_filebase_ == "") {
      RCLCPP_FATAL(this->get_logger(), "act_filebase is not set");
      return 1;
    }
    std::string wp_filename = act_filebase_ + std::to_string(n) + "_waypoints.csv";
    // ファイルを開く
    fp = fopen(wp_filename.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", wp_filename.c_str());
      return 1;
    }
    int cnt = 0;
    int i = 0, j = 0;

    // カンマ区切りのデータを読む。データを読むときは空データも考慮する。
    while (fgets(line, sizeof(line), fp)) {
      bool has_x = false;
      bool has_y = false;
      double x = 0.0;
      double y = 0.0;
      double theta = std::numeric_limits<double>::infinity();
      double v_trans = std::numeric_limits<double>::infinity();
      cnt = 0;
      i = 0;
      j = 0;
      while (true) {
        char c = line[i];
        if (c == ',' || c == '\r' || c == '\n' || c == '\0') {
          buff[j] = '\0';
          if (j > 0) {
            switch (cnt) {
              case 0:
                x = atof(buff);
                has_x = true;
                break;
              case 1:
                y = atof(buff);
                has_y = true;
                break;
              case 2:
                theta = atof(buff);
                break;
              case 3:
                v_trans = atof(buff);
                break;
            }
          }
          j = 0;
          if (c == ',') {
            cnt++;
          }

          if (c == '\n' || c == '\0') {
            break;
          }
        } else {
          buff[j] = c;
          j++;
        }
        i++;
      }
      if (has_x && has_y) {
        x_wp.push_back(x);
        y_wp.push_back(y);
        theta_wp.push_back(theta);
        v_trans_wp.push_back(v_trans);
      }
    }

    fclose(fp);

    // ゾーンが青ゾーンの場合は、x座標を反転する
    if (zone_ == "blue") {
      for (int i = 0; i < (int)x_wp.size(); i++) {
        x_wp[i] = -x_wp[i];
      }
      // TODO: thetaも反転させたほうがいいかも
    }

    return 0;
  }

  int generate_trajectory(int n)
  {
    std::vector<double> & x_wp = traj_x_wp_[n];
    std::vector<double> & y_wp = traj_y_wp_[n];
    std::vector<double> & theta_wp = traj_theta_wp_[n];
    std::vector<double> & v_trans_wp = traj_v_trans_wp_[n];
    double & dt = traj_dt_[n];
    double & v_max = traj_v_max_[n];
    double & a_max = traj_a_max_[n];
    double & j_max = traj_j_max_[n];
    double & omega_max = traj_omega_max_[n];
    // trajectory plannerの計算
    auto ret =
      traj_planner_[n]->calc(x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max);

    // 読み込んだwaypointをログに出力する
    // for (int i = 0; i < (int)x_wp.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "Waypoint %d: x=%f, y=%f", i, x_wp[i], y_wp[i]);
    // }
    // for (int i = 0; i < (int)theta_wp.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "Theta Waypoint %d: theta=%f", i, theta_wp[i]);
    // }
    // for (int i = 0; i < (int)v_trans_wp.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "Velocity Waypoint %d: v_trans=%f", i, v_trans_wp[i]);
    // }

    // FILE * debug_fp = fopen(("debug_act" + std::to_string(n) + ".csv").c_str(), "w");
    // if (debug_fp == NULL) {
    //   RCLCPP_FATAL(this->get_logger(), "Failed to open debug file for ACT%d", n);
    //   return 1;
    // }

    // traj_planner_[n]->print_csv_trajectory(debug_fp);
    // fclose(debug_fp);

    for (int i = 0; i < (int)ret.size(); i++) {
      if (ret[i] != 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to calculate trajectory for ACT%d", n);
        return 1;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Generated trajectory for ACT%d", n);
    return 0;
  }

  // 足回り速度指令値
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // 自己位置
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  // waypointsのPublisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_publisher_;
  // target_poseのPublisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
  // cmd_vel_arrowのPublisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vel_arrow_publisher_;
  // robot_markerのPublisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_publisher_;
  // robot_trajectoryのPublisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_trajectory_publisher_;
  // ACTのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr act_publisher_;
  // ACTのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr act_subscription_;
  // robot_moveのSubscription
  rclcpp::Subscription<r1_msgs::msg::RobotMove>::SharedPtr robot_move_subscription_;
  // chassis_error_tangentのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr chassis_error_tangent_publisher_;
  // chassis_error_lateralのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr chassis_error_lateral_publisher_;
  // chassis_error_thetaのPublisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr chassis_error_theta_publisher_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr visualize_timer_;
  // tf関連
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::Twist cmd_vel_;
  std::string cmd_vel_topic_;
  bool has_target_pose_ = false;
  ChassisAct act_step_ = ChassisAct::NONE;
  ChassisAct prev_act_step_ = ChassisAct::NONE;
  // ロボットの軌道保管用
  nav_msgs::msg::Path robot_trajectory_;
  // filepath
  std::string act_filebase_;
  // zone
  std::string zone_;
  // 内回り/外回りで減速判定に使う長方形中心の座標 [x, y, yaw]
  // yaw=0 のときは map 座標系に平行で、yaw を与えるとその分だけ長方形が回転する
  std::vector<std::vector<double>> inner_decel_center_pos_;
  std::vector<std::vector<double>> outer_decel_center_pos_;
  // 減速判定用長方形のサイズ
  // width は長方形ローカル x 方向、height は長方形ローカル y 方向の長さ
  double decel_height_;
  double decel_width_;
  // KFS回収時のオフセット（front_kfsかrear_kfsのうち、遠い方に適応する）
  double collect_kfs_offset_;
  // 減速時の速度
  double decel_speed_;
  // trajectory_follwerのparameter
  // mapを使用するか（Lidarを使用するか）
  bool use_map_;
  double search_radius_;  // 経路追従のための探索半径
  double lookahead_time_;
  // 通常時の接線方向位置[m]制御のゲイン
  double kp_pos_tangent_usual_;
  double ki_pos_tangent_usual_;
  double kd_pos_tangent_usual_;
  // ゴール時の接線方向位置[m]制御のゲイン
  double kp_pos_tangent_goal_;
  double ki_pos_tangent_goal_;
  double kd_pos_tangent_goal_;
  // 通常時の法線方向位置[m]制御のゲイン
  double kp_pos_normal_usual_;
  double ki_pos_normal_usual_;
  double kd_pos_normal_usual_;
  // ゴール時の法線方向位置[m]制御のゲイン
  double kp_pos_normal_goal_;
  double ki_pos_normal_goal_;
  double kd_pos_normal_goal_;
  // 速度の積分項のリミット
  double vel_i_limit_;
  // 速度の出力リミット
  double vel_output_limit_;
  // 通常時の角度[rad]制御のゲイン
  double kp_angle_usual_;
  double ki_angle_usual_;
  double kd_angle_usual_;
  // ゴール時の角度[rad]制御のゲイン
  double kp_angle_goal_;
  double ki_angle_goal_;
  double kd_angle_goal_;
  // 角速度の積分項のリミット
  double omega_i_limit_;
  // 角速度の出力リミット
  double omega_output_limit_;
  // 制御の終了判定閾値
  double goal_pos_range_;
  double goal_angle_range_;
  double finish_time_threshold_;
  double timer_rate_;
  double visualize_timer_rate_;
  double control_dt_;
  // 軌道出力の距離のしきい値
  double publish_robot_trajectory_dist_threshold_;  //[m]
  // 軌道出力の角度のしきい値
  double publish_robot_trajectory_angle_threshold_;  //[rad]
  bool enable_visualization_;
  double arrow_scale_;

  // 経路生成用waypoint変数
  std::vector<double> traj_dt_;
  std::vector<double> traj_v_max_;
  std::vector<double> traj_a_max_;
  std::vector<double> traj_j_max_;
  std::vector<double> traj_omega_max_;
  std::vector<std::vector<double>> traj_x_wp_;
  std::vector<std::vector<double>> traj_y_wp_;
  std::vector<std::vector<double>> traj_theta_wp_;
  std::vector<std::vector<double>> traj_v_trans_wp_;

  // trajectory planner
  std::vector<std::shared_ptr<TrajectoryPlanner>> traj_planner_;
  // trajectory follower
  std::vector<std::shared_ptr<TrajectoryFollower>> traj_follower_;
  // pos follower
  std::shared_ptr<PosFollower> pos_follower_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1ChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
