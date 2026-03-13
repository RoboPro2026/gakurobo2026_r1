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
#include <complex>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "r1_control/pos_follower.h"
#include "r1_control/trajectory_follower.h"
#include "r1_control/trajectory_planner.h"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
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
    declare_and_get_parameter("timer_rate", timer_rate_, 100.0);
    declare_and_get_parameter("visualize_timer_rate", visualize_timer_rate_, 10.0);
    control_dt_ = 1.0 / timer_rate_;

    // 足回り速度指令値
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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

    act_traj_planner_.resize(ACT_N);
    act_traj_follower_.resize(ACT_N);

    for (int i = 0; i < ACT_N; i++) {
      act_traj_planner_[i] = std::make_shared<TrajectoryPlanner>();
    }

    declare_and_get_parameter("act_filebase", act_filebase_, "");
    declare_and_get_parameter("zone", zone_, "red");
    declare_and_get_parameter("search_radius", search_radius_, 0.0);
    declare_and_get_parameter("kp_pos", kp_pos_, 0.0);
    declare_and_get_parameter("ki_pos", ki_pos_, 0.0);
    declare_and_get_parameter("kd_pos", kd_pos_, 0.0);
    declare_and_get_parameter("kff_pos", kff_pos_, 0.0);
    declare_and_get_parameter("vel_limit", vel_limit_, 0.0);
    declare_and_get_parameter("kp_angle", kp_angle_, 0.0);
    declare_and_get_parameter("ki_angle", ki_angle_, 0.0);
    declare_and_get_parameter("kd_angle", kd_angle_, 0.0);
    declare_and_get_parameter("kff_angle", kff_angle_, 0.0);
    declare_and_get_parameter("omega_limit", omega_limit_, 0.0);
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

    try {
      for (int i = 0; i < ACT_N; i++) {
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
      act_traj_follower_[i] = std::make_shared<TrajectoryFollower>(act_traj_planner_[i].get());
      act_traj_follower_[i]->set_param(
        kp_pos_, ki_pos_, kd_pos_, kff_pos_, vel_limit_, kp_angle_, ki_angle_, kd_angle_,
        kff_angle_, omega_limit_, control_dt_, search_radius_, goal_pos_range_, goal_angle_range_,
        finish_time_threshold_);
    }

    pos_follower_ = std::make_shared<PosFollower>();
    pos_follower_->set_param(
      kp_pos_, ki_pos_, kd_pos_, vel_limit_, kp_angle_, ki_angle_, kd_angle_, omega_limit_,
      control_dt_, goal_pos_range_, goal_angle_range_, finish_time_threshold_);

    act_step_ = ACT_NONE;

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
    act_step_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received act step: %d", act_step_);
  }

  void publish_path(int n)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    int inc = 20;  // 表示間隔。pathはデバッグ用に使用するので、点の数を間引く

    for (int i = 0;; i += inc) {
      if (i >= act_traj_planner_[n]->array_size_) {
        i = act_traj_planner_[n]->array_size_ - 1;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = act_traj_planner_[n]->x_[i];
      pose.pose.position.y = act_traj_planner_[n]->y_[i];
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, act_traj_planner_[n]->theta_[i]);  // roll, pitch, yaw
      q.normalize();
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      path.poses.push_back(pose);

      if (i >= act_traj_planner_[n]->array_size_ - 1) {
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
    if (!is_act_running()) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose_map;
    geometry_msgs::msg::PoseStamped pose_odom;
    pose_odom.header = odometry_.header;
    pose_odom.pose = odometry_.pose.pose;
    try {
      // odomからmapへのtf変換を行う。
      pose_map = tf_buffer_.transform(pose_odom, "map", tf2::durationFromSec(0.01));
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

  bool is_act_running() const
  {
    return act_step_ == ACT0 || act_step_ == ACT1 || act_step_ == ACT2;
  }

  void visualize_timer_callback()
  {
    if (!enable_visualization_) {
      return;
    }
    if (has_target_pose_) {
      latest_target_pose_.header.stamp = this->get_clock()->now();
      target_pose_publisher_->publish(latest_target_pose_);
    }
    publish_robot_marker();
    publish_cmd_vel_arrow();
    publish_robot_trajectory();
  }

  void timer_callback()
  {
    if (act_step_ == ACT0_START) {
      act_step_ = ACT0;
      act_traj_follower_[0]->reset();
      pos_follower_->reset();
      reset_robot_trajectory();
      // act0のpathをpublishする
      publish_path(0);
      RCLCPP_INFO(this->get_logger(), "Starting ACT0");
    } else if (act_step_ == ACT0) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[0]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      // goal_range_以内に到達したらFINISHに遷移する
      if (act_traj_follower_[0]->is_finished()) {
        act_step_ = ACT0_FINISH;
        RCLCPP_INFO(this->get_logger(), "Finished ACT0_MOVE");
        RCLCPP_INFO(
          this->get_logger(), "x = %.3f, y = %.3f, yaw = %.3f", odometry_.pose.pose.position.x,
          odometry_.pose.pose.position.y, tf2::getYaw(odometry_.pose.pose.orientation));
      }
    } else if (act_step_ == ACT0_FINISH) {
    } else if (act_step_ == ACT1_START) {
      act_step_ = ACT1;
      act_traj_follower_[1]->reset();
      reset_robot_trajectory();
      // act1のpathをpublishする
      publish_path(1);
      RCLCPP_INFO(this->get_logger(), "Starting ACT1");
    } else if (act_step_ == ACT1) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[1]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      // goal_range_以内に到達したらROTATEに遷移する
      if (act_traj_follower_[1]->is_finished()) {
        act_step_ = ACT1_FINISH;
        RCLCPP_INFO(this->get_logger(), "Finished ACT1");
        RCLCPP_INFO(
          this->get_logger(), "x = %.3f, y = %.3f, yaw = %.3f", odometry_.pose.pose.position.x,
          odometry_.pose.pose.position.y, tf2::getYaw(odometry_.pose.pose.orientation));
      }
    } else if (act_step_ == ACT1_FINISH) {
    } else if (act_step_ == ACT2_START) {
      act_step_ = ACT2;
      act_traj_follower_[2]->reset();
      reset_robot_trajectory();
      // act2のpathをpublishする
      publish_path(2);
      RCLCPP_INFO(this->get_logger(), "Starting ACT2");
    } else if (act_step_ == ACT2) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[2]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      update_target_pose(ret.first);
      publish_cmd_vel(ret.second);
      // goal_range_以内に到達したらROTATEに遷移する
      if (act_traj_follower_[2]->is_finished()) {
        act_step_ = ACT2_FINISH;
        RCLCPP_INFO(this->get_logger(), "Finished ACT2");
      }
    } else if (act_step_ == ACT2_FINISH) {
    }
    // 現在のact_step_をpublishする
    std_msgs::msg::Int32 act_status_msg;
    act_status_msg.data = act_step_;
    act_publisher_->publish(act_status_msg);
  }

  int generate_trajectory(int n)
  {
    // paramの読み込み
    double dt, v_max, a_max, j_max, omega_max;
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
    std::vector<double> x_wp, y_wp;
    std::vector<std::pair<int, double>> theta_wp, v_trans_wp;
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
    int i = 0, j = 0, line_cnt = 0;

    // カンマ区切りのデータを読む。データを読むときは空データも考慮する。
    while (fgets(line, sizeof(line), fp)) {
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
                x_wp.push_back(atof(buff));
                break;
              case 1:
                y_wp.push_back(atof(buff));
                break;
              case 2:
                theta_wp.emplace_back(line_cnt, atof(buff));
                break;
              case 3:
                v_trans_wp.emplace_back(line_cnt, atof(buff));
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
      line_cnt++;
    }

    fclose(fp);

    // 読み込んだwaypointをログに出力する
    // for (int i = 0; i < x_wp.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "Waypoint %d: x=%f, y=%f", i, x_wp[i], y_wp[i]);
    // }
    // for (int i = 0; i < theta_wp.size(); i++) {
    //   RCLCPP_INFO(
    //     this->get_logger(), "Theta Waypoint %d: index=%d, theta=%f", i, theta_wp[i].first,
    //     theta_wp[i].second);
    // }
    // for (int i = 0; i < v_trans_wp.size(); i++) {
    //   RCLCPP_INFO(
    //     this->get_logger(), "Velocity Waypoint %d: index=%d, v_trans=%f", i, v_trans_wp[i].first,
    //     v_trans_wp[i].second);
    // }

    // FILE * debug_fp = fopen(("debug_act" + std::to_string(n) + ".csv").c_str(), "w");
    // if (debug_fp == NULL) {
    //   RCLCPP_FATAL(this->get_logger(), "Failed to open debug file for ACT%d", n);
    //   return 1;
    // }

    // act_traj_planner_[n]->print_csv_trajectory(debug_fp);
    // fclose(debug_fp);

    // ゾーンが青ゾーンの場合は、x座標を反転する
    if (zone_ == "blue") {
      for (int i = 0; i < (int)x_wp.size(); i++) {
        x_wp[i] = -x_wp[i];
      }
      // TODO: thetaも反転させたほうがいいかも
      // for (int i = 0; i < theta_wp.size(); i++) {
      //   theta_wp[i].second = -theta_wp[i].second;
      // }
    }

    // trajectory plannerの計算
    auto ret = act_traj_planner_[n]->calc(
      x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max);

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
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr visualize_timer_;
  // tf関連
  tf2_ros::Buffer tf_buffer_{this->get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::PoseStamped latest_target_pose_;
  geometry_msgs::msg::Twist cmd_vel_;
  bool has_target_pose_ = false;
  int act_step_ = ACT_NONE;
  // ロボットの軌道保管用
  nav_msgs::msg::Path robot_trajectory_;
  // filepath
  std::string act_filebase_;
  // zone
  std::string zone_;
  // trajectory_follwerのparameter
  double search_radius_;  // 経路追従のための探索半径
  // 位置[m]制御のゲイン
  double kp_pos_;
  double ki_pos_;
  double kd_pos_;
  double kff_pos_;
  double vel_limit_;
  // 角度[rad]制御のゲイン
  double kp_angle_;
  double ki_angle_;
  double kd_angle_;
  double kff_angle_;
  double omega_limit_;
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

  // trajectory planner
  std::vector<std::shared_ptr<TrajectoryPlanner>> act_traj_planner_;
  // trajectory follower
  std::vector<std::shared_ptr<TrajectoryFollower>> act_traj_follower_;
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
