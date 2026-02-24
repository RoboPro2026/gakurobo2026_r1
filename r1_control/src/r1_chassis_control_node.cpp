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
#include "r1_control/trajectory_follower.h"
#include "r1_control/trajectory_planner.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

constexpr int ACT_N = 3;

constexpr int ACT_NONE = 0;
constexpr int ACT0_START = 1;
constexpr int ACT0 = 2;
constexpr int ACT0_FINISH = 3;
constexpr int ACT1_START = 11;
constexpr int ACT1 = 12;
constexpr int ACT1_FINISH = 13;
constexpr int ACT2_START = 21;
constexpr int ACT2 = 22;
constexpr int ACT2_FINISH = 23;

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

  void declare_and_get_parameter(
    const std::string & name, std::string & value, const std::string & default_value)
  {
    this->declare_parameter<std::string>(name, default_value);
    this->get_parameter(name, value);
  }

  R1ChassisControlNode() : Node("r1_chassis_control_node")
  {
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
    // robot_markerのPublisher
    robot_marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);

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
    declare_and_get_parameter("kp_angle", kp_angle_, 0.0);
    declare_and_get_parameter("ki_angle", ki_angle_, 0.0);
    declare_and_get_parameter("kd_angle", kd_angle_, 0.0);
    declare_and_get_parameter("kff_angle", kff_angle_, 0.0);
    declare_and_get_parameter("goal_pos_range", goal_pos_range_, 0.0);
    declare_and_get_parameter("goal_angle_range", goal_angle_range_, 0.0);
    declare_and_get_parameter("finish_time_threshold", finish_time_threshold_, 0.0);

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
        kp_pos_, ki_pos_, kd_pos_, kff_pos_, kp_angle_, ki_angle_, kd_angle_, kff_angle_, DT,
        search_radius_, goal_pos_range_, goal_angle_range_, finish_time_threshold_);
    }
    RCLCPP_INFO(this->get_logger(), "Generated trajectories for all ACTs");

    act_step_ = ACT_NONE;

    // timer
    timer_ = this->create_wall_timer(10ms, std::bind(&R1ChassisControlNode::timer_callback, this));
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
    path.header.frame_id = "odom";
    // NOTE: デバッグのためにodomにしている
    // path.header.frame_id = "odom";

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

  void publish_cmd_vel_and_target_pose(
    const WayPoint & waypoint, const geometry_msgs::msg::Twist & cmd_vel)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = this->get_clock()->now();
    // デバッグのためにmapからodomに変更
    // target_pose.header.frame_id = "map";
    target_pose.header.frame_id = "odom";
    target_pose.pose.position.x = waypoint.x;
    target_pose.pose.position.y = waypoint.y;
    target_pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, waypoint.theta);
    q.normalize();
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    target_pose_publisher_->publish(target_pose);
    cmd_vel_publisher_->publish(cmd_vel);
  }

  void publish_robot_marker(void)
  {
    auto & odom = odometry_;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->get_clock()->now();
    // NOTE: base_linkだとうまく動かないのでodomにしている
    marker.header.frame_id = "odom";
    // marker.header.frame_id = "base_link";
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;
    marker.pose.position.x = odom.pose.pose.position.x;
    marker.pose.position.y = odom.pose.pose.position.y;
    // marker.pose.position.x = 0.0;
    // marker.pose.position.y = 0.0;
    marker.pose.position.z = marker.scale.z / 2.0;
    marker.pose.orientation.x = odom.pose.pose.orientation.x;
    marker.pose.orientation.y = odom.pose.pose.orientation.y;
    marker.pose.orientation.z = odom.pose.pose.orientation.z;
    marker.pose.orientation.w = odom.pose.pose.orientation.w;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;  // 不透明
    marker.color.r = 0.0;
    marker.color.g = 1.0;  // 緑色
    marker.color.b = 0.0;
    robot_marker_publisher_->publish(marker);

    // ロボットの0度方向(前方)を示す線を描画
    // tf2::Quaternion q(
    //   odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
    //   odom.pose.pose.orientation.w);
    // double roll = 0.0, pitch = 0.0, yaw = 0.0;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // constexpr double heading_length = 0.7;  // [m]
    // visualization_msgs::msg::Marker heading;
    // heading.header = marker.header;
    // heading.ns = "robot_heading";
    // heading.id = 1;
    // heading.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // heading.action = visualization_msgs::msg::Marker::ADD;
    // heading.pose.orientation.w = 1.0;  // points are in the header frame
    // heading.scale.x = 0.03;            // line width
    // heading.color.a = 1.0;
    // heading.color.r = 1.0;  // 赤
    // heading.color.g = 0.0;
    // heading.color.b = 0.0;

    // geometry_msgs::msg::Point p0;
    // p0.x = odom.pose.pose.position.x;
    // p0.y = odom.pose.pose.position.y;
    // p0.z = marker.scale.z;  // cube上面付近

    // geometry_msgs::msg::Point p1;
    // p1.x = p0.x + heading_length * std::cos(yaw);
    // p1.y = p0.y + heading_length * std::sin(yaw);
    // p1.z = p0.z;

    // heading.points.push_back(p0);
    // heading.points.push_back(p1);
    // robot_marker_publisher_->publish(heading);
  }

  void timer_callback()
  {
    if (act_step_ == ACT0_START) {
      act_step_ = ACT0;
      act_traj_follower_[0]->reset();
      // act0のpathをpublishする
      publish_path(0);
      RCLCPP_INFO(this->get_logger(), "Starting ACT0");
    } else if (act_step_ == ACT0) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[0]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      publish_cmd_vel_and_target_pose(ret.first, ret.second);
      // goal_range_以内に到達したらFINISHに遷移する
      if (act_traj_follower_[0]->is_finished()) {
        act_step_ = ACT0_FINISH;
        RCLCPP_INFO(this->get_logger(), "Finished ACT0");
        RCLCPP_INFO(
          this->get_logger(), "x = %.3f, y = %.3f, yaw = %.3f", odometry_.pose.pose.position.x,
          odometry_.pose.pose.position.y, tf2::getYaw(odometry_.pose.pose.orientation));
      }
    } else if (act_step_ == ACT0_FINISH) {
    } else if (act_step_ == ACT1_START) {
      act_step_ = ACT1;
      act_traj_follower_[1]->reset();
      // act1のpathをpublishする
      publish_path(1);
      RCLCPP_INFO(this->get_logger(), "Starting ACT1");
    } else if (act_step_ == ACT1) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[1]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      publish_cmd_vel_and_target_pose(ret.first, ret.second);
      // goal_range_以内に到達したらFINISHに遷移する
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
      // act2のpathをpublishする
      publish_path(2);
      RCLCPP_INFO(this->get_logger(), "Starting ACT2");
    } else if (act_step_ == ACT2) {
      // 軌道追従の計算を行う
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[2]->update(odometry_);
      // 指令値と目標のwaypointをpublishする
      publish_cmd_vel_and_target_pose(ret.first, ret.second);
      // goal_range_以内に到達したらFINISHに遷移する
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
    // robot_markerをpublishする
    publish_robot_marker();
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
  // robot_markerのPublisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_publisher_;
  // ACTのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr act_publisher_;
  // ACTのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr act_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  int act_step_ = ACT_NONE;
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
  // 角度[rad]制御のゲイン
  double kp_angle_;
  double ki_angle_;
  double kd_angle_;
  double kff_angle_;
  // 制御の終了判定閾値
  double goal_pos_range_;
  double goal_angle_range_;
  double finish_time_threshold_;
  static constexpr double DT = 0.01;  //[s]

  // trajectory planner
  std::vector<std::shared_ptr<TrajectoryPlanner>> act_traj_planner_;
  // trajectory follower
  std::vector<std::shared_ptr<TrajectoryFollower>> act_traj_follower_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1ChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
