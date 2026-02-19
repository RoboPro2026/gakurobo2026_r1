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

using namespace std::chrono_literals;

constexpr int ACT_N = 1;

constexpr int ACT_NONE = 0;
constexpr int ACT0_START = 10;
constexpr int ACT0 = 11;
constexpr int ACT0_FINISH = 12;

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
      act_traj_follower_[i] = std::make_shared<TrajectoryFollower>();
    }

    declare_and_get_parameter("act_param_base_filepath", act_param_base_filepath_, "");
    declare_and_get_parameter("act_wp_base_filepath", act_wp_base_filepath_, "");
    declare_and_get_parameter("zone", zone_, "red");
    declare_and_get_parameter("search_radius", search_radius_, 0.0);
    declare_and_get_parameter("kp", kp_, 0.0);
    declare_and_get_parameter("ki", ki_, 0.0);
    declare_and_get_parameter("kd", kd_, 0.0);
    declare_and_get_parameter("kff", kff_, 0.0);
    declare_and_get_parameter("goal_range", goal_range_, 0.0);

    for (int i = 0; i < ACT_N; i++) {
      generate_trajectory(i);
    }
    RCLCPP_INFO(this->get_logger(), "Generated trajectories for all ACTs");

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
    path.header.frame_id = "map";

    int inc = 10;  // 軌道の点を10点ごとに表示する

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

  void publish_cmd_vel_and_target_pose(Waypoint waypoint, geometry_msgs::msg::Twist cmd_vel)
  {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = this->get_clock()->now();
    target_pose.header.frame_id = "map";
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

  void timer_callback()
  {
    if (act_step_ == ACT0_START) {
      act_step_ = ACT0;
      act_traj_follower_[0]->reset();
      // act0のpathをpublishする
      publish_path(&act_traj_planner_[0]);
      RCLCPP_INFO(this->get_logger(), "Starting ACT0");
    } else if (act_step_ == ACT0) {
      std::pair<WayPoint, geometry_msgs::msg::Twist> ret = act_traj_follower_[0]->update(odometry_);
      publish_cmd_vel_and_target_pose(ret.first, ret.second);
      if (act_traj_follower_[0]->is_finished()) {
        act_step_ = ACT0_FINISH;
      }
    } else if (act_step_ == ACT0_FINISH) {
      RCLCPP_INFO(this->get_logger(), "Finished ACT0");
    }
    // 現在のact_step_をpublishする
    std_msgs::msg::Int32 act_status_msg;
    act_status_msg.data = act_step_;
    act_publisher_->publish(act_status_msg);
  }

  void generate_trajectory(int n)
  {
    // paramの読み込み
    double dt, v_max, a_max, j_max, omega_max;
    char line[256], buff[256];
    // ファイル名が与えられていなかったらエラーにする
    if (act_param_base_filepath_ == "") {
      RCLCPP_FATAL(this->get_logger(), "act_param_base_filepath is not set");
      return;
    }
    // ファイルを開く
    std::string param_name = act_param_base_filepath_ + std::to_string(n) + ".csv";
    FILE * fp = fopen(param_name.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", param_name.c_str());
      return;
    }

    // 最初の1行目はzoneなので無視
    fscanf(fp, "%s", line);
    // 2行目はdt
    fscanf(fp, "dt,%lf", &dt);
    // 3行目はv_max
    fscanf(fp, "v_max,%lf", &v_max);
    // 4行目はa_max
    fscanf(fp, "a_max,%lf", &a_max);
    // 5行目はj_max
    fscanf(fp, "j_max,%lf", &j_max);
    // 6行目はomega_max
    fscanf(fp, "omega_max,%lf", &omega_max);
    // 7行目以降は無視
    fclose(fp);

    // waypointの読み込み
    std::vector<double> x_wp, y_wp;
    std::vector<std::pair<int, double>> theta_wp, v_trans_wp;
    // ファイル名が与えられていなかったらエラーにする
    if (act_wp_base_filepath_ == "") {
      RCLCPP_FATAL(this->get_logger(), "act_wp_base_filepath is not set");
      return;
    }
    std::string wp_filename = act_wp_base_filepath_ + std::to_string(n) + ".csv";
    // ファイルを開く
    fp = fopen(wp_filename.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", wp_filename.c_str());
      return;
    }
    int cnt = 0;
    int theta_wp_cnt = 0;
    int v_trans_wp_cnt = 0;
    int i = 0, j = 0;

    // カンマ区切りのデータを読む。データを読むときは空データも考慮する。
    while (fgets(line, sizeof(line), fp)) {
      cnt = 0;
      i = 0;
      j = 0;
      while (1) {
        char c = line[i];
        if (c == ',' || c == '\n' || c == '\0') {
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
                theta_wp.emplace_back(theta_wp_cnt, atof(buff));
                theta_wp_cnt++;
                break;
              case 3:
                v_trans_wp.emplace_back(v_trans_wp_cnt, atof(buff));
                v_trans_wp_cnt++;
                break;
            }
          }
          j = 0;
          if (c == ',') {
            cnt++;
          } else {
            cnt = 0;
          }
        } else {
          buff[j] = c;
          j++;
        }
        i++;
      }
    }

    fclose(fp);

    // trajectory plannerの計算
    act_traj_planner_[n]->calc(
      x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max);

    RCLCPP_INFO(this->get_logger(), "Generated trajectory for ACT%d", n);
  }

  // 足回り速度指令値
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // 自己位置
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  // waypointsのPublisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_publisher_;
  // target_poseのPublisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
  // ACTのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr act_publisher_;
  // ACTのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr act_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  int act_step_ = ACT_NONE;
  // filepath
  std::string act_param_base_filepath_;
  std::string act_wp_base_filepath_;
  // zone
  std::string zone_;
  // trajectory_follwerのparameter
  double search_radius_;  // 経路追従のための探索半径
  double kp_;
  double ki_;
  double kd_;
  double kff_;
  double goal_range_;

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