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

// TODO: スプライン補完で計算した関数の最近傍を見つけるアルゴリズムを書く

#include <chrono>
#include <complex>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

int ACT_NONE = 0;
int ACT0_START = 10;
int ACT0 = 11;
int ACT0_FINISH = 12;

class R1ChassisControlNode : public rclcpp::Node
{
public:
  R1ChassisControlNode() : Node("r1_chassis_control_node")
  {
    // 足回り速度指令値
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // 自己位置
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10,
      std::bind(&R1ChassisControlNode::odometry_callback, this, std::placeholders::_1));
    // ACTのPublisher
    act_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/chassis_act_status", 10);
    // ACTのSubscription
    act_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/chassis_act_ref", 10,
      std::bind(&R1ChassisControlNode::act_callback, this, std::placeholders::_1));
    // timer
    timer_ = this->create_wall_timer(10ms, std::bind(&R1ChassisControlNode::timer_callback, this));

    this->declare_parameter<std::string>("act0_param_filepath", "");
    this->declare_parameter<std::string>("act0_wp_filepath", "");
    this->get_parameter("act0_param_filepath", act0_param_filepath_);
    this->get_parameter("act0_wp_filepath", act0_wp_filepath_);

    generate_trajectory();
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

  void timer_callback()
  {
    if (act_step_ == ACT0_START) {
      act_step_ = ACT0;
      RCLCPP_INFO(this->get_logger(), "Starting ACT0");
    } else if (act_step_ == ACT0) {
      // TODO: 位置制御の処理を書く。PDとか
    } else if (act_step_ == ACT0_FINISH) {
      RCLCPP_INFO(this->get_logger(), "Finished ACT0");
    }
    // 現在のact_step_をpublishする
    std_msgs::msg::Int32 act_status_msg;
    act_status_msg.data = act_step_;
    act_publisher_->publish(act_status_msg);
  }

  void generate_trajectory(void)
  {
    // paramの読み込み
    double dt, v_max, a_max, j_max, omega_max;
    char line[256], buff[256];
    FILE * fp = fopen(act0_param_filepath_.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", act0_param_filepath_.c_str());
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
    fp = fopen(act0_wp_filepath_.c_str(), "r");
    if (fp == NULL) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", act0_wp_filepath_.c_str());
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
    act0_traj_planner_.calc(x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max);
  }

  // 足回り速度指令値
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // 自己位置
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  // ACTのPublisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr act_publisher_;
  // ACTのSubscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr act_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  // オドメトリ
  nav_msgs::msg::Odometry odometry_;
  int act_step_ = ACT_NONE;
  // filepath
  std::string act0_param_filepath_;
  std::string act0_wp_filepath_;

  // trajectory planner
  TrajectoryPlanner act0_traj_planner_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<R1ChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}