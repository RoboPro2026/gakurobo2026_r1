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
#include <complex>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "r1_util/r1_util.h"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_dummy_odometry_node")
  {
    this->declare_parameter<double>("timer_rate", 100.0);
    timer_rate_ = this->get_parameter("timer_rate").as_double();
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&MyNode::target_pose_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&MyNode::timer_callback, this));

    this->declare_parameter<double>("tau", 0.5);
    tau_ = this->get_parameter("tau").as_double();
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_ = *msg; }

  void timer_callback()
  {
    const double dt = 1.0 / timer_rate_;
    double x = lpf(pose_.pose.position.x, prev_x_, tau_, dt);
    double y = lpf(pose_.pose.position.y, prev_y_, tau_, dt);
    double z = 0;
    double theta = lpf(tf2::getYaw(pose_.pose.orientation), prev_theta_, tau_, dt);

    prev_x_ = x;
    prev_y_ = y;
    prev_theta_ = theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    // cmd_vel_に追従するオドメトリを生成してpublishする
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odometry_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);

    RCLCPP_INFO(this->get_logger(), "Published odometry: x=%.3f, y=%.3f, z=%.3f", x, y, z);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 速度指令値
  geometry_msgs::msg::PoseStamped pose_;

  // 時定数
  double timer_rate_;
  double tau_;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  double prev_theta_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
