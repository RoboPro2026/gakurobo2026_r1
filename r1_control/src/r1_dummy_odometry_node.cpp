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

#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_dummy_odometry_node")
  {
    this->declare_parameter<double>("timer_rate", 100.0);
    this->declare_parameter<double>("tau", 0.5);

    timer_rate_ = this->get_parameter("timer_rate").as_double();
    tau_ = this->get_parameter("tau").as_double();

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    target_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&MyNode::target_pose_callback, this, std::placeholders::_1));
    initialpose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&MyNode::initialpose_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&MyNode::timer_callback, this));

    publish_map_to_odom_tf();
  }

private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_ = *msg; }

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    pose_.header = msg->header;
    pose_.pose = msg->pose.pose;
    prev_x_ = pose_.pose.position.x;
    prev_y_ = pose_.pose.position.y;
    prev_theta_ = tf2::getYaw(pose_.pose.orientation);
    RCLCPP_INFO(
      this->get_logger(), "Reset dummy odometry to x=%.3f, y=%.3f, yaw=%.3f", prev_x_, prev_y_,
      prev_theta_);
  }

  void publish_map_to_odom_tf()
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(map_to_odom);
  }

  void timer_callback()
  {
    const double dt = 1.0 / timer_rate_;
    const double x = lpf(pose_.pose.position.x, prev_x_, tau_, dt);
    const double y = lpf(pose_.pose.position.y, prev_y_, tau_, dt);
    const double theta = lpf(tf2::getYaw(pose_.pose.orientation), prev_theta_, tau_, dt);

    prev_x_ = x;
    prev_y_ = y;
    prev_theta_ = theta;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odometry_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Published dummy odometry: x=%.3f, y=%.3f", x,
      y);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped pose_;

  double timer_rate_ = 100.0;
  double tau_ = 0.5;
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
