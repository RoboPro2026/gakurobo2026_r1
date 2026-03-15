/**
 * @file r1_laser_filter_node.cpp
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief R1用のlaser_filter_node。内積を求めて、cosの値がしきい値以上のときは、そのscanを除外する。
 * @ref 参考：https://x.com/DoradoraRobot/status/1984595988246643083?s=20 
 * @version 0.1
 * @date 2026-03-15
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
#include "sensor_msgs/msg/laser_scan.hpp"
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
  MyNode() : Node("r1_laser_filter_node")
  {
    // パラメータを宣言
    this->declare_parameter<double>("threshold", 0.8);
    this->declare_parameter<std::string>("filtered_scan_topic", "/filtered_scan");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    threshold_ = this->get_parameter("threshold").as_double();
    filtered_scan_topic_ = this->get_parameter("filtered_scan_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();

    filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      filtered_scan_topic_, rclcpp::SensorDataQoS());
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MyNode::scan_callback, this, std::placeholders::_1));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto filtered_scan = *msg;
    for (size_t i = 0; i < msg->ranges.size() - 1; i++) {
      // scan[i]とscan[i+1]が有限値であることを確認
      if (std::isfinite(msg->ranges[i]) == false || std::isfinite(msg->ranges[i + 1]) == false) {
        continue;
      }

      // scan[i]とscan[i+1]の内積からcosの値を求める
      double theta1 = msg->angle_min + i * msg->angle_increment;
      double theta2 = msg->angle_min + (i + 1) * msg->angle_increment;
      double x1 = msg->ranges[i] * std::cos(theta1);
      double y1 = msg->ranges[i] * std::sin(theta1);
      double x2 = msg->ranges[i + 1] * std::cos(theta2);
      double y2 = msg->ranges[i + 1] * std::sin(theta2);
      double dot = x1 * x2 + y1 * y2;
      double norm1 = std::sqrt(x1 * x1 + y1 * y1);
      double norm2 = std::sqrt(x2 * x2 + y2 * y2);
      double cos_value = dot / (norm1 * norm2);
      // 内積から求めたcosの値がしきい値以上のときであれば、scan[i]は除外する
      // 本来は90度に近いときが良いscanなので、cosの値が小さい時(0度や180度に近いとき)を除外する
      if (std::abs(cos_value) >= threshold_) {
        // scan[i]を除外するために、scan[i]を無限大にする
        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
    filtered_scan_publisher_->publish(filtered_scan);
  }
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  double threshold_ = 0.8;
  std::string filtered_scan_topic_;
  std::string scan_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
