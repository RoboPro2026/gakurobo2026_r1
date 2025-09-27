/**
 * @file r1_mecanum_node.cpp
 * @author Yamaguchi Yudai
 * @brief メカナムホイールの順運動学を計算するノード
 * @version 0.1
 * @date 2025-09-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("r1_mecanum_node") {}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}