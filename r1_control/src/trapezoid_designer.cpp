/**
 * @file trapezoid_designer.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("trapezoid_designer") {}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // plt::plot({1, 3, 2, 4});
  // plt::show();

  Eigen::Matrix2d A;
  A << 3, 2, 1, 2;

  Eigen::Vector2d b;
  b << 2, 0;

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

  std::cout << "solution = \n" << x << std::endl;
  std::cout << A(0, 0) * x(0) + A(0, 1) * x(1) << std::endl;

  rclcpp::shutdown();
  return 0;
}
