/**
 * @file test_minimum_jerk.cpp
 * @author Yudai Yamaguchi
 * @brief 5次の最小躍度曲線
 * @version 0.1
 * @date 2025-12-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdio>

#include "r1_control/minimum_jerk.h"
#include "rclcpp/rclcpp.hpp"

void test_minimum_jerk()
{
  // パラメータを設定
  const double x_start = 5.0;
  const double x_end = 10.0;
  const double t_start = 1.0;
  const double t_end = 2.0;

  MinimumJerk mj(x_start, x_end, t_start, t_end);

  FILE * fp = fopen("test_minimum_jerk_output.csv", "w");
  if (fp == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("minimum_jerk_test_node"), "Failed to open file.");
    return;
  }

  for (double t = t_start; t <= t_end; t += 0.01) {
    fprintf(fp, "%lf,%lf,%lf,%lf,%lf\n", t, mj.x(t), mj.v(t), mj.a(t), mj.j(t));
  }

  fclose(fp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  test_minimum_jerk();

  rclcpp::shutdown();
  return 0;
}