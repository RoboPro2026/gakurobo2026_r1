/**
 * @file test_trajectory_planner_node.cpp
 * @author Yudai Yamaguchi
 * @brief 軌道生成のテスト
 * @version 0.1
 * @date 2025-12-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdio>

#include "r1_control/trajectory_planner.h"

void test_trajectory_planner()
{
  // ウェイポイントを設定
  std::vector<double> x_wp = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y_wp = {0.0, 1.0, 0.5, 2.5, 2.0};
  std::vector<double> theta_wp = {0.0, M_PI / 6, M_PI / 4, M_PI / 3, M_PI / 2};
  std::vector<double> v_trans_wp = {0.0, 0.5, 1.0, 0.5, 0.0};
  double dt = 0.01;    // 制御周期
  double v_max = 1.0;  // 最大速度
  double a_max = 0.5;  // 最大加速度
  double j_max = 1.0;  // 最大躍度
  TrajectoryPlanner planner;
  planner.calc(x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max);
  FILE * fp_traj = fopen("test_trajectory_output.csv", "w");
  FILE * fp_wp = fopen("test_waypoint_output.csv", "w");
  if (fp_traj == NULL || fp_wp == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("trajectory_planner_test_node"), "Failed to open file.");
    return;
  }
  planner.print_csv_trajectory(fp_traj);
  planner.print_csv_waypoint(fp_wp);
  fclose(fp_traj);
  fclose(fp_wp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  test_trajectory_planner();

  rclcpp::shutdown();
  return 0;
}