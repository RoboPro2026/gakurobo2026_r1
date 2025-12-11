/**
 * @file trajectory_planner.cpp
 * @author Yudai Yamaguchi
 * @brief trajectory_plannerのPythonバインディング
 * @version 0.1
 * @date 2025-12-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "r1_control/trajectory_planner.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

namespace py = pybind11;

/**
   * @brief 軌道を計算する
   * 返り値は
   * [0]: 各区間の計算状態の配列、大きさはwaypoint数-1
   * [1]: 時刻tの配列
   * [2]: x座標の配列
   * [3]: y座標の配列
   * [4]: 角度thetaの配列
   * [5]: 走行距離distanceの配列
   * [6]: 並進速度v_transの配列
   * [7]: 並進加速度a_transの配列
   * [8]: 並進躍度j_transの配列
   * [9]: 角速度omegaの配列
   * [10]: 曲率curvatureの配列
   * 
   * @return std::tuple<
   * std::vector<int>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
   * std::vector<double>, std::vector<double>> 
   */
std::tuple<
  std::vector<int>, std::vector<double>, std::vector<double>, std::vector<double>,
  std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>,
  std::vector<double>, std::vector<double>, std::vector<double>>
trajectory_planner_calculate_trajectory(
  const std::vector<double> & x_wp, const std::vector<double> & y_wp,
  const std::vector<std::pair<int, double>> & theta_wp,
  const std::vector<std::pair<int, double>> & v_trans_wp, double dt, double v_max, double a_max,
  double j_max, double omega_max)
{
  TrajectoryPlanner planner;
  planner.calc(x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max);
  return planner.get_trajectory();
}

PYBIND11_MODULE(trajectory_planner, m)
{
  m.def("calculate_trajectory", &trajectory_planner_calculate_trajectory);
}
