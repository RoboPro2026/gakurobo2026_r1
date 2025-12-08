/**
 * @file trapezoid_test_node.cpp
 * @author Yudai Yamaguchi
 * @brief 台形制御のテストノード
 * @version 0.1
 * @date 2025-12-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdio>

#include "r1_control/accel_designer.h"
#include "rclcpp/rclcpp.hpp"

void test_trapezoid()
{
  // パラメータを設定
  const float j_max = 20;
  const float a_max = 5;
  const float v_max = 5;
  const float v_start = 0;
  const float v_target = 0;
  const float distance = 5;
  // 曲線を生成
  AccelDesigner ad(j_max, a_max, v_max, v_start, v_target, distance);

  FILE * fp = fopen("trapezoid_test_output.csv", "w");
  if (fp == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("trapezoid_test_node"), "Failed to open file.");
    return;
  }

  // 計算結果を配列に格納
  for (double t = 0; t < ad.t_end(); t += 0.001f) {
    fprintf(fp, "%lf,%lf,%lf,%lf,%lf\n", t, ad.x(t), ad.v(t), ad.a(t), ad.j(t));
  }
  fclose(fp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  test_trapezoid();

  rclcpp::shutdown();
  return 0;
}
