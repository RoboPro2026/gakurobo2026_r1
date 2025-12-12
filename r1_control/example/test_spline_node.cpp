/**
 * @file test_spline_node.cpp
 * @author Yudai Yamaguchi
 * @brief C2スプライン補完を計算するサンプルプログラム
 * @version 0.1
 * @date 2025-12-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdio>

#include "r1_control/spline.h"

void test_spline()
{
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y = {0.0, 1.0, 3.0, 2.5, 4.5};

  FILE * fp1 = fopen("test_spline_point.csv", "w");
  FILE * fp2 = fopen("test_spline.csv", "w");

  if (fp1 == NULL || fp2 == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("spline_test_node"), "Failed to open file.");
    return;
  }

  for (size_t i = 0; i < x.size(); i++) {
    fprintf(fp1, "%lf,%lf\n", x[i], y[i]);
  }

  Spline spline;
  spline.calc(x, y);

  for (double i = 0.0; i < 4.0; i += 0.01) {
    fprintf(fp2, "%lf,%lf,%lf\n", i, spline.get_y(i), spline.get_curvature(i));
  }

  fclose(fp1);
  fclose(fp2);
}

void test_spline2d()
{
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 2.0, 1.0};
  std::vector<double> y = {0.0, 1.0, 3.0, 2.5, 4.5, 3.0};

  FILE * fp1 = fopen("test_spline_point.csv", "w");
  FILE * fp2 = fopen("test_spline.csv", "w");

  if (fp1 == NULL || fp2 == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("spline_test_node"), "Failed to open file.");
    return;
  }

  for (size_t i = 0; i < x.size(); i++) {
    fprintf(fp1, "%lf,%lf\n", x[i], y[i]);
  }

  Spline2D spline2d;
  spline2d.calc(x, y);

  double t = 0.0;
  for (int i = 0; i <= 100; i++) {
    auto [sx, sy] = spline2d.get_pos(t);
    t += 0.01;
    fprintf(fp2, "%lf,%lf,%lf\n", sx, sy, spline2d.get_curvature(t));
  }

  fclose(fp1);
  fclose(fp2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // test_spline();
  test_spline2d();

  rclcpp::shutdown();
  return 0;
}
