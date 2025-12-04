/**
 * @file spline_test_node.cpp
 * @author Yudai Yamaguchi
 * @brief C2スプライン補完を計算するサンプルプログラム
 * @version 0.1
 * @date 2025-12-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "matplotlibcpp.h"
#include "r1_control/spline.h"

namespace plt = matplotlibcpp;

void test_spline()
{
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y = {0.0, 1.0, 3.0, 2.5, 4.5};

  Spline spline;
  spline.calc(x, y);

  std::vector<double> x_plot;
  std::vector<double> y_plot;
  std::vector<double> curvature_plot;
  for (double i = 0.0; i < 4.0; i += 0.01) {
    x_plot.push_back(i);
    y_plot.push_back(spline.get_y(i));
    curvature_plot.push_back(spline.get_curvature(i));
  }
  plt::figure();
  plt::title("Spline Interpolation");
  plt::scatter(x, y, 50.0);
  plt::plot(x_plot, y_plot);
  plt::figure();
  plt::title("Curvature");
  plt::plot(x_plot, curvature_plot);
  plt::show();
}

void test_spline2d()
{
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 2.0, 1.0};
  std::vector<double> y = {0.0, 1.0, 3.0, 2.5, 4.5, 3.0};

  Spline2D spline2d;
  spline2d.calc(x, y);

  std::vector<double> x_plot;
  std::vector<double> y_plot;
  std::vector<double> curvature_plot;
  double t = 0.0;
  for (int i = 0; i <= 100; i++) {
    auto [sx, sy] = spline2d.get_pos(t);
    x_plot.push_back(sx);
    y_plot.push_back(sy);
    curvature_plot.push_back(spline2d.get_curvature(t));
    t += 0.01;
  }
  plt::figure();
  plt::title("2D Spline Interpolation");
  plt::scatter(x, y, 50.0);
  plt::plot(x_plot, y_plot);
  plt::figure();
  plt::title("2D Curvature");
  plt::plot(linspace(0.0, 1.0, curvature_plot.size()), curvature_plot);
  plt::show();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // test_spline();
  test_spline2d();

  rclcpp::shutdown();
  return 0;
}
