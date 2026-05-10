/**
 * @file r1_util.h
 * @author Yudai Yamaguchi (yudai.yy0804@gmail.com)
 * @brief R1の共通ライブラリ
 * @version 0.1
 * @date 2026-02-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <complex>
#include <vector>

constexpr int FOREST_N = 12;
constexpr int ACT_N = 6;

enum class ChassisAct
{
  NONE = 0,
  ACT0_START = 1,
  ACT0 = 2,
  ACT0_FINISH = 3,
  ACT1_START = 11,
  ACT1 = 12,
  ACT1_FINISH = 13,
  ACT2_START = 21,
  ACT2 = 22,
  ACT2_FINISH = 23,
  ACT3_START = 31,
  ACT3 = 32,
  ACT3_FINISH = 33,
  ACT4_START = 41,
  ACT4 = 42,
  ACT4_FINISH = 43,
  ACT5_START = 51,
  ACT5 = 52,
  ACT5_FINISH = 53,
  ACT_PAUSE = 1000,
  ACT_RESUME = 1001,
};

// MainStateの定義
enum class MainState
{
  IDLE,
  READY,
  EMERGENCY
};

// いまユーザが見ている/操作モード
enum class OperationMode
{
  MODE1_DETECT_ORIGIN,
  MODE2_POLE,
  MODE3_SPEAR,
  MODE4_FKFS,
  MODE5_RKFS,
  MODE6_R2_LIFT,
  MODE7_SPEAR_ATTACK,
  MODE8_AUTO_COLLECT_KFS,
  MODE9_AUTO_CHASSIS
};

// 足回りの制御権
enum class ChassisControlMode
{
  HOLD,
  MANUAL,
  AUTO,
};

/**
 * @brief 1次のローパスフィルタ
 * 
 * @param x 現在の入力
 * @param prev_y 限界の出力
 * @param tau 時定数[s]
 * @param dt 制御周期[s]
 * @return double 出力 y = alpha * x + (1 - alpha) * prev_y
 * ただし alpha = dt / (tau + dt)
 */
double lpf(double x, double prev_y, double tau, double dt)
{
  double alpha = dt / (tau + dt);
  return alpha * x + (1 - alpha) * prev_y;
}

/**
 * @brief 指定した開始点から終了点までの範囲を、指定した数だけ等間隔に分割して配列を生成する
 * 
 * @param start 開始点
 * @param end 終了点
 * @param num 分割数
 * @return std::vector<double> 
 */
std::vector<double> linspace(double start, double end, int num)
{
  std::vector<double> x(num);
  double step = (end - start) / (num - 1);
  for (int i = 0; i < num; i++) {
    x[i] = start + step * i;
  }
  return x;
}

/**
   * @brief xの符号を返す
   * 
   * @param x 
   * @return double xが0.0以上なら1.0、負なら-1.0を返す
   */
double sign(double x) { return (x >= 0) ? 1.0 : -1.0; }

/**
   * @brief 角度を-pi~piの範囲に正規化する
   * 
   * @param angle 
   * @return double 
   */
double angle_normalize(double angle)
{
  std::complex<double> ret = std::polar(1.0, angle);
  return std::arg(ret);
}

/**
   * @brief 角度差を計算する。計算結果は-pi~pi
   * 
   * @param current_angle 
   * @param prev_angle 
   * @return double 
   */
double angle_diff(double current_angle, double prev_angle)
{
  std::complex<double> current = std::polar(1.0, current_angle);
  std::complex<double> prev = std::polar(1.0, prev_angle);
  std::complex<double> diff = current / prev;  // 位相差
  return std::arg(diff);
}

/**
 * @brief 現在の位置が、指定した開始点と終了点の範囲内にあるかどうかを判定する
 * 
 * @param current_x 
 * @param current_y 
 * @param x1 
 * @param y1 
 * @param x2 
 * @param y2 
 * @return true 
 * @return false 
 */
bool is_within_range(double current_x, double current_y, double x1, double y1, double x2, double y2)
{
  // x2とy2がx1とy1より大きくなるようにする。
  if (x1 > x2) {
    std::swap(x1, x2);
  }
  if (y1 > y2) {
    std::swap(y1, y2);
  }
  bool is_x_in_range = (x1 <= current_x && current_x <= x2);
  bool is_y_in_range = (y1 <= current_y && current_y <= y2);
  return is_x_in_range && is_y_in_range;
}

/**
 * @brief 現在位置が、指定した中心座標・姿勢・幅・高さを持つ長方形の内部にあるかを判定する
 *
 * yaw=0 のとき、width は map の x 方向、height は map の y 方向の長さとして扱う。
 * yaw != 0 のときは、yaw だけ回転した長方形ローカル座標系での幅と高さとして扱う。
 *
 * @param current_x 判定したい点の x 座標
 * @param current_y 判定したい点の y 座標
 * @param center_x 長方形中心の x 座標
 * @param center_y 長方形中心の y 座標
 * @param yaw 長方形の向き [rad]
 * @param width 長方形ローカル x 方向の幅 [m]
 * @param height 長方形ローカル y 方向の高さ [m]
 * @return true 点が長方形の内側にある
 * @return false 点が長方形の外側にある
 */
bool is_within_rotated_rectangle(
  double current_x, double current_y, double center_x, double center_y, double yaw, double width,
  double height)
{
  double dx = current_x - center_x;
  double dy = current_y - center_y;
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  // 長方形座標系へ変換するため、ワールド座標の点を -yaw だけ回転する。
  double local_x = cos_yaw * dx + sin_yaw * dy;
  double local_y = -sin_yaw * dx + cos_yaw * dy;

  return std::abs(local_x) <= width * 0.5 && std::abs(local_y) <= height * 0.5;
}

/**
 * @brief 始点と終点を通る直線に対して、現在位置がその直線を通過したかどうかを判定する
 * 
 * @param start_x 
 * @param start_y 
 * @param goal_x 
 * @param goal_y 
 * @param current_x 
 * @param current_y 
 * @return true goalを通過している
 * @return false goalを通過していない
 */
bool is_passed_goal_by_dot(
  double start_x, double start_y, double goal_x, double goal_y, double current_x, double current_y)
{
  double gsx = goal_x - start_x;
  double gsy = goal_y - start_y;
  double csx = current_x - start_x;
  double csy = current_y - start_y;
  double dist = std::hypot(gsx, gsy);
  if (dist < 1e-6) {
    // ゴールと開始点がほぼ同じ場合は、常にゴールを通過しているとみなす
    return true;
  }
  double dot = gsx * csx + gsy * csy;
  // 内積が正のときはゴールを通過している
  return dot > 0;
}