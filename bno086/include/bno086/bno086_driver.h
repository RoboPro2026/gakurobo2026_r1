/**
 * @file bno086_driver.h
 * @author Yamaguchi Yudai
 * @brief BNO086をUSBシリアルを使って動かす
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cmath>
#include <complex>

#include "bno086/serial_driver.h"
#include "rclcpp/rclcpp.hpp"

class BNO086Driver
{
public:
  struct Data
  {
    int index;
    double yaw_angle;
    double pitch_angle;
    double roll_angle;
    double x_axis_accel;
    double y_axis_accel;
    double z_axis_accel;
    // パソコン上で微分して計算した値
    double yaw_angular_velocity;
    double pitch_angular_velocity;
    double roll_angular_velocity;
  };

private:
  std::shared_ptr<SerialDriver> serial_;
  std::string logger_name_;
  rclcpp::Time current_time_;
  rclcpp::Time prev_time_;
  BNO086Driver::Data current_data_;
  BNO086Driver::Data prev_data_;
  BNO086Driver::Data offset_data_;
  static constexpr int BUFF_SIZE = 256;

public:
  // degreeからradianへの変換係数
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  // mgからm/s/sへの変換係数
  static constexpr double MG_TO_MSS = 0.00980665;
  //バッファ
  std::vector<uint8_t> buff_;

  uint8_t calc_check_sum()
  {
    // データシートより
    // Checksum (Csum): The Index, yaw, pitch, roll, acceleration and reserved data bytes are added to produce the
    // checksum.
    int ret = 0;
    for (int i = 2; i < 18; i++) ret += buff_[i];
    return ret & 0xff;
  }

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

  void update_sensor_value()
  {
    // ROS 2の座標系では、
    // yaw(Z軸回り)はロボットを真上から見たときに半時計周りが正の方向（左旋回）
    // pitch(Y軸周り)はロボットを左から見たときに半時計回りが正の方向（機首上げ）
    // roll(X軸回り)はロボットを後ろから見たときに半時計回りが正の方向（左傾き）

    double yaw_angle, pitch_angle, roll_angle;

    // prevを更新
    prev_data_ = current_data_;
    prev_time_ = current_time_;

    current_time_ = rclcpp::Clock().now();
    current_data_.index = buff_[2];

    // 角度を計算
    yaw_angle = (double)((int16_t)(buff_[3] + (buff_[4] << 8))) / 100.0 * DEG_TO_RAD;
    // UART-RVCモードでは、yawだけ、ROS 2の座標系と回転方向が逆なので、反転
    yaw_angle *= -1;
    // オフセットを適用。オフセットは反転後の角度を基準としている
    yaw_angle += offset_data_.yaw_angle;
    // 値を正規化(-pi~pi)の範囲にし、current_data_に代入
    current_data_.yaw_angle = angle_normalize(yaw_angle);

    pitch_angle = (double)((int16_t)(buff_[5] + (buff_[6] << 8))) / 100.0 * DEG_TO_RAD;
    // オフセットを適用
    pitch_angle += offset_data_.pitch_angle;
    // 値を正規化(-pi~pi)の範囲にし、current_data_に代入
    current_data_.pitch_angle = angle_normalize(pitch_angle);

    roll_angle = (double)((int16_t)(buff_[7] + (buff_[8] << 8))) / 100.0 * DEG_TO_RAD;
    // オフセットを適用
    roll_angle += offset_data_.roll_angle;
    // 値を正規化(-pi~pi)の範囲にし、current_data_に代入
    current_data_.roll_angle = angle_normalize(roll_angle);

    // 加速度を計算
    current_data_.x_axis_accel =
      (double)((int16_t)(buff_[9] + (buff_[10] << 8))) * MG_TO_MSS + offset_data_.x_axis_accel;
    current_data_.y_axis_accel =
      (double)((int16_t)(buff_[11] + (buff_[12] << 8))) * MG_TO_MSS + offset_data_.y_axis_accel;
    current_data_.z_axis_accel =
      (double)((int16_t)(buff_[13] + (buff_[14] << 8))) * MG_TO_MSS + offset_data_.z_axis_accel;

    // 角速度を計算
    double dt = current_time_.seconds() - prev_time_.seconds();
    double d_yaw = angle_diff(current_data_.yaw_angle, prev_data_.yaw_angle);
    double d_pitch = angle_diff(current_data_.pitch_angle, prev_data_.pitch_angle);
    double d_roll = angle_diff(current_data_.roll_angle, prev_data_.roll_angle);
    current_data_.yaw_angular_velocity = d_yaw / dt + offset_data_.yaw_angular_velocity;
    current_data_.pitch_angular_velocity = d_pitch / dt + offset_data_.pitch_angular_velocity;
    current_data_.roll_angular_velocity = d_roll / dt + offset_data_.roll_angular_velocity;
  }

  bool decode(std::vector<uint8_t> rx_buff)
  {
    static int i = 0;
    int j = 0;
    bool is_update = false;
    // 受信データを受信状況に応じて処理
    while (j < (int)rx_buff.size()) {
      switch (i) {
        case 0:
          buff_[i] = rx_buff[j];
          if ((buff_[0] = rx_buff[j]) == 0xAA) {
            i++;
          }
          break;
        case 1:
          buff_[i] = rx_buff[j];
          if ((buff_[1] = rx_buff[j]) == 0xAA) {
            i++;
          } else {
            i = 0;
          }
          break;
        case 18:
          // チェックサムを計算
          buff_[i] = rx_buff[j];
          if (buff_[18] == calc_check_sum()) {
            // 値を更新
            update_sensor_value();
            is_update = true;
          } else {
          }
          i = 0;
          break;
        default:
          buff_[i] = rx_buff[j];
          i++;
          break;
      }
      j++;
    }
    return is_update;
  }

public:
  BNO086Driver(std::shared_ptr<SerialDriver> serial, std::string logger_name = "bno086_driver")
  : serial_(serial), logger_name_(logger_name)
  {
    current_time_ = prev_time_ = rclcpp::Clock().now();
    buff_.resize(BUFF_SIZE);
  }

  bool update()
  {
    std::vector<uint8_t> rx_buff = serial_->read();
    bool is_update = decode(rx_buff);
    return is_update;
  }

  void print(Data data)
  {
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "--- Data Packet [Index: %d] ---\n"
      "  Angles (rad):    [Yaw: %.3f, Pitch: %.3f, Roll: %.3f]\n"
      "  Accel (m/s^2):   [X: %.3f, Y: %.3f, Z: %.3f]\n"
      "  Ang Vel (rad/s): [Yaw: %.3f, Pitch: %.3f, Roll: %.3f]",
      data.index, data.yaw_angle, data.pitch_angle, data.roll_angle, data.x_axis_accel,
      data.y_axis_accel, data.z_axis_accel, data.yaw_angular_velocity, data.pitch_angular_velocity,
      data.roll_angular_velocity);
  }

  /**
 * @brief オフセットを設定する。Data.indexは関係ないので、無視される。
 * 
 * @param offset_data 
 */
  void set_offset_data(BNO086Driver::Data offset_data) { offset_data_ = offset_data; }

  BNO086Driver::Data get_offset_data(void) { return offset_data_; }

  BNO086Driver::Data get_data() { return current_data_; }
};