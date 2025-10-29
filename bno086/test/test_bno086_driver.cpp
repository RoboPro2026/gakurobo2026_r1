/**
 * @file test_bno086_driver.cpp
 * @author Yamaguchi Yudai
 * @brief bno086_driverのテスト。
 * データシートに乗っているexampleデータを使って、正しくデコードできているかの確認を行う
 * @version 0.1
 * @date 2025-10-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "bno086/bno086_driver.h"

TEST(TestBNO086Driver, decodeExampleData)
{
  // 宣言のみのダミー
  std::shared_ptr<SerialDriver> serial = nullptr;
  BNO086Driver bno086_driver(serial);

  // BNO086のデータシート、21ページに載っているexampleデータ
  std::vector<uint8_t> example_data = {0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D,
                                       0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7};

  bno086_driver.decode(example_data);
  BNO086Driver::Data data = bno086_driver.get_data();
  bno086_driver.print(data);

  // 角度、加速度のテスト
  // データシートに載っている期待値
  EXPECT_EQ(data.index, 0xDE);
  EXPECT_NEAR(data.yaw_angle, 0.01 * BNO086Driver::DEG_TO_RAD, 1e-3);
  EXPECT_NEAR(data.pitch_angle, -1.10 * BNO086Driver::DEG_TO_RAD, 1e-3);
  EXPECT_NEAR(data.roll_angle, 20.85 * BNO086Driver::DEG_TO_RAD, 1e-3);
  EXPECT_NEAR(data.x_axis_accel, -3.638, 1e-3);
  EXPECT_NEAR(data.y_axis_accel, -0.196, 1e-3);
  EXPECT_NEAR(data.z_axis_accel, 9.581, 1e-3);
}