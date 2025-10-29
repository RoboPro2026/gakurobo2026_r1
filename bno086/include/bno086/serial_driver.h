/**
 * @file serial_driver.h
 * @author Yamaguchi Yudai
 * @brief Linuxでシリアル通信するプログラム
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <errno.h>    // エラー番号
#include <fcntl.h>    // ファイル制御 (open)
#include <stdio.h>    // 標準入出力
#include <string.h>   // 文字列操作
#include <termios.h>  // POSIXターミナル制御
#include <unistd.h>   // UNIX標準関数 (read, write, close)

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class SerialDriver
{
private:
  int serial_port_;
  struct termios tty_;
  std::string port_name_;
  std::string logger_name_ = "serial_driver";
  bool is_initialize_success;

public:
  SerialDriver(std::string port_name) : port_name_(port_name)
  {
    // 接続開始処理
    // --- シリアルポートを開く ---
    // O_RDWR: 読み書き両用で開く
    // O_NOCTTY: このポートを制御端末にしない
    // O_NDELAY: (今回は使わないが) 読み書き時にブロックしない設定
    serial_port_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);

    // open()が-1を返した場合、エラー
    if (serial_port_ < 0) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), port '%s' can't open", errno,
        strerror(errno), port_name_.c_str());
      is_initialize_success = false;
      return;
    }

    // --- 現在のシリアルポート設定を取得 ---
    if (tcgetattr(serial_port_, &tty_) != 0) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), tcgetattr failed.", errno,
        strerror(errno));
      is_initialize_success = false;
      return;
    }

    // --- 設定の変更 (8N1) ---

    // パリティなし (PARENBをクリア)
    tty_.c_cflag &= ~PARENB;
    // ストップビット 1 (CSTOPBをクリア)
    tty_.c_cflag &= ~CSTOPB;
    // データビット 8 (CSIZEをクリア -> CS8を設定)
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;
    // ハードウェアフロー制御なし
    tty_.c_cflag &= ~CRTSCTS;
    // ローカルモード、受信有効
    tty_.c_cflag |= CREAD | CLOCAL;

    // ローカルフラグ (lflag)
    // カノニカルモード(行単位)ではなく、非カノニカル(RAW)モードに設定
    tty_.c_lflag &= ~ICANON;
    // エコー無効
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHONL;
    // シグナル文字無効
    tty_.c_lflag &= ~ISIG;

    // 入力フラグ (iflag)
    // XON/XOFF フロー制御無効
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    // CRをNLに変換しない、パリティエラーを無視しない、BREAKをシグナルにしない
    tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // 出力フラグ (oflag)
    // RAW出力 (出力処理を無効化)
    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;  // (例: \n を \r\n に変換しない)

    // --- 読み取りタイムアウト設定 ---
    // VMIN = 0, VTIME = 0: 完全な非ブロッキング
    // VMIN > 0, VTIME = 0: VMIN バイト受信するまでブロック
    // VMIN = 0, VTIME > 0: VTIME デシ秒(0.1秒)待つか、1バイトでも受信したら戻る
    // VMIN > 0, VTIME > 0: VMIN バイト受信するか、(VTIMEデシ秒 * VMIN) 時間待つ
    tty_.c_cc[VMIN] = 0;   // 0バイトでも読み取りを返す (非ブロッキングに近い)
    tty_.c_cc[VTIME] = 5;  // 0.5秒のタイムアウト (単位は 0.1秒)

    // --- ボーレート設定 (115200) ---
    cfsetispeed(&tty_, B115200);
    cfsetospeed(&tty_, B115200);

    // --- 設定をポートに適用 ---
    if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), tcsetattr failed.", errno,
        strerror(errno));
      is_initialize_success = false;
      return;
    }
    // 接続完了
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s connect.", port_name_.c_str());
    is_initialize_success = true;
  }

  ~SerialDriver()
  {
    // 接続終了処理
    close(serial_port_);
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s closed.", port_name_.c_str());
  }

  bool get_is_initialize_success() { return is_initialize_success; }

  void write_buff(std::vector<uint8_t> tx_buff)
  {
    // vectorは値が連続していることが保証されているので、.data()でポインタを取得して、送信
    int n;
    if ((n = ::write(serial_port_, tx_buff.data(), tx_buff.size())) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "write failed.");
    } else {
      // ログの出力処理
      std::string msg;
      // RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%d byte write success.", n);
      for (int i = 0; i < n; i++) {
        char buff[256];
        sprintf(buff, "[%d] = %d, ", i, tx_buff[i]);
        msg += buff;
      }
      // RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s", msg.c_str());
    }
  }

  std::vector<uint8_t> read()
  {
    std::vector<uint8_t> ret;
    char rx_buff[256];
    int n;
    if ((n = ::read(serial_port_, &rx_buff, 256)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "read failed.");
    } else if (n == 0) {
      RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Receive Timeout.");
    } else {
      // 値をコピー
      ret.resize(n);
      for (int i = 0; i < n; i++) ret[i] = rx_buff[i];
      // ログの出力
      std::string msg;
      // RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%d byte read success.", n);
      for (int i = 0; i < n; i++) {
        char buff[256];
        sprintf(buff, "[%d] = %d, ", i, rx_buff[i]);
        msg += buff;
      }
      // RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s", msg.c_str());
    }
    return ret;
  }
};