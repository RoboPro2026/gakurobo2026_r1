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
  int serial_port_ = -1;
  struct termios tty_;
  std::string port_name_;
  std::string logger_name_ = "serial_driver";
  bool is_connected_ = false;
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  bool open_port()
  {
    serial_port_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), port '%s' can't open", errno,
        strerror(errno), port_name_.c_str());
      return false;
    }

    if (tcgetattr(serial_port_, &tty_) != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), tcgetattr failed.", errno,
        strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return false;
    }

    tty_.c_cflag &= ~PARENB;
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;
    tty_.c_cflag &= ~CRTSCTS;
    tty_.c_cflag |= CREAD | CLOCAL;
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHONL;
    tty_.c_lflag &= ~ISIG;
    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;

    // VMIN = 0, VTIME = 5: 0.5秒タイムアウト付き非ブロッキング読み取り
    tty_.c_cc[VMIN] = 0;
    tty_.c_cc[VTIME] = 5;

    cfsetispeed(&tty_, B115200);
    cfsetospeed(&tty_, B115200);

    if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(logger_name_), "errno = %i(%s), tcsetattr failed.", errno,
        strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s connected.", port_name_.c_str());
    is_connected_ = true;
    return true;
  }

public:
  SerialDriver(std::string port_name) : port_name_(port_name) { open_port(); }

  ~SerialDriver()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
      RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s closed.", port_name_.c_str());
    }
  }

  bool is_connected() { return is_connected_; }

  bool reconnect()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
      serial_port_ = -1;
    }
    is_connected_ = false;
    return open_port();
  }

  void write_buff(std::vector<uint8_t> tx_buff)
  {
    if (!is_connected_) return;
    int n;
    if ((n = ::write(serial_port_, tx_buff.data(), tx_buff.size())) < 0) {
      if (errno == EIO || errno == ENXIO || errno == EBADF) {
        is_connected_ = false;
        RCLCPP_ERROR(
          rclcpp::get_logger(logger_name_),
          "Serial port '%s' disconnected during write. errno = %i(%s)", port_name_.c_str(), errno,
          strerror(errno));
      } else {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(logger_name_), clock_, 1000, "write failed. errno = %i(%s)", errno,
          strerror(errno));
      }
    }
  }

  std::vector<uint8_t> read()
  {
    std::vector<uint8_t> ret;
    if (!is_connected_) return ret;

    char rx_buff[256];
    int n;
    if ((n = ::read(serial_port_, &rx_buff, 256)) < 0) {
      if (errno == EIO || errno == ENXIO || errno == EBADF) {
        // 断線を検出。以降のread()はis_connected_=falseで即時リターンする
        is_connected_ = false;
        RCLCPP_ERROR(
          rclcpp::get_logger(logger_name_), "Serial port '%s' disconnected. errno = %i(%s)",
          port_name_.c_str(), errno, strerror(errno));
      } else {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(logger_name_), clock_, 1000, "read failed. errno = %i(%s)", errno,
          strerror(errno));
      }
    } else if (n > 0) {
      ret.resize(n);
      for (int i = 0; i < n; i++) ret[i] = rx_buff[i];
    }
    // n == 0 はタイムアウト。このデバイスは常時送信しないため断線とは判断しない
    return ret;
  }
};
