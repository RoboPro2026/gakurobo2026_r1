/**
 * @file state_machine.h
 * @author Yamaguchi Yudai(yudai.yy0804@gmail.com)
 * @brief 状態遷移に関する定義
 * @version 0.1
 * @date 2025-09-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <string>

#include "magic_enum.hpp"
#include "r1_util/r1_util.h"
#include "rclcpp/rclcpp.hpp"

// 現在の状態を管理する構造体
struct RobotState
{
  MainState main;
  OperationMode operation_mode;
  ChassisControlMode chassis_control_mode;

  bool operator==(const RobotState & other) const
  {
    return main == other.main && operation_mode == other.operation_mode &&
           chassis_control_mode == other.chassis_control_mode;
  }

  bool operator!=(const RobotState & other) const { return !(*this == other); }
};

/**
 * @brief 状態遷移用クラス
 * 
 */
class StateMachine
{
public:
  /**
   * @brief Construct a new State Machine object
   * 
   * @param logger print_state用。引数にstd::string,返り値voidのstd::function
   */
  StateMachine(std::string logger_name = "state_machine") : logger_name_(logger_name) {}
  void set_prev_state(RobotState state) { prev_state_ = state; }
  RobotState get_prev_state(void) { return prev_state_; }
  void set_current_state(RobotState state) { current_state_ = state; }
  RobotState get_current_state(void) { return current_state_; }
  void set_next_state(RobotState state) { next_state_ = state; }
  RobotState get_next_state(void) { return next_state_; }
  void print_state(RobotState state, std::string prefix_msg = "")
  {
    std::string main_state_str = std::string(magic_enum::enum_name(state.main));
    std::string operation_mode_str = std::string(magic_enum::enum_name(state.operation_mode));
    std::string chassis_mode_str = std::string(magic_enum::enum_name(state.chassis_control_mode));

    auto s = prefix_msg + "main_state = " + main_state_str +
             ", operation_mode = " + operation_mode_str +
             ", chassis_control_mode = " + chassis_mode_str;
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "%s", s.c_str());
  }
  bool is_changed_state(RobotState state1, RobotState state2) { return state1 != state2; }

  /**
   * @brief current_state != next_stateのとき、状態を更新する
   * 
   */
  void update(void)
  {
    if (current_state_ != next_state_) {
      prev_state_ = current_state_;
      current_state_ = next_state_;
      print_state(current_state_, "Current State: ");
    }
  }

  void restore(void)
  {
    current_state_ = next_state_ = prev_state_;
    print_state(current_state_, "Restored State: ");
  }

private:
  std::string logger_name_;
  RobotState next_state_{
    MainState::IDLE, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::HOLD};
  RobotState current_state_{
    MainState::IDLE, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::HOLD};
  RobotState prev_state_{
    MainState::IDLE, OperationMode::MODE1_DETECT_ORIGIN, ChassisControlMode::HOLD};
};
