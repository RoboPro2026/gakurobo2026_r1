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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>

#include "magic_enum.hpp"
#include "rclcpp/rclcpp.hpp"

// MainStateの定義
enum class MainState
{
  IDLE,
  EMERGENCY,
  MANUAL,
  AUTO
};

// 各SubStateの定義
enum class IdleSubState
{
  NONE
};

enum class EmergencySubState
{
  NONE
};

enum class ManualSubState
{
  MODE1_DETECT_ORIGIN,
  MODE2_POLE,
  MODE3_SPEAR,
  MODE4_KFS,
  MODE5_R2_LIFT,
  TEST
};
enum class AutoSubState
{
  NONE
};

// SubStateを保持するvariant
using SubState = std::variant<IdleSubState, EmergencySubState, ManualSubState, AutoSubState>;
using namespace std::chrono_literals;

// 現在の状態を管理する構造体
struct RobotState
{
  MainState main;
  SubState sub;

  bool operator==(const RobotState & other) const { return main == other.main && sub == other.sub; }

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
    std::string sub_state_str =
      std::visit([](auto s) { return std::string(magic_enum::enum_name(s)); }, state.sub);

    auto s = prefix_msg + "main_state = " + main_state_str + ", sub_state = " + sub_state_str;
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
  RobotState next_state_{MainState::IDLE, IdleSubState::NONE};
  RobotState current_state_{MainState::IDLE, IdleSubState::NONE};
  RobotState prev_state_{MainState::IDLE, IdleSubState::NONE};
};
