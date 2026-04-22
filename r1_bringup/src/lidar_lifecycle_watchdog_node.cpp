#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LidarLifecycleWatchdogNode : public rclcpp::Node
{
public:
  LidarLifecycleWatchdogNode() : Node("lidar_lifecycle_watchdog_node")
  {
    node_names_ = this->declare_parameter<std::vector<std::string>>(
      "node_names", {"urg_node2_1", "urg_node2_2"});
    check_period_ = this->declare_parameter<double>("check_period", 1.0);
    service_timeout_ = this->declare_parameter<double>("service_timeout", 3.0);
    retry_interval_ = this->declare_parameter<double>("retry_interval", 2.0);
    startup_grace_period_ = this->declare_parameter<double>("startup_grace_period", 3.0);
    configure_unconfigured_ = this->declare_parameter<bool>("configure_unconfigured", true);
    activate_inactive_ = this->declare_parameter<bool>("activate_inactive", true);
    start_time_ = this->now();

    for (auto & name : node_names_) {
      name = normalize_node_name(name);
      auto context = std::make_shared<NodeContext>();
      context->get_state_client =
        this->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
      context->change_state_client =
        this->create_client<lifecycle_msgs::srv::ChangeState>(name + "/change_state");
      contexts_.emplace(name, context);
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(check_period_),
      std::bind(&LidarLifecycleWatchdogNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(), "Monitoring LiDAR lifecycle nodes: %s",
      join_node_names().c_str());
  }

private:
  struct NodeContext
  {
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture pending_get_state;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture pending_change_state;
    rclcpp::Time pending_deadline;
    std::string pending_transition_name;
    rclcpp::Time last_attempt_time;
    rclcpp::Time last_service_warn_time;
    bool has_pending_get_state = false;
    bool has_pending_change_state = false;
    bool has_last_attempt_time = false;
    bool has_last_service_warn_time = false;
  };

  static std::string normalize_node_name(const std::string & name)
  {
    if (!name.empty() && name.front() == '/') {
      return name;
    }
    return "/" + name;
  }

  std::string join_node_names() const
  {
    std::string joined;
    for (size_t i = 0; i < node_names_.size(); ++i) {
      if (i != 0) {
        joined += ", ";
      }
      joined += node_names_[i];
    }
    return joined;
  }

  void timer_callback()
  {
    for (const auto & name : node_names_) {
      auto context = contexts_.at(name);
      if (handle_pending_request(name, *context)) {
        continue;
      }

      if (!context->get_state_client->service_is_ready()) {
        if (is_startup_grace_period_finished()) {
          warn_service_unavailable(name, *context, "get_state");
        }
        continue;
      }

      auto future_and_request_id = context->get_state_client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      context->pending_get_state = future_and_request_id.future.share();
      context->pending_deadline = this->now() + rclcpp::Duration::from_seconds(service_timeout_);
      context->has_pending_get_state = true;
    }
  }

  bool handle_pending_request(const std::string & name, NodeContext & context)
  {
    if (context.has_pending_get_state) {
      if (context.pending_get_state.wait_for(0s) != std::future_status::ready) {
        return handle_timeout(name, context, "get_state");
      }
      auto response = context.pending_get_state.get();
      context.has_pending_get_state = false;
      handle_state(name, context, response->current_state);
      return true;
    }

    if (context.has_pending_change_state) {
      if (context.pending_change_state.wait_for(0s) != std::future_status::ready) {
        return handle_timeout(name, context, context.pending_transition_name);
      }
      auto response = context.pending_change_state.get();
      context.has_pending_change_state = false;
      if (response->success) {
        RCLCPP_INFO(
          this->get_logger(), "Requested %s for %s",
          context.pending_transition_name.c_str(), name.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(), "%s rejected %s transition", name.c_str(),
          context.pending_transition_name.c_str());
      }
      return true;
    }

    return false;
  }

  bool handle_timeout(const std::string & name, NodeContext & context, const std::string & request)
  {
    if (this->now() <= context.pending_deadline) {
      return true;
    }

    context.has_pending_get_state = false;
    context.has_pending_change_state = false;
    RCLCPP_WARN(this->get_logger(), "Timed out waiting for %s response from %s", request.c_str(), name.c_str());
    return false;
  }

  void handle_state(
    const std::string & name, NodeContext & context, const lifecycle_msgs::msg::State & state)
  {
    if (state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    if (is_transition_state(state.id)) {
      return;
    }

    if (!can_retry(context)) {
      return;
    }
    context.last_attempt_time = this->now();
    context.has_last_attempt_time = true;

    if (
      state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
      configure_unconfigured_)
    {
      request_transition(
        name, context, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure");
      return;
    }

    if (state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE && activate_inactive_) {
      request_transition(
        name, context, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");
      return;
    }

    RCLCPP_WARN(
      this->get_logger(), "%s is in lifecycle state %s [%u], no recovery transition configured",
      name.c_str(), state.label.c_str(), state.id);
  }

  bool can_retry(const NodeContext & context) const
  {
    if (!context.has_last_attempt_time) {
      return true;
    }
    return (this->now() - context.last_attempt_time).seconds() >= retry_interval_;
  }

  bool is_startup_grace_period_finished() const
  {
    return (this->now() - start_time_).seconds() >= startup_grace_period_;
  }

  static bool is_transition_state(uint8_t state_id)
  {
    return state_id >= lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
           state_id <= lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING;
  }

  void request_transition(
    const std::string & name, NodeContext & context, uint8_t transition_id,
    const std::string & transition_name)
  {
    if (!context.change_state_client->service_is_ready()) {
      warn_service_unavailable(name, context, "change_state");
      return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;
    auto future_and_request_id = context.change_state_client->async_send_request(request);
    context.pending_change_state = future_and_request_id.future.share();
    context.pending_deadline = this->now() + rclcpp::Duration::from_seconds(service_timeout_);
    context.pending_transition_name = transition_name;
    context.has_pending_change_state = true;
  }

  void warn_service_unavailable(
    const std::string & name, NodeContext & context, const std::string & service_name)
  {
    if (
      context.has_last_service_warn_time &&
      (this->now() - context.last_service_warn_time).seconds() < retry_interval_)
    {
      return;
    }

    context.last_service_warn_time = this->now();
    context.has_last_service_warn_time = true;
    RCLCPP_WARN(
      this->get_logger(), "%s/%s service is not available", name.c_str(), service_name.c_str());
  }

  std::vector<std::string> node_names_;
  double check_period_ = 1.0;
  double service_timeout_ = 3.0;
  double retry_interval_ = 2.0;
  double startup_grace_period_ = 3.0;
  bool configure_unconfigured_ = true;
  bool activate_inactive_ = true;
  rclcpp::Time start_time_;
  std::unordered_map<std::string, std::shared_ptr<NodeContext>> contexts_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLifecycleWatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
