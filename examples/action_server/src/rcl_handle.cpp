#include "example_action/rcl_handle.hpp"  // for RclHandle

#include "example_action/fibber.hpp"        // for Fibber::Fibonacci
#include "rclcpp/rclcpp.hpp"                // for Node, RCLCPP_INFO
#include "rclcpp_action/rclcpp_action.hpp"  // for create_server

#include <memory>       // for make_shared, shared_ptr
#include <string_view>  // for string_view

RclHandle::RclHandle(std::shared_ptr<rclcpp::Node> node)
    : node_{std::move(node)}, rate_{1} {}

void RclHandle::register_handles(GoalHandle goal, CancelHandle cancel,
                                 AcceptedHandle accept) {
  action_server_ = rclcpp_action::create_server<Fibber::Fibonacci>(
      node_, node_->get_name(), goal,
      [this,
       cancel](std::shared_ptr<Fibber::GoalHandleFibonacci> const goal_handle) {
        goal_handle_ = goal_handle;
        return cancel(this);
      },
      [this,
       accept](std::shared_ptr<Fibber::GoalHandleFibonacci> const goal_handle) {
        goal_handle_ = goal_handle;
        accept(this);
      });
}

void RclHandle::log(std::string_view message) {
  RCLCPP_INFO(node_->get_logger(), "%s", std::string{message}.c_str());
}

std::shared_ptr<Fibber::Fibonacci::Goal const> const RclHandle::get_goal()
    const {
  return goal_handle_->get_goal();
}

bool RclHandle::is_canceling() const { return goal_handle_->is_canceling(); }

void RclHandle::canceled(std::shared_ptr<Fibber::Fibonacci::Result> result) {
  goal_handle_->canceled(result);
}

void RclHandle::publish_feedback(
    std::shared_ptr<Fibber::Fibonacci::Feedback> feedback) {
  goal_handle_->publish_feedback(feedback);
}

void RclHandle::succeed(std::shared_ptr<Fibber::Fibonacci::Result> result) {
  return goal_handle_->succeed(result);
}

void RclHandle::sleep() { rate_.sleep(); }

bool RclHandle::ok() { return rclcpp::ok(); }
