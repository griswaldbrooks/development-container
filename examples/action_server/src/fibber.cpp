#include "example_action/fibber.hpp"  // for Fibber

#include "example_interfaces/action/fibonacci.hpp"  // for Fibonacci::Goal
#include "rclcpp/rclcpp.hpp"                        // for rclcpp::ok, Rate
#include "rclcpp_action/rclcpp_action.hpp"  // for rclcpp_action::*Response

#include <memory>       // for unique_ptr
#include <string_view>  // for string_view
#include <thread>       // for thread
#include <utility>      // for move

Fibber::Fibber(std::unique_ptr<RclHandle> mw) : mw_{std::move(mw)} {
  mw_->register_handles(
      [this](rclcpp_action::GoalUUID const& uuid,
             std::shared_ptr<Fibonacci::Goal const> goal) {
        return handle_goal(uuid, std::move(goal));
      },

      [this](RclHandle* const rcl_handle) { return handle_cancel(rcl_handle); },
      [this](RclHandle* const rcl_handle) {
        std::thread{[this, rcl_handle] {
          handle_accepted(rcl_handle);
        }}.detach();
      });
}

rclcpp_action::GoalResponse Fibber::handle_goal(
    [[maybe_unused]] rclcpp_action::GoalUUID const&,
    std::shared_ptr<Fibonacci::Goal const> goal) {
  mw_->log("Received goal request with order " + std::to_string(goal->order));

  // Let's reject sequences that are over 9000
  // if (goal->order > 9000) {
  if (running_) {
    mw_->log("Reject goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  running_ = true;
  mw_->log("Accept goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Fibber::handle_cancel(
    RclHandle* const rcl_handle) {
  rcl_handle->log("Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Fibber::handle_accepted(RclHandle* const rcl_handle) {
  rcl_handle->log("Executing goal");
  const auto goal = rcl_handle->get_goal();
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  auto& sequence = feedback->sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  auto result = std::make_shared<Fibonacci::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (rcl_handle->is_canceling()) {
      result->sequence = sequence;
      rcl_handle->canceled(result);
      rcl_handle->log("Goal Canceled");
      return;
    }
    // Update sequence
    sequence.push_back(sequence[static_cast<size_t>(i)] +
                       sequence[static_cast<size_t>(i - 1)]);
    // Publish feedback
    rcl_handle->publish_feedback(feedback);
    rcl_handle->log("Publish Feedback");

    rcl_handle->sleep();
  }

  // Check if goal is done
  if (rcl_handle->ok()) {
    result->sequence = sequence;
    rcl_handle->succeed(result);
    rcl_handle->log("Goal Succeeded");
  }
  running_ = false;
}
