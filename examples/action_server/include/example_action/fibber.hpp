#pragma once

#include "example_interfaces/action/fibonacci.hpp"  // for Fibonacci
#include "rclcpp_action/rclcpp_action.hpp"          // for ServerGoalHandle

#include <atomic>
#include <functional>   // for function
#include <memory>       // for unique_ptr, shared_ptr
#include <string_view>  // for string_view

struct Fibber {
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  struct RclHandle {
    using GoalHandle = std::function<rclcpp_action::GoalResponse(
        rclcpp_action::GoalUUID const&,
        std::shared_ptr<Fibonacci::Goal const>)>;
    using CancelHandle = std::function<rclcpp_action::CancelResponse(

        RclHandle* const rcl_handle)>;
    using AcceptedHandle = std::function<void(RclHandle* const rcl_handle)>;
    virtual void register_handles(GoalHandle, CancelHandle, AcceptedHandle) = 0;

    virtual void log(std::string_view) = 0;

    virtual std::shared_ptr<Fibonacci::Goal const> const get_goal() const = 0;
    virtual bool is_canceling() const = 0;
    virtual void canceled(std::shared_ptr<Fibonacci::Result> result) = 0;
    virtual void publish_feedback(
        std::shared_ptr<Fibonacci::Feedback> feedback) = 0;
    virtual void succeed(std::shared_ptr<Fibonacci::Result> result) = 0;
    virtual void sleep() = 0;
    virtual bool ok() = 0;

    virtual ~RclHandle() = default;
  };

  explicit Fibber(std::unique_ptr<RclHandle> mw);

 private:
  std::unique_ptr<RclHandle> mw_;
  std::atomic<bool> running_ = false;
  rclcpp_action::GoalResponse handle_goal(
      rclcpp_action::GoalUUID const&,
      std::shared_ptr<Fibonacci::Goal const> goal);

  rclcpp_action::CancelResponse handle_cancel(RclHandle* const rcl_handle);

  void handle_accepted(RclHandle* const rcl_handle);
};
