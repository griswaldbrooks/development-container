#pragma once

#include "example_action/fibber.hpp"        // for Fibber::Fibonacci
#include "rclcpp/rclcpp.hpp"                // for Node
#include "rclcpp_action/rclcpp_action.hpp"  // for rclcpp_action::Server

#include <memory>       // for shared_ptr
#include <string_view>  // for string_view

struct RclHandle : public Fibber::RclHandle {
  RclHandle(std::shared_ptr<rclcpp::Node> node);

  void register_handles(GoalHandle goal, CancelHandle cancel,
                        AcceptedHandle accept) override;

  void log(std::string_view message) override;

  std::shared_ptr<Fibber::Fibonacci::Goal const> const get_goal()
      const override;

  bool is_canceling() const override;

  void canceled(std::shared_ptr<Fibber::Fibonacci::Result> result) override;

  void publish_feedback(
      std::shared_ptr<Fibber::Fibonacci::Feedback> feedback) override;

  void succeed(std::shared_ptr<Fibber::Fibonacci::Result> result) override;

  void sleep() override;

  bool ok() override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp_action::Server<Fibber::Fibonacci>> action_server_;
  std::shared_ptr<Fibber::GoalHandleFibonacci> goal_handle_;
  rclcpp::Rate rate_;
};
