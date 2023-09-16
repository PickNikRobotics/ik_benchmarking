#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ik_benchmarking/action/ik_benchmark.hpp"
#include "ik_benchmarking/ik_benchmarking.hpp"

using IKBenchmark = ik_benchmarking::action::IKBenchmark;
using GoalHandleIKBenchmark = rclcpp_action::ServerGoalHandle<IKBenchmark>;

class IKBenchmarkingServer : public rclcpp::Node
{
public: 
  IKBenchmarkingServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<IKBenchmark>::SharedPtr action_server_; 

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const IKBenchmark::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle);

  void execute(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle, bool execute_once = false);
};
