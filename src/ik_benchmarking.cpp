#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <random>
#include <chrono>
#include <numeric>

#include "ik_benchmarking/ik_benchmarking.hpp"

using namespace std::chrono_literals;

void IKBenchmarking::initialize()
{
  generator_ = std::mt19937(rd_());

  robot_state_->setToDefaultValues();

  move_group_name_ = node_->get_parameter("move_group").as_string();
  joint_model_group_ = robot_model_->getJointModelGroup(move_group_name_);

  joint_names_ = joint_model_group_->getVariableNames();
  joint_bounds_.resize(joint_model_group_->getVariableCount());

  // TODO(Mohamed): use only active joints/variables because getVariableCount returns all joint types
  for (size_t i = 0; i < joint_model_group_->getVariableCount(); ++i)
  {
    auto const &name = joint_names_.at(i);
    auto const &bounds = robot_model_->getVariableBounds(name);

    bool bounded = bounds.position_bounded_;

    if (bounded)
    {
      fmt::print("Joint {} has bounds of {} and {}\n", i + 1,
                 bounds.min_position_, bounds.max_position_);
      joint_bounds_.at(i).min_position = bounds.min_position_;
      joint_bounds_.at(i).max_position = bounds.max_position_;
    }
    else
    {
      fmt::print("Joints are unbounded!\n");
      // TODO (Mohamed): Handle this case. Should we assume a range?
    }
  }

  // Load the tip link name (not the end effector)
  auto const &link_names = joint_model_group_->getLinkModelNames();

  if (!link_names.empty())
  {
    tip_link_name_ = link_names.back();
  }
  else
  {
    RCLCPP_ERROR(logger_, "ERROR: The move group is corrupted. Links count is zero.\n");
    rclcpp::shutdown();
  }
}

void IKBenchmarking::gather_date()
{
  const Eigen::Isometry3d &tip_link_pose = robot_state_->getGlobalLinkTransform(tip_link_name_);
  // Collect IK solving data
  sample_size_ = node_->get_parameter("sample_size").as_int();

  for (size_t i = 0; i < sample_size_; ++i)
  {
    std::vector<double> joint_values;

    for (const auto &bound : joint_bounds_)
    {
      std::uniform_real_distribution<> distribution(bound.min_position,
                                                    bound.max_position);
      joint_values.push_back(distribution(generator_));
    }
    // fmt::print("Random joint values are:\n{}\n", joint_values);

    // Solve Forward Kinematics
    robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
    robot_state_->updateLinkTransforms();

    // Solve Inverse kinematics
    const auto start_time = std::chrono::high_resolution_clock::now();

    bool found_ik =
        robot_state_->setFromIK(joint_model_group_, tip_link_pose, 0.1);

    const auto end_time = std::chrono::high_resolution_clock::now();

    if (found_ik)
    {
      success_count_++;
      const auto solve_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      solve_times_.push_back(solve_time.count());

      data_file_ << i + 1 << ",yes," << solve_time.count() << "\n";
    }
    else
    {
      data_file_ << i + 1 << ",no,not_available" << "\n";
    }
  }

  // Average IK solving time and success rate
  average_solve_time_ = std::accumulate(solve_times_.begin(), solve_times_.end(), 0.0) / solve_times_.size();
  success_rate_ = success_count_ / sample_size_;

  fmt::print("Success rate = {} and average IK solving time is {} ms\n", success_rate_ , average_solve_time_);

  calculation_done_ = true;
}

void IKBenchmarking::run()
{
  this->initialize();
  this->gather_date();
  
  this->data_file_.close();
}

double IKBenchmarking::get_success_rate() const
{
  return success_rate_;
}

double IKBenchmarking::get_average_solve_time() const
{
  return average_solve_time_;
}

bool IKBenchmarking::calculation_done() const
{
  return calculation_done_;
}