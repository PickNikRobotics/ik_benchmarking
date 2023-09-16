#ifndef IK_BENCHMARKING_HPP
#define IK_BENCHMARKING_HPP

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <limits.h>
#include <random>
#include <fstream>

struct JointBounds
{
  double min_position;
  double max_position;

  JointBounds()
      : min_position(std::numeric_limits<double>::min()),
        max_position(std::numeric_limits<double>::max()) {}
};

class IKBenchmarking
{
public:
  explicit IKBenchmarking(rclcpp::Node::SharedPtr node)
      : node_(node),
        logger_(node->get_logger()),
        robot_model_loader_(node),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false)
  {
      data_file_.open("ik_benchmarking_data.csv", std::ios::app);
      data_file_ << "trial,found_ik,solve_time,position_error,orientation_error,joints_error\n";
  }

  IKBenchmarking(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(rclcpp::Node::make_shared(node_name, options)),
        logger_(node_->get_logger()),
        robot_model_loader_(node_),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false)
  {   
      data_file_.open("ik_benchmarking_data"".csv", std::ios::app);
      data_file_ << "trial,found_ik,solve_time,position_error,orientation_error,joints_error\n";
  }
  
  // Constructor used by the IKBenchmarkingServer to customize the output name
  IKBenchmarking(const std::string& node_name, const std::string& solver, const std::string output_file, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(rclcpp::Node::make_shared(node_name, options)),
        logger_(node_->get_logger()),
        robot_model_loader_(node_),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false)
  {   
      //Todo: Mohamed, customize data file name based on solver and output file path
      data_file_.open(std::string(solver)+"_ik_benchmarking_data.csv", std::ios::app);
      data_file_ << "trial,found_ik,solve_time,position_error,orientation_error,joints_error\n";
  }
  
  void run();
  double get_success_rate() const;
  double get_average_solve_time() const;
  bool calculation_done() const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  std::string move_group_name_;
  const moveit::core::JointModelGroup *joint_model_group_;

  std::string tip_link_name_;
  std::vector<JointBounds> joint_bounds_;
  std::vector<std::string> joint_names_;

  std::random_device rd_;
  std::mt19937 generator_;

  unsigned int sample_size_;
  double success_count_;
  std::vector<int> solve_times_; // microseconds
  double average_solve_time_; 
  double success_rate_; 
  bool calculation_done_;

  std::ofstream data_file_;

  void initialize();
  void gather_date();
};

#endif // IK_BENCHMARKING_HPP