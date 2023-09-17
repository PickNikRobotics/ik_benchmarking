#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <limits.h>
#include <random>
#include <fstream>

/**
 * @brief Represents the minimum and maximum positional limits of a joint.
 * 
 * This struct is used to define the bounds for the position of a robotic joint.
 * It stores the minimum and maximum allowed positions for the joint, and is initialized
 * to the broadest possible range by default.
 */
struct JointBounds
{
  double min_position; ///< The minimum allowed position for the joint.
  double max_position; ///< The maximum allowed position for the joint.

  /**
   * @brief Default constructor for JointBounds.
   * 
   * Initialize min_position to the smalled representable double value, 
   * and max_position to the largest representable double value.
   */
  JointBounds()
      : min_position(std::numeric_limits<double>::min()),
        max_position(std::numeric_limits<double>::max()) {}
};

/**
 * @brief Class for Inverse Kinematics Benchmarking
 * 
 * This class is responsible for benchmarking the performance
 * of Inverse Kinematics (IK) solvers. It runs a set of trials 
 * to evaluate success rate, solve time, and error metrics.
 */
class IKBenchmarking
{
public:
  /**
   * @brief Constructor using a shared pointer to an rclcpp::Node.
   * 
   * @param node Shared pointer to the node.
   */
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

  /**
   * @brief Overloaded constructor that creates a node.
   * 
   * @param node_name The name of the node to create.
   * @param options Additional node options.
   */
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
  
  /**
   * @brief Overloaded constructor to create a node, specify IK solver, and output file.
   *
   * @param node_name The name of the node to create.
   * @param solver The name of the IK solver to use.
   * @param output_file The path of the output file for data collection.
   * @param options Additional node options.
   */
  IKBenchmarking(const std::string& node_name, const std::string& solver, const std::string output_file, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(rclcpp::Node::make_shared(node_name, options)),
        logger_(node_->get_logger()),
        robot_model_loader_(node_),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false)
  {   
      //Todo: Mohamed, customize data file name based on output file path
      data_file_.open(std::string(solver)+"_ik_benchmarking_data.csv", std::ios::app);
      data_file_ << "trial,found_ik,solve_time,position_error,orientation_error,joints_error\n";
  }
  
  /**
   * @brief Start collecting benchmarking data.
   * 
   * This function calls other member functions responsible for 
   * initializing relevant member variables start the process of
   * collecting IK solving data.
   */
  void run();

  /**
   * @brief Get the success rate of the solver.
   *
   * @return Success rate as a percentage.
   */
  double get_success_rate() const;

  /**
   * @brief Get the average solve time.
   *
   * @return Average solve time in microseconds.
   */
  double get_average_solve_time() const;

  /**
   * @brief Check if the benchmarking calculation is done.
   *
   * @return True if done, false otherwise.
   */
  bool calculation_done() const;

private:
  rclcpp::Node::SharedPtr node_; ///< Shared pointer to a ROS 2 node.
  rclcpp::Logger logger_; ///< ROS 2 logger for this class.

  robot_model_loader::RobotModelLoader robot_model_loader_; ///< Loader for the robot model to be utilized for solving IK.
  moveit::core::RobotModelPtr robot_model_; ///< Shared pointer to the robot model to be loaded using the robot_model_loader_.
  moveit::core::RobotStatePtr robot_state_; ///< Shared pointer to the robot state which carries information about joint values.

  std::string move_group_name_; ///< The name of the move group (planning group) for which to compute IK.
  const moveit::core::JointModelGroup *joint_model_group_; ///< Pointer to the joint model group used to get information about the group and solve IK.

  std::string tip_link_name_; ///< The name of the tip link in the planning group used to solve IK.
  std::vector<JointBounds> joint_bounds_; ///< Vector containing the bounds of each joint in the planning group.
  std::vector<std::string> joint_names_; ///< Names of the joints in the planning group.

  std::random_device rd_; ///< Random device to seed random number generator.
  std::mt19937 generator_; ///< Generator for random joint values within bounds.

  unsigned int sample_size_; ///< The number of samples to run for collecting benchmarking data.
  double success_count_; ///< Count of successful IK solves from among the whole sample size.
  std::vector<int> solve_times_; ///< Times taken for each successful solve, in microseconds.
  double average_solve_time_; ///< Overall average solve time for all the successful samples.
  double success_rate_;  ///< Overall success rate as a percentage.
  bool calculation_done_; ///< Flag to indicate whether the calculation is done.

  std::ofstream data_file_; ///< Output file stream for IK solving data.

  /**
   * @brief Initialize the relevant class variables and prepare for benchmarking.
   */
  void initialize();

  /**
   * @brief Collect and save the benchmarking data.
   */
  void gather_date();
};
