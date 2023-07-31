#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <limits.h>
#include <random>
#include <chrono>
#include <numeric>
#include <fstream>

#include <rclcpp_action/rclcpp_action.hpp>
#include "ik_benchmarking/action/ik_benchmark.hpp"
#include "ik_benchmarking/ik_benchmarking.hpp"
#include "ik_benchmarking_server/ik_benchmarking_server.hpp"

// using IKBenchmark = ik_benchmarking::action::IKBenchmark;
// using GoalHandleIKBenchmark = rclcpp_action::ServerGoalHandle<IKBenchmark>;

// class IKBenchmarkingServer : public rclcpp::Node
// {
// public: 
//   IKBenchmarkingServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()):
//   Node("ik_benchamrking_server", options)
//   {
//     RCLCPP_INFO(this->get_logger(), "Inside IKBenchmarkingServer constructor");
//     using namespace std::placeholders; 

//     this->action_server_ = rclcpp_action::create_server<IKBenchmark>(
//       this,
//       "ik_benchmark",
//       std::bind(&IKBenchmarkingServer::handle_goal, this, _1, _2),
//       std::bind(&IKBenchmarkingServer::handle_cancel, this, _1),
//       std::bind(&IKBenchmarkingServer::handle_accepted, this, _1));

//   }

// private:
// rclcpp_action::Server<IKBenchmark>::SharedPtr action_server_; 

// rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
// std::shared_ptr<const IKBenchmark::Goal> goal){
//   RCLCPP_INFO(this->get_logger(), "Inside handle_goal function");
//   RCLCPP_INFO(this->get_logger(), "Received IKBenchmark goal request with solver %s", goal->solver_name.c_str());
//   (void)uuid;
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle){
//   RCLCPP_INFO(this->get_logger(), "Inside handle_cancel function");
//   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// void handle_accepted(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle){
//   RCLCPP_INFO(this->get_logger(), "Inside handle_accepted function");
//   using namespace std::placeholders;
//   // std::function<void(std::shared_ptr<GoalHandleIKBenchmark>)> execute_callback = 
//   // std::bind(&IKBenchmarkingServer::execute, this, _1);

//   std::thread{std::bind(&IKBenchmarkingServer::execute, this, _1), goal_handle}.detach();

// }

// void execute(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle){
//   RCLCPP_INFO(this->get_logger(), "Inside execute function");
//   RCLCPP_INFO(this->get_logger(), "Executing goal");

//   rclcpp::Rate loop_rate(1);//Todo: Mohamed, check this rate
//   const auto goal = goal_handle->get_goal();
//   auto progress = std::make_shared<IKBenchmark::Feedback>();//Todo: Mohamed, handle the progress feedback

//   // Set the solver and output file path
//   // Todo: Mohamed, refactor the Benchmarking class to receive a solver and an output file
//   IKBenchmarking ik_benchmarker("ik_benchmarker", goal->solver_name.c_str(), goal->csv_filename.c_str(), this->get_node_options());
//   ik_benchmarker.run(); 

//   // Create result message 
//   auto result = std::make_shared<IKBenchmark::Result>();

//   while(!ik_benchmarker.calculation_done()){
//     loop_rate.sleep();
//   }
//   // Todo: Mohamed, make sure this does not run until the solving is done
//   result->calculation_done = true;
//   result->success_rate = ik_benchmarker.get_success_rate(); //Todo: Mohamed, check this rate
//   result->average_solve_time = ik_benchmarker.get_average_solve_time(); //Todo: Mohamed, check this rate
  
//   goal_handle->succeed(result);
// }
// };

int main(int argc, char *argv[])
{
  // Initialize the node
  // rclcpp::init(argc, argv);
  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  // auto node = rclcpp::Node::make_shared("ik_benchmarking_node", node_options);

  // IKBenchmarking ik_benchmarker(node);
  // ik_benchmarker.run();

  // For the server node to start 
  // Todo: Mohamed, how to insert the node name?
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto action_server = std::make_shared<IKBenchmarkingServer>(node_options);
  
  RCLCPP_INFO(action_server->get_logger(), "IK Benchmarking action server started.");

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}