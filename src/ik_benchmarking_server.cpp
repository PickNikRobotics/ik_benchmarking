#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ik_benchmarking/action/ik_benchmark.hpp"
#include "ik_benchmarking/ik_benchmarking.hpp"
#include "ik_benchmarking_server/ik_benchmarking_server.hpp"

using IKBenchmark = ik_benchmarking::action::IKBenchmark;
using GoalHandleIKBenchmark = rclcpp_action::ServerGoalHandle<IKBenchmark>;

IKBenchmarkingServer::IKBenchmarkingServer(const rclcpp::NodeOptions& options):
Node("ik_benchmarking_server", options)
{
  using namespace std::placeholders; 

  this->action_server_ = rclcpp_action::create_server<IKBenchmark>(
    this,
    "ik_benchmark",
    std::bind(&IKBenchmarkingServer::handle_goal, this, _1, _2),
    std::bind(&IKBenchmarkingServer::handle_cancel, this, _1),
    std::bind(&IKBenchmarkingServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse IKBenchmarkingServer::handle_goal(const rclcpp_action::GoalUUID & uuid, 
std::shared_ptr<const IKBenchmark::Goal> goal){
  RCLCPP_INFO(this->get_logger(), "Received IKBenchmark goal request with solver %s", goal->solver_name.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IKBenchmarkingServer::handle_cancel(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle){
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void IKBenchmarkingServer::handle_accepted(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle){
  using namespace std::placeholders;

  bool execute_once{true}; // Shutdown server node after handling one action goal to enable sequencing while collecting IK solution data
  std::thread{std::bind(&IKBenchmarkingServer::execute, this, _1, _2), goal_handle, execute_once}.detach();

}

void IKBenchmarkingServer::execute(const std::shared_ptr<GoalHandleIKBenchmark> goal_handle, bool execute_once){
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  rclcpp::Rate loop_rate(1);//Todo: Mohamed, check this rate
  const auto goal = goal_handle->get_goal();
  auto progress = std::make_shared<IKBenchmark::Feedback>();//Todo: Mohamed, handle the progress feedback

  // Set the solver and output file path
  // Here is the point to customize the server node name if needed
  IKBenchmarking ik_benchmarker("ik_benchmarker", goal->solver_name.c_str(), goal->csv_filename.c_str(), this->get_node_options());
  ik_benchmarker.run(); 

  // Create result message 
  auto result = std::make_shared<IKBenchmark::Result>();

  while(!ik_benchmarker.calculation_done()){
    loop_rate.sleep();
  }
  // Todo: Mohamed, handle the feedback publishing
  result->calculation_done = true;
  result->success_rate = ik_benchmarker.get_success_rate(); //Todo: Mohamed, check this rate
  result->average_solve_time = ik_benchmarker.get_average_solve_time(); //Todo: Mohamed, check this rate
  
  goal_handle->succeed(result);

  if (execute_once)
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto action_server = std::make_shared<IKBenchmarkingServer>(node_options);
  
  RCLCPP_INFO(action_server->get_logger(), "IK Benchmarking action server started.");

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
