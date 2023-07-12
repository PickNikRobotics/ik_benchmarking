#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ik_benchmarking/action/ik_benchmark.hpp"

using IKBenchmark = ik_benchmarking::action::IKBenchmark;
using Client = rclcpp_action::Client<IKBenchmark>;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_benchmarking_client");

    RCLCPP_INFO(node->get_logger(), "IK Benchmarking action client started.");

    // Todo: Mohamed, get the solver and output path with ros parameters

    // Setup solver names for testing 
    const auto ik_solver{"solver1"};
    const auto csv_output_file{"solver1_output"};
    auto client = rclcpp_action::create_client<IKBenchmark>(node, "ik_benchmark");
    
    // Send a goal to the server
    auto goal = IKBenchmark::Goal();
    goal.solver_name = ik_solver;
    goal.csv_filename = csv_output_file;
    
    auto send_goal_options = rclcpp_action::Client<IKBenchmark>::SendGoalOptions();


    // Setup a callback function to handle the result
    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<IKBenchmark>::WrappedResult & result) {
        if (result.result->calculation_done) {
            RCLCPP_INFO(node->get_logger(), "The IK Benchmarking action succeeded.");
        } else {
            RCLCPP_ERROR(node->get_logger(), "The IK Benchmarking action failed.");
        }
        // Shutdown after receiving the result
        rclcpp::shutdown();
    };

    client->async_send_goal(goal, send_goal_options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}