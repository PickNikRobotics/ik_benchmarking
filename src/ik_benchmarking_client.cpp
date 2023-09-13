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
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ik_benchmarking_client", node_options);

    RCLCPP_INFO(node->get_logger(), "IK Benchmarking action client started.");

    // Get the solver and output path with ros parameters
    std::string ik_solver{};
    try{
        ik_solver = node->get_parameter("ik_solver").as_string();
    }
    catch(const rclcpp::exceptions::ParameterNotDeclaredException& e){
        RCLCPP_ERROR(node->get_logger(), "Ik Benchmarking client failed to load 'ik_solver' parameter. Shutting down node.");
        rclcpp::shutdown();
        return 1;
    }
    
    const std::string csv_output_file{ik_solver+std::string("_output")};
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
