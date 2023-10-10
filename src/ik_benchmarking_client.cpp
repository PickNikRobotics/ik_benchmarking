#include <chrono>
#include <functional>
#include <memory>

#include "ik_benchmarking/action/ik_benchmark.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace {
constexpr auto kActionClientTimeout = std::chrono::seconds{5};
}

using IKBenchmark = ik_benchmarking::action::IKBenchmark;
using Client = rclcpp_action::Client<IKBenchmark>;

int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ik_benchmarking_client", node_options);

    RCLCPP_INFO(node->get_logger(), "IK Benchmarking action client started.");

    // Get the solver and output path with ros parameters
    std::string ik_solver{};
    try {
        ik_solver = node->get_parameter("ik_solver").as_string();
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
        RCLCPP_ERROR(
            node->get_logger(),
            "Ik Benchmarking client failed to load 'ik_solver' parameter. Shutting down node.");
        rclcpp::shutdown();
        return 1;
    }

    const std::string csv_output_file = "ik_benchmarking_data";
    auto client = rclcpp_action::create_client<IKBenchmark>(node, "ik_benchmark");
    if (!client->wait_for_action_server(kActionClientTimeout)) {
        RCLCPP_ERROR(node->get_logger(), "Timed out waiting for IK Benchmarking server.");
        rclcpp::shutdown();
        return 1;
    }

    // Send a goal to the server
    auto goal = IKBenchmark::Goal();
    goal.solver_name = ik_solver;
    goal.csv_filename = csv_output_file;

    auto send_goal_options = rclcpp_action::Client<IKBenchmark>::SendGoalOptions();

    // Set up a callback function to handle the result
    send_goal_options.result_callback =
        [&](const rclcpp_action::ClientGoalHandle<IKBenchmark>::WrappedResult& result) {
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
