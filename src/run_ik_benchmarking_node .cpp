#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ik_benchmarking/ik_benchmarking.hpp"

int main(int argc, char *argv[])
{
  // Initialize the node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ik_benchmarking_node", node_options);

  IKBenchmarking ik_benchmarker(node);
  ik_benchmarker.run();

  rclcpp::shutdown();
  return 0;
}