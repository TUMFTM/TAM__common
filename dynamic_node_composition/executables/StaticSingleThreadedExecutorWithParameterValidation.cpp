// Copyright 2024
#include <dynamic_node_composition/composition.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_handler_cpp/vehicle_handler.hpp>

#include "param_management_ros2_integration_cpp/helper_functions.hpp"
/// Load an arbitrary amount of components into a StaticSingleThreadedExecutor
/// Additionally validate parameter overrides.
int main(int argc, char ** argv)
{
  std::cout << "Starting TUM Autnomous Motorsport's Dynamic Node Composition" << std::endl;

  rclcpp::init(argc, argv);

  // Init Vehicle Handler
  std::unique_ptr<tam::common::VehicleHandler> vehicle_ =
    tam::common::VehicleHandler::from_pkg_config();
  vehicle_->init_param_backend();
  std::cout << "Initialzed Parameter backend via VehicleHandler!" << std::endl;

  // TODO(Simon S) Add function for initing the parameter backend. This might not be possible
  // without name conflicts....

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto nodes = tam::dynamic_compositon::load_and_create_nodes(argc, argv, options);
  if (nodes.empty()) {
    std::cout << "No components have been specified! Exiting!" << std::endl;
    return 0;
  }
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto node : nodes) {
    // Validate Parameter
    tam::pmg::validate_param_overrides(
      argc, argv, node.get());  // Protects against trying to set non existant parameters
    executor.add_node(node);
    std::cout << "StaticSingleThreadedExecutor | Added Node: " << node->get_fully_qualified_name()
              << " | Parameter Loading Validated" << std::endl;
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
