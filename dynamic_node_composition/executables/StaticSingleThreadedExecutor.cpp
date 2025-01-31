// Copyright 2024
#include <dynamic_node_composition/composition.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_handler_cpp/vehicle_handler.hpp"
/// Load an arbitrary amount of components into a StaticSingleThreadedExecutor

int main(int argc, char ** argv)
{
  std::cout << "Starting TUM Autonomous Motorsport's Dynamic Node Composition!" << std::endl;

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
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  if (nodes.empty()) {
    std::cout << "No components have been specified! Exiting!" << std::endl;
    return 0;
  }
  for (auto node : nodes) {
    executor.add_node(node);
    std::cout << "StaticSingleThreadedExecutor | Added Node: " << node->get_fully_qualified_name()
              << std::endl;
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
