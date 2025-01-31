// Copyright 2024 Simon Sagmeister

#include <dynamic_node_composition/types.hpp>
#include <dynamic_node_composition/utils.hpp>
#include <vector>
namespace tam::dynamic_compositon
{
/// @brief Load an rclcpp component and create the corresponding node
/// @param comp The component description
/// @return rclcpp::Node::SharedPtr The ptr to the created node
rclcpp::Node::SharedPtr load_and_create_node(
  ComponentDescription const & comp, rclcpp::NodeOptions options = rclcpp::NodeOptions());

/// @brief Load rclcpp components and create the corresponding nodes
/// @param components A vector of component descriptions
/// @return A list of node shared ptr to the created nodes
std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
  std::vector<ComponentDescription> const & components,
  rclcpp::NodeOptions options = rclcpp::NodeOptions());

/// @brief Parse a list of component descriptions from command line arguments
/// @note The syntax for the command line arguments is:
/// @note --component <package_name1> <component_name1> --component <package_name2>
/// <component_name2>
/// @param argc Number of command line arguments
/// @param argv Arguments
/// @return A parsed list of component descriptions
std::vector<ComponentDescription> parse_components_from_command_line_arguments(
  int argc, char ** argv);

/// @brief Load rclcpp components and create the corresponding nodes from cli inputs.
/// @note For a detailed documentation of parsing the cli-args, see the function
/// `parse_components_from_command_line_arguments`
/// @param argc Number of command line arguments
/// @param argv Arguments
/// @return A list of node shared ptr to the created nodes
std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
  int argc, char ** argv, rclcpp::NodeOptions options = rclcpp::NodeOptions());
}  // namespace tam::dynamic_compositon
