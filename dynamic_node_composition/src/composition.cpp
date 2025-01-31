// Copyright 2024 Simon Sagmeister
#include <dynamic_node_composition/composition.hpp>
namespace tam::dynamic_compositon
{
rclcpp::Node::SharedPtr load_and_create_node(
  ComponentDescription const & comp, rclcpp::NodeOptions options)
{
  auto library_path = utils::get_shared_library_path(comp);
  return utils::create_node_from_library(library_path, comp.component_name, options);
}
std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
  std::vector<ComponentDescription> const & components, rclcpp::NodeOptions options)
{
  std::vector<rclcpp::Node::SharedPtr> nodes;
  for (auto const & comp : components) {
    nodes.push_back(load_and_create_node(comp, options));
  }
  return nodes;
}
std::vector<rclcpp::Node::SharedPtr> load_and_create_nodes(
  int argc, char ** argv, rclcpp::NodeOptions options)
{
  auto components = parse_components_from_command_line_arguments(argc, argv);
  return load_and_create_nodes(components, options);
}
std::vector<ComponentDescription> parse_components_from_command_line_arguments(
  int argc, char ** argv)
{
  std::vector<ComponentDescription> components;
  bool next_argv_is_component = false;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--component") {
      next_argv_is_component = true;
    }
    if (next_argv_is_component == true) {
      components.emplace_back(std::string(argv[i + 1]), std::string(argv[i + 2]));
      next_argv_is_component = false;
    }
  }
  return components;
}
}  // namespace tam::dynamic_compositon
