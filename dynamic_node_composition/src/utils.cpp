// Copyright 2024 Simon Sagmeister
#include <dynamic_node_composition/types.hpp>
#include <dynamic_node_composition/utils.hpp>
namespace tam::dynamic_compositon::utils
{
std::string get_shared_library_path(ComponentDescription const & comp)
{
  std::string amend_index_content;
  std::string base_path;
  // check_package_name_is_valid(package_name);
  ament_index_cpp::get_resource(
    "rclcpp_components", comp.package_name, amend_index_content, &base_path);
  if (amend_index_content.find(comp.component_name) == std::string::npos) {
    throw exceptions::ComponentNotFound(comp);
  }

  std::vector<std::string> lines = rcpputils::split(amend_index_content, '\n', true);
  std::filesystem::path library_path;
  for (const auto & line : lines) {
    std::vector<std::string> parts = rcpputils::split(line, ';');
    // check_split_result<std::string>(parts, 2);
    if (parts[0] == comp.component_name) library_path = parts[1];
  }
  if (!library_path.is_absolute()) {
    return base_path / library_path;
  }
  return library_path;
}
rclcpp::Node::SharedPtr create_node_from_library(
  std::string library_path, std::string class_name, rclcpp::NodeOptions options)
{
  auto loader = std::make_unique<class_loader::ClassLoader>(library_path);
  class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";
  std::shared_ptr<rclcpp_components::NodeFactory> node_factory =
    loader->createInstance<rclcpp_components::NodeFactory>(class_name);
  // rclcpp has to be initialized otherwise this line gives a runtime error
  rclcpp_components::NodeInstanceWrapper wrapper = node_factory->create_node_instance(options);
  return std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
}
}  // namespace tam::dynamic_compositon::utils
