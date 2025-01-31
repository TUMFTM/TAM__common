// Copyright 2024 Simon Sagmeister
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/has_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/split.hpp"
namespace tam::dynamic_compositon::utils
{
std::string get_shared_library_path(ComponentDescription const & comp);
rclcpp::Node::SharedPtr create_node_from_library(
  std::string library_path, std::string class_name, rclcpp::NodeOptions options);
}  // namespace tam::dynamic_compositon::utils
