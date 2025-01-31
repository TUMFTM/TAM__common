// Copyright 2024 Maximilian Leitenstern

#pragma once

#include <boost/algorithm/string.hpp>
#include <boost/pfr.hpp>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "param_management_cpp/param_reference_manager.hpp"
#include "param_management_cpp/types.hpp"
#include "tum_types_cpp/vehicle.hpp"
#include "yaml-cpp/yaml.h"

// clang-format off
// Macro to execute for all param types (copied from param manager)
#define FOR_EVERY_SUPPORTED_PARAM_TYPE(MACRO)                \
  MACRO(BOOL, bool, bool)                                    \
  MACRO(INTEGER, std::int64_t, int)                          \
  MACRO(DOUBLE, double, double)                              \
  MACRO(STRING, std::string, string)                         \
  MACRO(BYTE_ARRAY, std::vector<std::uint8_t>, byte_array)   \
  MACRO(BOOL_ARRAY, std::vector<bool>, bool_array)           \
  MACRO(INTEGER_ARRAY, std::vector<std::int64_t>, int_array) \
  MACRO(DOUBLE_ARRAY, std::vector<double>, double_array)     \
  MACRO(STRING_ARRAY, std::vector<std::string>, string_array)
// clang-format on

namespace utils
{
// Required to check if a certain type is part of a variant
// From:
// https://stackoverflow.com/questions/45892170/how-do-i-check-if-an-stdvariant-can-hold-a-certain-type
template <class T>
struct type
{
};
template <class T>
constexpr type<T> type_v{};
template <class T, class... Ts, template <class...> class Tp>
constexpr bool is_one_of(type<Tp<Ts...>>, type<T>)
{
  return (std::is_same_v<Ts, T> || ...);
}
// Connect to struct for param_value_variant..
// Generic should maybe be part of param manager?!
void connect_to_param_manager(
  tam::pmg::param_ptr_variant_t & value, const std::string & name,
  tam::pmg::ParamReferenceManager * pmg)
{
  std::visit(
    [name, pmg](auto && arg) {
      using T = std::decay_t<decltype(*arg)>;
      if constexpr (std::is_same_v<T, bool>) {
        pmg->declare_parameter(name, arg, false, tam::pmg::ParameterType::BOOL, "");
      }
      if constexpr (std::is_same_v<T, int64_t>) {
        pmg->declare_parameter(name, arg, 0, tam::pmg::ParameterType::INTEGER, "");
      }
      if constexpr (std::is_same_v<T, double>) {
        pmg->declare_parameter(name, arg, 0.0, tam::pmg::ParameterType::DOUBLE, "");
      }
      if constexpr (std::is_same_v<T, std::string>) {
        pmg->declare_parameter(name, arg, "", tam::pmg::ParameterType::STRING, "");
      }
      if constexpr (std::is_same_v<T, std::vector<std::uint8_t>>) {
        pmg->declare_parameter(
          name, arg, std::vector<std::uint8_t>{}, tam::pmg::ParameterType::BYTE_ARRAY, "");
      }
      if constexpr (std::is_same_v<T, std::vector<bool>>) {
        pmg->declare_parameter(
          name, arg, std::vector<bool>{}, tam::pmg::ParameterType::BOOL_ARRAY, "");
      }
      if constexpr (std::is_same_v<T, std::vector<std::int64_t>>) {
        pmg->declare_parameter(
          name, arg, std::vector<std::int64_t>{}, tam::pmg::ParameterType::INTEGER_ARRAY, "");
      }
      if constexpr (std::is_same_v<T, std::vector<double>>) {
        pmg->declare_parameter(
          name, arg, std::vector<double>{}, tam::pmg::ParameterType::DOUBLE_ARRAY, "");
      }
      if constexpr (std::is_same_v<T, std::vector<std::string>>) {
        pmg->declare_parameter(
          name, arg, std::vector<std::string>{}, tam::pmg::ParameterType::STRING_ARRAY, "");
      }
    },
    value);
}
// Called only if value is part of param_value_variant_t
// Convert it to variant -> to use generic function above
// Required for connect to struct if you dont call it with a variant directly
template <typename T>
std::enable_if_t<is_one_of(type_v<tam::pmg::param_value_variant_t>, type_v<T>)> connect_to_struct(
  T * param, const std::string & param_name, tam::pmg::ParamReferenceManager * pmg)
{
  tam::pmg::param_ptr_variant_t ptr_variant = param;
  connect_to_param_manager(ptr_variant, param_name, pmg);
}
// Called if the Type is not part of param_value_variant_t.
// Currently assuming it is a struct
// TODO(Maxi): Maybe there is a solution to check for struct type
template <typename T>
std::enable_if_t<!is_one_of(type_v<tam::pmg::param_value_variant_t>, type_v<T>)> connect_to_struct(
  T * param, const std::string & param_name, tam::pmg::ParamReferenceManager * pmg)
{
  boost::pfr::for_each_field(*param, [param_name, pmg](auto & f, auto i) {
    // Currently not accounting for vectors!! use different check
    std::string name{boost::pfr::get_name<i, T>()};
    name = param_name + "." + name;
    connect_to_struct(&f, name, pmg);
  });
}
/**
 * @brief Connects all parameters of a struct to the param manager
 * @param pmg Param manager
 * @param vehicle Vehicle struct
 * @param tires Tire struct
 * @note -> requires all parameters of the struct to be called
 */
void assign_string_keys_to_storage(
  tam::pmg::ParamReferenceManager::UniquePtr & pmg, tam::types::vehicle_params::Vehicle & vehicle,
  tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & tires)
{
  // --------------------------
  // Vehicle Params
  // --------------------------
  connect_to_struct(&vehicle, "vehicle", pmg.get());
  // --------------------------
  // Tire Params
  // --------------------------
  connect_to_struct(&tires.front_left, "tires.front_left", pmg.get());
  connect_to_struct(&tires.front_right, "tires.front_right", pmg.get());
  connect_to_struct(&tires.rear_left, "tires.rear_left", pmg.get());
  connect_to_struct(&tires.rear_right, "tires.rear_right", pmg.get());
}
/**
 * @brief Extract value from yaml and set via param reference manager
 * @param pmg Param manager
 * @param config_path Path to config files
 * @param vehicle_name Name of the vehicle
 */
void set_from_config(
  const tam::pmg::ParamReferenceManager::UniquePtr & pmg, const std::string & config_path,
  const std::string & vehicle_name)
{
  // clang-format off
  std::filesystem::path file_path_vehicle = std::filesystem::path(config_path).append(vehicle_name).append("vehicle_config.yaml"); // NOLINT
  std::filesystem::path file_path_tire = std::filesystem::path(config_path).append(vehicle_name).append("tire_config.yaml"); // NOLINT
  if (!std::filesystem::exists(file_path_vehicle) || !std::filesystem::exists(file_path_tire)) {
    throw std::runtime_error("Config file not found: " + file_path_vehicle.string() + " or " + file_path_tire.string()); // NOLINT
  }
  // clang-format on

  // Load yaml files
  YAML::Node config_vehicle = YAML::LoadFile(file_path_vehicle);
  YAML::Node config_tire = YAML::LoadFile(file_path_tire);

  // Iterate over all parameters registered in param reference manager
  tam::pmg::MgmtInterface * interface = pmg.get();
  for (const auto & param : interface->list_parameters()) {
    // Split parameter into substrings
    std::vector<std::string> category{};
    boost::split(category, param, boost::is_any_of("."));
    // Get vehicle or tire config
    YAML::Node config = category.front() == "vehicle" ? Clone(config_vehicle["vehicle"])
                                                      : Clone(config_tire["tires"]);
    category.erase(category.begin());

    // Search for parameter in yaml file
    for (const auto & cat : category) {
      if (config[cat]) {
        config = config[cat];
      } else {
        throw std::runtime_error("Node not found in config file: " + cat);
      }
    }
    // Set value depending on param type
    // clang-format off
    #define SET_VALUE_FROM_CONFIG(enum_val, typename, name)                 \
      if (interface->get_type(param) == tam::pmg::ParameterType::enum_val) { \
        interface->set_value(param, config.as<typename>());                  \
      }
        FOR_EVERY_SUPPORTED_PARAM_TYPE(SET_VALUE_FROM_CONFIG);
    #undef SET_VALUE_FROM_CONFIG
    // clang-format on
  }
}
}  // namespace utils
