#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <memory>

#include "param_management_cpp/param_value_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "string"
namespace tam::core
{
class TimeoutValueProvider
{
public:
  TimeoutValueProvider()
  {
    // declare parameters
    declare_parameters();
    // Get Path to timeout.yml
    std::filesystem::path path = ament_index_cpp::get_package_share_directory("ros2_watchdog_cpp")
                                   .append("/config/timeout.yml");
    // Fill param manager with values in timeout.yml
    tam::pmg::load_overwrites_from_yaml(
      param_manager_.get(), path.string(), "/TimeoutValues_ms", true);

    // Save this values to avoid lookup at runtime
    multiplier = param_manager_->get_value("Multiplier").as_double();
    default_timeout = param_manager_->get_value("Default").as_int();
  }
  /**
   * @brief Get the timeout value in ms of a certain node. Will throw an exception if parameter does
   * not exist.
   *
   * @param node Node Name
   * @return int timeout value in ms
   */
  int value(const std::string & node)
  {
    return param_manager_->get_value(node).as_int() * multiplier;
  }
  /**
   * @brief Get the timeout value in std::chrono::milliseconds of a certain node. Will throw an
   * exception if parameter does not exist.
   *
   * @param node Node Name
   * @return int timeout value in std::chrono::milliseconds
   */
  std::chrono::milliseconds value_ms(const std::string & node)
  {
    return std::chrono::milliseconds(
      static_cast<int>(param_manager_->get_value(node).as_int() * multiplier));
  }
  /**
   * @brief Get the timeout value in ms of a certain node. If no timeout for this node was
   * specified, the default value is returned. Won't throw an exception if parameter does not exist.
   *
   * @param node Node Name
   * @return int timeout value in ms
   */
  int default_or(const std::string & node)
  {
    if (static_cast<tam::pmg::MgmtInterface *>(param_manager_.get())->has_parameter(node)) {
      return param_manager_->get_value(node).as_int() * multiplier;
    }
    return default_timeout * multiplier;
  }
  /**
   * @brief Get the timeout value in std::chrono::milliseconds of a certain node. If no timeout for
   * this node was specified, the default value is returned. Wont throw an exception if parameter
   * does not exist.
   *
   * @param node Node Name
   * @return int timeout value in ms
   */
  std::chrono::milliseconds default_or_ms(const std::string & node)
  {
    if (static_cast<tam::pmg::MgmtInterface *>(param_manager_.get())->has_parameter(node)) {
      return std::chrono::milliseconds(
        static_cast<int>(param_manager_->get_value(node).as_int() * multiplier));
    }
    return std::chrono::milliseconds(static_cast<int>(default_timeout * multiplier));
  }

private:
  double multiplier{1.0};
  int default_timeout{500};
  tam::pmg::ParamValueManager::UniquePtr param_manager_ =
    std::make_unique<tam::pmg::ParamValueManager>();
  void declare_parameters()
  {
    param_manager_->declare_parameter("Default", 500, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("Multiplier", 1.0, tam::pmg::ParameterType::DOUBLE, "");
    param_manager_->declare_parameter(
      "TrackingController", 40, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter(
      "TrackingPrediction", 100, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("JoystickManual", 1000, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter(
      "JoystickAutonomous", 4000, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("WatchdogNode", 100, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("RacelinePlanner", 500, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("SamplingPlanner", 500, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("StateEstimation", 40, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter(
      "JoystickEngineShutoff", 60000, tam::pmg::ParameterType::INTEGER, "");
    param_manager_->declare_parameter("StateMachine", 100, tam::pmg::ParameterType::INTEGER, "");
  }
};
}  // namespace tam::core
