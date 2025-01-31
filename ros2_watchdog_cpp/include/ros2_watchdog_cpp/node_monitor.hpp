#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"

using namespace std::chrono_literals;
namespace tam::core
{
/// @brief  If instantiated with a Node Handle, this publishes diagnostic_msgs/Status for the
/// current node. Use public functions to set tam::types::ErrorLvl, Message or values
class NodeMonitor
{
public:
  /// @brief Create a Monitor for your Node
  /// @param node Instance to your parent Node
  /// @param automatic_causing_key will set the key-value pair "causing_key" automatically based on
  /// the key associated with the error (default=true). Set to false if you want to specify the
  /// "causing_key" manually
  explicit NodeMonitor(rclcpp::Node * node, bool automatic_causing_key = true);
  /// @brief Set the Error Level for a specific key. The published Error Level is the worst Error
  /// Level across all keys
  /// @param key Key to the according error level.
  /// @param lvl Error Level (OK, WARN, ERROR)
  void set_error_lvl(const std::string & key, const tam::types::ErrorLvl & lvl);
  /// @brief Update the Message that is published within the diagnostic_msgs/Status
  /// @param message Message
  void set_message(const std::string & message);
  /// @brief Will set the status message from "Initializing"- ERROR, to "Initialized" - OK
  void initialization_finished();
  /// @brief Adds a key-value pair to the diagnostic_msgs/Status.
  /// @param name name of the value
  /// @param value value
  void report_value(const std::string & name, const std::string & value);
  void report_value(const std::string & name, const int value)
  {
    report_value(name, std::to_string(value));
  }
  void report_value(const std::string & name, const double value)
  {
    report_value(name, std::to_string(value));
  }
  /// @brief Adds a user defined status code diagnostic_msgs/Status in the key-value field
  /// "status_code"
  /// @param code User defined status code. Only for debugging purpose.
  void set_status_code(const int code);
  /// @brief returns a std::function of the NodeMonitors update function
  std::function<void()> get_update_function();
  tam::types::ErrorLvl get_max_error_lvl();
  int get_status_code();
  void update();

private:
  std::vector<diagnostic_msgs::msg::KeyValue> get_key_value_msg();

  rclcpp::Node * node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  diagnostic_msgs::msg::DiagnosticStatus status_msg_;
  std::unordered_map<std::string, tam::types::ErrorLvl> error_items_;
  std::unordered_map<std::string, std::string> key_value_map_;
  bool automatic_causing_key_{true};

public:
  using SharedPtr = std::shared_ptr<tam::core::NodeMonitor>;
  using UniquePtr = std::unique_ptr<tam::core::NodeMonitor>;
};
}  // namespace tam::core
