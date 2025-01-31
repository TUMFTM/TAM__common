#include "ros2_watchdog_cpp/node_monitor.hpp"
namespace tam::core
{
NodeMonitor::NodeMonitor(rclcpp::Node * node, bool automatic_causing_key)
: node_(node), automatic_causing_key_(automatic_causing_key)
{
  status_pub_ = this->node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    std::string("/core/orchestration/") + std::string(this->node_->get_name()) +
      std::string("_status"),
    1);

  status_msg_.name = this->node_->get_name();
  set_error_lvl("startup", tam::types::ErrorLvl::ERROR);
  set_message("Initializing");
}
std::function<void()> NodeMonitor::get_update_function()
{
  return std::bind(&NodeMonitor::update, this);
}
int NodeMonitor::get_status_code() { return std::stoi(key_value_map_.at("status_code")); }
void NodeMonitor::update()
{
  status_msg_.level = tam::type_conversions::diagnostic_level_from_type(get_max_error_lvl());
  status_msg_.values = get_key_value_msg();
  status_pub_->publish(status_msg_);
}
void NodeMonitor::set_error_lvl(const std::string & key, const tam::types::ErrorLvl & lvl)
{
  error_items_.insert_or_assign(key, lvl);
}
void NodeMonitor::set_message(const std::string & message) { status_msg_.message = message; }
void NodeMonitor::initialization_finished()
{
  set_error_lvl("startup", tam::types::ErrorLvl::OK);
  set_message("Initialized");
}
void NodeMonitor::set_status_code(const int code) { report_value("status_code", code); }
void NodeMonitor::report_value(const std::string & name, const std::string & value)
{
  key_value_map_.insert_or_assign(name, value);
}
tam::types::ErrorLvl NodeMonitor::get_max_error_lvl()
{
  auto min = std::max_element(
    error_items_.begin(), error_items_.end(),
    [](const auto & l, const auto & r) { return l.second < r.second; });
  if (automatic_causing_key_) {
    report_value("causing_key", (*min).first);
  }
  return (*min).second;
}
std::vector<diagnostic_msgs::msg::KeyValue> NodeMonitor::get_key_value_msg()
{
  std::vector<diagnostic_msgs::msg::KeyValue> msg;
  for (const auto & item : key_value_map_) {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = item.first;
    key_value.value = item.second;
    msg.push_back(key_value);
  }
  return msg;
}
}  // namespace tam::core
