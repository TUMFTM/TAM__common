#include <rclcpp/rclcpp.hpp>

#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
#include "std_msgs/msg/int16.hpp"
#include "tum_helpers_cpp/containers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
class TestNode : public rclcpp::Node
{
private:
  tam::core::NodeMonitor::UniquePtr monitor_;
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr obj_subs_;

public:
  TestNode() : Node("Test")
  {
    tam::core::TimeoutValueProvider timeout_values;
    monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
    topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
    pub_timer_ = this->create_wall_timer(500ms, std::bind(&TestNode::function_queue_callback,this));

    // Setup monitor
    topic_watchdog_->add_subscription<std_msgs::msg::Int16>(
      "test", 10, std::bind(&TestNode::topic_callback, this, _1),
      std::bind(&TestNode::timeout_callback, this, _1, _2),
      timeout_values.value_ms("TrackingController"));

    // Alternatively -> Manually timeout subscription
    // NOTE: Do not use auto as return type here. If so the compiler
    // won't be able to deduce the type when trying to register the timeoutet
    // callback in the create_subscription method of the rclcpp node
    std::function<void(std_msgs::msg::Int16::SharedPtr)> timeoutet_callback =
      topic_watchdog_->timeout_callback(
        [this](auto msg) { topic_callback(msg); },
        [this](auto a, auto b) { this->timeout_callback(a, b); },
        timeout_values.value_ms("TrackingController"));

    this->create_subscription<std_msgs::msg::Int16>("/test_sub", 10, timeoutet_callback);

    // Set the initialization finished separately
    monitor_->initialization_finished();
    monitor_->set_status_code(30);  // Can be module specific (only for debugging purpose!)
  }
  void function_queue_callback()
  {
    topic_watchdog_->get_update_function();
    update_callback();
    monitor_->get_update_function();
  }
  void update_callback() { monitor_->set_message("Running"); }
  void topic_callback(std_msgs::msg::Int16::SharedPtr msg)
  {
    monitor_->report_value("std_dev", static_cast<int>(msg->data));
    if (msg->data > 2.0) {
      monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::ERROR);
    } else {
      monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::OK);
    }
  }
  void timeout_callback(bool timeout, std::chrono::milliseconds timeout_now)
  {
    monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::OK);
    if (timeout_now > 300ms) {
      monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::WARN);
    }
    if (timeout) {
      monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::ERROR);
    }
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
