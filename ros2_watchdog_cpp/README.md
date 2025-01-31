# ros2_watchdog_cpp

This library exists to give a common solution for making sure that new data is arriving on time. Instead of tracking this in a module itself, the module can use this library to make tracking timeouts a simple process.

Full example using everything there is in this package is located [in this file](./src/example_monitor.cpp).

## 👀 Topic Watchdog 👀

Instead of creating a simple subscription:

```cpp
this->create_subscription<std_msgs::msg::Int16>(
    "test", // Topic
    10, // QoS
    std::bind(&TestNode::topic_callback, this, _1) // Message callback
);
```

Instead do this to automatically check for timeouts on important topics:

```cpp
// Add this to your node
topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);

topic_watchdog_->add_subscription<std_msgs::msg::Int16>(
    "test", // Topic
    10, // QoS
    std::bind(&TestNode::topic_callback, this, _1), // Message callback
    std::bind(&TestNode::timeout_callback, this, _1, _2), // Timeout callback
    200ms // Timeout is triggered after 200ms
);

...

// Required function signature
void timeout_callback(bool timeout, std::chrono::milliseconds timeout_now)
{
    // Do something in case of timeout
    // e.g. report timeout in node monitor
}

...

// You will still need to trigger the check for timeouts manually
void timer_callback()
{
    topic_watchdog_->check_timeouts();
}
```

## 📺 Node Monitor 📺

Additionally this library provides a Node Monitor. This allows you to easily integrate a node status for your module that works with regular diagnostics and the dashboard.

```cpp
//Add this to your node
monitor_ = std::make_unique<tam::core::NodeMonitor>(this);

monitor_->initialization_finished(); // Finish initialization
...
monitor_->set_status_code(30);  // Set your modules current status code for dashboard
...
monitor_->report_value("std_dev", static_cast<int>(msg->data)); // Report data for dashboard
...
// Report errors
if (msg->data > 2.0)
{
    monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::ERROR); // Set error
} else {
    monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::OK); // Otherwise clear error
}

// You will still need to trigger the node status manually
void timer_callback()
{
    monitor_->update();
}
```