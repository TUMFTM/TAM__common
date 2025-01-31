# ros2_watchdog_py
## topic_watchdog
Allows to monitor the subscription timeout of a topic. To connect the watchdog to you nodes on_update function checkout [monitor_example.py](ros2_watchdog_py/monitor_example.py)

The recommended way if you already have a timer callback in your node is to use the watchdog according to: [monitor_example.py](ros2_watchdog_py/monitor_example.py).

## node_monitor
Creates a publisher for diagnostic_msgs in your node and provides functions to adjust the published message. Check [monitor_example.py](ros2_watchdog_py/monitor_example.py). Standalone it can be used similar to the topic_watchdog.