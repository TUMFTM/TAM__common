from rclpy.node import Node
import rclpy
from topic_watchdog import TopicWatchdog
from node_monitor import NodeMonitor
from tum_types_py.common import ErrorLvl
from std_msgs.msg import Int16
from rclpy.duration import Duration

class TestNode(Node):
    def __init__(self):
        super().__init__("TestNode")

        self.node_monitor = NodeMonitor(self)
        self.topic_watchdog = TopicWatchdog(self)
        self.create_timer(0.1, self.function_queue_callback)
        
        self.topic_watchdog.add_subscription(
            Int16,
            "test",
            10,
            self.topic_callback,
            self.timeout_callback,
            5.0
        )

        self.node_monitor.initialization_finished()
        self.node_monitor.set_status_code(30)

    def function_queue_callback(self):
        self.topic_watchdog.get_update_function()
        self.update_callback()
        self.node_monitor.get_update_function()

    def update_callback(self):
        self.node_monitor.set_message("Running")

    def topic_callback(self, msg: Int16):
        self.node_monitor.report_value("std_dev", msg.data)
        if msg.data > 2.0:
            self.node_monitor.set_error_lvl("std_dev_1", ErrorLvl.ERROR)
        else:
            self.node_monitor.set_error_lvl("std_dev_1", ErrorLvl.OK)

    def timeout_callback(self, timeout: bool, timeout_now: Duration):
        self.node_monitor.set_error_lvl("topic_timeout", ErrorLvl.OK)
        if timeout_now.nanoseconds * 1e-9 > 0.3:
            self.node_monitor.set_error_lvl("topic_timeout", ErrorLvl.WARN)
        if timeout:
            self.node_monitor.set_error_lvl("topic_timeout", ErrorLvl.ERROR)

def main(args=None):
    """Execute tracking_node."""
    rclpy.init(args=args)

    test_node = TestNode()

    rclpy.spin(test_node)

    rclpy.shutdown()

    test_node.destroy_node()


if __name__ == "__main__":
    main()
