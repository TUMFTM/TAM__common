from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.subscription import Subscription
from typing import Callable
from typing import TypeVar

T = TypeVar("T")


class WatchdogSubscription:
    def __init__(self, timeout_callback: Callable[[Duration], None], timeout: Duration, last_update: Time) -> None:
        self.timeout_callback = timeout_callback
        self.timeout = timeout
        self.last_update = last_update

    def set_sub(self, subscription: Subscription):
        self.subscription = subscription


class TopicWatchdog:
    def __init__(self, node: Node) -> None:
        self.node: Node = node
        self.subs = []

    def get_update_function(self):
        return self.timer_callback

    def timer_callback(self) -> None:
        now: Time
        now = self.node.get_clock().now()
        for sub in self.subs:
            sub: WatchdogSubscription
            timeout_now = now - sub.last_update
            sub.timeout_callback(timeout_now > sub.timeout, timeout_now)

    def add_subscription(
        self,
        msg_type: T,
        topic: str,
        qos: QoSProfile,
        subscription_callback: Callable[[T], None],
        timeout_callback: Callable[[Duration], None],
        timeout_sec: float,
    ) -> Subscription:
        watchdog_sub: WatchdogSubscription
        watchdog_sub = WatchdogSubscription(
            timeout_callback=timeout_callback,
            timeout=Duration(seconds=timeout_sec),
            last_update=self.node.get_clock().now(),
        )

        def sub_callback(msg):
            watchdog_sub.last_update = self.node.get_clock().now()
            subscription_callback(msg)

        ros_sub: Subscription = self.node.create_subscription(
            msg_type=msg_type, topic=topic, callback=sub_callback, qos_profile=qos
        )

        watchdog_sub.set_sub(ros_sub)

        self.subs.append(watchdog_sub)

        return ros_sub
