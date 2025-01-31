from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.subscription import Subscription
from typing import TypeVar, List, Callable
import diagnostic_msgs
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from tum_types_py.common import ErrorLvl
from tum_type_conversions_ros_py.common import diagnostic_level_from_type


class NodeMonitor:
    def __init__(self, node: Node) -> None:
        self.__node: Node = node
        self.__status_pub_ = self.__node.create_publisher(
            DiagnosticStatus, "/core/orchestration/" + self.__node.get_name() + "_status", 1
        )
        self.__error_items_ = {}
        self.__key_value_map_ = {}
        self.__status_msg_ = DiagnosticStatus()
        self.__status_msg_.name = self.__node.get_name()
        self.set_error_lvl("startup", ErrorLvl.ERROR)
        self.set_message("Initializing")

    def update(self) -> None:
        self.__status_msg_.level = diagnostic_level_from_type(self.get_max_error_lvl())
        self.__status_msg_.values = self.__get_key_value_msg()
        self.__status_pub_.publish(self.__status_msg_)

    def set_error_lvl(self, key: str, lvl: ErrorLvl) -> None:
        self.__error_items_[key] = lvl

    def set_message(self, message: str) -> None:
        self.__status_msg_.message = message

    def initialization_finished(self) -> None:
        self.set_error_lvl("startup", ErrorLvl.OK)
        self.set_message("Initialized")

    def report_value(self, name: str, value) -> None:
        self.__key_value_map_[name] = value

    def set_status_code(self, code: int) -> None:
        self.report_value("status_code", code)
    
    def get_status_code(self) -> None:
        return self.__key_value_map_["status_code"]

    def get_update_function(self) -> Callable:
        return self.update

    def get_max_error_lvl(self) -> ErrorLvl:
        key = max(self.__error_items_, key=self.__error_items_.get)
        self.report_value("causing_key", key)
        return self.__error_items_[key]
    
    def __get_key_value_msg(self) -> List[KeyValue]:
        key_value_list = []
        for item_key in self.__key_value_map_:
            key_value = KeyValue()
            key_value.key = item_key
            key_value.value = str(self.__key_value_map_[item_key])
            key_value_list.append(key_value)
        return key_value_list
