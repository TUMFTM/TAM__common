from geometry_msgs.msg import Quaternion
from tum_types_py.common import EulerYPR
from tum_type_conversions_ros_py._cpp_binding import (
    _bound_quaternion_msg_to_euler_type,
    _bound_euler_type_to_quaternion_msg,
)


def euler_type_to_quaternion_msg(euler_angles: EulerYPR) -> Quaternion:
    q_msg = Quaternion()
    coeff_list = _bound_euler_type_to_quaternion_msg(euler_angles)
    q_msg.x = coeff_list[0]
    q_msg.y = coeff_list[1]
    q_msg.z = coeff_list[2]
    q_msg.w = coeff_list[3]
    return q_msg


def quaternion_msg_to_euler_type(msg: Quaternion) -> EulerYPR:
    return _bound_quaternion_msg_to_euler_type(msg.x, msg.y, msg.z, msg.w)
