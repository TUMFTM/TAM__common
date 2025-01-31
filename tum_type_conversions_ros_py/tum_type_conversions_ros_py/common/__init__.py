from typing import Union

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header as Header_msg
from tum_types_py.common import Vector3D, Header, ErrorLvl
from diagnostic_msgs.msg import DiagnosticStatus

def Vector3D_type_from_msg(msg: Union[Point, Vector3]) -> Vector3D:
    return Vector3D(msg.x, msg.y, msg.z)

def Vector3D_type_to_msg(vector: Vector3) -> Vector3D:
    vector_out = Vector3()
    vector_out.x = vector.x
    vector_out.y = vector.y
    vector_out.z = vector.z

    return vector_out


def Vector3D_type_geometry_msgs_Point(vector: Vector3) -> Point:
    point = Point()
    point.x = vector.x
    point.y = vector.y
    point.z = vector.z

    return point


def Header_type_from_msg(header: Header_msg) -> Header:
    head = Header()
    head.time_stamp_ns = header.stamp.sec * 10**9 + header.stamp.nanosec
    head.frame_id = header.frame_id
    return head

def diagnostic_level_from_type(lvl: ErrorLvl) -> bytes:
    if lvl == ErrorLvl.OK:
        return DiagnosticStatus.OK
    elif lvl == ErrorLvl.STALE:
        return DiagnosticStatus.STALE
    elif lvl == ErrorLvl.ERROR:
        return DiagnosticStatus.ERROR
    elif lvl == ErrorLvl.WARN:
        return DiagnosticStatus.WARN
    else:
        print("Unknown Error Level")
        return DiagnosticStatus.ERROR

def error_type_from_diagnostic_level(lvl: bytes) -> ErrorLvl:
    if lvl == DiagnosticStatus.OK:
        return ErrorLvl.OK
    elif lvl == DiagnosticStatus.STALE:
        return ErrorLvl.STALE
    elif lvl == DiagnosticStatus.ERROR:
        return ErrorLvl.ERROR
    elif lvl == DiagnosticStatus.WARN:
        return ErrorLvl.WARN
    else:
        print("Unknown Error Level")
        return ErrorLvl.ERROR