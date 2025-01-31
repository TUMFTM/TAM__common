from typing import Union

from tum_types_py.control import (
    Trajectory,
    TrajectoryPoint,
    AdditionalInfoPoint,
    AdditionalTrajectoryInfos,
    ControlConstraintPoint,
    ControlConstraints,
    Odometry,
    AccelerationwithCovariances,
)

# from nav_msgs.msg import Odometry_msg
from tier4_planning_msgs.msg import TrajectoryPoint as TrajectoryPoint_msg, Trajectory as Trajectory_msg

from tum_helpers_py.type_conversion import Vector3D_type_from_EulerYPR_type

from tum_type_conversions_ros_py.common import Vector3D_type_from_msg, Header_type_from_msg
from tum_type_conversions_ros_py.orientation import quaternion_msg_to_euler_type

from tum_msgs.msg import (
    TUMAdditionalInfoPoint,
    TUMAdditionalTrajectoryInfos,
    TUMControlConstraints,
    TUMControlConstraintPoint,
)

from nav_msgs.msg import Odometry as Odometry_msg
from geometry_msgs.msg import AccelWithCovariance, AccelWithCovarianceStamped


def TrajectoryPoint_type_from_msg(point: TrajectoryPoint_msg):
    traj_point = TrajectoryPoint()
    traj_point.position_m = Vector3D_type_from_msg(point.pose.position)
    traj_point.orientation_rad = Vector3D_type_from_EulerYPR_type(
        quaternion_msg_to_euler_type(point.pose.orientation)
    )
    traj_point.velocity_mps = Vector3D_type_from_msg(point.twist.linear)
    traj_point.angular_velocity_radps = Vector3D_type_from_msg(point.twist.angular)
    traj_point.acceleration_mps2 = Vector3D_type_from_msg(point.accel.linear)
    traj_point.angular_acceleration_radps2 = Vector3D_type_from_msg(point.accel.angular)

    return traj_point


def Trajectory_type_from_msg(msg: Trajectory_msg) -> Trajectory:
    traj = Trajectory()
    traj.header = Header_type_from_msg(msg.header)
    traj.points = [TrajectoryPoint_type_from_msg(point) for point in msg.points]

    return traj


def ControlConstraintPoint_type_from_msg(msg: TUMControlConstraintPoint) -> ControlConstraintPoint:
    constr_point = ControlConstraintPoint()

    constr_point.a_x_min_mps2 = msg.ax_min_mps2
    constr_point.a_x_max_mps2 = msg.ax_max_mps2
    constr_point.a_y_min_mps2 = msg.ay_min_mps2
    constr_point.a_y_max_mps2 = msg.ay_max_mps2
    constr_point.a_x_max_engine_mps2 = msg.ax_max_engine_mps2
    constr_point.shape_factor_acceleration = msg.shape_factor_acceleration
    constr_point.shape_factor_deceleration = msg.shape_factor_deceleration
    constr_point.lateral_error_min_m = msg.lateral_error_min_m
    constr_point.lateral_error_max_m = msg.lateral_error_max_m

    return constr_point


def ControlConstraints_type_from_msg(msg: TUMControlConstraints) -> ControlConstraints:
    control_constraints = ControlConstraints()
    control_constraints.header = Header_type_from_msg(msg.header)
    control_constraints.points = [ControlConstraintPoint_type_from_msg(point) for point in msg.points]

    return control_constraints


def AdditionalInfoPoint_type_from_msg(msg: TUMAdditionalInfoPoint) -> AdditionalInfoPoint:
    add_info = AdditionalInfoPoint()
    add_info.s_global_m = msg.s_global_m
    add_info.s_local_m = msg.s_local_m
    add_info.kappa_1pm = msg.kappa_1pm
    add_info.lap_cnt = msg.lap_cnt
    return add_info


def AdditionalTrajectoryInfos_type_from_msg(msg: TUMAdditionalTrajectoryInfos) -> AdditionalTrajectoryInfos:
    add_info_type = AdditionalTrajectoryInfos()
    add_info_type.header = Header_type_from_msg(msg.header)
    add_info_type.points = [AdditionalInfoPoint_type_from_msg(point) for point in msg.points]

    return add_info_type


def Odometry_type_from_msg(msg: Odometry_msg) -> Odometry:
    odom = Odometry()
    odom.position_m = Vector3D_type_from_msg(msg.pose.pose.position)

    odom.orientation_rad = Vector3D_type_from_EulerYPR_type(
        quaternion_msg_to_euler_type(msg.pose.pose.orientation)
    )

    odom.pose_covariance = msg.pose.covariance

    odom.velocity_mps = Vector3D_type_from_msg(msg.twist.twist.linear)
    odom.angular_velocity_radps = Vector3D_type_from_msg(msg.twist.twist.angular)
    odom.velocity_covariance = msg.twist.covariance

    return odom


def AccelerationwithCovariances_type_from_msg(msg: Union[AccelWithCovarianceStamped, AccelWithCovariance]):
    accel = AccelerationwithCovariances()

    accel.acceleration_mps2 = Vector3D_type_from_msg(msg.accel.accel.linear)
    accel.angular_acceleration_radps2 = Vector3D_type_from_msg(msg.accel.accel.angular)
    accel.acceleration_covariance = msg.accel.covariance

    return accel
