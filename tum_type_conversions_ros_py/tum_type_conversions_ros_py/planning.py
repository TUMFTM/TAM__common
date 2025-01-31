from tum_msgs.msg import TUMTrajectory
from tum_msgs.msg import (
    TUMAdditionalInfoPoint,
    TUMControlConstraintPoint,
    TUMAdditionalTrajectoryInfos,
    TUMControlConstraints,
)
from tier4_planning_msgs.msg import TrajectoryPoint, Trajectory
from tum_types_py.common import EulerYPR
from tum_type_conversions_ros_py.orientation import euler_type_to_quaternion_msg
from typing import Tuple


def new_trajectory_msgs_from_TUMTrajectory(
    old_trajectory_message: TUMTrajectory,
) -> Tuple[Trajectory, TUMControlConstraints, TUMAdditionalTrajectoryInfos]:
    num_points = len(old_trajectory_message.s_local_m)

    traj_msg = Trajectory()
    constraint_msg = TUMControlConstraints()
    add_info_msg = TUMAdditionalTrajectoryInfos()

    traj_msg.points = [TrajectoryPoint() for _ in range(num_points)]
    constraint_msg.points = [TUMControlConstraintPoint() for _ in range(num_points)]
    add_info_msg.points = [TUMAdditionalInfoPoint() for _ in range(num_points)]

    for index in range(num_points):
        traj_msg.points[index].pose.position.x = old_trajectory_message.x_m[index]
        traj_msg.points[index].pose.position.y = old_trajectory_message.y_m[index]
        traj_msg.points[index].pose.position.z = 0.0

        traj_msg.points[index].pose.orientation = euler_type_to_quaternion_msg(
            EulerYPR(old_trajectory_message.psi_rad[index], 0.0, old_trajectory_message.bank_rad[index])
        )

        traj_msg.points[index].twist.linear.x = old_trajectory_message.v_mps[index]
        traj_msg.points[index].twist.linear.y = 0.0
        traj_msg.points[index].twist.linear.z = 0.0
        traj_msg.points[index].accel.linear.x = old_trajectory_message.ax_mps2[index]
        traj_msg.points[index].accel.linear.y = (
            old_trajectory_message.v_mps[index] ** 2
        ) * old_trajectory_message.kappa_radpm[index]
        traj_msg.points[index].accel.linear.z = 0.0

        constraint_msg.points[index].ax_min_mps2 = -old_trajectory_message.ax_max_mps2[index]
        constraint_msg.points[index].ax_max_mps2 = old_trajectory_message.ax_max_mps2[index]
        constraint_msg.points[index].ay_min_mps2 = -old_trajectory_message.ay_max_mps2[index]
        constraint_msg.points[index].ay_max_mps2 = old_trajectory_message.ay_max_mps2[index]
        constraint_msg.points[index].ax_max_engine_mps2 = 0.0
        constraint_msg.points[index].shape_factor_acceleration = 0.0
        constraint_msg.points[index].shape_factor_deceleration = 0.0
        constraint_msg.points[index].lateral_error_max_m = old_trajectory_message.tube_l_m[index]
        constraint_msg.points[index].lateral_error_min_m = -old_trajectory_message.tube_r_m[index]

        add_info_msg.points[index].kappa_1pm = old_trajectory_message.kappa_radpm[index]
        add_info_msg.points[index].lap_cnt = float(
            old_trajectory_message.lap_cnt
        )  # FIXME Define lap cnt as int
        add_info_msg.points[index].s_global_m = old_trajectory_message.s_global_m[index]
        add_info_msg.points[index].s_local_m = old_trajectory_message.s_local_m[index]
        add_info_msg.points[index].time_s = old_trajectory_message.time_s[index]

    return traj_msg, constraint_msg, add_info_msg
