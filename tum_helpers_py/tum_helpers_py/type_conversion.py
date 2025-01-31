from tum_types_py.common import Vector3D, EulerYPR


def Vector3D_type_from_EulerYPR_type(euler_orientation: EulerYPR):
    return Vector3D(euler_orientation.roll, euler_orientation.pitch, euler_orientation.yaw)


def EulerYPR_type_from_Vector3D_type(euler_orientation: Vector3D):
    return EulerYPR(euler_orientation.z, euler_orientation.y, euler_orientation.x)