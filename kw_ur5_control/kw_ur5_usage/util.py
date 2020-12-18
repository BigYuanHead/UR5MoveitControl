# third-party libraries
from geometry_msgs.msg import Pose, Quaternion, Point, Transform, Vector3
import numpy as np
from tf import transformations

def matrix_4x4_to_pose_msg(trans_mat_npy_4x4):
    tmp_rot_mat = np.eye(4)
    tmp_rot_mat[:3, :3] = trans_mat_npy_4x4[:3, :3]
    quaternion_npy_array_4x1 = transformations.quaternion_from_euler(*transformations.euler_from_matrix(tmp_rot_mat))
    ros_geometry_quaternion = Quaternion(*quaternion_npy_array_4x1)
    x = trans_mat_npy_4x4[0, 3]
    y = trans_mat_npy_4x4[1, 3]
    z = trans_mat_npy_4x4[2, 3]
    ros_geometry_point = Point(x, y, z)
    pose_msg = Pose(ros_geometry_point, ros_geometry_quaternion)
    return pose_msg



def matrix_4x4_to_xyzrpy_6x1(trans_mat_npy_4x4):
    tmp_rot_mat = np.eye(4)
    tmp_rot_mat[:3, :3] = trans_mat_npy_4x4[:3, :3]
    x = trans_mat_npy_4x4[0, 3]
    y = trans_mat_npy_4x4[1, 3]
    z = trans_mat_npy_4x4[2, 3]
    roll, pitch, yaw = transformations.euler_from_matrix(tmp_rot_mat, 'rzyx')
    return np.array([x, y, z, roll, pitch, yaw])

def matrix_4x4_to_transform_msg(trans_mat_npy_4x4):
    tmp_rot_mat = np.eye(4)
    tmp_rot_mat[:3, :3] = trans_mat_npy_4x4[:3, :3]
    translation_npy_array_3x1 = trans_mat_npy_4x4[:3, 3]
    rotation_npy_array_4x1 = transformations.quaternion_from_matrix(tmp_rot_mat)
    ros_geometry_transform_msg = Transform(Vector3(*translation_npy_array_3x1),
                                           Quaternion(*rotation_npy_array_4x1))
    return ros_geometry_transform_msg

def xyz_xyzw_to_matrix_4x4(x, y, z, qx, qy, qz, qw):
    trans_mat_npy_4x4 = transformations.quaternion_matrix(np.array([qx, qy, qz, qw]))
    trans_mat_npy_4x4[0, 3] = x
    trans_mat_npy_4x4[1, 3] = y
    trans_mat_npy_4x4[2, 3] = z
    return trans_mat_npy_4x4

def pose_msg_to_matrix_4x4(pose_msg):
    trans_mat_npy_4x4 = transformations.quaternion_matrix(np.array([pose_msg.orientation.x,
                                                                    pose_msg.orientation.y,
                                                                    pose_msg.orientation.z,
                                                                    pose_msg.orientation.w]))
    trans_mat_npy_4x4[0, 3] = pose_msg.position.x
    trans_mat_npy_4x4[1, 3] = pose_msg.position.y
    trans_mat_npy_4x4[2, 3] = pose_msg.position.z
    return trans_mat_npy_4x4


def xyzrpy_to_matrix_4x4(x, y, z, roll, pitch, yaw):
    trans_mat_npy_4x4 = transformations.euler_matrix(roll, pitch, yaw, axes='rzyx')
    trans_mat_npy_4x4[0, 3] = x
    trans_mat_npy_4x4[1, 3] = y
    trans_mat_npy_4x4[2, 3] = z
    return trans_mat_npy_4x4

def transform_msg_to_matrix_4x4(transform_msg):
    trans_mat_npy_4x4 = xyz_xyzw_to_matrix_4x4(transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z,
                                                transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z, transform_msg.rotation.w)
    return trans_mat_npy_4x4




