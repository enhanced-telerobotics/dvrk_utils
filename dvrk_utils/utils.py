import PyKDL
import numpy as np
from spatialmath import SE3, SO3
from spatialmath.base import q2r, r2q
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState


def stamped_to_se3(transform: TransformStamped) -> SE3:
    """
    Convert a TransformStamped message to a 4x4 transformation matrix.
    """
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    pose = SE3()
    pose.t = np.array([translation.x, translation.y, translation.z])
    pose.R = q2r([rotation.w, rotation.x, rotation.y, rotation.z])

    return pose


def frame_to_se3(frame: PyKDL.Frame) -> SE3:
    """
    Convert a frame to an SE3 object.
    """
    pose = SE3()
    pose.t = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    pose.R = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                       [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                       [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
    return pose


def se3_to_frame(pose: SE3) -> PyKDL.Frame:
    """
    Convert an SE3 object to a PyKDL Frame.
    """
    frame = PyKDL.Frame()
    frame.p = PyKDL.Vector(pose.t[0], pose.t[1], pose.t[2])
    frame.M = PyKDL.Rotation(pose.R[0, 0], pose.R[0, 1], pose.R[0, 2],
                             pose.R[1, 0], pose.R[1, 1], pose.R[1, 2],
                             pose.R[2, 0], pose.R[2, 1], pose.R[2, 2])
    return frame


def jp_config_to_command(jp_config: list) -> np.ndarray:
    jp_command = np.asarray(jp_config, dtype=float).copy()
    if jp_command.size >= 2:
        jp_command[:2] = np.deg2rad(jp_command[:2])
    if jp_command.size >= 3:
        jp_command[2] = jp_command[2] / 1000.0
    if jp_command.size > 3:
        jp_command[3:] = np.deg2rad(jp_command[3:])
    return jp_command


def np_to_se3(matrix) -> SE3:
    return SE3(np.asarray(matrix, dtype=float))


def pose_stamped_to_se3(message: PoseStamped) -> SE3:
    pose = SE3()
    pose.t = np.array([
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ], dtype=float)
    pose.R = q2r([
        message.pose.orientation.w,
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
    ])
    return pose


def se3_to_pose_stamped(pose: SE3, frame_id: str = '') -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = float(pose.t[0])
    message.pose.position.y = float(pose.t[1])
    message.pose.position.z = float(pose.t[2])
    quaternion = r2q(pose.R)
    message.pose.orientation.w = float(quaternion[0])
    message.pose.orientation.x = float(quaternion[1])
    message.pose.orientation.y = float(quaternion[2])
    message.pose.orientation.z = float(quaternion[3])
    return message


def joint_state_message(position: np.ndarray) -> JointState:
    message = JointState()
    message.position = np.asarray(position, dtype=float).tolist()
    return message
