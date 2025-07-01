import PyKDL
import numpy as np
from spatialmath import SE3, SO3
from spatialmath.base import q2r
from geometry_msgs.msg import TransformStamped


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
