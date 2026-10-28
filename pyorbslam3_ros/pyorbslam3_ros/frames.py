"""SE(3) helpers and frame conventions (pure numpy; msg helpers take msg objects).

Naming: T_a_b is the pose of frame b in frame a, i.e. it maps points from b
into a (p_a = T_a_b @ p_b). ORB-SLAM3's pose_wc is T_world_camera with the
camera in the optical convention (x right, y down, z forward).
"""

import numpy as np

# Optical (x right, y down, z forward) -> ROS body (x forward, y left, z up).
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])
T_BODY_OPTICAL = np.eye(4)
T_BODY_OPTICAL[:3, :3] = R_BODY_OPTICAL


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    return make_T(R.T, -R.T @ T[:3, 3])


def quat_from_R(R: np.ndarray):
    """Rotation matrix -> unit quaternion (x, y, z, w), Shepperd's method."""
    m = np.asarray(R, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x, y, z = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w, x, y, z = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s,
                      (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w, x, y, z = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s,
                      (m[1, 2] + m[2, 1]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w, x, y, z = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                      (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return tuple(float(v) for v in q)


def R_from_quat(x, y, z, w) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=np.float64)
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_from_R(R: np.ndarray) -> np.ndarray:
    """SO(3) log map: rotation matrix -> axis * angle."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    if angle < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2.0
    if np.pi - angle < 1e-6:
        # near pi: axis from the largest diagonal entry of (R + I) / 2
        B = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(B)))
        axis = B[:, i] / np.sqrt(B[i, i])
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def body_twist(T_prev: np.ndarray, T_curr: np.ndarray, dt: float):
    """Finite-difference (linear, angular) velocity of the current frame, in that frame."""
    R_curr = T_curr[:3, :3]
    v_world = (T_curr[:3, 3] - T_prev[:3, 3]) / dt
    w_body = rotvec_from_R(T_prev[:3, :3].T @ R_curr) / dt
    return R_curr.T @ v_world, w_body


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply T to an (N, 3) array; returns float32 (N, 3)."""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    return (pts @ T[:3, :3].T + T[:3, 3]).astype(np.float32)


# -- ROS message helpers -------------------------------------------------------

def fill_pose(pose, T: np.ndarray):
    """Write T into a geometry_msgs/Pose."""
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quat_from_R(T[:3, :3])
    return pose


def fill_transform(tf, T: np.ndarray):
    """Write T into a geometry_msgs/Transform."""
    tf.translation.x, tf.translation.y, tf.translation.z = (float(v) for v in T[:3, 3])
    (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w) = quat_from_R(T[:3, :3])
    return tf


def transform_to_matrix(tf) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4."""
    r, t = tf.rotation, tf.translation
    return make_T(R_from_quat(r.x, r.y, r.z, r.w), [t.x, t.y, t.z])
