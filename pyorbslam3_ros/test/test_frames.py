from geometry_msgs.msg import Pose, Transform
import numpy as np
from pyorbslam3_ros import frames
import pytest


def random_T(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return frames.make_T(frames.R_from_quat(*q), rng.normal(size=3))


def test_inverse():
    rng = np.random.default_rng(1)
    for _ in range(10):
        T = random_T(rng)
        np.testing.assert_allclose(T @ frames.inverse(T), np.eye(4), atol=1e-12)


def test_quaternion_roundtrip_including_near_pi():
    rng = np.random.default_rng(2)
    rots = [random_T(rng)[:3, :3] for _ in range(20)]
    rots += [np.diag([1.0, -1.0, -1.0]), np.diag([-1.0, 1.0, -1.0]), np.diag([-1.0, -1.0, 1.0])]
    for R in rots:
        q = frames.quat_from_R(R)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-12
        assert q[3] >= 0.0
        np.testing.assert_allclose(frames.R_from_quat(*q), R, atol=1e-9)


def test_pose_and_transform_msgs_roundtrip():
    T = random_T(np.random.default_rng(3))
    pose = frames.fill_pose(Pose(), T)
    tf = frames.fill_transform(Transform(), T)
    np.testing.assert_allclose(frames.transform_to_matrix(tf), T, atol=1e-9)
    assert pose.position.x == pytest.approx(T[0, 3])


def test_optical_axes_map_to_ros_body():
    R = frames.R_BODY_OPTICAL
    np.testing.assert_array_equal(R @ [0, 0, 1], [1, 0, 0])    # optical forward -> body x
    np.testing.assert_array_equal(R @ [1, 0, 0], [0, -1, 0])   # optical right -> body -y
    np.testing.assert_array_equal(R @ [0, 1, 0], [0, 0, -1])   # optical down -> body -z
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_map_odom_composition():
    rng = np.random.default_rng(4)
    T_map_base, T_odom_base = random_T(rng), random_T(rng)
    T_map_odom = T_map_base @ frames.inverse(T_odom_base)
    np.testing.assert_allclose(T_map_odom @ T_odom_base, T_map_base, atol=1e-12)


def test_non_inertial_map_starts_at_base():
    # map = initial base pose: at T_wc = I, T_map_base must be identity
    T_cam_base = random_T(np.random.default_rng(5))
    T_map_base = frames.inverse(T_cam_base) @ np.eye(4) @ T_cam_base
    np.testing.assert_allclose(T_map_base, np.eye(4), atol=1e-12)


def test_rotvec_and_body_twist():
    R = frames.R_from_quat(0, 0, np.sin(0.25), np.cos(0.25))   # 0.5 rad about z
    np.testing.assert_allclose(frames.rotvec_from_R(R), [0, 0, 0.5], atol=1e-12)
    np.testing.assert_allclose(frames.rotvec_from_R(np.diag([1.0, -1.0, -1.0])),
                               [np.pi, 0, 0], atol=1e-6)
    T0 = np.eye(4)
    T1 = frames.make_T(R, [1.0, 0.0, 0.0])
    v, w = frames.body_twist(T0, T1, 0.5)
    np.testing.assert_allclose(w, [0, 0, 1.0], atol=1e-12)
    np.testing.assert_allclose(R @ v, [2.0, 0, 0], atol=1e-12)   # world velocity 2 m/s along x


def test_transform_points():
    T = frames.make_T(np.eye(3), [1, 2, 3])
    out = frames.transform_points(T, np.zeros((5, 3), np.float32))
    assert out.dtype == np.float32
    np.testing.assert_allclose(out, np.tile([1, 2, 3], (5, 1)))
    assert frames.transform_points(T, np.empty((0, 3))).shape == (0, 3)
