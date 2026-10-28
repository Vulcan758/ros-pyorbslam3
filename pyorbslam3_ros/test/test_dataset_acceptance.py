"""End-to-end acceptance on TUM-VI room1 (specs.md 9.2).

Skipped unless PYORBSLAM3_ROS_TUMVI points at dataset-room1_512_16. Modes are
taken from PYORBSLAM3_ROS_MODES (comma-separated, default stereo_imu). Each
mode plays the whole sequence in lockstep (about 10 min on a laptop), records
~/odometry and ~/tracking_status, exercises the services, stops the launch
with SIGINT and checks the shutdown trajectory, then scores both against mocap
with a sim(3) alignment.

The two scores answer different questions. The shutdown trajectory is ORB-SLAM3's
final, BA-refined estimate: it must match running pyorbslam3 directly, which
shows the node feeds ORB-SLAM3 the same data. ~/odometry is the causal online
estimate a robot consumes; it is necessarily worse (direct pyorbslam3 online
poses on room1 stereo_imu: 3.7 cm vs 0.78 cm final).
"""

import json
import os
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
import pytest

DATASET = os.environ.get('PYORBSLAM3_ROS_TUMVI', '')
MODES = [m for m in os.environ.get('PYORBSLAM3_ROS_MODES', 'stereo_imu').split(',') if m]
RESULTS = os.environ.get('PYORBSLAM3_ROS_RESULTS', '')

# mode: (min % frames OK, max final ATE [m], max odometry ATE [m], scale range or None,
#        check gravity)
CRITERIA = {
    'stereo_imu': (95.0, 0.02, 0.03, (0.98, 1.02), True),
    'mono_imu': (90.0, 0.03, 0.03, (0.97, 1.03), True),
    'stereo': (95.0, 0.15, 0.05, None, False),
}
# Inertial modes: odometry is asserted from this long after IMU init. ORB-SLAM3 refines
# the first IMU initialization (inertial BA after ~5 s and ~15 s of motion); before
# that a mono_imu scale estimate can be several % off (room1: 50 cm errors in 1 of 4
# runs, 1.25 cm once settled). The full-window ATE is still reported.
ODOM_SETTLE_NS = 20_000_000_000

pytestmark = pytest.mark.skipif(
    not os.path.isdir(os.path.join(DATASET, 'mav0')),
    reason='set PYORBSLAM3_ROS_TUMVI to a TUM-VI dataset-room1_512_16 folder')


def umeyama(est, gt):
    """Sim(3) s, R, t minimising |gt - (s R est + t)|."""
    mu_e, mu_g = est.mean(0), gt.mean(0)
    e, g = est - mu_e, gt - mu_g
    U, D, Vt = np.linalg.svd(g.T @ e / len(est))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / e.var(0).sum()
    return s, R, mu_g - s * R @ mu_e


def load_mocap(root):
    data = np.loadtxt(os.path.join(root, 'mav0', 'mocap0', 'data.csv'), delimiter=',')
    return data[:, 0].astype(np.int64), data[:, 1:4]


def load_trajectory(path):
    """Stamps [ns] and positions of a saved euroc/tum trajectory (stamp column in ns)."""
    rows = [line.split() for line in open(path) if line.strip() and not line.startswith('#')]
    # parse the stamp as text: float64 would round 19-digit ns stamps by ~100 ns
    t = np.array([int(r[0].split('.')[0]) for r in rows], dtype=np.int64)
    return t, np.array([[float(v) for v in r[1:4]] for r in rows])


def associate(t_est, t_gt, max_dt_ns=5_000_000):
    idx = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    left, right = t_gt[idx - 1], t_gt[idx]
    idx = np.where(np.abs(t_est - left) <= np.abs(right - t_est), idx - 1, idx)
    ok = np.abs(t_gt[idx] - t_est) <= max_dt_ns
    return np.nonzero(ok)[0], idx[ok]


def score(t_est, p_est, t_gt, p_gt):
    """Sim(3)-aligned ATE [m], scale and map-z vs gravity angle [deg]."""
    i_est, i_gt = associate(t_est, t_gt)
    s, R, t = umeyama(p_est[i_est], p_gt[i_gt])
    err = p_gt[i_gt] - (s * (p_est[i_est] @ R.T) + t)
    ate = float(np.sqrt((err ** 2).sum(1).mean()))
    return ate, float(s), float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))), len(i_est)


class Recorder:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from pyorbslam3_ros_msgs.msg import TrackingStatus
        from rclpy.executors import SingleThreadedExecutor
        from tf2_ros import Buffer, TransformListener

        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('acceptance_recorder', context=self.context)
        self.odom = []
        self.status = None
        self.first_imu_init_ns = None
        self.node.create_subscription(Odometry, '/orbslam3/odometry', self._odom, 1000)
        self.node.create_subscription(TrackingStatus, '/orbslam3/tracking_status',
                                      self._status, 1000)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._stop = False
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        while not self._stop:
            self.executor.spin_once(timeout_sec=0.1)

    @staticmethod
    def _ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((self._ns(msg.header.stamp), p.x, p.y, p.z))

    def _status(self, msg):
        self.status = msg
        if msg.imu_initialized and self.first_imu_init_ns is None:
            self.first_imu_init_ns = self._ns(msg.header.stamp)

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.node.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f'{name} not available'
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        assert future.done(), f'{name} timed out'
        return future.result()

    def close(self):
        import rclpy
        self._stop = True
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.mark.parametrize('mode', MODES)
def test_tumvi_room1(mode):
    from pyorbslam3_ros_msgs.srv import SaveTrajectory
    from rclpy.time import Time
    from std_srvs.srv import SetBool

    min_ok, max_ate_final, max_ate_odom, scale_range, check_gravity = CRITERIA[mode]
    tmp = tempfile.mkdtemp(prefix=f'pyorbslam3_ros_{mode}_')
    shutdown_traj = os.path.join(tmp, 'shutdown_traj.txt')
    rec = Recorder()
    proc = subprocess.Popen(
        ['ros2', 'launch', 'pyorbslam3_ros', 'dataset.launch.py', 'dataset:=tumvi',
         f'dataset_path:={DATASET}', f'sensor:={mode}',
         f'shutdown_trajectory_path:={shutdown_traj}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True,
        env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    finished = threading.Event()
    died = threading.Event()
    log_lines = []

    def read_output():
        for line in proc.stdout:
            log_lines.append(line)
            if 'playback finished' in line:
                finished.set()
            elif 'process has died' in line:
                died.set()
                finished.set()

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    try:
        finished.wait(timeout=3600)
        tail = ''.join(log_lines[-40:])
        assert not died.is_set(), 'a node died:\n' + tail
        assert finished.is_set(), 'playback did not finish:\n' + tail
        time.sleep(2.0)

        # services
        req = SaveTrajectory.Request(path=os.path.join(tmp, 'svc_traj.txt'), format='tum')
        res = rec.call(SaveTrajectory, '/orbslam3/save_trajectory', req)
        assert res.success, res.message
        assert os.path.getsize(res.message) > 0
        bad = rec.call(SaveTrajectory, '/orbslam3/save_trajectory',
                       SaveTrajectory.Request(path=os.path.join(tmp, 'x.txt'), format='kitti',
                                              keyframes_only=True))
        assert not bad.success
        for on in (True, False):
            assert rec.call(SetBool, '/orbslam3/set_localization_mode',
                            SetBool.Request(data=on)).success

        # TF published as map -> base_link (tf_mode map_base in dataset.launch.py)
        assert rec.tf_buffer.can_transform('map', 'base_link', Time())
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        rec.close()

    assert os.path.getsize(shutdown_traj) > 0, 'shutdown trajectory not written'
    assert os.path.getsize(shutdown_traj + '.kf') > 0

    status = rec.status
    ok_pct = 100.0 * status.frames_ok / max(1, status.frames_total)
    # keep stamps as int64: a float64 column would round them by ~100 ns
    t_est = np.array([o[0] for o in rec.odom], dtype=np.int64)
    p_est = np.array([o[1:] for o in rec.odom], dtype=np.float64)
    if mode.endswith('_imu'):
        # no pose may be published before the IMU is initialized
        assert rec.first_imu_init_ns is not None
        assert t_est[0] >= rec.first_imu_init_ns

    log_path = os.path.join(tmp, 'launch.log')
    with open(log_path, 'w') as f:
        f.writelines(log_lines)
    log_text = ''.join(log_lines)
    np.savetxt(os.path.join(tmp, 'odometry.txt'), np.column_stack([t_est, p_est]),
               fmt=['%d', '%.6f', '%.6f', '%.6f'], header='stamp_ns x y z')

    t_gt, p_gt = load_mocap(DATASET)
    ate_all = score(t_est, p_est, t_gt, p_gt)[0]
    settled = t_est >= t_est[0] + (ODOM_SETTLE_NS if mode.endswith('_imu') else 0)
    ate, s, gravity_deg, pairs = score(t_est[settled], p_est[settled], t_gt, p_gt)
    t_final, p_final = load_trajectory(shutdown_traj)
    if mode.endswith('_imu'):
        # ORB-SLAM3 saves frames tracked before IMU init with the scale of the initial
        # (monocular) map: tens of cm off in mono_imu. Whether they are in the file at
        # all depends on early map resets, so score the same window as ~/odometry.
        keep = t_final >= rec.first_imu_init_ns
        t_final, p_final = t_final[keep], p_final[keep]
    ate_final, s_final, _, _ = score(t_final, p_final, t_gt, p_gt)
    summary = {'mode': mode, 'frames': status.frames_total, 'ok_pct': round(ok_pct, 1),
               'skipped': status.frames_skipped, 'poses': len(t_est), 'gt_pairs': pairs,
               'ate_final_cm': round(ate_final * 100, 2), 'scale_final': round(s_final, 4),
               'ate_cm': round(ate * 100, 2), 'ate_all_cm': round(ate_all * 100, 2),
               'scale': round(s, 4),
               'gravity_deg': round(gravity_deg, 2),
               # ORB-SLAM3 events that explain run-to-run accuracy differences
               'loops': log_text.count('*Loop detected'),
               'merges': log_text.count('*Merge detected'),
               'maps': log_text.count('Creation of new map with id'),
               'log': log_path}
    print('ACCEPTANCE', json.dumps(summary))
    if RESULTS:
        with open(RESULTS, 'a') as f:
            f.write(json.dumps(summary) + '\n')

    assert ok_pct >= min_ok, summary
    assert ate_final <= max_ate_final, summary
    assert ate <= max_ate_odom, summary
    if scale_range:
        assert scale_range[0] <= s_final <= scale_range[1], summary
        assert scale_range[0] <= s <= scale_range[1], summary
    if check_gravity:
        assert gravity_deg <= 2.0, summary
