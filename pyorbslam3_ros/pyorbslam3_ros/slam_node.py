"""ORB-SLAM3 ROS 2 node built on pyorbslam3.

Inputs are fixed relative topic names, selected by the `sensor` parameter and
remapped at launch:

    mono:   image                    (+ imu for mono_imu)
    stereo: left/image, right/image  (+ imu for stereo_imu)
    rgbd:   rgb/image, depth/image   (+ imu for rgbd_imu)

Outputs (~/odometry, ~/pose, ~/path, ~/tracked_points, ~/tracking_status, TF)
are expressed for `base_frame` in `map_frame` following REP-105; see specs.md
section 5 for the frame math.
"""

import importlib.metadata
import os
import signal
import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from geometry_msgs.msg import PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry, Path
import numpy as np
from pyorbslam3_ros_msgs.msg import TrackingStatus
from pyorbslam3_ros_msgs.srv import SaveTrajectory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from . import frames
from .image_conversion import to_depth, to_gray, UnsupportedEncoding
from .imu_buffer import ImuBuffer
from .sensor_mode import parse_sensor, SensorMode
from .settings import (check_depth_encoding, check_settings, depth_map_factor,
                       load_settings, SettingsParseError)

TRAJECTORY_FORMATS = ('tum', 'euroc', 'kitti')
_OK_STATES = ('OK', 'OK_KLT')
# entered when ORB-SLAM3 starts a map from scratch (startup, reset, after LOST)
_NEW_MAP_STATES = ('NO_IMAGES_YET', 'NOT_INITIALIZED')


def _stamp_to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _ns_to_stamp(t_ns: int):
    return Time(nanoseconds=t_ns).to_msg()


class StartupError(RuntimeError):
    pass


def validate_save_request(fmt: str, keyframes_only: bool, sensor: str) -> Optional[str]:
    """Reason a trajectory save request must be rejected, or None."""
    if fmt not in TRAJECTORY_FORMATS:
        return f"format must be one of {', '.join(TRAJECTORY_FORMATS)}; got {fmt!r}"
    if fmt == 'kitti' and sensor == 'mono':
        return 'ORB-SLAM3 cannot save KITTI format for the monocular sensor'
    if fmt == 'kitti' and keyframes_only:
        return 'keyframes_only is not available for KITTI format'
    return None


class SlamNode(Node):
    def __init__(self, **kwargs):
        super().__init__('orbslam3', **kwargs)
        self._declare_parameters()
        p = self._param

        self.mode: SensorMode = self._check(lambda: parse_sensor(p('sensor')))
        self.map_frame = p('map_frame')
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.camera_frame_override = p('camera_frame')
        self.publish_tf = p('publish_tf')
        self.tf_mode = p('tf_mode')
        if self.tf_mode not in ('map_odom', 'map_base'):
            raise StartupError(f'tf_mode must be map_odom or map_base; got {self.tf_mode!r}')
        self.tf_timeout = Duration(seconds=p('tf_timeout'))
        self.image_time_offset_ns = int(round(p('image_time_offset') * 1e9))
        self.imu_wait_timeout = p('imu_wait_timeout')
        self.publish_path = p('publish_path')
        self.path_max_length = p('path_max_length')
        self.path_publish_period = p('path_publish_period')
        self.publish_points = p('publish_tracked_points')
        self.points_every_n = max(1, p('tracked_points_every_n'))
        self.publish_before_imu_init = p('publish_before_imu_init')
        self.pose_cov = self._covariance(p('pose_covariance_diagonal'), 'pose_covariance_diagonal')
        self.twist_cov = self._covariance(p('twist_covariance_diagonal'),
                                          'twist_covariance_diagonal')
        self.twist_max_dt = p('twist_max_dt')
        self.shutdown_traj_path = p('shutdown_trajectory_path')
        self.shutdown_traj_format = p('shutdown_trajectory_format')
        if self.shutdown_traj_path:
            err = self._validate_save_request(self.shutdown_traj_format, False)
            if err:
                raise StartupError(f'shutdown_trajectory_*: {err}')

        settings_file = self._resolve_settings(p('settings_file'))
        self._depth_factor = self._run_settings_checks(settings_file)
        vocab = p('vocabulary_file') or self._default_vocabulary()

        # -- state --------------------------------------------------------
        self._slam_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self.imu = ImuBuffer(p('imu_buffer_duration'))
        self.t_prev: Optional[float] = None
        self.frames_total = self.frames_ok = self.frames_skipped = 0
        self.last_state_name = None
        self.last_num_points = 0
        self.last_frame_wall: Optional[float] = None
        self.imu_init_seen = False
        self.localization_mode = False
        self.T_cam_base: Optional[np.ndarray] = None
        self.prev_pose = None          # (t, T_map_base)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self._path_last_pub = 0.0
        self._depth_checked = False
        self._fps_window = (time.monotonic(), 0)
        self._fps = 0.0
        self._shut_down = False

        # -- SLAM system: construct before subscribing (vocabulary load blocks) --
        import pyorbslam3
        self.get_logger().info(
            f'Starting ORB-SLAM3: sensor={self.mode.name} settings={settings_file}')
        self.get_logger().info(f'Loading vocabulary {vocab} ...')
        self.slam = pyorbslam3.Slam(vocab_file=vocab, settings_file=settings_file,
                                    sensor=self.mode.pyorbslam3_sensor(),
                                    use_viewer=p('use_viewer'))
        if p('localization_mode'):
            self.slam.activate_localization_mode()
            self.localization_mode = True

        # -- callback groups (see specs.md 4.7) ---------------------------
        self.slam_group = MutuallyExclusiveCallbackGroup()
        self.imu_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # -- outputs --------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '~/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.path_pub = self.create_publisher(Path, '~/path', 1) if self.publish_path else None
        self.points_pub = (self.create_publisher(PointCloud2, '~/tracked_points', 2)
                           if self.publish_points else None)
        self.status_pub = self.create_publisher(TrackingStatus, '~/tracking_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # status reflects processing, so it runs on wall time even under use_sim_time
        self.diag = diagnostic_updater.Updater(self, period=1.0)
        self.diag.setHardwareID('orbslam3')
        self.diag.add('tracking', self._diagnose)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.destroy_timer(self.diag.timer)
        self.diag.timer = self.create_timer(1.0, self.diag.update,
                                            callback_group=self.timer_group, clock=steady)
        period = p('status_log_period')
        if period > 0:
            self.create_timer(period, self._log_status, callback_group=self.timer_group,
                              clock=steady)

        # -- services -------------------------------------------------------
        g = self.slam_group
        self.create_service(Trigger, '~/reset', self._srv_reset, callback_group=g)
        self.create_service(Trigger, '~/reset_active_map', self._srv_reset_active_map,
                            callback_group=g)
        self.create_service(SetBool, '~/set_localization_mode', self._srv_localization,
                            callback_group=g)
        self.create_service(SaveTrajectory, '~/save_trajectory', self._srv_save, callback_group=g)

        # -- inputs ---------------------------------------------------------
        # best effort matches any live driver; reliable is needed when frames and IMU
        # samples must not be dropped (dataset playback with a reliable publisher)
        reliability = (ReliabilityPolicy.RELIABLE if p('reliable_qos')
                       else qos_profile_sensor_data.reliability)
        image_qos = QoSProfile(depth=max(1, p('image_queue_size')), reliability=reliability,
                               history=HistoryPolicy.KEEP_LAST)
        if self.mode.camera == 'mono':
            self.create_subscription(Image, 'image', self._mono_cb, image_qos, callback_group=g)
        else:
            a, b = self.mode.image_topics
            subs = [Subscriber(self, Image, a, qos_profile=image_qos, callback_group=g),
                    Subscriber(self, Image, b, qos_profile=image_qos, callback_group=g)]
            self._sync = ApproximateTimeSynchronizer(subs, p('sync_queue_size'), p('sync_slop'))
            self._sync.registerCallback(
                self._stereo_cb if self.mode.camera == 'stereo' else self._rgbd_cb)
        if self.mode.inertial:
            imu_qos = QoSProfile(depth=1000, reliability=reliability,
                                 history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Imu, 'imu', self._imu_cb, imu_qos,
                                     callback_group=self.imu_group)

        topics = [self.resolve_topic_name(t) for t in self.mode.input_topics]
        self.get_logger().info(
            f"ready: sensor={self.mode.name} inputs=[{', '.join(topics)}] "
            f"tf={self.tf_mode if self.publish_tf else 'off'} base_frame={self.base_frame}")
        if self.mode.inertial and not self.publish_before_imu_init:
            self.get_logger().info(
                'poses are published once the IMU is initialized; move the camera '
                '(rotation + acceleration) to initialize it')

    # -- parameters ------------------------------------------------------------

    def _declare_parameters(self):
        d = self.declare_parameter
        d('sensor', '')
        d('settings_file', '')
        d('vocabulary_file', '')
        d('use_viewer', False)
        d('localization_mode', False)
        d('map_frame', 'map')
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        d('camera_frame', '')
        d('publish_tf', True)
        d('tf_mode', 'map_odom')
        d('tf_timeout', 0.1)
        d('image_time_offset', 0.0)
        d('image_queue_size', 2)
        d('reliable_qos', False)
        d('sync_queue_size', 10)
        d('sync_slop', 0.01)
        d('imu_wait_timeout', 0.15)
        d('imu_buffer_duration', 5.0)
        d('publish_path', True)
        d('path_max_length', 10000)
        d('path_publish_period', 0.2)
        d('publish_tracked_points', True)
        d('tracked_points_every_n', 1)
        d('publish_before_imu_init', False)
        d('pose_covariance_diagonal', [1e-4] * 6)
        d('twist_covariance_diagonal', [1e-2] * 6)
        d('twist_max_dt', 0.5)
        d('status_log_period', 5.0)
        d('shutdown_trajectory_path', '')
        d('shutdown_trajectory_format', 'euroc')

    def _param(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _check(fn):
        try:
            return fn()
        except ValueError as e:
            raise StartupError(str(e)) from None

    @staticmethod
    def _covariance(diag, name):
        if len(diag) != 6:
            raise StartupError(f'{name} must have 6 entries; got {len(diag)}')
        cov = np.zeros(36)
        cov[::7] = diag
        return cov.tolist()

    def _resolve_settings(self, path: str) -> str:
        if not path:
            raise StartupError('parameter settings_file is required')
        if not os.path.isabs(path) and not os.path.isfile(path):
            from ament_index_python.packages import get_package_share_directory
            candidate = os.path.join(get_package_share_directory('pyorbslam3_ros'),
                                     'config', 'orbslam3', path)
            if os.path.isfile(candidate):
                path = candidate
        if not os.path.isfile(path):
            raise StartupError(f'settings_file not found: {path}')
        return os.path.abspath(path)

    def _run_settings_checks(self, path: str) -> Optional[float]:
        try:
            settings = load_settings(path)
        except (SettingsParseError, OSError) as e:
            self.get_logger().warn(
                f'could not pre-check settings file ({e}); leaving it to ORB-SLAM3')
            return None
        errors = []
        try:
            bindings_version = importlib.metadata.version('pyorbslam3')
        except importlib.metadata.PackageNotFoundError:
            bindings_version = None  # source build: no package metadata
        for issue in check_settings(settings, self.mode, bindings_version):
            if issue.level == 'error':
                errors.append(issue.message)
            else:
                self.get_logger().warn(f'settings: {issue.message}')
        if errors:
            raise StartupError('settings: ' + ' | '.join(errors))
        return depth_map_factor(settings)

    def _default_vocabulary(self) -> str:
        import pyorbslam3
        self.get_logger().info(
            'vocabulary_file not set; using pyorbslam3.vocabulary_path() '
            '(downloads ORBvoc.txt on first use)')
        return pyorbslam3.vocabulary_path()

    # -- input callbacks ----------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        w, a = msg.angular_velocity, msg.linear_acceleration
        # same ns -> float conversion as frame stamps, so equal stamps compare equal
        self.imu.add((_stamp_to_ns(msg.header.stamp) * 1e-9, w.x, w.y, w.z, a.x, a.y, a.z))

    def _mono_cb(self, msg: Image):
        self._process(msg, lambda imu, t: self.slam.track_monocular(to_gray(msg), t, imu=imu))

    def _stereo_cb(self, left: Image, right: Image):
        self._process(left, lambda imu, t: self.slam.track_stereo(
            to_gray(left), to_gray(right), t, imu=imu))

    def _rgbd_cb(self, rgb: Image, depth: Image):
        if not self._depth_checked:
            self._depth_checked = True
            warning = check_depth_encoding(depth.encoding, self._depth_factor)
            if warning:
                self.get_logger().warn(f'settings: {warning}')
        self._process(rgb, lambda imu, t: self.slam.track_rgbd(
            to_gray(rgb), to_depth(depth), t, imu=imu))

    # -- per-frame pipeline (specs.md 4.8) -------------------------------------------

    def _process(self, msg: Image, track):
        # outputs carry the exact (offset) image stamp; ORB-SLAM3 gets float seconds
        t_ns = _stamp_to_ns(msg.header.stamp) + self.image_time_offset_ns
        t = t_ns * 1e-9
        with self._slam_lock:
            if self._shut_down or not rclpy.ok():
                return
            imu = self._imu_window(t)
            if imu is None:
                self._count_skip()
                return
            try:
                result = track(imu, t)
            except UnsupportedEncoding as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                self._count_skip()
                return
            except Exception as e:  # one bad frame must not kill the executor
                self.get_logger().error(f'tracking failed: {e}', throttle_duration_sec=2.0)
                self._count_skip()
                return
            self.t_prev = t
            try:
                self._after_track(result, t, t_ns, msg.header.frame_id)
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f'publishing failed: {e}', throttle_duration_sec=2.0)

    def _imu_window(self, t: float):
        """IMU rows for the frame at t, or None if the frame must be skipped."""
        if self.t_prev is not None and t <= self.t_prev:
            self.get_logger().error(
                f'frame time went backwards ({self.t_prev:.3f} -> {t:.3f}); dropping frame and '
                'flushing IMU. Replaying a bag with --loop is not supported; call ~/reset.',
                throttle_duration_sec=2.0)
            self.imu.clear()
            return None
        if not self.mode.inertial:
            return np.empty((0, 7))
        if not self.imu.wait_for(t, self.imu_wait_timeout):
            oldest, newest = self.imu.span()
            self.get_logger().warn(
                f'no IMU up to frame t={t:.3f} within {self.imu_wait_timeout}s; skipping frame '
                f'(buffer n={len(self.imu)} newest-t='
                f"{'n/a' if newest is None else f'{newest - t:+.3f}s'})",
                throttle_duration_sec=2.0)
            return None
        trimmed = self.imu.trimmed_until
        if self.t_prev is not None and trimmed is not None and trimmed > self.t_prev:
            _, newest = self.imu.span()
            self.get_logger().error(
                f'IMU samples for this frame were discarded: tracking lags the IMU stream by '
                f'{(newest or t) - t:.1f}s, more than imu_buffer_duration. Tracking is slower '
                'than the input rate; lower the rate (or use lockstep dataset playback) or '
                'raise imu_buffer_duration. Expect ORB-SLAM3 to reset the map.',
                throttle_duration_sec=5.0)
        rows = self.imu.take_window(self.t_prev, t)
        if rows.shape[0] == 0 and self.t_prev is not None:
            self.get_logger().warn(f'empty IMU window for frame t={t:.3f}; skipping frame',
                                   throttle_duration_sec=2.0)
            return None
        return rows

    def _count_skip(self):
        with self._stats_lock:
            self.frames_skipped += 1

    def _after_track(self, result, t: float, t_ns: int, header_frame: str):
        state_name = result.state.name
        prev_state = self.last_state_name
        with self._stats_lock:
            self.frames_total += 1
            if state_name in _OK_STATES:
                self.frames_ok += 1
            self.last_state_name = state_name
            self.last_num_points = result.num_tracked_points
            self.last_frame_wall = time.monotonic()
            start, n = self._fps_window
            self._fps_window = (start, n + 1)

        if state_name != prev_state:
            self.get_logger().info(f'tracking state -> {state_name}')
            if prev_state == 'LOST' or (state_name in _NEW_MAP_STATES
                                        and prev_state not in _NEW_MAP_STATES + (None,)):
                # ORB-SLAM3 starts a new map after LOST, and resets the active map when it
                # rejects an IMU initialization ("not enough motion"): new world frame
                self.get_logger().warn('new map started (tracking loss or map reset); poses '
                                       'are not continuous with the previous map, path cleared')
                self._clear_path()
                self.imu_init_seen = False
        if self.slam.map_changed():
            self.get_logger().info('big map change (loop closure / map merge); ~/path history '
                                   'is not corrected')

        imu_ready = self.slam.imu_initialized() if self.mode.inertial else False
        if self.mode.inertial and imu_ready and not self.imu_init_seen:
            self.imu_init_seen = True
            self.get_logger().info('IMU initialized: map is gravity-aligned and metric')
            if self.publish_before_imu_init:
                self.get_logger().info('map was rotated/rescaled at IMU init; path cleared')
                self._clear_path()

        self._publish_status(result, t_ns, imu_ready)

        if not result.pose_valid:
            return
        if self.mode.inertial and not imu_ready and not self.publish_before_imu_init:
            return
        T_cam_base = self._camera_extrinsic(header_frame)
        if T_cam_base is None:
            return
        T_wc = np.asarray(result.pose_wc, dtype=np.float64)
        T_map_world = np.eye(4) if self.mode.inertial else frames.inverse(T_cam_base)
        T_map_base = T_map_world @ T_wc @ T_cam_base
        self._publish_pose(T_map_base, t, t_ns)
        if self.points_pub is not None and self.frames_total % self.points_every_n == 0 \
                and self.points_pub.get_subscription_count() > 0:
            self._publish_points(T_map_world, t_ns)

    # -- outputs -------------------------------------------------------------

    def _camera_extrinsic(self, header_frame: str) -> Optional[np.ndarray]:
        """T_cam_base from TF, looked up once and cached (assumed static)."""
        if self.T_cam_base is not None:
            return self.T_cam_base
        cam = self.camera_frame_override or header_frame
        if not cam:
            self.get_logger().error(
                'image header has no frame_id and camera_frame is not set; cannot place '
                'base_frame', throttle_duration_sec=5.0)
            return None
        if cam == self.base_frame:
            self.T_cam_base = np.eye(4)
            return self.T_cam_base
        try:
            tf = self.tf_buffer.lookup_transform(cam, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'waiting for TF {self.base_frame} -> {cam} (camera extrinsic); no poses '
                f'published until it is available: {e}', throttle_duration_sec=5.0)
            return None
        self.T_cam_base = frames.transform_to_matrix(tf.transform)
        self.get_logger().info(f'camera extrinsic {self.base_frame} -> {cam} acquired')
        return self.T_cam_base

    def _publish_status(self, result, t_ns: int, imu_ready: bool):
        msg = TrackingStatus()
        msg.header.stamp = _ns_to_stamp(t_ns)
        msg.header.frame_id = self.map_frame
        msg.state = int(result.state)
        msg.pose_valid = bool(result.pose_valid)
        msg.imu_initialized = bool(imu_ready)
        msg.localization_mode = self.localization_mode
        msg.num_tracked_points = int(result.num_tracked_points)
        with self._stats_lock:
            msg.frames_total = self.frames_total
            msg.frames_ok = self.frames_ok
            msg.frames_skipped = self.frames_skipped
        self.status_pub.publish(msg)

    def _publish_pose(self, T_map_base: np.ndarray, t: float, t_ns: int):
        stamp = _ns_to_stamp(t_ns)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        frames.fill_pose(pose.pose, T_map_base)
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose.pose
        odom.pose.covariance = self.pose_cov
        if self.prev_pose is not None and 0.0 < t - self.prev_pose[0] <= self.twist_max_dt:
            v, w = frames.body_twist(self.prev_pose[1], T_map_base, t - self.prev_pose[0])
            tw = odom.twist.twist
            tw.linear.x, tw.linear.y, tw.linear.z = (float(x) for x in v)
            tw.angular.x, tw.angular.y, tw.angular.z = (float(x) for x in w)
            odom.twist.covariance = self.twist_cov
        else:
            odom.twist.covariance = [-1.0 if i % 7 == 0 else 0.0 for i in range(36)]
        self.odom_pub.publish(odom)
        self.prev_pose = (t, T_map_base)

        if self.path_pub is not None:
            self.path_msg.poses.append(pose)
            if self.path_max_length > 0 and len(self.path_msg.poses) > self.path_max_length:
                del self.path_msg.poses[: len(self.path_msg.poses) - self.path_max_length]
            now = time.monotonic()
            if now - self._path_last_pub >= self.path_publish_period:
                self._path_last_pub = now
                self.path_msg.header.stamp = stamp
                self.path_pub.publish(self.path_msg)

        if self.tf_broadcaster is not None:
            self._publish_tf(T_map_base, stamp)

    def _publish_tf(self, T_map_base: np.ndarray, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        if self.tf_mode == 'map_base':
            tf.child_frame_id = self.base_frame
            frames.fill_transform(tf.transform, T_map_base)
        else:
            try:
                odom_base = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time.from_msg(stamp), self.tf_timeout)
            except TransformException as e:
                self.get_logger().warn(
                    f'no TF {self.odom_frame} -> {self.base_frame} at frame time; skipping '
                    f'{self.map_frame} -> {self.odom_frame} ({e})', throttle_duration_sec=5.0)
                return
            T_odom_base = frames.transform_to_matrix(odom_base.transform)
            tf.child_frame_id = self.odom_frame
            frames.fill_transform(tf.transform, T_map_base @ frames.inverse(T_odom_base))
        self.tf_broadcaster.sendTransform(tf)

    def _publish_points(self, T_map_world: np.ndarray, t_ns: int):
        pts = frames.transform_points(T_map_world, self.slam.tracked_map_points())
        msg = PointCloud2()
        msg.header.stamp = _ns_to_stamp(t_ns)
        msg.header.frame_id = self.map_frame
        msg.height, msg.width = 1, int(pts.shape[0])
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.points_pub.publish(msg)

    def _clear_path(self):
        self.path_msg.poses = []
        self.prev_pose = None

    # -- services --------------------------------------------------------------

    def _reset_tracking_state(self):
        self.t_prev = None
        self.imu.clear()
        self.imu_init_seen = False
        self._clear_path()
        if self.path_pub is not None:
            self.path_pub.publish(self.path_msg)

    def _srv_reset(self, request, response):
        with self._slam_lock:
            self.slam.reset()
            self._reset_tracking_state()
            with self._stats_lock:
                self.frames_total = self.frames_ok = self.frames_skipped = 0
        self.get_logger().info('SLAM reset requested')
        response.success, response.message = True, 'reset requested'
        return response

    def _srv_reset_active_map(self, request, response):
        with self._slam_lock:
            self.slam.reset_active_map()
            self._reset_tracking_state()
        self.get_logger().info('active map reset requested')
        response.success, response.message = True, 'active map reset requested'
        return response

    def _srv_localization(self, request, response):
        with self._slam_lock:
            if request.data:
                self.slam.activate_localization_mode()
            else:
                self.slam.deactivate_localization_mode()
            self.localization_mode = bool(request.data)
        state = 'on' if request.data else 'off'
        self.get_logger().info(f'localization mode {state}')
        response.success, response.message = True, f'localization mode {state}'
        return response

    def _validate_save_request(self, fmt: str, keyframes_only: bool) -> Optional[str]:
        return validate_save_request(fmt, keyframes_only, self.mode.name)

    def _save(self, path: str, fmt: str, keyframes_only: bool) -> str:
        fn = {
            ('tum', False): self.slam.save_trajectory_tum,
            ('tum', True): self.slam.save_keyframe_trajectory_tum,
            ('euroc', False): self.slam.save_trajectory_euroc,
            ('euroc', True): self.slam.save_keyframe_trajectory_euroc,
            ('kitti', False): self.slam.save_trajectory_kitti,
        }[(fmt, keyframes_only)]
        fn(path)
        return path

    def _srv_save(self, request, response):
        fmt = request.format.strip().lower()
        err = self._validate_save_request(fmt, request.keyframes_only)
        path = os.path.abspath(os.path.expanduser(request.path)) if request.path else ''
        if not err and not path:
            err = 'path is empty'
        if not err and not os.access(os.path.dirname(path) or '.', os.W_OK):
            err = f'directory not writable: {os.path.dirname(path)}'
        if err:
            response.success, response.message = False, err
            return response
        with self._slam_lock:
            if self.frames_total == 0:
                response.success, response.message = False, 'no frames tracked yet'
                return response
            try:
                self._save(path, fmt, request.keyframes_only)
            except Exception as e:
                response.success, response.message = False, f'save failed: {e}'
                return response
        self.get_logger().info(f'trajectory saved ({fmt}) to {path}')
        response.success, response.message = True, path
        return response

    # -- status ----------------------------------------------------------------

    def _stats(self):
        with self._stats_lock:
            now = time.monotonic()
            start, n = self._fps_window
            if now - start >= 1.0:
                self._fps = n / (now - start)
                self._fps_window = (now, 0)
            age = None if self.last_frame_wall is None else now - self.last_frame_wall
            return {'state': self.last_state_name, 'total': self.frames_total,
                    'ok': self.frames_ok, 'skipped': self.frames_skipped,
                    'points': self.last_num_points, 'fps': self._fps, 'age': age}

    def _diagnose(self, stat):
        s = self._stats()
        state, age = s['state'], s['age']
        waiting_imu = self.mode.inertial and not self.imu_init_seen
        if state is None:
            stat.summary(DiagnosticStatus.WARN, 'no frames received yet')
        elif state == 'LOST' or (age is not None and age > 2.0):
            stat.summary(DiagnosticStatus.ERROR,
                         'tracking lost' if state == 'LOST' else f'no frames for {age:.1f}s')
        elif state not in _OK_STATES or waiting_imu:
            stat.summary(DiagnosticStatus.WARN,
                         'waiting for IMU initialization' if state in _OK_STATES else state)
        else:
            stat.summary(DiagnosticStatus.OK, 'tracking')
        stat.add('sensor', self.mode.name)
        stat.add('state', str(state))
        stat.add('fps', f"{s['fps']:.1f}")
        stat.add('frames ok/total', f"{s['ok']}/{s['total']}")
        stat.add('frames skipped', str(s['skipped']))
        stat.add('tracked points', str(s['points']))
        stat.add('imu initialized', str(self.imu_init_seen))
        stat.add('localization mode', str(self.localization_mode))
        stat.add('last frame age [s]', 'n/a' if age is None else f'{age:.2f}')
        return stat

    def _log_status(self):
        s = self._stats()
        if s['state'] is None:
            return
        self.get_logger().info(
            f"[status] state={s['state']} fps={s['fps']:.1f} ok={s['ok']}/{s['total']} "
            f"skipped={s['skipped']} tracked_pts={s['points']}"
            + (f' imu_init={self.imu_init_seen}' if self.mode.inertial else ''))

    # -- shutdown --------------------------------------------------------------

    def shutdown_slam(self):
        with self._slam_lock:
            if self._shut_down:
                return
            self._shut_down = True
            s = self._stats()
            self.get_logger().info(
                f"shutting down ORB-SLAM3 ({s['ok']}/{s['total']} frames OK, "
                f"{s['skipped']} skipped)")
            try:
                self.slam.shutdown()
                if self.shutdown_traj_path and self.frames_total > 0:
                    path = os.path.abspath(os.path.expanduser(self.shutdown_traj_path))
                    fmt = self.shutdown_traj_format
                    self._save(path, fmt, False)
                    if fmt != 'kitti':
                        self._save(path + '.kf', fmt, True)
                    self.get_logger().info(f'trajectory saved ({fmt}) to {path}')
            except Exception as e:
                self.get_logger().error(f'shutdown/save failed: {e}')

    def destroy_node(self):
        self.shutdown_slam()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SlamNode()
    except StartupError as e:
        rclpy.logging.get_logger('orbslam3').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under `ros2 launch` delivers SIGINT twice (terminal + launch);
        # don't let the second one abort the trajectory save
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
