"""Play a EuRoC-layout dataset folder (EuRoC, TUM-VI) as ROS 2 topics.

Publishes cam0/image_raw, cam1/image_raw (mono8), imu0 (sensor_msgs/Imu) and
/clock, merged in timestamp order so the IMU covering a frame is always out
before the frame.

Timing is either real-time scaled by `rate`, or `lockstep`: each frame is
published only after the SLAM node reported a TrackingStatus for the previous
one, so playback runs exactly as fast as tracking and never drops frames.

Also publishes static TF base_frame -> cam0 (and -> cam1, imu0 when known).
The camera extrinsic comes from, in order: IMU.T_b_c1 in `settings_file`,
mav0/cam0/sensor.yaml T_BS, or a plain optical->body rotation.
"""

import csv
import os
import signal
import threading
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
from pyorbslam3_ros_msgs.msg import TrackingStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from tf2_ros import StaticTransformBroadcaster
import yaml

from . import frames
from .settings import load_settings, SettingsParseError


def _read_csv(path):
    with open(path, newline='') as f:
        return [row for row in csv.reader(f) if row and not row[0].startswith('#')]


def _read_T_BS(sensor_yaml):
    try:
        with open(sensor_yaml) as f:
            d = yaml.safe_load(f)
        T = np.asarray(d['T_BS']['data'], dtype=np.float64).reshape(4, 4)
        return T
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def load_png_gray(path) -> np.ndarray:
    from PIL import Image as PilImage
    with PilImage.open(path) as im:
        arr = np.asarray(im)
    if arr.ndim == 3:
        arr = np.asarray(PilImage.fromarray(arr).convert('L'))
    if arr.dtype != np.uint8:  # TUM-VI ships 16-bit PNGs
        arr = (arr.astype(np.uint32) >> 8).astype(np.uint8)
    return np.ascontiguousarray(arr)


class DatasetPlayer(Node):
    def __init__(self, **kwargs):
        super().__init__('dataset_player', **kwargs)
        d = self.declare_parameter
        d('dataset_path', '')
        d('settings_file', '')
        d('rate', 1.0)
        d('start_offset', 0.0)
        d('duration', 0.0)
        d('publish_imu', True)
        d('publish_stereo', True)
        d('publish_clock', True)
        d('base_frame', 'base_link')
        d('frame_id_cam0', 'cam0')
        d('frame_id_cam1', 'cam1')
        d('frame_id_imu', 'imu0')
        d('lockstep', False)
        d('status_topic', '/orbslam3/tracking_status')
        d('lockstep_timeout', 10.0)
        d('wait_for_subscribers', True)
        d('start_delay', 0.5)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        root = p('dataset_path')
        if os.path.isdir(os.path.join(root, 'mav0')):
            root = os.path.join(root, 'mav0')
        if not os.path.isfile(os.path.join(root, 'cam0', 'data.csv')):
            raise RuntimeError(f'not a EuRoC/TUM-VI mav0 folder: {root!r}')
        self.root = root
        self.rate = float(p('rate'))
        if self.rate <= 0.0:
            raise RuntimeError('rate must be > 0')
        self.frame_cam0 = p('frame_id_cam0')
        self.frame_cam1 = p('frame_id_cam1')
        self.frame_imu = p('frame_id_imu')
        self.lockstep = p('lockstep')
        self.lockstep_timeout = float(p('lockstep_timeout'))
        self.wait_for_subscribers = p('wait_for_subscribers')
        self.start_delay = float(p('start_delay'))
        self.publish_clock = p('publish_clock')

        self.events = self._build_timeline(
            p('publish_stereo'), p('publish_imu'), float(p('start_offset')), float(p('duration')))
        n_frames = sum(1 for e in self.events if e[1] == 'frame')
        self.get_logger().info(
            f'{root}: {n_frames} frames, {len(self.events) - n_frames} IMU samples, '
            f'{(self.events[-1][0] - self.events[0][0]) * 1e-9:.1f}s at rate {self.rate}')

        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_cam0 = self.create_publisher(Image, 'cam0/image_raw', sensor_qos)
        self.pub_cam1 = self.create_publisher(Image, 'cam1/image_raw', sensor_qos)
        self.pub_imu = self.create_publisher(Imu, 'imu0', QoSProfile(depth=1000))
        self.pub_clock = self.create_publisher(Clock, '/clock', 10) if self.publish_clock else None

        self._status_cond = threading.Condition()
        self._last_status_ns = -1
        if self.lockstep:
            self.create_subscription(TrackingStatus, p('status_topic'), self._status_cb, 10)

        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tf(p('base_frame'), p('settings_file'))

        self._stop = threading.Event()
        self.finished = threading.Event()
        self._thread = threading.Thread(target=self._play, daemon=True)
        self._thread.start()

    # -- dataset ------------------------------------------------------------

    def _build_timeline(self, stereo, imu, start_offset, duration):
        cam0_csv = os.path.join(self.root, 'cam0', 'data.csv')
        cam0 = {int(r[0]): r[1].strip() for r in _read_csv(cam0_csv)}
        frames_ = sorted(cam0)
        cam1 = {}
        if stereo:
            cam1_csv = os.path.join(self.root, 'cam1', 'data.csv')
            if not os.path.isfile(cam1_csv):
                raise RuntimeError('publish_stereo is true but the dataset has no cam1')
            cam1 = {int(r[0]): r[1].strip() for r in _read_csv(cam1_csv)}
            frames_ = [t for t in frames_ if t in cam1]
        t0 = frames_[0] + int(start_offset * 1e9)
        t1 = t0 + int(duration * 1e9) if duration > 0 else None
        events = [(t, 'frame', cam0[t], cam1.get(t)) for t in frames_
                  if t >= t0 and (t1 is None or t <= t1)]
        if imu:
            for r in _read_csv(os.path.join(self.root, 'imu0', 'data.csv')):
                t = int(r[0])
                # keep IMU from slightly before the first frame so its window is full
                if t >= t0 - int(0.1e9) and (t1 is None or t <= t1):
                    events.append((t, 'imu', tuple(float(x) for x in r[1:7]), None))
        # IMU before frames at equal stamps
        events.sort(key=lambda e: (e[0], 0 if e[1] == 'imu' else 1))
        return events

    def _publish_static_tf(self, base_frame, settings_file):
        T_b_c0, source = None, None
        T_c0_c1 = None
        if settings_file:
            try:
                s = load_settings(settings_file)
                if 'IMU.T_b_c1' in s:
                    T_b_c0, source = np.asarray(s['IMU.T_b_c1']), 'settings IMU.T_b_c1'
                if 'Stereo.T_c1_c2' in s:
                    T_c0_c1 = np.asarray(s['Stereo.T_c1_c2'])
            except (OSError, SettingsParseError) as e:
                self.get_logger().warn(f'could not read settings_file for extrinsics: {e}')
        if T_b_c0 is None:
            T_b_c0 = _read_T_BS(os.path.join(self.root, 'cam0', 'sensor.yaml'))
            source = 'cam0/sensor.yaml T_BS' if T_b_c0 is not None else None
            T_b_c1 = _read_T_BS(os.path.join(self.root, 'cam1', 'sensor.yaml'))
            if T_b_c0 is not None and T_b_c1 is not None and T_c0_c1 is None:
                T_c0_c1 = frames.inverse(T_b_c0) @ T_b_c1
        if T_b_c0 is None:
            T_b_c0, source = frames.T_BODY_OPTICAL, 'default optical->body rotation'
            self.get_logger().warn(
                f'no camera extrinsic found; {base_frame} -> {self.frame_cam0} is a pure '
                'optical->body rotation (base_frame = camera body frame)')
        else:
            # EuRoC/TUM-VI body frame is the IMU
            self.get_logger().info(f'{base_frame} -> {self.frame_cam0} from {source}')
        tfs = [self._tf(base_frame, self.frame_cam0, T_b_c0)]
        if T_c0_c1 is not None:
            tfs.append(self._tf(self.frame_cam0, self.frame_cam1, T_c0_c1))
        if source != 'default optical->body rotation':
            tfs.append(self._tf(base_frame, self.frame_imu, np.eye(4)))
        self._static_tf.sendTransform(tfs)

    def _tf(self, parent, child, T):
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        frames.fill_transform(tf.transform, np.asarray(T, dtype=np.float64))
        return tf

    # -- playback ---------------------------------------------------------------

    def _image_msg(self, path, t_ns, frame_id):
        arr = load_png_gray(os.path.join(path))
        msg = Image()
        msg.header.stamp = Time(nanoseconds=t_ns).to_msg()
        msg.header.frame_id = frame_id
        msg.height, msg.width = arr.shape
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = arr.shape[1]
        msg.data = arr.tobytes()
        return msg

    def _status_cb(self, msg):
        with self._status_cond:
            self._last_status_ns = Time.from_msg(msg.header.stamp).nanoseconds
            self._status_cond.notify_all()

    def _wait_tracked(self, t_ns) -> bool:
        with self._status_cond:
            return self._status_cond.wait_for(
                lambda: self._last_status_ns >= t_ns or self._stop.is_set(),
                timeout=self.lockstep_timeout)

    def _play(self):
        try:
            self._play_events()
        except Exception as e:  # context torn down mid-publish on Ctrl-C
            if rclpy.ok():
                self.get_logger().error(f'playback failed: {e}')

    def _play_events(self):
        if self.wait_for_subscribers:
            # the SLAM node subscribes only after its vocabulary has loaded
            self.get_logger().info('waiting for a subscriber on cam0/image_raw ...')
            while self.pub_cam0.get_subscription_count() == 0:
                if self._stop.wait(0.1):
                    return
        if self.start_delay > 0 and self._stop.wait(self.start_delay):
            return
        self.get_logger().info('playback started' + (' (lockstep)' if self.lockstep else ''))
        events = self.events
        t_data0 = events[0][0]
        t_wall0 = time.monotonic()
        timeouts = 0
        i = 0
        while i < len(events):
            if self._stop.is_set() or not rclpy.ok():
                return
            t_ns, kind = events[i][0], events[i][1]
            if not self.lockstep:
                delay = t_wall0 + (t_ns - t_data0) * 1e-9 / self.rate - time.monotonic()
                if delay > 0 and self._stop.wait(delay):
                    return
            self._publish_event(events[i])
            i += 1
            if self.lockstep and kind == 'frame':
                # the SLAM node waits for IMU at or past the frame time: send it first
                while i < len(events) and events[i][1] == 'imu':
                    self._publish_event(events[i])
                    i += 1
                    if events[i - 1][0] >= t_ns:
                        break
                if not self._wait_tracked(t_ns):
                    timeouts += 1
                    self.get_logger().warn(
                        f'no tracking status for frame {t_ns * 1e-9:.3f} within '
                        f'{self.lockstep_timeout}s (frame skipped by the SLAM node?)',
                        throttle_duration_sec=5.0)
        wall = time.monotonic() - t_wall0
        self.get_logger().info(
            f'playback finished ({wall:.1f}s wall'
            + (f', {timeouts} lockstep timeouts)' if self.lockstep else ')'))
        self.finished.set()

    def _publish_event(self, event):
        t_ns, kind, a, b = event
        stamp = Time(nanoseconds=t_ns).to_msg()
        if self.pub_clock is not None:
            self.pub_clock.publish(Clock(clock=stamp))
        if kind == 'imu':
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_imu
            msg.orientation_covariance[0] = -1.0
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
             msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = a
            self.pub_imu.publish(msg)
            return
        try:
            left = self._image_msg(os.path.join(self.root, 'cam0', 'data', a), t_ns,
                                   self.frame_cam0)
            right = (self._image_msg(os.path.join(self.root, 'cam1', 'data', b), t_ns,
                                     self.frame_cam1) if b is not None else None)
        except OSError as e:
            self.get_logger().error(f'failed to read image: {e}')
            return
        self.pub_cam0.publish(left)
        if right is not None:
            self.pub_cam1.publish(right)

    def destroy_node(self):
        self._stop.set()
        with self._status_cond:
            self._status_cond.notify_all()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DatasetPlayer()
    except RuntimeError as e:
        rclpy.logging.get_logger('dataset_player').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
