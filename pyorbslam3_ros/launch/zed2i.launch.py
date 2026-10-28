"""ORB-SLAM3 on a ZED 2i through the SDK-less driver from the orbulation package.

    ros2 launch pyorbslam3_ros zed2i.launch.py sensor:=stereo_imu start_camera:=true

The driver (orbulation's zed_stereo_ros) publishes /zed/left/image_raw,
/zed/right/image_raw and /zed/imu/data, and applies the calibrated -80 ms
V4L2 image time offset itself, so the SLAM node runs with no offset. It does
not publish TF, so this launch adds zed_camera_link -> zed_left_link (the
left optical frame) and reports poses for zed_camera_link.

Replaying a bag recorded before the driver applied the offset:

    ros2 launch pyorbslam3_ros zed2i.launch.py use_sim_time:=true image_time_offset:=-0.08
    ros2 bag play <bag> --clock
"""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pyorbslam3_ros.launch_helpers import rviz_node, slam_node, static_tf_node

SENSORS = ('stereo_imu', 'stereo')
ARGS = [
    ('sensor', 'stereo_imu', ' | '.join(SENSORS)),
    ('start_camera', 'false', "also start orbulation's zed_stereo_ros driver"),
    ('driver_params_file', '', "zed_stereo_ros params (default: orbulation's)"),
    ('settings_file', '', 'override the bundled zed2i_<sensor>.yaml'),
    ('tf_mode', 'map_base', 'map_base (handheld) | map_odom (robot with odometry)'),
    ('image_time_offset', '0.0',
     's added to image stamps; -0.08 for bags recorded without the driver offset'),
    ('use_sim_time', 'false', 'true when replaying a bag with --clock'),
    ('rviz', 'false', 'start RViz'),
    ('use_viewer', 'false', 'Pangolin viewer'),
]


def _setup(context):
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    sensor = cfg['sensor']
    if sensor not in SENSORS:
        raise RuntimeError(f'sensor must be one of {SENSORS}; got {sensor!r}')
    remaps = {'left/image': '/zed/left/image_raw', 'right/image': '/zed/right/image_raw',
              'imu': '/zed/imu/data'}
    actions = [
        slam_node(sensor, cfg['settings_file'] or f'zed2i_{sensor}.yaml', remaps,
                  use_viewer=cfg['use_viewer'], use_sim_time=cfg['use_sim_time'],
                  overrides={'base_frame': 'zed_camera_link', 'tf_mode': cfg['tf_mode'],
                             'sync_slop': 0.02,
                             'image_time_offset': float(cfg['image_time_offset'])}),
        # body (x forward, z up) -> left optical (z forward, y down)
        static_tf_node('zed_camera_link', 'zed_left_link',
                       rpy=(-math.pi / 2, 0.0, -math.pi / 2)),
    ]
    if cfg['start_camera'].lower() == 'true':
        try:
            from ament_index_python.packages import get_package_share_directory
            orbulation = get_package_share_directory('orbulation')
        except Exception:
            raise RuntimeError(
                'start_camera:=true needs the orbulation package (zed_stereo_ros driver) in '
                'the workspace; or start your own driver publishing /zed/left/image_raw, '
                '/zed/right/image_raw and /zed/imu/data (rad/s, m/s^2)') from None
        params = cfg['driver_params_file'] or os.path.join(
            orbulation, 'config', 'zed_stereo_params.yaml')
        actions.append(Node(package='orbulation', executable='zed_stereo_ros',
                            name='zed_stereo_ros', output='screen', parameters=[params]))
    if cfg['rviz'].lower() == 'true':
        actions.append(rviz_node())
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in ARGS]
        + [OpaqueFunction(function=_setup)])
