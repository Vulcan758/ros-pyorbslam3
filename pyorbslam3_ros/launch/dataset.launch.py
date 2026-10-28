"""Play a EuRoC or TUM-VI folder through the SLAM node.

    ros2 launch pyorbslam3_ros dataset.launch.py dataset:=tumvi \
        dataset_path:=/data/dataset-room1_512_16 sensor:=stereo_imu rviz:=true

The player starts once the SLAM node has subscribed (after the vocabulary
loads). By default it runs in lockstep: each frame waits until the previous
one has been tracked, so results do not depend on machine speed. Use
lockstep:=false rate:=<factor> for real-time playback (ORB-SLAM3 must then
keep up, or frames are dropped).

base_link is the dataset's IMU/body frame in every mode, so poses are
directly comparable with the dataset ground truth.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pyorbslam3_ros.launch_helpers import rviz_node, share_dir, slam_node
from pyorbslam3_ros.sensor_mode import parse_sensor

DATASETS = ('tumvi', 'euroc')
ARGS = [
    ('dataset', 'tumvi', 'tumvi | euroc'),
    ('dataset_path', '', 'sequence folder (containing mav0/) or the mav0 folder itself'),
    ('sensor', 'stereo_imu', 'mono|mono_imu|stereo|stereo_imu'),
    ('settings_file', '', 'override the bundled <dataset>_<sensor>.yaml'),
    ('lockstep', 'true', 'wait for each frame to be tracked before sending the next'),
    ('rate', '1.0', 'playback speed factor when lockstep is false'),
    ('start_offset', '0.0', 'seconds to skip at the start'),
    ('duration', '0.0', 'seconds to play (0 = all)'),
    ('rviz', 'false', 'start RViz'),
    ('use_viewer', 'false', 'Pangolin viewer'),
    ('shutdown_trajectory_path', '', 'save the trajectory here on shutdown'),
]


def _setup(context):
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    if cfg['dataset'] not in DATASETS:
        raise RuntimeError(f"dataset must be one of {DATASETS}; got {cfg['dataset']!r}")
    mode = parse_sensor(cfg['sensor'])
    if mode.camera == 'rgbd':
        raise RuntimeError('EuRoC/TUM-VI have no depth; use mono/stereo sensors')
    if not cfg['dataset_path']:
        raise RuntimeError('dataset_path is required')

    settings_dir = os.path.join(share_dir(), 'config', 'orbslam3')
    settings = cfg['settings_file'] or os.path.join(
        settings_dir, f"{cfg['dataset']}_{mode.name}.yaml")
    # the inertial settings carry IMU.T_b_c1 and Stereo.T_c1_c2 for the static TF
    extrinsics = os.path.join(settings_dir, f"{cfg['dataset']}_stereo_imu.yaml")

    player = Node(
        package='pyorbslam3_ros',
        executable='dataset_player',
        name='dataset_player',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'dataset_path': cfg['dataset_path'],
            'settings_file': extrinsics,
            'lockstep': cfg['lockstep'].lower() == 'true',
            'rate': float(cfg['rate']),
            'start_offset': float(cfg['start_offset']),
            'duration': float(cfg['duration']),
            'publish_stereo': mode.camera == 'stereo',
            'publish_imu': mode.inertial,
        }],
    )
    remaps = {'image': '/cam0/image_raw', 'left/image': '/cam0/image_raw',
              'right/image': '/cam1/image_raw', 'imu': '/imu0'}
    slam = slam_node(mode.name, settings, remaps, use_sim_time=True,
                     use_viewer=cfg['use_viewer'],
                     overrides={
                         'tf_mode': 'map_base',
                         'reliable_qos': True,
                         'image_queue_size': 100,
                         'sync_queue_size': 100,
                         'imu_wait_timeout': 1.0,
                         'imu_buffer_duration': 30.0,
                         'shutdown_trajectory_path': cfg['shutdown_trajectory_path'],
                     })
    actions = [slam, player]
    if cfg['rviz'].lower() == 'true':
        actions.append(rviz_node(use_sim_time=True))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in ARGS]
        + [OpaqueFunction(function=_setup)])
