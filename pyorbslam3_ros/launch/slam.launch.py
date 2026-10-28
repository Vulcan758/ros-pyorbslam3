"""Generic ORB-SLAM3 launch.

    ros2 launch pyorbslam3_ros slam.launch.py sensor:=stereo_imu \
        settings_file:=/path/to/orbslam3.yaml \
        left_image:=/cam/left/image_raw right_image:=/cam/right/image_raw imu:=/cam/imu

Only the remaps that the chosen sensor uses are applied. `settings_file` may
also be the name of a bundled settings file (e.g. tumvi_stereo_imu.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from pyorbslam3_ros.launch_helpers import rviz_node, slam_node

ARGS = [
    ('sensor', 'stereo_imu', 'mono|mono_imu|stereo|stereo_imu|rgbd|rgbd_imu'),
    ('settings_file', '', 'ORB-SLAM3 settings YAML (path or bundled file name)'),
    ('params_file', '', 'optional ROS parameter file for the slam node'),
    ('namespace', '', 'node namespace'),
    ('node_name', 'orbslam3', 'node name'),
    ('use_viewer', 'false', 'Pangolin viewer'),
    ('rviz', 'false', 'start RViz with the bundled config'),
    ('use_sim_time', 'false', 'use /clock (bag playback)'),
    ('base_frame', 'base_link', 'robot frame the pose is reported for'),
    ('tf_mode', 'map_odom', 'map_odom | map_base'),
    ('image', 'image', 'mono input topic'),
    ('left_image', 'left/image', 'stereo left input topic'),
    ('right_image', 'right/image', 'stereo right input topic'),
    ('rgb_image', 'rgb/image', 'RGB-D colour input topic'),
    ('depth_image', 'depth/image', 'RGB-D depth input topic'),
    ('imu', 'imu', 'IMU input topic'),
]


def _setup(context):
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    remaps = {
        'image': cfg['image'],
        'left/image': cfg['left_image'],
        'right/image': cfg['right_image'],
        'rgb/image': cfg['rgb_image'],
        'depth/image': cfg['depth_image'],
        'imu': cfg['imu'],
    }
    overrides = {
        'base_frame': cfg['base_frame'],
        'tf_mode': cfg['tf_mode'],
    }
    actions = [slam_node(cfg['sensor'], cfg['settings_file'], remaps,
                         params_file=cfg['params_file'], overrides=overrides,
                         namespace=cfg['namespace'], name=cfg['node_name'],
                         use_viewer=cfg['use_viewer'], use_sim_time=cfg['use_sim_time'])]
    if cfg['rviz'].lower() == 'true':
        actions.append(rviz_node(cfg['use_sim_time']))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in ARGS]
        + [OpaqueFunction(function=_setup)])
