"""ORB-SLAM3 on an Intel RealSense D435i.

    ros2 launch pyorbslam3_ros d435i.launch.py sensor:=stereo_imu start_camera:=true

sensor:
  stereo_imu (default)  infra1 + infra2 + IMU, emitter off -- best for handheld
  stereo                infra1 + infra2,       emitter off
  rgbd_imu              colour + aligned depth + IMU, emitter on
  rgbd                  colour + aligned depth,       emitter on

With start_camera:=true the realsense2_camera driver is started with the
settings these modes require (they are why a stock rs_launch.py does not work):
  * unite_imu_method=2 -- without it there is no combined imu topic at all;
  * depth_module.emitter_enabled=0 for infrared modes -- the projected dot
    pattern moves with the camera and destroys feature matching;
  * align_depth.enable=true for RGB-D -- raw depth sits in the infra1 frame.
When running the driver yourself, use namespace/name camera/camera and the
same settings. Poses are reported for camera_link, whose TF the driver
publishes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pyorbslam3_ros.launch_helpers import rviz_node, slam_node
from pyorbslam3_ros.sensor_mode import parse_sensor

SENSORS = ('stereo_imu', 'stereo', 'rgbd_imu', 'rgbd')
ARGS = [
    ('sensor', 'stereo_imu', ' | '.join(SENSORS)),
    ('start_camera', 'false', 'also start realsense2_camera with the required settings'),
    ('settings_file', '', 'override the bundled d435i_<sensor>.yaml'),
    ('imu_rate', '200', 'gyro/accel Hz (200 or 400); must match IMU.Frequency in settings'),
    ('initial_reset', 'false', 'hardware-reset the camera on start'),
    ('base_frame', 'camera_link', 'frame the pose is reported for'),
    ('tf_mode', 'map_base', 'map_base (handheld) | map_odom (robot with odometry)'),
    ('rviz', 'false', 'start RViz'),
    ('use_viewer', 'false', 'Pangolin viewer'),
]

PREFIX = '/camera/camera'


def driver_params(sensor, imu_rate, initial_reset):
    mode = parse_sensor(sensor)
    infrared = mode.camera == 'stereo'
    params = {
        'enable_sync': True,
        'initial_reset': initial_reset,
        'depth_module.emitter_enabled': 0 if infrared else 1,
        'enable_infra1': infrared,
        'enable_infra2': infrared,
        'enable_color': not infrared,
        'enable_depth': not infrared,
        'align_depth.enable': not infrared,
        'enable_gyro': mode.inertial,
        'enable_accel': mode.inertial,
        'unite_imu_method': 2 if mode.inertial else 0,
    }
    if infrared:
        params['depth_module.infra_profile'] = '640x480x30'
    else:
        params['rgb_camera.color_profile'] = '640x480x30'
        params['depth_module.depth_profile'] = '640x480x30'
    if mode.inertial:
        params['gyro_fps'] = imu_rate
        params['accel_fps'] = imu_rate
    return params


def _setup(context):
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    sensor = cfg['sensor']
    if sensor not in SENSORS:
        raise RuntimeError(f'sensor must be one of {SENSORS}; got {sensor!r}')
    mode = parse_sensor(sensor)
    remaps = {
        'left/image': f'{PREFIX}/infra1/image_rect_raw',
        'right/image': f'{PREFIX}/infra2/image_rect_raw',
        'rgb/image': f'{PREFIX}/color/image_raw',
        'depth/image': f'{PREFIX}/aligned_depth_to_color/image_raw',
        'imu': f'{PREFIX}/imu',
    }
    overrides = {
        'base_frame': cfg['base_frame'],
        'tf_mode': cfg['tf_mode'],
        # infra pair is hardware-synchronised; colour/depth alignment can drift a frame
        'sync_slop': 0.005 if mode.camera == 'stereo' else 0.035,
    }
    actions = [slam_node(sensor, cfg['settings_file'] or f'd435i_{sensor}.yaml', remaps,
                         overrides=overrides, use_viewer=cfg['use_viewer'])]
    if cfg['start_camera'].lower() == 'true':
        actions.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[driver_params(sensor, int(cfg['imu_rate']),
                                      cfg['initial_reset'].lower() == 'true')],
        ))
    if cfg['rviz'].lower() == 'true':
        actions.append(rviz_node())
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in ARGS]
        + [OpaqueFunction(function=_setup)])
