"""Shared pieces for the pyorbslam3_ros launch files."""

import os

from launch_ros.actions import Node

from .sensor_mode import parse_sensor


def share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('pyorbslam3_ros')


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def slam_node(sensor, settings_file, remaps, params_file='', overrides=None,
              namespace='', name='orbslam3', use_viewer=False, use_sim_time=False):
    """slam_node with only the input remaps the sensor uses."""
    mode = parse_sensor(sensor)
    params = []
    if params_file:
        params.append(params_file)
    params.append({
        'sensor': mode.name,
        'settings_file': settings_file,
        'use_viewer': _as_bool(use_viewer),
        'use_sim_time': _as_bool(use_sim_time),
        **(overrides or {}),
    })
    remappings = [(topic, remaps[topic]) for topic in mode.input_topics
                  if remaps.get(topic) and remaps[topic] != topic]
    return Node(
        package='pyorbslam3_ros',
        executable='slam_node',
        name=name,
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=params,
        remappings=remappings,
    )


def rviz_node(use_sim_time=False):
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share_dir(), 'config', 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': _as_bool(use_sim_time)}],
        output='log',
    )


def static_tf_node(parent, child, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), name=None):
    """tf2_ros static_transform_publisher with the long-form flags (Humble and Jazzy)."""
    args = ['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', parent, '--child-frame-id', child]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=name or f'static_tf_{child}', arguments=args, output='log')
