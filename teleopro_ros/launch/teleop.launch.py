"""Start the teleopro node.

Usage:
    ros2 launch teleopro_ros teleop.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

OVERRIDABLE = [
    'input_type', 'input_base_frame', 'input_source_frame', 'twist_topic', 'twist_frame',
    'robot_base_frame', 'robot_source_frame', 'align_rpy', 'translation_scale',
    'rotation_scale', 'max_lin_vel', 'max_ang_vel', 'rate', 'pose_topic',
]


def parse(value):
    # Numeric parameters are doubles
    v = yaml.safe_load(value)
    if v is None:
        return ''
    if isinstance(v, list):
        return [float(x) for x in v]
    if isinstance(v, int) and not isinstance(v, bool):
        return float(v)
    return v


def launch_node(context):
    overrides = {}
    for name in OVERRIDABLE:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = parse(value)
    return [Node(
        package='teleopro_ros',
        executable='teleop_node',
        name='teleopro',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides],
    )]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('teleopro_ros'), 'config', 'teleop.yaml')
    args = [DeclareLaunchArgument('params_file', default_value=default_params,
                                  description='Parameter file for the teleopro node')]
    args += [DeclareLaunchArgument(name, default_value='',
                                   description='Overrides the yaml value')
             for name in OVERRIDABLE]
    return LaunchDescription(args + [OpaqueFunction(function=launch_node)])
