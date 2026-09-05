"""Launch the GPS/NTRIP bridge with a configurable YAML file."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context, package_share):
    config_file = LaunchConfiguration('config_file').perform(context)
    with open(config_file, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    parameters = document.get('gps_ntrip_node', {}).get('ros__parameters', {})
    ntrip_enabled = bool(parameters.get('ntrip_enabled', True))
    profile = str(parameters.get('ntrip_profile', 'ml')).strip().lower()

    parameter_files = [config_file]
    if ntrip_enabled:
        if profile not in ('ml', 'hk'):
            raise RuntimeError(f'Unsupported ntrip_profile: {profile!r}; expected ml or hk')
        parameter_files.append(
            os.path.join(package_share, 'config', f'gps_ntrip_{profile}.yaml')
        )

    return [Node(
        package='wtrtk_982dt_driver',
        executable='gps_ntrip_node',
        name='gps_ntrip_node',
        output='screen',
        parameters=parameter_files,
    )]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('wtrtk_982dt_driver')
    default_config = os.path.join(
        package_share,
        'config',
        'gps_ntrip.yaml',
    )
    config_argument = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='GPS/NTRIP node parameter YAML file',
    )
    return LaunchDescription([
        config_argument,
        OpaqueFunction(function=_launch_node, args=[package_share]),
    ])
