import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("livox_ros_driver2")
    config_path = os.path.join(package_share, "config", "MID360_config.json")
    rviz_config_path = os.path.join(
        package_share, "config", "display_point_cloud_ROS2.rviz"
    )
    rviz_enabled = LaunchConfiguration("rviz")
    pointcloud_format = LaunchConfiguration("format")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 with the Livox point cloud configuration.",
            ),
            DeclareLaunchArgument(
                "format",
                default_value="0",
                description=(
                    "Point cloud format: 0=PointCloud2, "
                    "1=Livox CustomMsg, 2=PCL PointXYZI."
                ),
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_driver_node",
                output="screen",
                parameters=[
                    {
                        "xfer_format": ParameterValue(
                            pointcloud_format, value_type=int
                        ),
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "user_config_path": config_path,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="livox_rviz",
                output="screen",
                arguments=["--display-config", rviz_config_path],
                condition=IfCondition(rviz_enabled),
            ),
        ]
    )
