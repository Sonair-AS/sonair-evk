import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# ros2 launch sonair_evk sonair_evk.launch.py

def generate_launch_description():
    foxglove_dir = os.path.join(
        get_package_share_directory("foxglove_bridge"), "launch"
    )
    return LaunchDescription(
        [
            # Foxglove bridge
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(foxglove_dir, "foxglove_bridge_launch.xml")
                )
            ),
            # Static transform
            # Rotation of frames from robot coordinate system to adar coordinate system
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "adar_link",
                ],
                output="screen",
            ),
            # Static transform
            # Rotation of frames from robot coordinate system to adar coordinate system
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "1.5708",
                    "0",
                    "1.5708",
                    "adar_link",
                    "adar",
                ],
                output="screen",
            ),
            # Point cloud publisher
            Node(
                package="sonair_evk",
                executable="pointcloud",
                emulate_tty=True,
                output="screen",
            ),
        ]
    )
