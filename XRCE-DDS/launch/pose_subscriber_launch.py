# pose_subscriber_pkg/launch/pose_subscriber_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_subscriber_pkg',
            executable='pose_subscriber',
            name='pose_subscriber',
            output='screen',
        ),
    ])
