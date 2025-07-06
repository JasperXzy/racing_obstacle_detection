# racing_obstacle_detection/launch/obstacle_detection.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racing_obstacle_detection',
            executable='racing_obstacle_detection',
            name='racing_obstacle_detection',
            output='screen',
            parameters=[
            ],
            remappings=[
            ]
        )
    ])