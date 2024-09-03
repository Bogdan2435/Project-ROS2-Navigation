from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='project_path_planning',
            executable='spots_recorder',
            name='spots_recorder',
            output='screen'),
    ])