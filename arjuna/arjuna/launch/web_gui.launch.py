import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arjuna',
            executable='web_gui',
            name='web_gui_launcher',
            output='screen'
        )
    ])
