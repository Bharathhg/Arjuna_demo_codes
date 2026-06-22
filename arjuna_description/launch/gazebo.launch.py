from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():

    urdf_file = os.path.expanduser('~/arjuna2_ws/src/arjuna_description/urdf/arjuna.urdf')
    world_file = os.path.expanduser('~/arjuna2_ws/src/arjuna_description/worlds/arjuna.world')

    return LaunchDescription([

        # 🔹 Start Gazebo with custom world
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
                'world:=' + world_file
            ],
            output='screen'
        ),

        # 🔹 Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_file]),
                    value_type=str
                )
            }],
            output='screen'
        ),

        # 🔹 Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'arjuna_robot',
                '-file', urdf_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        ),
    ])