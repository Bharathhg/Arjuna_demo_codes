from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    urdf_file = os.path.expanduser('~/arjuna2_ws/src/arjuna_description/urdf/arjuna.urdf')
    world_file = os.path.expanduser('~/arjuna2_ws/src/arjuna_description/worlds/arjuna.world')
    map_file = os.path.expanduser('~/arjuna2_ws/src/arjuna/arjuna/maps/my_map.yaml')
    rviz_file = os.path.expanduser('~/arjuna2_ws/src/arjuna/arjuna/rviz/arjuna.rviz')

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # 🔹 RViz (loads config)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description   
        }],
        output='screen'
    )

    # 🔹 Spawn robot node variable
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arjuna_robot',
            '-topic', 'robot_description',   
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([

        # 🔹 Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
                'world:=' + world_file
            ],
            output='screen'
        ),

        # 🔹 Robot State Publisher (publishes URDF to /robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 🔹 Spawn robot in Gazebo
        TimerAction(
            period=3.0,
            actions=[spawn_entity_node]
        ),

        # 🔹 Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': True,
                'frame_id': 'map'
            }]
        ),

        # 🔹 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # 🔹 TEMP TF (Starts only after spawn_entity completes to ensure /clock is active)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        arguments=[
                            '--x', '0', '--y', '0', '--z', '0',
                            '--yaw', '0', '--pitch', '0', '--roll', '0',
                            '--frame-id', 'map',
                            '--child-frame-id', 'odom'
                        ],
                        parameters=[{'use_sim_time': True}]
                    )
                ]
            )
        ),

        # 🔹 RViz
        rviz_node,

        # 🔹 Auto-shutdown when RViz exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )

    ])