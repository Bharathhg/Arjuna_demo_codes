#!/usr/bin/env python3
"""
Arjuna Navigation with IMU Fusion
Wheel odometry + IMU → robot_localization → fused output for better navigation
CRITICAL: EKF_data_pub must have publish_tf parameter support
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_arjuna = get_package_share_directory('arjuna')
    pkg_arjuna_description = get_package_share_directory('arjuna_description')
    
    gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='True',
        description='Whether to start RViz'
    )
    
    publish_map_odom_arg = DeclareLaunchArgument(
        'publish_map_odom',
        default_value='True',
        description='Whether to publish static map->odom transform'
    )
    
    # Find the active workspace path dynamically (check /home/hacker or /root)
    ws_base = '/root/arjuna_ros2/arjuna2_ws'
    for path in ['/home/hacker/arjuna2_ws', '/root/arjuna_ros2/arjuna2_ws']:
        if os.path.exists(path):
            ws_base = path
            break

    map_yaml_file = os.path.join(ws_base, 'src/arjuna/arjuna/maps/my_map.yaml')
    
    # Fix permissions on map files
    maps_dir = os.path.join(ws_base, 'src/arjuna/arjuna/maps')
    if os.path.exists(maps_dir):
        os.system(f"chmod -R 755 {maps_dir}")
    if os.path.exists(map_yaml_file):
        os.system(f"chmod 644 {map_yaml_file}")
    pgm_file = os.path.join(os.path.dirname(map_yaml_file), "my_map.pgm")
    if os.path.exists(pgm_file):
        os.system(f"chmod 644 {pgm_file}")
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=map_yaml_file,
        description='Path to map file'
    )

    # Resolve URDF path dynamically
    urdf_file = os.path.join(pkg_arjuna_description, 'urdf', 'arjuna.urdf')
    if not os.path.exists(urdf_file):
        urdf_file = os.path.join(ws_base, 'src/arjuna_description/urdf/arjuna.urdf')
        
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # ========== TRANSFORM LAYER (Static TFs) ==========
    
    # Map → odom (for localization)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        condition=IfCondition(LaunchConfiguration('publish_map_odom'))
    )
    
    # Note: Handled by robot_state_publisher and URDF definition
    # base_link_broadcaster = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_broadcaster',
    #     arguments=['0', '0', '0.09', '0', '0', '0', 'base_footprint', 'base_link']
    # )
    
    # base_link_to_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_laser',
    #     arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'laser']
    # )
    
    # imu_broadcaster = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='imu_broadcaster',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    # )
    
    # ========== MAP SERVER ==========
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False,
            'autostart': True
        }]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # ========== SENSOR LAYER ==========
    
    # Motor encoder publisher
    arjuna_ticks_pub = Node(
        package='motor_ops',
        executable='ticks_pub',
        name='Arjuna_Ticks_Pub'
    )
    
    # Wheel odometry (NO TF - robot_localization will publish TF)
    ekf_data_pub = Node(
        package='odometry',
        executable='ekf_data_pub',
        name='ekf_data_pub',
        parameters=[{'publish_tf': False}],  # CRITICAL: Disable TF
        remappings=[
            ('/odom', '/wheel/odom'),
            ('/odom_data_quat', '/wheel/odom_data_quat'),
            ('/odom_data_euler', '/wheel/odom_data_euler')
        ]
    )
    
    # IMU
    imu_node = Node(
        package='imu_bno055',
        executable='bno055_i2c_node',
        name='imu_node',
        namespace='imu',
        parameters=[{
            'device': '/dev/i2c-0',
            'address': 40,
            'frame_id': 'imu'
        }],
        remappings=[
            ('/data', '/imu/data'),
            ('/raw', '/imu/raw'),
            ('/mag', '/imu/mag'),
            ('/status', '/imu/status'),
            ('/temp', 'imu/temp'),
        ],
    )
    
    # LIDAR
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )
    
    # ========== FUSION LAYER ==========
    
    # Robot Localization EKF (fuses wheel + IMU)
    # THIS publishes the TF (odom→base_link)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_pose_ekf',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 1.0,
            'two_d_mode': True,
            'publish_tf': True,  # THIS node publishes TF
            
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            
            # Wheel odometry input
            'odom0': '/wheel/odom',
            'odom0_config': [False, False, False,    # position
                            False, False, False,    # yaw orientation (disabled absolute)
                            True,  False, False,    # x, y velocity (fusing forward velocity)
                            False, False, True,     # yaw velocity
                            False, False, False],   # acceleration
            'odom0_differential': False,
            
            # IMU input
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,    # position
                          False, False, False,     # yaw orientation (disabled absolute)
                          False, False, False,     # velocity
                          False, False, True,      # yaw velocity (fuse gyro yaw rate)
                          False, False, False],    # x,y accel (disabled to prevent accelerometer drift)
            'imu0_differential': False,
            'imu0_remove_gravitational_acceleration': True,
            
            'debug': False
        }],
        remappings=[
            ('odometry/filtered', 'robot_pose_ekf/odom_combined')
        ]
    )
    
    # RViz data publisher
    rviz_data_pub = Node(
        package='odometry',
        executable='rviz_data_pub',
        name='rviz_data_pub',
        output='screen'
    )
    
    # ========== VISUALIZATION ==========
    
    rviz_config_path = os.path.join(pkg_arjuna, 'rviz', 'arjuna.rviz')
    if not os.path.exists(rviz_config_path):
        alt_rviz_paths = [
            os.path.join(ws_base, 'src/arjuna/rviz/arjuna.rviz'),
            os.path.join(ws_base, 'src/arjuna/arjuna/rviz/arjuna.rviz'),
            os.path.join(ws_base, 'install/arjuna/share/arjuna/rviz/arjuna.rviz')
        ]
        for path in alt_rviz_paths:
            if os.path.exists(path):
                rviz_config_path = path
                break
    
    if os.path.exists(rviz_config_path):
        os.system(f"chmod 644 {rviz_config_path}")
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )
    
    return LaunchDescription([
        gui_arg,
        publish_map_odom_arg,
        map_file_arg,
        # Static TFs / Robot State
        map_to_odom,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # Map server
        map_server_node,
        lifecycle_manager_node,
        # Sensors
        arjuna_ticks_pub,
        ekf_data_pub,
        imu_node,
        lidar_node,
        # Fusion
        robot_localization_node,  # Publishes TF
        # Utilities
        rviz_data_pub,
        rviz_node,
        # Shutdown when RViz exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[EmitEvent(event=Shutdown())]
            ),
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    ])