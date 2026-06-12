#!/usr/bin/env python3
"""
SLAM Mapping with IMU Fusion
Wheel odometry + IMU → robot_localization → fused output for better mapping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    arjuna_slam_share = FindPackageShare('arjuna')
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='LIDAR serial port'
    )


    


    # Find the active workspace path dynamically
    ws_base = '/root/arjuna_ros2/arjuna2_ws'
    for path in ['/home/hacker/arjuna2_ws', '/root/arjuna_ros2/arjuna2_ws']:
        if os.path.exists(path):
            ws_base = path
            break

    pkg_arjuna = get_package_share_directory('arjuna')
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

    pkg_arjuna_description = get_package_share_directory('arjuna_description')
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

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,

        
        # ========== SENSOR LAYER ==========
        
        # Motor Encoder Publisher
        Node(
            package='motor_ops',
            executable='ticks_pub',
            name='Arjuna_Ticks_Pub',
            output='screen'
        ),
        
        # Wheel Odometry (NO TF - robot_localization will publish TF)
        Node(
            package='odometry',
            executable='ekf_data_pub',
            name='EKF_data_pub',
            output='screen',
            parameters=[{'publish_tf': False}],  # CRITICAL: Disable TF
            remappings=[
                ('/odom', '/wheel/odom'),  # Rename to avoid conflict
                ('/odom_data_quat', '/wheel/odom_data_quat'),
                ('/odom_data_euler', '/wheel/odom_data_euler')
            ]
        ),
        
        # IMU
        Node(
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
            output='screen'
        ),
        
        # LIDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),
        
        # ========== TRANSFORM LAYER ==========
        # Note: Handled by robot_state_publisher and URDF definition
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        # ========== FUSION LAYER ==========
        
        # Robot Localization EKF (fuses wheel + IMU)
        # THIS publishes the TF (odom→base_footprint)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'sensor_timeout': 1.0,
                'two_d_mode': False,  # Changed to False to estimate Z, roll, and pitch
                'publish_tf': True,  # THIS node publishes TF
                
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
                              True,  True,  False,     # roll, pitch orientation (fused absolute roll/pitch)
                              False, False, False,     # velocity
                              True,  True,  True,      # roll, pitch, yaw velocity (fused gyro rates)
                              False, False, False],    # x,y accel (disabled to prevent accelerometer drift)
                'imu0_differential': False,
                'imu0_remove_gravitational_acceleration': True,
                
                'debug': False
            }],
            remappings=[
                ('/odometry/filtered', '/odom')  # Output as standard /odom
            ]
        ),
        
        # ========== SLAM LAYER ==========
        
        # SLAM Toolbox (uses FUSED /odom)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                
                'minimum_travel_distance': 0.01,
                'minimum_travel_heading': 0.01,
                'minimum_time_interval': 0.1,
                
                'transform_publish_period': 0.02,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'map_update_interval': 1.0,
                
                'throttle_scans': 1,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': True,
                
                'do_loop_closing': True,
                'loop_search_maximum_distance': 3.0,
                'loop_match_minimum_chain_size': 10,
                
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01
            }]
        ),
        
        # RViz
        rviz_node,

        # Auto-shutdown when RViz exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )
    ])