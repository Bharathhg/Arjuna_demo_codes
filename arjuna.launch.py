#!/usr/bin/env python3
"""
Arjuna Navigation with IMU Fusion
Wheel odometry + IMU → robot_localization → fused output for better navigation
CRITICAL: EKF_data_pub must have publish_tf parameter support
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
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
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='LIDAR serial port'
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

    map_yaml_file = os.path.join(ws_base, 'src/arjuna/arjuna/maps/final_map.yaml')
    
    # Fix permissions on map files
    maps_dir = os.path.join(ws_base, 'src/arjuna/arjuna/maps')
    if os.path.exists(maps_dir):
        os.system(f"chmod -R 755 {maps_dir}")
    if os.path.exists(map_yaml_file):
        os.system(f"chmod 644 {map_yaml_file}")
    pgm_file = os.path.join(os.path.dirname(map_yaml_file), "final_map.pgm")
    if os.path.exists(pgm_file):
        os.system(f"chmod 644 {pgm_file}")
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=map_yaml_file,
        description='Path to map file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description='Navigation mode: "nav" (static map + AMCL), "slam" (SLAM Toolbox online mapping), or "none" (raw odom only)'
    )
    
    map_posegraph_arg = DeclareLaunchArgument(
        'map_posegraph',
        default_value='',
        description='Path to existing posegraph file prefix (without extension) for lifelong SLAM'
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
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('publish_map_odom'), "' == 'True' and '",
            LaunchConfiguration('mode'), "' not in ['nav', 'slam']"
        ]))
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
    
    # ========== LOCALIZATION AND SLAM LAYER ==========
    
    # Map Server (only in nav mode)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False,
            'autostart': True
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"]))
    )
    
    # AMCL (only in nav mode)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_footprint',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.1,      # Enabled for global localization recovery
            'recovery_alpha_slow': 0.001,    # Enabled for global localization recovery
            'resample_interval': 1,
            'robot_model_type': 'differential',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'transform_tolerance': 1.0,
            'update_min_a': 0.05,            # Lowered to trigger particle updates on small rotations
            'update_min_d': 0.05,            # Lowered to trigger particle updates on small translations
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"]))
    )
    
    # Lifecycle Manager (only in nav mode)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"]))
    )



    # SLAM Toolbox evaluation function for loading existing pose graph
    def evaluate_slam_params(context, *args, **kwargs):
        map_posegraph_val = context.perform_substitution(LaunchConfiguration('map_posegraph'))
        
        params = {
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
        }
        
        if map_posegraph_val.strip() != '':
            params['map_file_name'] = map_posegraph_val
            params['map_start_at_dock'] = False
            
        return [Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'slam'"]))
        )]
    
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
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )
    
    # DepthAI OAK-D Lite Camera
    depthai_camera_node = Node(
        package='demo_okd_lite_camera',
        executable='okd_camera_pub',
        name='demo_camera_publisher',
        output='screen'
    )
    
    # Static transform for RViz visualization (camera_optical_frame to base_link)
    camera_base_link_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_broadcaster',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_optical_frame']
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
            'two_d_mode': False,  # Changed to False to estimate Z, roll, and pitch
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
                          True,  True,  False,     # roll, pitch orientation (fused absolute roll/pitch)
                          False, False, False,     # velocity
                          True,  True,  True,      # roll, pitch, yaw velocity (fused gyro rates)
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
        serial_port_arg,
        publish_map_odom_arg,
        map_file_arg,
        mode_arg,
        map_posegraph_arg,
        # Static TFs / Robot State
        map_to_odom,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # Map server & AMCL
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        # SLAM Toolbox
        OpaqueFunction(function=evaluate_slam_params),
        # Sensors
        arjuna_ticks_pub,
        ekf_data_pub,
        imu_node,
        lidar_node,
        depthai_camera_node,
        camera_base_link_broadcaster,
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