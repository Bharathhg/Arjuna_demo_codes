import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    print("\n" + "=" * 60)
    print("            ARJUNA MAP SAVER")
    print("=" * 60)
    
    # Dynamically resolve active workspace maps folder path
    ws_base = '/home/hacker/arjuna2_ws'
    for path in ['/home/hacker/arjuna2_ws', '/root/arjuna_ros2/arjuna2_ws']:
        if os.path.exists(path):
            ws_base = path
            break
            
    maps_dir = os.path.join(ws_base, 'src/arjuna/arjuna/maps')
    if not os.path.exists(maps_dir):
        os.makedirs(maps_dir)
        
    # Declare launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Name of the map to save'
    )
    
    map_name = LaunchConfiguration('map_name')
    map_path = PathJoinSubstitution([maps_dir, map_name])
    
    print(f"Saving map in: {maps_dir}...")
    print("=" * 60 + "\n")
    
    return LaunchDescription([
        map_name_arg,
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '--ros-args',
                '-p', 'save_map_timeout:=20.0'
            ],
            output='screen'
        )
    ])
