import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    print("\n" + "=" * 60)
    print("            ARJUNA MAP SAVER")
    print("=" * 60)
    
    # Prompt the user for the map name at start of launch
    try:
        map_name = input("Enter map name to save (default: my_map): ").strip()
    except (EOFError, KeyboardInterrupt):
        map_name = ""
        
    if not map_name:
        map_name = "my_map"
        
    # Dynamically resolve active workspace maps folder path
    ws_base = '/home/hacker/arjuna2_ws'
    for path in ['/home/hacker/arjuna2_ws', '/root/arjuna_ros2/arjuna2_ws']:
        if os.path.exists(path):
            ws_base = path
            break
            
    maps_dir = os.path.join(ws_base, 'src/arjuna/arjuna/maps')
    if not os.path.exists(maps_dir):
        os.makedirs(maps_dir)
        
    map_path = os.path.join(maps_dir, map_name)
    
    print(f"Saving map as '{map_name}' in: {maps_dir}...")
    print("=" * 60 + "\n")
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path
            ],
            output='screen'
        )
    ])
