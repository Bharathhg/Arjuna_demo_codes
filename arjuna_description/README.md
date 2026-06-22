# arjuna_description — Robot URDF & Gazebo Model

ROS 2 package providing the Arjuna robot's URDF description, Gazebo SDF model, and associated launch files for visualization and simulation.

## Package Structure

```
arjuna_description/
├── urdf/
│   ├── arjuna.urdf    # Robot URDF — links, joints, sensors
│   └── arjuna.sdf     # Gazebo SDF model (for simulation)
├── launch/
│   ├── all_in_one.launch.py   # Start everything: URDF + Gazebo + RViz
│   ├── gazebo.launch.py       # Gazebo simulation only
│   └── rviz.launch.py         # RViz visualization only
├── Frames/                    # TF frame reference diagrams
├── include/                   # C++ headers (if any)
├── src/                       # C++ source (if any)
├── worlds/                    # Gazebo world (.world) files
├── CMakeLists.txt
└── package.xml
```

## How to Run

```bash
source ~/arjuna2_ws/install/setup.bash

# Visualize URDF in RViz only
ros2 launch arjuna_description rviz.launch.py

# Launch Gazebo simulation only
ros2 launch arjuna_description gazebo.launch.py

# Launch all (URDF + Gazebo + RViz)
ros2 launch arjuna_description all_in_one.launch.py
```

## TF Frame Tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── laser        (LIDAR)
                ├── imu          (IMU BNO055)
                └── camera_link  (USB Camera)
```

## Key Nodes Started

| Node | Package | Description |
|------|---------|-------------|
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF from URDF |
| `joint_state_publisher` | `joint_state_publisher` | Publishes joint states |
| `gazebo` | `gazebo_ros` | Gazebo simulator |
| `rviz2` | `rviz2` | 3D visualizer |

## License
MIT — Newrro Tech
