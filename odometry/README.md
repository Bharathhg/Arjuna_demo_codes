# odometry — Wheel Odometry (C++)

ROS 2 C++ package that computes wheel odometry from encoder ticks and publishes data suitable for the `robot_localization` EKF.

## Package Structure

```
odometry/
├── src/
│   ├── ekf_data_pub.cpp    # Computes odometry from ticks → /wheel/odom
│   └── rviz_data_pub.cpp   # Publishes RViz markers for odometry visualization
├── config/                 # Configuration YAMLs (if any)
├── launch/
│   └── odometru_launch.py  # Launch odometry nodes
├── CMakeLists.txt
└── package.xml
```

## How to Run

```bash
source ~/arjuna2_ws/install/setup.bash

# Launch via launch file
ros2 launch odometry odometru_launch.py

# Or run nodes directly
ros2 run odometry ekf_data_pub
ros2 run odometry rviz_data_pub
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ticks` | custom | Subscribed | Encoder ticks from `motor_ops` |
| `/wheel/odom` | `nav_msgs/Odometry` | Published | Wheel odometry (for EKF) |
| `/wheel/odom_data_quat` | custom | Published | Odometry in quaternion format |
| `/wheel/odom_data_euler` | custom | Published | Odometry in Euler format |

> **Note:** `publish_tf` is set to `False` in the navigation stack — `robot_localization` handles TF publication.

## License
MIT — Newrro Tech
