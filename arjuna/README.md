# arjuna — Main Robot Package

ROS 2 package for the Arjuna autonomous robot. Contains all launch files, navigation scripts, SLAM configuration, web GUI, camera vision, CPU/RAM monitoring, and ML/DL/RL model directories.

## Package Structure

```
arjuna/
├── config/
│   ├── arjuna_params.yaml              # General robot parameters
│   ├── arjuna_slam.yaml                # Full SLAM Toolbox configuration
│   ├── ekf.yaml                        # robot_localization EKF parameters
│   └── mapper_params_online_async.yaml # Online async SLAM params (used in mapping)
├── database/                           # Runtime storage — logs, records (add files here)
├── launch/
│   ├── arjuna.launch.py                # Full navigation bringup (map + EKF + Nav2 + RViz)
│   ├── slam_mapping.launch.py          # SLAM mapping session
│   ├── save_map.launch.py              # Save the SLAM map to disk
│   └── web_gui.launch.py              # Start the web control dashboard
├── maps/
│   ├── my_map.yaml / my_map.pgm       # Primary saved map
│   └── my_map1.yaml / my_map1.pgm     # Secondary saved map
├── models/
│   ├── ml_model/                      # Place ML model files here (sklearn, etc.)
│   ├── dl_model/                      # Place DL model files here (PyTorch, TF, etc.)
│   └── rl_model/                      # Place RL model files here (stable-baselines, etc.)
├── rviz/
│   └── arjuna.rviz                    # RViz2 display configuration
├── scripts/
│   ├── web_gui.py                     # Web GUI server node
│   ├── cpu_ram/
│   │   └── cpu_ram.py                 # CPU & RAM stats publisher
│   ├── navigation/
│   │   ├── go_to_point_rviz.py        # Navigate to RViz 2D goal click
│   │   ├── go_to_point_terminal.py    # Navigate via terminal coordinate input
│   │   ├── multi_point_rviz.py        # Multi-waypoint nav via RViz
│   │   ├── multi_point_terminal.py    # Multi-waypoint nav via terminal
│   │   ├── obstacle_avoidance.py      # Reactive obstacle avoidance node
│   │   ├── algorithm_selector_rviz.py # Algorithm picker with RViz panel
│   │   ├── Algorithms/
│   │   │   ├── astar.py               # A* path planning
│   │   │   ├── dijkstra.py            # Dijkstra shortest-path
│   │   │   ├── rrt_star.py            # RRT* sampling-based planner
│   │   │   ├── apf.py                 # Artificial Potential Field
│   │   │   ├── bug0.py                # Bug0 reactive algorithm
│   │   │   ├── bug1.py                # Bug1 reactive algorithm
│   │   │   ├── bug2.py                # Bug2 reactive algorithm
│   │   │   └── zigzag.py              # ZigZag coverage sweep
│   │   └── Newrro_NavLib/
│   │       ├── Newrro_Navigation.py   # Newrro custom nav helper library
│   │       └── Newrro_obs_script.py   # Newrro obstacle handling library
│   └── opencv/
│       ├── camera_pub.py              # Publish /camera/image_raw from USB webcam
│       ├── camera_sub.py              # Subscribe and display camera feed
│       ├── qr_rec.py                  # QR code recognizer node
│       ├── qr_ct.py                   # QR code continuous tracking node
│       └── qr_tracking.py             # Robot follows detected QR code
└── webpage/
    ├── Arjuna.html                    # Web dashboard UI
    ├── Arjuna.css                     # Styles (glassmorphism theme)
    └── Arjuna.js                      # rosbridge WebSocket client logic
```

## How to Run

```bash
source ~/arjuna2_ws/install/setup.bash

# ── SLAM Mapping ────────────────────────────────────────────────
ros2 launch arjuna slam_mapping.launch.py

# ── Save Map ────────────────────────────────────────────────────
ros2 launch arjuna save_map.launch.py

# ── Full Navigation ─────────────────────────────────────────────
ros2 launch arjuna arjuna.launch.py
ros2 launch arjuna arjuna.launch.py gui:=False     # Without RViz

# ── Web Dashboard ───────────────────────────────────────────────
ros2 launch arjuna web_gui.launch.py               # Open http://localhost:8080

# ── Navigation Nodes ────────────────────────────────────────────
ros2 run arjuna go_to_point_rviz
ros2 run arjuna go_to_point_terminal
ros2 run arjuna multi_point_rviz
ros2 run arjuna multi_point_terminal
ros2 run arjuna obstacle_avoidance
ros2 run arjuna algorithm_selector

# ── Camera / Vision ─────────────────────────────────────────────
ros2 run arjuna camera_pub
ros2 run arjuna camera_sub
ros2 run arjuna qr_recog
ros2 run arjuna qr_tracking
ros2 run arjuna qr_ct

# ── System Monitor ──────────────────────────────────────────────
ros2 run arjuna cpu_ram_pub
```

## Key Topics Published / Subscribed

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/cmd_vel` | Subscribed | Drive commands from Nav2 or teleop |
| `/map` | Subscribed | Occupancy grid from map_server |
| `/scan` | Subscribed | LIDAR data for navigation |
| `/camera/image_raw` | Published | Raw USB camera frames |
| `/cpu_ram_stats` | Published | CPU & RAM usage |

## License
MIT — Newrro Tech
