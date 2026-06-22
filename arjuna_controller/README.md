# arjuna_controller — Keyboard Teleop

Terminal-based keyboard teleoperation node for the Arjuna robot. Publishes `geometry_msgs/msg/Twist` to `/cmd_vel`.

## Features
- Supports **Normal (Differential)** and **Mecanum** drive modes
- Stateful — robot keeps last velocity until you issue a new command
- Live speed adjustment via keyboard
- Emergency stop with `Space`

## Package Structure

```
arjuna_controller/
├── arjuna_controller/
│   └── controller_node.py    # Main teleoperation node
├── resource/
├── package.xml
├── setup.cfg
└── setup.py
```

## How to Run

```bash
source ~/arjuna2_ws/install/setup.bash
ros2 run arjuna_controller arjuna_controller
```

## Key Bindings

### Movement (works in both modes)

| Key | Action |
|-----|--------|
| `↑` Arrow Up | Move Forward |
| `↓` Arrow Down | Move Backward |
| `←` Arrow Left | Turn Left **(Normal)** / Strafe Left **(Mecanum)** |
| `→` Arrow Right | Turn Right **(Normal)** / Strafe Right **(Mecanum)** |
| `Space` | **Emergency Stop** |

### Speed Adjustment

| Key | Effect |
|-----|--------|
| `q` | Increase Linear + Angular (+10%) |
| `z` | Decrease Linear + Angular (−10%) |
| `w` | Increase Linear speed only (+10%) |
| `x` | Decrease Linear speed only (−10%) |
| `e` | Increase Angular speed only (+10%) |
| `c` | Decrease Angular speed only (−10%) |

### Mode & Misc

| Key | Effect |
|-----|--------|
| `k` | Toggle **Normal ↔ Mecanum** mode |
| `Ctrl-C` | Quit cleanly |

## Mecanum Extras (Mecanum mode only)

| Key | Action |
|-----|--------|
| `u` | Diagonal Forward-Left |
| `o` | Diagonal Forward-Right |
| `m` | Diagonal Backward-Left |
| `.` | Diagonal Backward-Right |

## ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed` | `0.5` | Initial linear speed (m/s) |
| `turn` | `1.0` | Initial angular speed (rad/s) |

```bash
# Run with custom speeds
ros2 run arjuna_controller arjuna_controller --ros-args -p speed:=0.3 -p turn:=0.8
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands to motors |

## License
MIT — Newrro Tech
