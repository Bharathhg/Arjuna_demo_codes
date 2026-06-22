# motor_ops — Motor Driver Interface

ROS 2 package that interfaces with physical motors via the **STServo SDK** over serial (USB). Subscribes to `/cmd_vel` and drives wheels. Also publishes encoder ticks.

## Package Structure

```
motor_ops/
├── motor_ops/
│   ├── cmd_vel_sub.py          # Differential drive: /cmd_vel → motor commands
│   ├── mec_cmd_vel_sub.py      # Mecanum drive: /cmd_vel → 4-wheel commands
│   ├── ticks_pub.py            # Encoder ticks publisher → /ticks
│   └── STservo_sdk/            # ST Robotics serial servo SDK
│       ├── __init__.py
│       ├── port_handler.py     # Serial port abstraction
│       ├── protocol_packet_handler.py  # Servo protocol implementation
│       ├── group_sync_read.py  # Sync read from multiple servos
│       ├── group_sync_write.py # Sync write to multiple servos
│       ├── scscl.py            # SCS/SCL series servo driver
│       ├── sts.py              # STS series servo driver
│       └── stservo_def.py      # Register address definitions
├── resource/
├── package.xml
├── setup.cfg
└── setup.py
```

## How to Run

```bash
source ~/arjuna2_ws/install/setup.bash

# Differential (Normal) drive mode — subscribes /cmd_vel
ros2 run motor_ops cmd_vel_sub

# Mecanum drive mode — subscribes /cmd_vel
ros2 run motor_ops mec_cmd_vel_sub

# Encoder ticks publisher
ros2 run motor_ops ticks_pub
```

## Hardware Setup

| Device | Default Port | Baud Rate |
|--------|-------------|-----------|
| Motor controller | `/dev/ttyUSB0` | `1000000` |

```bash
# Set port permissions
sudo chmod 666 /dev/ttyUSB0
# Or permanently:
sudo usermod -aG dialout $USER
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribed | Drive velocity commands |
| `/ticks` | custom | Published | Encoder tick counts from motors |

## License
MIT — Newrro Tech
