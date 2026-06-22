#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# ─────────────────────────────────────────────────────────────────────────────
# Help text shown on startup
# ─────────────────────────────────────────────────────────────────────────────
msg = """
╔══════════════════════════════════════════════════════╗
║           Arjuna Robot Controller  v2.0              ║
╠══════════════════════════════════════════════════════╣
║  Movement (Normal & Mecanum):                        ║
║    ↑  Arrow Up    → Forward                          ║
║    ↓  Arrow Down  → Backward                         ║
║    ←  Arrow Left  → Turn Left  (Normal) / Strafe L   ║
║    →  Arrow Right → Turn Right (Normal) / Strafe R   ║
║    SPACE          → Emergency Stop                   ║
╠══════════════════════════════════════════════════════╣
║  Velocity Adjustments:                               ║
║    q → Increase Linear + Angular  (both +10%)        ║
║    z → Decrease Linear + Angular  (both -10%)        ║
║    w → Increase Linear only       (+10%)             ║
║    x → Decrease Linear only       (-10%)             ║
║    e → Increase Angular only      (+10%)             ║
║    c → Decrease Angular only      (-10%)             ║
╠══════════════════════════════════════════════════════╣
║  Mode:                                               ║
║    k → Toggle Normal ↔ Mecanum drive                 ║
║  Mecanum extras (Mecanum mode only):                 ║
║    u / o  → Diagonal Fwd-Left / Fwd-Right            ║
║    m / .  → Diagonal Bwd-Left / Bwd-Right            ║
╠══════════════════════════════════════════════════════╣
║  CTRL-C → Quit                                       ║
╚══════════════════════════════════════════════════════╝
"""

# ─────────────────────────────────────────────────────────────────────────────
# Key → (linear_x, linear_y, linear_z, angular_z)  multipliers
# ─────────────────────────────────────────────────────────────────────────────

# Normal (differential) drive: arrows control forward/backward/turn
normalBindings = {
    'up':    ( 1.0,  0.0,  0.0,  0.0),   # Forward
    'down':  (-1.0,  0.0,  0.0,  0.0),   # Backward
    'left':  ( 0.0,  0.0,  0.0,  1.0),   # Turn left
    'right': ( 0.0,  0.0,  0.0, -1.0),   # Turn right
}

# Mecanum drive: arrows control forward/backward/strafe
mecanumBindings = {
    'up':    ( 1.0,  0.0,  0.0,  0.0),   # Forward
    'down':  (-1.0,  0.0,  0.0,  0.0),   # Backward
    'left':  ( 0.0,  1.0,  0.0,  0.0),   # Strafe left
    'right': ( 0.0, -1.0,  0.0,  0.0),   # Strafe right
    'u':     ( 1.0,  1.0,  0.0,  0.0),   # Diagonal Fwd-Left
    'o':     ( 1.0, -1.0,  0.0,  0.0),   # Diagonal Fwd-Right
    'm':     (-1.0,  1.0,  0.0,  0.0),   # Diagonal Bwd-Left
    '.':     (-1.0, -1.0,  0.0,  0.0),   # Diagonal Bwd-Right
}

# ─────────────────────────────────────────────────────────────────────────────
# Velocity adjustment bindings
# Each entry: (linear_multiplier, angular_multiplier)
# ─────────────────────────────────────────────────────────────────────────────
speedBindings = {
    'q': (1.1, 1.1),   # Both increase
    'z': (0.9, 0.9),   # Both decrease
    'w': (1.1, 1.0),   # Linear increase
    'x': (0.9, 1.0),   # Linear decrease
    'e': (1.0, 1.1),   # Angular increase
    'c': (1.0, 0.9),   # Angular decrease
}


def getKey():
    """Read one key press; translates arrow escape sequences into named strings."""
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if   extra == '[A': key = 'up'
            elif extra == '[B': key = 'down'
            elif extra == '[C': key = 'right'
            elif extra == '[D': key = 'left'
            else:               key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "  speed: {:.3f}  |  turn: {:.3f}".format(speed, turn)


class ArjunaController(Node):
    def __init__(self):
        super().__init__('Arjuna_Controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # ROS parameters
        self.speed = self.declare_parameter('speed', 0.5).value
        self.turn  = self.declare_parameter('turn',  1.0).value

        # Motion state
        self.x  = 0.0  # linear x
        self.y  = 0.0  # linear y (mecanum strafe)
        self.z  = 0.0  # linear z (unused)
        self.th = 0.0  # angular z

        # Drive mode: 'normal' | 'mecanum'
        self.mode = 'normal'

        self.twist = Twist()

    # ------------------------------------------------------------------
    def publish_twist(self):
        self.twist.linear.x  = self.x  * self.speed
        self.twist.linear.y  = self.y  * self.speed
        self.twist.linear.z  = self.z  * self.speed
        self.twist.angular.z = self.th * self.turn
        self.publisher_.publish(self.twist)

    # ------------------------------------------------------------------
    def run_controller(self):
        print(msg)
        self.get_logger().info("Mode: NORMAL  " + vels(self.speed, self.turn))

        while rclpy.ok():
            key = getKey()

            # ── Velocity adjustments ──────────────────────────────────
            if key in speedBindings:
                self.speed *= speedBindings[key][0]
                self.turn  *= speedBindings[key][1]
                self.get_logger().info(
                    "[{}] ".format(key.upper()) + vels(self.speed, self.turn)
                )

            # ── Mode toggle ───────────────────────────────────────────
            elif key == 'k':
                # Stop robot when switching modes
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.publish_twist()
                if self.mode == 'normal':
                    self.mode = 'mecanum'
                    self.get_logger().info("Mode switched → MECANUM")
                else:
                    self.mode = 'normal'
                    self.get_logger().info("Mode switched → NORMAL")

            # ── Emergency stop ────────────────────────────────────────
            elif key == ' ':
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.get_logger().info("⛔ EMERGENCY STOP")

            # ── Quit ──────────────────────────────────────────────────
            elif key == '\x03':  # CTRL-C
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.publish_twist()
                self.get_logger().info("Exiting controller…")
                break

            # ── Motion keys ───────────────────────────────────────────
            else:
                bindings = normalBindings if self.mode == 'normal' else mecanumBindings
                if key in bindings:
                    self.x, self.y, self.z, self.th = bindings[key]

            self.publish_twist()


def run_arjuna(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ArjunaController()
    try:
        node.run_controller()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    run_arjuna()