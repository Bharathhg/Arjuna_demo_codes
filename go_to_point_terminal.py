#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
from time import sleep
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
import os

# Add Newrro_NavLib to path (use realpath to resolve symlinks from install/)
_NAV_DIR = os.path.realpath(os.path.dirname(__file__))
sys.path.append(os.path.join(_NAV_DIR, 'Newrro_NavLib'))
from Newrro_Navigation import (
    fix_yaw, go_straight_ahead, stop, set_cmd_publisher
)
from Newrro_obs_script import (
    is_path_clear, avoid_obstacle, is_obstacle_detected, reset_avoidance_state, is_currently_avoiding
)

# Navigation parameters
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.8
RECOVERY_TIMEOUT = 5.0

class GoToPointTerminal(Node):
    def __init__(self):
        super().__init__('go_to_point_terminal')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0  # 0=rotate, 1=move, 2=goal reached
        
        # LIDAR regions
        self.regions = {}
        
        # Stuck detection
        self.last_position_ = Point()
        self.stuck_time_ = 0.0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        set_cmd_publisher(self.cmd_pub)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub_combined = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose_ekf/odom_combined',
            self.odom_combined_callback,
            10
        )
        
        self.odom_sub_raw = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_raw_callback,
            10
        )
        
        # TF listener for drift-corrected localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Go To Point (Terminal) Node Started")
        self.get_logger().info(f"Linear velocity: {LINEAR_VELOCITY} m/s")
        self.get_logger().info(f"Angular velocity: {ANGULAR_VELOCITY} rad/s")
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
            
        def get_region_min(start_fraction, end_fraction):
            start_idx = int(num_ranges * start_fraction)
            end_idx = int(num_ranges * end_fraction)
            if start_idx >= end_idx or end_idx > num_ranges:
                return 10.0
            region_ranges = msg.ranges[start_idx:end_idx]
            if len(region_ranges) == 0:
                return 10.0
            valid_ranges = [r for r in region_ranges if 0.1 < r < 10.0]
            return min(valid_ranges) if valid_ranges else 10.0
            
        self.regions = {
            'front_L': get_region_min(0.0, 0.15),
            'fleft':   get_region_min(0.15, 0.27),
            'left':    get_region_min(0.27, 0.33),
            'right':   get_region_min(0.67, 0.73),
            'fright':  get_region_min(0.73, 0.85),
            'front_R': get_region_min(0.85, 1.0)
        }
        
    def odom_combined_callback(self, msg):
        """Process filtered odometry data"""
        self.update_pose(msg.pose.pose)
        
    def odom_raw_callback(self, msg):
        """Process raw odometry data"""
        self.update_pose(msg.pose.pose)
        
    def update_pose(self, pose):
        """Update current position and orientation"""
        self.position_ = pose.position
        
        # Extract yaw from quaternion
        orientation = pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw_ = math.atan2(siny_cosp, cosy_cosp)
        
    def check_if_stuck(self):
        """Check if robot is stuck"""
        dist_moved = math.sqrt(
            (self.position_.x - self.last_position_.x)**2 +
            (self.position_.y - self.last_position_.y)**2
        )
        
        if dist_moved < 0.01 and self.state_ == 1:
            self.stuck_time_ += 0.1
            if self.stuck_time_ > RECOVERY_TIMEOUT:
                self.get_logger().warn("Robot stuck! Activating recovery")
                self.state_ = 0
                self.stuck_time_ = 0.0
        else:
            self.stuck_time_ = 0.0
            
        self.last_position_.x = self.position_.x
        self.last_position_.y = self.position_.y
        
    def get_current_pose(self):
        """Try to get the robot's pose from TF map -> base_footprint.
        Falls back to the callback-based pose if TF is unavailable."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            position = Point()
            position.x = trans.transform.translation.x
            position.y = trans.transform.translation.y
            position.z = trans.transform.translation.z
            
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return position, yaw
        except Exception as ex:
            return None

    def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to goal point with persistent obstacle avoidance"""
        desired_pose = Point()
        desired_pose.x = goal_x
        desired_pose.y = goal_y
        
        # Reset avoidance state for new navigation
        reset_avoidance_state()
        
        # Initialize state
        self.state_ = 0
        self.last_position_.x = self.position_.x
        self.last_position_.y = self.position_.y
        self.stuck_time_ = 0.0
        
        self.get_logger().info("-------------------------")
        self.get_logger().info(f"Goal point: ({goal_x:.2f}, {goal_y:.2f})")
        self.get_logger().info("-------------------------")
        self.get_logger().info("*** NAVIGATION STARTED ***")
        
        rate = self.create_rate(10)  # 10Hz
        
        while rclpy.ok():
            # Try to update pose from TF first
            tf_pose = self.get_current_pose()
            if tf_pose is not None:
                self.position_, self.yaw_ = tf_pose
            
            # Check for stuck condition
            self.check_if_stuck()
            
            # PRIORITY: Check for obstacles - this MUST complete before navigation
            if self.regions and (is_obstacle_detected(self.regions) or is_currently_avoiding()):
                # Get obstacle avoidance command
                obstacle_cmd = avoid_obstacle(self.regions)
                
                if obstacle_cmd:
                    # Execute obstacle avoidance - this blocks normal navigation
                    self.cmd_pub.publish(obstacle_cmd)
                    # Don't execute navigation while avoiding
                    rate.sleep()
                    continue
            
            # Only execute navigation if no obstacle avoidance is active
            if self.state_ == 0:
                # Orientation correction
                self.yaw_, self.position_, self.state_ = fix_yaw(
                    desired_pose, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 1:
                # Move straight
                self.yaw_, self.position_, self.state_ = go_straight_ahead(
                    desired_pose, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 2:
                # Goal reached
                stop()
                self.get_logger().info("*** NAVIGATION COMPLETED ***")
                return True
            else:
                self.get_logger().error(f"Unknown state: {self.state_}")
                stop()
                return False
                
            # Display progress every 3 seconds
            if int(self.get_clock().now().seconds_nanoseconds()[0]) % 3 == 0:
                distance_to_goal = math.sqrt(
                    (desired_pose.y - self.position_.y)**2 +
                    (desired_pose.x - self.position_.x)**2
                )
                self.get_logger().info(
                    f"Distance: {distance_to_goal:.2f}m, State: {self.state_}"
                )
            
            rate.sleep()
        
        return False

def main(args=None):
    rclpy.init(args=args)
    node = GoToPointTerminal()
    
    # Spin node in background thread so callbacks are processed in real-time
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Wait for initial sensor data
    sleep(1)
    
    try:
        while rclpy.ok():
            # Get goal from user
            try:
                x = float(input("\nEnter X coordinate for goal: "))
                y = float(input("Enter Y coordinate for goal: "))
            except ValueError:
                print("Invalid input. Please enter numeric values.")
                continue
            
            # Navigate to goal
            node.navigate_to_goal(x, y)
            
            # Ask to continue
            cont = input("\nNavigate to another point? (y/n): ")
            if cont.lower() != 'y':
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()