#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
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
    stop, set_cmd_publisher
)
from Newrro_obs_script import (
    is_path_clear, avoid_obstacle, is_obstacle_detected, reset_avoidance_state, is_currently_avoiding
)

# Navigation parameters
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.8
RECOVERY_TIMEOUT = 5.0
GOAL_TOLERANCE = 0.05
YAW_TOLERANCE = 0.087

class MultiPointTerminal(Node):
    def __init__(self):
        super().__init__('multi_point_terminal')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
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
            Odometry,
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
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Multi-Point Navigation (Terminal) Started")
        self.get_logger().info("===========================================")
        self.get_logger().info(f"Linear velocity: {LINEAR_VELOCITY} m/s")
        self.get_logger().info(f"Angular velocity: {ANGULAR_VELOCITY} rad/s")
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
            
        def get_region_min(start_deg, end_deg):
            """Calculate min distance in a degree-based zone"""
            start_idx = int((start_deg / 360.0) * num_ranges)
            end_idx = int((end_deg / 360.0) * num_ranges)
            
            if start_idx < end_idx:
                subset = msg.ranges[start_idx:end_idx]
            else:
                subset = msg.ranges[start_idx:] + msg.ranges[:end_idx]
            
            # Filter valid points (between min/max range, ignore chassis at < 0.15m)
            min_range = max(msg.range_min, 0.15)
            valid = [r for r in subset if min_range < r < msg.range_max]
            return min(valid) if valid else 10.0
            
        self.regions = {
            'front_L': get_region_min(0, 30),
            'fleft':   get_region_min(30, 75),
            'left':    get_region_min(75, 135),
            'right':   get_region_min(225, 285),
            'fright':  get_region_min(285, 330),
            'front_R': get_region_min(330, 360)
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
        """Try to get the robot's pose from TF odom -> base_footprint.
        Falls back to the callback-based pose if TF is unavailable."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
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

    def normalize_angle(self, angle):
        """Keep angle between -π and π"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to a single goal point"""
        desired_pose = Point()
        desired_pose.x = goal_x
        desired_pose.y = goal_y
        
        # Reset avoidance state
        reset_avoidance_state()
        
        # Initialize state
        self.state_ = 0  # 0: normal navigation, 2: goal reached
        self.last_position_.x = self.position_.x
        self.last_position_.y = self.position_.y
        self.stuck_time_ = 0.0
        
        self.get_logger().info("-------------------------")
        self.get_logger().info(f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
        self.get_logger().info("-------------------------")
        
        rate = self.create_rate(10)
        
        while rclpy.ok():
            # Try to update pose from TF first
            tf_pose = self.get_current_pose()
            if tf_pose is not None:
                self.position_, self.yaw_ = tf_pose
            
            # Check for stuck
            self.check_if_stuck()
            
            # Check for obstacles - priority
            if self.regions and (is_obstacle_detected(self.regions) or is_currently_avoiding()):
                obstacle_cmd = avoid_obstacle(self.regions)
                if obstacle_cmd:
                    self.cmd_pub.publish(obstacle_cmd)
                    rate.sleep()
                    continue
            
            # ===== STEP 1: RECALCULATE navigation parameters =====
            dx = desired_pose.x - self.position_.x
            dy = desired_pose.y - self.position_.y
            distance_to_goal = math.sqrt(dx * dx + dy * dy)
            heading_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(heading_to_goal - self.yaw_)
            
            # ===== STEP 2: CHECK if goal reached =====
            if distance_to_goal <= GOAL_TOLERANCE:
                stop()
                self.get_logger().info("Waypoint reached")
                return True
            
            # ===== STEP 3: NORMAL NAVIGATION =====
            cmd = Twist()
            
            # Check if we need to rotate
            if abs(angle_error) > YAW_TOLERANCE:
                # Rotate to face goal with proportional control
                kp_angular = 1.5
                cmd.angular.z = kp_angular * angle_error
                # Clamp to max velocity
                if cmd.angular.z > ANGULAR_VELOCITY:
                    cmd.angular.z = ANGULAR_VELOCITY
                elif cmd.angular.z < -ANGULAR_VELOCITY:
                    cmd.angular.z = -ANGULAR_VELOCITY
                cmd.linear.x = 0.0
            else:
                # Move forward with minor heading correction
                cmd.linear.x = LINEAR_VELOCITY
                kp_angular = 0.8
                cmd.angular.z = kp_angular * angle_error
                # Clamp angular correction
                if cmd.angular.z > ANGULAR_VELOCITY * 0.3:
                    cmd.angular.z = ANGULAR_VELOCITY * 0.3
                elif cmd.angular.z < -ANGULAR_VELOCITY * 0.3:
                    cmd.angular.z = -ANGULAR_VELOCITY * 0.3
            
            self.cmd_pub.publish(cmd)
            
            # Display progress
            if int(self.get_clock().now().seconds_nanoseconds()[0]) % 3 == 0:
                self.get_logger().info(f"Distance: {distance_to_goal:.2f}m, Angle error: {math.degrees(angle_error):.1f}°")
                
            rate.sleep()
        
        return False
    
    def get_waypoints(self):
        """Collect waypoints from user"""
        waypoints = []
        print("\n===========================================")
        print("MULTI-POINT WAYPOINT COLLECTION")
        print("===========================================")
        
        # Ask for number of waypoints
        while True:
            try:
                num_points_input = input("Enter the number of waypoints to navigate: ")
                num_points = int(num_points_input.strip())
                if num_points <= 0:
                    print("Please enter a positive integer.")
                    continue
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")
                
        print(f"\nPlease enter coordinates for {num_points} waypoints.")
        print("")
        
        waypoint_num = 1
        while waypoint_num <= num_points:
            print(f"Waypoint #{waypoint_num}:")
            user_input = input("  Enter X,Y coordinates (example: 1.5,2.3): ")
            
            try:
                coords = user_input.split(',')
                if len(coords) != 2:
                    print("  Invalid format. Use: X,Y")
                    continue
                    
                x = float(coords[0].strip())
                y = float(coords[1].strip())
                
                waypoints.append((x, y))
                print(f"  ✓ Waypoint #{waypoint_num} added: ({x:.2f}, {y:.2f})")
                waypoint_num += 1
                
            except ValueError:
                print("  Invalid input. Enter numbers.")
                continue
        
        print("\n===========================================")
        print("WAYPOINT COLLECTION COMPLETE")
        print("===========================================")
        print(f"Total waypoints: {len(waypoints)}")
        for i, (x, y) in enumerate(waypoints):
            print(f"  #{i+1}: ({x:.2f}, {y:.2f})")
        print("")
        
        return waypoints
    
    def execute_waypoints(self, waypoints):
        """Navigate through all waypoints"""
        print("===========================================")
        print("STARTING MULTI-POINT NAVIGATION")
        print("===========================================")
        print(f"Total waypoints: {len(waypoints)}")
        print("")
        
        for i, (x, y) in enumerate(waypoints):
            print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f})")
            
            success = self.navigate_to_goal(x, y)
            
            if not success:
                print(f"\n✗ Navigation failed at waypoint #{i+1}")
                return False
            
            print(f"✓ Waypoint #{i+1} reached successfully")
            
            # Pause between waypoints
            if i < len(waypoints) - 1:
                sleep(1)
        
        print("\n===========================================")
        print("ALL WAYPOINTS COMPLETED SUCCESSFULLY")
        print("===========================================")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointTerminal()
    
    # Spin node in background thread so callbacks are processed in real-time
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Wait for initial data
    sleep(1)
    
    try:
        while rclpy.ok():
            # Get waypoints
            waypoints = node.get_waypoints()
            
            # Execute navigation
            node.execute_waypoints(waypoints)
            
            # Ask to continue
            print("")
            cont = input("Plan another multi-point route? (y/n): ")
            if cont.lower() != 'y':
                print("\nExiting multi-point navigation.")
                break
                
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
