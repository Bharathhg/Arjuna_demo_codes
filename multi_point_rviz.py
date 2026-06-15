#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
import std_srvs.srv
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

class MultiPointRViz(Node):
    def __init__(self):
        super().__init__('multi_point_rviz')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
        # Waypoint management
        self.waypoints = []
        self.collecting_waypoints = False
        self.executing_waypoints = False
        self.current_waypoint_index = 0
        self.target_waypoints_count = 0
        
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
        
        self.goal_sub_2d = self.create_subscription(
            PoseStamped,
            '/goal_2d',
            self.goal_callback,
            10)
        
        self.goal_sub_simple = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10)
            
        self.goal_sub_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        
        # TF listener for drift-corrected localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for navigation
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Multi-Point Navigation (RViz) Started")
        self.get_logger().info("===========================================")
        self.print_instructions()
        
    def print_instructions(self):
        """Print usage instructions"""
        self.get_logger().info("")
        self.get_logger().info("INSTRUCTIONS:")
        self.get_logger().info("1. Call 'start_collection' service to begin")
        self.get_logger().info("2. Set waypoints using '2D Goal Pose' in RViz")
        self.get_logger().info("3. Call 'execute_waypoints' service to start navigation")
        self.get_logger().info("")
        
        # Create services for waypoint management
        self.start_srv = self.create_service(
            std_srvs.srv.Trigger,
            'start_collection',
            self.start_collection_callback
        )
        self.execute_srv = self.create_service(
            std_srvs.srv.Trigger,
            'execute_waypoints',
            self.execute_waypoints_callback
        )
        self.clear_srv = self.create_service(
            std_srvs.srv.Trigger,
            'clear_waypoints',
            self.clear_waypoints_callback
        )
        
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
        
        orientation = pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw_ = math.atan2(siny_cosp, cosy_cosp)
        
    def goal_callback(self, msg):
        """Process goal from RViz"""
        if self.collecting_waypoints:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # Deduplicate by checking if coordinates are extremely close to the last waypoint
            if self.waypoints:
                last_wp = self.waypoints[-1]
                dist = math.sqrt((x - last_wp.x)**2 + (y - last_wp.y)**2)
                if dist < 0.05: # if closer than 5cm, it's a duplicate click
                    return
            
            # Deduplicate by timestamp to prevent duplicate messages from multiple topics
            current_time = self.get_clock().now()
            if hasattr(self, 'last_goal_time'):
                time_diff = (current_time - self.last_goal_time).nanoseconds / 1e9
                if time_diff < 0.5: # 500ms debounce
                    return
            self.last_goal_time = current_time
            
            waypoint = Point()
            waypoint.x = x
            waypoint.y = y
            waypoint.z = 0.0
            
            self.waypoints.append(waypoint)
            
            if hasattr(self, 'target_waypoints_count') and self.target_waypoints_count > 0:
                self.get_logger().info(
                    f"Waypoint #{len(self.waypoints)}/{self.target_waypoints_count} added: ({waypoint.x:.2f}, {waypoint.y:.2f})"
                )
                if len(self.waypoints) >= self.target_waypoints_count:
                    self.collecting_waypoints = False
                    self.executing_waypoints = True
                    self.current_waypoint_index = 0
                    self.state_ = 0
                    reset_avoidance_state()
                    self.get_logger().info("=== ALL WAYPOINTS COLLECTED ===")
                    self.get_logger().info(f"Starting execution of {len(self.waypoints)} waypoints...")
            else:
                self.get_logger().info(
                    f"Waypoint #{len(self.waypoints)} added: ({waypoint.x:.2f}, {waypoint.y:.2f})"
                )
                
    def start_collection_rviz(self, num_points):
        self.target_waypoints_count = num_points
        self.waypoints = []
        self.collecting_waypoints = True
        self.executing_waypoints = False
        self.current_waypoint_index = 0
        self.get_logger().info(f"=== READY TO COLLECT {num_points} WAYPOINTS ===")
        self.get_logger().info("Please set goals in RViz using [2D Nav Goal]...")
        
    def start_collection_callback(self, request, response):
        """Start collecting waypoints"""
        self.waypoints = []
        self.collecting_waypoints = True
        self.executing_waypoints = False
        
        response.success = True
        response.message = "Started collecting waypoints. Set goals in RViz."
        self.get_logger().info("=== WAYPOINT COLLECTION STARTED ===")
        return response
        
    def execute_waypoints_callback(self, request, response):
        """Start executing collected waypoints"""
        if len(self.waypoints) == 0:
            response.success = False
            response.message = "No waypoints to execute"
            return response
        
        self.collecting_waypoints = False
        self.executing_waypoints = True
        self.current_waypoint_index = 0
        self.state_ = 0
        reset_avoidance_state()
        
        response.success = True
        response.message = f"Executing {len(self.waypoints)} waypoints"
        self.get_logger().info("=== STARTING WAYPOINT EXECUTION ===")
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
        
        return response
        
    def clear_waypoints_callback(self, request, response):
        """Clear all waypoints"""
        self.waypoints = []
        self.collecting_waypoints = False
        self.executing_waypoints = False
        
        response.success = True
        response.message = "Waypoints cleared"
        self.get_logger().info("Waypoints cleared")
        return response
        
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

    def normalize_angle(self, angle):
        """Keep angle between -π and π"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def navigation_loop(self):
        """Main navigation loop"""
        if not self.executing_waypoints:
            return
            
        # Try to update pose from TF first
        tf_pose = self.get_current_pose()
        if tf_pose is not None:
            self.position_, self.yaw_ = tf_pose
            
        if self.current_waypoint_index >= len(self.waypoints):
            # All waypoints completed
            stop()
            self.get_logger().info("=== ALL WAYPOINTS COMPLETED ===")
            self.executing_waypoints = False
            return
        
        # Get current waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Check for stuck
        self.check_if_stuck()
        
        if self.regions and (is_obstacle_detected(self.regions) or is_currently_avoiding()):
            obstacle_cmd = avoid_obstacle(self.regions)
            if obstacle_cmd:
                self.cmd_pub.publish(obstacle_cmd)
                return
        
        # ===== STEP 1: RECALCULATE navigation parameters =====
        dx = current_waypoint.x - self.position_.x
        dy = current_waypoint.y - self.position_.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        heading_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(heading_to_goal - self.yaw_)
        
        # ===== STEP 2: CHECK if waypoint reached =====
        GOAL_TOLERANCE = 0.05
        YAW_TOLERANCE = 0.087
        
        if distance_to_goal <= GOAL_TOLERANCE:
            # Current waypoint reached
            stop()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached"
            )
            
            # Move to next waypoint
            self.current_waypoint_index += 1
            reset_avoidance_state()
            
            if self.current_waypoint_index < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(
                    f"Navigating to waypoint {self.current_waypoint_index + 1}: "
                    f"({next_wp.x:.2f}, {next_wp.y:.2f})"
                )
            return
        
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

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointRViz()
    
    # Spin node in background thread so callbacks are processed in real-time
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Wait for initial sensor data
    from time import sleep
    sleep(1)
    
    try:
        while rclpy.ok():
            print("\n===========================================")
            print("MULTI-POINT RViz NAVIGATION")
            print("===========================================")
            try:
                num_points_input = input("Enter the number of waypoints to set from RViz: ")
                num_points = int(num_points_input.strip())
                if num_points <= 0:
                    print("Please enter a positive integer.")
                    continue
            except ValueError:
                print("Invalid input. Please enter an integer.")
                continue
                
            node.start_collection_rviz(num_points)
            
            # Wait until execution completes (or collection)
            # collecting_waypoints or executing_waypoints is active during operation
            while rclpy.ok() and (node.collecting_waypoints or node.executing_waypoints):
                sleep(1)
                
            print("")
            cont = input("Plan another multi-point route? (y/n): ")
            if cont.lower() != 'y':
                print("\nExiting multi-point navigation.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()