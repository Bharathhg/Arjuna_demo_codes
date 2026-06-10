#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import math
import sys
import os
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Add Newrro_NavLib and Algorithms to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'Newrro_NavLib'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'Algorithms'))

from Newrro_Navigation import (
    fix_yaw, go_straight_ahead, stop, set_cmd_publisher
)

# Import our custom planners
from astar import AStarPlanner
from dijkstra import DijkstraPlanner
from bug0 import Bug0Planner
from bug1 import Bug1Planner
from bug2 import Bug2Planner
from apf import APFPlanner
from rrt_star import RRTStarPlanner
from zigzag import ZigZagPlanner

class ArjunaAlgorithmSelector(Node):
    def __init__(self, algo_choice):
        super().__init__('arjuna_algorithm_selector')
        
        self.algo_choice = algo_choice
        
        # State variables
        self.current_position = Point()
        self.current_yaw = 0.0
        self.goal_position = Point()
        self.has_goal = False
        
        # LIDAR regions
        self.regions = {}
        
        # Occupancy Grid Map
        self.map_msg = None
        
        # Waypoints for path planners (A*, Dijkstra)
        self.waypoints = []
        self.current_wp_idx = 0
        self.wp_state = 0 # 0: rotate, 1: straight, 2: reached
        
        # Instantiate the chosen algorithm planner
        self.planner = None
        if algo_choice == 1:
            self.planner = AStarPlanner(self.get_logger())
            self.get_logger().info("Using A* Path Planning Algorithm")
        elif algo_choice == 2:
            self.planner = DijkstraPlanner(self.get_logger())
            self.get_logger().info("Using Dijkstra's Path Planning Algorithm")
        elif algo_choice == 3:
            self.planner = Bug0Planner(self.get_logger())
            self.get_logger().info("Using Bug 0 Reactive Navigation Algorithm")
        elif algo_choice == 4:
            self.planner = Bug1Planner(self.get_logger())
            self.get_logger().info("Using Bug 1 Reactive Navigation Algorithm")
        elif algo_choice == 5:
            self.planner = Bug2Planner(self.get_logger())
            self.get_logger().info("Using Bug 2 Reactive Navigation Algorithm")
        elif algo_choice == 6:
            self.planner = APFPlanner(self.get_logger())
            self.get_logger().info("Using Artificial Potential Field (APF) Algorithm")
        elif algo_choice == 7:
            self.planner = RRTStarPlanner(self.get_logger())
            self.get_logger().info("Using RRT* Path Planning Algorithm")
        elif algo_choice == 8:
            self.planner = ZigZagPlanner(self.get_logger())
            self.get_logger().info("Using Zig-Zag Coverage Path Planning Algorithm")
            
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        set_cmd_publisher(self.cmd_pub)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.odom_sub_combined = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/robot_pose_ekf/odom_combined',
            self.odom_combined_callback, 
            10)
        
        self.odom_sub_raw = self.create_subscription(
            Odometry, 
            '/odom',
            self.odom_raw_callback, 
            10)
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # Transient Local QoS profile for map subscriber
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=1,
            depth=1
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos)
            
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
            
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
            
        # TF listener for drift-corrected localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        # 20 Hz control loop timer
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Arjuna Algorithm Selector Node Initialized.")
        self.get_logger().info("Waiting for goal coordinates from RViz...")

    # ========== CALLBACKS ==========

    def odom_combined_callback(self, msg):
        self.update_pose(msg.pose.pose)
        
    def initial_pose_callback(self, msg):
        self.update_pose(msg.pose.pose)
        self.get_logger().info(f"Initial pose reset from RViz: ({self.current_position.x:.2f}, {self.current_position.y:.2f})")
        
    def odom_raw_callback(self, msg):
        self.update_pose(msg.pose.pose)
        
    def update_pose(self, pose):
        self.current_position.x = pose.position.x
        self.current_position.y = pose.position.y
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def laser_callback(self, msg):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
            
        # Process LIDAR into 6 symmetric regions
        def get_region_min(start_frac, end_frac):
            start_idx = int(num_ranges * start_frac)
            end_idx = int(num_ranges * end_frac)
            if start_idx >= end_idx or end_idx > num_ranges:
                return 10.0
            subset = msg.ranges[start_idx:end_idx]
            valid = [r for r in subset if 0.05 < r < 10.0]
            return min(valid) if valid else 10.0
            
        self.regions = {
            'front_L': get_region_min(0.0, 0.15),
            'fleft':   get_region_min(0.15, 0.27),
            'left':    get_region_min(0.27, 0.33),
            'right':   get_region_min(0.67, 0.73),
            'fright':  get_region_min(0.73, 0.85),
            'front_R': get_region_min(0.85, 1.0)
        }

    def map_callback(self, msg):
        self.map_msg = msg

    def goal_callback(self, msg):
        self.goal_position.x = msg.pose.position.x
        self.goal_position.y = msg.pose.position.y
        self.get_logger().info(f"New goal received: ({self.goal_position.x:.2f}, {self.goal_position.y:.2f})")
        
        # Grid-based/Path planners (A*, Dijkstra, RRT*, Zig-Zag)
        if self.algo_choice in [1, 2, 7, 8]:
            if self.map_msg is None:
                self.get_logger().warn("Map not received yet! Cannot plan path.")
                return
            
            # Create start pose object for planner
            start_pose = PoseStamped().pose
            start_pose.position = self.current_position
            
            # Execute path planner
            self.waypoints = self.planner.plan(self.map_msg, start_pose, msg.pose)
            
            if len(self.waypoints) == 0:
                self.get_logger().error("Path planning failed. No waypoints generated.")
                self.has_goal = False
            else:
                self.current_wp_idx = 0
                self.wp_state = 0
                self.has_goal = True
                self.get_logger().info(f"Path generated with {len(self.waypoints)} waypoints. Starting execution.")
                
                # Publish planned path for RViz visualization
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for wp in self.waypoints:
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = wp.x
                    pose.pose.position.y = wp.y
                    pose.pose.position.z = wp.z
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
                self.path_pub.publish(path_msg)
                
        # Reactive planners (Bug 0, Bug 1, Bug 2, APF)
        else:
            self.planner.reset()
            self.has_goal = True

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

    # ========== CONTROL LOOP ==========

    def control_loop(self):
        if not self.has_goal:
            return
            
        # Try to update pose from TF first
        tf_pose = self.get_current_pose()
        if tf_pose is not None:
            self.current_position, self.current_yaw = tf_pose
            
        # Check sensor regions are loaded for reactive algorithms
        if self.algo_choice in [3, 4, 5, 6] and not self.regions:
            return

        # 1. Grid/Path Plan Waypoints Following (A*, Dijkstra, RRT*, Zig-Zag)
        if self.algo_choice in [1, 2, 7, 8]:
            if self.current_wp_idx >= len(self.waypoints):
                stop()
                self.get_logger().info("All waypoints completed! Goal reached.")
                self.has_goal = False
                return
                
            current_wp = self.waypoints[self.current_wp_idx]
            
            # Execute standard waypoints tracking state machine
            if self.wp_state == 0:
                self.current_yaw, self.current_position, self.wp_state = fix_yaw(
                    current_wp, self.current_yaw, self.current_position, self.wp_state
                )
            elif self.wp_state == 1:
                self.current_yaw, self.current_position, self.wp_state = go_straight_ahead(
                    current_wp, self.current_yaw, self.current_position, self.wp_state
                )
            elif self.wp_state == 2:
                # Current waypoint reached, move to next
                self.get_logger().info(f"Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} reached.")
                self.current_wp_idx += 1
                self.wp_state = 0
                stop()
                
        # 2. Reactive Navigation (Bug 0, Bug 1, Bug 2, APF)
        else:
            linear_x, angular_z, reached = self.planner.compute_velocity(
                self.current_position, self.current_yaw, self.goal_position, self.regions
            )
            
            if reached:
                stop()
                self.get_logger().info("Goal reached successfully!")
                self.has_goal = False
                return
                
            # Publish velocity command
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.cmd_pub.publish(twist)


def main(args=None):
    # Print welcome menu and read algorithm choice
    print("\n" + "=" * 50)
    print("      ARJUNA MULTI-ALGORITHM NAVIGATION SELECTOR")
    print("=" * 50)
    print("Choose one of the following navigation algorithms:")
    print("  [1] A* Grid Path Planning")
    print("  [2] Dijkstra's Grid Path Planning")
    print("  [3] Bug 0 Reactive Wall-Following")
    print("  [4] Bug 1 Circumnavigation Wall-Following")
    print("  [5] Bug 2 M-Line Intersection Wall-Following")
    print("  [6] Artificial Potential Fields (APF)")
    print("  [7] RRT* Path Planning")
    print("  [8] Zig-Zag Coverage Path Planning")
    print("=" * 50)
    
    choice = 0
    while choice not in [1, 2, 3, 4, 5, 6, 7, 8]:
        try:
            choice = int(input("Select algorithm (1-8): "))
            if choice not in [1, 2, 3, 4, 5, 6, 7, 8]:
                print("Invalid option. Please choose a number between 1 and 8.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            
    print("\nStarting ROS 2 selector node...")
    
    rclpy.init(args=args)
    node = ArjunaAlgorithmSelector(choice)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down algorithm selector...")
        stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
