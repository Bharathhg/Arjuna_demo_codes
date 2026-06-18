#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

# Constants
OBSTACLE_DIST_THRESHOLD = 0.5
SAFE_DISTANCE = 0.7

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Regions dictionary
        self.regions = {}
        
        # Avoidance state
        self.avoiding = False
        self.avoidance_type = None
        self.start_time = 0.0
        self.min_avoidance_time = 2.0
        
        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.get_logger().info("Obstacle Avoidance Node Started")
        self.get_logger().info(f"Threshold: {OBSTACLE_DIST_THRESHOLD}m")
        
    def laser_callback(self, msg):
        """Process LIDAR safely and take action"""
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
        
        self.take_action()

        
    def is_path_clear(self):
        """Check if path is completely clear"""
        return (self.regions['front_L'] > SAFE_DISTANCE and 
                self.regions['front_R'] > SAFE_DISTANCE and
                self.regions['fleft'] > SAFE_DISTANCE and
                self.regions['fright'] > SAFE_DISTANCE)
    
    def take_action(self):
        """Decide action based on laser scan with persistent avoidance"""
        msg = Twist()
        current_time = time.time()
        
        # Check obstacles
        front_obstacle = (self.regions['front_L'] < OBSTACLE_DIST_THRESHOLD or 
                         self.regions['front_R'] < OBSTACLE_DIST_THRESHOLD)
        left_obstacle = (self.regions['fleft'] < OBSTACLE_DIST_THRESHOLD or 
                        self.regions['left'] < OBSTACLE_DIST_THRESHOLD)
        right_obstacle = (self.regions['fright'] < OBSTACLE_DIST_THRESHOLD or 
                         self.regions['right'] < OBSTACLE_DIST_THRESHOLD)
        
        # Continue current avoidance if active
        if self.avoiding:
            elapsed = current_time - self.start_time
            
            if elapsed < self.min_avoidance_time or not self.is_path_clear():
                # Continue current maneuver
                if self.avoidance_type == 'back_up':
                    state = 'CONTINUING: Backing up'
                    msg.linear.x = -0.1
                    msg.angular.z = 0.0
                elif self.avoidance_type == 'turn_right':
                    state = 'CONTINUING: Turning right'
                    msg.linear.x = 0.0
                    msg.angular.z = -0.8
                elif self.avoidance_type == 'turn_left':
                    state = 'CONTINUING: Turning left'
                    msg.linear.x = 0.0
                    msg.angular.z = 0.8
                elif self.avoidance_type == 'adjust_right':
                    state = 'CONTINUING: Adjusting right'
                    msg.linear.x = 0.1
                    msg.angular.z = -0.4
                elif self.avoidance_type == 'adjust_left':
                    state = 'CONTINUING: Adjusting left'
                    msg.linear.x = 0.1
                    msg.angular.z = 0.4
                
                self.get_logger().info(state)
                self.pub.publish(msg)
                return
            else:
                # Avoidance complete
                self.get_logger().info("AVOIDANCE COMPLETE - Path clear")
                self.avoiding = False
                self.avoidance_type = None
        
        # Start new avoidance or normal navigation
        if front_obstacle and left_obstacle and right_obstacle:
            state = 'NEW AVOIDANCE: Surrounded'
            msg.linear.x = -0.1
            msg.angular.z = 0.0
            self.avoiding = True
            self.avoidance_type = 'back_up'
            self.start_time = current_time
            self.min_avoidance_time = 3.0
            
        elif front_obstacle and left_obstacle:
            state = 'NEW AVOIDANCE: Front and left'
            msg.linear.x = 0.0
            msg.angular.z = -0.8
            self.avoiding = True
            self.avoidance_type = 'turn_right'
            self.start_time = current_time
            self.min_avoidance_time = 2.0
            
        elif front_obstacle and right_obstacle:
            state = 'NEW AVOIDANCE: Front and right'
            msg.linear.x = 0.0
            msg.angular.z = 0.8
            self.avoiding = True
            self.avoidance_type = 'turn_left'
            self.start_time = current_time
            self.min_avoidance_time = 2.0
            
        elif front_obstacle:
            if self.regions['left'] > self.regions['right']:
                state = 'NEW AVOIDANCE: Front - turn left'
                msg.angular.z = 0.8
                self.avoidance_type = 'turn_left'
            else:
                state = 'NEW AVOIDANCE: Front - turn right'
                msg.angular.z = -0.8
                self.avoidance_type = 'turn_right'
            msg.linear.x = 0.0
            self.avoiding = True
            self.start_time = current_time
            self.min_avoidance_time = 2.0
            
        elif left_obstacle:
            state = 'NEW AVOIDANCE: Left obstacle'
            msg.linear.x = 0.15
            msg.angular.z = -0.4
            self.avoiding = True
            self.avoidance_type = 'adjust_right'
            self.start_time = current_time
            self.min_avoidance_time = 1.5
            
        elif right_obstacle:
            state = 'NEW AVOIDANCE: Right obstacle'
            msg.linear.x = 0.15
            msg.angular.z = 0.4
            self.avoiding = True
            self.avoidance_type = 'adjust_left'
            self.start_time = current_time
            self.min_avoidance_time = 1.5
            
        else:
            state = 'Clear path - moving forward'
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        
        self.get_logger().info(state)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()