#!/usr/bin/env python3

import math
import time

class Bug1Planner:
    def __init__(self, logger):
        self.logger = logger
        self.state = 'GO_TO_GOAL'  # 'GO_TO_GOAL', 'CIRCUMNAVIGATE', 'GO_TO_CLOSEST'
        
        # State tracking variables
        self.start_pos = None
        self.closest_pos = None
        self.min_dist_to_goal = float('inf')
        self.circumnavigated_started = False
        
        # Avoid loop latching
        self.last_depart_time = 0.0
        
        # Parameters
        self.obs_threshold = 0.4
        self.clear_threshold = 0.55
        self.goal_tolerance = 0.1
        self.yaw_tolerance = 0.15
        self.linear_vel = 0.25
        self.angular_vel = 0.6
        self.loop_closed_threshold = 0.25
        self.leave_start_dist = 0.35

    def reset(self):
        self.state = 'GO_TO_GOAL'
        self.start_pos = None
        self.closest_pos = None
        self.min_dist_to_goal = float('inf')
        self.circumnavigated_started = False
        self.logger.info("Bug 1 state fully reset")

    def compute_velocity(self, position, yaw, goal_position, regions):
        """
        Computes velocity commands using the Bug 1 algorithm.
        """
        dx = goal_position.x - position.x
        dy = goal_position.y - position.y
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if dist_to_goal < self.goal_tolerance:
            self.logger.info("Goal reached!")
            return 0.0, 0.0, True
            
        goal_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(goal_yaw - yaw)
        
        # Obstacle detection (only active if not in immediate depart cooldown)
        cooldown_active = (time.time() - self.last_depart_time) < 3.0
        front_blocked = (regions.get('front_L', 10.0) < self.obs_threshold or 
                         regions.get('front_R', 10.0) < self.obs_threshold)
        
        # State transitions
        if self.state == 'GO_TO_GOAL':
            if front_blocked and not cooldown_active:
                self.logger.info("Obstacle hit! Entering CIRCUMNAVIGATE mode.")
                self.state = 'CIRCUMNAVIGATE'
                self.start_pos = (position.x, position.y)
                self.closest_pos = (position.x, position.y)
                self.min_dist_to_goal = dist_to_goal
                self.circumnavigated_started = False
                
        elif self.state == 'CIRCUMNAVIGATE':
            # Check if we have left the start position first
            curr_start_dist = math.sqrt((position.x - self.start_pos[0])**2 + (position.y - self.start_pos[1])**2)
            if not self.circumnavigated_started and curr_start_dist > self.leave_start_dist:
                self.circumnavigated_started = True
                self.logger.info("Successfully moved away from start point, watching for loop closure...")
                
            # Track closest position to goal
            if dist_to_goal < self.min_dist_to_goal:
                self.min_dist_to_goal = dist_to_goal
                self.closest_pos = (position.x, position.y)
                
            # Check if we completed the loop (returned to start point)
            if self.circumnavigated_started and curr_start_dist < self.loop_closed_threshold:
                self.logger.info("Circumnavigation complete! Switching to GO_TO_CLOSEST.")
                self.state = 'GO_TO_CLOSEST'
                
        elif self.state == 'GO_TO_CLOSEST':
            # Check if we have reached the closest point to goal
            curr_closest_dist = math.sqrt((position.x - self.closest_pos[0])**2 + (position.y - self.closest_pos[1])**2)
            if curr_closest_dist < self.loop_closed_threshold:
                self.logger.info("Reached closest point to goal! Heading to GO_TO_GOAL.")
                self.state = 'GO_TO_GOAL'
                self.last_depart_time = time.time()
                
        # Execute movement
        linear_x = 0.0
        angular_z = 0.0
        
        if self.state == 'GO_TO_GOAL':
            if abs(yaw_error) > self.yaw_tolerance:
                linear_x = 0.0
                angular_z = self.angular_vel if yaw_error > 0 else -self.angular_vel
            else:
                linear_x = self.linear_vel
                angular_z = 0.3 * yaw_error
                
        elif self.state in ['CIRCUMNAVIGATE', 'GO_TO_CLOSEST']:
            # For simplicity, follow left wall (turn right when blocked)
            front = min(regions.get('front_L', 10.0), regions.get('front_R', 10.0))
            fleft = regions.get('fleft', 10.0)
            left = regions.get('left', 10.0)
            
            if front < self.obs_threshold or fleft < self.obs_threshold:
                # Wall ahead/left-ahead, rotate right in place
                linear_x = 0.0
                angular_z = -self.angular_vel
            elif left > self.clear_threshold:
                # Wall is drifting away, turn left to follow it
                linear_x = self.linear_vel * 0.7
                angular_z = self.angular_vel * 0.5
            else:
                # Go straight along wall
                linear_x = self.linear_vel
                angular_z = 0.0
                
        return linear_x, angular_z, False

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
