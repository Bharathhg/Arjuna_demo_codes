#!/usr/bin/env python3

import math
import time

class Bug2Planner:
    def __init__(self, logger):
        self.logger = logger
        self.state = 'GO_TO_GOAL'  # 'GO_TO_GOAL', 'WALL_FOLLOW'
        
        # M-line definition (Ax + By + C = 0)
        self.start_pos = None
        self.m_line_A = 0.0
        self.m_line_B = 0.0
        self.m_line_C = 0.0
        
        # State tracking
        self.hit_pos = None
        self.hit_dist_to_goal = float('inf')
        self.left_mline = False
        
        # Cooldown
        self.last_depart_time = 0.0
        
        # Parameters
        self.obs_threshold = 0.4
        self.clear_threshold = 0.55
        self.goal_tolerance = 0.1
        self.yaw_tolerance = 0.15
        self.linear_vel = 0.25
        self.angular_vel = 0.6
        self.m_line_tolerance = 0.12  # max distance to be considered "on the m-line"
        self.leave_hit_dist = 0.35

    def reset(self):
        self.state = 'GO_TO_GOAL'
        self.start_pos = None
        self.hit_pos = None
        self.hit_dist_to_goal = float('inf')
        self.left_mline = False
        self.logger.info("Bug 2 state reset")

    def compute_velocity(self, position, yaw, goal_position, regions):
        """
        Computes velocity commands using the Bug 2 algorithm.
        """
        dx = goal_position.x - position.x
        dy = goal_position.y - position.y
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if dist_to_goal < self.goal_tolerance:
            self.logger.info("Goal reached!")
            return 0.0, 0.0, True
            
        goal_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(goal_yaw - yaw)
        
        # Initialize M-Line if not yet defined
        if self.start_pos is None:
            self.start_pos = (position.x, position.y)
            # Line equation from start to goal
            self.m_line_A = goal_position.y - self.start_pos[1]
            self.m_line_B = self.start_pos[0] - goal_position.x
            self.m_line_C = goal_position.x * self.start_pos[1] - goal_position.y * self.start_pos[0]
            self.logger.info(f"M-Line initialized: {self.m_line_A:.2f}x + {self.m_line_B:.2f}y + {self.m_line_C:.2f} = 0")
            
        # Obstacle detection
        cooldown_active = (time.time() - self.last_depart_time) < 3.0
        front_blocked = (regions.get('front_L', 10.0) < self.obs_threshold or 
                         regions.get('front_R', 10.0) < self.obs_threshold)
                         
        # State transitions
        if self.state == 'GO_TO_GOAL':
            if front_blocked and not cooldown_active:
                self.logger.info("Obstacle hit! Switching to WALL_FOLLOW mode.")
                self.state = 'WALL_FOLLOW'
                self.hit_pos = (position.x, position.y)
                self.hit_dist_to_goal = dist_to_goal
                self.left_mline = False
                
        elif self.state == 'WALL_FOLLOW':
            # Check if we have left the hit point
            curr_hit_dist = math.sqrt((position.x - self.hit_pos[0])**2 + (position.y - self.hit_pos[1])**2)
            if not self.left_mline and curr_hit_dist > self.leave_hit_dist:
                self.left_mline = True
                self.logger.info("Moved away from hit point, watching for M-Line crossing...")
                
            # Check if we are crossing the M-line again
            # Distance from point to line: |Ax + By + C| / sqrt(A^2 + B^2)
            m_line_denom = math.sqrt(self.m_line_A**2 + self.m_line_B**2)
            dist_to_m_line = abs(self.m_line_A * position.x + self.m_line_B * position.y + self.m_line_C) / m_line_denom
            
            # If we are on the m-line, and we are closer to the goal than the hit point
            if self.left_mline and dist_to_m_line < self.m_line_tolerance:
                if dist_to_goal < self.hit_dist_to_goal - 0.2:
                    self.logger.info("M-Line crossed closer to the goal! Returning to GO_TO_GOAL mode.")
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
                
        elif self.state == 'WALL_FOLLOW':
            # Follow left wall
            front = min(regions.get('front_L', 10.0), regions.get('front_R', 10.0))
            fleft = regions.get('fleft', 10.0)
            left = regions.get('left', 10.0)
            
            if front < self.obs_threshold or fleft < self.obs_threshold:
                linear_x = 0.0
                angular_z = -self.angular_vel
            elif left > self.clear_threshold:
                linear_x = self.linear_vel * 0.7
                angular_z = self.angular_vel * 0.5
            else:
                linear_x = self.linear_vel
                angular_z = 0.0
                
        return linear_x, angular_z, False

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
