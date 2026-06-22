#!/usr/bin/env python3

import math

class APFPlanner:
    def __init__(self, logger):
        self.logger = logger
        
        # APF Gains & Parameters
        self.k_att = 1.2        # Attraction gain
        self.k_rep = 0.15       # Repulsion gain
        self.d_0 = 1.0          # Obstacle influence distance (1 meter)
        
        # Constraints
        self.goal_tolerance = 0.1
        self.max_linear_vel = 0.3
        self.max_angular_vel = 0.7
        
        # Approximate angles for each sector in radians
        self.sector_angles = {
            'front_L': math.radians(15),
            'front_R': math.radians(-15),
            'fleft': math.radians(45),
            'fright': math.radians(-45),
            'left': math.radians(90),
            'right': math.radians(-90)
        }

    def reset(self):
        self.logger.info("APF Planner reset")

    def compute_velocity(self, position, yaw, goal_position, regions):
        """
        Computes velocity commands using Artificial Potential Fields.
        """
        # Calculate goal distance and heading
        dx = goal_position.x - position.x
        dy = goal_position.y - position.y
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if dist_to_goal < self.goal_tolerance:
            self.logger.info("Goal reached!")
            return 0.0, 0.0, True
            
        # 1. Attraction Force (Global Frame)
        # Force grows with distance (clamped to avoid infinite forces)
        att_mag = min(dist_to_goal, 2.0)
        goal_yaw = math.atan2(dy, dx)
        att_x_global = self.k_att * att_mag * math.cos(goal_yaw)
        att_y_global = self.k_att * att_mag * math.sin(goal_yaw)
        
        # Transform Attraction Force to Robot's Local Frame
        # Rotation matrix: [cos -sin; sin cos] inverted is [cos sin; -sin cos]
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        att_x_local = att_x_global * cos_y + att_y_global * sin_y
        att_y_local = -att_x_global * sin_y + att_y_global * cos_y
        
        # 2. Repulsion Forces (Robot's Local Frame)
        rep_x_local = 0.0
        rep_y_local = 0.0
        
        for sector, angle in self.sector_angles.items():
            dist = regions.get(sector, 10.0)
            
            # Repulsion only active within influence range
            if dist < self.d_0:
                # Basic APF repulsion formula: F = k_rep * (1/d - 1/d0) * (1/d^2)
                rep_mag = self.k_rep * (1.0 / dist - 1.0 / self.d_0) * (1.0 / (dist * dist))
                
                # Force direction is opposite to the obstacle angle
                rep_x_local += -rep_mag * math.cos(angle)
                rep_y_local += -rep_mag * math.sin(angle)
                
        # 3. Sum Potential Field Forces
        force_x = att_x_local + rep_x_local
        force_y = att_y_local + rep_y_local
        
        # Map force components to linear and angular velocities
        # force_x drives forward/backward speed
        linear_x = force_x
        # force_y drives steering/angular speed (turn towards net force vector)
        angular_z = 1.5 * force_y
        
        # Clamp velocities to limits
        linear_x = max(-0.1, min(linear_x, self.max_linear_vel)) # limit backup speed
        angular_z = max(-self.max_angular_vel, min(angular_z, self.max_angular_vel))
        
        # If there's a strong repulsion ahead, slow down or stop forward speed
        front_dist = min(regions.get('front_L', 10.0), regions.get('front_R', 10.0))
        if front_dist < 0.35 and linear_x > 0.0:
            linear_x = 0.0
            
        return linear_x, angular_z, False
