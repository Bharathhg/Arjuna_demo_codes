#!/usr/bin/env python3

import math

class Bug0Planner:
    def __init__(self, logger):
        self.logger = logger
        self.state = 'GO_TO_GOAL' # 'GO_TO_GOAL', 'WALL_FOLLOW'
        self.wall_follow_side = 'left' # Side to keep wall on: 'left' or 'right'
        
        # Parameters
        self.obs_threshold = 0.4
        self.clear_threshold = 0.55
        self.goal_tolerance = 0.1
        self.yaw_tolerance = 0.15  # ~8 degrees
        self.linear_vel = 0.25
        self.angular_vel = 0.6

    def reset(self):
        self.state = 'GO_TO_GOAL'
        self.logger.info("Bug 0 state reset to GO_TO_GOAL")

    def compute_velocity(self, position, yaw, goal_position, regions):
        """
        Computes velocity commands using the Bug 0 algorithm.
        
        :param position: geometry_msgs/msg/Point (current)
        :param yaw: float (current yaw in radians)
        :param goal_position: geometry_msgs/msg/Point (goal)
        :param regions: dict of LIDAR ranges
        :return: (linear_x, angular_z, reached)
        """
        # Calculate distance to goal
        dx = goal_position.x - position.x
        dy = goal_position.y - position.y
        dist_to_goal = math.sqrt(dx*dx + dy*dy)
        
        if dist_to_goal < self.goal_tolerance:
            self.logger.info("Goal reached!")
            return 0.0, 0.0, True
            
        # Calculate heading to goal
        goal_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(goal_yaw - yaw)
        
        # Check obstacle status
        front_blocked = (regions.get('front_L', 10.0) < self.obs_threshold or 
                         regions.get('front_R', 10.0) < self.obs_threshold)
        
        # 1. State transitions
        if self.state == 'GO_TO_GOAL':
            if front_blocked:
                self.logger.info("Obstacle detected! Switching to WALL_FOLLOW")
                self.state = 'WALL_FOLLOW'
                # Decide wall follow side based on which side is further from obstacle
                left_dist = regions.get('left', 10.0)
                right_dist = regions.get('right', 10.0)
                self.wall_follow_side = 'left' if left_dist > right_dist else 'right'
                self.logger.info(f"Will follow wall on the {self.wall_follow_side}")
                
        elif self.state == 'WALL_FOLLOW':
            # In Bug 0, if the path towards the goal is clear, we immediately leave the wall!
            # We check if the angle towards the goal points to a clear sector.
            goal_is_clear = self.check_goal_path_clear(yaw_error, regions)
            if goal_is_clear and not front_blocked:
                self.logger.info("Goal direction is clear! Returning to GO_TO_GOAL")
                self.state = 'GO_TO_GOAL'

        # 2. Execute velocities based on active state
        linear_x = 0.0
        angular_z = 0.0
        
        if self.state == 'GO_TO_GOAL':
            if abs(yaw_error) > self.yaw_tolerance:
                # Rotate to face goal
                linear_x = 0.0
                angular_z = self.angular_vel if yaw_error > 0 else -self.angular_vel
            else:
                # Move straight ahead
                linear_x = self.linear_vel
                angular_z = 0.3 * yaw_error # minor heading corrections
                
        elif self.state == 'WALL_FOLLOW':
            linear_x, angular_z = self.follow_wall(regions)
            
        return linear_x, angular_z, False

    def check_goal_path_clear(self, yaw_error, regions):
        """Returns True if the direction of the goal is clear of obstacles"""
        # Map yaw_error (heading difference to goal) to LIDAR sectors
        # If goal is in front
        if -0.2 < yaw_error < 0.2:
            return regions.get('front_L', 10.0) > self.clear_threshold and regions.get('front_R', 10.0) > self.clear_threshold
        # If goal is to the left
        elif 0.2 <= yaw_error < 1.0:
            return regions.get('fleft', 10.0) > self.clear_threshold
        elif 1.0 <= yaw_error < 2.0:
            return regions.get('left', 10.0) > self.clear_threshold
        # If goal is to the right
        elif -1.0 < yaw_error <= -0.2:
            return regions.get('fright', 10.0) > self.clear_threshold
        elif -2.0 < yaw_error <= -1.0:
            return regions.get('right', 10.0) > self.clear_threshold
        # Otherwise (goal is behind us)
        return True

    def follow_wall(self, regions):
        """Standard wall following controller"""
        linear_x = 0.0
        angular_z = 0.0
        
        # Get ranges
        front = min(regions.get('front_L', 10.0), regions.get('front_R', 10.0))
        fleft = regions.get('fleft', 10.0)
        fright = regions.get('fright', 10.0)
        left = regions.get('left', 10.0)
        right = regions.get('right', 10.0)
        
        if self.wall_follow_side == 'left':
            # Follow wall on the left
            if front < self.obs_threshold or fleft < self.obs_threshold:
                # Turn right away from wall
                linear_x = 0.0
                angular_z = -self.angular_vel
            elif left > self.clear_threshold:
                # Turn left toward the wall to follow it
                linear_x = self.linear_vel * 0.7
                angular_z = self.angular_vel * 0.5
            else:
                # Go straight
                linear_x = self.linear_vel
                angular_z = 0.0
        else:
            # Follow wall on the right
            if front < self.obs_threshold or fright < self.obs_threshold:
                # Turn left away from wall
                linear_x = 0.0
                angular_z = self.angular_vel
            elif right > self.clear_threshold:
                # Turn right toward the wall to follow it
                linear_x = self.linear_vel * 0.7
                angular_z = -self.angular_vel * 0.5
            else:
                # Go straight
                linear_x = self.linear_vel
                angular_z = 0.0
                
        return linear_x, angular_z

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
