#!/usr/bin/env python3

import math
import random

class RRTStarPlanner:
    def __init__(self, logger):
        self.logger = logger
        self.max_iter = 400
        self.step_size = 0.3
        self.search_radius = 0.6
        self.goal_sample_rate = 0.15

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None
            self.cost = 0.0

    def plan(self, map_msg, start_pose, goal_pose):
        """
        Plans a path using RRT* algorithm.
        """
        self.logger.info("RRT* Path Planning started...")
        
        # Map parameters
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        start = self.Node(start_pose.position.x, start_pose.position.y)
        goal = self.Node(goal_pose.position.x, goal_pose.position.y)
        
        # Calculate boundaries in world coordinates
        min_x = origin_x
        max_x = origin_x + width * resolution
        min_y = origin_y
        max_y = origin_y + height * resolution
        
        node_list = [start]
        
        for i in range(self.max_iter):
            # Sample a random point
            if random.random() < self.goal_sample_rate:
                rnd_node = self.Node(goal.x, goal.y)
            else:
                rnd_node = self.Node(
                    random.uniform(min_x, max_x),
                    random.uniform(min_y, max_y)
                )
                
            # Find nearest node
            nearest_idx = self.get_nearest_node_index(node_list, rnd_node)
            nearest_node = node_list[nearest_idx]
            
            # Step from nearest toward random
            new_node = self.step(nearest_node, rnd_node)
            
            # Check collision
            if self.is_collision(nearest_node, new_node, map_msg):
                continue
                
            # Find all nodes in neighborhood (within search_radius)
            near_indices = self.find_near_nodes(node_list, new_node)
            
            # Choose best parent (minimum path cost to start)
            new_node = self.choose_parent(node_list, near_indices, nearest_node, new_node, map_msg)
            
            if new_node is None:
                continue
                
            node_list.append(new_node)
            
            # Rewire the neighborhood
            self.rewire(node_list, near_indices, new_node, map_msg)
            
            # Check if we can directly reach the goal
            if self.distance(new_node, goal) <= self.step_size:
                if not self.is_collision(new_node, goal, map_msg):
                    goal.parent = new_node
                    goal.cost = new_node.cost + self.distance(new_node, goal)
                    node_list.append(goal)
                    break
                    
        # Reconstruct path
        if goal.parent is None:
            # Look for the node closest to the goal in the tree
            best_idx = self.get_nearest_node_index(node_list, goal)
            closest_node = node_list[best_idx]
            if self.distance(closest_node, goal) < 0.8:
                goal.parent = closest_node
                goal.cost = closest_node.cost + self.distance(closest_node, goal)
                node_list.append(goal)
            else:
                self.logger.warn("RRT* failed to find a valid path!")
                return []
                
        # Build list of waypoints (geometry_msgs/Point)
        from geometry_msgs.msg import Point
        waypoints = []
        curr = goal
        while curr is not None:
            wp = Point()
            wp.x = curr.x
            wp.y = curr.y
            wp.z = 0.0
            waypoints.append(wp)
            curr = curr.parent
            
        waypoints.reverse()
        self.logger.info(f"RRT* Path found with {len(waypoints)} waypoints.")
        return waypoints

    def step(self, from_node, to_node):
        """Step toward to_node from from_node by step_size"""
        d = self.distance(from_node, to_node)
        if d <= self.step_size:
            return self.Node(to_node.x, to_node.y)
        else:
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            return self.Node(
                from_node.x + self.step_size * math.cos(theta),
                from_node.y + self.step_size * math.sin(theta)
            )

    def is_collision(self, n1, n2, map_msg):
        """Line collision checking between n1 and n2 using occupancy grid"""
        # Interpolate points along the segment
        steps = int(self.distance(n1, n2) / (map_msg.info.resolution / 2.0))
        steps = max(2, steps)
        
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        for step in range(steps + 1):
            t = step / steps
            x = n1.x + t * (n2.x - n1.x)
            y = n1.y + t * (n2.y - n1.y)
            
            grid_x = int((x - origin_x) / resolution)
            grid_y = int((y - origin_y) / resolution)
            
            if not (0 <= grid_x < width and 0 <= grid_y < height):
                return True
                
            index = grid_y * width + grid_x
            occupancy = map_msg.data[index]
            if occupancy > 50 or occupancy == -1:
                return True
                
        return False

    def find_near_nodes(self, node_list, new_node):
        near_indices = []
        for idx, node in enumerate(node_list):
            if self.distance(node, new_node) <= self.search_radius:
                near_indices.append(idx)
        return near_indices

    def choose_parent(self, node_list, near_indices, nearest_node, new_node, map_msg):
        if not near_indices:
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)
            return new_node
            
        min_cost = float('inf')
        best_parent = None
        
        for idx in near_indices:
            near_node = node_list[idx]
            cost = near_node.cost + self.distance(near_node, new_node)
            if cost < min_cost:
                if not self.is_collision(near_node, new_node, map_msg):
                    min_cost = cost
                    best_parent = near_node
                    
        if best_parent is None:
            # Fallback to nearest if collision free
            if not self.is_collision(nearest_node, new_node, map_msg):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)
                return new_node
            return None
            
        new_node.parent = best_parent
        new_node.cost = min_cost
        return new_node

    def rewire(self, node_list, near_indices, new_node, map_msg):
        for idx in near_indices:
            near_node = node_list[idx]
            new_cost = new_node.cost + self.distance(new_node, near_node)
            if new_cost < near_node.cost:
                if not self.is_collision(new_node, near_node, map_msg):
                    near_node.parent = new_node
                    near_node.cost = new_cost

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [self.distance(node, rnd_node) for node in node_list]
        return dlist.index(min(dlist))

    def distance(self, n1, n2):
        return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
