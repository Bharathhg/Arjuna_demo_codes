#!/usr/bin/env python3

import math
import heapq

class AStarPlanner:
    def __init__(self, logger):
        self.logger = logger

    def plan(self, map_msg, start_pose, goal_pose):
        """
        Plans a path on the occupancy grid map using the A* algorithm.
        
        :param map_msg: nav_msgs/msg/OccupancyGrid
        :param start_pose: geometry_msgs/msg/Pose (or Point)
        :param goal_pose: geometry_msgs/msg/Pose (or Point)
        :return: list of geometry_msgs/msg/Point waypoints
        """
        self.logger.info("A* Path Planning started...")
        
        # Extract map metadata
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # Convert start/goal world coordinates (meters) to grid coordinates
        start_x = int((start_pose.position.x - origin_x) / resolution)
        start_y = int((start_pose.position.y - origin_y) / resolution)
        goal_x = int((goal_pose.position.x - origin_x) / resolution)
        goal_y = int((goal_pose.position.y - origin_y) / resolution)
        
        # Clamp start/goal to map boundaries
        start_x = max(0, min(width - 1, start_x))
        start_y = max(0, min(height - 1, start_y))
        goal_x = max(0, min(width - 1, goal_x))
        goal_y = max(0, min(height - 1, goal_y))
        
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        
        self.logger.info(f"Start grid: {start}, Goal grid: {goal}")
        
        # 8-connectivity directions (dx, dy, move_cost)
        directions = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0), # orthogonal
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)) # diagonal
        ]
        
        # Priority queue stores: (f_score, g_score, (x, y))
        open_set = []
        heapq.heappush(open_set, (0.0 + self.heuristic(start, goal), 0.0, start))
        
        came_from = {}
        g_score = {start: 0.0}
        
        closed_set = set()
        
        found = False
        while open_set:
            current_f, current_g, current = heapq.heappop(open_set)
            
            if current == goal:
                found = True
                break
                
            if current in closed_set:
                continue
                
            closed_set.add(current)
            
            cx, cy = current
            for dx, dy, cost in directions:
                neighbor = (cx + dx, cy + dy)
                nx, ny = neighbor
                
                # Check bounds
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                    
                # Check obstacle (occupancy > 50 is occupied)
                index = ny * width + nx
                occupancy = map_msg.data[index]
                if occupancy > 50 or occupancy == -1: # Treat unknown as obstacle for safety
                    continue
                    
                tentative_g = current_g + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                    
        if not found:
            self.logger.warn("A* failed to find a valid path!")
            return []
            
        # Reconstruct path in grid coordinates
        grid_path = []
        curr = goal
        while curr != start:
            grid_path.append(curr)
            curr = came_from[curr]
        grid_path.append(start)
        grid_path.reverse()
        
        self.logger.info(f"Path found with {len(grid_path)} grid nodes.")
        
        # Convert path from grid coordinates back to world Coordinates (geometry_msgs/Point)
        from geometry_msgs.msg import Point
        waypoints = []
        for x, y in grid_path:
            wp = Point()
            wp.x = x * resolution + origin_x + (resolution / 2.0)
            wp.y = y * resolution + origin_y + (resolution / 2.0)
            wp.z = 0.0
            waypoints.append(wp)
            
        return waypoints

    def heuristic(self, p1, p2):
        """Euclidean distance heuristic"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
