#!/usr/bin/env python3

import math

class ZigZagPlanner:
    def __init__(self, logger):
        self.logger = logger
        self.lane_spacing = 0.40  # 40cm sweep spacing (width of sweep)

    def plan(self, map_msg, start_pose, goal_pose):
        """
        Generates a systematic zig-zag coverage path spanning from start to goal coordinates.
        The bounding box corners are defined by start and goal.
        """
        self.logger.info("Zig-Zag Coverage Planning started...")
        
        # Bounding box limits
        xs = start_pose.position.x
        ys = start_pose.position.y
        xg = goal_pose.position.x
        yg = goal_pose.position.y
        
        min_x = min(xs, xg)
        max_x = max(xs, xg)
        min_y = min(ys, yg)
        max_y = max(ys, yg)
        
        # Ensure we have a minimum rectangle area
        if abs(max_x - min_x) < 0.2:
            max_x += 0.5
        if abs(max_y - min_y) < 0.2:
            max_y += 0.5
            
        self.logger.info(f"Coverage bounds: X[{min_x:.2f} to {max_x:.2f}], Y[{min_y:.2f} to {max_y:.2f}]")
        
        # Calculate number of lanes
        width = max_x - min_x
        num_lanes = int(math.ceil(width / self.lane_spacing))
        num_lanes = max(2, num_lanes)
        
        grid_width = map_msg.info.width
        grid_height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        # Helper to check if a point is in collision
        def is_occupied(wx, wy):
            gx = int((wx - origin_x) / resolution)
            gy = int((wy - origin_y) / resolution)
            if not (0 <= gx < grid_width and 0 <= gy < grid_height):
                return True
            index = gy * grid_width + gx
            occupancy = map_msg.data[index]
            return occupancy > 50 or occupancy == -1
            
        grid_path = []
        
        # Generate sweep path (parallel to Y axis, moving along X)
        for i in range(num_lanes):
            x = min_x + i * self.lane_spacing
            
            # Divide each lane into discrete waypoints along the Y axis
            lane_len = max_y - min_y
            num_points_in_lane = int(math.ceil(lane_len / 0.2)) # waypoint every 20cm
            num_points_in_lane = max(2, num_points_in_lane)
            
            y_points = [min_y + j * (lane_len / (num_points_in_lane - 1)) for j in range(num_points_in_lane)]
            
            # Alternate directions for zig-zag pattern
            if i % 2 == 1:
                y_points.reverse()
                
            for y in y_points:
                # Only add waypoint if it is collision-free
                if not is_occupied(x, y):
                    grid_path.append((x, y))
                    
        # Convert path to geometry_msgs/Point list
        from geometry_msgs.msg import Point
        waypoints = []
        for x, y in grid_path:
            wp = Point()
            wp.x = x
            wp.y = y
            wp.z = 0.0
            waypoints.append(wp)
            
        self.logger.info(f"Zig-Zag Planner generated {len(waypoints)} collision-free waypoints.")
        return waypoints
