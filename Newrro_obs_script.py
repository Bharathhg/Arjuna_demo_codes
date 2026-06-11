#!/usr/bin/env python3
"""
Company Name : NEWRRO TECH
ROS Version  : ROS 2
Obstacle avoidance functions with persistent two-stage avoidance to prevent oscillations.
"""

from geometry_msgs.msg import Twist
import time

# Navigation parameters
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.8
OBSTACLE_DIST_THRESHOLD = 0.35  # Keep consistent with nodes (35cm)
SAFE_DISTANCE = 0.5            # Distance considered safe after avoidance

# Track avoidance state
avoidance_state = {
    'avoiding': False,
    'avoidance_type': None,     # 'back_up', 'turn_right', 'turn_left', 'adjust_right', 'adjust_left'
    'start_time': 0.0,
    'stage_start_time': 0.0,
    'stage': 1,                 # 1: Turn/Rotate/Back up, 2: Move forward to clear obstacle
    'min_avoidance_time': 2.0   # Default duration
}

def is_currently_avoiding():
    """Check if the robot is currently executing an avoidance maneuver"""
    return avoidance_state['avoiding']

def is_path_clear(regions):
    """Check if front path is clear of obstacles"""
    if not regions:
        return True
    
    # Path is clear only if all critical front regions are safe
    return (regions['front_L'] > SAFE_DISTANCE and 
            regions['front_R'] > SAFE_DISTANCE and
            regions['fleft'] > SAFE_DISTANCE and
            regions['fright'] > SAFE_DISTANCE)

def is_obstacle_detected(regions):
    """Check if any obstacle is within threshold (front and front-diagonal only for starting new avoidance)"""
    if not regions:
        return False
    
    # We prioritize front sectors to start avoidance to avoid locking on purely side obstacles
    return (regions['front_L'] < OBSTACLE_DIST_THRESHOLD or 
            regions['front_R'] < OBSTACLE_DIST_THRESHOLD or
            regions['fleft'] < OBSTACLE_DIST_THRESHOLD or
            regions['fright'] < OBSTACLE_DIST_THRESHOLD or
            regions['left'] < 0.20 or # extremely close on side
            regions['right'] < 0.20)

def avoid_obstacle(regions):
    """
    Generate avoidance command with persistent behavior.
    Stage 1: Turn/rotate until front is clear.
    Stage 2: Move forward to clear the obstacle.
    """
    global avoidance_state
    
    twist_msg = Twist()
    current_time = time.time()
    
    # Check obstacle configurations
    front_obstacle = (regions['front_L'] < OBSTACLE_DIST_THRESHOLD or 
                     regions['front_R'] < OBSTACLE_DIST_THRESHOLD)
    left_obstacle = (regions['fleft'] < OBSTACLE_DIST_THRESHOLD or 
                    regions['left'] < OBSTACLE_DIST_THRESHOLD)
    right_obstacle = (regions['fright'] < OBSTACLE_DIST_THRESHOLD or 
                     regions['right'] < OBSTACLE_DIST_THRESHOLD)
    
    # If already avoiding, process current state and stage transitions
    if avoidance_state['avoiding']:
        elapsed_in_stage = current_time - avoidance_state['stage_start_time']
        maneuver = avoidance_state['avoidance_type']
        
        # ── STAGE 1: Turning or Backing up ──
        if avoidance_state['stage'] == 1:
            if maneuver == 'back_up':
                if elapsed_in_stage >= 2.0:
                    # Back up complete, now choose a direction to turn
                    print("Back up complete - choosing turn direction")
                    avoidance_state['stage'] = 1
                    avoidance_state['stage_start_time'] = current_time
                    if regions['left'] > regions['right']:
                        avoidance_state['avoidance_type'] = 'turn_left'
                    else:
                        avoidance_state['avoidance_type'] = 'turn_right'
                else:
                    twist_msg.linear.x = -0.1
                    twist_msg.angular.z = 0.0
                    return twist_msg
                    
            elif maneuver in ['turn_left', 'turn_right']:
                # Transition to Stage 2 (Clearance) when front is clear AND min rotation time has passed
                if elapsed_in_stage >= 0.8 and is_path_clear(regions):
                    print("Front clear - transitioning to Stage 2: Moving forward to clear obstacle")
                    avoidance_state['stage'] = 2
                    avoidance_state['stage_start_time'] = current_time
                    avoidance_state['min_avoidance_time'] = 2.0
                else:
                    # Continue rotation
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = ANGULAR_VELOCITY if maneuver == 'turn_left' else -ANGULAR_VELOCITY
                    return twist_msg
                    
            elif maneuver in ['adjust_left', 'adjust_right']:
                if elapsed_in_stage >= 1.5 and is_path_clear(regions):
                    # Adjustment complete
                    print("Side adjustment complete")
                    avoidance_state['avoiding'] = False
                    avoidance_state['avoidance_type'] = None
                    return None
                else:
                    twist_msg.linear.x = 0.15
                    twist_msg.angular.z = ANGULAR_VELOCITY * 0.5 if maneuver == 'adjust_left' else -ANGULAR_VELOCITY * 0.5
                    return twist_msg
                    
        # ── STAGE 2: Translate forward to clear obstacle ──
        elif avoidance_state['stage'] == 2:
            # Check if a new obstacle is detected directly in front during clearance
            if regions['front_L'] < OBSTACLE_DIST_THRESHOLD or regions['front_R'] < OBSTACLE_DIST_THRESHOLD:
                print("New front obstacle detected during clearance! Resetting to evaluate new maneuver.")
                avoidance_state['avoiding'] = False
                avoidance_state['avoidance_type'] = None
                # Let next block handle it
            elif elapsed_in_stage < avoidance_state['min_avoidance_time']:
                # Move straight forward
                twist_msg.linear.x = 0.25
                twist_msg.angular.z = 0.0
                return twist_msg
            else:
                # Clearance duration complete!
                print("Clearance complete - resuming normal navigation")
                avoidance_state['avoiding'] = False
                avoidance_state['avoidance_type'] = None
                return None
                
    # ── START NEW AVOIDANCE MANEUVER ──
    if front_obstacle and left_obstacle and right_obstacle:
        print("NEW AVOIDANCE: Surrounded - backing up")
        twist_msg.linear.x = -0.1
        twist_msg.angular.z = 0.0
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'back_up'
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    elif front_obstacle and left_obstacle:
        print("NEW AVOIDANCE: Front and left blocked - turning right")
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -ANGULAR_VELOCITY
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'turn_right'
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    elif front_obstacle and right_obstacle:
        print("NEW AVOIDANCE: Front and right blocked - turning left")
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = ANGULAR_VELOCITY
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'turn_left'
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    elif front_obstacle:
        if regions['left'] > regions['right']:
            print("NEW AVOIDANCE: Front blocked - turning left (more space)")
            twist_msg.angular.z = ANGULAR_VELOCITY
            avoidance_state['avoidance_type'] = 'turn_left'
        else:
            print("NEW AVOIDANCE: Front blocked - turning right (more space)")
            twist_msg.angular.z = -ANGULAR_VELOCITY
            avoidance_state['avoidance_type'] = 'turn_right'
        twist_msg.linear.x = 0.0
        avoidance_state['avoiding'] = True
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    elif left_obstacle:
        print("NEW AVOIDANCE: Left obstacle - adjusting right")
        twist_msg.linear.x = 0.15
        twist_msg.angular.z = -ANGULAR_VELOCITY * 0.5
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'adjust_right'
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    elif right_obstacle:
        print("NEW AVOIDANCE: Right obstacle - adjusting left")
        twist_msg.linear.x = 0.15
        twist_msg.angular.z = ANGULAR_VELOCITY * 0.5
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'adjust_left'
        avoidance_state['start_time'] = current_time
        avoidance_state['stage_start_time'] = current_time
        avoidance_state['stage'] = 1
        
    else:
        return None
        
    return twist_msg

def reset_avoidance_state():
    """Reset avoidance state - call when starting new navigation task"""
    global avoidance_state
    avoidance_state['avoiding'] = False
    avoidance_state['avoidance_type'] = None
    avoidance_state['start_time'] = 0.0
    avoidance_state['stage_start_time'] = 0.0
    avoidance_state['stage'] = 1