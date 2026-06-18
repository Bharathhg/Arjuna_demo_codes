#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import select
import termios
import tty
import time
import threading

# Import the Feetech SDK from motor_ops
from motor_ops.STservo_sdk import *

class InteractiveCalibrator(Node):
    def __init__(self):
        super().__init__('interactive_calibrator')
        
        # Serial communication settings - same as ticks_pub.py
        self.BAUDRATE = 115200
        self.DEVICENAME = '/dev/ttyUSB2'  
        self.MOTOR_SPEED = 200  # Precise jog speed
        self.MOTOR_ACCL = 0
        
        # State variables
        self.selector_r = 0
        self.selector_l = 0
        self.Left_ticks = 0
        self.Right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        
        self.Left_velocity = 0
        self.Right_velocity = 0
        
        # Initialize serial communication
        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = sts(self.port_handler)
        
        print(f"Opening serial port {self.DEVICENAME}...")
        if not self.port_handler.openPort():
            print("ERROR: Failed to open port. Check connection / permissions.")
            raise RuntimeError("Failed to open port")
        
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            print("ERROR: Failed to set baudrate.")
            raise RuntimeError("Failed to set baudrate")
            
        print("Serial port opened successfully!")
        
        # Enable wheel mode for all motors
        self.enable_wheel_mode()
        
        # Start background encoder tracking thread (10Hz)
        self.tracking = True
        self.tracking_thread = threading.Thread(target=self.tracking_loop, daemon=True)
        self.tracking_thread.start()

    def wheel_mode(self, motor_id):
        result, error = self.packet_handler.WheelMode(motor_id)
        if result != COMM_SUCCESS:
            print(f"WARN: WheelMode error on ID {motor_id}: {self.packet_handler.getTxRxResult(result)}")

    def enable_wheel_mode(self):
        for motor_id in [1, 2, 3, 4]:
            self.wheel_mode(motor_id)
        print("Enabled wheel mode for motors 1, 2, 3, 4.")

    def run_motor(self, motor_id, motor_speed, motor_accel):
        result, error = self.packet_handler.WriteSpec(motor_id, motor_speed, motor_accel)
        if result != COMM_SUCCESS:
            pass

    def present_pos(self, motor_id):
        position, speed, result, error = self.packet_handler.ReadPosSpeed(motor_id)
        if result != COMM_SUCCESS:
            return None
        return position

    def stop_robot(self):
        self.run_motor(1, 0, self.MOTOR_ACCL)
        self.run_motor(4, 0, self.MOTOR_ACCL)
        self.run_motor(2, 0, self.MOTOR_ACCL)
        self.run_motor(3, 0, self.MOTOR_ACCL)

    def pulse_left_wheel(self, forward=True):
        speed = self.MOTOR_SPEED if forward else -self.MOTOR_SPEED
        self.Left_velocity = speed
        self.run_motor(1, -speed, self.MOTOR_ACCL)
        self.run_motor(4, -speed, self.MOTOR_ACCL)
        time.sleep(0.08)
        self.Left_velocity = 0
        self.run_motor(1, 0, self.MOTOR_ACCL)
        self.run_motor(4, 0, self.MOTOR_ACCL)

    def pulse_right_wheel(self, forward=True):
        speed = self.MOTOR_SPEED if forward else -self.MOTOR_SPEED
        self.Right_velocity = speed
        self.run_motor(2, speed, self.MOTOR_ACCL)
        self.run_motor(3, speed, self.MOTOR_ACCL)
        time.sleep(0.08)
        self.Right_velocity = 0
        self.run_motor(2, 0, self.MOTOR_ACCL)
        self.run_motor(3, 0, self.MOTOR_ACCL)

    def tracking_loop(self):
        """Background loop to update left and right ticks at 10Hz (same as ticks_pub.py)"""
        while self.tracking:
            self.update_left_ticks()
            self.update_right_ticks()
            time.sleep(0.1)

    def update_left_ticks(self):
        pos = self.present_pos(1)
        if pos is None:
            return
            
        self.Left_ticks = int(pos / 2.5)
        
        if self.Left_velocity > 0:
            self.selector_l = 1
        elif self.Left_velocity < 0:
            self.selector_l = 2
            
        if self.prev_left_ticks != 0:
            tick_diff = self.Left_ticks - self.prev_left_ticks
            
            if tick_diff > 16000:
                tick_diff = tick_diff - 32768
            elif tick_diff < -16000:
                tick_diff = tick_diff + 32768
                
            if self.selector_l == 1 and tick_diff < 0:
                self.total_left_ticks += abs(tick_diff)
            elif self.selector_l == 1 and tick_diff > 0:
                self.total_left_ticks += tick_diff
            elif self.selector_l == 2 and tick_diff > 0:
                self.total_left_ticks -= tick_diff
            elif self.selector_l == 2 and tick_diff < 0:
                self.total_left_ticks += tick_diff
                
        self.prev_left_ticks = self.Left_ticks

    def update_right_ticks(self):
        pos = self.present_pos(2)
        if pos is None:
            return
            
        self.Right_ticks = int(pos / 2.5)
        
        if self.Right_velocity > 0:
            self.selector_r = 1
        elif self.Right_velocity < 0:
            self.selector_r = 2
            
        if self.prev_right_ticks != 0:
            tick_diff = self.Right_ticks - self.prev_right_ticks
            
            if tick_diff > 16000:
                tick_diff = tick_diff - 32768
            elif tick_diff < -16000:
                tick_diff = tick_diff + 32768
                
            if self.selector_r == 1 and tick_diff > 0:
                self.total_right_ticks += tick_diff
            elif self.selector_r == 1 and tick_diff < 0:
                self.total_right_ticks += abs(tick_diff)
            elif self.selector_r == 2 and tick_diff < 0:
                self.total_right_ticks += tick_diff
            elif self.selector_r == 2 and tick_diff > 0:
                self.total_right_ticks -= tick_diff
                
        self.prev_right_ticks = self.Right_ticks

    def shutdown(self):
        self.tracking = False
        self.stop_robot()
        self.port_handler.closePort()
        print("Port closed.")


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def run_calibration_menu(node):
    print("\n=============================================")
    print("  INTERACTIVE ODOMETRY CALIBRATION WIZARD   ")
    print("=============================================\n")
    
    try:
        trials_input = input("Enter number of trials to run (e.g. 10 or 20): ")
        num_trials = int(trials_input.strip())
        if num_trials <= 0:
            print("Please enter a positive integer.")
            return
    except ValueError:
        print("Invalid input. Defaulting to 10 trials.")
        num_trials = 10
        
    print(f"\nWe will run {num_trials} trials.")
    print("Key Map for Jogging:")
    print("  [7] Left Wheel Forward    |  [9] Right Wheel Forward")
    print("  [1] Left Wheel Backward   |  [3] Right Wheel Backward")
    print("  [5] Save trial & Next     |  [q] Quit calibration")
    
    left_trials = []
    right_trials = []
    
    settings = termios.tcgetattr(sys.stdin)
    
    for trial in range(1, num_trials + 1):
        print(f"\n--- TRIAL #{trial}/{num_trials} ---")
        print("1. Align the starting mark on the wheel.")
        print("2. Jog using [7]/[1] or [9]/[3] until wheel completes EXACTLY 1 full turn (360°).")
        print("3. Press [5] to save this trial.")
        
        # Save start ticks
        start_left = node.total_left_ticks
        start_right = node.total_right_ticks
        
        while True:
            key = get_key(settings)
            
            if key == '7':
                node.pulse_left_wheel(forward=True)
            elif key == '1':
                node.pulse_left_wheel(forward=False)
            elif key == '9':
                node.pulse_right_wheel(forward=True)
            elif key == '3':
                node.pulse_right_wheel(forward=False)
            elif key == '5':
                end_left = node.total_left_ticks
                end_right = node.total_right_ticks
                
                diff_left = abs(end_left - start_left)
                diff_right = abs(end_right - start_right)
                
                if diff_left > diff_right:
                    left_trials.append(diff_left)
                    print(f"\n-> Trial #{trial} Saved: Left Wheel turned {diff_left} ticks.")
                else:
                    right_trials.append(diff_right)
                    print(f"\n-> Trial #{trial} Saved: Right Wheel turned {diff_right} ticks.")
                break
            elif key == 'q':
                print("\nCalibration cancelled by user.")
                return
            
            # Print current ticks change in real-time
            dl = abs(node.total_left_ticks - start_left)
            dr = abs(node.total_right_ticks - start_right)
            sys.stdout.write(f"\rCurrent Diff -> Left: {dl:5d} ticks | Right: {dr:5d} ticks")
            sys.stdout.flush()
            
            time.sleep(0.01)
            
    # Calculate averages
    print("\n\n=============================================")
    print("             CALIBRATION RESULTS             ")
    print("=============================================")
    
    avg_left = sum(left_trials) / len(left_trials) if left_trials else 0
    avg_right = sum(right_trials) / len(right_trials) if right_trials else 0
    
    print(f"Total Left Wheel Trials  : {len(left_trials)}")
    if left_trials:
        print(f"  Trials Ticks : {left_trials}")
        print(f"  Average Ticks: {avg_left:.1f}")
        
    print(f"Total Right Wheel Trials : {len(right_trials)}")
    if right_trials:
        print(f"  Trials Ticks : {right_trials}")
        print(f"  Average Ticks: {avg_right:.1f}")
        
    # Calculate target ticks per meter (based on 4cm radius wheel)
    r = 0.04
    circ = 2 * 3.1415926535 * r
    
    print("\nRECOMMENDED CALIBRATION PARAMETERS:")
    if avg_left > 0:
        tpm_left = avg_left / circ
        print(f"  If using Left Wheel : TICKS_PER_METER = {tpm_left:.2f}")
    if avg_right > 0:
        tpm_right = avg_right / circ
        print(f"  If using Right Wheel: TICKS_PER_METER = {tpm_right:.2f}")
        
    if avg_left > 0 and avg_right > 0:
        avg_combined = (avg_left + avg_right) / 2.0
        tpm_comb = avg_combined / circ
        print(f"  Combined Average     : TICKS_PER_METER = {tpm_comb:.2f}")
        
    print("\nOpen 'ekf_data_pub.cpp' and update the TICKS_PER_METER parameter.")
    print("=============================================\n")


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCalibrator()
    
    # Spin node in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        run_calibration_menu(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
