#!/usr/bin/env python3
"""
Manual calibration test script.

This script:
1. Performs orientation calibration
2. Lets you send movement commands via keyboard
3. Helps verify calibration is working correctly

Controls:
  W - Move forward (+Y in OptiTrack)
  S - Move backward (-Y in OptiTrack)
  A - Move left (-X in OptiTrack)
  D - Move right (+X in OptiTrack)
  Q - Diagonal forward-left
  E - Diagonal forward-right
  Z - Diagonal backward-left
  C - Diagonal backward-right
  X - Stop/Exit
"""

import sys
import time
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).resolve().parent))

from src.core.orientation_calibrator import OrientationCalibrator
from src.tracking.robot_config import RobotManager


def print_header():
    """Print script header."""
    print("\n" + "="*70)
    print("  MANUAL CALIBRATION TEST")
    print("="*70)
    print("\nThis script will:")
    print("  1. Run orientation calibration")
    print("  2. Let you send movement commands with keyboard")
    print("  3. Help verify calibration is working correctly")
    print("\n" + "="*70 + "\n")


def print_controls():
    """Print control instructions."""
    print("\n" + "-"*70)
    print("CONTROLS (all movements are in OptiTrack coordinates):")
    print("-"*70)
    print("  W - Move forward (+Y direction)")
    print("  S - Move backward (-Y direction)")
    print("  A - Move left (-X direction)")
    print("  D - Move right (+X direction)")
    print("  Q - Diagonal forward-left")
    print("  E - Diagonal forward-right")
    print("  Z - Diagonal backward-left")
    print("  C - Diagonal backward-right")
    print("  X - Stop and exit")
    print("-"*70 + "\n")


def main():
    """Main test function."""
    print_header()
    
    # Get robot name
    robot_name = input("Enter robot name (e.g., umh_2): ").strip()
    if not robot_name:
        print("No robot name provided. Exiting.")
        return 1
    
    # Get movement distance
    try:
        distance_str = input("Enter movement distance in meters (default: 0.3): ").strip()
        distance = float(distance_str) if distance_str else 0.3
    except ValueError:
        print("Invalid distance. Using default: 0.3m")
        distance = 0.3
    
    print(f"\nUsing robot: {robot_name}")
    print(f"Movement distance: {distance}m")
    
    # Create robot
    print("\nInitializing robot...")
    robot_manager = RobotManager()
    
    available = robot_manager.get_available_robots()
    if robot_name not in available:
        print(f"ERROR: Robot '{robot_name}' not found in config.")
        print(f"Available robots: {', '.join(available)}")
        return 1
    
    robot = robot_manager.create_robot(robot_name)
    print("Robot initialized. Waiting for OptiTrack data...")
    time.sleep(2.0)
    
    # Show initial position
    x, y, yaw = robot.get_position()
    print(f"Current position: ({x:.3f}, {y:.3f}), yaw: {yaw:.3f} rad")
    
    # Run calibration
    print("\n" + "="*70)
    input("Press ENTER to start calibration...")
    print("="*70)
    
    calibrator = OrientationCalibrator(
        calibration_distance=0.5,
        settling_time=3.0
    )
    
    try:
        offset = calibrator.calibrate(robot, verbose=True)
        robot.set_orientation_calibration(offset, enabled=True)
        
        # Save calibration
        calib_file = f".calibration_{robot_name}.json"
        calibrator.save_calibration(calib_file)
        print(f"\nCalibration saved to {calib_file}")
        
    except Exception as e:
        print(f"\nCALIBRATION FAILED: {e}")
        use_anyway = input("Continue without calibration? (y/n): ").strip().lower()
        if use_anyway not in ['y', 'yes']:
            return 1
    
    # Interactive control loop
    print("\n" + "="*70)
    print("  MANUAL CONTROL MODE")
    print("="*70)
    print_controls()
    
    print("You can now send movement commands.")
    print("The robot should move in the OPTITRACK coordinate directions.\n")
    
    # Command mapping: key -> (dx, dy, description)
    commands = {
        'w': (0, distance, "Forward (+Y)"),
        's': (0, -distance, "Backward (-Y)"),
        'a': (-distance, 0, "Left (-X)"),
        'd': (distance, 0, "Right (+X)"),
        'q': (-distance*0.707, distance*0.707, "Forward-Left diagonal"),
        'e': (distance*0.707, distance*0.707, "Forward-Right diagonal"),
        'z': (-distance*0.707, -distance*0.707, "Backward-Left diagonal"),
        'c': (distance*0.707, -distance*0.707, "Backward-Right diagonal"),
    }
    
    try:
        while True:
            # Get command
            cmd = input("\nCommand (W/S/A/D/Q/E/Z/C or X to exit): ").strip().lower()
            
            if cmd == 'x':
                print("\nExiting...")
                break
            
            if cmd == 'h' or cmd == '?':
                print_controls()
                continue
            
            if cmd not in commands:
                print(f"Unknown command: '{cmd}'. Press H for help.")
                continue
            
            # Get current position
            x_before, y_before, yaw_before = robot.get_position()
            
            # Send command
            dx, dy, description = commands[cmd]
            print(f"\nSending: {description}")
            print(f"  OptiTrack coordinates: dx={dx:+.3f}, dy={dy:+.3f}")
            
            # Show what robot will receive (if calibrated)
            if robot.use_orientation_calibration:
                dx_robot, dy_robot = robot._transform_vector(dx, dy)
                print(f"  Robot coordinates: dx={dx_robot:+.3f}, dy={dy_robot:+.3f}")
            
            robot.move_by_vector(dx, dy)
            
            # Wait for movement
            print("  Waiting for movement...")
            time.sleep(2.5)
            
            # Show result
            x_after, y_after, yaw_after = robot.get_position()
            actual_dx = x_after - x_before
            actual_dy = y_after - y_before
            
            print(f"\n  Position before: ({x_before:.3f}, {y_before:.3f})")
            print(f"  Position after:  ({x_after:.3f}, {y_after:.3f})")
            print(f"  Actual movement: dx={actual_dx:+.3f}, dy={actual_dy:+.3f}")
            
            # Calculate error
            error_x = actual_dx - dx
            error_y = actual_dy - dy
            import math
            error_magnitude = math.hypot(error_x, error_y)
            
            print(f"  Error: {error_magnitude:.3f}m")
            
            if error_magnitude < 0.05:
                print("  ✓ Good accuracy!")
            elif error_magnitude < 0.10:
                print("  ⚠ Moderate accuracy")
            else:
                print("  ✗ Large error - calibration may need adjustment")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        robot.stop()
        robot_manager.stop()
        print("Done!")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

