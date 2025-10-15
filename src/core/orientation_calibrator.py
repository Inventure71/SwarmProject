#!/usr/bin/env python3
"""
Orientation calibration for handling coordinate system mismatch between
OptiTrack (UDP tracking) and robot's internal coordinate system.

The calibration process:
1. Record robot's starting position from OptiTrack
2. Command robot to move "straight forward" in its own coordinate frame
3. Record robot's ending position from OptiTrack
4. Calculate the actual movement vector in OptiTrack coordinates
5. Compute rotation offset between robot frame and OptiTrack frame
6. Apply this offset to all subsequent movement commands
"""

import math
import time
import json
from typing import Tuple, Optional
from pathlib import Path


class OrientationCalibrator:
    """Handles orientation calibration between OptiTrack and robot coordinate systems."""
    
    def __init__(self, calibration_distance: float = 0.5, settling_time: float = 2.0):
        """
        Initialize the calibrator.
        
        Args:
            calibration_distance: Distance to move during calibration (meters)
            settling_time: Time to wait for robot to complete movement (seconds)
        """
        self.calibration_distance = calibration_distance
        self.settling_time = settling_time
        self.orientation_offset = 0.0  # Rotation offset in radians
        self.is_calibrated = False
    
    def calibrate(self, robot, verbose: bool = True) -> float:
        """
        Perform orientation calibration.
        
        Args:
            robot: Robot instance with position tracking and movement control
            verbose: Print calibration progress
            
        Returns:
            Orientation offset in radians
        """
        if verbose:
            print("\n" + "=" * 60)
            print("ORIENTATION CALIBRATION")
            print("=" * 60)
            print("This will calibrate the orientation offset between OptiTrack")
            print("and the robot's internal coordinate system.")
            print(f"The robot will move {self.calibration_distance}m forward.")
            print("=" * 60)
        
        # Wait a moment for system to stabilize
        time.sleep(0.5)
        
        # Step 1: Record starting position
        start_x, start_y, start_yaw = robot.get_position()
        if verbose:
            print(f"\nStep 1: Starting position")
            print(f"  Position: ({start_x:.4f}, {start_y:.4f})")
            print(f"  Yaw: {start_yaw:.4f} rad ({math.degrees(start_yaw):.2f}°)")
        
        # Step 2: Command robot to move straight in its own frame
        # In robot frame: move forward means dx=calibration_distance, dy=0
        if verbose:
            print(f"\nStep 2: Commanding robot to move {self.calibration_distance}m forward")
            print("  Robot frame command: dx={:.3f}, dy=0.000".format(self.calibration_distance))
        
        robot.move_by_vector(self.calibration_distance, 0.0)
        
        # Wait for movement to complete
        if verbose:
            print(f"  Waiting {self.settling_time}s for movement to complete...")
        time.sleep(self.settling_time)
        
        # Step 3: Record ending position
        end_x, end_y, end_yaw = robot.get_position()
        if verbose:
            print(f"\nStep 3: Ending position")
            print(f"  Position: ({end_x:.4f}, {end_y:.4f})")
            print(f"  Yaw: {end_yaw:.4f} rad ({math.degrees(end_yaw):.2f}°)")
        
        # Step 4: Calculate actual movement vector in OptiTrack coordinates
        actual_dx = end_x - start_x
        actual_dy = end_y - start_y
        actual_distance = math.hypot(actual_dx, actual_dy)
        
        if verbose:
            print(f"\nStep 4: Actual movement in OptiTrack frame")
            print(f"  Vector: dx={actual_dx:.4f}, dy={actual_dy:.4f}")
            print(f"  Distance: {actual_distance:.4f}m")
        
        # Check if robot actually moved
        if actual_distance < self.calibration_distance * 0.3:
            raise RuntimeError(
                f"Robot did not move sufficiently during calibration.\n"
                f"Expected ~{self.calibration_distance}m, got {actual_distance:.4f}m.\n"
                f"Check robot connection and movement system."
            )
        
        # Step 5: Calculate orientation offset
        # The robot thinks it moved along (1, 0) in its frame
        # But actually moved along (actual_dx, actual_dy) in OptiTrack frame
        # The angle of actual movement is the orientation offset
        actual_heading = math.atan2(actual_dy, actual_dx)
        
        # The offset is the difference between what the robot thinks (0 radians = forward)
        # and what OptiTrack observed
        self.orientation_offset = actual_heading
        self.is_calibrated = True
        
        if verbose:
            print(f"\nStep 5: Calibration complete!")
            print(f"  Orientation offset: {self.orientation_offset:.4f} rad ({math.degrees(self.orientation_offset):.2f}°)")
            print(f"\nInterpretation:")
            print(f"  When robot thinks it's moving 'forward' (0°),")
            print(f"  it actually moves at {math.degrees(self.orientation_offset):.2f}° in OptiTrack coordinates.")
            print(f"\nAll subsequent movement commands will be rotated by this offset.")
            print("=" * 60 + "\n")
        
        return self.orientation_offset
    
    def transform_vector(self, dx: float, dy: float) -> Tuple[float, float]:
        """
        Transform a movement vector from OptiTrack frame to robot frame.
        
        This rotates the desired movement vector (in OptiTrack coordinates)
        by the negative of the orientation offset, so that when the robot
        executes it in its own frame, it will move in the correct direction
        in OptiTrack coordinates.
        
        Args:
            dx: Desired X displacement in OptiTrack frame
            dy: Desired Y displacement in OptiTrack frame
            
        Returns:
            (dx_robot, dy_robot): Vector in robot's frame
        """
        if not self.is_calibrated:
            # If not calibrated, pass through without transformation
            return (dx, dy)
        
        # Rotate by negative offset to convert from OptiTrack to robot frame
        cos_theta = math.cos(-self.orientation_offset)
        sin_theta = math.sin(-self.orientation_offset)
        
        dx_robot = dx * cos_theta - dy * sin_theta
        dy_robot = dx * sin_theta + dy * cos_theta
        
        return (dx_robot, dy_robot)
    
    def save_calibration(self, filepath: str) -> None:
        """Save calibration to file."""
        data = {
            'orientation_offset': self.orientation_offset,
            'is_calibrated': self.is_calibrated,
            'calibration_distance': self.calibration_distance
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def load_calibration(self, filepath: str) -> bool:
        """
        Load calibration from file.
        
        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            self.orientation_offset = data['orientation_offset']
            self.is_calibrated = data['is_calibrated']
            return True
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            return False
    
    def get_offset(self) -> float:
        """Get the current orientation offset in radians."""
        return self.orientation_offset
    
    def reset(self) -> None:
        """Reset calibration."""
        self.orientation_offset = 0.0
        self.is_calibrated = False


def quick_calibration_test():
    """Quick test function for calibration logic."""
    print("Testing orientation calibrator...")
    
    # Create mock robot for testing
    class MockRobot:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.commanded_dx = 0.0
            self.commanded_dy = 0.0
        
        def get_position(self):
            return (self.x, self.y, self.yaw)
        
        def move_by_vector(self, dx, dy):
            self.commanded_dx = dx
            self.commanded_dy = dy
            # Simulate robot moving at 45° offset from commanded direction
            offset = math.radians(45)
            actual_dx = dx * math.cos(offset) - dy * math.sin(offset)
            actual_dy = dx * math.sin(offset) + dy * math.cos(offset)
            self.x += actual_dx
            self.y += actual_dy
    
    robot = MockRobot()
    calibrator = OrientationCalibrator(calibration_distance=1.0, settling_time=0.1)
    
    # Perform calibration
    offset = calibrator.calibrate(robot, verbose=True)
    
    # Test transformation
    print("\nTesting vector transformation:")
    test_vector = (1.0, 0.0)  # Want to move 1m in X direction (OptiTrack)
    transformed = calibrator.transform_vector(*test_vector)
    print(f"  Input (OptiTrack frame): {test_vector}")
    print(f"  Output (Robot frame): ({transformed[0]:.4f}, {transformed[1]:.4f})")
    
    expected_angle = math.radians(45)
    print(f"  Expected offset: {expected_angle:.4f} rad (45°)")
    print(f"  Actual offset: {offset:.4f} rad ({math.degrees(offset):.2f}°)")


if __name__ == "__main__":
    quick_calibration_test()

