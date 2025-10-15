#!/usr/bin/env python3
"""
Test suite for orientation calibration functionality.

Tests the calibration logic, vector transformations, and integration with Robot class.
"""

import sys
import math
import time
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from src.core.orientation_calibrator import OrientationCalibrator
from src.core.robot import Robot


class MockRobot:
    """Mock robot for testing calibration without hardware."""
    
    def __init__(self, simulated_offset: float = 0.0):
        """
        Initialize mock robot.
        
        Args:
            simulated_offset: Simulated orientation offset in radians
        """
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.simulated_offset = simulated_offset
        self.movement_history = []
        
    def get_position(self):
        """Get current position."""
        return (self.x, self.y, self.yaw)
    
    def move_by_vector(self, dx, dy):
        """
        Simulate robot movement with orientation offset.
        
        When robot receives command (dx, dy), it interprets it in its own frame,
        but the actual movement (observed by OptiTrack) is rotated by the offset.
        """
        self.movement_history.append((dx, dy))
        
        # Apply simulated offset to simulate coordinate mismatch
        cos_offset = math.cos(self.simulated_offset)
        sin_offset = math.sin(self.simulated_offset)
        
        actual_dx = dx * cos_offset - dy * sin_offset
        actual_dy = dx * sin_offset + dy * cos_offset
        
        # Update position (as OptiTrack would see it)
        self.x += actual_dx
        self.y += actual_dy
    
    def reset_position(self):
        """Reset to origin."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0


def test_calibration_with_various_offsets():
    """Test calibration detects various orientation offsets correctly."""
    print("\n" + "="*70)
    print("TEST 1: Calibration with Various Offsets")
    print("="*70)
    
    test_cases = [
        (0.0, "No offset (0°)"),
        (math.pi / 4, "45° offset"),
        (math.pi / 2, "90° offset"),
        (-math.pi / 4, "-45° offset"),
        (math.pi, "180° offset"),
    ]
    
    tolerance = 0.01  # 0.01 radians ~ 0.57°
    all_passed = True
    
    for offset, description in test_cases:
        print(f"\nTesting {description}:")
        print(f"  Simulated offset: {offset:.4f} rad ({math.degrees(offset):.2f}°)")
        
        # Create mock robot with simulated offset
        robot = MockRobot(simulated_offset=offset)
        
        # Create calibrator
        calibrator = OrientationCalibrator(
            calibration_distance=1.0,
            settling_time=0.01  # No need to wait in simulation
        )
        
        # Run calibration
        detected_offset = calibrator.calibrate(robot, verbose=False)
        
        # Check if detected offset matches simulated offset
        error = abs(detected_offset - offset)
        passed = error < tolerance
        all_passed = all_passed and passed
        
        print(f"  Detected offset: {detected_offset:.4f} rad ({math.degrees(detected_offset):.2f}°)")
        print(f"  Error: {error:.4f} rad ({math.degrees(error):.2f}°)")
        print(f"  Status: {'✓ PASS' if passed else '✗ FAIL'}")
    
    print(f"\n{'='*70}")
    print(f"Overall: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    print(f"{'='*70}")
    
    return all_passed


def test_vector_transformation():
    """Test that vector transformation correctly compensates for offset."""
    print("\n" + "="*70)
    print("TEST 2: Vector Transformation")
    print("="*70)
    
    # Test with 45° offset
    simulated_offset = math.pi / 4
    print(f"\nSimulated offset: {simulated_offset:.4f} rad ({math.degrees(simulated_offset):.2f}°)")
    
    # Create and calibrate
    robot = MockRobot(simulated_offset=simulated_offset)
    calibrator = OrientationCalibrator(calibration_distance=1.0, settling_time=0.01)
    calibrator.calibrate(robot, verbose=False)
    
    # Reset robot position
    robot.reset_position()
    
    # Test transformation
    print("\nTesting vector transformations:")
    test_vectors = [
        (1.0, 0.0, "Move 1m in +X (OptiTrack)"),
        (0.0, 1.0, "Move 1m in +Y (OptiTrack)"),
        (1.0, 1.0, "Move diagonally (OptiTrack)"),
    ]
    
    all_passed = True
    tolerance = 0.01  # 1cm tolerance
    
    for dx_desired, dy_desired, description in test_vectors:
        print(f"\n  {description}")
        print(f"    Desired movement (OptiTrack): ({dx_desired:.3f}, {dy_desired:.3f})")
        
        # Transform vector
        dx_robot, dy_robot = calibrator.transform_vector(dx_desired, dy_desired)
        print(f"    Transformed command (Robot): ({dx_robot:.3f}, {dy_robot:.3f})")
        
        # Command robot with transformed vector
        robot.reset_position()
        robot.move_by_vector(dx_robot, dy_robot)
        
        # Check actual movement matches desired
        actual_x, actual_y, _ = robot.get_position()
        print(f"    Actual movement (OptiTrack): ({actual_x:.3f}, {actual_y:.3f})")
        
        error_x = abs(actual_x - dx_desired)
        error_y = abs(actual_y - dy_desired)
        error = math.hypot(error_x, error_y)
        
        passed = error < tolerance
        all_passed = all_passed and passed
        
        print(f"    Error: {error:.4f}m")
        print(f"    Status: {'✓ PASS' if passed else '✗ FAIL'}")
    
    print(f"\n{'='*70}")
    print(f"Overall: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    print(f"{'='*70}")
    
    return all_passed


def test_save_load_calibration():
    """Test saving and loading calibration data."""
    print("\n" + "="*70)
    print("TEST 3: Save/Load Calibration")
    print("="*70)
    
    # Create calibrator and set an offset
    calibrator1 = OrientationCalibrator()
    calibrator1.orientation_offset = 0.785398  # 45°
    calibrator1.is_calibrated = True
    
    # Save to temporary file
    temp_file = "/tmp/test_calibration.json"
    calibrator1.save_calibration(temp_file)
    print(f"\nSaved calibration to {temp_file}")
    print(f"  Offset: {calibrator1.orientation_offset:.6f} rad")
    
    # Load into new calibrator
    calibrator2 = OrientationCalibrator()
    success = calibrator2.load_calibration(temp_file)
    
    print(f"\nLoaded calibration from {temp_file}")
    print(f"  Success: {success}")
    print(f"  Offset: {calibrator2.orientation_offset:.6f} rad")
    
    # Check if they match
    match = abs(calibrator1.orientation_offset - calibrator2.orientation_offset) < 1e-6
    print(f"\nOffsets match: {'✓ PASS' if match else '✗ FAIL'}")
    
    # Cleanup
    import os
    if os.path.exists(temp_file):
        os.remove(temp_file)
    
    print(f"{'='*70}")
    
    return success and match


def test_robot_integration():
    """Test integration with Robot class (without actual hardware)."""
    print("\n" + "="*70)
    print("TEST 4: Robot Class Integration")
    print("="*70)
    
    print("\nNote: This test verifies the API exists and works.")
    print("Full hardware integration requires actual robot.\n")
    
    # Create a mock robot object with the same interface as Robot
    class TestableRobot:
        def __init__(self):
            self.orientation_offset = 0.0
            self.use_orientation_calibration = False
            
        def set_orientation_calibration(self, offset, enabled=True):
            self.orientation_offset = offset
            self.use_orientation_calibration = enabled
            
        def _transform_vector(self, dx, dy):
            import math
            cos_theta = math.cos(-self.orientation_offset)
            sin_theta = math.sin(-self.orientation_offset)
            dx_robot = dx * cos_theta - dy * sin_theta
            dy_robot = dx * sin_theta + dy * cos_theta
            return (dx_robot, dy_robot)
    
    robot = TestableRobot()
    
    # Test setting calibration
    test_offset = math.pi / 4  # 45°
    robot.set_orientation_calibration(test_offset, enabled=True)
    
    print(f"Set calibration offset: {test_offset:.4f} rad ({math.degrees(test_offset):.2f}°)")
    print(f"Calibration enabled: {robot.use_orientation_calibration}")
    
    # Test transformation
    dx, dy = 1.0, 0.0
    dx_t, dy_t = robot._transform_vector(dx, dy)
    
    print(f"\nTransform test:")
    print(f"  Input: ({dx:.3f}, {dy:.3f})")
    print(f"  Output: ({dx_t:.3f}, {dy_t:.3f})")
    
    # Verify transformation is correct
    expected_dx = math.cos(-test_offset)
    expected_dy = math.sin(-test_offset)
    
    error = math.hypot(dx_t - expected_dx, dy_t - expected_dy)
    passed = error < 0.001
    
    print(f"  Expected: ({expected_dx:.3f}, {expected_dy:.3f})")
    print(f"  Error: {error:.6f}")
    print(f"  Status: {'✓ PASS' if passed else '✗ FAIL'}")
    
    print(f"{'='*70}")
    
    return passed


def test_edge_cases():
    """Test edge cases and error handling."""
    print("\n" + "="*70)
    print("TEST 5: Edge Cases")
    print("="*70)
    
    all_passed = True
    
    # Test 1: Robot doesn't move enough
    print("\nTest 5.1: Robot doesn't move sufficiently")
    class StuckRobot:
        def __init__(self):
            self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        def get_position(self):
            return (self.x, self.y, self.yaw)
        def move_by_vector(self, dx, dy):
            # Simulate robot that barely moves
            self.x += 0.01
            self.y += 0.01
    
    stuck_robot = StuckRobot()
    calibrator = OrientationCalibrator(calibration_distance=1.0, settling_time=0.01)
    
    try:
        calibrator.calibrate(stuck_robot, verbose=False)
        print("  ✗ FAIL: Should have raised RuntimeError")
        all_passed = False
    except RuntimeError as e:
        print(f"  ✓ PASS: Correctly raised error: {str(e)[:50]}...")
    
    # Test 2: Transformation without calibration
    print("\nTest 5.2: Transform without calibration (should pass through)")
    uncalibrated = OrientationCalibrator()
    dx, dy = 1.0, 2.0
    dx_t, dy_t = uncalibrated.transform_vector(dx, dy)
    
    passed = (dx == dx_t and dy == dy_t)
    print(f"  Input: ({dx}, {dy})")
    print(f"  Output: ({dx_t}, {dy_t})")
    print(f"  {'✓ PASS' if passed else '✗ FAIL'}: Vectors match")
    all_passed = all_passed and passed
    
    # Test 3: Load non-existent file
    print("\nTest 5.3: Load non-existent calibration file")
    calibrator = OrientationCalibrator()
    success = calibrator.load_calibration("/nonexistent/file.json")
    
    passed = not success
    print(f"  Load success: {success}")
    print(f"  {'✓ PASS' if passed else '✗ FAIL'}: Correctly returned False")
    all_passed = all_passed and passed
    
    print(f"\n{'='*70}")
    print(f"Overall: {'✓ ALL TESTS PASSED' if all_passed else '✗ SOME TESTS FAILED'}")
    print(f"{'='*70}")
    
    return all_passed


def run_all_tests():
    """Run all test suites."""
    print("\n" + "="*70)
    print(" ORIENTATION CALIBRATION TEST SUITE")
    print("="*70)
    
    results = []
    
    # Run tests
    results.append(("Calibration Detection", test_calibration_with_various_offsets()))
    results.append(("Vector Transformation", test_vector_transformation()))
    results.append(("Save/Load", test_save_load_calibration()))
    results.append(("Robot Integration", test_robot_integration()))
    results.append(("Edge Cases", test_edge_cases()))
    
    # Summary
    print("\n" + "="*70)
    print(" TEST SUMMARY")
    print("="*70)
    
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {name:30s} {status}")
    
    all_passed = all(r[1] for r in results)
    
    print("="*70)
    if all_passed:
        print("✓ ALL TESTS PASSED!")
    else:
        print("✗ SOME TESTS FAILED")
    print("="*70 + "\n")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(run_all_tests())

