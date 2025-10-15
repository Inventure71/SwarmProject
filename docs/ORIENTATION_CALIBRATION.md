# Orientation Calibration Guide

## Overview

The orientation calibration feature automatically handles coordinate system mismatches between OptiTrack (UDP position tracking) and the RosBOT robot's internal coordinate system.

## The Problem

When you command the robot to move in a certain direction using OptiTrack coordinates, the robot interprets those commands in its own local coordinate frame. If there's a rotation offset between these two coordinate systems, the robot will move in the wrong direction.

**Example:**
- OptiTrack says "move 1m in the +X direction"
- Robot's coordinate system is rotated 45° relative to OptiTrack
- Robot moves at 45° instead of the intended direction

## The Solution

The calibration process automatically detects and corrects this offset:

1. **Record Starting Position**: Captures the robot's initial position from OptiTrack
2. **Command Forward Movement**: Tells the robot to move "straight forward" in its own coordinate frame
3. **Measure Actual Direction**: Observes where the robot actually moved in OptiTrack coordinates
4. **Calculate Offset**: Computes the rotation difference between robot and OptiTrack frames
5. **Apply Correction**: All subsequent movement commands are rotated to compensate

## Usage

### Automatic Calibration (Recommended)

When running with a real robot, calibration happens automatically:

```bash
python src/apps/path_test.py --robot-name umh_2 --track-type circle
```

The calibration process will:
- Display the calibration procedure
- Move the robot forward
- Calculate and save the offset
- Apply the correction to all movements

### Calibration Options

```bash
# Skip calibration (not recommended)
python src/apps/path_test.py --robot-name umh_2 --skip-calibration

# Force new calibration (ignore saved calibration)
python src/apps/path_test.py --robot-name umh_2 --force-calibration

# Customize calibration parameters
python src/apps/path_test.py --robot-name umh_2 \
    --calibration-distance 1.0 \
    --calibration-settling-time 3.0
```

### Calibration Parameters

- `--calibration-distance`: Distance the robot moves during calibration (default: 0.5m)
  - Larger = more accurate but takes more space
  - Smaller = faster but potentially less accurate
  
- `--calibration-settling-time`: Time to wait for robot to complete movement (default: 2.0s)
  - Increase if robot is slow or track has obstacles
  - Decrease for faster calibration if robot responds quickly

### Saved Calibrations

Calibrations are automatically saved to `.calibration_<robot_name>.json` files. The system will:
- Reuse saved calibrations on subsequent runs
- Prompt you to confirm before using saved calibration
- Allow you to force new calibration with `--force-calibration`

## Example Output

```
============================================================
ORIENTATION CALIBRATION
============================================================
This will calibrate the orientation offset between OptiTrack
and the robot's internal coordinate system.
The robot will move 0.5m forward.
============================================================

Step 1: Starting position
  Position: (0.0000, 0.0000)
  Yaw: 0.0000 rad (0.00°)

Step 2: Commanding robot to move 0.500m forward
  Robot frame command: dx=0.500, dy=0.000
  Waiting 2.0s for movement to complete...

Step 3: Ending position
  Position: (0.3536, 0.3536)
  Yaw: 0.0000 rad (0.00°)

Step 4: Actual movement in OptiTrack frame
  Vector: dx=0.3536, dy=0.3536
  Distance: 0.5000m

Step 5: Calibration complete!
  Orientation offset: 0.7854 rad (45.00°)

Interpretation:
  When robot thinks it's moving 'forward' (0°),
  it actually moves at 45.00° in OptiTrack coordinates.

All subsequent movement commands will be rotated by this offset.
============================================================
```

## Technical Details

### Coordinate Transform

The calibration applies a rotation transformation to all movement vectors:

```python
# Convert from OptiTrack coordinates to robot coordinates
dx_robot = dx_optitrack * cos(-offset) - dy_optitrack * sin(-offset)
dy_robot = dx_optitrack * sin(-offset) + dy_optitrack * cos(-offset)
```

### Integration Points

The calibration is integrated at multiple levels:

1. **OrientationCalibrator** (`src/core/orientation_calibrator.py`):
   - Performs the calibration procedure
   - Saves/loads calibration data
   - Provides vector transformation

2. **Robot** (`src/core/robot.py`):
   - Stores orientation offset
   - Applies transformation before sending commands
   - Controlled via `set_orientation_calibration()`

3. **PathTestApp** (`src/apps/path_test.py`):
   - Orchestrates calibration on startup
   - Manages saved calibrations
   - Provides user interaction

## Troubleshooting

### "Robot did not move sufficiently during calibration"

**Causes:**
- Robot not connected properly
- Movement commands not reaching robot
- Robot stuck or blocked
- OptiTrack not tracking robot

**Solutions:**
- Verify robot is powered and connected
- Check ZMQ connection (should be on port 5555)
- Ensure robot has clear path to move
- Verify OptiTrack is sending position updates
- Increase `--calibration-settling-time` if robot is slow

### Calibration seems inaccurate

**Solutions:**
- Increase `--calibration-distance` (e.g., 1.0m instead of 0.5m)
- Ensure robot has stable OptiTrack tracking
- Run calibration when robot is stationary initially
- Use `--force-calibration` to recalibrate

### Robot still moves in wrong direction after calibration

**Check:**
- Calibration actually ran (look for calibration output)
- Calibration file was saved
- No `--skip-calibration` flag was used
- OptiTrack coordinates are consistent

## API Reference

### OrientationCalibrator

```python
from core.orientation_calibrator import OrientationCalibrator

# Create calibrator
calibrator = OrientationCalibrator(
    calibration_distance=0.5,  # meters
    settling_time=2.0           # seconds
)

# Perform calibration
offset = calibrator.calibrate(robot, verbose=True)

# Transform vectors
dx_robot, dy_robot = calibrator.transform_vector(dx_optitrack, dy_optitrack)

# Save/load calibration
calibrator.save_calibration("calibration.json")
calibrator.load_calibration("calibration.json")
```

### Robot Integration

```python
from core.robot import Robot

robot = Robot(robot_ip="192.168.1.2")

# Enable calibration
robot.set_orientation_calibration(offset=0.785, enabled=True)

# Movement commands are now automatically transformed
robot.move_by_vector(1.0, 0.0)  # Will be rotated by offset
```

## See Also

- [Real Robot Guide](REAL_ROBOT_GUIDE.md) - General guide for using real robots
- [OptiTrack Guide](OPTITRACK_GUIDE.md) - OptiTrack setup and configuration
- [Quick Start](QUICK_START.md) - Getting started with the system

