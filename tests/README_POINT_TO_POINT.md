# Point-to-Point Path Following Test

A specialized test for debugging waypoint-based path following with visual vector display.

## Features

### Modified Path Following Behavior
- **Waypoint-by-waypoint**: Robot targets one waypoint at a time
- **Blocking behavior**: Waits until waypoint is reached before moving to next
- **Explicit vector display**: Large orange arrow shows exact movement vector
- **Reach detection**: Visual tolerance circle shows when waypoint is reached

### Calibration Support
- Full orientation calibration support for real robots
- Saves/loads calibration data per robot
- Can skip or force recalibration with flags

### Enhanced DummyRobot
The dummy robot now simulates the real robot's `mov_latest.py` behavior:
- **Local-to-world frame transformation**: Use `use_local_frame=True` for realistic simulation
- **Proportional control**: Mimics real robot's velocity control
- **Mecanum/holonomic mode**: Default mode matches real robot
- **Blocking movement option**: Can block until movement completes

## Usage

### Basic Test (Dummy Robot)
```bash
# Default: square path with 20 waypoints
python tests/test_point_to_point_path.py

# Circle path with more waypoints
python tests/test_point_to_point_path.py --track-type circle --num-waypoints 30

# Tighter tolerance for precision testing
python tests/test_point_to_point_path.py --tolerance 0.03
```

### Real Robot Test
```bash
# With calibration (recommended)
python tests/test_point_to_point_path.py --robot-name umh_5

# Skip calibration (for quick tests)
python tests/test_point_to_point_path.py --robot-name umh_5 --skip-calibration

# Force new calibration
python tests/test_point_to_point_path.py --robot-name umh_5 --force-calibration
```

### Track Options
```bash
# Different track types
--track-type circle      # Circular path
--track-type square      # Square path (default)
--track-type line        # Straight line
--track-type figure8     # Figure-8 path

# Adjust track size
--track-size 1.5         # Size in meters (radius or half-length)

# Control waypoint density
--num-waypoints 20       # Total waypoints to generate
```

### Control Options
```bash
# Waypoint tolerance (how close to get before "reached")
--tolerance 0.05         # Default: 5cm

# Control loop rate
--control-rate-hz 10.0   # Default: 10Hz
```

### Calibration Options
```bash
--skip-calibration              # Skip calibration entirely
--force-calibration             # Force new calibration
--calibration-distance 0.5      # Calibration movement distance (m)
--calibration-settling-time 2.0 # Wait time during calibration (s)
```

## What You'll See

### Visual Display
1. **Gray dots**: Waypoints not yet reached
2. **Green dots**: Waypoints already reached
3. **Red star**: Current target waypoint
4. **Red dashed circle**: Tolerance zone around target
5. **Orange arrow**: Movement vector (THE KEY FEATURE!)
6. **Green line**: Robot's path trail
7. **Blue dot**: Current robot position

### Information Panels
- **Top-left (light blue)**: General info (position, waypoints, progress)
- **Bottom-left (yellow)**: Movement vector details with angle

### Vector Information
```
→ MOVING TO WAYPOINT
🎯 MOVEMENT VECTOR:
  ΔX: +0.234 m
  ΔY: -0.156 m
  Distance: 0.281 m
  Angle: -33.7°
```

## Differences from Standard Path Follower

| Aspect | Standard PathFollower | PointToPointPathFollower |
|--------|----------------------|--------------------------|
| Target | Lookahead distance | One waypoint at a time |
| Progression | Continuous | Waits at each waypoint |
| Completion | Distance-based | Explicit reach detection |
| Visual feedback | Target point | Full vector arrow |
| Use case | Smooth following | Waypoint debugging |

## Debugging Tips

1. **Vector not pointing at target?**
   - Check if calibration was performed (for real robots)
   - Verify `use_local_frame` setting in dummy robot

2. **Robot oscillating near waypoint?**
   - Increase `--tolerance` value
   - Check for controller gain issues

3. **Robot not reaching waypoints?**
   - Decrease tolerance
   - Check movement command execution
   - Verify OptiTrack data (for real robots)

4. **Path looks wrong?**
   - Check frame transformation (local vs world)
   - Verify track generation parameters

## Example Commands

```bash
# Debug with small square and tight tolerance
python tests/test_point_to_point_path.py \
  --track-type square \
  --track-size 0.5 \
  --num-waypoints 12 \
  --tolerance 0.03

# Real robot with recalibration
python tests/test_point_to_point_path.py \
  --robot-name umh_5 \
  --force-calibration \
  --track-type circle \
  --track-size 1.0

# Fast test without UI
python tests/test_point_to_point_path.py \
  --no-ui \
  --num-waypoints 5 \
  --track-type line
```

## Implementation Notes

### DummyRobot Enhancements
- Added `use_local_frame` parameter for frame transformation
- Implemented blocking movement with event-based synchronization
- Added proportional control (kp_lin = 0.9)
- Simulates mecanum drive mode by default
- Position tolerance matches real robot (0.03m default)

### Calibration Integration
- Uses `OrientationCalibrator` from `src/core/orientation_calibrator.py`
- Saves calibration to `.calibration_{robot_name}.json`
- Prompts to reuse existing calibration
- Handles calibration failures gracefully

## Files Modified
1. `tests/test_point_to_point_path.py` - New test file with point-to-point follower
2. `src/core/dummy_robot.py` - Enhanced to mimic `mov_latest.py` behavior

## See Also
- `src/apps/path_test.py` - Standard path following test
- `Idk/ForRobot/mov_latest.py` - Real robot movement node
- `docs/ORIENTATION_CALIBRATION.md` - Calibration documentation

