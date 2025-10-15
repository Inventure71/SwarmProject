# Real Robot Integration Guide

## Overview

The PathTesting system now supports both **dummy robots** (simulation) and **real robots** with external tracking (OptiTrack/Vicon). This guide explains how to use real robots.

## Architecture

```
OptiTrack/Vicon → UDP packets → RobotTracker → Robot.update_position()
                                                      ↓
                                              PathFollower reads position
                                                      ↓
                                              Sends movement commands
                                                      ↓
                                              Your robot hardware
```

## Quick Start

### 1. Configure Your Robots

Edit `/config.json` in the project root:

```json
{
    "ROBOT_CONFIG": {
        "umh_2": 9876,
        "umh_3": 9877,
        "umh_4": 9878,
        "umh_5": 9879
    }
}
```

- **Key**: Robot name (matches OptiTrack rigid body name)
- **Value**: UDP port number to listen on

### 2. Start OptiTrack Streaming

Make sure your OptiTrack/Vicon system is configured to stream robot poses via UDP to the ports specified in `config.json`.

Expected UDP packet format:
- 28 bytes (7 floats, little-endian)
- `[pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]`

### 3. Run with Real Robot

```bash
# Use real robot
python path_test.py --robot-name umh_2 --track-type circle --show-ui

# Multiple options
python path_test.py --robot-name umh_2 --track-type figure8 --offset 0.3 --show-ui

# Use dummy robot (simulation, default)
python path_test.py --track-type circle --show-ui
```

## Command-Line Options

### Real Robot Mode

```bash
--robot-name <name>    # Specify robot name from config.json (e.g., umh_2)
                       # If not provided, uses dummy robot
```

### Dummy Robot Mode (Default)

```bash
--start-x <float>      # Starting X position (default: track start)
--start-y <float>      # Starting Y position (default: track start)
--max-speed <float>    # Maximum speed in m/s (default: 0.5)
```

## Examples

### Example 1: Basic Real Robot Usage

```bash
python path_test.py --robot-name umh_2 --track-type circle --show-ui
```

### Example 2: Real Robot with Offset

```bash
python path_test.py --robot-name umh_3 --track-type square --offset -0.2 --show-ui
```

### Example 3: Real Robot Following Custom Track

```bash
# First record a track
python track_recorder.py --robot-name umh_2 --mode autonomous --duration 30 --output my_track.json

# Then follow it with another robot
python path_test.py --robot-name umh_3 --track-file my_track.json --show-ui
```

### Example 4: Dummy Robot (Simulation)

```bash
python path_test.py --track-type circle --show-ui
```

## Programmatic Usage

### Single Robot

```python
from robot_config import RobotManager
from track import TrackGenerator
from path_follower import PathFollower

# Create robot manager
manager = RobotManager()

# Create tracked robot
robot = manager.create_robot('umh_2')
# Robot position is now automatically updated from OptiTrack

# Create track and follower
track = TrackGenerator.generate_circle(radius=2.0)
follower = PathFollower(robot, track)

# Start following
follower.start()

# Main loop
while True:
    follower.update()
    
    # Get last commanded vector to send to hardware
    dx, dy = robot.get_last_command()
    # TODO: Send to your robot via UDP/Serial/etc.
    
    time.sleep(0.05)

# Cleanup
manager.stop()
```

### Multiple Robots

```python
from robot_config import RobotManager

manager = RobotManager()

# Create multiple tracked robots
robots = manager.create_multiple_robots(['umh_2', 'umh_3', 'umh_4'])

# Access individual robots
robot1 = robots['umh_2']
robot2 = robots['umh_3']
robot3 = robots['umh_4']

# All robots are now tracked automatically
x1, y1, yaw1 = robot1.get_position()
x2, y2, yaw2 = robot2.get_position()
```

## Testing

### Test Configuration

```bash
cd PathTesting
python robot_config.py
```

This will:
- Load config.json
- Display available robots
- Start tracking one robot
- Show position updates for 5 seconds

### Test Integration

```bash
cd PathTesting
python test_real_robot_integration.py
```

This runs comprehensive tests:
1. Config loading
2. Single robot creation
3. Multiple robot tracking
4. Integration with path_test.py

## Troubleshooting

### "Robot not found in config"

Make sure the robot name matches exactly what's in `config.json`.

```bash
# List available robots
python robot_config.py
```

### "Could not bind to port"

The UDP port is already in use. Either:
- Stop the other process using that port
- Change the port number in `config.json`

### Position Not Updating

Check that:
1. OptiTrack/Vicon is streaming to the correct IP and port
2. Firewall isn't blocking UDP packets
3. Robot rigid body is being tracked in OptiTrack

### Robot Not Moving

Remember: The `Robot` class only stores movement commands. You need to implement sending these commands to your actual robot hardware.

Get the last command:
```python
dx, dy = robot.get_last_command()
# Send this to your robot via UDP/Serial/HTTP/etc.
```

## Files

- `config.json` - Robot configuration (in project root)
- `robot.py` - Robot class (simplified for external tracking)
- `robot_config.py` - Configuration loader and RobotManager
- `Optitrack/robot_tracker.py` - UDP listener for position updates
- `path_test.py` - Main path following script (now supports real robots)

## Advanced: Custom Tracking Integration

If you're not using the UDP packet format expected by `RobotTracker`, you can:

1. Directly use the `Robot` class:
   ```python
   from robot import Robot
   
   robot = Robot()
   
   # In your custom tracking callback:
   def on_tracking_update(x, y, yaw):
       robot.update_position(x, y, yaw)
   ```

2. Subclass `RobotTracker` to parse different packet formats

3. Use a different tracking system entirely - just call `robot.update_position()` from your callback

## Summary

- **Dummy Mode**: `python path_test.py --track-type circle`
- **Real Robot Mode**: `python path_test.py --robot-name umh_2 --track-type circle`
- **Configuration**: Edit `config.json` to add your robots
- **Position Updates**: Automatic via `RobotTracker` listening on UDP
- **Movement Commands**: Retrieved via `robot.get_last_command()` - you implement sending to hardware

The system handles position tracking automatically. You just need to:
1. Configure robots in `config.json`
2. Start OptiTrack streaming
3. Run your script with `--robot-name`
4. Implement sending movement commands to your robot hardware

