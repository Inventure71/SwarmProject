# Path Testing System

A modular robot path following system with real-time visualization.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run with visualization
python path_test.py --show-ui

# Run demos
python demo_path_test.py
```

## Modules

- **`dummy_robot.py`** - Simulated robot for testing
- **`robot.py`** - Real robot (ROS2) implementation + factory
- **`track.py`** - Track generation and management
- **`path_follower.py`** - Path following logic
- **`path_visualizer.py`** - Real-time visualization UI
- **`path_test.py`** - Main application
- **`demo_path_test.py`** - Interactive demos
- **`track_recorder.py`** - Record robot movement to create tracks

## Usage

```python
from robot import create_robot
from track import TrackGenerator
from path_follower import PathFollower

# Create robot
robot = create_robot("dummy")

# Generate track
track = TrackGenerator.generate_circle(radius=2.0)

# Follow path
follower = PathFollower(robot, track)
follower.start()
```

## Examples

```bash
# Different tracks
python path_test.py --track-type circle --show-ui
python path_test.py --track-type figure8 --show-ui
python path_test.py --track-type square --show-ui

# Adjust parameters
python path_test.py --lookahead-distance 0.3 --show-ui
python path_test.py --track-size 3.0 --show-ui
```

## Lane Offset for Racing

The offset feature allows robots to follow parallel paths to the main track, perfect for multi-robot racing.

### Features

✅ **Smooth corner handling** - Uses direction smoothing to navigate sharp turns  
✅ **Stuck detection & recovery** - Automatically detects and recovers from stuck situations  
✅ **Safe offset limits** - Automatically clamps offset to safe range  
✅ **Works on all track types** - Circle, figure-8, square, straight line  
✅ **Real-time adjustment** - Change lanes while robot is moving  

### Usage

```bash
# Run with UI and adjust offset in real-time
python path_test.py --track-type square --offset -0.2 --show-ui

# Use keyboard controls:
# ← or A: Move left (decrease offset)
# → or D: Move right (increase offset)
# C: Center (reset to 0)
```

Or programmatically:

```python
from path_follower import PathFollower

# Create robot in right lane (+0.5m from center)
follower = PathFollower(robot, track, offset=0.5)

# Change lanes in real-time
follower.set_offset(-0.5)  # Move to left lane
```

### Technical Details

The offset system includes several advanced features to handle edge cases:

1. **Smoothed Direction Calculation** - Averages direction from nearby track segments to handle sharp corners
2. **Adaptive Lookahead** - Automatically increases lookahead distance for offset paths (tighter turns need more lookahead)
3. **Target Validation** - Ensures offset target is always ahead of robot (prevents backwards movement)
4. **Stuck Detection** - Monitors progress and auto-recovers if robot gets stuck at corners
5. **Safety Clamping** - Limits offset to 1.5x lookahead distance to prevent extreme situations

### Examples

```bash
# Multi-robot racing demo
python example_racing_offset.py

# Comprehensive corner handling tests
python test_corner_fixes.py
```

### Corner Handling

The offset feature has been extensively tested on sharp corners:
- ✅ Square tracks with inside/outside offset
- ✅ Tight circles with large offset
- ✅ Figure-8 tracks with overlapping paths
- ✅ Dynamic offset changes while moving

## Recording Custom Tracks

Record a robot's movement to create custom tracks:

```bash
# Record from a real ROS2 robot for 30 seconds
python track_recorder.py --mode autonomous --robot-type ros2 --duration 30 --output my_track.json

# Use the recorded track
python path_test.py --track-file my_track.json --show-ui
```

Or programmatically:

```python
from robot import create_robot
from track_recorder import TrackRecorder
from path_follower import PathFollower
from track import TrackGenerator

# Make a robot follow a path while recording
robot = create_robot("dummy")
track = TrackGenerator.generate_circle(radius=2.0)
follower = PathFollower(robot, track)

# Record the movement
recorder = TrackRecorder(robot, sample_rate_hz=10)
recorder.start_recording()

follower.start()
for _ in range(100):
    follower.update()
    recorder.update()

recorder.stop_recording()
recorded_track = recorder.create_track("My Custom Track")
recorded_track.save("custom_track.json")
```

