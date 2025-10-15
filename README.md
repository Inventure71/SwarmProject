# SwarmProject - Robot Path Following System

A modular system for robot path tracking and following with support for both simulated (dummy) and real robots with external tracking (OptiTrack/Vicon).

## Quick Start

### Using Dummy Robot (Simulation)
```bash
python path_test.py --track-type circle --show-ui
```

### Using Real Robot (OptiTrack)
```bash
python path_test.py --robot-name umh_2 --track-type circle --show-ui
```

### Recording a Track
```bash
python track_recorder.py --mode autonomous --duration 30 --output my_track.json
```

## Project Structure

```
SwarmProject/
├── config/
│   └── config.json          # Robot configurations (UDP ports)
├── docs/                    # All documentation
│   ├── QUICK_START.md
│   ├── REAL_ROBOT_GUIDE.md
│   ├── PATH_TESTING_README.md
│   └── ...
├── src/
│   ├── core/                # Core modules
│   │   ├── robot.py         # Robot interface and Real robot
│   │   ├── dummy_robot.py   # Simulated robot
│   │   ├── track.py         # Track data structures
│   │   ├── path_follower.py # Path following logic
│   │   └── path_visualizer.py # Real-time UI
│   ├── tracking/            # External tracking support
│   │   ├── robot_tracker.py # UDP listener for OptiTrack
│   │   └── robot_config.py  # Configuration manager
│   └── apps/                # Main applications
│       ├── path_test.py     # Path following with UI
│       └── track_recorder.py # Track creation
├── tools/
│   └── packet_inspector.py  # OptiTrack packet debugger
├── path_test.py             # Launcher for path_test
└── track_recorder.py        # Launcher for track_recorder
```

## Features

✅ **Path Following**
- Real-time visualization
- Multiple track types (circle, figure-8, square, line)
- Adjustable lane offset for racing
- Smooth corner handling

✅ **Robot Support**
- Dummy robot for simulation/testing
- Real robot with external tracking (OptiTrack/Vicon)
- Seamless switching between robot types

✅ **Track Management**
- Generate predefined tracks
- Record custom tracks from robot movement
- Save/load tracks (JSON format)

✅ **Advanced Features**
- Lateral offset for multi-robot racing
- Infinite looping on circular tracks
- Intersection handling (figure-8)
- Real-time offset adjustment (keyboard controls)

## Common Commands

### Path Following

```bash
# Dummy robot on circle track
python path_test.py --track-type circle --show-ui

# Real robot on figure-8 with offset
python path_test.py --robot-name umh_2 --track-type figure8 --offset 0.3 --show-ui

# Load custom track
python path_test.py --track-file my_track.json --show-ui
```

### Track Recording

```bash
# Record for 30 seconds
python track_recorder.py --mode autonomous --duration 30 --output track.json

# Record with manual stop (Ctrl+C when done)
python track_recorder.py --mode manual --output track.json
```

### Diagnostics

```bash
# Inspect OptiTrack packets
cd tools
python packet_inspector.py 9880
```

## Configuration

Edit `config/config.json` to add your robots:

```json
{
    "ROBOT_CONFIG": {
        "umh_2": 9876,
        "umh_3": 9877,
        "umh_4": 9878,
        "umh_5": 9880
    }
}
```

- **Key**: Robot name (matches OptiTrack rigid body name)
- **Value**: UDP port for tracking data

## UI Controls

When running with `--show-ui`:

- **← or A**: Move left (decrease offset)
- **→ or D**: Move right (increase offset)
- **C**: Center (reset offset to 0)

## UI Information Display

The UI now shows:

- 🤖 Robot type (Dummy/Real)
- 📍 Detailed position information
  - Robot position (X, Y, Yaw)
  - Target position
  - All values in meters/degrees
- 📊 Metrics
  - Distance to target
  - Distance from track
  - Lookahead distance
  - Current offset
- 🎮 Control help

## Documentation

See `docs/` directory for detailed documentation:

- `QUICK_START.md` - 5-minute quick reference
- `REAL_ROBOT_GUIDE.md` - Complete guide for real robots
- `PATH_TESTING_README.md` - Detailed feature documentation
- `PACKET_FORMAT_FIX.md` - OptiTrack packet format details

## Requirements

- Python 3.8+
- matplotlib
- numpy (for track generation)

## Development

The codebase follows clean architecture principles:

- **core/**: Pure business logic, no external dependencies
- **tracking/**: Integration with external tracking systems
- **apps/**: User-facing applications

All modules use relative imports within packages and absolute imports via `sys.path` for cross-package imports.

## Troubleshooting

### "Robot not found in config"
Add the robot to `config/config.json`

### "Could not bind to port"
Port is already in use - change port in config or stop other process

### Position not updating (real robot)
1. Check OptiTrack is streaming to correct port
2. Use `tools/packet_inspector.py` to verify packets
3. Check firewall settings

### UI not showing robot movement
- For **dummy robots**: Check track is generated correctly
- For **real robots**: Verify OptiTrack is sending position updates (check UI shows robot type as "Real")

## License

[Your License Here]

## Support

For issues or questions, see documentation in `docs/` directory.

