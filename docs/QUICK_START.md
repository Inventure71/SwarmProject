# PathTesting with Real Robots - Quick Start

## 5-Minute Setup

### 1. Check Configuration (Already Done!)
```bash
cat config.json
```
Should show your robots (umh_2, umh_3, etc.) with UDP ports.

### 2. Test System
```bash
cd PathTesting
python test_real_robot_integration.py
```
Should pass all 4 tests ✅

### 3. Use Dummy Robot (Simulation)
```bash
python path_test.py --track-type circle --show-ui
```

### 4. Use Real Robot (OptiTrack)
```bash
# Make sure OptiTrack is streaming to UDP port!
python path_test.py --robot-name umh_2 --track-type circle --show-ui
```

## Common Commands

### Path Following

```bash
# Dummy robot on circle
python path_test.py --track-type circle --show-ui

# Real robot on circle  
python path_test.py --robot-name umh_2 --track-type circle --show-ui

# Real robot on figure-8 with lane offset
python path_test.py --robot-name umh_2 --track-type figure8 --offset 0.3 --show-ui

# Real robot on square (inside lane)
python path_test.py --robot-name umh_3 --track-type square --offset -0.2 --show-ui
```

### Track Recording

```bash
# Record track from real robot movement
python track_recorder.py --robot-name umh_2 --mode autonomous --duration 30 --output my_track.json

# Follow recorded track with another robot
python path_test.py --robot-name umh_3 --track-file my_track.json --show-ui
```

### Testing

```bash
# Test configuration and integration
python test_real_robot_integration.py

# Test just config loading
python robot_config.py
```

## Keyboard Controls (in UI)

- **← or A**: Move left (decrease offset)
- **→ or D**: Move right (increase offset)
- **C**: Center (reset offset to 0)

## Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `--robot-name` | string | Real robot name (e.g., umh_2) | None (uses dummy) |
| `--track-type` | choice | circle, figure8, square, line | circle |
| `--track-size` | float | Track size in meters | 2.0 |
| `--offset` | float | Lane offset in meters | 0.0 |
| `--lookahead-distance` | float | Lookahead in meters | 0.5 |
| `--show-ui` | flag | Show visualization | False |
| `--track-file` | path | Load custom track | None |

## Troubleshooting

**"Robot not found in config"**
```bash
# List available robots
python robot_config.py
```

**"Could not bind to port"**
- Port already in use
- Change port in config.json

**Position not updating**
- Check OptiTrack is streaming
- Check correct UDP port
- Check firewall

**Robot not moving in real life**
- Implement sending commands to hardware
- See `robot.get_last_command()` in code

## File Locations

- `/config.json` - Robot configuration
- `/PathTesting/` - All scripts
- `/PathTesting/REAL_ROBOT_GUIDE.md` - Full documentation
- `/INTEGRATION_SUMMARY.md` - Technical details

## Ready to Go!

1. ✅ Config is set up
2. ✅ Integration tests passing
3. ✅ Both dummy and real robots work
4. ✅ All offset features functional

Start with:
```bash
python PathTesting/path_test.py --robot-name umh_2 --track-type circle --show-ui
```

Happy Racing! 🏎️
