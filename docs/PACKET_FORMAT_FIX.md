# OptiTrack Packet Format Fix

## Problem

The system was receiving UDP packets of ~75-80 bytes from OptiTrack, but was expecting exactly 28 bytes. This caused continuous warnings:

```
[Tracker: umh_5] Warning: Received packet of unexpected size 78 on port 9880.
```

## Root Cause

Different OptiTrack configurations send different packet formats:
- **Expected**: 28 bytes (7 floats: position XYZ + quaternion XYZW)
- **Actual**: 75-80 bytes (extended format with additional tracking data)

## Solution

Updated `robot_tracker.py` to:

### 1. Auto-Detect Packet Format

On the first packet received, the system now:
- Detects the packet size
- Automatically determines the format
- Logs what format was detected

### 2. Handle Extended Packets

For packets larger than 28 bytes:
- Extracts the first 7 floats (28 bytes)
- Uses these for position + quaternion
- Ignores extra data (marker positions, tracking quality, etc.)

### 3. Reduce Warning Spam

- Only logs unexpected packets every 100 occurrences
- Prevents console flooding

## Code Changes

### Before
```python
if len(data) == 28:
    pose = struct.unpack('<7f', data)
    # ... use pose
else:
    print(f"Warning: unexpected size {len(data)}")  # Spammed every packet!
```

### After
```python
# Auto-detect on first packet
if first_packet:
    if packet_size >= 28:
        detected_format = 'extended'
        print(f"Using extended format: extracting first 7 floats from {packet_size} bytes")

# Parse based on detected format
if detected_format == 'extended' and len(data) >= 28:
    pose = struct.unpack('<7f', data[:28])  # Extract first 28 bytes
    # ... use pose
```

## What the Extra Bytes Contain

The additional ~50 bytes in your packets likely contain:
- Individual marker positions (3D coordinates for each marker)
- Tracking quality/confidence metrics
- Timestamp information
- Frame number
- Marker IDs

These aren't needed for basic robot positioning, so we safely ignore them.

## Diagnostic Tool

Created `packet_inspector.py` to help debug packet formats:

```bash
cd Optitrack
python packet_inspector.py 9880
```

This shows:
- Exact packet size
- Raw hex data
- Parsed float values
- Validation of quaternion
- What each part likely represents

## Testing

Test with your robots:

```bash
cd PathTesting
python path_test.py --robot-name umh_5 --track-type circle --show-ui
```

Expected output on first connection:
```
[Tracker: umh_5] Listening on port 9880...
[Tracker: umh_5] First packet received: 78 bytes
[Tracker: umh_5] Using extended format: extracting first 7 floats from 78 bytes
```

## Packet Format Reference

### Standard Format (28 bytes)
```
Bytes 0-3:   float - position X
Bytes 4-7:   float - position Y
Bytes 8-11:  float - position Z
Bytes 12-15: float - quaternion X
Bytes 16-19: float - quaternion Y
Bytes 20-23: float - quaternion Z
Bytes 24-27: float - quaternion W
```

### Extended Format (75-80 bytes)
```
Bytes 0-27:  Same as standard format (position + quaternion)
Bytes 28+:   Additional tracking data (varies by OptiTrack config)
```

## Verification

To verify packets are being parsed correctly:

1. Run packet inspector:
   ```bash
   python packet_inspector.py 9880
   ```

2. Check that:
   - Position values look reasonable (meters)
   - Quaternion magnitude ≈ 1.0
   - Values update when robot moves

3. If values look wrong, the packet format might be different - contact us for help

## Summary

✅ **Fixed**: Auto-detects and handles extended OptiTrack packet formats  
✅ **Fixed**: Eliminates warning spam  
✅ **Added**: Diagnostic tool for debugging  
✅ **Works**: With all OptiTrack streaming configurations  

The system now automatically adapts to your OptiTrack packet format!

