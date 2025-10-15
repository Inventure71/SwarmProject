#!/usr/bin/env python3
"""
Packet Inspector - Debug tool for OptiTrack UDP packets.

This tool listens on a UDP port and displays detailed information
about the packets being received, helping you understand the format.
"""

import socket
import struct
import sys


def inspect_packet(data):
    """Analyze and display packet contents."""
    print("\n" + "="*70)
    print(f"Packet received: {len(data)} bytes")
    print("="*70)
    
    # Show raw hex
    print("\nRaw hex (first 64 bytes):")
    hex_str = data[:64].hex()
    for i in range(0, len(hex_str), 32):
        print(f"  {hex_str[i:i+32]}")
    
    # Try to parse as floats
    print("\nAttempting to parse as floats (<f = little-endian float):")
    num_floats = min(len(data) // 4, 20)  # Show up to 20 floats
    
    if len(data) >= 4:
        floats = struct.unpack(f'<{num_floats}f', data[:num_floats*4])
        for i, val in enumerate(floats):
            print(f"  Float {i:2d}: {val:12.6f}")
            
            # Highlight likely position/orientation values
            if i < 3:
                print(f"           ^ Likely position {'XYZ'[i]}")
            elif i < 7:
                print(f"           ^ Likely quaternion {'XYZW'[i-3]}")
    
    # Check if first 7 floats make sense as pose
    if len(data) >= 28:
        pose = struct.unpack('<7f', data[:28])
        x, y, z, qx, qy, qz, qw = pose
        
        print("\nFirst 7 floats interpreted as pose:")
        print(f"  Position: ({x:.4f}, {y:.4f}, {z:.4f})")
        print(f"  Quaternion: ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")
        
        # Check if quaternion is normalized (should be close to 1)
        quat_mag = (qx**2 + qy**2 + qz**2 + qw**2)**0.5
        print(f"  Quaternion magnitude: {quat_mag:.6f} (should be ~1.0)")
        
        if abs(quat_mag - 1.0) < 0.1:
            print("  ✓ Quaternion looks valid!")
        else:
            print("  ✗ Quaternion might be in wrong position or format")
    
    # Show remaining bytes
    if len(data) > 28:
        remaining = len(data) - 28
        print(f"\nRemaining {remaining} bytes after pose data:")
        print("  These might be:")
        print("    - Additional tracking data")
        print("    - Marker positions")
        print("    - Tracking quality metrics")
        print("    - Timestamp")
        print("    - Frame number")


def main(port=9880):
    """Listen on UDP port and inspect packets."""
    print("="*70)
    print("OPTITRACK PACKET INSPECTOR")
    print("="*70)
    print(f"\nListening on UDP port {port}...")
    print("Press Ctrl+C to stop\n")
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            s.bind(("0.0.0.0", port))
        except OSError as e:
            print(f"ERROR: Could not bind to port {port}")
            print(f"       {e}")
            print("\nMake sure:")
            print("  1. Port is not already in use")
            print("  2. OptiTrack is streaming to this port")
            return 1
        
        s.settimeout(5.0)
        
        packet_count = 0
        try:
            while True:
                try:
                    data, addr = s.recvfrom(1024)
                    packet_count += 1
                    
                    print(f"\n[Packet #{packet_count}] From: {addr[0]}:{addr[1]}")
                    inspect_packet(data)
                    
                    # Only show first 5 packets in detail, then summarize
                    if packet_count >= 5:
                        print("\n" + "="*70)
                        print(f"Shown {packet_count} packets. Continuing to receive...")
                        print("Packet format appears consistent.")
                        print("="*70)
                        
                        # Continue receiving but just count
                        while True:
                            data, addr = s.recvfrom(1024)
                            packet_count += 1
                            if packet_count % 100 == 0:
                                print(f"Received {packet_count} packets (size: {len(data)} bytes)...")
                    
                except socket.timeout:
                    if packet_count == 0:
                        print("⏳ No packets received yet. Waiting...")
                        print("   Make sure OptiTrack is streaming to this port!")
                    continue
                    
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            print(f"Total packets received: {packet_count}")
    
    return 0


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9880
    
    print("\nUsage: python packet_inspector.py [port]")
    print(f"Using port: {port}\n")
    
    sys.exit(main(port))

