#!/usr/bin/env python3
"""
Track recorder - Record robot positions over time to create a track.

This tool allows you to:
1. Record a robot's movement (manual control or autonomous)
2. Generate a track from the recorded positions
3. Save the track for later use
"""

import argparse
import sys
import time
from typing import List, Optional
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from core.robot import create_robot
from core.track import Track, TrackPoint
from tracking.robot_config import RobotManager


class TrackRecorder:
    """Records robot positions over time to create a track."""
    
    def __init__(
        self,
        robot,
        sample_rate_hz: float = 10.0,
        min_distance_threshold: float = 0.05
    ):
        """
        Initialize track recorder.
        
        Args:
            robot: Robot instance to record
            sample_rate_hz: Position sampling rate in Hz
            min_distance_threshold: Minimum distance between points to record (meters)
        """
        self.robot = robot
        self.sample_rate_hz = sample_rate_hz
        self.min_distance_threshold = min_distance_threshold
        
        self.recorded_points: List[TrackPoint] = []
        self.recording = False
        self.start_time = None
    
    def start_recording(self) -> None:
        """Start recording robot positions."""
        if not self.robot.is_ready():
            raise RuntimeError("Robot is not ready")
        
        self.recorded_points = []
        self.start_time = time.time()
        self.recording = True
        
        # Record initial position
        x, y, _ = self.robot.get_position()
        self.recorded_points.append(TrackPoint(x, y, 0.0))
        
        print(f"Recording started at ({x:.3f}, {y:.3f})")
    
    def stop_recording(self) -> None:
        """Stop recording."""
        self.recording = False
        print(f"Recording stopped. Captured {len(self.recorded_points)} points")
    
    def update(self) -> bool:
        """
        Record current position if conditions are met.
        
        Returns:
            True if recording, False if stopped
        """
        if not self.recording:
            return False
        
        # Get current position and time
        x, y, _ = self.robot.get_position()
        current_time = time.time() - self.start_time
        
        # Check if we should record this point (based on distance threshold)
        if self.recorded_points:
            last_point = self.recorded_points[-1]
            distance = ((x - last_point.x)**2 + (y - last_point.y)**2)**0.5
            
            if distance < self.min_distance_threshold:
                return True  # Skip this point, too close to last one
        
        # Record the point
        self.recorded_points.append(TrackPoint(x, y, current_time))
        
        return True
    
    def create_track(self, name: str = "Recorded Track") -> Track:
        """
        Create a track from recorded points.
        
        Args:
            name: Name for the track
            
        Returns:
            Track object created from recorded points
        """
        if len(self.recorded_points) < 2:
            raise ValueError("Need at least 2 points to create a track")
        
        return Track(self.recorded_points, name=name)
    
    def get_stats(self) -> dict:
        """Get recording statistics."""
        if len(self.recorded_points) < 2:
            return {
                'points': len(self.recorded_points),
                'duration': 0.0,
                'distance': 0.0,
                'avg_speed': 0.0
            }
        
        duration = self.recorded_points[-1].timestamp
        
        # Calculate total distance
        total_distance = 0.0
        for i in range(1, len(self.recorded_points)):
            p1 = self.recorded_points[i - 1]
            p2 = self.recorded_points[i]
            distance = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
            total_distance += distance
        
        avg_speed = total_distance / duration if duration > 0 else 0.0
        
        return {
            'points': len(self.recorded_points),
            'duration': duration,
            'distance': total_distance,
            'avg_speed': avg_speed
        }


class ManualControlRecorder:
    """Interactive manual control for recording tracks."""
    
    def __init__(self, robot, recorder: TrackRecorder):
        self.robot = robot
        self.recorder = recorder
        self.running = False
    
    def print_controls(self) -> None:
        """Print control instructions."""
        print("\n" + "=" * 60)
        print("MANUAL CONTROL MODE")
        print("=" * 60)
        print("Controls:")
        print("  W - Move forward")
        print("  S - Move backward")
        print("  A - Move left")
        print("  D - Move right")
        print("  Q - Stop movement")
        print("  R - Start/Resume recording")
        print("  P - Pause recording")
        print("  X - Exit and save")
        print("=" * 60)
        print()
    
    def run(self) -> None:
        """Run manual control loop."""
        self.print_controls()
        
        print("Press R to start recording...")
        
        try:
            import sys
            import tty
            import termios
            
            # Save terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            
            try:
                tty.setcbreak(sys.stdin.fileno())
                self.running = True
                
                while self.running:
                    # Check for key press (non-blocking)
                    import select
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1).lower()
                        self._handle_key(key)
                    
                    # Update recorder
                    if self.recorder.recording:
                        self.recorder.update()
                    
                    time.sleep(0.1)
            
            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        except ImportError:
            # Fallback for Windows or systems without termios
            print("Manual control not available on this system.")
            print("Recording for 10 seconds automatically...")
            self.recorder.start_recording()
            
            for _ in range(100):
                self.recorder.update()
                time.sleep(0.1)
            
            self.recorder.stop_recording()
    
    def _handle_key(self, key: str) -> None:
        """Handle keyboard input."""
        step = 0.1  # meters
        
        if key == 'w':
            self.robot.move_by_vector(step, 0)
            print("↑ Forward", end='\r')
        elif key == 's':
            self.robot.move_by_vector(-step, 0)
            print("↓ Backward", end='\r')
        elif key == 'a':
            self.robot.move_by_vector(0, step)
            print("← Left", end='\r')
        elif key == 'd':
            self.robot.move_by_vector(0, -step)
            print("→ Right", end='\r')
        elif key == 'q':
            self.robot.stop()
            print("⏸ Stopped", end='\r')
        elif key == 'r':
            if not self.recorder.recording:
                self.recorder.start_recording()
        elif key == 'p':
            if self.recorder.recording:
                self.recorder.stop_recording()
        elif key == 'x':
            print("\nExiting...")
            self.running = False


def record_autonomous(args: argparse.Namespace) -> Track:
    """Record track from autonomous robot movement."""
    print("Recording track from autonomous movement...")
    print(f"Duration: {args.duration}s")
    
    # Create robot
    robot = create_robot(args.robot_type)
    
    # Wait for robot to be ready
    print("Waiting for robot to be ready...")
    while not robot.is_ready():
        time.sleep(0.1)
    
    # Create recorder
    recorder = TrackRecorder(
        robot,
        sample_rate_hz=args.sample_rate,
        min_distance_threshold=args.min_distance
    )
    
    # Start recording
    recorder.start_recording()
    
    # Record for specified duration
    start_time = time.time()
    dt = 1.0 / args.sample_rate
    
    print(f"Recording for {args.duration} seconds...")
    while time.time() - start_time < args.duration:
        recorder.update()
        
        # Print progress
        elapsed = time.time() - start_time
        if int(elapsed) % 1 == 0:
            x, y, _ = robot.get_position()
            print(f"  {elapsed:.1f}s - Position: ({x:.3f}, {y:.3f})", end='\r')
        
        time.sleep(dt)
    
    print()
    recorder.stop_recording()
    
    # Show stats
    stats = recorder.get_stats()
    print(f"\nRecording Statistics:")
    print(f"  Points recorded: {stats['points']}")
    print(f"  Duration: {stats['duration']:.2f}s")
    print(f"  Distance traveled: {stats['distance']:.2f}m")
    print(f"  Average speed: {stats['avg_speed']:.3f}m/s")
    
    # Create and return track
    track = recorder.create_track(args.track_name)
    robot.shutdown()
    
    return track


def record_manual(args: argparse.Namespace) -> Track:
    """Record track from manual control."""
    print("Recording track from manual control...")
    
    # Create robot (must be dummy for manual control)
    robot = create_robot("dummy", initial_x=0.0, initial_y=0.0, max_speed=1.0)
    
    # Wait for robot to be ready
    print("Waiting for robot to be ready...")
    while not robot.is_ready():
        time.sleep(0.1)
    
    # Create recorder
    recorder = TrackRecorder(
        robot,
        sample_rate_hz=args.sample_rate,
        min_distance_threshold=args.min_distance
    )
    
    # Run manual control
    controller = ManualControlRecorder(robot, recorder)
    controller.run()
    
    # Show stats
    stats = recorder.get_stats()
    print(f"\nRecording Statistics:")
    print(f"  Points recorded: {stats['points']}")
    print(f"  Duration: {stats['duration']:.2f}s")
    print(f"  Distance traveled: {stats['distance']:.2f}m")
    print(f"  Average speed: {stats['avg_speed']:.3f}m/s")
    
    # Create and return track
    track = recorder.create_track(args.track_name)
    robot.shutdown()
    
    return track


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Record robot positions to create a track",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Mode selection
    parser.add_argument(
        '--mode',
        type=str,
        default='autonomous',
        choices=['autonomous', 'manual'],
        help='Recording mode'
    )
    
    # Robot options
    parser.add_argument(
        '--robot-type',
        type=str,
        default='dummy',
        choices=['dummy', 'ros2'],
        help='Type of robot to record from'
    )
    
    # Recording options
    parser.add_argument(
        '--duration',
        type=float,
        default=10.0,
        help='Recording duration in seconds (autonomous mode only)'
    )
    parser.add_argument(
        '--sample-rate',
        type=float,
        default=10.0,
        help='Position sampling rate in Hz'
    )
    parser.add_argument(
        '--min-distance',
        type=float,
        default=0.05,
        help='Minimum distance between recorded points (meters)'
    )
    
    # Output options
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Output file path for the track (JSON or pickle)'
    )
    parser.add_argument(
        '--track-name',
        type=str,
        default='Recorded Track',
        help='Name for the recorded track'
    )
    
    args = parser.parse_args(argv)
    
    try:
        # Record track based on mode
        if args.mode == 'autonomous':
            track = record_autonomous(args)
        else:
            track = record_manual(args)
        
        # Save track
        print(f"\nSaving track to {args.output}...")
        track.save(args.output)
        print(f"✓ Track saved successfully!")
        
        print(f"\nTo use this track:")
        print(f"  python path_test.py --track-file {args.output} --show-ui")
        
        return 0
    
    except KeyboardInterrupt:
        print("\n\nRecording interrupted by user")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())

