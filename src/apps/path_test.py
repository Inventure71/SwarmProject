#!/usr/bin/env python3
"""
Path testing script for robot trajectory following.

Features:
- Generate or load predefined tracks
- Calculate target position based on lookahead distance
- Send movement commands to robot
- Real-time visualization and debugging UI
"""

import argparse
import sys
import time
from typing import Optional, List
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from core.robot import create_robot
from core.track import Track, TrackGenerator
from core.path_follower import PathFollower
from core.path_visualizer import PathFollowingDebugUI
from tracking.robot_config import RobotManager


class PathTestApp:
    """Main application for path testing."""
    
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.robot = None
        self.track = None
        self.follower = None
        self.robot_manager = None  # For real robot tracking
    
    def setup(self) -> None:
        """Set up the application."""
        # Create or load track
        if self.args.track_file:
            print(f"Loading track from {self.args.track_file}...")
            self.track = Track.load(self.args.track_file)
        else:
            print(f"Generating {self.args.track_type} track...")
            self.track = self._generate_track()
            
            # Save if requested
            if self.args.save_track:
                print(f"Saving track to {self.args.save_track}...")
                self.track.save(self.args.save_track)
        
        print(f"Track: {self.track.name}")
        print(f"  Points: {len(self.track.points)}")
        print(f"  Duration: {self.track.get_total_duration():.2f}s")
        
        # Create robot
        if self.args.robot_name:
            # Use real robot with OptiTrack tracking
            print(f"Creating real robot: {self.args.robot_name}")
            self.robot_manager = RobotManager()
            
            # Verify robot exists
            available = self.robot_manager.get_available_robots()
            if self.args.robot_name not in available:
                raise ValueError(
                    f"Robot '{self.args.robot_name}' not found in config.\n"
                    f"Available robots: {', '.join(available)}"
                )
            
            # Create robot with tracking
            config = self.robot_manager.get_robot_config(self.args.robot_name)
            print(f"  Robot IP: {config['ip']}")
            print(f"  UDP Port: {config['port']}")
            print(f"  Starting OptiTrack listener...")
            
            self.robot = self.robot_manager.create_robot(self.args.robot_name)
            print(f"  Robot ready! Position updates will come from OptiTrack.")
        else:
            # Use dummy robot for simulation
            print("Creating dummy robot (simulation mode)")
            
            # Auto-set starting position to track start if not specified
            start_x = self.args.start_x if self.args.start_x is not None else self.track.points[0].x
            start_y = self.args.start_y if self.args.start_y is not None else self.track.points[0].y
            
            robot_kwargs = {
                'initial_x': start_x,
                'initial_y': start_y,
                'max_speed': self.args.max_speed
            }
            
            print(f"  Starting position: ({start_x:.2f}, {start_y:.2f})")
            
            self.robot = create_robot("dummy", **robot_kwargs)
            
            # Wait for dummy robot to be ready
            print("Waiting for robot to be ready...")
            timeout = 10.0
            start = time.time()
            while not self.robot.is_ready():
                if time.time() - start > timeout:
                    raise RuntimeError("Robot failed to become ready")
                time.sleep(0.1)
            print("Robot ready!")
        
        # Create path follower
        self.follower = PathFollower(
            self.robot,
            self.track,
            lookahead_distance=self.args.lookahead_distance,
            control_rate_hz=self.args.control_rate_hz,
            offset=self.args.offset
        )
        
        if self.args.offset != 0.0:
            print(f"  Lane offset: {self.args.offset:+.2f}m")

    
    def _generate_track(self) -> Track:
        """Generate a track based on arguments."""
        if self.args.track_type == "circle":
            return TrackGenerator.generate_circle(
                radius=self.args.track_size,
                speed=self.args.track_speed
            )
        elif self.args.track_type == "figure8":
            return TrackGenerator.generate_figure_eight(
                radius=self.args.track_size,
                speed=self.args.track_speed
            )
        elif self.args.track_type == "square":
            return TrackGenerator.generate_square(
                side_length=self.args.track_size * 2,
                speed=self.args.track_speed
            )
        elif self.args.track_type == "line":
            return TrackGenerator.generate_straight_line(
                length=self.args.track_size * 2,
                speed=self.args.track_speed
            )
        else:
            raise ValueError(f"Unknown track type: {self.args.track_type}")
    
    def run(self) -> None:
        """Run the application."""
        try:
            # Start path following
            print("Starting path follower...")
            self.follower.start()
            
            if self.args.show_ui:
                # Run with UI
                print("Starting debug UI...")
                ui = PathFollowingDebugUI(self.follower)
                ui.run()
            else:
                # Run without UI
                print("Running without UI (use --show-ui to enable visualization)...")
                dt = 1.0 / self.args.control_rate_hz
                
                while self.follower.running:
                    still_running = self.follower.update()
                    if not still_running:
                        break
                    time.sleep(dt)
                
                print("Track complete!")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            # Cleanup
            print("Cleaning up...")
            if self.follower:
                self.follower.stop()
            if self.robot:
                self.robot.shutdown()
            if self.robot_manager:
                self.robot_manager.stop()
            print("Done!")


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Path testing script for robot trajectory following",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Track options
    track_group = parser.add_argument_group('Track Options')
    track_group.add_argument(
        '--track-file',
        type=str,
        help='Load track from file (JSON or pickle)'
    )
    track_group.add_argument(
        '--track-type',
        type=str,
        default='circle',
        choices=['circle', 'figure8', 'square', 'line'],
        help='Type of track to generate'
    )
    track_group.add_argument(
        '--track-size',
        type=float,
        default=2.0,
        help='Size parameter for generated track (radius or length)'
    )
    track_group.add_argument(
        '--track-speed',
        type=float,
        default=0.5,
        help='Average speed for generated track (m/s)'
    )
    track_group.add_argument(
        '--save-track',
        type=str,
        help='Save generated track to file'
    )
    
    # Robot options
    robot_group = parser.add_argument_group('Robot Options')
    robot_group.add_argument(
        '--robot-name',
        type=str,
        default=None,
        help='Name of real robot to use (e.g., umh_2). If not specified, uses dummy robot.'
    )
    robot_group.add_argument(
        '--start-x',
        type=float,
        default=None,
        help='Starting X position for dummy robot (default: auto-set to track start)'
    )
    robot_group.add_argument(
        '--start-y',
        type=float,
        default=None,
        help='Starting Y position for dummy robot (default: auto-set to track start)'
    )
    robot_group.add_argument(
        '--max-speed',
        type=float,
        default=0.5,
        help='Maximum speed for dummy robot (m/s)'
    )
    
    # Control options
    control_group = parser.add_argument_group('Control Options')
    control_group.add_argument(
        '--lookahead-distance',
        type=float,
        default=0.5,
        help='Lookahead distance in meters (how far ahead on the track to target)'
    )
    control_group.add_argument(
        '--control-rate-hz',
        type=float,
        default=10.0,
        help='Control loop rate in Hz'
    )
    control_group.add_argument(
        '--offset',
        type=float,
        default=0.0,
        help='Lateral offset from track center in meters (positive = right, negative = left)'
    )
    
    # UI options
    ui_group = parser.add_argument_group('UI Options')
    ui_group.add_argument(
        '--show-ui',
        action='store_true',
        help='Show debug visualization UI'
    )
    ui_group.add_argument(
        '--no-ui',
        dest='show_ui',
        action='store_false',
        help='Run without UI'
    )
    parser.set_defaults(show_ui=True)
    
    args = parser.parse_args(argv)
    
    # Run application
    app = PathTestApp(args)
    app.setup()
    app.run()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
