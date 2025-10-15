#!/usr/bin/env python3
"""
Test for point-to-point path following.

This test uses a modified path follower that:
1. Waits for the robot to reach each point before moving to the next
2. Displays the movement vector on screen
3. Shows detailed debugging information about reaching each waypoint
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Optional, List, Tuple

# Add parent directory to path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from src.core.robot import create_robot
from src.core.track import Track, TrackGenerator
from src.core.path_follower import PathFollower
from src.core.orientation_calibrator import OrientationCalibrator
from src.tracking.robot_config import RobotManager

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrow, Circle


class PointToPointPathFollower(PathFollower):
    """
    Modified path follower that waits at each waypoint.
    
    Unlike the standard path follower which uses lookahead distance,
    this version targets one waypoint at a time and only moves to the
    next waypoint when the robot has actually reached the current one.
    """
    
    def __init__(
        self,
        robot,
        track: Track,
        waypoint_tolerance: float = 0.05,
        control_rate_hz: float = 10.0
    ):
        """
        Initialize point-to-point path follower.
        
        Args:
            robot: Robot interface
            track: Track to follow
            waypoint_tolerance: Distance threshold to consider waypoint reached (meters)
            control_rate_hz: Control loop rate in Hz
        """
        # Initialize parent but we'll override most of the behavior
        super().__init__(
            robot=robot,
            track=track,
            lookahead_distance=0.1,  # Not used in this mode
            control_rate_hz=control_rate_hz,
            position_tolerance=waypoint_tolerance
        )
        
        self.waypoint_tolerance = waypoint_tolerance
        self.current_waypoint_index = 0  # Which waypoint we're targeting
        self.waypoints_reached = []  # Track which waypoints we've reached
        self.total_waypoints = len(track.points)
        
        # Statistics
        self.waypoint_reach_times = []  # Time taken to reach each waypoint
        self.last_waypoint_time = None
    
    def get_current_waypoint(self) -> Tuple[float, float]:
        """Get the current waypoint we're targeting."""
        if self.current_waypoint_index >= len(self.track.points):
            # Return last point if we're at the end
            point = self.track.points[-1]
        else:
            point = self.track.points[self.current_waypoint_index]
        return (point.x, point.y)
    
    def get_target_position(self) -> Tuple[float, float]:
        """Override parent method to return current waypoint."""
        return self.get_current_waypoint()
    
    def distance_to_current_waypoint(self) -> float:
        """Calculate distance from robot to current waypoint."""
        robot_x, robot_y, _ = self.robot.get_position()
        waypoint_x, waypoint_y = self.get_current_waypoint()
        return math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)
    
    def get_movement_vector(self) -> Tuple[float, float, float]:
        """
        Get the movement vector from robot to current waypoint.
        
        Returns:
            Tuple of (dx, dy, distance)
        """
        robot_x, robot_y, _ = self.robot.get_position()
        waypoint_x, waypoint_y = self.get_current_waypoint()
        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y
        distance = math.hypot(dx, dy)
        return (dx, dy, distance)
    
    def check_waypoint_reached(self) -> bool:
        """Check if current waypoint has been reached."""
        distance = self.distance_to_current_waypoint()
        return distance < self.waypoint_tolerance
    
    def advance_to_next_waypoint(self) -> bool:
        """
        Advance to the next waypoint.
        
        Returns:
            True if there is a next waypoint, False if we've reached the end
        """
        current_time = self.get_current_time()
        
        # Record time taken to reach this waypoint
        if self.last_waypoint_time is not None:
            time_taken = current_time - self.last_waypoint_time
            self.waypoint_reach_times.append(time_taken)
        self.last_waypoint_time = current_time
        
        # Mark as reached
        self.waypoints_reached.append(self.current_waypoint_index)
        
        print(f"✓ Reached waypoint {self.current_waypoint_index + 1}/{self.total_waypoints}")
        
        # Move to next waypoint
        self.current_waypoint_index += 1
        
        # Check if we've completed the path
        if self.current_waypoint_index >= len(self.track.points):
            print(f"🏁 Path complete! Reached all {self.total_waypoints} waypoints")
            return False
        
        return True
    
    def update(self) -> bool:
        """
        Update control loop.
        
        Returns:
            True if path is still being followed, False if complete
        """
        if not self.running:
            return False
        
        # Check if we've reached the current waypoint
        if self.check_waypoint_reached():
            # Waypoint reached! Stop and advance to next
            self.robot.stop()
            time.sleep(0.2)  # Brief pause at waypoint
            
            has_next = self.advance_to_next_waypoint()
            if not has_next:
                # Path complete
                self.stop()
                return False
        
        # Get movement vector to current waypoint
        dx, dy, distance = self.get_movement_vector()
        
        # Only send movement command if we're not at the waypoint
        if distance > self.waypoint_tolerance:
            # Send movement command
            self.robot.move_by_vector(dx, dy)
        
        return True
    
    def get_statistics(self) -> dict:
        """Get statistics about the path following."""
        return {
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': self.total_waypoints,
            'waypoints_reached': len(self.waypoints_reached),
            'waypoint_reach_times': self.waypoint_reach_times,
            'average_time_per_waypoint': sum(self.waypoint_reach_times) / len(self.waypoint_reach_times) if self.waypoint_reach_times else 0
        }


class PointToPointDebugUI:
    """Visualization for point-to-point path following."""
    
    def __init__(
        self,
        path_follower: PointToPointPathFollower,
        update_interval_ms: int = 100,
        trail_length: int = 100
    ):
        """
        Initialize debug UI.
        
        Args:
            path_follower: PointToPointPathFollower instance
            update_interval_ms: UI update interval in milliseconds
            trail_length: Number of past positions to show in trail
        """
        self.follower = path_follower
        self.update_interval_ms = update_interval_ms
        self.trail_length = trail_length
        
        # Data for visualization
        self.robot_trail_x = []
        self.robot_trail_y = []
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Point-to-Point Path Following - Vector Display', fontsize=14, fontweight='bold')
        
        # Plot elements
        self._setup_plot_elements()
        
        # Animation
        self.anim = None
    
    def _setup_plot_elements(self) -> None:
        """Set up plot elements."""
        # All waypoints (track points)
        track_x = [p.x for p in self.follower.track.points]
        track_y = [p.y for p in self.follower.track.points]
        
        # Track path (connecting line)
        self.track_line, = self.ax.plot(
            track_x, track_y, 'b--', linewidth=1, label='Path', alpha=0.3
        )
        
        # All waypoints
        self.all_waypoints = self.ax.scatter(
            track_x, track_y, c='gray', s=50, alpha=0.4, label='Waypoints (pending)', zorder=2
        )
        
        # Reached waypoints (will be updated)
        self.reached_waypoints = self.ax.scatter(
            [], [], c='green', s=60, marker='o', alpha=0.6, label='Waypoints (reached)', zorder=3
        )
        
        # Current target waypoint (larger, highlighted)
        self.current_waypoint = self.ax.scatter(
            [], [], c='red', s=300, marker='*', 
            edgecolors='darkred', linewidths=2,
            label='Current Target', zorder=4
        )
        
        # Tolerance circle around current waypoint
        self.tolerance_circle = None
        
        # Robot trail
        self.robot_trail, = self.ax.plot(
            [], [], 'g-', linewidth=2, label='Robot Trail', alpha=0.7, zorder=1
        )
        
        # Current robot position
        self.robot_pos = self.ax.scatter(
            [], [], c='blue', s=200, marker='o', 
            edgecolors='darkblue', linewidths=2,
            label='Robot', zorder=5
        )
        
        # Movement vector arrow (this is the key feature!)
        self.vector_arrow = None
        
        # Info text
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontfamily='monospace', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9)
        )
        
        # Vector info text (separate, larger)
        self.vector_text = self.ax.text(
            0.02, 0.02, '', transform=self.ax.transAxes,
            verticalalignment='bottom', fontfamily='monospace', fontsize=11,
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9),
            fontweight='bold'
        )
        
        # Set axis limits with padding
        all_x = track_x
        all_y = track_y
        x_range = max(all_x) - min(all_x)
        y_range = max(all_y) - min(all_y)
        padding = 0.3 * max(x_range, y_range)
        
        self.ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
        self.ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
        
        self.ax.legend(loc='upper right', fontsize=10)
    
    def _update_frame(self, frame: int) -> tuple:
        """Update animation frame."""
        # Update the path follower
        still_running = self.follower.update()
        
        if not still_running:
            # Path complete - stop animation
            self.anim.event_source.stop()
        
        # Get current positions
        robot_x, robot_y, robot_yaw = self.follower.robot.get_position()
        waypoint_x, waypoint_y = self.follower.get_current_waypoint()
        
        # Update robot trail
        self.robot_trail_x.append(robot_x)
        self.robot_trail_y.append(robot_y)
        if len(self.robot_trail_x) > self.trail_length:
            self.robot_trail_x.pop(0)
            self.robot_trail_y.pop(0)
        
        self.robot_trail.set_data(self.robot_trail_x, self.robot_trail_y)
        
        # Update robot position
        self.robot_pos.set_offsets([[robot_x, robot_y]])
        
        # Update current target waypoint
        self.current_waypoint.set_offsets([[waypoint_x, waypoint_y]])
        
        # Update reached waypoints
        if self.follower.waypoints_reached:
            reached_x = [self.follower.track.points[i].x for i in self.follower.waypoints_reached]
            reached_y = [self.follower.track.points[i].y for i in self.follower.waypoints_reached]
            self.reached_waypoints.set_offsets(list(zip(reached_x, reached_y)))
        
        # Update tolerance circle
        if self.tolerance_circle is not None:
            self.tolerance_circle.remove()
        self.tolerance_circle = Circle(
            (waypoint_x, waypoint_y),
            self.follower.waypoint_tolerance,
            fill=False,
            edgecolor='red',
            linestyle='--',
            linewidth=2,
            alpha=0.5,
            zorder=3
        )
        self.ax.add_patch(self.tolerance_circle)
        
        # Update movement vector arrow (THE KEY FEATURE)
        if self.vector_arrow is not None:
            self.vector_arrow.remove()
        
        dx, dy, distance = self.follower.get_movement_vector()
        
        if distance > 0.01:  # Only draw if there's meaningful distance
            # Draw the full vector from robot to waypoint
            self.vector_arrow = FancyArrow(
                robot_x, robot_y,
                dx, dy,
                width=0.02, 
                head_width=0.1, 
                head_length=0.08,
                fc='orange', 
                ec='darkorange', 
                alpha=0.8, 
                zorder=6,
                linewidth=2
            )
            self.ax.add_patch(self.vector_arrow)
        
        # Update info text
        current_time = self.follower.get_current_time()
        stats = self.follower.get_statistics()
        
        # Determine robot type
        robot_type = "Dummy" if hasattr(self.follower.robot, 'sim_thread') else "Real"
        
        info_lines = [
            f"🤖 Robot Type: {robot_type}",
            f"⏱️  Time: {current_time:.2f}s",
            f"",
            f"📍 WAYPOINTS:",
            f"  Current: {self.follower.current_waypoint_index + 1}/{self.follower.total_waypoints}",
            f"  Reached: {len(self.follower.waypoints_reached)}",
            f"  Tolerance: {self.follower.waypoint_tolerance:.3f}m",
            f"",
            f"📊 POSITION:",
            f"  Robot:    ({robot_x:+7.3f}, {robot_y:+7.3f})",
            f"  Waypoint: ({waypoint_x:+7.3f}, {waypoint_y:+7.3f})",
            f"  Yaw: {math.degrees(robot_yaw):+6.1f}°",
        ]
        
        if stats['average_time_per_waypoint'] > 0:
            info_lines.append(f"")
            info_lines.append(f"⏱️  AVG TIME/WAYPOINT:")
            info_lines.append(f"  {stats['average_time_per_waypoint']:.2f}s")
        
        self.info_text.set_text("\n".join(info_lines))
        
        # Update vector info text (larger, more prominent)
        vector_info = [
            f"🎯 MOVEMENT VECTOR:",
            f"  ΔX: {dx:+7.3f} m",
            f"  ΔY: {dy:+7.3f} m",
            f"  Distance: {distance:.3f} m",
            f"  Angle: {math.degrees(math.atan2(dy, dx)):+6.1f}°"
        ]
        
        # Add status indicator
        if distance < self.follower.waypoint_tolerance:
            vector_info.insert(0, "✓ AT WAYPOINT")
        else:
            vector_info.insert(0, "→ MOVING TO WAYPOINT")
        
        self.vector_text.set_text("\n".join(vector_info))
        
        return (
            self.robot_trail,
            self.robot_pos,
            self.current_waypoint,
            self.reached_waypoints,
            self.info_text,
            self.vector_text
        )
    
    def run(self) -> None:
        """Run the debug UI."""
        self.anim = animation.FuncAnimation(
            self.fig,
            self._update_frame,
            interval=self.update_interval_ms,
            blit=False,
            cache_frame_data=False
        )
        
        plt.show()


class PointToPointTestApp:
    """Test application for point-to-point path following."""
    
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.robot = None
        self.track = None
        self.follower = None
        self.robot_manager = None
        self.calibrator = None  # For orientation calibration
    
    def setup(self) -> None:
        """Set up the test."""
        # Create or load track
        if self.args.track_file:
            print(f"Loading track from {self.args.track_file}...")
            self.track = Track.load(self.args.track_file)
        else:
            print(f"Generating {self.args.track_type} track...")
            self.track = self._generate_track()
        
        print(f"Track: {self.track.name}")
        print(f"  Points: {len(self.track.points)}")
        print(f"  Duration: {self.track.get_total_duration():.2f}s")
        
        # Create robot
        if self.args.robot_name:
            # Use real robot with OptiTrack tracking
            print(f"Creating real robot: {self.args.robot_name}")
            self.robot_manager = RobotManager()
            
            available = self.robot_manager.get_available_robots()
            if self.args.robot_name not in available:
                raise ValueError(
                    f"Robot '{self.args.robot_name}' not found.\n"
                    f"Available: {', '.join(available)}"
                )
            
            config = self.robot_manager.get_robot_config(self.args.robot_name)
            print(f"  Robot IP: {config['ip']}")
            print(f"  UDP Port: {config['port']}")
            print(f"  Starting OptiTrack listener...")
            
            self.robot = self.robot_manager.create_robot(self.args.robot_name)
            print(f"  Robot ready! Position updates will come from OptiTrack.")
            
            # Perform orientation calibration for real robots
            if not self.args.skip_calibration:
                print("\nPerforming orientation calibration...")
                self.calibrator = OrientationCalibrator(
                    calibration_distance=self.args.calibration_distance,
                    settling_time=self.args.calibration_settling_time
                )
                
                # Check if we should load previous calibration
                calibration_file = Path(f".calibration_{self.args.robot_name}.json")
                if calibration_file.exists() and not self.args.force_calibration:
                    print(f"Found existing calibration file: {calibration_file}")
                    if self.calibrator.load_calibration(str(calibration_file)):
                        print(f"  Loaded calibration: offset = {self.calibrator.get_offset():.4f} rad "
                              f"({math.degrees(self.calibrator.get_offset()):.2f}°)")
                        user_input = input("Use this calibration? (y/n, default=y): ").strip().lower()
                        if user_input not in ['n', 'no']:
                            # Use loaded calibration
                            self.robot.set_orientation_calibration(
                                self.calibrator.get_offset(), 
                                enabled=True
                            )
                            print("Using loaded calibration.")
                        else:
                            # Run new calibration
                            self._run_calibration(calibration_file)
                    else:
                        # Failed to load, run new calibration
                        self._run_calibration(calibration_file)
                else:
                    # No existing calibration or forced recalibration
                    self._run_calibration(calibration_file)
            else:
                print("\nSkipping orientation calibration (--skip-calibration flag set)")
                print("WARNING: Movement commands may not align with OptiTrack coordinates!")
        else:
            # Use dummy robot
            print("Creating dummy robot (simulation mode)")
            
            start_x = self.args.start_x if self.args.start_x is not None else self.track.points[0].x
            start_y = self.args.start_y if self.args.start_y is not None else self.track.points[0].y
            
            print(f"  Starting position: ({start_x:.2f}, {start_y:.2f})")
            
            self.robot = create_robot(
                "dummy",
                initial_x=start_x,
                initial_y=start_y,
                max_speed=self.args.max_speed
            )
            
            # Wait for ready
            print("Waiting for robot to be ready...")
            timeout = 10.0
            start = time.time()
            while not self.robot.is_ready():
                if time.time() - start > timeout:
                    raise RuntimeError("Robot failed to become ready")
                time.sleep(0.1)
            print("Robot ready!")
        
        # Create point-to-point path follower
        print(f"\nCreating point-to-point path follower...")
        print(f"  Waypoint tolerance: {self.args.tolerance}m")
        print(f"  Control rate: {self.args.control_rate_hz}Hz")
        
        self.follower = PointToPointPathFollower(
            self.robot,
            self.track,
            waypoint_tolerance=self.args.tolerance,
            control_rate_hz=self.args.control_rate_hz
        )
    
    def _run_calibration(self, calibration_file: Path) -> None:
        """Run orientation calibration and save results."""
        print("\nWaiting for OptiTrack position to stabilize...")
        time.sleep(2.0)  # Give OptiTrack time to get good position data
        
        try:
            offset = self.calibrator.calibrate(self.robot, verbose=True)
            
            # Apply calibration to robot
            self.robot.set_orientation_calibration(offset, enabled=True)
            
            # Save calibration
            self.calibrator.save_calibration(str(calibration_file))
            print(f"Calibration saved to {calibration_file}")
            
        except RuntimeError as e:
            print(f"\nCALIBRATION FAILED: {e}")
            print("Continuing WITHOUT calibration. Movement may be inaccurate!")
            user_input = input("Continue anyway? (y/n): ").strip().lower()
            if user_input not in ['y', 'yes']:
                raise
    
    def _generate_track(self) -> Track:
        """Generate a track based on arguments."""
        if self.args.track_type == "circle":
            return TrackGenerator.generate_circle(
                radius=self.args.track_size,
                speed=self.args.track_speed,
                num_points=self.args.num_waypoints
            )
        elif self.args.track_type == "figure8":
            return TrackGenerator.generate_figure_eight(
                radius=self.args.track_size,
                speed=self.args.track_speed,
                num_points=self.args.num_waypoints
            )
        elif self.args.track_type == "square":
            # Square uses points_per_side, so divide total waypoints by 4 sides
            points_per_side = max(2, self.args.num_waypoints // 4)
            return TrackGenerator.generate_square(
                side_length=self.args.track_size * 2,
                speed=self.args.track_speed,
                points_per_side=points_per_side
            )
        elif self.args.track_type == "line":
            return TrackGenerator.generate_straight_line(
                length=self.args.track_size * 2,
                speed=self.args.track_speed,
                num_points=self.args.num_waypoints
            )
        else:
            raise ValueError(f"Unknown track type: {self.args.track_type}")
    
    def run(self) -> None:
        """Run the test."""
        try:
            print("\n" + "="*60)
            print("STARTING POINT-TO-POINT PATH FOLLOWING TEST")
            print("="*60)
            
            # Start path following
            self.follower.start()
            
            if self.args.show_ui:
                # Run with UI
                print("\nStarting visualization UI...")
                print("The orange arrow shows the movement vector!")
                ui = PointToPointDebugUI(self.follower)
                ui.run()
            else:
                # Run without UI
                print("\nRunning without UI...")
                dt = 1.0 / self.args.control_rate_hz
                
                while self.follower.running:
                    still_running = self.follower.update()
                    if not still_running:
                        break
                    time.sleep(dt)
            
            # Print statistics
            print("\n" + "="*60)
            print("TEST COMPLETE - STATISTICS")
            print("="*60)
            stats = self.follower.get_statistics()
            print(f"Total waypoints: {stats['total_waypoints']}")
            print(f"Waypoints reached: {stats['waypoints_reached']}")
            if stats['waypoint_reach_times']:
                print(f"Average time per waypoint: {stats['average_time_per_waypoint']:.2f}s")
                print(f"Total time: {sum(stats['waypoint_reach_times']):.2f}s")
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            # Cleanup
            print("\nCleaning up...")
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
        description="Point-to-Point Path Following Test",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Track options
    track_group = parser.add_argument_group('Track Options')
    track_group.add_argument(
        '--track-file',
        type=str,
        help='Load track from file'
    )
    track_group.add_argument(
        '--track-type',
        type=str,
        default='square',
        choices=['circle', 'figure8', 'square', 'line'],
        help='Type of track to generate'
    )
    track_group.add_argument(
        '--track-size',
        type=float,
        default=1.0,
        help='Size parameter for track (radius or length in meters)'
    )
    track_group.add_argument(
        '--track-speed',
        type=float,
        default=0.3,
        help='Track speed (m/s) - only affects timing, not actual robot speed'
    )
    track_group.add_argument(
        '--num-waypoints',
        type=int,
        default=20,
        help='Number of waypoints to generate in the track'
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
        help='Starting X position for dummy robot'
    )
    robot_group.add_argument(
        '--start-y',
        type=float,
        default=None,
        help='Starting Y position for dummy robot'
    )
    robot_group.add_argument(
        '--max-speed',
        type=float,
        default=0.3,
        help='Maximum speed for dummy robot (m/s)'
    )
    
    # Calibration options
    calibration_group = parser.add_argument_group('Calibration Options')
    calibration_group.add_argument(
        '--skip-calibration',
        action='store_true',
        help='Skip orientation calibration (not recommended for real robots)'
    )
    calibration_group.add_argument(
        '--force-calibration',
        action='store_true',
        help='Force new calibration even if saved calibration exists'
    )
    calibration_group.add_argument(
        '--calibration-distance',
        type=float,
        default=0.5,
        help='Distance to move during calibration (meters)'
    )
    calibration_group.add_argument(
        '--calibration-settling-time',
        type=float,
        default=2.0,
        help='Time to wait for robot movement during calibration (seconds)'
    )
    
    # Control options
    control_group = parser.add_argument_group('Control Options')
    control_group.add_argument(
        '--tolerance',
        type=float,
        default=0.05,
        help='Distance tolerance for reaching waypoints (meters)'
    )
    control_group.add_argument(
        '--control-rate-hz',
        type=float,
        default=10.0,
        help='Control loop rate in Hz'
    )
    
    # UI options
    ui_group = parser.add_argument_group('UI Options')
    ui_group.add_argument(
        '--show-ui',
        action='store_true',
        default=True,
        help='Show visualization UI'
    )
    ui_group.add_argument(
        '--no-ui',
        dest='show_ui',
        action='store_false',
        help='Run without UI'
    )
    
    args = parser.parse_args(argv)
    
    # Run test
    app = PointToPointTestApp(args)
    app.setup()
    app.run()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

