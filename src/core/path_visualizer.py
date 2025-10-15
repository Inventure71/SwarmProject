#!/usr/bin/env python3
"""
Real-time visualization for path following debugging.
"""

import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrow
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from path_follower import PathFollower


class PathFollowingDebugUI:
    """Real-time visualization for path following debugging."""
    
    def __init__(
        self,
        path_follower: 'PathFollower',
        update_interval_ms: int = 100,
        trail_length: int = 50
    ):
        """
        Initialize debug UI.
        
        Args:
            path_follower: PathFollower instance to visualize
            update_interval_ms: UI update interval in milliseconds
            trail_length: Number of past positions to show in trail
        """
        self.follower = path_follower
        self.update_interval_ms = update_interval_ms
        self.trail_length = trail_length
        
        # Data for visualization
        self.robot_trail_x = []
        self.robot_trail_y = []
        
        # Offset control
        self.offset_step = 0.1  # meters per key press
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Path Following - Debug View')
        
        # Plot elements
        self._setup_plot_elements()
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        
        # Animation
        self.anim = None
    
    def _setup_plot_elements(self) -> None:
        """Set up plot elements."""
        # Track path
        track_x = [p.x for p in self.follower.track.points]
        track_y = [p.y for p in self.follower.track.points]
        self.track_line, = self.ax.plot(
            track_x, track_y, 'b-', linewidth=2, label='Track', alpha=0.5
        )
        
        # Track points
        self.track_points = self.ax.scatter(
            track_x, track_y, c='blue', s=20, alpha=0.3, label='Track Points'
        )
        
        # Robot trail
        self.robot_trail, = self.ax.plot(
            [], [], 'g-', linewidth=1.5, label='Robot Trail', alpha=0.7
        )
        
        # Current robot position
        self.robot_pos = self.ax.scatter(
            [], [], c='green', s=200, marker='o', label='Robot', zorder=5
        )
        
        # Target position
        self.target_pos = self.ax.scatter(
            [], [], c='red', s=150, marker='x', linewidths=3, label='Target', zorder=5
        )
        
        # Direction arrow
        self.direction_arrow = None
        
        # Info text
        self.info_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        # Set axis limits with some padding
        all_x = track_x
        all_y = track_y
        x_range = max(all_x) - min(all_x)
        y_range = max(all_y) - min(all_y)
        padding = 0.2 * max(x_range, y_range)
        
        self.ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
        self.ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
        
        self.ax.legend(loc='upper right')
    
    def _on_key_press(self, event) -> None:
        """Handle keyboard input for offset control."""
        if event.key == 'left' or event.key == 'a':
            # Move left (decrease offset)
            self.follower.set_offset(self.follower.offset - self.offset_step)
            print(f"Offset: {self.follower.offset:.2f}m (←)")
        elif event.key == 'right' or event.key == 'd':
            # Move right (increase offset)
            self.follower.set_offset(self.follower.offset + self.offset_step)
            print(f"Offset: {self.follower.offset:.2f}m (→)")
        elif event.key == 'c':
            # Center (reset offset to 0)
            self.follower.set_offset(0.0)
            print(f"Offset: {self.follower.offset:.2f}m (centered)")
    
    def _update_frame(self, frame: int) -> tuple:
        """Update animation frame."""
        # Update the path follower (this actually moves the robot!)
        self.follower.update()
        
        # Get current positions
        robot_x, robot_y, robot_yaw = self.follower.robot.get_position()
        target_x, target_y = self.follower.get_target_position()
        
        # Update robot trail
        self.robot_trail_x.append(robot_x)
        self.robot_trail_y.append(robot_y)
        if len(self.robot_trail_x) > self.trail_length:
            self.robot_trail_x.pop(0)
            self.robot_trail_y.pop(0)
        
        self.robot_trail.set_data(self.robot_trail_x, self.robot_trail_y)
        
        # Update robot position
        self.robot_pos.set_offsets([[robot_x, robot_y]])
        
        # Update target position
        self.target_pos.set_offsets([[target_x, target_y]])
        
        # Update direction arrow
        if self.direction_arrow is not None:
            self.direction_arrow.remove()
        
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)
        
        if dist > 0.01:  # Only draw arrow if there's meaningful distance
            arrow_length = min(dist, 0.3)  # Cap arrow length
            self.direction_arrow = FancyArrow(
                robot_x, robot_y,
                (dx / dist) * arrow_length,
                (dy / dist) * arrow_length,
                width=0.05, head_width=0.15, head_length=0.1,
                fc='orange', ec='darkorange', alpha=0.7, zorder=4
            )
            self.ax.add_patch(self.direction_arrow)
        
        # Update info text
        current_time = self.follower.get_current_time()
        closest_idx = self.follower.current_track_index
        _, dist_from_track = self.follower.track.find_closest_point_index(
            robot_x, robot_y, start_from=closest_idx, backward_window=3, forward_window=3
        )
        progress = (closest_idx / len(self.follower.track.points)) * 100
        
        # Calculate lap count for circular tracks
        is_circular = self.follower.is_circular_track()
        if is_circular:
            # Estimate laps based on time and track duration
            track_duration = self.follower.track.get_total_duration()
            laps_completed = int(current_time / track_duration) if track_duration > 0 else 0
            progress_str = f"Lap {laps_completed + 1} - {progress:.1f}% ({closest_idx}/{len(self.follower.track.points)} pts)"
        else:
            progress_str = f"{progress:.1f}% ({closest_idx}/{len(self.follower.track.points)} pts)"
        
        # Determine robot type for display
        robot_type = "Dummy" if hasattr(self.follower.robot, 'sim_thread') else "Real"
        robot_type_color = "🤖" if robot_type == "Dummy" else "🏎️"
        
        # Build info string with enhanced debugging
        info_lines = []
        if is_circular:
            info_lines.append(f"Mode: Looping")
        
        info_lines.append(f"{robot_type_color} Robot Type: {robot_type}")
        info_lines.append(f"Time: {current_time:.2f}s")
        info_lines.append(f"Progress: {progress_str}")
        info_lines.append(f"")  # Blank line
        info_lines.append(f"📍 POSITION:")
        info_lines.append(f"  Robot:  ({robot_x:+7.3f}, {robot_y:+7.3f})")
        info_lines.append(f"  Target: ({target_x:+7.3f}, {target_y:+7.3f})")
        info_lines.append(f"  Yaw: {math.degrees(robot_yaw):+6.1f}°")
        info_lines.append(f"")  # Blank line
        info_lines.append(f"📊 METRICS:")
        info_lines.append(f"  Distance to target: {dist:.3f}m")
        info_lines.append(f"  Distance from track: {dist_from_track:.3f}m")
        info_lines.append(f"  Lookahead: {self.follower.lookahead_distance:.2f}m")
        info_lines.append(f"  Offset: {self.follower.offset:+.2f}m")
        info_lines.append(f"")  # Blank line
        info_lines.append(f"🎮 CONTROLS:")
        info_lines.append(f"  ← → (or A D) to adjust offset")
        info_lines.append(f"  C to center")
        
        info_str = "\n".join(info_lines)
        self.info_text.set_text(info_str)
        
        # Return all modified artists
        return (
            self.robot_trail,
            self.robot_pos,
            self.target_pos,
            self.info_text
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

