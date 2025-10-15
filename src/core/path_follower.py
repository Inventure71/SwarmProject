#!/usr/bin/env python3
"""
Path following logic for robot trajectory control.
"""

import math
import time
from typing import Tuple

from .track import Track


class PathFollower:
    """Manages robot path following behavior."""
    
    def __init__(
        self,
        robot,
        track: Track,
        lookahead_distance: float = 0.5,
        control_rate_hz: float = 10.0,
        position_tolerance: float = 0.1,
        offset: float = 0.0
    ):
        """
        Initialize path follower.
        
        Args:
            robot: Robot interface
            track: Track to follow
            lookahead_distance: Lookahead distance in meters (how far ahead to look)
            control_rate_hz: Control loop rate in Hz
            position_tolerance: Position tolerance in meters
            offset: Lateral offset from track center in meters (positive = right)
        """
        self.robot = robot
        self.track = track
        self.lookahead_distance = lookahead_distance
        self.control_rate_hz = control_rate_hz
        self.position_tolerance = position_tolerance
        
        # Validate and set offset
        max_safe_offset = lookahead_distance * 1.5
        if abs(offset) > max_safe_offset:
            print(f"Warning: Offset {offset:.2f}m exceeds safe maximum {max_safe_offset:.2f}m. Clamping.")
            offset = max(-max_safe_offset, min(max_safe_offset, offset))
        self.offset = offset
        
        self.start_time = None
        self.running = False
        self.current_track_index = 0  # Track progress along the path
        self.last_target_x = None  # For detecting stuck situations
        self.last_target_y = None
        self.stuck_counter = 0  # Counter for detecting stuck state
    
    def start(self) -> None:
        """Start following the track."""
        if not self.robot.is_ready():
            raise RuntimeError("Robot is not ready")
        
        self.start_time = time.time()
        self.running = True
    
    def stop(self) -> None:
        """Stop following the track."""
        self.running = False
        self.robot.stop()
    
    def get_current_time(self) -> float:
        """Get current time relative to track start."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
    
    def get_target_position(self) -> Tuple[float, float]:
        """Get current target position based on robot's location."""
        robot_x, robot_y, _ = self.robot.get_position()
        target_x, target_y, updated_index = self.track.get_lookahead_position(
            robot_x, robot_y, self.lookahead_distance, self.current_track_index, self.offset
        )
        self.current_track_index = updated_index
        return (target_x, target_y)
    
    def set_offset(self, offset: float) -> None:
        """
        Update the lateral offset in real-time.
        
        Args:
            offset: Lateral offset in meters. Will be clamped to safe range.
        """
        # Clamp offset to reasonable range to prevent issues
        # Max offset should be less than lookahead to avoid extreme situations
        max_safe_offset = self.lookahead_distance * 1.5
        self.offset = max(-max_safe_offset, min(max_safe_offset, offset))
    
    def is_circular_track(self) -> bool:
        """Check if the track is circular (start and end are at same position)."""
        start_x, start_y = self.track.points[0].x, self.track.points[0].y
        end_x, end_y = self.track.points[-1].x, self.track.points[-1].y
        return math.hypot(end_x - start_x, end_y - start_y) < 0.1
    
    def is_at_end_of_track(self) -> bool:
        """
        Check if robot has reached the end of the track.
        Circular tracks never "end" - they loop forever.
        """
        # Circular tracks loop infinitely
        if self.is_circular_track():
            # Reset track index when we reach the end to loop
            if self.current_track_index >= len(self.track.points) - 1:
                self.current_track_index = 0
            return False  # Never complete for circular tracks
        else:
            # For non-circular tracks, check if we're at the end position
            robot_x, robot_y, _ = self.robot.get_position()
            end_x, end_y = self.track.points[-1].x, self.track.points[-1].y
            dist_to_end = math.hypot(end_x - robot_x, end_y - robot_y)
            return dist_to_end < self.position_tolerance
    
    def update(self) -> bool:
        """
        Update control loop.
        
        Returns:
            True if track is still being followed, False if complete
        """
        if not self.running:
            return False
        
        # Check if track is complete
        if self.is_at_end_of_track():
            self.stop()
            return False
        
        # Get current and target positions
        robot_x, robot_y, robot_yaw = self.robot.get_position()
        target_x, target_y = self.get_target_position()
        
        # Detect stuck situation (target hasn't moved significantly)
        if self.last_target_x is not None and self.last_target_y is not None:
            target_moved = math.hypot(target_x - self.last_target_x, target_y - self.last_target_y)
            robot_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
            
            # If target barely moved and robot is very close to it, we might be stuck
            if target_moved < 0.05 and robot_to_target < self.lookahead_distance * 0.4:
                self.stuck_counter += 1
                
                # If stuck for multiple cycles, temporarily increase lookahead or reduce offset
                if self.stuck_counter > 5:
                    # Force progress by jumping ahead in track index
                    self.current_track_index = min(
                        self.current_track_index + 3,
                        len(self.track.points) - 1
                    )
                    self.stuck_counter = 0  # Reset counter
                    # Get new target with updated index
                    target_x, target_y = self.get_target_position()
            else:
                self.stuck_counter = 0
        
        # Store current target for next iteration
        self.last_target_x = target_x
        self.last_target_y = target_y
        
        # Calculate displacement vector
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Only send command if target is far enough away
        # This prevents oscillation when very close to target
        distance_to_target = math.hypot(dx, dy)
        if distance_to_target > self.position_tolerance * 0.5:
            # Send movement command
            self.robot.move_by_vector(dx, dy)
        
        return True

