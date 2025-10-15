#!/usr/bin/env python3
"""
Track generation and management for robot path following.
"""

import json
import math
import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


@dataclass
class TrackPoint:
    """Represents a point on the track."""
    x: float
    y: float
    timestamp: float  # Time when robot should reach this point (seconds)


class Track:
    """Represents a path/track for the robot to follow."""
    
    def __init__(self, points: List[TrackPoint], name: str = "unnamed"):
        """
        Initialize a track.
        Args:
            points: List of track points
            name: Name/description of the track
        """
        self.points = points
        self.name = name
        self._validate()
    
    def _validate(self) -> None:
        """Validate track consistency."""
        if len(self.points) < 2:
            raise ValueError("Track must have at least 2 points")
        
        # Check timestamps are monotonically increasing
        for i in range(1, len(self.points)):
            if self.points[i].timestamp <= self.points[i-1].timestamp:
                raise ValueError("Track timestamps must be monotonically increasing")
    
    def get_total_duration(self) -> float:
        """Get total duration of the track."""
        return self.points[-1].timestamp - self.points[0].timestamp
    
    def find_closest_point_index(self, x: float, y: float, start_from: int = 0, 
                                 backward_window: int = 5, forward_window: int = 20) -> Tuple[int, float]:
        """
        Find the closest point on the track to the given position, using a sliding search window.
        
        Args:
            x: X coordinate
            y: Y coordinate
            start_from: Index to start searching from (for overlapping paths)
            backward_window: How many points backward to check (for tolerance)
            forward_window: How many points forward to check (prevents jumping to distant overlaps)
        
        Returns:
            Tuple of (index, distance) where index is the closest point index
        """
        min_dist = float('inf')
        closest_idx = start_from
        
        # Search in a sliding window around current position
        search_start = max(0, start_from - backward_window)
        search_end = min(len(self.points), start_from + forward_window)
        
        for i in range(search_start, search_end):
            point = self.points[i]
            dist = math.hypot(point.x - x, point.y - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx, min_dist
    
    def _get_smoothed_direction(self, point_index: int, t: float = 0.5, window: int = 3) -> Tuple[float, float]:
        """
        Get smoothed track direction at a point by averaging nearby segments.
        This prevents issues at sharp corners.
        
        Args:
            point_index: Index of the track point
            t: Interpolation factor within the segment (0 to 1)
            window: Number of segments to average (forward and backward)
        
        Returns:
            Tuple of (normalized_dx, normalized_dy)
        """
        directions = []
        
        # Collect directions from nearby segments
        start_idx = max(0, point_index - window)
        end_idx = min(len(self.points) - 2, point_index + window)
        
        for i in range(start_idx, end_idx + 1):
            if i < len(self.points) - 1:
                p1 = self.points[i]
                p2 = self.points[i + 1]
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length = math.hypot(dx, dy)
                
                if length > 1e-6:  # Avoid division by zero
                    # Weight closer segments more heavily
                    weight = 1.0 / (1.0 + abs(i - point_index))
                    directions.append((dx / length * weight, dy / length * weight, weight))
        
        if not directions:
            # Fallback to current segment
            if point_index < len(self.points) - 1:
                p1 = self.points[point_index]
                p2 = self.points[point_index + 1]
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length = math.hypot(dx, dy)
                return (dx / length, dy / length) if length > 1e-6 else (1.0, 0.0)
            return (1.0, 0.0)
        
        # Weighted average of directions
        total_weight = sum(w for _, _, w in directions)
        avg_dx = sum(dx * w for dx, dy, w in directions) / total_weight
        avg_dy = sum(dy * w for dx, dy, w in directions) / total_weight
        
        # Normalize
        length = math.hypot(avg_dx, avg_dy)
        if length > 1e-6:
            return (avg_dx / length, avg_dy / length)
        return (1.0, 0.0)
    
    def get_lookahead_position(self, robot_x: float, robot_y: float, lookahead_distance: float, 
                               current_track_index: int = 0, offset: float = 0.0) -> Tuple[float, float, int]:
        """
        Get target position based on robot's current position and lookahead distance.
        
        Args:
            robot_x: Current robot X position
            robot_y: Current robot Y position
            lookahead_distance: Lookahead distance in meters
            current_track_index: Current position on track (for overlapping paths)
            offset: Lateral offset from track center in meters (positive = right, negative = left)
        
        Returns:
            Tuple of (target_x, target_y, updated_track_index)
        """
        # Find closest point on track, searching forward from current position
        closest_idx, _ = self.find_closest_point_index(robot_x, robot_y, start_from=current_track_index)
        
        # For offset paths, increase lookahead slightly to avoid getting stuck at corners
        # The inside path (negative offset) has tighter turns, so needs more lookahead
        effective_lookahead = lookahead_distance
        if offset != 0.0:
            # Increase lookahead proportionally to offset magnitude
            # This helps avoid getting stuck at sharp corners
            effective_lookahead = lookahead_distance * (1.0 + abs(offset) * 0.5)
        
        # Walk forward along the track to find the lookahead point
        accumulated_distance = 0.0
        current_idx = closest_idx
        
        while current_idx < len(self.points) - 1:
            p1 = self.points[current_idx]
            p2 = self.points[current_idx + 1]
            
            segment_length = math.hypot(p2.x - p1.x, p2.y - p1.y)
            
            if accumulated_distance + segment_length >= effective_lookahead:
                # Target is on this segment
                remaining = effective_lookahead - accumulated_distance
                t = remaining / segment_length if segment_length > 0 else 0
                
                target_x = p1.x + t * (p2.x - p1.x)
                target_y = p1.y + t * (p2.y - p1.y)
                
                # Apply lateral offset perpendicular to track direction
                if offset != 0.0:
                    # Use smoothed direction to handle corners better
                    dx, dy = self._get_smoothed_direction(current_idx, t)
                    
                    # Perpendicular direction (rotate 90° clockwise for positive = right)
                    perp_x = dy
                    perp_y = -dx
                    
                    # Apply offset
                    offset_target_x = target_x + perp_x * offset
                    offset_target_y = target_y + perp_y * offset
                    
                    # Validation: ensure offset target is ahead of robot
                    # Calculate vector from robot to offset target
                    to_target_x = offset_target_x - robot_x
                    to_target_y = offset_target_y - robot_y
                    
                    # Check if target is ahead (dot product with track direction)
                    dot_product = to_target_x * dx + to_target_y * dy
                    
                    # If target would be behind or too close, use centerline target instead
                    target_dist = math.hypot(to_target_x, to_target_y)
                    if dot_product > 0 and target_dist > lookahead_distance * 0.3:
                        target_x = offset_target_x
                        target_y = offset_target_y
                    # else: fall back to centerline target (no offset applied)
                
                return (target_x, target_y, closest_idx)
            
            accumulated_distance += segment_length
            current_idx += 1
        
        # If we've reached the end, return the last point (with offset if applicable)
        target_x = self.points[-1].x
        target_y = self.points[-1].y
        
        # Apply offset using smoothed direction near the end
        if offset != 0.0 and len(self.points) >= 2:
            dx, dy = self._get_smoothed_direction(len(self.points) - 2, 1.0)
            
            perp_x = dy
            perp_y = -dx
            
            offset_target_x = target_x + perp_x * offset
            offset_target_y = target_y + perp_y * offset
            
            # Same validation as above
            to_target_x = offset_target_x - robot_x
            to_target_y = offset_target_y - robot_y
            dot_product = to_target_x * dx + to_target_y * dy
            target_dist = math.hypot(to_target_x, to_target_y)
            
            if dot_product > 0 and target_dist > lookahead_distance * 0.3:
                target_x = offset_target_x
                target_y = offset_target_y
        
        return (target_x, target_y, closest_idx)
    
    def save(self, filepath: str) -> None:
        """Save track to file."""
        data = {
            'name': self.name,
            'points': [(p.x, p.y, p.timestamp) for p in self.points]
        }
        
        path = Path(filepath)
        if path.suffix == '.json':
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
        else:
            with open(filepath, 'wb') as f:
                pickle.dump(data, f)
    
    @classmethod
    def load(cls, filepath: str) -> 'Track':
        """Load track from file."""
        path = Path(filepath)
        
        if path.suffix == '.json':
            with open(filepath, 'r') as f:
                data = json.load(f)
        else:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
        
        points = [TrackPoint(x, y, t) for x, y, t in data['points']]
        return cls(points, data['name'])


class TrackGenerator:
    """Factory for generating predefined tracks."""
    
    @staticmethod
    def generate_circle(
        radius: float = 2.0,
        num_points: int = 50,
        speed: float = 0.5,
        center_x: float = 0.0,
        center_y: float = 0.0
    ) -> Track:
        """Generate a circular track."""
        points = []
        circumference = 2 * math.pi * radius
        total_time = circumference / speed
        
        for i in range(num_points):
            angle = (2 * math.pi * i) / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            t = (total_time * i) / num_points
            points.append(TrackPoint(x, y, t))
        
        # Add final point to close the loop
        points.append(TrackPoint(
            center_x + radius,
            center_y,
            total_time
        ))
        
        return Track(points, name="Circle")
    
    @staticmethod
    def generate_figure_eight(
        radius: float = 1.5,
        num_points: int = 100,
        speed: float = 0.4
    ) -> Track:
        """Generate a figure-eight track."""
        points = []
        
        # Approximate path length (two circles)
        path_length = 2 * (2 * math.pi * radius)
        total_time = path_length / speed
        
        for i in range(num_points + 1):
            t = (2 * math.pi * i) / num_points
            # Lemniscate (figure-8) parametric equations
            x = radius * math.sin(t)
            y = radius * math.sin(t) * math.cos(t)
            timestamp = (total_time * i) / num_points
            points.append(TrackPoint(x, y, timestamp))
        
        return Track(points, name="Figure-Eight")
    
    @staticmethod
    def generate_square(
        side_length: float = 3.0,
        points_per_side: int = 10,
        speed: float = 0.5
    ) -> Track:
        """Generate a square track."""
        points = []
        half = side_length / 2
        
        # Define corners
        corners = [
            (-half, -half),
            (half, -half),
            (half, half),
            (-half, half),
            (-half, -half)  # Close the loop
        ]
        
        total_time = (4 * side_length) / speed
        point_count = 0
        total_points = 4 * points_per_side
        
        for i in range(len(corners) - 1):
            x1, y1 = corners[i]
            x2, y2 = corners[i + 1]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                timestamp = (total_time * point_count) / total_points
                points.append(TrackPoint(x, y, timestamp))
                point_count += 1
        
        # Add final point
        points.append(TrackPoint(corners[-1][0], corners[-1][1], total_time))
        
        return Track(points, name="Square")
    
    @staticmethod
    def generate_straight_line(
        length: float = 5.0,
        num_points: int = 20,
        speed: float = 0.5,
        angle: float = 0.0
    ) -> Track:
        """Generate a straight line track."""
        points = []
        total_time = length / speed
        
        for i in range(num_points + 1):
            t = i / num_points
            x = t * length * math.cos(angle)
            y = t * length * math.sin(angle)
            timestamp = t * total_time
            points.append(TrackPoint(x, y, timestamp))
        
        return Track(points, name="Straight-Line")

