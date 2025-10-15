#!/usr/bin/env python3
"""
Robot interface and implementations.

Architecture:
- DummyRobot: Simulated robot with internal position tracking
- Robot: Real robot with external position tracking
"""

import json
import math
import threading
from abc import ABC, abstractmethod
from typing import Tuple

import zmq
from .dummy_robot import DummyRobot


class RobotInterface(ABC):
    """Abstract interface for robot control."""
    
    @abstractmethod
    def is_ready(self) -> bool:
        pass
    
    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        pass
    
    @abstractmethod
    def move_by_vector(self, dx: float, dy: float) -> bool:
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        pass


class Robot(RobotInterface):
    """
    Real robot implementation for external tracking.
    
    Position is updated externally via update_position().
    Movement commands are sent via move_by_vector().
    """
    
    def __init__(self, initial_x: float = 0.0, initial_y: float = 0.0, initial_yaw: float = 0.0, robot_ip: str = "localhost"):
        """
        Initialize robot.
        
        Args:
            initial_x: Initial x position in meters
            initial_y: Initial y position in meters
            initial_yaw: Initial yaw angle in radians
            robot_ip: IP address of the robot for sending movement commands
        """
        self.lock = threading.Lock()
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw
        self.last_dx = 0.0
        self.last_dy = 0.0
        self._ready = True
        self.robot_ip = robot_ip

        self.mecanum = True  # TODO: Make this configurable
        
        # Orientation calibration support
        self.orientation_offset = 0.0  # Rotation offset in radians
        self.use_orientation_calibration = False

        self.socket = self._create_command_socket()

    def _create_command_socket(self):
        """Create and connect the ZeroMQ socket used for command delivery."""
        context = zmq.Context.instance()
        socket = context.socket(zmq.PUSH)
        socket.connect(f"tcp://{self.robot_ip}:5555")
        return socket
    
    def is_ready(self) -> bool:
        """Check if robot is ready."""
        return self._ready
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (x, y, yaw)."""
        with self.lock:
            return (self.x, self.y, self.yaw)
    
    def update_position(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Update robot position from external tracking system."""
        with self.lock:
            self.x = x
            self.y = y
            self.yaw = yaw
    
    def move_by_vector(self, dx: float, dy: float) -> bool:
        """
        Command robot to move by vector.
        
        If orientation calibration is enabled, the vector is transformed
        from OptiTrack coordinates to robot's local coordinates.
        
        Args:
            dx: X displacement in OptiTrack frame (or robot frame if not calibrated)
            dy: Y displacement in OptiTrack frame (or robot frame if not calibrated)
        """
        dx_robot, dy_robot = self._transform_if_calibrated(dx, dy)
        self._remember_last_command(dx, dy)
        self._send_move_command(dx_robot, dy_robot)
        return True
    
    def get_last_command(self) -> Tuple[float, float]:
        """Get last commanded movement vector."""
        with self.lock:
            return (self.last_dx, self.last_dy)
    
    def stop(self) -> None:
        """Stop robot movement."""
        self._remember_last_command(0.0, 0.0)
        self.send_command({"command": "stop"})
    
    def shutdown(self) -> None:
        """Shutdown robot."""
        with self.lock:
            self._ready = False
        if hasattr(self, "socket") and not self.socket.closed:
            self.socket.close(0)
    
    def send_command(self, command: dict) -> None:
        """Send a dictionary command to the robot as JSON."""
        self.send_command_json(json.dumps(command))

    def send_command_json(self, command_json: str) -> None:
        """Send command to robot."""
        self.socket.send_string(command_json)
    
    def set_orientation_calibration(self, offset: float, enabled: bool = True) -> None:
        """
        Set orientation calibration offset.
        
        Args:
            offset: Rotation offset in radians between OptiTrack and robot frame
            enabled: Whether to use the calibration
        """
        with self.lock:
            self.orientation_offset = offset
            self.use_orientation_calibration = enabled
    
    def _transform_vector(self, dx: float, dy: float) -> Tuple[float, float]:
        """
        Transform vector from OptiTrack frame to robot frame.
        
        Rotates the vector by the negative of the orientation offset.
        """
        cos_theta = math.cos(-self.orientation_offset)
        sin_theta = math.sin(-self.orientation_offset)
        
        dx_robot = dx * cos_theta - dy * sin_theta
        dy_robot = dx * sin_theta + dy * cos_theta
        
        return (dx_robot, dy_robot)

    def _transform_if_calibrated(self, dx: float, dy: float) -> Tuple[float, float]:
        """Apply calibration transform if enabled."""
        if not self.use_orientation_calibration:
            return dx, dy
        return self._transform_vector(dx, dy)

    def _remember_last_command(self, dx: float, dy: float) -> None:
        """Persist the original command vector for diagnostics."""
        with self.lock:
            self.last_dx = dx
            self.last_dy = dy

    def _send_move_command(self, dx: float, dy: float) -> None:
        """Send the prepared move command to the robot."""
        self.send_command({
            "command": "move",
            "dx": dx,
            "dy": dy,
            "mecanum": self.mecanum,
        })


def create_robot(robot_type: str = "dummy", **kwargs):
    """Factory function to create robot instance."""
    robot_type = robot_type.lower()
    if robot_type == "dummy":
        return DummyRobot(**kwargs)
    elif robot_type in ["real", "ros2"]:
        return Robot(**kwargs)
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
