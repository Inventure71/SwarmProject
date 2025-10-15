#!/usr/bin/env python3
"""
Robot interface and implementations.

Architecture:
- DummyRobot: Simulated robot with internal position tracking
- Robot: Real robot with external position tracking
"""

import threading
import zmq
import json
from typing import Tuple
from abc import ABC, abstractmethod
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

        self.mecanum = False # TODO: Make this configurable

        context = zmq.Context()
        self.socket = context.socket(zmq.PUSH)
        self.socket.connect(f"tcp://{self.robot_ip}:5555")
    
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
        """Command robot to move by vector."""
        with self.lock:
            self.last_dx = dx
            self.last_dy = dy
        # TODO: Send to actual robot hardware
        self.send_command_json(json.dumps({"command": "move", "dx": dx, "dy": dy, "mecanum": self.mecanum}))
        
        return True
    
    def get_last_command(self) -> Tuple[float, float]:
        """Get last commanded movement vector."""
        with self.lock:
            return (self.last_dx, self.last_dy)
    
    def stop(self) -> None:
        """Stop robot movement."""
        with self.lock:
            self.last_dx = 0.0
            self.last_dy = 0.0
    
    def shutdown(self) -> None:
        """Shutdown robot."""
        with self.lock:
            self._ready = False
            
    
    def send_command_json(self, command_json: str) -> None:
        """Send command to robot."""
        self.socket.send_string(command_json)


def create_robot(robot_type: str = "dummy", **kwargs):
    """Factory function to create robot instance."""
    robot_type = robot_type.lower()
    if robot_type == "dummy":
        return DummyRobot(**kwargs)
    elif robot_type in ["real", "ros2"]:
        return Robot(**kwargs)
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
