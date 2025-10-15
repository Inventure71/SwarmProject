#!/usr/bin/env python3
"""
Robot configuration loader and tracker integration.

This module provides utilities to:
1. Load robot configuration from config.json
2. Create and manage real robots with OptiTrack tracking
3. Integrate with existing PathTesting scripts
"""

import json
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Dict

# Add parent directory to path to import modules
sys.path.append(str(Path(__file__).resolve().parent.parent))
from tracking.robot_tracker import RobotTracker
from core.robot import Robot


class RobotManager:
    """
    Manages real robots with external tracking.
    
    This class:
    - Loads configuration from config.json
    - Starts RobotTracker to listen for position updates
    - Provides robot instances that are automatically updated
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize robot manager.
        
        Args:
            config_path: Path to config.json. If None, uses default location.
        """
        if config_path is None:
            # Default: config.json in config directory
            config_path = Path(__file__).parent.parent.parent / "config" / "config.json"
        
        self.config_path = Path(config_path)
        self.config = self._load_config()
        self.tracker: Optional[RobotTracker] = None
        self._started = False
    
    def _load_config(self) -> Dict:
        """Load configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
            return config
        except FileNotFoundError:
            raise FileNotFoundError(
                f"Config file not found: {self.config_path}\n"
                f"Please create config.json in the project root."
            )
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in config file: {e}")
    
    def get_available_robots(self) -> list[str]:
        """Get list of available robot names from config."""
        return list(self.config.get("ROBOT_CONFIG", {}).keys())
    
    def get_robot_config(self, robot_name: str) -> dict:
        """
        Get configuration for a specific robot.
        
        Args:
            robot_name: Name of the robot (e.g., 'umh_2')
        
        Returns:
            Dictionary with 'ip' and 'port' keys
        
        Raises:
            KeyError: If robot name not found in config
        """
        robot_config = self.config.get("ROBOT_CONFIG", {})
        if robot_name not in robot_config:
            available = ", ".join(robot_config.keys())
            raise KeyError(
                f"Robot '{robot_name}' not found in config.\n"
                f"Available robots: {available}"
            )
        return robot_config[robot_name]
    
    def get_robot_port(self, robot_name: str) -> int:
        """
        Get UDP port for a specific robot.
        
        Args:
            robot_name: Name of the robot (e.g., 'umh_2')
        
        Returns:
            UDP port number
        
        Raises:
            KeyError: If robot name not found in config
        """
        return self.get_robot_config(robot_name)["port"]
    
    def get_robot_ip(self, robot_name: str) -> str:
        """
        Get IP address for a specific robot.
        
        Args:
            robot_name: Name of the robot (e.g., 'umh_2')
        
        Returns:
            IP address string
        
        Raises:
            KeyError: If robot name not found in config
        """
        return self.get_robot_config(robot_name)["ip"]
    
    def create_robot(self, robot_name: str, start_tracking: bool = True) -> Robot:
        """
        Create a real robot with automatic position tracking.
        
        Args:
            robot_name: Name of the robot (e.g., 'umh_2')
            start_tracking: If True, automatically start the tracker
        
        Returns:
            Robot instance that will be updated by RobotTracker
        
        Example:
            >>> manager = RobotManager()
            >>> robot = manager.create_robot('umh_2')
            >>> # Robot position is now automatically updated from OptiTrack
            >>> x, y, yaw = robot.get_position()
        """
        # Verify robot exists in config and get its configuration
        config = self.get_robot_config(robot_name)
        
        # Create tracker if not already created
        if self.tracker is None:
            # For single robot, only track this one
            robot_config_mapping = {robot_name: config}
            self.tracker = RobotTracker(robot_config_mapping)
        
        # Start tracking if requested and not already started
        if start_tracking and not self._started:
            self.tracker.start()
            self._started = True
            # Give tracker a moment to start
            time.sleep(0.5)
        
        # Return the robot instance from tracker
        if robot_name in self.tracker.robots:
            return self.tracker.robots[robot_name]
        else:
            raise RuntimeError(f"Failed to create robot '{robot_name}'")
    
    def create_multiple_robots(self, robot_names: list[str]) -> Dict[str, Robot]:
        """
        Create multiple robots with tracking.
        
        Args:
            robot_names: List of robot names (e.g., ['umh_2', 'umh_3'])
        
        Returns:
            Dictionary mapping robot names to Robot instances
        
        Example:
            >>> manager = RobotManager()
            >>> robots = manager.create_multiple_robots(['umh_2', 'umh_3'])
            >>> robot1 = robots['umh_2']
            >>> robot2 = robots['umh_3']
        """
        # Build config mapping
        robot_config_mapping = {}
        for name in robot_names:
            robot_config_mapping[name] = self.get_robot_config(name)
        
        # Create tracker with all robots
        self.tracker = RobotTracker(robot_config_mapping)
        self.tracker.start()
        self._started = True
        
        # Give tracker a moment to start
        time.sleep(0.5)
        
        return self.tracker.robots
    
    def stop(self):
        """Stop the robot tracker."""
        if self.tracker is not None and self._started:
            self.tracker.stop()
            self._started = False
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - stop tracker."""
        self.stop()


# Convenience function for simple use cases
def get_robot(robot_name: str, config_path: Optional[str] = None) -> tuple[Robot, RobotManager]:
    """
    Quick way to get a tracked robot.
    
    Args:
        robot_name: Name of the robot (e.g., 'umh_2')
        config_path: Optional path to config.json
    
    Returns:
        Tuple of (robot, manager). Keep manager reference to stop tracking later.
    
    Example:
        >>> robot, manager = get_robot('umh_2')
        >>> # Use robot...
        >>> manager.stop()  # Stop when done
    """
    manager = RobotManager(config_path)
    robot = manager.create_robot(robot_name)
    return robot, manager


if __name__ == "__main__":
    """Test the robot manager."""
    print("=" * 70)
    print("ROBOT MANAGER TEST")
    print("=" * 70)
    print()
    
    manager = RobotManager()
    
    print("Available robots:")
    for name in manager.get_available_robots():
        config = manager.get_robot_config(name)
        print(f"  - {name}: IP {config['ip']}, port {config['port']}")
    print()
    
    print("Testing single robot creation...")
    robot = manager.create_robot('umh_2')
    print(f"✓ Created robot 'umh_2'")
    print()
    
    print("Tracking robot position for 5 seconds...")
    print("(Move the robot to see position updates)")
    print()
    
    for i in range(10):
        x, y, yaw = robot.get_position()
        print(f"  [{i+1}/10] Position: ({x:.4f}, {y:.4f}), Yaw: {yaw:.4f} rad")
        time.sleep(0.5)
    
    print()
    manager.stop()
    print("✓ Test complete!")

