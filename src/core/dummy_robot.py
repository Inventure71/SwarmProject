#!/usr/bin/env python3
"""
Dummy robot implementation for testing without hardware.
"""

import math
import threading
import time
from typing import Tuple


class DummyRobot:
    """Simulated robot for testing without hardware."""
    
    def __init__(
        self,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        max_speed: float = 0.5,
        update_rate_hz: float = 50.0
    ):
        """
        Initialize a dummy robot.
        Args:
            initial_x: Starting x position (meters)
            initial_y: Starting y position (meters)
            initial_yaw: Starting yaw angle (radians)
            max_speed: Maximum movement speed (m/s)
            update_rate_hz: Simulation update rate (Hz)
        """
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw
        
        self.target_x = initial_x
        self.target_y = initial_y
        
        self.max_speed = max_speed
        self.update_rate_hz = update_rate_hz
        self.pos_tolerance = 0.03  # meters
        
        self.lock = threading.Lock()
        self.running = True
        self._ready = True
        
        # Start simulation loop
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current robot position."""
        with self.lock:
            return (self.x, self.y, self.yaw)
    
    def move_by_vector(self, dx: float, dy: float) -> bool:
        """
        Command robot to move by a displacement vector.
        Sets an absolute target position based on current position plus displacement.
        """
        with self.lock:
            # Set target as absolute position (current + displacement)
            self.target_x = self.x + dx
            self.target_y = self.y + dy
        return True
    
    def is_ready(self) -> bool:
        """Check if robot is ready."""
        return self._ready
    
    def stop(self) -> None:
        """Stop the robot."""
        with self.lock:
            self.target_x = self.x
            self.target_y = self.y
    
    def shutdown(self) -> None:
        """Shutdown the simulation."""
        self.running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
    
    def _simulation_loop(self) -> None:
        """Internal simulation loop."""
        dt = 1.0 / self.update_rate_hz
        
        while self.running:
            with self.lock:
                # Calculate distance to target
                dx = self.target_x - self.x
                dy = self.target_y - self.y
                dist = math.hypot(dx, dy)
                
                if dist > self.pos_tolerance:
                    # Move towards target
                    speed = min(self.max_speed, dist / dt)
                    vx = (dx / dist) * speed * dt
                    vy = (dy / dist) * speed * dt
                    
                    self.x += vx
                    self.y += vy
                    
                    # Update yaw to face movement direction
                    self.yaw = math.atan2(dy, dx)
            
            time.sleep(dt)


if __name__ == "__main__":
    # Simple test
    print("Testing DummyRobot...")
    robot = DummyRobot(initial_x=0.0, initial_y=0.0)
    
    print(f"Initial position: {robot.get_position()}")
    print("Moving by vector (1.0, 0.5)...")
    robot.move_by_vector(1.0, 0.5)
    
    # Wait for movement
    time.sleep(3)
    print(f"Final position: {robot.get_position()}")
    
    robot.shutdown()
    print("Test complete!")

