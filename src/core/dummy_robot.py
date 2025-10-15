#!/usr/bin/env python3
"""
Dummy robot implementation for testing without hardware.

This implementation simulates the behavior of the real robot's mov_latest.py node,
including local-to-world frame transformation and blocking movement commands.
"""

import math
import threading
import time
from typing import Tuple


def ang_norm(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a


class DummyRobot:
    """
    Simulated robot for testing without hardware.
    
    Mimics the behavior of mov_latest.py:
    - Transforms local displacement vectors to world frame
    - Uses proportional control for smooth movement
    - Blocks until movement is complete
    """
    
    def __init__(
        self,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        max_speed: float = 0.5,
        update_rate_hz: float = 50.0,
        mecanum: bool = True,  # Default to holonomic mode like real robot
        pos_tolerance: float = 0.03,
        kp_lin: float = 0.9,  # Proportional gain for linear control
        use_local_frame: bool = False,  # If True, interprets vectors in robot's local frame
    ):
        """
        Initialize a dummy robot.
        Args:
            initial_x: Starting x position (meters)
            initial_y: Starting y position (meters)
            initial_yaw: Starting yaw angle (radians)
            max_speed: Maximum movement speed (m/s)
            update_rate_hz: Simulation update rate (Hz)
            mecanum: Use mecanum/holonomic drive mode
            pos_tolerance: Position tolerance for goal reaching (meters)
            kp_lin: Linear proportional gain
            use_local_frame: If True, vectors are in robot frame (like real robot).
                           If False, vectors are in world frame (default, for backward compatibility)
        """
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw
        
        # Movement parameters
        self.max_speed = max_speed
        self.update_rate_hz = update_rate_hz
        self.pos_tolerance = pos_tolerance
        self.kp_lin = kp_lin
        self.mecanum = mecanum
        self.use_local_frame = use_local_frame
        
        # Goal tracking
        self.goal_x = None
        self.goal_y = None
        self.start_yaw = None
        self.goal_active = False
        self.goal_complete_event = threading.Event()
        
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
    
    def move_by_vector(self, dx: float, dy: float, blocking: bool = False) -> bool:
        """
        Command robot to move by a displacement vector.
        
        Behavior depends on use_local_frame setting:
        - If use_local_frame=True: (dx, dy) are in robot's local frame (like mov_latest.py)
        - If use_local_frame=False: (dx, dy) are in world frame (default, backward compatible)
        
        Args:
            dx: X displacement (meters) - frame depends on use_local_frame
            dy: Y displacement (meters) - frame depends on use_local_frame
            blocking: If True, blocks until movement completes (default: False for compatibility)
        
        Returns:
            True if movement completed successfully
        """
        with self.lock:
            # Record starting yaw (like mov_latest does)
            self.start_yaw = self.yaw
            
            # Transform to world frame if needed
            if self.use_local_frame:
                # Transform local displacement to world frame (like mov_latest.py)
                cos_yaw = math.cos(self.yaw)
                sin_yaw = math.sin(self.yaw)
                world_dx = cos_yaw * dx - sin_yaw * dy
                world_dy = sin_yaw * dx + cos_yaw * dy
            else:
                # Already in world frame (backward compatible behavior)
                world_dx = dx
                world_dy = dy
            
            # Set goal in world coordinates
            self.goal_x = self.x + world_dx
            self.goal_y = self.y + world_dy
            self.goal_active = True
            self.goal_complete_event.clear()
        
        # Block until movement completes (like mov_latest does)
        if blocking:
            self.goal_complete_event.wait()
        
        return True
    
    def is_ready(self) -> bool:
        """Check if robot is ready."""
        return self._ready
    
    def stop(self) -> None:
        """Stop the robot immediately."""
        with self.lock:
            self.goal_x = self.x
            self.goal_y = self.y
            self.goal_active = False
            self.goal_complete_event.set()
    
    def shutdown(self) -> None:
        """Shutdown the simulation."""
        self.running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
    
    def _simulation_loop(self) -> None:
        """
        Internal simulation loop.
        
        Mimics the control loop from mov_latest.py with proportional control.
        """
        dt = 1.0 / self.update_rate_hz
        
        while self.running:
            with self.lock:
                if not self.goal_active or self.goal_x is None or self.goal_y is None:
                    # No active goal
                    time.sleep(dt)
                    continue
                
                # Calculate distance to goal
                dx = self.goal_x - self.x
                dy = self.goal_y - self.y
                dist = math.hypot(dx, dy)
                
                # Check if goal reached
                if dist <= self.pos_tolerance:
                    self.goal_active = False
                    self.goal_complete_event.set()
                    continue
                
                # Proportional control (like mov_latest.py)
                v_cmd = min(self.max_speed, self.kp_lin * dist)
                
                if self.mecanum:
                    # Holonomic motion: move directly toward goal
                    # This matches the mecanum mode in mov_latest.py
                    if dist > 1e-6:
                        # Calculate velocity in world frame
                        vx_world = (dx / dist) * v_cmd
                        vy_world = (dy / dist) * v_cmd
                        
                        # Apply velocity for this timestep
                        self.x += vx_world * dt
                        self.y += vy_world * dt
                        
                        # Maintain starting orientation (like mov_latest does in mecanum mode)
                        # This prevents yaw drift during movement
                        if self.start_yaw is not None:
                            # In simulation, we'll keep the yaw constant
                            # (In real robot, there's a yaw controller to maintain start_yaw)
                            pass
                else:
                    # Differential drive: point toward goal first, then move
                    goal_heading = math.atan2(dy, dx)
                    heading_err = ang_norm(goal_heading - self.yaw)
                    
                    # Simple differential drive simulation
                    if abs(heading_err) > 0.1:  # Need to turn
                        # Turn toward goal
                        self.yaw += heading_err * 0.5 * dt  # Simple turning
                        self.yaw = ang_norm(self.yaw)
                    else:
                        # Move forward
                        self.x += v_cmd * math.cos(self.yaw) * dt
                        self.y += v_cmd * math.sin(self.yaw) * dt
                        # Update yaw to face movement direction
                        self.yaw = goal_heading
            
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

