#!/usr/bin/env python3
import socket
import threading
import time
import sys
from pathlib import Path

# Add the parent directory to the path to allow importing modules
sys.path.append(str(Path(__file__).resolve().parent.parent))
from core.robot import Robot, create_robot

class RobotTracker:
    """
    Listens to UDP ports for pose data and updates Robot objects in real-time.
    """

    def __init__(self, robot_config_mapping: dict[str, dict]):
        """
        Initializes the tracker.
        
        Args:
            robot_config_mapping: A dictionary mapping a robot's name/ID to its config dict.
                                 Each config dict should have 'ip' and 'port' keys.
                                 Example: {'umh_2': {'ip': '192.168.1.2', 'port': 9876}}
        """
        self.robot_config_mapping = robot_config_mapping
        self.robots = {
            name: create_robot(robot_type="real", robot_ip=config['ip']) 
            for name, config in robot_config_mapping.items()
        }
        self._should_exit = threading.Event()
        self._threads = []

        self.x_offset = -250 #-180
        self.y_offset = -200 #-200
        self.scale_factor = 1/40

    def _listener_thread(self, robot_name: str, port: int, host: str = "0.0.0.0"):
        """
        Internal thread function to listen on a port for a specific robot.
        Receives UDP packets in format: [x,y,z,w] where w is yaw angle.
        """
        robot = self.robots[robot_name]
        print(f"[Tracker: {robot_name}] Listening on port {port}...")
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port))
            except OSError as e:
                print(f"[Tracker: {robot_name}] ERROR: Could not bind to port {port}. {e}")
                return

            s.settimeout(1.0)  # Timeout to allow checking the exit flag
            
            while not self._should_exit.is_set():
                try:
                    data, _ = s.recvfrom(100)
                    
                    # Decode the message as string
                    msg = data.decode()
                    
                    # Remove brackets and split by comma: [x,y,z,w] -> x,y,z,w
                    msg = msg.strip('[]')
                    x, y, z, yaw = map(float, msg.split(','))
                    x = (x + self.x_offset) * self.scale_factor
                    y = (y + self.y_offset) * self.scale_factor
                    #z = z * self.scale_factor
                    
                    # Update robot position (we use x, y, and yaw; z is ignored for 2D movement)
                    robot.update_position(x, y, yaw)

                except socket.timeout:
                    continue  # Loop again to check exit flag
                except Exception as e:
                    print(f"[Tracker: {robot_name}] Error parsing packet: {e}")

        print(f"[Tracker: {robot_name}] Listener on port {port} has shut down.")

    def start(self):
        """
        Starts the tracking threads for all robots.
        """
        if self._threads:
            print("Tracker is already running.")
            return

        print("Starting real-time robot trackers...")
        self._should_exit.clear()
        for name, config in self.robot_config_mapping.items():
            port = config['port']
            thread = threading.Thread(target=self._listener_thread, args=(name, port), daemon=True)
            self._threads.append(thread)
            thread.start()

    def stop(self):
        """
        Signals all tracking threads to stop.
        """
        print("Stopping robot trackers...")
        self._should_exit.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        print("All trackers have been stopped.")

def main() -> int:
    """
    Example usage of the RobotTracker.
    """
    # --- CONFIGURATION ---
    # This dictionary defines which robots to track with their IP and UDP port.
    # The names should match the rigid body names from OptiTrack/ROS.
    ROBOT_CONFIG = {
        'umh_2': {'ip': '192.168.1.2', 'port': 9876},
        'umh_3': {'ip': '192.168.1.3', 'port': 9877},
        'umh_4': {'ip': '192.168.1.4', 'port': 9878},
        'umh_5': {'ip': '192.168.1.5', 'port': 9880},
    }
    # ---------------------

    tracker = RobotTracker(ROBOT_CONFIG)
    tracker.start()

    print("\nRobot positions will be updated in the background.")
    print("Displaying current robot positions every 2 seconds. Press Ctrl+C to stop.")

    try:
        while True:
            print("\n--- Current Robot States ---")
            for name, robot in tracker.robots.items():
                pos = robot.get_position()
                print(f"  - {name}: Position(x,y): ({pos[0]:.4f}, {pos[1]:.4f}), Yaw: {pos[2]:.4f} rad")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nCaught interrupt signal.")
    finally:
        tracker.stop()

    return 0

if __name__ == "__main__":
    sys.exit(main())
