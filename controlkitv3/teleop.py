#!/usr/bin/env python3
import os
import threading
import time

import rclpy
from bb_controls_msgs.srv import Controller, SplineTraj
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sshkeyboard import listen_keyboard


class RobotTeleop(Node):
    def __init__(self):
        super().__init__("robot_teleop")

        # Initialize controller client
        self.controller_client = self.create_client(
            Controller, "/auv4/controls/controller"
        )
        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Controller service not available, waiting...")

        # Initialize trajectory client
        self.traj_client = self.create_client(SplineTraj, "/auv4/controls/xyzTraj")
        while not self.traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trajectory service not available, waiting...")

        # Teleop state
        self.is_active = False
        self.keys_pressed = set()
        self.pending_activation = (
            False  # Flag to indicate pending controller activation
        )

        # Configuration
        self.translation_step = 0.1  # meters per keypress
        self.rotation_step = 10.0  # degrees per keypress
        self.depth_step = 0.1  # meters per keypress

        # Step size adjustment parameters
        self.step_multiplier = 1.5  # Factor to multiply/divide step sizes
        self.min_translation_step = 0.01  # Minimum translation step (1cm)
        self.max_translation_step = 1.0  # Maximum translation step (1m)
        self.min_rotation_step = 1.0  # Minimum rotation step (1 degree)
        self.max_rotation_step = 90.0  # Maximum rotation step (90 degrees)
        self.min_depth_step = 0.01  # Minimum depth step (1cm)
        self.max_depth_step = 1.0  # Maximum depth step (1m)

        # For key repeat handling
        self.last_command_time = time.time()
        self.key_repeat_delay = 0.2  # seconds

        # To cleanly exit
        self.running = True

        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(
            target=listen_keyboard,
            args=(self.on_key_press, self.on_key_release),
            daemon=True,
        )
        self.keyboard_thread.start()

        # Create a timer for processing keys
        self.timer = self.create_timer(0.05, self.process_keys)  # 20Hz

        # Create a timer for checking service futures
        self.controller_future = None
        self.service_timer = self.create_timer(0.1, self.check_service_futures)  # 10Hz

        self.get_logger().info("Teleop initialized. Press SPACE to activate.")
        self.print_status()

    def clear_screen(self):
        os.system("cls" if os.name == "nt" else "clear")

    def print_status(self):
        self.clear_screen()
        status = "ACTIVE" if self.is_active else "INACTIVE"
        if self.pending_activation:
            status = "ACTIVATING..."
        print(f"=== ROBOT TELEOP [{status}] ===")
        print("\nControls:")
        print("  SPACE - Toggle teleop")
        print("  WASD  - Move forward/backward/left/right")
        print("  ↑/↓   - Decrease/increase depth")
        print("  ←/→   - Yaw left/right")
        print("  +     - Increase step sizes")
        print("  -     - Decrease step sizes")
        print("  Q     - Quit")

        print("\nCurrent Step Sizes:")
        print(f"  Translation: {self.translation_step:.3f} m")
        print(f"  Depth:       {self.depth_step:.3f} m")
        print(f"  Rotation:    {self.rotation_step:.1f}°")

        if not self.is_active and not self.pending_activation:
            print("\nPress SPACE to activate teleop")
        elif self.pending_activation:
            print("\nActivating controller...")
        else:
            print("\nTeleop is ACTIVE - sending trajectory commands")

    def increase_step_sizes(self):
        """Increase all step sizes by the multiplier factor"""
        self.translation_step = min(
            self.translation_step * self.step_multiplier, self.max_translation_step
        )
        self.rotation_step = min(
            self.rotation_step * self.step_multiplier, self.max_rotation_step
        )
        self.depth_step = min(
            self.depth_step * self.step_multiplier, self.max_depth_step
        )
        self.get_logger().info(
            f"Increased step sizes - Translation: {self.translation_step:.3f}m, "
            f"Depth: {self.depth_step:.3f}m, Rotation: {self.rotation_step:.1f}°"
        )
        self.print_status()

    def decrease_step_sizes(self):
        """Decrease all step sizes by the multiplier factor"""
        self.translation_step = max(
            self.translation_step / self.step_multiplier, self.min_translation_step
        )
        self.rotation_step = max(
            self.rotation_step / self.step_multiplier, self.min_rotation_step
        )
        self.depth_step = max(
            self.depth_step / self.step_multiplier, self.min_depth_step
        )
        self.get_logger().info(
            f"Decreased step sizes - Translation: {self.translation_step:.3f}m, "
            f"Depth: {self.depth_step:.3f}m, Rotation: {self.rotation_step:.1f}°"
        )
        self.print_status()

    def enable_controller(self):
        request = Controller.Request()
        request.enable = True
        request.pause = False
        request.disable_altitude = False

        self.controller_future = self.controller_client.call_async(request)
        self.pending_activation = True
        self.print_status()

    def check_service_futures(self):
        # Check controller service future
        if self.controller_future and self.controller_future.done():
            if self.controller_future.result() is not None:
                status = self.controller_future.result().status
                self.get_logger().info(f"Controller enabled. Status: {status}")
                self.is_active = True
            else:
                self.get_logger().error("Failed to enable controller")
                self.is_active = False

            self.pending_activation = False
            self.controller_future = None
            self.print_status()

    def send_trajectory(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        request = SplineTraj.Request()

        # Set the waypoint (relative to current position)
        request.position_poses = [Vector3(x=x, y=y, z=z)]
        request.orientation_rpy = [Vector3(x=0.0, y=0.0, z=yaw)]

        # Set parameters
        request.relative_xy_position_goal = True
        request.relative_z_position_goal = True
        request.relative_orientation_goal = True
        request.set_faster_limits = False
        request.is_altitude_tracking = False
        request.specified_heading = 1
        request.planner = ""

        # Send request asynchronously - no need to wait for result
        self.traj_client.call_async(request)

    def process_keys(self):
        now = time.time()

        # Check if enough time has passed since last command
        if now - self.last_command_time < self.key_repeat_delay:
            return

        # Don't process movement keys if teleop is not active
        if not self.is_active or not self.keys_pressed:
            return

        # Reset the timer
        self.last_command_time = now

        # Initialize movement values
        x_move = 0.0
        y_move = 0.0
        z_move = 0.0
        yaw_move = 0.0

        # Process currently pressed keys
        if "w" in self.keys_pressed:
            x_move += self.translation_step

        if "s" in self.keys_pressed:
            x_move -= self.translation_step

        if "a" in self.keys_pressed:
            y_move -= self.translation_step

        if "d" in self.keys_pressed:
            y_move += self.translation_step

        if "up" in self.keys_pressed:
            z_move -= self.depth_step  # Decrease depth (move up)

        if "down" in self.keys_pressed:
            z_move += self.depth_step  # Increase depth (move down)

        if "left" in self.keys_pressed:
            yaw_move -= self.rotation_step  # Yaw left (decrease angle)

        if "right" in self.keys_pressed:
            yaw_move += self.rotation_step  # Yaw right (increase angle)

        # Send trajectory if any movement is needed
        if x_move != 0.0 or y_move != 0.0 or z_move != 0.0 or yaw_move != 0.0:
            self.send_trajectory(x=x_move, y=y_move, z=z_move, yaw=yaw_move)

    def on_key_press(self, key):
        if key == "q":
            self.running = False
            return False  # Stop keyboard listener

        if key == "space":
            if not self.is_active and not self.pending_activation:
                # Enable controller when activating
                self.enable_controller()
            elif self.is_active:
                self.is_active = False
                self.get_logger().info(
                    "Teleop deactivated. Robot will stationkeep at last position."
                )
                self.print_status()
            return

        # Handle step size adjustment keys
        if key == "+":
            self.increase_step_sizes()
            return

        if key == "-":
            self.decrease_step_sizes()
            return

        # Add key to pressed keys set
        normalized_key = key.lower()

        # Map arrow keys to readable names
        key_mapping = {
            "key_up": "up",
            "key_down": "down",
            "key_left": "left",
            "key_right": "right",
        }

        if normalized_key in key_mapping:
            normalized_key = key_mapping[normalized_key]

        self.keys_pressed.add(normalized_key)

    def on_key_release(self, key):
        normalized_key = key.lower()

        # Map arrow keys to readable names
        key_mapping = {
            "key_up": "up",
            "key_down": "down",
            "key_left": "left",
            "key_right": "right",
        }

        if normalized_key in key_mapping:
            normalized_key = key_mapping[normalized_key]

        # Remove key from pressed keys set
        if normalized_key in self.keys_pressed:
            self.keys_pressed.remove(normalized_key)


def main(args=None):
    rclpy.init(args=args)
    teleop = RobotTeleop()

    try:
        # Run the main ROS loop on the main thread
        while rclpy.ok() and teleop.running:
            rclpy.spin_once(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
