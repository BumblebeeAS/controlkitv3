#!/usr/bin/env python3

import select
import sys
import termios
import threading
import tty

import rclpy
from bb_auv_msgs.srv import ResetPose
from bb_controls_msgs.srv import Controller
from rclpy.node import Node


class ServiceCalls(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")

        # Initialize service clients
        self.reset_pose_client = self.create_client(ResetPose, "/auv4/nav/reset_pose")
        self.controller_client = self.create_client(
            Controller, "/auv4/controls/controller"
        )

        # Wait for services to be available
        self.get_logger().info("Waiting for services to be available...")
        while not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Reset pose service not available, waiting again...")

        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Controller service not available, waiting again...")

        self.get_logger().info("Services are available!")

        # Track current controller state for space bar toggle
        self.controls_enabled = False

        # Store original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Setup terminal for character-by-character input
        tty.setraw(sys.stdin.fileno())

        # Start keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.print_help()

    def print_help(self):
        """Print available commands"""
        print("\n" + "=" * 50)
        print("AUV4 Keyboard Control Node")
        print("=" * 50)
        print("Commands:")
        print("  e - Enable controls")
        print("  d - Disable controls")
        print("  r - Reset pose")
        print("  space - Toggle controls enable/disable")
        print("  h - Show this help")
        print("  q - Quit")
        print("=" * 50)
        print("Waiting for input...")

    def keyboard_listener(self):
        """Listen for keyboard input in a separate thread"""
        while self.running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    char = sys.stdin.read(1)
                    self.handle_keypress(char)
                except:
                    break

    def handle_keypress(self, char):
        """Handle individual key presses"""
        if char.lower() == "e":
            self.enable_controls()
        elif char.lower() == "d":
            self.disable_controls()
        elif char.lower() == "r":
            self.reset_pose()
        elif char == " ":  # Space bar
            self.toggle_controls()
        elif char.lower() == "h":
            self.print_help()
        elif char.lower() == "q":
            self.get_logger().info("Quitting...")
            self.running = False
            rclpy.shutdown()
        else:
            print(f"Unknown command: '{char}'. Press 'h' for help.")

    def enable_controls(self):
        """Enable AUV controls"""
        request = Controller.Request()
        request.enable = True
        request.pause = False
        request.disable_altitude = False

        future = self.controller_client.call_async(request)
        future.add_done_callback(
            lambda f: self.service_callback(f, "Enable controls", True)
        )

    def disable_controls(self):
        """Disable AUV controls"""
        request = Controller.Request()
        request.enable = False
        request.pause = False
        request.disable_altitude = False

        future = self.controller_client.call_async(request)
        future.add_done_callback(
            lambda f: self.service_callback(f, "Disable controls", False)
        )

    def toggle_controls(self):
        """Toggle controls between enabled and disabled"""
        if self.controls_enabled:
            self.disable_controls()
        else:
            self.enable_controls()

    def reset_pose(self):
        """Reset AUV pose"""
        request = ResetPose.Request()
        request.reset = 1

        future = self.reset_pose_client.call_async(request)
        future.add_done_callback(lambda f: self.reset_pose_callback(f))

    def service_callback(self, future, action, new_state):
        """Callback for controller service calls"""
        try:
            response = future.result()
            self.controls_enabled = new_state
            state_str = "ENABLED" if new_state else "DISABLED"
            self.get_logger().info(f"{action} successful - Controls: {state_str}")
            print(f"> {action} successful - Controls: {state_str}")
        except Exception as e:
            self.get_logger().error(f"{action} failed: {e}")
            print(f"> {action} failed: {e}")

    def reset_pose_callback(self, future):
        """Callback for reset pose service calls"""
        try:
            response = future.result()
            self.get_logger().info("Reset pose successful")
            print("> Reset pose successful")
        except Exception as e:
            self.get_logger().error(f"Reset pose failed: {e}")
            print(f"> Reset pose failed: {e}")

    def cleanup(self):
        """Restore terminal settings"""
        self.running = False
        if hasattr(self, "old_settings"):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)

    node = ServiceCalls()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
