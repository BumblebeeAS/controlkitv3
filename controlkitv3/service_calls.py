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
        super().__init__("keyboard_service_calls_node")

        # Clear terminal and store original settings
        self.clear_terminal()
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Initialize service clients
        self.reset_pose_client = self.create_client(ResetPose, "/auv4/nav/reset_pose")
        self.controller_client = self.create_client(
            Controller, "/auv4/controls/controller"
        )

        # Wait for services to be available
        print("Waiting for services to be available...", flush=True)
        while not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
            print("Reset pose service not available, waiting again...", flush=True)

        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            print("Controller service not available, waiting again...", flush=True)

        print("Services are available!", flush=True)

        # Track current controller state for space bar toggle
        self.controls_enabled = False

        # Setup terminal for character-by-character input
        tty.setraw(sys.stdin.fileno())

        # Start keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.draw_interface()

    def clear_terminal(self):
        """Clear the terminal screen"""
        print("\033[2J\033[H", end="", flush=True)

    def draw_interface(self):
        """Draw the complete interface with help pinned at top"""
        # Temporarily restore normal terminal mode for clean printing
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        # Clear screen and position cursor at top
        print("\033[2J\033[H", end="")

        # Draw the pinned help section
        print("=" * 60)
        print("AUV4 Service Calls")
        print("=" * 60)
        print("Commands:")
        print("  e - Enable controls")
        print("  d - Disable controls")
        print("  r - Reset pose")
        print("  space - Toggle controls enable/disable")
        print("  h - Refresh display")
        print("  q - Quit")
        print("=" * 60)
        print(
            f"Controls Status: {('ENABLED' if self.controls_enabled else 'DISABLED')}"
        )
        print("=" * 60)
        print("Status Messages:")
        print("Ready - Waiting for input...")
        print("", flush=True)

        # Return to raw mode
        tty.setraw(sys.stdin.fileno())

    def print_help(self):
        """Refresh the entire interface"""
        self.draw_interface()

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
            self.print_message("Quitting...")
            self.running = False
            rclpy.shutdown()
        else:
            self.print_message(f"Unknown command: '{char}'. Press 'h' for help.")

    def print_message(self, message):
        """Print a message in the status area without disturbing the help"""
        # Temporarily restore normal terminal mode for clean printing
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        # Position cursor at status message line (line 15)
        print("\033[15;1H", end="")
        # Clear multiple lines to handle message wrapping
        for i in range(4):  # Clear 4 lines to handle long messages
            print("\033[2K", end="")  # Clear entire line
            if i < 3:  # Don't move down after the last line
                print("\033[1B", end="")  # Move cursor down one line

        # Return to line 15 and print the message
        print("\033[15;1H", end="")
        print(f"{message}", flush=True)

        # Return to raw mode
        tty.setraw(sys.stdin.fileno())

    def update_status(self):
        """Update the controls status display"""
        # Temporarily restore normal terminal mode for clean printing
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        # Position cursor at status line (line 12)
        print("\033[12;1H", end="")
        # Clear the entire line
        print("\033[2K", end="")  # Clear entire line
        print(
            f"Controls Status: {('ENABLED' if self.controls_enabled else 'DISABLED')}",
            flush=True,
        )

        # Return to raw mode
        tty.setraw(sys.stdin.fileno())

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
            self.update_status()  # Update the status display
            self.print_message(f"{action} successful - Controls: {state_str}")
        except Exception as e:
            self.get_logger().error(f"{action} failed: {e}")
            self.print_message(f"{action} failed: {e}")

    def reset_pose_callback(self, future):
        """Callback for reset pose service calls"""
        try:
            response = future.result()
            self.get_logger().info("Reset pose successful")
            self.print_message("Reset pose successful")
        except Exception as e:
            self.get_logger().error(f"Reset pose failed: {e}")
            self.print_message(f"Reset pose failed: {e}")

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
