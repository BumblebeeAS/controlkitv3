#!/usr/bin/env python3

import fcntl
import os
import sys
import termios
import time
import tty
from io import BytesIO

import numpy as np
import rclpy
from ascii_magic import AsciiArt
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as RosImage


class ImageViewer(Node):
    def __init__(self):
        super().__init__("image_viewer")
        self.old_settings = None
        self.old_flags = None
        self.subscription = None

        # Clear terminal on startup
        sys.stdout.write("\033[H\033[J")
        sys.stdout.flush()

        # Start topic selection
        self.select_and_subscribe()

    def select_and_subscribe(self):
        """Select topic and create subscription."""
        # Clean up existing subscription if any
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        # Temporarily restore normal terminal mode for input
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            fcntl.fcntl(
                sys.stdin.fileno(), fcntl.F_SETFL, self.old_flags
            )  # Restore blocking mode
            sys.stdout.write("\033[?25h")  # Show cursor
            sys.stdout.flush()

        # Select topic
        self.topic_name, self.is_compressed = self.select_topic()
        if not self.topic_name:
            self.cleanup()
            rclpy.shutdown()
            return

        # Setup/restore terminal for image viewing
        self.setup_terminal()

        # Subscribe to selected topic
        msg_type = CompressedImage if self.is_compressed else RosImage
        self.subscription = self.create_subscription(
            msg_type, self.topic_name, self.on_image, 10
        )

        print(f"Viewing: {self.topic_name}")
        print("Press 'q' to quit, 'r' to reselect topic\n")

    def select_topic(self):
        """Find image topics and let user choose one."""
        print("Finding image topics...")
        time.sleep(1)  # Wait for discovery

        topics = []
        for name, types in self.get_topic_names_and_types():
            for topic_type in types:
                if "CompressedImage" in topic_type:
                    topics.append((name, True))
                    break
                elif "sensor_msgs/msg/Image" in topic_type:
                    topics.append((name, False))
                    break

        if not topics:
            print("No image topics found")
            return None, None

        print(f"\nFound {len(topics)} image topics:")
        for i, (name, compressed) in enumerate(topics, 1):
            type_str = "compressed" if compressed else "raw"
            print(f"{i}. {name} ({type_str})")

        while True:
            try:
                choice = input(f"\nSelect (1-{len(topics)}) or 'q' to quit: ").strip()
                if choice.lower() == "q":
                    return None, None

                idx = int(choice) - 1
                if 0 <= idx < len(topics):
                    return topics[idx]

            except (ValueError, KeyboardInterrupt):
                return None, None

    def setup_terminal(self):
        """Prepare terminal for display."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Store original flags and set non-blocking input
        self.old_flags = fcntl.fcntl(sys.stdin.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)

        os.system("clear")
        print("\033[?25l", end="")  # Hide cursor

    def on_image(self, msg):
        """Process and display image."""
        try:
            action = self.check_quit()
            if action == "quit":
                self.cleanup()
                rclpy.shutdown()
                return
            elif action == "restart":
                # Clear terminal and reselect topic
                sys.stdout.write("\033[H\033[J")
                sys.stdout.flush()
                self.select_and_subscribe()
                return

            # Convert message to PIL Image
            if hasattr(msg, "data") and len(msg.data) > 100:  # CompressedImage
                img = Image.open(BytesIO(msg.data))
            else:  # Regular Image
                if msg.encoding == "rgb8":
                    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3
                    )
                    img = Image.fromarray(data)
                elif msg.encoding == "bgr8":
                    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3
                    )
                    img = Image.fromarray(data[:, :, ::-1])  # BGR to RGB
                else:
                    return  # Skip unsupported formats

            # Get terminal size
            cols, rows = os.get_terminal_size()

            # Ensure minimum usable dimensions (much more reasonable minimums)
            min_cols, min_rows = 10, 5

            if cols < min_cols or rows < min_rows:
                # Terminal too small, show message instead
                sys.stdout.write(f"\033[H Terminal too small: {cols}x{rows}\n")
                sys.stdout.write(f"Minimum required: {min_cols}x{min_rows}")
                sys.stdout.flush()
                return

            # Calculate available space (reserve 2 lines for status)
            display_rows = max(1, rows - 2)

            # Use slightly smaller width to ensure lines don't wrap
            # (some terminals/ascii_magic can produce lines slightly longer than requested)
            ascii_cols = max(min_cols, cols - 1)

            # Convert to ASCII
            ascii_art = AsciiArt.from_pillow_image(img).to_ascii(columns=ascii_cols)

            # Clip to fit available height
            lines = ascii_art.split("\n")
            if len(lines) > display_rows:
                lines = lines[:display_rows]

            # Display image using stdout directly (clear screen first)
            sys.stdout.write("\033[H\033[J" + "\n".join(lines))
            sys.stdout.flush()

            # Show status line
            topic_short = (
                self.topic_name.split("/")[-1]
                if len(self.topic_name) > 40
                else self.topic_name
            )
            status = (
                f"Topic: {topic_short} | Size: {cols}x{rows} | 'q':quit 'r':reselect"
            )

            # Truncate status if too long
            if len(status) > cols:
                status = status[: cols - 3] + "..."

            sys.stdout.write(f"\n{status}")
            sys.stdout.flush()

        except:
            pass  # Skip frame on any error

    def check_quit(self):
        """Check if user pressed 'q' to quit or 'r' to restart."""
        try:
            char = sys.stdin.read(1)
            if char:
                if char.lower() == "q":
                    return "quit"
                elif char.lower() == "r":
                    return "restart"
            return None
        except:
            return None

    def cleanup(self):
        """Restore terminal."""
        try:
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            if self.old_flags is not None:
                fcntl.fcntl(sys.stdin.fileno(), fcntl.F_SETFL, self.old_flags)
            sys.stdout.write("\033[?25h")  # Show cursor
            sys.stdout.flush()
            os.system("clear")
            sys.stdout.write("Goodbye!\n")
            sys.stdout.flush()
        except:
            pass


def main():
    rclpy.init()
    try:
        viewer = ImageViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        if "viewer" in locals():
            viewer.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
