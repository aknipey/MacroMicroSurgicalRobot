#!/usr/bin/env python3
"""Keyboard jogging of the UR5e end effector through MoveIt Servo.

Needs the UR driver + MoveIt Servo running (see launch/ur5e_teleop.launch.py),
then run in its own terminal:

    ros2 run ls_thesis ur5e_keyboard_teleop.py

A terminal only reports key presses, not releases, so a key moves the robot for
a short window after each press. Holding a key keeps it moving via the OS key
auto-repeat; letting go stops it within `hold_time` seconds. SPACE stops at once.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from ls_thesis.ur5e_common import (
    SERVO_TWIST_TOPIC,
    TELEOP_CONTROLLER,
    RecorderClient,
    activate_controller,
    start_servo_twist_mode,
)

# key: (twist field, axis, sign)
KEY_BINDINGS = {
    "w": ("linear", "x", +1), "s": ("linear", "x", -1),
    "a": ("linear", "y", +1), "d": ("linear", "y", -1),
    "r": ("linear", "z", +1), "f": ("linear", "z", -1),
    "u": ("angular", "x", +1), "o": ("angular", "x", -1),
    "i": ("angular", "y", +1), "k": ("angular", "y", -1),
    "j": ("angular", "z", +1), "l": ("angular", "z", -1),
}

# Seconds a key stays "held" after the first press. Covers the OS delay before
# key auto-repeat kicks in (typically 250-600 ms).
FIRST_PRESS_HOLD = 0.6

HELP = """
==================== UR5e KEYBOARD TELEOP ====================
 Translate        w/s : +X/-X     a/d : +Y/-Y     r/f : +Z/-Z
 Rotate           u/o : roll      i/k : pitch     j/l : yaw
 SPACE : stop      +/- : speed up/down      t : toggle frame
 [ : start recording trajectory     ] : stop + save recording
 h : help          Ctrl+C : quit
===============================================================
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("ur5e_keyboard_teleop")

        self.declare_parameter("linear_speed", 0.05)    # m/s
        self.declare_parameter("angular_speed", 0.3)    # rad/s
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("hold_time", 0.15)       # s, after auto-repeat presses
        self.declare_parameter("publish_rate", 50.0)    # Hz
        self.declare_parameter("setup_servo", True)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.frames = [self.get_parameter("base_frame").value, self.get_parameter("tool_frame").value]
        self.frame_index = 0
        self.hold_time = self.get_parameter("hold_time").value

        self.twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
        self.recorder = RecorderClient(self)

        # key -> time at which it stops counting as held
        self.held_until = {}

        if self.get_parameter("setup_servo").value:
            activate_controller(self, TELEOP_CONTROLLER)
            if not start_servo_twist_mode(self):
                self.get_logger().error(
                    "Could not configure MoveIt Servo - is ur5e_teleop.launch.py running?")

        self.create_timer(1.0 / self.get_parameter("publish_rate").value, self.publish_twist)

    @property
    def frame(self):
        return self.frames[self.frame_index]

    def handle_key(self, key):
        now = time.monotonic()
        if key in KEY_BINDINGS:
            if self.held_until.get(key, 0.0) < now:
                self.held_until[key] = now + FIRST_PRESS_HOLD
            else:
                self.held_until[key] = max(self.held_until[key], now + self.hold_time)
        elif key == " ":
            self.held_until.clear()
            self.get_logger().info("Stop")
        elif key in ("+", "="):
            self.scale_speed(1.25)
        elif key in ("-", "_"):
            self.scale_speed(0.8)
        elif key == "t":
            self.held_until.clear()
            self.frame_index = 1 - self.frame_index
            self.get_logger().info(f"Commands now in frame '{self.frame}'")
        elif key == "[":
            self.recorder.start()
        elif key == "]":
            self.recorder.stop()
        elif key == "h":
            print(HELP)

    def scale_speed(self, factor):
        self.linear_speed = min(self.linear_speed * factor, 0.25)
        self.angular_speed = min(self.angular_speed * factor, 1.0)
        self.get_logger().info(
            f"Speed: {self.linear_speed * 100:.1f} cm/s, {self.angular_speed:.2f} rad/s")

    def publish_twist(self):
        now = time.monotonic()
        held = [k for k, until in self.held_until.items() if until >= now]
        if not held and not self.held_until:
            # Nothing pressed: publish nothing so Servo's command timeout halts the arm.
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        for key in held:
            field, axis, sign = KEY_BINDINGS[key]
            speed = self.linear_speed if field == "linear" else self.angular_speed
            setattr(getattr(msg.twist, field), axis, sign * speed)
        self.twist_pub.publish(msg)

        if not held:
            # Send one explicit zero after the last key expires, then go quiet.
            self.held_until.clear()


def main():
    rclpy.init()
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error("Keyboard teleop needs an interactive terminal (stdin is not a TTY)")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                node.handle_key(sys.stdin.read(1).lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
