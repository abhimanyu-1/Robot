#!/usr/bin/env python3
"""
WASD Keyboard Controller for TurtleBot3 Burger.

Publishes geometry_msgs/msg/Twist to /cmd_vel based on keyboard input.

Controls:
    W / ↑  : Move forward
    S / ↓  : Move backward
    A / ←  : Rotate left (CCW)
    D / →  : Rotate right (CW)
    Q      : Forward + rotate left
    E      : Forward + rotate right
    Z      : Backward + rotate left
    C      : Backward + rotate right
    SPACE  : Emergency stop
    X      : Emergency stop
    +/-    : Increase / decrease speed by 10%
    ESC    : Quit

Usage:
    source /opt/ros/humble/setup.bash
    source turtlebot3_ws/install/setup.bash
    python3 robot_project/scripts/keyboard_controller.py
"""

import sys
import os
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── TurtleBot3 Burger Limits ──
MAX_LINEAR  = 0.22   # m/s
MAX_ANGULAR = 2.84   # rad/s
LINEAR_STEP = 0.02   # m/s per key press
ANGULAR_STEP = 0.15  # rad/s per key press

# ── Key bindings ──
FORWARD     = {'w', 'W'}
BACKWARD    = {'s', 'S'}
LEFT        = {'a', 'A'}
RIGHT       = {'d', 'D'}
FWD_LEFT    = {'q', 'Q'}
FWD_RIGHT   = {'e', 'E'}
BWD_LEFT    = {'z', 'Z'}
BWD_RIGHT   = {'c', 'C'}
STOP        = {' ', 'x', 'X'}
SPEED_UP    = {'+', '='}
SPEED_DOWN  = {'-', '_'}
QUIT        = {'\x1b'}  # ESC


BANNER = """
╔══════════════════════════════════════════════╗
║   TurtleBot3 Burger – Keyboard Controller   ║
╠══════════════════════════════════════════════╣
║                                              ║
║          Q (↰)    W (↑)    E (↱)             ║
║          A (←)    STOP     D (→)             ║
║          Z (↲)    S (↓)    C (↳)             ║
║                                              ║
║   SPACE / X : Emergency Stop                 ║
║   + / -     : Speed ±10%%                     ║
║   ESC       : Quit                           ║
║                                              ║
╚══════════════════════════════════════════════╝
"""


def get_key(timeout: float = 0.1) -> str:
    """Read a single keypress from stdin (non-blocking with timeout)."""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (escape sequences like \x1b[A)
            if key == '\x1b':
                extra = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                if extra == '[A':
                    return 'w'  # Up arrow → forward
                elif extra == '[B':
                    return 's'  # Down arrow → backward
                elif extra == '[D':
                    return 'a'  # Left arrow → left
                elif extra == '[C':
                    return 'd'  # Right arrow → right
                else:
                    return '\x1b'  # Plain ESC
            return key
        return ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.speed_scale = 1.0

    def clamp(self):
        """Clamp velocities to TurtleBot3 limits."""
        max_lin = MAX_LINEAR * self.speed_scale
        max_ang = MAX_ANGULAR * self.speed_scale
        self.linear_vel = max(-max_lin, min(max_lin, self.linear_vel))
        self.angular_vel = max(-max_ang, min(max_ang, self.angular_vel))

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher_.publish(msg)

    def stop(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.publish()

    def run(self):
        print(BANNER)
        print(f'  Speed scale : {self.speed_scale:.0%}')
        print(f'  Max linear  : {MAX_LINEAR * self.speed_scale:.2f} m/s')
        print(f'  Max angular : {MAX_ANGULAR * self.speed_scale:.2f} rad/s')
        print()

        try:
            while rclpy.ok():
                key = get_key(0.1)
                if not key:
                    # Gradual deceleration when no key is pressed
                    self.linear_vel *= 0.6
                    self.angular_vel *= 0.6
                    if abs(self.linear_vel) < 0.001:
                        self.linear_vel = 0.0
                    if abs(self.angular_vel) < 0.001:
                        self.angular_vel = 0.0
                elif key in QUIT:
                    print('\n  Shutting down…')
                    self.stop()
                    break
                elif key in STOP:
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                elif key in FORWARD:
                    self.linear_vel += LINEAR_STEP
                    self.angular_vel = 0.0
                elif key in BACKWARD:
                    self.linear_vel -= LINEAR_STEP
                    self.angular_vel = 0.0
                elif key in LEFT:
                    self.angular_vel += ANGULAR_STEP
                    self.linear_vel = 0.0
                elif key in RIGHT:
                    self.angular_vel -= ANGULAR_STEP
                    self.linear_vel = 0.0
                elif key in FWD_LEFT:
                    self.linear_vel += LINEAR_STEP
                    self.angular_vel += ANGULAR_STEP
                elif key in FWD_RIGHT:
                    self.linear_vel += LINEAR_STEP
                    self.angular_vel -= ANGULAR_STEP
                elif key in BWD_LEFT:
                    self.linear_vel -= LINEAR_STEP
                    self.angular_vel += ANGULAR_STEP
                elif key in BWD_RIGHT:
                    self.linear_vel -= LINEAR_STEP
                    self.angular_vel -= ANGULAR_STEP
                elif key in SPEED_UP:
                    self.speed_scale = min(1.5, self.speed_scale + 0.1)
                    print(f'\r  Speed scale : {self.speed_scale:.0%}    ', end='', flush=True)
                elif key in SPEED_DOWN:
                    self.speed_scale = max(0.1, self.speed_scale - 0.1)
                    print(f'\r  Speed scale : {self.speed_scale:.0%}    ', end='', flush=True)

                self.clamp()
                self.publish()

                # Status line
                status = (
                    f'\r  linear: {self.linear_vel:+.3f} m/s  |  '
                    f'angular: {self.angular_vel:+.3f} rad/s  |  '
                    f'scale: {self.speed_scale:.0%}    '
                )
                print(status, end='', flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
