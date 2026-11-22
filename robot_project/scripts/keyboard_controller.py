#!/usr/bin/env python3
"""
Interactive WASD Keyboard Controller for TurtleBot3 Burger in ROS 2.

Features:
- Sustained velocity: Commands keep the robot driving continuously until stopped.
- 10 Hz continuous publisher: Keeps Gazebo diff_drive controller smoothly engaged.
- Real-time /odom telemetry: Displays live robot coordinates and actual speed from Gazebo.
- Emergency STOP: Instantly halts movement with SPACE or X.
- Full WASD + Arrow Keys + Diagonal controls.
- Live HUD displaying current velocity, odometry pose, and driving state.
"""

import os
import sys
import math
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── TurtleBot3 Burger Kinematic Limits ──
MAX_LIN_VEL = 0.22   # m/s
MAX_ANG_VEL = 2.84   # rad/s

# Default speeds
DEFAULT_LIN_SPEED = 0.20  # m/s
DEFAULT_ANG_SPEED = 0.80  # rad/s
LIN_STEP = 0.04           # m/s step per press
ANG_STEP = 0.20           # rad/s step per press

BANNER = """
╔══════════════════════════════════════════════════════════════════════════╗
║                TurtleBot3 – Interactive Teleop Controller                ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                          ║
║     [Q] Forward-Left        [W / ↑] Forward        [E] Forward-Right     ║
║     [A / ←] Turn Left       [SPACE / X] STOP       [D / →] Turn Right    ║
║     [Z] Reverse-Left        [S / ↓] Reverse        [C] Reverse-Right     ║
║                                                                          ║
║     [+] Speed +15%           [-] Speed -15%                              ║
║     [ESC / Ctrl+C] Quit                                                  ║
║                                                                          ║
║  * Tapping W / S / A / D keeps the robot driving steadily.               ║
║  * Live X, Y coordinates from Gazebo /odom are displayed in real-time.   ║
║  * Press SPACE or X to stop anytime!                                     ║
╚══════════════════════════════════════════════════════════════════════════╝
"""


def get_key_raw(timeout=0.08):
    """Non-blocking keyboard read with arrow key detection."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            # Arrow key escape sequence: \x1b[A, \x1b[B, \x1b[C, \x1b[D
            if ch == '\x1b':
                r2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if r2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        r3, _, _ = select.select([sys.stdin], [], [], 0.02)
                        if r3:
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A':
                                return 'UP'
                            elif ch3 == 'B':
                                return 'DOWN'
                            elif ch3 == 'C':
                                return 'RIGHT'
                            elif ch3 == 'D':
                                return 'LEFT'
                return 'ESC'
            return ch
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.speed_scale = 1.0
        self.lock = threading.Lock()
        self.state_label = "IDLE"

        # Live telemetry from Gazebo
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0
        self.has_odom = False

        # Continuous 10 Hz publisher timer ensures Gazebo diff_drive stays active
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_speed = msg.twist.twist.linear.x
            self.has_odom = True

    def timer_callback(self):
        with self.lock:
            msg = Twist()
            msg.linear.x = float(self.linear_vel)
            msg.angular.z = float(self.angular_vel)
            self.pub.publish(msg)

    def set_motion(self, lin, ang, label):
        with self.lock:
            max_lin = MAX_LIN_VEL * self.speed_scale
            max_ang = MAX_ANG_VEL * self.speed_scale
            self.linear_vel = max(-max_lin, min(max_lin, lin))
            self.angular_vel = max(-max_ang, min(max_ang, ang))
            self.state_label = label

    def step_linear(self, delta):
        with self.lock:
            new_lin = self.linear_vel + delta
            max_lin = MAX_LIN_VEL * self.speed_scale
            self.linear_vel = max(-max_lin, min(max_lin, new_lin))
            if abs(self.linear_vel) < 0.005:
                self.linear_vel = 0.0
            self.state_label = "FORWARD" if self.linear_vel > 0 else ("REVERSE" if self.linear_vel < 0 else "STOPPED")

    def step_angular(self, delta):
        with self.lock:
            new_ang = self.angular_vel + delta
            max_ang = MAX_ANG_VEL * self.speed_scale
            self.angular_vel = max(-max_ang, min(max_ang, new_ang))
            if abs(self.angular_vel) < 0.01:
                self.angular_vel = 0.0
            self.state_label = "TURN LEFT" if self.angular_vel > 0 else ("TURN RIGHT" if self.angular_vel < 0 else "DRIVING")

    def emergency_stop(self):
        with self.lock:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.state_label = "STOPPED"
        # Publish immediately
        msg = Twist()
        self.pub.publish(msg)

    def adjust_scale(self, factor):
        with self.lock:
            self.speed_scale = max(0.2, min(1.5, self.speed_scale + factor))
            max_lin = MAX_LIN_VEL * self.speed_scale
            max_ang = MAX_ANG_VEL * self.speed_scale
            self.linear_vel = max(-max_lin, min(max_lin, self.linear_vel))
            self.angular_vel = max(-max_ang, min(max_ang, self.angular_vel))

    def render_hud(self):
        with self.lock:
            lin = self.linear_vel
            ang = self.angular_vel
            scale = self.speed_scale
            lbl = self.state_label
            rx = self.robot_x
            ry = self.robot_y
            has_o = self.has_odom

        odom_str = f"Pos: ({rx:+5.2f}, {ry:+5.2f})m" if has_o else "Waiting for Gazebo /odom..."

        hud = (
            f"\r  [{lbl:<10}]  "
            f"Cmd: Lin={lin:+5.2f}m/s  Ang={ang:+5.2f}r/s  ({scale*100:3.0f}%)  |  "
            f"{odom_str}    "
        )
        sys.stdout.write(hud)
        sys.stdout.flush()


def main():
    rclpy.init()
    node = KeyboardTeleopNode()

    # Spin ROS 2 in a background thread so callbacks and timer fire continuously
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    os.system('clear' if os.name != 'nt' else 'cls')
    print(BANNER)
    node.render_hud()

    try:
        while rclpy.ok():
            key = get_key_raw(timeout=0.08)
            if not key:
                node.render_hud()
                continue

            # Exit
            if key == 'ESC':
                break
            if key == '\x03':  # Ctrl+C
                break

            # Emergency Stop
            if key in (' ', 'x', 'X'):
                node.emergency_stop()

            # Forward
            elif key in ('w', 'W', 'UP'):
                cur_lin = node.linear_vel
                if cur_lin < 0.05:
                    node.set_motion(DEFAULT_LIN_SPEED * node.speed_scale, 0.0, "FORWARD")
                else:
                    node.step_linear(LIN_STEP)

            # Reverse
            elif key in ('s', 'S', 'DOWN'):
                cur_lin = node.linear_vel
                if cur_lin > -0.05:
                    node.set_motion(-DEFAULT_LIN_SPEED * node.speed_scale, 0.0, "REVERSE")
                else:
                    node.step_linear(-LIN_STEP)

            # Rotate Left
            elif key in ('a', 'A', 'LEFT'):
                cur_ang = node.angular_vel
                if cur_ang < 0.1:
                    node.set_motion(node.linear_vel, DEFAULT_ANG_SPEED * node.speed_scale, "TURN LEFT")
                else:
                    node.step_angular(ANG_STEP)

            # Rotate Right
            elif key in ('d', 'D', 'RIGHT'):
                cur_ang = node.angular_vel
                if cur_ang > -0.1:
                    node.set_motion(node.linear_vel, -DEFAULT_ANG_SPEED * node.speed_scale, "TURN RIGHT")
                else:
                    node.step_angular(-ANG_STEP)

            # Diagonal combos
            elif key in ('q', 'Q'):
                node.set_motion(DEFAULT_LIN_SPEED * node.speed_scale * 0.8,
                                DEFAULT_ANG_SPEED * node.speed_scale * 0.7,
                                "FWD-LEFT")
            elif key in ('e', 'E'):
                node.set_motion(DEFAULT_LIN_SPEED * node.speed_scale * 0.8,
                                -DEFAULT_ANG_SPEED * node.speed_scale * 0.7,
                                "FWD-RIGHT")
            elif key in ('z', 'Z'):
                node.set_motion(-DEFAULT_LIN_SPEED * node.speed_scale * 0.8,
                                DEFAULT_ANG_SPEED * node.speed_scale * 0.7,
                                "REV-LEFT")
            elif key in ('c', 'C'):
                node.set_motion(-DEFAULT_LIN_SPEED * node.speed_scale * 0.8,
                                -DEFAULT_ANG_SPEED * node.speed_scale * 0.7,
                                "REV-RIGHT")

            # Speed Scaling
            elif key in ('+', '='):
                node.adjust_scale(+0.15)
            elif key in ('-', '_'):
                node.adjust_scale(-0.15)

            node.render_hud()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.emergency_stop()
        print("\n\n  [STOPPED] Controller exited safely.\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
