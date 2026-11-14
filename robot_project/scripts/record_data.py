import argparse
import csv
import math
import os
import numpy as np

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def generate_synthetic_csv(output_path: str):
    """Generates a realistic 60-second TurtleBot3 trajectory if Gazebo is not running."""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    dt = 0.05
    steps = 1200
    x, y, yaw = 0.0, 0.0, 0.0

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'gt_x', 'gt_y', 'gt_yaw', 'v_true', 'w_true', 'imu_yaw', 'imu_wz'])
        rng = np.random.default_rng(42)

        for i in range(steps):
            t = i * dt
            v = 0.18
            w = 0.25 * math.sin(0.15 * t) + 0.10 * math.cos(0.07 * t)
            x += v * dt * math.cos(yaw)
            y += v * dt * math.sin(yaw)
            yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))

            imu_yaw = math.atan2(
                math.sin(yaw + rng.normal(0.0, 0.015)),
                math.cos(yaw + rng.normal(0.0, 0.015))
            )
            imu_wz = w + rng.normal(0.0, 0.01)

            writer.writerow([
                f"{t:.4f}", f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}",
                f"{v:.5f}", f"{w:.5f}", f"{imu_yaw:.5f}", f"{imu_wz:.5f}"
            ])
    print(f"Saved {steps} synthetic sensor rows to {output_path}")

def record_from_ros2(output_path: str):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    class DataRecorder(Node):
        def __init__(self):
            super().__init__('data_recorder')
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
            self.latest_imu_yaw = 0.0
            self.latest_imu_wz = 0.0
            self.csv_file = open(output_path, 'w', newline='')
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(['timestamp', 'gt_x', 'gt_y', 'gt_yaw', 'v_true', 'w_true', 'imu_yaw', 'imu_wz'])
            self.get_logger().info(f"Recording to {output_path}... Drive TurtleBot3 now. Press Ctrl+C to finish.")

        def imu_cb(self, msg: Imu):
            self.latest_imu_yaw = quaternion_to_yaw(msg.orientation)
            self.latest_imu_wz = msg.angular_velocity.z

        def odom_cb(self, msg: Odometry):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gt_x = msg.pose.pose.position.x
            gt_y = msg.pose.pose.position.y
            gt_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            self.writer.writerow([
                f"{t:.4f}", f"{gt_x:.5f}", f"{gt_y:.5f}", f"{gt_yaw:.5f}",
                f"{v:.5f}", f"{w:.5f}", f"{self.latest_imu_yaw:.5f}", f"{self.latest_imu_wz:.5f}"
            ])

        def destroy_node(self):
            self.csv_file.close()
            super().destroy_node()

    rclpy.init()
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nRecording saved to {output_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='data/trajectory_run.csv')
    parser.add_argument('--synthetic', action='store_true', help='Generate synthetic trajectory without Gazebo')
    args = parser.parse_args()

    if args.synthetic:
        generate_synthetic_csv(args.output)
    else:
        record_from_ros2(args.output)

