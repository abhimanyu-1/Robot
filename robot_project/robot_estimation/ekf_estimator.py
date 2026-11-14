import math
import numpy as np
from robot_estimation.odometry_baseline import wrap_to_pi

class ExtendedKalmanFilter:
    def __init__(self, initial_state, q_diag, r_imu_yaw):
        self.x = np.array(initial_state, dtype=float)
        self.P = np.eye(3) * 0.01
        self.Q = np.diag(q_diag)
        self.R = np.array([[r_imu_yaw]])
        self.H = np.array([[0.0, 0.0, 1.0]])

    def predict(self, v_m: float, w_m: float, dt: float):
        theta = self.x[2]
        self.x[0] += v_m * dt * math.cos(theta)
        self.x[1] += v_m * dt * math.sin(theta)
        self.x[2] = wrap_to_pi(theta + w_m * dt)

        F = np.array([
            [1.0, 0.0, -v_m * dt * math.sin(theta)],
            [0.0, 1.0,  v_m * dt * math.cos(theta)],
            [0.0, 0.0,  1.0]
        ])
        self.P = F @ self.P @ F.T + self.Q

    def update_imu_yaw(self, imu_yaw: float):
        innovation = wrap_to_pi(imu_yaw - self.x[2])
        y = np.array([innovation])
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.x[2] = wrap_to_pi(self.x[2])
        self.P = (np.eye(3) - K @ self.H) @ self.P

def run_ekf_estimator(timestamps, v_noisy, w_noisy, imu_yaw, initial_state, q_diag, r_imu_yaw):
    n = len(timestamps)
    estimates = np.zeros((n, 3))
    estimates[0] = initial_state
    ekf = ExtendedKalmanFilter(initial_state, q_diag, r_imu_yaw)

    for k in range(1, n):
        dt = max(timestamps[k] - timestamps[k - 1], 1e-4)
        ekf.predict(v_noisy[k], w_noisy[k], dt)
        ekf.update_imu_yaw(imu_yaw[k])
        estimates[k] = ekf.x.copy()

    return estimates

