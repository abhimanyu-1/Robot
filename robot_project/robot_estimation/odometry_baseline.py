import math
import numpy as np

def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def run_odometry_baseline(timestamps, v_noisy, w_noisy, initial_state):
    """Dead-reckoning trajectory estimation using only wheel odometry."""
    n = len(timestamps)
    estimates = np.zeros((n, 3))
    estimates[0] = initial_state

    for k in range(1, n):
        dt = max(timestamps[k] - timestamps[k - 1], 1e-4)
        x_prev, y_prev, theta_prev = estimates[k - 1]

        x_new = x_prev + v_noisy[k] * dt * math.cos(theta_prev)
        y_new = y_prev + v_noisy[k] * dt * math.sin(theta_prev)
        theta_new = wrap_to_pi(theta_prev + w_noisy[k] * dt)

        estimates[k] = [x_new, y_new, theta_new]

    return estimates

