import csv
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from robot_estimation.odometry_baseline import run_odometry_baseline
from robot_estimation.ekf_estimator import run_ekf_estimator

def compute_metrics(gt_xy, est_xy):
    errors = np.linalg.norm(gt_xy - est_xy, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    mean_err = float(np.mean(errors))
    max_err = float(np.max(errors))
    final_err = float(errors[-1])
    return errors, rmse, mean_err, max_err, final_err

def main():
    with open('config/settings.yaml', 'r') as f:
        cfg = yaml.safe_load(f)

    data = np.genfromtxt('data/trajectory_run.csv', delimiter=',', skip_header=1)
    timestamps = data[:, 0]
    gt_x, gt_y, gt_yaw = data[:, 1], data[:, 2], data[:, 3]
    v_true, w_true = data[:, 4], data[:, 5]
    imu_yaw = data[:, 6]

    rng = np.random.default_rng(cfg['simulation']['seed'])
    v_noisy = v_true + rng.normal(0.0, cfg['simulation']['wheel_linear_noise_std'], size=len(timestamps))
    w_noisy = (
        w_true
        + cfg['simulation']['wheel_angular_bias']
        + rng.normal(0.0, cfg['simulation']['wheel_angular_noise_std'], size=len(timestamps))
    )

    init_state = [gt_x[0], gt_y[0], gt_yaw[0]]
    odom_est = run_odometry_baseline(timestamps, v_noisy, w_noisy, init_state)
    ekf_est = run_ekf_estimator(
        timestamps, v_noisy, w_noisy, imu_yaw, init_state,
        cfg['ekf']['q_diag'], cfg['ekf']['r_imu_yaw']
    )

    gt_xy = np.column_stack((gt_x, gt_y))
    odom_err, odom_rmse, odom_mean, odom_max, odom_final = compute_metrics(gt_xy, odom_est[:, :2])
    ekf_err, ekf_rmse, ekf_mean, ekf_max, ekf_final = compute_metrics(gt_xy, ekf_est[:, :2])

    os.makedirs('results', exist_ok=True)
    with open('results/metrics.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Method', 'RMSE_m', 'Mean_Error_m', 'Max_Error_m', 'Final_Drift_m'])
        writer.writerow(['Wheel Odometry Baseline', f"{odom_rmse:.4f}", f"{odom_mean:.4f}", f"{odom_max:.4f}", f"{odom_final:.4f}"])
        writer.writerow(['EKF Sensor Fusion (Odom + IMU)', f"{ekf_rmse:.4f}", f"{ekf_mean:.4f}", f"{ekf_max:.4f}", f"{ekf_final:.4f}"])

    print(f"Odometry Baseline RMSE : {odom_rmse:.4f} m")
    print(f"EKF Sensor Fusion RMSE : {ekf_rmse:.4f} m")

    # Plot 1: 2D Trajectory Comparison (Output 2)
    plt.figure(figsize=(8, 6))
    plt.plot(gt_x, gt_y, 'k-', linewidth=2.2, label='Ground Truth (Simulator)')
    plt.plot(odom_est[:, 0], odom_est[:, 1], 'r--', linewidth=1.8, label=f'Wheel Odometry (RMSE={odom_rmse:.3f}m)')
    plt.plot(ekf_est[:, 0], ekf_est[:, 1], 'b-.', linewidth=2.0, label=f'EKF Fusion (RMSE={ekf_rmse:.3f}m)')
    plt.scatter([gt_x[0]], [gt_y[0]], c='green', s=70, zorder=5, label='Start')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('2D Mobile Robot Trajectory Comparison')
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('results/trajectories.png', dpi=300)
    plt.close()

    # Plot 2: Position Error Over Time (Output 3)
    rel_time = timestamps - timestamps[0]
    plt.figure(figsize=(8, 4.5))
    plt.plot(rel_time, odom_err, 'r-', linewidth=1.6, label='Wheel Odometry Error')
    plt.plot(rel_time, ekf_err, 'b-', linewidth=1.8, label='EKF Sensor Fusion Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Euclidean Position Error (m)')
    plt.title('Position Estimation Error Over Time')
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    plt.savefig('results/position_error.png', dpi=300)
    plt.close()
    print("Saved plots to results/trajectories.png and results/position_error.png")

if __name__ == '__main__':
    main()

