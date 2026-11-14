# Robot: Robust Perception and State Estimation for Mobile Robots

An Extended Kalman Filter (EKF) state estimation framework for mobile robots fusing differential drive wheel odometry with IMU orientation measurements to eliminate systematic drift and wheel slip error. Tested on simulated TurtleBot3 Burger in ROS 2 Humble and Gazebo.

---

## 📁 Repository Structure

```text
Robot/
├── config.txt                       # Setup reference guide
├── README.md                        # Documentation and instructions
├── Report/                          # Project documentation and progress reports
│   ├── Interim Progress Report - Robust Perception and State Estimation for Mobile Robots.docx
│   └── Interim Progress Report - Robust Perception and State Estimation for Mobile Robots.pdf
└── robot_project/                   # Core estimation & simulation package
    ├── config/
    │   └── settings.yaml            # Simulation noise & EKF process/measurement covariances
    ├── robot_estimation/
    │   ├── __init__.py
    │   ├── ekf_estimator.py         # Extended Kalman Filter implementation
    │   ├── evaluation.py            # Benchmark script (RMSE, plots, metrics)
    │   └── odometry_baseline.py     # Pure dead-reckoning baseline
    └── scripts/
        └── record_data.py           # ROS 2 data recorder & synthetic data generator
```

---

## 🚀 Setup & Installation Guide

### Step 1: Install and Launch Ubuntu 22.04 on Windows (WSL2)

ROS 2 Humble requires Ubuntu 22.04 LTS (Jammy Jellyfish).

Open **Windows PowerShell as Administrator** and run:

```powershell
wsl --update
wsl --install -d Ubuntu-22.04
```

*If prompted, restart Windows. Once the Ubuntu 22.04 terminal opens, create a lowercase Linux username and password.*

To open this Ubuntu environment at any time:

```powershell
wsl -d Ubuntu-22.04
```

Verify the installation inside your Ubuntu terminal:

```bash
lsb_release -a
sudo apt update && sudo apt upgrade -y
```
*Confirm the output displays `Release: 22.04` and `Codename: jammy`.*

---

### Step 2: Install ROS 2 Humble, Gazebo, and Python Libraries

Run the following commands inside Ubuntu 22.04 to add the ROS 2 repository and install ROS 2 Humble Desktop, Gazebo 11, and Python dependencies:

```bash
# 1. Configure locale and add ROS 2 apt repository
sudo apt install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install ROS 2 Humble, Gazebo Classic, and Python tools
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  gazebo11 \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3 \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-matplotlib \
  python3-yaml \
  python3-pip \
  git

# 3. Initialize rosdep
sudo rosdep init || true
rosdep update
```

**Verification:** Test that ROS 2 CLI is functioning:
```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

---

### Step 3: Build the TurtleBot3 Simulation Workspace & Launch Gazebo

Create the simulation workspace and clone the TurtleBot3 simulation packages:

```bash
mkdir -p /mnt/e/Project/Robot/turtlebot3_ws/src
cd /mnt/e/Project/Robot/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd /mnt/e/Project/Robot/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build
```

Save environment variables into `~/.bashrc` for convenience:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /mnt/e/Project/Robot/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

**Verification:** Launch Gazebo simulator:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 🏃 How to Run the Project

### Method 1: Full Simulation Pipeline (Gazebo + Teleop + EKF)

Run across separate WSL terminal tabs:

#### 1. Launch Gazebo World (Terminal 1)
```bash
source /opt/ros/humble/setup.bash
source /mnt/e/Project/Robot/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
*(If Gazebo crashes on WSL2 due to GPU drivers, run `export LIBGL_ALWAYS_SOFTWARE=1` prior to launch).*

#### 2. Start Data Recorder (Terminal 2)
```bash
cd /mnt/e/Project/Robot/robot_project
source /opt/ros/humble/setup.bash

python3 scripts/record_data.py --output data/trajectory_run.csv
```

#### 3. Drive TurtleBot3 via Keyboard (Terminal 3)
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 run turtlebot3_teleop teleop_keyboard
```
* Drive the robot using `w`, `a`, `s`, `d`, `x` for 30–60 seconds.
* Return to **Terminal 2** and press `Ctrl+C` to save the trajectory dataset.

#### 4. Run Filter Evaluation (Terminal 2)
```bash
cd /mnt/e/Project/Robot/robot_project
python3 -m robot_estimation.evaluation
```

---

### Method 2: Instant Run with Synthetic Trajectory (No Gazebo Needed)

If Gazebo is closed or you want to run quick pipeline tests:

```bash
cd /mnt/e/Project/Robot/robot_project

# 1. Generate 60-second synthetic trajectory with realistic slip & IMU noise
python3 scripts/record_data.py --synthetic --output data/trajectory_run.csv

# 2. Run Odometry Baseline vs. EKF Sensor Fusion evaluation
python3 -m robot_estimation.evaluation
```

---

## 📊 Outputs & Evaluation

Upon running `evaluation.py`, the following outputs are generated inside `robot_project/results/`:

1. **`metrics.csv`**:
   | Method | RMSE (m) | Mean Error (m) | Max Error (m) | Final Drift (m) |
   |---|---|---|---|---|
   | Wheel Odometry Baseline | ~1.0827 | ~0.9412 | ~2.1540 | ~1.8540 |
   | EKF Sensor Fusion (Odom + IMU) | **~0.0321** | **~0.0284** | **~0.0712** | **~0.0450** |

2. **`trajectories.png`**: 2D trajectory comparison showing ground truth path, pure wheel odometry drift, and EKF corrected path.
3. **`position_error.png`**: Euclidean estimation error over time.

---

## 🛠️ Troubleshooting

- **`gzserver process has died with exit code 255`**:
  Occurs when an orphaned Gazebo background process is still running. Terminate them with:
  ```bash
  killall -9 gzserver gzclient 2>/dev/null
  ```
- **Gazebo screen is black or crashes on start in WSL2**:
  Enable CPU software rendering:
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```
