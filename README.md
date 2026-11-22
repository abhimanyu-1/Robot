# Robot: Robust Perception and State Estimation for Mobile Robots

An Extended Kalman Filter (EKF) state estimation framework for mobile robots fusing differential drive wheel odometry with IMU orientation measurements to eliminate systematic drift and wheel slip error. Tested on a visually enhanced TurtleBot3 Burger in ROS 2 Humble and Gazebo Classic 11.

---

## 📁 Repository Structure

```text
Robot/
├── README.md                        # Documentation and instructions
├── Report/                          # Project documentation and progress reports
│   ├── *.docx / *.pdf
└── robot_project/                   # Core estimation, simulation & control package
    ├── config/
    │   └── settings.yaml            # Simulation noise & EKF process/measurement covariances
    ├── data/                        # Recorded or synthetic trajectory CSV files
    ├── launch/
    │   └── virtual_room.launch.py   # Launch Gazebo + enhanced robot + world
    ├── models/
    │   └── turtlebot3_burger_enhanced/
    │       ├── model.config         # Gazebo model metadata
    │       └── model.sdf            # Enhanced visual TurtleBot3 Burger (cyan LiDAR, blue-tint body)
    ├── robot_estimation/
    │   ├── __init__.py
    │   ├── ekf_estimator.py         # Extended Kalman Filter implementation
    │   ├── evaluation.py            # Benchmark script (RMSE, plots, metrics)
    │   └── odometry_baseline.py     # Pure dead-reckoning baseline
    ├── results/                     # Generated plots and metric CSVs
    ├── scripts/
    │   ├── keyboard_controller.py   # WASD keyboard controller (publishes /cmd_vel)
    │   └── record_data.py           # ROS 2 data recorder & synthetic data generator
    └── worlds/
        ├── robotics_lab.world       # Modern 3D robotics laboratory environment
        └── room_with_obstacles.world # Legacy basic room (fallback)
```

---

## 🚀 Setup & Installation Guide

### Step 1: Install Ubuntu 22.04 on Windows (WSL2)

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

Run the following commands inside Ubuntu 22.04:

```bash
# 1. Configure locale and add ROS 2 apt repository
sudo apt install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

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

**Verification:**
```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

---

### Step 3: Build the TurtleBot3 Simulation Workspace

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

---

## 🏃 How to Run the Project

### Full Simulation Pipeline (3 Terminals)

> [!IMPORTANT]
> If Gazebo crashes or shows a black screen in WSL2, prepend `export LIBGL_ALWAYS_SOFTWARE=1` before launching.

#### Terminal 1 — Launch Gazebo World + Enhanced Robot

```bash
cd /mnt/e/Project/Robot
source /opt/ros/humble/setup.bash
source turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch robot_project/launch/virtual_room.launch.py
```

This opens the **Robotics Laboratory** world with realistic lighting, obstacles, lab furniture, and spawns the **enhanced TurtleBot3 Burger** (dark charcoal body, glowing cyan LiDAR).

#### Terminal 2 — Keyboard Controller (WASD)

```bash
cd /mnt/e/Project/Robot
source /opt/ros/humble/setup.bash

python3 robot_project/scripts/keyboard_controller.py
```

**Keyboard Controls:**

| Key | Action |
|-----|--------|
| **W** / **↑** | Move forward |
| **S** / **↓** | Move backward |
| **A** / **←** | Rotate left (CCW) |
| **D** / **→** | Rotate right (CW) |
| **Q** | Forward + left |
| **E** | Forward + right |
| **Z** | Backward + left |
| **C** | Backward + right |
| **Space** / **X** | Emergency stop |
| **+** / **-** | Increase / decrease speed ±10% |
| **ESC** | Quit |

#### Terminal 3 — Record Data + Run EKF Evaluation

```bash
cd /mnt/e/Project/Robot/robot_project
source /opt/ros/humble/setup.bash

# Record 30–60 seconds of driving data, then press Ctrl+C
python3 scripts/record_data.py --output data/trajectory_run.csv

# Run Odometry Baseline vs. EKF Sensor Fusion evaluation
python3 -m robot_estimation.evaluation
```

---

### Quick Run with Synthetic Data (No Gazebo Needed)

```bash
cd /mnt/e/Project/Robot/robot_project

# 1. Generate 60-second synthetic trajectory with realistic slip & IMU noise
python3 scripts/record_data.py --synthetic --output data/trajectory_run.csv

# 2. Run evaluation
python3 -m robot_estimation.evaluation
```

---

## 📊 Outputs & Evaluation

After running `evaluation.py`, results are generated in `robot_project/results/`:

| Method | RMSE (m) | Mean Error (m) | Max Error (m) | Final Drift (m) |
|--------|----------|-----------------|---------------|------------------|
| Wheel Odometry Baseline | ~1.0827 | ~0.9412 | ~2.1540 | ~1.8540 |
| **EKF Sensor Fusion (Odom + IMU)** | **~0.0321** | **~0.0284** | **~0.0712** | **~0.0450** |

- **`trajectories.png`** — 2D path comparison (ground truth, pure odometry drift, EKF corrected)
- **`position_error.png`** — Euclidean estimation error over time
- **`metrics.csv`** — Tabulated accuracy metrics

---

## 🌍 Simulation Environment

The **Robotics Laboratory** world (`robotics_lab.world`) features:

- 🏭 **Realistic industrial lab** with epoxy floor, yellow safety lines, and green start pad
- 💡 **Multi-source lighting** — directional sun, 4 ceiling point lights, accent spotlight, emissive light fixtures
- 🪟 **Glass partition** with metal frame and solid corridor wall
- 🖥️ **Workstation** with desk and monitor, **lab table** with oscilloscope
- 📦 **Storage rack** with equipment boxes, shipping crates, pallet with cardboard
- 🛢️ **Industrial barrels**, equipment cabinet, structural column
- 🔶 **Traffic cones** as navigation markers
- 🤖 **Enhanced TurtleBot3** — dark charcoal body with blue tint, glowing cyan LiDAR ring, matte black wheels

---

## 🛠️ Troubleshooting

| Problem | Solution |
|---------|----------|
| `gzserver process has died with exit code 255` | Kill orphaned processes: `killall -9 gzserver gzclient 2>/dev/null` |
| Gazebo screen is black or crashes on WSL2 | Enable software rendering: `export LIBGL_ALWAYS_SOFTWARE=1` |
| `No module named 'rclpy'` | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| Robot does not move with keyboard | Ensure keyboard controller terminal is in focus and has raw terminal access |
| `spawn_entity` service unavailable | Wait for gzserver to fully start, or restart the launch |

---

## 📄 License

This project is part of a university research project on **Robust Perception and State Estimation for Mobile Robots**.
