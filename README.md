# my-ros2

ROS 2 Jazzy based differential-drive robot workspace for a small mobile robot using Dynamixel XL430 motors, SLLidar C1, BNO085 IMU, SLAM Toolbox, and Nav2.

## Overview

This repository contains the `my_robot` package and supporting scripts used to:

- drive the robot with Dynamixel motors
- publish wheel odometry and IMU data
- run EKF-based localization
- build maps with SLAM Toolbox
- navigate to goals with Nav2

## Hardware

| Item | Value |
|------|-------|
| Board | Raspberry Pi 5 |
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Motors | Dynamixel XL430-W250-T x2 |
| LiDAR | SLLidar C1 |
| IMU | BNO085 |
| Base | 15 cm diameter differential-drive base |

## Repository Layout

```text
my-ros2/
├── src/my_robot/          # ROS 2 package
├── scripts/
│   ├── 99-my-robot.rules  # udev rules
│   └── test_imu_axes.py   # IMU axis test script
├── RUN_GUIDE.md           # detailed run guide
├── README.md
└── .gitignore
```

Generated workspace folders such as `build/`, `install/`, and `log/` are intentionally ignored and should not be committed.

## Dependencies

Install the main ROS packages:

```bash
sudo apt update
sudo apt install -y \
  chrony \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server
```

Python packages used by the hardware nodes:

```bash
pip install dynamixel_sdk adafruit-circuitpython-bno08x
```

This repository expects `sllidar_ros2` to be available in your ROS environment. Install it separately or place it in the same workspace.

## First-Time Setup

Install the udev rule:

```bash
sudo cp scripts/99-my-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Enable time sync:

```bash
sudo systemctl enable chrony
sudo systemctl start chrony
```

## Build

```bash
cd ~/my-ros2
colcon build
source install/setup.bash
```

## Run

Terminal 1:

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot bringup.launch.py
```

Terminal 2:

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot slam.launch.py
```

Terminal 3:

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot navigation.launch.py
```

Optional keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## RViz

Suggested RViz setup:

1. Set `Fixed Frame` to `map`
2. Add `/map`
3. Add `/scan`
4. Add `TF`
5. Use `2D Goal Pose` to send a target

## Documentation

- Detailed startup and troubleshooting guide: `RUN_GUIDE.md`
- IMU axis validation script: `scripts/test_imu_axes.py`

## Notes For GitHub

- Commit the source and documentation only.
- Do not commit `build/`, `install/`, or `log/`.
- If you publish this repository, consider updating the maintainer information in `src/my_robot/package.xml`.
