# DWM Robot

**Dog Waste Management Robot** is a tracked robotics project for detecting dog
waste, approaching a selected target and operating a cleaning brush. It combines
YOLO26n perception on an NVIDIA Jetson with OAK-D Pro depth, Arduino motor control,
RPLidar S2 sensing and ROS 2 navigation. A browser dashboard provides manual
driving, telemetry, cleaning controls, mapping and ordered waypoints.

## Engineering scope

| Area | Project implementation |
| --- | --- |
| Perception | Synchronized RGB/depth capture, target tracking, depth-based approach and a dark-on-grass fallback |
| Embedded integration | Serial command protocol, motor polarity, FG RPM feedback, active-low brush relay and sensor telemetry |
| Motion control | Differential track commands, wheel-speed regulation, acceleration limits and a timed cleaning sequence |
| Navigation | ROS sensor/odometry bridge, footprint-aware clearance, map persistence and ordered Nav2 goals |
| Operator interface | Live detection, depth, RPM, IMU, lidar/map views and browser control ownership |

The project integrates established libraries rather than implementing them from
scratch: Ultralytics supplies YOLO and BoT-SORT, DepthAI supplies camera access,
SLAM Toolbox supplies mapping, and Nav2 supplies localization/planning/control
servers. The application, hardware integration, control logic and browser-to-ROS
coordination are the project-specific parts. [Architecture](docs/architecture.md)
describes the interfaces and design decisions.

## Hardware operation

The project was developed and exercised on a physical tracked robot. Hardware
checks covered manual forward/reverse/left/right motion, wheel RPM feedback,
IMU and lidar heading comparisons, camera-based target approach, the forward
cleaning pass, brush timing and short waypoint navigation. The operator confirmed
motor directions and tested the approach/pass behavior on the robot.

This repository organizes the robot application as ROS 2 packages, with source
for control, perception, navigation, the dashboard and firmware. Regression
source files are included in `src/dwm_robot/test`.

Waypoint navigation and target cleaning operate as separate modes. Automatic
cleaning detours with route resumption and automatic pickup verification remain
future work. Absolute travel accuracy depends on FG pulse and wheel-scale
calibration.

## Detection results

The included YOLO26n checkpoint detects class 0, `dog_poop`, at a 640-pixel input
size. Runtime confidence is 0.30. The original training run contains 92 epochs;
the highest recorded mAP50-95 occurs at epoch 62:

| Validation metric | Value |
| --- | --- |
| Precision | 0.92049 |
| Recall | 0.81818 |
| mAP50 | 0.88437 |
| mAP50-95 | 0.76505 |

These values come directly from the retained [training CSV](src/dwm_perception/training/yolo26n_poop/results.csv).
They describe that validation split, not cleaning success or field reliability.
The [perception guide](docs/perception.md) documents the model checksum, retained
plots, dataset requirements and detector commands.

## Workspace layout

```text
dwm-robot/
├── src/
│   ├── dwm_core/          # Control, odometry, geometry and runtime paths
│   ├── dwm_perception/    # OAK capture, YOLO tools, runtime model and evaluation
│   ├── dwm_robot/         # Hardware server, browser dashboard and regression tests
│   ├── dwm_navigation/    # ROS bridge, map storage and waypoint execution
│   └── dwm_bringup/       # Launch files and platform configuration
├── firmware/             # Arduino DWM2 controller
├── docs/                 # Technical and operating documentation
├── requirements/         # Application and development dependencies
└── dwm_robot.repos       # External lidar-driver dependency
```

The five project packages use `ament_python` and build as a colcon workspace.
Runtime settings, maps, routes and logs are stored outside the repository.

## Build and launch

The target platform requires ROS 2 Jazzy and a JetPack-compatible CUDA
PyTorch/Torchvision environment. Follow [installation](docs/installation.md)
before building. Use the ROS system Python for the build:

```bash
cd ~/dwm-robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
```

Set `DWM_VISION_PYTHON` to the CUDA-capable interpreter on the Jetson, then launch:

```bash
ros2 launch dwm_bringup robot.launch.py hardware:=true \
  robot_python:="$DWM_VISION_PYTHON"
```

The launch starts the dashboard, bridge and lidar driver, with motion stopped.
Mapping, routes and cleaning start only through operator commands.
`hardware:=false` is the default and disables Arduino writes; camera and model
dependencies are still required. Launch arguments also select USB ports and GPU.

| Dashboard | URL |
| --- | --- |
| Detection, manual drive and cleaning | `http://<robot-ip>:8080/camera` |
| Mapping and waypoints | `http://<robot-ip>:8080/navigation` |

Control access is intended for a trusted local network. Use one visible control
page; hidden pages and expired control ownership can stop the robot.

## Documentation

- [Architecture](docs/architecture.md): package responsibilities, ROS interfaces,
  ownership, planning and implementation boundaries.
- [Hardware](docs/hardware.md): dimensions, wiring, frames and feedback assumptions.
- [Installation](docs/installation.md): dependencies, build, Python environments
  and writable data locations.
- [Operation](docs/operation.md): manual driving, cleaning, mapping, waypoints
  and troubleshooting.
- [Perception](docs/perception.md): training evidence, model provenance and CLI.

Maintained by [Rayan-01](https://github.com/Rayan-01).
Project and dependency licensing are described in [NOTICE](NOTICE.md).
