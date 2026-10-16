# Installation

## Platform

The application targets a Jetson installation with ROS 2 Jazzy,
an OAK-D Pro over USB, an Arduino controller and an RPLidar S2 USB adapter.
Use the JetPack-compatible PyTorch and Torchvision builds already working on
the device. Generic PyPI torch wheels can replace the CUDA build and should not
be used as an automatic environment repair.

## ROS dependencies

Install the standard ROS development tools and runtime packages after following
the ROS 2 Jazzy installation instructions for the operating system:

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-pil python3-serial \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

If `rplidar_ros` is already installed, use it. Otherwise import its source from
the manifest before resolving dependencies:

```bash
cd ~/dwm-robot
source /opt/ros/jazzy/setup.bash
vcs import src < dwm_robot.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

Run `sudo rosdep init` once only if rosdep has not been initialized. The manifest
tracks the driver's ROS 2 branch; pin a known-working revision for a deployment.

## Build the workspace

Use the system Python that belongs to the ROS installation. Leave the vision
virtual environment deactivated while building.

```bash
cd ~/dwm-robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
```

Package dependencies are declared in each `package.xml`. The top-level workspace
is not a Python package and is not installed with `pip install .`.

## Vision environment

Select the existing CUDA-capable interpreter before installing application
dependencies. Set the path below to the actual interpreter on the Jetson. Review the proposed dependency changes before installation; retain
its working Torch/Torchvision versions. The existing deployment used Ultralytics
8.4.164 and the DepthAI v2 API.

```bash
export DWM_VISION_PYTHON=/absolute/path/to/cuda-environment/bin/python
"$DWM_VISION_PYTHON" -m pip install -r ~/dwm-robot/requirements/jetson.txt
```

OpenCV must provide `cv2`. Prefer the established Jetson OpenCV installation.
The DepthAI compatibility range in the requirements is not a lock of every
installed dependency. Use an interpreter matching the ROS Python ABI when
combining a virtual environment with the sourced workspace.

After sourcing `install/setup.bash`, the selected vision interpreter can import
the project packages through the workspace overlay. The ROS sensor bridge is
started as an ament executable using system Python; `robot_python` selects only
the hardware and vision server interpreter.

## USB and runtime data

The recorded Arduino device is `/dev/ttyACM0`. The lidar adapter was `/dev/ttyUSB0`
at 1,000,000 baud. Select the actual devices on the target machine. Serial access
usually requires membership in the `dialout` group; reconnect the login session
after changing group membership.

Writable state is separate from the source and install trees:

```text
~/.local/share/dwm_robot/
├── config.json
├── navigation_geometry.json
├── maps/<name>/map.yaml + map.pgm
├── routes/<map-name>.json
└── logs/
```

Set `DWM_DATA_DIR` to use a different directory. Defaults are copied from
`dwm_bringup/config` on first startup; existing runtime settings are preserved.
Editing package defaults does not overwrite settings already in the data folder.

## Firmware

The working firmware source is `firmware/dwm_robot_safe/dwm_robot_safe.ino`.
It uses Wire, Adafruit Unified Sensor, Adafruit BNO055 and Adafruit ADS1X15.
Select the actual AVR board and processor in the Arduino tools. Firmware upload
is separate from colcon and is not performed by any launch file.


## Source checks

Regression sources are kept in `src/dwm_robot/test`. They cover control state,
telemetry parsing, HTTP behavior, wheel/gyro math, vision fallback and firmware
protocol behavior. Some checks require Linux PTY support or a host C++ compiler.
The package reorganization has not been validated by a colcon build or test run.
No passing result is claimed for this checkout.
