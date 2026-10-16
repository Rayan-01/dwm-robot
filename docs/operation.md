# Operating the robot

## Start and stop

Source ROS and the workspace in a Jetson terminal, then start the integrated
launch. Set `DWM_VISION_PYTHON` to the CUDA-capable interpreter described in
[installation](installation.md). Keep the terminal open during operation.

```bash
cd ~/dwm-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch dwm_bringup robot.launch.py hardware:=true \
  robot_python:="$DWM_VISION_PYTHON"
```

Use `start_lidar:=false` only when a single matching lidar driver is already
running. The launch also accepts `serial_port`, `lidar_port` and `device`.
It starts stopped. Use the dashboard Stop before closing the launch with Ctrl+C.

## Manual drive

1. Open `/camera` on the robot web server and wait for fresh telemetry.
2. Press **Enable manual**.
3. Hold **Forward**, **Reverse**, **Left** or **Right**. Release to stop wheels.
4. Press **Wheels stop** for drive only, or **STOP ROBOT + BRUSH** for both.

Space also stops the robot when focus is outside a text input.
Hiding the page or losing browser ownership can stop motion;
keep one controlling page visible. After a latched stop, enable the desired
mode again.

**Brush on** stops wheel movement before starting the brush. **Brush off** stops
the brush. Drive and brush run together only during the automatic forward pass.

## Automatic cleaning

Open `/camera`, select the pass settings and press **Auto detect + pass + clean**.
The default sequence is:

| Stage | Action |
| --- | --- |
| Approach | Centre the target and approach to about 45 cm from the camera |
| Pause | Stop for 1 second |
| Pass | Run brush and drive together for an estimated 1 m |
| Settle | Stop drive and brush for 2 seconds |
| Finish | Run brush alone for 20 seconds when the finish checkbox is enabled |
| Complete | Turn drive and brush off |

Pass-start distance is measured from the camera lens. Set the pass distance
and length through the dashboard before starting the cycle. A zero pass length
disables the forward pass.

## Create a map

1. Open `/navigation` and check the live lidar view.
2. Verify known walls appear on the correct side. Current mounting yaw is 0°.
3. Press **Create new map**.
4. Use the mapping **Hold forward/back/left/right** controls to explore.
5. Enter a unique name and press **Save map**.
6. End the map session, select the map and load it for navigation.

Mappings are saved as occupancy images with Nav2 YAML metadata. The new map store
does not overwrite an existing map name. Choose a new name for each revision.

## Run ordered waypoints

1. Load the correct map and check the current robot position.
2. If needed, select **Set robot position on map**, enter heading and click the
   track-midpoint position. Heading 0° faces screen-right; 90° faces screen-up.
3. Check the robot arrow and green lidar points against the map, then press
   **Confirm robot & scan match map**.
4. Click known free space to add waypoints in order. Scroll to zoom and drag to pan.
5. Save the route if it should persist, then press **Run route**.

The dashed orange line shows waypoint order, not the collision-free path. The
solid orange plan shows the Nav2 path when available. A wall may cause a detour
if a traversable path exists. Detections remain visible during navigation.

Run route displays any unmet readiness requirements beneath the button. Changing
the map session clears position confirmation. Reapplying the same lidar angle
preserves it. Screen-left is not necessarily robot-left; use the heading arrow.


## Troubleshooting

| Symptom | Checks |
| --- | --- |
| Dashboard unavailable | Confirm Jetson power, network address, port 8080 and server output in the launch terminal |
| Camera or depth missing | Confirm OAK USB/power and the DepthAI v2 environment; avoid a second camera owner |
| Lidar unavailable | Confirm one driver, the adapter path, 1,000,000 baud and ROS_DOMAIN_ID=42 |
| Serial/RPM errors | Confirm the Arduino port, permission, 115200 baud, one serial writer and FG pulse assumptions |
| Nav2 startup failure | End the current session; inspect runtime logs/navigation-session.log before retrying |
| Unexpected stop | Read the displayed reason; check page visibility, browser ownership, command age and sensor freshness |

An invalid depth pixel is not a reliable distance measurement. A map/scan
mismatch should be corrected through pose alignment or a new map, rather than
confirming an inaccurate position. Enable the desired mode again after resolving
the cause of a latched stop.
