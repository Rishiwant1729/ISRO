# ISRO Autonomous Drone — GPS-Denied Navigation & Precision Landing

An autonomous drone navigation system built using **ROS 2 Jazzy, Intel RealSense D455, RTAB-Map, MAVROS, and ArduPilot**.

The project aims to enable autonomous drone navigation without relying on conventional GPS positioning. It uses RGB-D visual odometry and IMU data to estimate motion, communicates navigation information to the flight controller through MAVROS, and supports autonomous path execution and ArUco-marker-based precision landing.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Running the System](#running-the-system)
- [ROS 2 Topics](#ros-2-topics)
- [Flight Control](#flight-control)
- [Autonomous Missions](#autonomous-missions)
- [ArUco Precision Landing](#aruco-precision-landing)
- [Configuration](#configuration)
- [Monitoring and Debugging](#monitoring-and-debugging)
- [Troubleshooting](#troubleshooting)
- [Safety Considerations](#safety-considerations)
- [Future Development](#future-development)

---

# 1. Overview

Traditional autonomous drones commonly depend on GPS for outdoor positioning. However, GPS may be unavailable, unreliable, or unsuitable in certain environments.

This project explores a vision-based navigation approach using an Intel RealSense D455 camera and an onboard companion computer.

The system processes camera and IMU information to estimate the drone's movement. The resulting odometry is forwarded to ArduPilot through MAVROS, allowing the flight controller to use External Navigation through EKF3.

The project also includes autonomous movement controllers and an ArUco-based landing system.

### Navigation Pipeline

1. Capture RGB, depth, and IMU data from the RealSense D455.
2. Process IMU measurements using Madgwick filtering.
3. Estimate motion using RTAB-Map RGB-D odometry.
4. Publish visual odometry through `/vio/odometry`.
5. Forward odometry to the flight controller through MAVROS.
6. Use ArduPilot EKF3 ExternalNav for navigation.
7. Execute autonomous flight patterns.
8. Detect and align with an ArUco marker for precision landing.

---

# 2. Features

| Feature | Description |
|---|---|
| GPS-denied navigation | Uses visual odometry instead of relying on GPS position |
| RGB-D perception | Uses RGB and depth information from RealSense D455 |
| IMU integration | Uses gyroscope and accelerometer measurements |
| Visual odometry | Estimates motion using RTAB-Map |
| External Navigation | Provides odometry to ArduPilot EKF3 |
| MAVROS integration | Communication between ROS 2 and the flight controller |
| Keyboard control | Supports manual velocity commands through keyboard input |
| Autonomous square path | Executes a configured square movement pattern |
| Yaw hold | Supports heading correction during autonomous movement |
| Lawnmower pattern | Supports area-coverage-style autonomous movement |
| ArUco detection | Detects a configured landing marker |
| Precision landing | Supports marker alignment and controlled landing |
| Health monitoring | Checks the navigation and MAVROS system state |

---

# 3. System Architecture

```text
                  INTEL REALSENSE D455
                           |
             +-------------+-------------+
             |             |             |
          RGB IMAGE    DEPTH IMAGE      IMU
             |             |             |
             +-------------+-------------+
                           |
                     IMU FILTERING
                      (Madgwick)
                           |
                           |
                    RGB-D ODOMETRY
                      (RTAB-Map)
                           |
                           |
                    /vio/odometry
                           |
                           |
                         MAVROS
                           |
                           |
                    ARDUPILOT FCU
                           |
                           |
                         EKF3
                      ExternalNav
                           |
                           |
                    FLIGHT CONTROL
                           |
              +------------+------------+
              |            |            |
           Keyboard     Autonomous    Precision
            Control       Paths        Landing
                                      (ArUco)
```

### Main Software Components

**RealSense Camera Driver**

Publishes RGB, depth, and IMU data from the D455.

**Madgwick Filter**

Processes IMU measurements to estimate orientation.

**RTAB-Map RGB-D Odometry**

Uses camera images and depth information to estimate camera motion.

**VIO Bridge**

Connects the visual odometry pipeline to the project's `/vio/odometry` interface.

**MAVROS**

Provides communication between ROS 2 and ArduPilot using MAVLink.

**ArduPilot EKF3**

Uses the configured ExternalNav information for navigation estimation.

**Flight Controllers**

Provide keyboard-based control and autonomous mission execution.

---

# 4. Hardware Requirements

| Component | Purpose |
|---|---|
| Raspberry Pi 5 | Onboard companion computer |
| Intel RealSense D455 | RGB-D camera and IMU |
| Pixhawk flight controller | Flight stabilization and control |
| ArduPilot-compatible drone | Flight platform |
| Motors and ESCs | Propulsion |
| Battery | Power supply |
| ArUco marker | Precision landing reference |

### RealSense D455 Configuration

| Parameter | Value |
|---|---|
| Camera | Intel RealSense D455 |
| Serial Number | `035322250580` |
| Firmware | `5.17.3.10` |
| Color Resolution | 640 × 480 |
| Depth Resolution | 640 × 480 |
| RGB Frame Rate | 30 FPS |
| Depth Frame Rate | 30 FPS |
| Gyroscope Rate | 400 Hz |
| Accelerometer Rate | 250 Hz |
| Depth Alignment | Enabled |

---

# 5. Software Requirements

| Software | Version |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| DDS Middleware | Cyclone DDS |
| ArduPilot | ArduCopter 4.7.1 |
| MAVROS | ROS 2 Jazzy compatible |
| RealSense SDK | librealsense 2.58.4 |
| RealSense ROS Wrapper | 4.58.4 |
| RTAB-Map | ROS 2 compatible |
| Python | Ubuntu system Python |

The project is designed around Ubuntu 24.04 and ROS 2 Jazzy.

---

# 6. Repository Structure

```text
isro_final/
│
├── src/
│   └── irocu_vio_bridge/
│       ├── launch/
│       │   └── vio_pipeline.launch.py
│       │
│       ├── irocu_vio_bridge/
│       │   ├── __init__.py
│       │   ├── ov_to_vio.py
│       │   ├── phase2_health_check.py
│       │   ├── phase2_takeoff_land.py
│       │   ├── phase3_velocity_test.py
│       │   ├── phase3_square_path.py
│       │   ├── phase3_square_yaw_hold.py
│       │   ├── phase3_keyboard_control.py
│       │   ├── phase3_keyboard_rth.py
│       │   ├── phase3_lawnmower_path.py
│       │   ├── phase3_lawnmower_aruco.py
│       │   ├── phase3_aruco_detector.py
│       │   └── phase3_aruco_detector_custom.py
│       │
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
├── contexts/
├── control_commands.json
├── README.md
└── ...
```

*This is a representative overview of the main package components. Additional files may exist in the repository.*

---

# 7. Installation

## 7.1 Install ROS 2 Jazzy

Install Ubuntu 24.04 and ROS 2 Jazzy following the official ROS installation guide:

https://docs.ros.org/en/jazzy/Installation.html

Load ROS in the terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

## 7.2 Install Development Tools

```bash
sudo apt update

sudo apt install -y \
  git \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential
```

## 7.3 Install ROS Dependencies

```bash
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-opencv
```

Install Python dependencies used by the relevant nodes:

```bash
sudo apt install -y \
  python3-numpy \
  python3-opencv
```

> Package availability depends on the configured Ubuntu and ROS repositories. Verify all dependencies against `package.xml` and the actual launch files before using a fresh installation.

## 7.4 MAVROS Geographic Dataset

Install the GeographicLib datasets required by MAVROS:

```bash
sudo geographiclib-get-geoids egm96-5
```

If the command is unavailable, install the GeographicLib tools package first.

---

# 8. Building the Workspace

Clone the repository:

```bash
git clone https://github.com/Rishiwant1729/ISRO.git
```

Enter the project:

```bash
cd ISRO
```

Source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Install dependencies:

```bash
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --packages-select irocu_vio_bridge --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

For every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/isro_final/install/setup.bash
```

---

# 9. Running the System

The following workflow uses separate terminals over SSH.

All terminals should run on the Raspberry Pi unless otherwise specified.

## Terminal 1 — RealSense D455

Connect the D455 to a USB 3 port.

Start the camera:

```bash
source /opt/ros/jazzy/setup.bash

pkill -f realsense2_camera

ros2 launch realsense2_camera rs_launch.py \
camera_namespace:=camera \
camera_name:=d455 \
enable_color:=true \
enable_depth:=true \
enable_infra1:=false \
enable_infra2:=false \
enable_gyro:=true \
enable_accel:=true \
unite_imu_method:=2 \
align_depth.enable:=true \
enable_sync:=true \
rgb_camera.color_profile:=640x480x30 \
depth_module.depth_profile:=640x480x30 \
rgb_camera.color_format:=RGB8 \
gyro_fps:=400 \
accel_fps:=250
```

Expected topics:

```text
/camera/d455/color/image_raw
/camera/d455/color/camera_info
/camera/d455/aligned_depth_to_color/image_raw
/camera/d455/imu
/camera/d455/imu_filtered
```

Keep this terminal running.

## Terminal 2 — VIO Pipeline

```bash
cd ~/isro_final

source /opt/ros/jazzy/setup.bash
source ~/isro_final/install/setup.bash

ros2 launch irocu_vio_bridge vio_pipeline.launch.py
```

This starts the configured visual odometry pipeline.

Expected main output:

```text
/vio/odometry
```

Keep this terminal running.

## Terminal 3 — MAVROS

Connect the Pixhawk to the Raspberry Pi.

Check the serial device:

```bash
ls -l /dev/ttyACM*
```

Start MAVROS:

```bash
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 launch mavros apm.launch \
fcu_url:=/dev/ttyACM0:57600
```

If the device is different, replace `/dev/ttyACM0` accordingly.

Keep this terminal running.

## Terminal 4 — Static TF Publishers

Start the camera frame transform:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0 \
--roll 3.141592653589793 \
--pitch 0 \
--yaw 0 \
--frame-id d455_link \
--child-frame-id d455_link_frd &
```

Start the odometry frame transform:

```bash
ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0 \
--roll 3.141592653589793 \
--pitch 0 \
--yaw 1.5707963267948966 \
--frame-id odom \
--child-frame-id odom_ned
```

Keep the second process running.

## Terminal 5 — MAVROS Runtime Configuration

Source the environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/isro_final/install/setup.bash
```

Configure odometry frames:

```bash
ros2 param set /mavros/odometry fcu.map_id_des odom

ros2 param set /mavros/odometry fcu.odom_parent_id_des odom

ros2 param set /mavros/odometry fcu.odom_child_id_des d455_link
```

Set the geographic origin:

```bash
ros2 topic pub --once \
/mavros/global_position/set_gp_origin \
geographic_msgs/msg/GeoPointStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, position: {latitude: 28.000000, longitude: 77.000000, altitude: 0.0}}"
```

Configure MAVROS stream rates:

```bash
ros2 service call /mavros/set_stream_rate \
mavros_msgs/srv/StreamRate \
"{stream_id: 6, message_rate: 10, on_off: true}"
```

```bash
ros2 service call /mavros/set_stream_rate \
mavros_msgs/srv/StreamRate \
"{stream_id: 0, message_rate: 10, on_off: true}"
```

Repeat the stream-rate configuration after restarting MAVROS or the flight controller.

---

# 10. ROS 2 Topics

### Camera Topics

| Topic | Data |
|---|---|
| `/camera/d455/color/image_raw` | RGB image |
| `/camera/d455/color/camera_info` | Camera calibration information |
| `/camera/d455/aligned_depth_to_color/image_raw` | Depth aligned to RGB |
| `/camera/d455/imu` | Combined IMU data |
| `/camera/d455/imu_filtered` | Filtered IMU data |

### Navigation Topics

| Topic | Purpose |
|---|---|
| `/vio/odometry` | Visual odometry output |
| `/mavros/local_position/pose` | FCU local position |
| `/mavros/state` | Flight controller connection and mode |
| `/mavros/odometry/out` | MAVROS odometry interface |

The actual topic connections should be checked against the running launch configuration.

---

# 11. Phase 2 — External Navigation

Phase 2 validates communication between the visual odometry system and the flight controller.

### Flight Controller Parameters

The project uses the following ExternalNav-related settings:

| Parameter | Configured Value |
|---|---|
| `AHRS_EKF_TYPE` | 3 |
| `EK3_ENABLE` | 1 |
| `EK3_SRC1_POSXY` | 6 |
| `EK3_SRC1_VELXY` | 6 |
| `EK3_SRC1_POSZ` | ExternalNav |
| `EK3_SRC1_VELZ` | 6 |
| `EK3_SRC1_YAW` | 6 |
| `VISO_TYPE` | 1 |
| `VISO_ORIENT` | Forward |
| `GPS1_TYPE` | None |

Verify the exact parameter values and ArduPilot parameter definitions for the installed firmware.

### Set GUIDED Mode

After verifying the navigation system and confirming the aircraft is in a safe test configuration:

```bash
ros2 service call /mavros/set_mode \
mavros_msgs/srv/SetMode \
"{base_mode: 0, custom_mode: 'GUIDED'}"
```

### Run Health Check

```bash
ros2 run irocu_vio_bridge phase2_health_check
```

Expected result:

```text
PHASE2_HEALTH_PASS
```

The health check is intended to validate VIO, local-position, MAVROS connection, and flight-mode conditions.

A health-check pass does not guarantee that the aircraft is safe to fly.

---

# 12. Phase 3 — Flight Control

The project contains several flight-control nodes.

## 12.1 Keyboard Control

Executable:

```bash
ros2 run irocu_vio_bridge phase3_keyboard_control
```

Configured command:

```bash
ros2 run irocu_vio_bridge phase3_keyboard_control --ros-args \
-p confirm_flight:=true \
-p target_altitude:=0.5 \
-p linear_speed:=0.08 \
-p vertical_speed:=0.05 \
-p yaw_rate:=0.15 \
-p max_altitude:=1.0 \
-p min_altitude:=0.30 \
-p hover_before_control_seconds:=5.0 \
-p setpoint_rate_hz:=20.0
```

### Keyboard Controls

| Key | Action |
|---|---|
| W | Forward |
| S | Backward |
| A | Left |
| D | Right |
| R | Increase altitude |
| F | Decrease altitude |
| Z | Yaw |
| X | Reverse yaw |
| SPACE | Stop movement |
| L | Land |
| Q | Land and exit |

The command above is a documented configuration. Inspect the node's safety checks and actual altitude handling before flight.

## 12.2 Velocity Test

Executable:

```bash
ros2 run irocu_vio_bridge phase3_velocity_test
```

Use the node's declared parameters and configuration before supplying flight setpoints.

## 12.3 Autonomous Square Path

```bash
ros2 run irocu_vio_bridge phase3_square_path --ros-args \
-p confirm_flight:=true \
-p target_altitude:=0.5 \
-p move_speed:=0.08 \
-p side_length:=0.4 \
-p max_segment_seconds:=8.0 \
-p hover_before_square_seconds:=5.0 \
-p hold_between_segments_seconds:=3.0 \
-p hover_after_square_seconds:=5.0 \
-p setpoint_rate_hz:=20.0
```

## 12.4 Square Path with Yaw Hold

```bash
ros2 run irocu_vio_bridge phase3_square_yaw_hold --ros-args \
-p confirm_flight:=true \
-p target_altitude:=0.5 \
-p move_speed:=0.08 \
-p side_length:=0.4 \
-p max_segment_seconds:=8.0 \
-p hover_before_square_seconds:=5.0 \
-p hold_between_segments_seconds:=3.0 \
-p hover_after_square_seconds:=5.0 \
-p yaw_hold_enabled:=true \
-p yaw_kp:=0.8 \
-p max_yaw_rate:=0.25 \
-p yaw_deadband:=0.03 \
-p yaw_control_sign:=1.0 \
-p setpoint_rate_hz:=20.0
```

Yaw correction direction depends on the coordinate-frame conventions and controller implementation. Verify it before flight.

## 12.5 Lawnmower Path

Executable:

```bash
ros2 run irocu_vio_bridge phase3_lawnmower_path
```

This node is intended for autonomous coverage-style movement.

Inspect its declared parameters before execution:

```bash
ros2 run irocu_vio_bridge phase3_lawnmower_path --ros-args --show-args
```

---

# 13. ArUco Precision Landing

The project includes the ArUco-enabled lawnmower node:

```text
phase3_lawnmower_aruco
```

Its intended mission sequence is:

1. Start the configured autonomous pattern.
2. Follow the configured lawnmower lanes.
3. Return toward the home position.
4. Search for the configured ArUco marker.
5. Detect the marker.
6. Align the drone with the landing reference.
7. Perform controlled descent.
8. Complete landing.

### Current Mission Configuration

The following records the project's configured 5 m × 5 m mission:

```bash
ros2 run irocu_vio_bridge phase3_lawnmower_aruco --ros-args \
-p confirm_flight:=true \
-p target_altitude:=2.0 \
-p max_length:=5.0 \
-p max_breadth:=5.0 \
-p lane_spacing:=1.0 \
-p boundary_margin:=0.20 \
-p geofence_tolerance:=0.20 \
-p geofence_recovery_tolerance:=0.60 \
-p min_segment_distance:=0.08 \
-p max_segment_seconds:=60.0 \
-p waypoint_kp_xy:=0.55 \
-p waypoint_accept_radius:=0.15 \
-p waypoint_speed_max:=0.12 \
-p hover_before_pattern_seconds:=8.0 \
-p hold_between_segments_seconds:=3.0 \
-p hover_after_pattern_seconds:=5.0 \
-p return_home_enabled:=true \
-p return_speed_max:=0.10 \
-p return_kp_xy:=0.50 \
-p return_kp_z:=0.40 \
-p return_accept_radius:=0.18 \
-p return_accept_z:=0.15 \
-p return_timeout_seconds:=60.0 \
-p yaw_hold_enabled:=true \
-p yaw_kp:=0.45 \
-p max_yaw_rate:=0.15 \
-p yaw_deadband:=0.04 \
-p yaw_control_sign:=1.0 \
-p precision_landing_enabled:=true \
-p aruco_marker_id:=4 \
-p aruco_search_timeout_seconds:=35.0 \
-p aruco_align_timeout_seconds:=90.0 \
-p aruco_lost_timeout_seconds:=1.0 \
-p aruco_accept_xy:=0.10 \
-p aruco_accept_hold_seconds:=2.0 \
-p aruco_precision_kp_x:=0.02 \
-p aruco_precision_kp_y:=0.02 \
-p aruco_precision_speed_max:=0.001 \
-p aruco_descent_speed:=0.006 \
-p aruco_final_land_altitude:=0.35 \
-p aruco_vx_sign:=-1.0 \
-p aruco_vy_sign:=-1.0 \
-p landing_yaw_offset_deg:=0.0 \
-p landing_yaw_accept_deg:=6.0 \
-p landing_yaw_hold_seconds:=2.0 \
-p setpoint_rate_hz:=20.0
```

**Important:** These values document a project configuration, not a validated flight envelope. Verify every parameter against the source code and test the marker detection and alignment behavior independently before attempting the full mission.

---

# 14. Configuration

The main configuration areas are:

| Area | Important Settings |
|---|---|
| Camera | RGB/depth resolution, FPS, IMU |
| VIO | Odometry source, frames, filtering |
| MAVROS | Serial connection, stream rates, odometry frames |
| ArduPilot | EKF3, ExternalNav, flight modes |
| Keyboard control | Movement speed, altitude, yaw |
| Autonomous paths | Length, speed, waypoint tolerance |
| ArUco landing | Marker ID, alignment thresholds, descent speed |

### Important Coordinate Frames

The current runtime setup uses:

```text
odom
  |
  +--- odom_ned

d455_link
  |
  +--- d455_link_frd
```

These transforms are part of the configured frame-conversion setup. Confirm their rotations and physical mounting assumptions before flight.

---

# 15. Monitoring and Debugging

### List all ROS topics

```bash
ros2 topic list
```

### Check camera topics

```bash
ros2 topic list | grep camera
```

### Check VIO odometry

```bash
ros2 topic echo /vio/odometry
```

### Check VIO frequency

```bash
ros2 topic hz /vio/odometry
```

### Check flight-controller state

```bash
ros2 topic echo /mavros/state
```

### Check local position

```bash
ros2 topic echo /mavros/local_position/pose
```

### List active nodes

```bash
ros2 node list
```

### Inspect node parameters

```bash
ros2 param list
```

### Inspect a specific node's parameters

```bash
ros2 param list /mavros/odometry
```

### Inspect transforms

```bash
ros2 run tf2_tools view_frames
```

### Check serial devices

```bash
ls -l /dev/ttyACM*
```

---

# 16. Troubleshooting

| Problem | Possible Check |
|---|---|
| Camera not detected | Check USB 3 connection and RealSense device detection |
| Camera topics missing | Check camera launch terminal and ROS topic list |
| VIO not publishing | Check camera inputs and VIO launch logs |
| MAVROS disconnected | Check serial device and baud rate |
| Local position missing | Check ExternalNav data and MAVROS stream rates |
| TF warning | Check required static transforms and frame IDs |
| Health check fails | Check VIO freshness, FCU connection, local position and mode |
| ArUco marker not detected | Check camera stream, marker ID, lighting and detector configuration |
| Flight controller rejects ExternalNav | Check EKF3 and VISO parameters, frames and incoming odometry |

### Rebuild After Code Changes

```bash
cd ~/isro_final

source /opt/ros/jazzy/setup.bash

colcon build --packages-select irocu_vio_bridge --symlink-install

source install/setup.bash
```

---

# 17. Safety Considerations

This project interfaces with a real aircraft. Incorrect setpoints, coordinate-frame errors, stale odometry, or incorrect controller parameters can cause unexpected movement.

Before any powered flight:

- Inspect the drone mechanically.
- Verify propeller direction and motor configuration.
- Verify the D455 mounting orientation.
- Validate visual odometry while stationary and during controlled movement.
- Confirm the odometry coordinate frames.
- Confirm ExternalNav is accepted by ArduPilot.
- Verify the drone's movement direction against each commanded direction.
- Verify yaw correction direction.
- Validate altitude limits and geofence behavior.
- Verify return-home and landing logic.
- Keep a working RC transmitter and manual recovery procedure available.
- Test in a controlled, clear area with appropriate precautions.

**Never assume that successful ROS startup or a passing health check proves flight safety.**

### LAND Mode Request

If MAVROS is functioning, the following requests LAND mode:

```bash
ros2 service call /mavros/set_mode \
mavros_msgs/srv/SetMode \
"{base_mode: 0, custom_mode: 'LAND'}"
```

Disarming while airborne can cause a crash. Do not use an emergency disarm command as a substitute for a controlled landing.

---

# 18. Development Roadmap

- [x] ROS 2 Jazzy workspace
- [x] RealSense D455 integration
- [x] RGB-D and IMU pipeline
- [x] Visual odometry integration
- [x] MAVROS communication
- [x] ExternalNav configuration workflow
- [x] Phase 2 health-check node
- [x] Keyboard-control implementation
- [x] Autonomous path-control nodes
- [x] ArUco landing implementation
- [ ] Validate all autonomous movement directions
- [ ] Validate yaw-hold behavior
- [ ] Validate precision landing in controlled conditions
- [ ] Validate complete autonomous mission and recovery sequence

---

# 19. Contributing

Contributions and improvements are welcome.

Suggested contribution workflow:

1. Fork the repository.
2. Create a feature branch.
3. Make the required changes.
4. Build the ROS 2 workspace.
5. Test changes in simulation or a safe controlled environment.
6. Submit a pull request with a description of the changes.

For flight-control changes, include relevant test results and configuration details.

---

# 20. License

A license has not yet been specified for this repository.

Until a license is added, do not assume that the source code is available for unrestricted reuse, redistribution, or modification.

---

# 21. Repository

**GitHub:** https://github.com/Rishiwant1729/ISRO

**Primary ROS 2 package:** `irocu_vio_bridge`

**Workspace:** `~/isro_final`

---

Developed as part of the ISRO autonomous drone project.
