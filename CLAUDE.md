# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 (Humble) autonomous RTK-guided cleaning robot. The robot navigates via GPS waypoints using a Stanley controller, communicates with motors over CAN bus, and reports status/accepts commands via MQTT.

## Build & Run

```bash
# Build (from repo root)
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs motor_control rtk_nav mqtt_ros2

# Source the workspace
source install/setup.bash

# Launch everything (sensors, motors, RTK nav, MQTT bridge)
ros2 launch rtk_nav run.launch.py robot_ID:=<robot_id>

# Run a single node
ros2 run motor_control motor_control
ros2 run rtk_nav rtk_nav
```

## Testing

```bash
# Run test suites (pytest-based)
colcon test --packages-select motor_control rtk_nav mqtt_ros2
colcon test-result --verbose

# Real-time navigation test: comment out `wtrtk_parse_txt` in the launch file,
# uncomment `wtrtk_serial_driver`, and launch.
#
# Bag playback test: uncomment `wtrtk_parse_txt` with a recorded data file.
```

## Package Architecture

### `custom_msgs` (ament_cmake)
Custom ROS2 interfaces defined as `.msg`/`.srv` files:
- `WTRTK.msg` — RTK sensor data (position, heading, fix status, INS values)
- `LatLonPoint.msg` — Longitude/latitude pair
- `ChargeControl.srv` — Start/stop wireless charging

This package must be built first; other packages depend on it.

### `motor_control` (ament_python)
The central hardware-interface package. Contains these executables (from `setup.py` entry points):

| Executable | Source | Role |
|---|---|---|
| `motor_control` | `motor_control.py` | **Main orchestration node** — state machine, mode switching, all subsystem coordination |
| `motor_driver` | `motor_driver.py` | CAN bus motor driver (socketcan/can0, 1Mbps, 3 motors: left wheel, right wheel, brush) |
| `remote_control` | `remote_control.py` | SBUS remote controller parser (serial, /dev/remote_control) |
| `sensors_485` | `sensors_485.py` | 485 IO sensor polling (8-channel Modbus, publishes `/io_data`) |
| `laser_distance` | `laser_distance.py` | Dual-laser distance sensor reading (Modbus, publishes `/laser_distance`) |
| `charging` | `charging.py` | 485 wireless charging control + BMS battery monitoring (shared bus with baudrate switching, services: `/start_charging`, `/stop_charging`, `/query_volt_curr`, `/query_fault_code`) |

The main `motor_control` node (`MotorControlNode` in `motor_control.py`) is the state machine hub. It:
- Manages mode transitions (NORMAL / REMOTE / AUTO_CLEANING / HOLD / DISABLE)
- Subscribes to `/keyboard/control` (String), `/rtk_speed` (Vector3), `/fix` (NavSatFix), `/wtrtk_data` (WTRTK), `/io_data`, `/battery_data`, `/laser_distance`, `/charging_volt_curr`, `/charging_fault_code`, `/odom`
- Publishes motor speed commands to `motor_driver` via `motor_speed_commands` (Float32MultiArray) and `motor_velocities`
- Implements loading/unloading (docking) state machines with laser-guided final alignment

### `rtk_nav` (ament_python)
RTK navigation and path planning. Executables:

| Executable | Source | Role |
|---|---|---|
| `rtk_nav` | `rtk_nav.py` | **Stanley-controller waypoint navigator** — reads waypoint files, state machine (IDLE → INITIAL_MOVE → WAYPOINT_MOVE → WAYPOINT_CALIB → COMPLETED), publishes motor speed on `/rtk_speed` |
| `wtrtk_serial_driver` | `wtrtk_serial_driver.py` | Real-time RTK GPS serial reader (publishes `/fix` and `/wtrtk_data`) |
| `wtrtk_parse_txt` | `wtrtk_parse_txt.py` | Offline RTK data replay from text files (for bag-playback testing) |
| `cleaning_path_planner` | `cleaning_path_planner.py` | Single-area cleaning path generation |
| `full_path_planner` | `full_path_planner.py` | Multi-area path concatenation |
| `three_point_planner` | `three_point_planner.py` | Three-point calibration path generation |
| `navsat_key_publisher` | `navsat_key_publisher.py` | NavSatFix helper publisher |

Key navigation parameters (in `rtk_nav.py` module-level constants):
- `STANLEY_K = 2.0` — Stanley gain
- `RTK_WAYPOINT_TOLERANCE = 0.15m` — waypoint arrival threshold
- `LINEAR_SPEED_BASE = 10.0` — base driving speed (~0.4 m/s)
- `RTK_HEADING_TOLERANCE = 1.0°` — heading calibration precision
- `LOW_DISTANCE = 1.5m` — slow-down trigger distance
- `MAX_CORRECTION = 2.0` — max differential correction

### `mqtt_ros2` (ament_python)
Single executable (`mqtt_ros2_bridge`) — MQTT ↔ ROS2 bridge using paho-mqtt:
- Publishes robot status and dock status to MQTT topics
- Subscribes to MQTT command topics and forwards to ROS2 `/keyboard/control`
- **Security note**: `topic_command` handler executes remote shell commands via `subprocess.Popen(shell=True)` — a P0 risk per REVIEW.md

## Mode Switching (during runtime)

```bash
ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'r'}" -1   # RTK auto-nav mode
ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'n'}" -1   # Normal (keyboard) mode
ros2 topic pub /keyboard/control std_msgs/msg/String "{data: 'm'}" -1   # Remote control mode
```

Keyboard controls in NORMAL mode: `w`/`s`/`a`/`d` for movement, `z` for stop, `x` for enable.

## Key ROS Topics

| Topic | Type | Description |
|---|---|---|
| `/fix` | NavSatFix | GPS position from RTK |
| `/wtrtk_data` | WTRTK | Full RTK sensor data |
| `/rtk_speed` | Vector3 | Desired L/R wheel speed from navigator |
| `/motor_speed_commands` | Float32MultiArray | Speed commands to motor driver |
| `/motor_feedback` | Float32MultiArray | Motor position/velocity/torque/temp |
| `/keyboard/control` | String | Mode and movement commands |
| `/io_data` | UInt8 | 8-channel IO sensor bitmap |
| `/battery_data` | Float32MultiArray | Battery capacity/current/voltage/temp |
| `/laser_distance` | UInt16MultiArray | Dual laser distances |
| `/charging_fault_code` | Int16 | Wireless charging fault code |
| `/odom` | Odometry | Wheel odometry |

## Hardware Interface

- **Motors**: 3 CAN bus motors (socketcan/can0, 1Mbps): left wheel (ID=1), right wheel (ID=2), brush (ID=3)
- **RTK GPS**: Serial at 230400 baud, publishes GNGGA + proprietary WTRTK frames
- **Remote control**: SBUS protocol over serial at 115200 baud, 16 channels
- **Laser distance**: 2-channel Modbus RTU over serial
- **IO sensors**: 8-channel 485 Modbus RTU
- **Charging + BMS**: Shared 485 bus with baudrate switching (19200/9600)

## Known Issues (from REVIEW.md)

1. **P0**: MQTT bridge executes remote shell commands — production should disable or add authentication/whitelisting
2. **P1**: Device paths (`/dev/ttyS4`, `/dev/laser`, `/dev/WTRTK`) and file paths (`/home/ztl/...`) are hardcoded in launch files
3. **P1**: Configuration mixed with business logic — device paths, MQTT credentials, behavioral parameters live directly in code/launch
4. **P2**: Field-customized values (robot ID, MQTT broker IP/credentials) are committed rather than parameterized
