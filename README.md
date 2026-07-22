# `climbingrobot_hardware_interface`

ROS 1 Noetic hardware interface for the real ALPINE climbing robot.

The package provides:

- serial communication with the left and right winch controllers;
- winch telemetry, brake control, rope zeroing, and torque/velocity/position commands;
- the ESP32 body dongle bridge for IMU telemetry, pneumatic valves, and propeller commands;
- manual and planner-driven jump interfaces;
- homing, odometry, logging, and experimental test utilities.

The low-level package owns hardware communication and actuator interfaces. The high-level `locosim` controller owns model state, sequence logic, target generation, and planner integration. The onboard body firmware owns the embedded attitude controller.

## Runtime architecture

| Component | Nominal rate | Responsibility |
|---|---:|---|
| Left/right winch telemetry nodes | 200 Hz | Serial telemetry, rope conversion, modes, brakes, and commands |
| Rope-position outer loop | 50 Hz | Rope-length position tracking through ODrive velocity mode |
| Body ESP32 / ESP-NOW path | 100 Hz | Dual-IMU telemetry and bidirectional body commands |
| Manual jump state machine | 100 Hz | Valve sequence, rope-force handover, and final hold |
| Embedded attitude controller | 100 Hz | Pitch/yaw stabilization on the body ESP32 |
| ESC pulse generation | 50 Hz | EDF command output |

## Build

From the ROS workspace containing `locosim`:

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

Verify that ROS can find the package:

```bash
rospack find climbingrobot_hardware_interface
```

Use persistent serial paths whenever possible:

```bash
ls -l /dev/serial/by-id/
```

Do not hard-code `/dev/ttyUSB0`, because the device number can change after reconnecting a USB cable.

## Low-level bring-up

Start the two winches, body dongle bridge, jump node, and odometry bridge:

```bash
roslaunch climbingrobot_hardware_interface alpine_low_level_bringup.launch
```

The real `locosim` controller expects the hardware interface to be started externally. Do not launch `alpine_low_level_bringup.launch` a second time from the high-level controller.

Useful checks after bring-up:

```bash
rosnode list
rostopic list | grep -E 'winch|alpine_body|odom'
rosservice list | grep -E 'winch|alpine'
```

Before commanding the robot, confirm that both winch telemetry streams and the body telemetry stream are updating.

## Safety-first startup

The robot is suspended and contains high-power winches, pneumatic actuation, and six EDFs. Keep the work area clear and retain immediate access to power isolation and emergency stop actions.

Start body actuation from a disabled state unless a tested launch configuration intentionally enables it:

```bash
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'attoff'"
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'proff'"
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String \
  "data: 'THR,0,0,0,0,0,0'"
```

Do not call `/alpine/jump` until:

- both winches publish fresh telemetry;
- the body bridge publishes fresh telemetry;
- brake and rope directions have been checked;
- the suspended area is clear;
- the current jump parameters have been reviewed.

## Winch interface

Each `telemetry_node.py` instance is configured with `~side:=left` or `~side:=right` and exposes the same interface under `/winch/<side>`.

### Main parameters

| Parameter | Typical default | Description |
|---|---:|---|
| `~side` | `left` | Selects the `left` or `right` namespace |
| `~serial_port` | `/dev/ttyUSB0` | Winch-controller serial device |
| `~baud` | `1000000` | Serial baud rate |
| `~poll_rate_hz` | `200.0` | Telemetry polling rate |
| `~config_path` | package JSON file | Telemetry fields requested from the MCU |
| `~debug_mode` | `false` | Enables direct terminal command injection |
| `~rope_position_outer_loop_enabled` | `true` | Uses the ROS rope-length outer loop |

Manual start example:

```bash
rosrun climbingrobot_hardware_interface telemetry_node.py \
  _side:=left \
  _serial_port:=/dev/serial/by-id/<LEFT_WINCH_ID> \
  _config_path:=$(rospack find climbingrobot_hardware_interface)/config/arganelloTelemetry.json
```

Run a second instance with `side:=right` and the right winch serial ID.

### Topics

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Published | `/winch/<side>/telemetry` | `RopeTelemetry` | Filtered rope, motor, force, and brake telemetry |
| Published | `/winch/<side>/telemetry/csv` | `std_msgs/String` | CSV telemetry stream |
| Published | `/winch/<side>/telemetry/debug` | `DebugMessage` | Debug values |
| Subscribed | `/winch/<side>/command` | `RopeCommand` | Rope force, velocity, or position reference |
| Subscribed | `/winch/<side>/set_motor_mode` | `std_msgs/String` | Low-level mode selection |

### Services

| Service | Type | Purpose |
|---|---|---|
| `/winch/<side>/set_control_mode` | `RopeControlMode` | Mode interface used by the real `locosim` controller |
| `/winch/<side>/brake_engage` | `std_srvs/Trigger` | Engage the mechanical brake |
| `/winch/<side>/brake_disengage` | `std_srvs/Trigger` | Release the mechanical brake |
| `/winch/<side>/rope_zero` | `std_srvs/Trigger` | Set the current roller and motor references as rope zero |
| `/winch/<side>/sync_now` | `std_srvs/Trigger` | Synchronize the MCU timestamp |

### Control modes

Supported mode names are:

- `idle`
- `closed_loop_torque`
- `closed_loop_velocity`
- `closed_loop_position`

Historical `close_loop_*` aliases are accepted, but new code should use `closed_loop_*`.

The service interface is preferred because it is the interface used by `climbingrobot_controller2_real.py`:

```bash
rosservice call /winch/left/set_control_mode \
  "message: 'closed_loop_torque'"

rosservice call /winch/right/set_control_mode \
  "message: 'closed_loop_torque'"
```

The topic interface remains available for direct testing:

```bash
rostopic pub -1 /winch/left/set_motor_mode std_msgs/String \
  "data: 'closed_loop_position'"
```

When the ROS position outer loop is enabled, entering `closed_loop_position` first holds the currently measured rope length. A subsequent finite `rope_position` field updates the reference. The outer loop runs at 50 Hz and commands ODrive velocity mode internally.

### `RopeCommand` convention

`telemetry_node.py` acts on every **finite** field in a `RopeCommand`. Unused fields must therefore be `NaN`, not zero.

Torque-only command:

```bash
rostopic pub -1 /winch/left/command \
  climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: -20.0, rope_velocity: .nan, rope_position: .nan}"
```

Velocity-only command:

```bash
rostopic pub -1 /winch/left/command \
  climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: .nan, rope_velocity: 0.02, rope_position: .nan}"
```

Position-only command:

```bash
rostopic pub -1 /winch/left/command \
  climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: .nan, rope_velocity: .nan, rope_position: 0.30}"
```

For sustained manual tests, publish at an appropriate refresh rate only after checking the selected mode and sign convention:

```bash
rostopic pub -r 50 /winch/left/command \
  climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: -20.0, rope_velocity: .nan, rope_position: .nan}"
```

The force field is a rope-force-equivalent command. The node converts it to motor torque using the current transmission ratio, effective roller radius, and side-dependent direction.

### Brake, zero, and synchronization

```bash
rosservice call /winch/left/brake_engage "{}"
rosservice call /winch/left/brake_disengage "{}"
rosservice call /winch/right/brake_engage "{}"
rosservice call /winch/right/brake_disengage "{}"

rosservice call /winch/left/rope_zero "{}"
rosservice call /winch/right/rope_zero "{}"

rosservice call /winch/left/sync_now "{}"
rosservice call /winch/right/sync_now "{}"
```

Monitor the main output:

```bash
rostopic echo /winch/left/telemetry
rostopic echo /winch/right/telemetry
```

## Homing and rope coordinates

The current real pipeline is:

```text
bring-up -> homing -> pre-jump positioning -> manual jump -> post-jump hold
```

The effective homing sequence is implemented in:

```text
scripts/homing_procedure.py
```

This script is the source of truth for the executed force profile and timings. Parameters with similar `homing_*` names elsewhere do not change the procedure unless `homing_procedure.py` explicitly reads them.

At the end of homing, `/winch/<side>/rope_zero` makes the telemetry-node rope coordinate homing-relative:

```text
left raw rope_length  = 0 m
right raw rope_length = 0 m
```

The high-level `locosim` controller separately applies model offsets and signs for odometry and RViz. These model signs are not the same as low-level position-command signs.

For the currently tested pre-jump descent, the high-level controller adds the same positive raw increment to both telemetry-node coordinates. Do not apply the right-side model sign directly to `/winch/right/command`.

Standalone homing can be run for commissioning, but the real `locosim` controller already instantiates `WinchStartupSequence` during its integrated startup. Do not execute two homing procedures concurrently.

## ALPINE body dongle bridge

`dongle_node.py` bridges the USB-connected ESP32 dongle to the onboard ALPINE body firmware.

Manual start example:

```bash
rosrun climbingrobot_hardware_interface dongle_node.py \
  _serial_port:=/dev/serial/by-id/<BODY_DONGLE_ID> \
  _baud:=115200 \
  _poll_rate:=100.0
```

### Published topics

| Topic | Type | Content |
|---|---|---|
| `/alpine_body/telemetry/raw` | `std_msgs/String` | Unmodified lines received from the body firmware |
| `/alpine_body/telemetry_array` | `std_msgs/Float32MultiArray` | `[epoch_ms, imu1[0..10], imu2[0..10]]` |

The odometry bridge converts the parsed body stream into the custom `/alpine_body/telemetry` message used by the high-level controller.

### Subscribed topics

| Topic | Type | Firmware command |
|---|---|---|
| `/alpine_body/motorSpeed` | `std_msgs/Float32` | `m<value>` |
| `/alpine_body/servoValve1` | `std_msgs/Float32` | `s1 <deg>` |
| `/alpine_body/servoValve2` | `std_msgs/Float32` | `s2 <deg>` |
| `/alpine_body/cmd_raw` | `std_msgs/String` | Raw firmware command |
| `/alpine_body/wrench_cmd` | `geometry_msgs/Wrench` | `WRC,fx,fy,mz` |
| `/alpine_body/propeller_command` | `PropellerCommand` | `PROP_COMMAND,p0,p1,p2,p3` |

Examples:

```bash
# Firmware status
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String \
  "data: 'status'"

# Direct valve tests
rostopic pub -1 /alpine_body/servoValve1 std_msgs/Float32 "data: 30.0"
rostopic pub -1 /alpine_body/servoValve2 std_msgs/Float32 "data: 30.0"

# High-level body wrench command
rostopic pub -1 /alpine_body/wrench_cmd geometry_msgs/Wrench \
  "{force: {x: 0.10, y: 0.00, z: 0.0}, \
    torque: {x: 0.0, y: 0.0, z: 0.10}}"

# Reset the body wrench
rostopic pub -1 /alpine_body/wrench_cmd geometry_msgs/Wrench \
  "{force: {x: 0.0, y: 0.0, z: 0.0}, \
    torque: {x: 0.0, y: 0.0, z: 0.0}}"
```

The body firmware owns the embedded pitch/yaw controller and actuator allocation. Host-side wrench commands are forwarded to the firmware; they are not direct per-thruster commands.

## Propeller and attitude commissioning

Use a restrained bench-test order:

1. disable attitude hold and open-loop bias;
2. verify T1 through T6 one at a time at a low command;
3. verify positive and negative pitch action;
4. enable pitch hold;
5. enable yaw hold;
6. enable full attitude hold;
7. test `wrench_cmd`;
8. test a jump only after all previous checks pass.

Thruster numbering used by the current firmware:

| Thruster | Physical role |
|---|---|
| `T1` | front-left yaw EDF |
| `T2` | front-right yaw EDF |
| `T3` | rear-right yaw EDF |
| `T4` | rear-left yaw EDF |
| `T5` | upper pitch EDF |
| `T6` | lower pitch EDF |

Safe baseline:

```bash
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'attoff'"
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'proff'"
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String \
  "data: 'THR,0,0,0,0,0,0'"
```

Example low-command T1 test:

```bash
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String \
  "data: 'THR,0.12,0,0,0,0,0'"

rostopic pub -1 /alpine_body/cmd_raw std_msgs/String \
  "data: 'THR,0,0,0,0,0,0'"
```

Move the non-zero value to the desired channel to test T2 through T6. Stop all EDFs after every individual test.

Capture the current attitude and enable embedded hold:

```bash
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'attzero'"
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'atton'"
```

Disable it with:

```bash
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'attoff'"
```

Exact gains, deadbands, active floors, saturation limits, and calibrated thrust mappings belong to the firmware and launch configuration. Check the active configuration before testing and increase commands only after safe bench validation.

## Jump interface

`jump_node.py` intentionally keeps two command paths separate.

### Manual standalone jump

Service:

```text
/alpine/jump    std_srvs/Trigger
```

The manual path controls:

- both pneumatic valves;
- local left/right rope forces;
- the position-to-torque handover;
- force ramping and post-jump hold.

The node uses recent rope-force feedback as the initial preload when available, reducing the step during the transfer from position hold to torque control.

Commands:

```bash
# Optional: store the current rope coordinates as the return reference
rosservice call /alpine/capture_takeoff "{}"

# Start the configured manual jump
rosservice call /alpine/jump "{}"

# Abort the active sequence, close the valves, and apply the configured light hold
rosservice call /alpine/jump_abort "{}"

# Stop and clear the active sequence/holds
rosservice call /alpine/jump_stop "{}"

# Return gradually to the captured take-off rope coordinates
rosservice call /alpine/return_to_takeoff "{}"
```

### Planner-driven body command

Service:

```text
/alpine_body/command    climbingrobot_hardware_interface/AlpineBodyCommand
```

This is the optimized path used by the high-level controller. It maps the requested leg action to a pneumatic valve sequence but does **not** publish rope commands. Planned left and right rope-force trajectories remain the responsibility of the high-level controller.

Additional experimental inputs:

| Topic | Type | Purpose |
|---|---|---|
| `/alpine_body/calib_inlet_deg` | `std_msgs/Float32` | Direct inlet-valve calibration, valves only |
| `/alpine_body/jump_impulse_des` | `std_msgs/Float32` | Desired impulse mapped through the experimental lookup |

## Odometry and RViz

The odometry bridge reconstructs the body state from rope coordinates and body IMU data. Typical outputs are:

```text
/odom                         nav_msgs/Odometry
/alpine/odometry/pose         geometry_msgs/PoseStamped
/alpine/odometry/debug        std_msgs/Float32MultiArray
/alpine_body/telemetry        climbingrobot_hardware_interface/AlpineBodyTelemetry
```

Check the actual running graph rather than assuming that every optional topic is enabled:

```bash
rostopic list | grep -E 'odom|odometry|alpine_body/telemetry'
rostopic hz /winch/left/telemetry
rostopic hz /winch/right/telemetry
rostopic hz /alpine_body/telemetry
```

RViz visualization in the real controller is read-only: it consumes telemetry and publishes joint states, markers, and transforms. It must not command winches, change modes, or trigger a jump.

## Experimental utilities

### PlotJuggler

```bash
rosrun plotjuggler PlotJuggler
```

### Friction estimator

```bash
rosrun climbingrobot_hardware_interface friction_estimator.py _side:=left
```

### Position-control logger

```bash
rosrun climbingrobot_hardware_interface position_control_logger.py \
  _side:=left \
  _output_csv:=/tmp/position_control_log.csv
```

### Position step test

```bash
rosrun climbingrobot_hardware_interface position_step_test.py \
  _side:=left \
  _step_m:=0.30 \
  _hold_0_s:=2.0 \
  _hold_step_s:=18.0 \
  _hold_back_s:=18.0
```

Run automated tests only with the suspended area clear and with the rope sign, travel limit, and brake behavior already verified manually.

## Troubleshooting

### No serial device

```bash
lsusb
ls -l /dev/serial/by-id/
dmesg | tail -n 50
```

Confirm permissions for the serial group and reconnect the device after adding the user:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in before retrying.

### Stale or duplicate nodes

```bash
rosnode list
rosnode kill <duplicate_node>
```

Do not run a manual `telemetry_node.py` instance while the bring-up launch already owns the same serial port.

### Unexpected winch motion

Immediately stop publishing commands, restore a safe mode, and check:

- selected control mode;
- finite versus `NaN` fields in `RopeCommand`;
- left/right command signs;
- whether another node is publishing to the same command topic;
- whether the position reference is a raw homing-relative coordinate or a model-space coordinate.

Find command publishers with:

```bash
rostopic info /winch/left/command
rostopic info /winch/right/command
```

### Body commands have no effect

```bash
rostopic echo /alpine_body/telemetry/raw
rostopic info /alpine_body/cmd_raw
rostopic pub -1 /alpine_body/cmd_raw std_msgs/String "data: 'status'"
```

Confirm the configured baud rate, persistent serial path, firmware runtime mode, and ESP-NOW link.

## Source of truth

Before changing parameters or interfaces, inspect the implementation currently checked out in the repository:

```text
launch/alpine_low_level_bringup.launch
scripts/telemetry_node.py
scripts/dongle_node.py
scripts/jump_node.py
scripts/homing_procedure.py
scripts/alpine_odometry_node.py
config/arganelloTelemetry.json
```

Avoid copying old ROS 2 commands, obsolete `/alpine/dongle/*` namespaces, or the legacy `arganello_node.py` interface into new procedures.
