🚀 Arganello ROS 1 Interface - Complete Command Reference
This document provides an updated list of all available features and commands for interacting with Arganello motor controllers via ROS 1 Noetic.
> ⚠️ This package has been ported from ROS 2 (rclpy) to ROS 1 (rospy).
> All nodes are plain Python classes using `rospy`. Parameters use the `~` private namespace prefix.
> Build with `catkin_make` and source `devel/setup.bash` before running any node.
---
Build & Setup
```bash
cd ~/Desktop/ros1_ws
catkin_make
source devel/setup.bash
```
---
Launch Files
Full System Bringup (both winches + dongle + jump node)
```bash
roslaunch climbingrobot_hardware_interface alpine_low_level_bringup.launch
```
Starts:
`telemetry_node_left`  — left winch at 200 Hz
`telemetry_node_right` — right winch at 200 Hz
`dongle_node`          — ESP32 dongle bridge at 100 Hz
`jump_node`            — SAFE, sends no commands until /alpine/jump is called
`alpine_odometry_node` — body telemetry + odometry bridge
Homing (bringup + homing procedure with 5 s delay)
```bash
roslaunch climbingrobot_hardware_interface homing.launch
```
Includes `alpine_low_level_bringup.launch`, then starts `homing_procedure` after a 5 s delay
to allow services and topics to come up.
Legacy Interface Launch (arganello_node sx/dx)
```bash
roslaunch climbingrobot_hardware_interface interface_launch.launch
```
Starts the legacy `arganello_node.py` for both sx and dx at 200 Hz.
---
Telemetry Node
Interfaces with both the left and right winch firmware via the `side` parameter.
Sends a list of required telemetry values at startup for high-rate streaming without per-request overhead.
Runs at 200 Hz.
Start node for left and right:
We specify the package and node as usual, then pass private parameters:
`side`        — automatically assigns the correct namespace (`left` / `right`)
`serial_port` — serial ID of the MCU; auto-reconnects even if the USB port changes
`config_path` — injects a list of high-rate telemetry requests to the winch MCU (ODrive fields, no MCU code changes needed)
`debug_mode`  — enables direct command injection to the winch MCU for testing
```bash
# Left winch
rosrun climbingrobot_hardware_interface telemetry_node.py \
  _side:=left \
  _serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970047399-if00 \
  _config_path:=/home/msi/Desktop/ros1_ws/src/climbingrobot_hardware_interface/config/arganelloTelemetry.json \
  _debug_mode:=true

# Right winch
rosrun climbingrobot_hardware_interface telemetry_node.py \
  _side:=right \
  _serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970046081-if00 \
  _config_path:=/home/msi/Desktop/ros1_ws/src/climbingrobot_hardware_interface/config/arganelloTelemetry.json \
  _debug_mode:=true
```
Motor Control Mode
Select which control mode (torque / velocity / position) the closed loop will use:
```bash
# Left winch
rostopic pub -1 /winch/left/set_motor_mode std_msgs/String "data: 'idle'"
rostopic pub -1 /winch/left/set_motor_mode std_msgs/String "data: 'closed_loop_torque'"
rostopic pub -1 /winch/left/set_motor_mode std_msgs/String "data: 'closed_loop_velocity'"
rostopic pub -1 /winch/left/set_motor_mode std_msgs/String "data: 'closed_loop_position'"

# Right winch
rostopic pub -1 /winch/right/set_motor_mode std_msgs/String "data: 'idle'"
rostopic pub -1 /winch/right/set_motor_mode std_msgs/String "data: 'closed_loop_torque'"
rostopic pub -1 /winch/right/set_motor_mode std_msgs/String "data: 'closed_loop_velocity'"
rostopic pub -1 /winch/right/set_motor_mode std_msgs/String "data: 'closed_loop_position'"
```
Brake Control
Engage and disengage the motor brake:
```bash
rosservice call /winch/left/brake_engage "{}"
rosservice call /winch/left/brake_disengage "{}"

rosservice call /winch/right/brake_engage "{}"
rosservice call /winch/right/brake_disengage "{}"
```
Rope Control
Given the selected motor mode, the node discards the irrelevant fields:
`closed_loop_torque`   -> uses `rope_force`
`closed_loop_velocoty` -> uses `rope_velocity`
`closed_loop_position` -> uses `rope_position`
```bash
# Continuous publish at 100 Hz
rostopic pub -r 100 /winch/left/command climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"

rostopic pub -r 100 /winch/right/command climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"

# Single publish
rostopic pub -1 /winch/left/command climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"

rostopic pub -1 /winch/right/command climbingrobot_hardware_interface/RopeCommand \
  "{rope_force: 10.0, rope_velocity: 0.0, rope_position: 0.0}"
```
Rope Zero & Sync
```bash
rosservice call /winch/left/rope_zero "{}"
rosservice call /winch/right/rope_zero "{}"

rosservice call /winch/left/sync_now "{}"
rosservice call /winch/right/sync_now "{}"
```
Output Topics
```bash
rostopic echo /winch/left/telemetry/debug
rostopic echo /winch/left/telemetry/csv
rostopic echo /winch/left/telemetry
```
Debug with PlotJuggler
```bash
rosrun plotjuggler PlotJuggler
```
---
Friction Estimator
ROS 1 node used to experimentally estimate the friction of the winches.
```bash
rosrun climbingrobot_hardware_interface friction_estimator.py _side:=left
```
---
Dongle Node
The dongle_node bridges the USB serial interface of the dongle ESP32 to ROS 1 topics.
The dongle forwards commands to the onboard Alpine body microcontroller and relays telemetry back.
This node wraps the low-level serial protocol into ROS 1 topics.
Features
Opens a serial connection to the dongle ESP32 (`serial_port`, `baud`).
Converts ROS 1 topics into serial commands:
`/alpine/dongle/motorSpeed`  -> `m<val>`
`/alpine/dongle/servoValve1` -> `s1 <deg>`
`/alpine/dongle/servoValve2` -> `s2 <deg>`
Reads serial data, reconstructs complete CSV lines, and republishes:
`/alpine/dongle/telemetry/raw` (std_msgs/String): unmodified CSV strings.
`/alpine/dongle/telemetry` (std_msgs/Float32MultiArray): parsed structure [epoch_ms, imu1[11], imu2[11]].
Parameters
Name	Type	Default	Description
serial_port	string	/dev/ttyUSB0	Serial device path for the dongle ESP32.
baud	int	1000000	Serial baudrate.
poll_rate	float	200.0	Polling frequency in Hz.
Topics
Publishers
/alpine/dongle/telemetry/raw  (std_msgs/String)            — Raw CSV lines from the dongle.
/alpine/dongle/telemetry      (std_msgs/Float32MultiArray) — Parsed structured telemetry: [epoch_ms, imu1[0..10], imu2[0..10]].
Subscribers
/alpine/dongle/motorSpeed  (std_msgs/Float32) -> m<val>
/alpine/dongle/servoValve1 (std_msgs/Float32) -> s1 <deg>
/alpine/dongle/servoValve2 (std_msgs/Float32) -> s2 <deg>
Example Usage
Start the node with a persistent USB device path:
```bash
rosrun climbingrobot_hardware_interface dongle_node.py \
  _serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A010904-if00 \
  _baud:=1000000 \
  _poll_rate:=200.0
```
In this configuration it can send and receive commands at up to 100 Hz.
Publish commands:
```bash
# Set motor speed
rostopic pub -1 /alpine/dongle/motorSpeed std_msgs/Float32 "data: 0.3"

# Move servo valve 1
rostopic pub -1 /alpine/dongle/servoValve1 std_msgs/Float32 "data: 45.0"

# Move servo valve 2
rostopic pub -1 /alpine/dongle/servoValve2 std_msgs/Float32 "data: 30.0"
```
Listen to telemetry:
```bash
rostopic echo /alpine/dongle/telemetry/raw
rostopic echo /alpine/dongle/telemetry
```
---
Jump Node
A small demonstration of jumping for events — an open-loop pre-programmed sequence of servo valve
positions for a simple jump and landing.
```bash
rosrun climbingrobot_hardware_interface jump_node.py
```
Trigger the jump:
```bash
rosservice call /alpine/jump "{}"
```
Abort/stop the jump:
``bash
rosservice call /alpine/jump_abort "{}"
rosservice call /alpine/jump_stop "{}"
```

---
Position Control Logger
Logs rope position control data (reference vs actual) to CSV and plots results on shutdown.
```bash
rosrun climbingrobot_hardware_interface position_control_logger.py \
  _side:=left \
  _output_csv:=/tmp/position_control_log.csv
```
---
Position Step Test
Runs an automated step-response test: holds at 0 m, steps to `step_m`, returns to 0 m,
then saves CSV and plots results.
```bash
rosrun climbingrobot_hardware_interface position_step_test.py \
  _side:=left \
  _step_m:=0.30 \
  _hold_0_s:=2.0 \
  _hold_step_s:=18.0 \
  _hold_back_s:=18.0
```
---
Homing Procedure
Runs the automated homing sequence for a winch.
```bash
rosrun climbingrobot_hardware_interface homing_procedure.py _side:=left
```
Or via launch file (includes full bringup with 5 s startup delay):
```bash
roslaunch climbingrobot_hardware_interface homing.launch
```
---
Alpine Odometry Node
Publishes odometry from Alpine IMU/encoder data.
```bash
rosrun climbingrobot_hardware_interface alpine_odometry_node.py
```
```bash
/odom                         nav_msgs/Odometry
/alpine/odometry/pose         geometry_msgs/PoseStamped
/alpine/odometry/debug        std_msgs/Float32MultiArray
/alpine_body/telemetry        climbingrobot_hardware_interface/AlpineBodyTelemetry
```
---
ROS 2 -> ROS 1 Quick Reference
ROS 2 CLI	ROS 1 CLI
ros2 run <pkg> <node>	rosrun <pkg> <node>
--ros-args -p key:=val	_key:=val
ros2 topic pub --once /t T "{...}"	rostopic pub -1 /t T "{...}"
ros2 topic pub --rate N /t T "{...}"	rostopic pub -r N /t T "{...}"
ros2 topic echo /t	rostopic echo /t
ros2 service call /s T "{}"	rosservice call /s "{}"
ros2 topic list	rostopic list
ros2 node list	rosnode list
ros2 param set /node key val	rosparam set /node/key val
ros2 launch pkg file.launch.py	roslaunch pkg file.launch
FindPackageShare("pkg")	$(find pkg)
IncludeLaunchDescription(...)	<include file="..."/>
TimerAction(period=N, actions=[...])	launch-prefix="bash -c 'sleep N && exec ...'
