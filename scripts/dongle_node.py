#!/usr/bin/env python3
"""
dongle_node.py — ROS 1 USB serial bridge for ALPINE body.

Subs:
  alpine_body/motorSpeed          std_msgs/Float32  -> m<val>
  alpine_body/servoValve1         std_msgs/Float32  -> s1 <deg>
  alpine_body/servoValve2         std_msgs/Float32  -> s2 <deg>
  alpine_body/cmd_raw             std_msgs/String   -> raw firmware command
  alpine_body/wrench_cmd          geometry_msgs/Wrench -> WRC,fx,fy,mz
  alpine_body/propeller_command   PropellerCommand  -> PROP_COMMAND,p0,p1,p2,p3

Pubs:
  alpine_body/telemetry/raw       std_msgs/String
  alpine_body/telemetry_array     std_msgs/Float32MultiArray
"""

import re
import threading
import time
from typing import List, Optional, Tuple

import rospy
import serial

from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Wrench

try:
    from climbingrobot_hardware_interface.msg import PropellerCommand
except Exception:
    PropellerCommand = None


_RX_PREFIX = re.compile(r'^\[RX [0-9A-Fa-f:]{17}\]\s*')


def _strip_prefix(line: str) -> str:
    return _RX_PREFIX.sub('', line).strip()


def _split_flex_csv(s: str) -> List[str]:
    s = s.replace('\t', ',')
    s = re.sub(r'[ ,]+', ',', s.strip())
    if s.endswith(','):
        s = s[:-1]
    return [p for p in s.split(',') if p != '']


def _parse_dual_imu(line: str) -> Optional[Tuple[int, List[float], List[float]]]:
    core = _strip_prefix(line)
    parts = _split_flex_csv(core)

    # Expected:
    # epoch_ms + 11 fields IMU1 + 11 fields IMU2 = 23 fields total
    if len(parts) < 23:
        return None

    try:
        epoch_ms = int(float(parts[0]))
        vals = [float(x) for x in parts[1:]]
    except Exception:
        return None

    if len(vals) < 22:
        return None

    imu1 = vals[0:11]
    imu2 = vals[11:22]
    return epoch_ms, imu1, imu2


class DongleNode:
    def __init__(self):
        # ---- Parameters ------------------------------------------------
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud = int(rospy.get_param('~baud', 115200))
        self.poll_rate = float(rospy.get_param('~poll_rate', 100.0))
        self.period = max(0.001, 1.0 / self.poll_rate)

        # ---- Propeller config-on-start --------------------------------
        self.apply_propeller_config_on_start = bool(
            rospy.get_param('~apply_propeller_config_on_start', False)
        )
        self.apply_propeller_config_delay_s = float(
            rospy.get_param('~apply_propeller_config_delay_s', 1.0)
        )
        self.propeller_low_level_ns = str(
            rospy.get_param('~propeller_low_level_ns', '/alpine/propellers/low_level')
        ).rstrip('/')
        self._startup_cfg_scheduled = False

        # ---- Serial robustness ----------------------------------------
        self.serial_settle_delay_s = float(rospy.get_param('~serial_settle_delay_s', 1.5))
        self.rx_watchdog_timeout_s = float(rospy.get_param('~rx_watchdog_timeout_s', 0.0))
        self._last_rx_monotonic = time.monotonic()

        # Runtime firmware mode command, e.g. "ros on"
        self.body_serial_mode_on_start = bool(
            rospy.get_param('~body_serial_mode_on_start', False)
        )
        self.body_serial_mode_cmd = str(
            rospy.get_param('~body_serial_mode_cmd', 'ros on')
        )

        # ---- Publishers ------------------------------------------------
        self.pub_raw = rospy.Publisher(
            'alpine_body/telemetry/raw',
            String,
            queue_size=100
        )
        self.pub_parsed = rospy.Publisher(
            'alpine_body/telemetry_array',
            Float32MultiArray,
            queue_size=100
        )

        # ---- Subscribers -----------------------------------------------
        rospy.Subscriber('alpine_body/motorSpeed',  Float32, self._cb_motor,  queue_size=10)
        rospy.Subscriber('alpine_body/servoValve1', Float32, self._cb_s1,     queue_size=10)
        rospy.Subscriber('alpine_body/servoValve2', Float32, self._cb_s2,     queue_size=10)

        rospy.Subscriber('alpine_body/cmd_raw',    String, self._cb_raw,    queue_size=20)
        rospy.Subscriber('alpine_body/wrench_cmd', Wrench, self._cb_wrench, queue_size=20)

        if PropellerCommand is not None:
            rospy.Subscriber(
                'alpine_body/propeller_command',
                PropellerCommand,
                self._cb_prop_command,
                queue_size=20
            )
        else:
            rospy.logwarn(
                "PropellerCommand msg not importable; "
                "skipping /alpine_body/propeller_command subscriber."
            )

        # ---- Serial state ----------------------------------------------
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()
        self._ser_lock = threading.Lock()
        self._last_open_attempt = 0.0

        # Tiny debounce for repeated identical commands
        self._last_tx = {}
        self._min_repeat_dt = 0.01

        self._open_serial()

        rospy.Timer(rospy.Duration(self.period), self._poll_serial)

        rospy.loginfo(
            "body_serial_node/dongle_node: port=%s baud=%d poll_rate=%.1f Hz",
            self.serial_port,
            self.baud,
            self.poll_rate,
        )

    # ------------------------------------------------------------------ #
    # Config helpers
    # ------------------------------------------------------------------ #

    def _cfg(self, suffix: str, default):
        path = f"{self.propeller_low_level_ns}/{suffix}".replace('//', '/')
        return rospy.get_param(path, default)

    def _schedule_propeller_config(self):
        if not self.apply_propeller_config_on_start:
            return
        if self._startup_cfg_scheduled:
            return

        self._startup_cfg_scheduled = True
        thread = threading.Thread(
            target=self._startup_propeller_config_worker,
            daemon=True
        )
        thread.start()

    def _startup_propeller_config_worker(self):
        try:
            delay = max(0.0, float(self.apply_propeller_config_delay_s))
            if delay > 0.0:
                time.sleep(delay)

            cmds = []

            pitch_dead = float(self._cfg('pitch/deadband_deg', 3.0))
            yaw_dead   = float(self._cfg('yaw/deadband_deg',   3.0))

            pitch_kp = float(self._cfg('pitch/pid/kp', 1.20))
            pitch_ki = float(self._cfg('pitch/pid/ki', 0.00))
            pitch_kd = float(self._cfg('pitch/pid/kd', 0.05))

            yaw_kp = float(self._cfg('yaw/pid/kp', 0.90))
            yaw_kd = float(self._cfg('yaw/pid/kd', 0.04))

            pitch_umax = float(self._cfg('pitch/umax', 0.20))
            yaw_umax   = float(self._cfg('yaw/umax',   0.15))

            cmds.append(f"pdb {pitch_dead:.3f}")
            cmds.append(f"ydb {yaw_dead:.3f}")
            cmds.append(f"pid {pitch_kp:.6f} {pitch_ki:.6f} {pitch_kd:.6f}")
            cmds.append(f"ypid {yaw_kp:.6f} {yaw_kd:.6f}")
            cmds.append(f"umax {pitch_umax:.3f}")
            cmds.append(f"yumax {yaw_umax:.3f}")

            send_attzero = bool(self._cfg('startup/send_attzero_on_start', False))
            enable_att   = bool(self._cfg('startup/enable_attitude_hold_on_start', False))
            enable_prop  = bool(self._cfg('startup/enable_open_loop_on_start', False))

            if send_attzero:
                cmds.append('attzero')

            cmds.append('atton' if enable_att else 'attoff')
            cmds.append('pron' if enable_prop else 'proff')

            rospy.loginfo(
                "[body_serial_node] Applying propeller low-level config from %s",
                self.propeller_low_level_ns
            )

            for cmd in cmds:
                self._send_line(cmd)
                time.sleep(0.05)

            self._send_line('status')
            rospy.loginfo("[body_serial_node] Propeller low-level config sent.")

        except Exception as exc:
            rospy.logwarn(
                "[body_serial_node] Failed to apply propeller config on start: %s",
                exc
            )

    # ------------------------------------------------------------------ #
    # Serial helpers
    # ------------------------------------------------------------------ #

    def _open_serial(self):
        now = time.monotonic()
        if now - self._last_open_attempt < 0.5:
            return

        self._last_open_attempt = now

        try:
            with self._ser_lock:
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass

                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud,
                    timeout=0.01,
                    write_timeout=0.01,
                    exclusive=False,
                    rtscts=False,
                    dsrdtr=False,
                )

            settle = max(0.0, float(self.serial_settle_delay_s))
            if settle > 0.0:
                time.sleep(settle)

            with self._ser_lock:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self._buf.clear()

            self._last_rx_monotonic = time.monotonic()

            rospy.loginfo("Opened serial: %s @ %d", self.serial_port, self.baud)

            if self.body_serial_mode_on_start:
                self._send_line(self.body_serial_mode_cmd)
                time.sleep(0.05)

            self._startup_cfg_scheduled = False
            self._schedule_propeller_config()

        except Exception as exc:
            rospy.logerr("Serial open failed: %s", exc)
            self.ser = None

    def _close_serial(self):
        try:
            with self._ser_lock:
                if self.ser is not None:
                    self.ser.close()
        except Exception:
            pass

        self.ser = None
        self._startup_cfg_scheduled = False

    def _send_line(self, text: str):
        text = str(text).strip()
        if not text:
            return

        if not text.endswith('\n'):
            text += '\n'

        now = time.monotonic()
        prev = self._last_tx.get(text)
        if prev is not None and (now - prev) < self._min_repeat_dt:
            return

        self._last_tx[text] = now

        try:
            with self._ser_lock:
                if self.ser is not None and self.ser.is_open:
                    self.ser.write(text.encode('utf-8', errors='ignore'))
                else:
                    rospy.logwarn("Serial not open, dropping command: %r", text.strip())
        except Exception as exc:
            rospy.logwarn("Serial write error: %s", exc)
            self._close_serial()

    # ------------------------------------------------------------------ #
    # Subscribers
    # ------------------------------------------------------------------ #

    def _cb_motor(self, msg: Float32):
        self._send_line(f"m{float(msg.data):.6f}")

    def _cb_s1(self, msg: Float32):
        self._send_line(f"s1 {float(msg.data):.3f}")

    def _cb_s2(self, msg: Float32):
        self._send_line(f"s2 {float(msg.data):.3f}")

    def _cb_raw(self, msg: String):
        cmd = (msg.data or '').strip()
        if cmd:
            self._send_line(cmd)

    def _cb_wrench(self, msg: Wrench):
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        mz = float(msg.torque.z)
        self._send_line(f"WRC,{fx:.6f},{fy:.6f},{mz:.6f}")

    def _cb_prop_command(self, msg):
        try:
            p0 = float(msg.propeller_thrust_0)
            p1 = float(msg.propeller_thrust_1)
            p2 = float(msg.propeller_thrust_2)
            p3 = float(msg.propeller_thrust_3)
            self._send_line(f"PROP_COMMAND,{p0:.6f},{p1:.6f},{p2:.6f},{p3:.6f}")
        except Exception as exc:
            rospy.logwarn("Invalid PropellerCommand: %s", exc)

    # ------------------------------------------------------------------ #
    # Serial RX polling
    # ------------------------------------------------------------------ #

    def _poll_serial(self, event):
        del event

        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            return

        if self.rx_watchdog_timeout_s > 0.0:
            age = time.monotonic() - self._last_rx_monotonic
            if age > self.rx_watchdog_timeout_s:
                rospy.logwarn(
                    "Serial RX watchdog timeout %.2fs -> reopening %s",
                    age,
                    self.serial_port
                )
                self._close_serial()
                self._open_serial()
                return

        try:
            with self._ser_lock:
                chunk = self.ser.read(1024)

            if chunk:
                self._buf.extend(chunk)

            if len(self._buf) > 8192:
                rospy.logwarn("RX buffer overflow guard: clearing partial buffer")
                self._buf.clear()

            while True:
                nl = self._buf.find(b'\n')
                if nl < 0:
                    break

                line_bytes = self._buf[:nl + 1]
                del self._buf[:nl + 1]

                line = line_bytes.decode(errors='ignore').strip()
                if not line:
                    continue

                self._last_rx_monotonic = time.monotonic()
                self.pub_raw.publish(String(data=line))

                parsed = _parse_dual_imu(line)
                if parsed is not None:
                    epoch_ms, imu1, imu2 = parsed
                    data = (
                        [float(epoch_ms)]
                        + (imu1 + [0.0] * 11)[:11]
                        + (imu2 + [0.0] * 11)[:11]
                    )
                    layout = MultiArrayLayout(
                        dim=[
                            MultiArrayDimension(
                                label='fields',
                                size=len(data),
                                stride=len(data)
                            )
                        ],
                        data_offset=0
                    )
                    self.pub_parsed.publish(
                        Float32MultiArray(layout=layout, data=data)
                    )

        except Exception as exc:
            rospy.logwarn("Serial read error: %s", exc)
            self._close_serial()


def main():
    rospy.init_node('dongle_node')
    DongleNode()
    rospy.spin()


if __name__ == '__main__':
    main()
