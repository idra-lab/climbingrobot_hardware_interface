#!/usr/bin/env python3
"""
dongle_node.py — ROS 1 node that bridges USB serial to topics.

Private params:
  ~serial_port  (string, default /dev/ttyUSB0)
  ~baud         (int,    default 115200)
  ~poll_rate    (float,  default 200.0 Hz)

Existing subs:
  alpine/dongle/motorSpeed  -> send "m<val>"
  alpine/dongle/servoValve1 -> send "s1 <deg>"
  alpine/dongle/servoValve2 -> send "s2 <deg>"

Added for 6 propellers:
  alpine/dongle/wrench_cmd  (geometry_msgs/Wrench)
      -> send "WRC,fx,fy,mz"
  alpine/dongle/cmd_raw     (std_msgs/String)
      -> send raw commands like: atton, attzero, apzero, ayon, attoff, ...

Pubs:
  alpine/dongle/telemetry/raw  (std_msgs/String):             raw CSV lines
  alpine/dongle/telemetry      (std_msgs/Float32MultiArray):  [epoch_ms, imu1[11], imu2[11]]
"""

import re
import threading
import time
from typing import List, Optional, Tuple

import rospy

from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Wrench

import serial

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
        self.baud        = int(rospy.get_param('~baud', 115200))
        self.poll_rate   = float(rospy.get_param('~poll_rate', 200.0))
        self.period      = max(0.001, 1.0 / self.poll_rate)

        # ---- Publishers ------------------------------------------------
        self.pub_raw    = rospy.Publisher('alpine/dongle/telemetry/raw', String,            queue_size=100)
        self.pub_parsed = rospy.Publisher('alpine/dongle/telemetry',     Float32MultiArray, queue_size=100)

        # ---- Subscribers -----------------------------------------------
        rospy.Subscriber('alpine/dongle/motorSpeed',  Float32, self._cb_motor,  queue_size=10)
        rospy.Subscriber('alpine/dongle/servoValve1', Float32, self._cb_s1,     queue_size=10)
        rospy.Subscriber('alpine/dongle/servoValve2', Float32, self._cb_s2,     queue_size=10)
        rospy.Subscriber('alpine/dongle/wrench_cmd',  Wrench,  self._cb_wrench, queue_size=20)
        rospy.Subscriber('alpine/dongle/cmd_raw',     String,  self._cb_raw,    queue_size=20)

        # ---- Serial state ----------------------------------------------
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()
        self._ser_lock = threading.Lock()
        self._last_open_attempt = 0.0

        # Optional tiny de-bounce for repeated identical commands
        self._last_tx: dict = {}
        self._min_repeat_dt = 0.01  # 10 ms

        self._open_serial()

        # ---- Polling timer ---------------------------------------------
        rospy.Timer(rospy.Duration(self.period), self._poll_serial)

        rospy.loginfo(
            f"dongle_node: port={self.serial_port} baud={self.baud} poll_rate={self.poll_rate} Hz"
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
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud,
                    timeout=0.01,
                    write_timeout=0.01,
                    exclusive=False,
                )
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self._buf.clear()
            rospy.loginfo(f"Opened serial: {self.serial_port} @ {self.baud}")
        except Exception as e:
            rospy.logerr(f"Serial open failed: {e}")
            self.ser = None

    def _close_serial(self):
        try:
            with self._ser_lock:
                if self.ser is not None:
                    self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _send_line(self, text: str):
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
                    rospy.logwarn("Serial not open, dropping command")
        except Exception as e:
            rospy.logwarn(f"Serial write error: {e}")
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

    # ------------------------------------------------------------------ #
    # Poll serial (timer callback — ROS 1 requires event argument)
    # ------------------------------------------------------------------ #

    def _poll_serial(self, event):
        if self.ser is None or not self.ser.is_open:
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

                self.pub_raw.publish(String(data=line))

                parsed = _parse_dual_imu(line)
                if parsed is not None:
                    epoch_ms, imu1, imu2 = parsed
                    data = [float(epoch_ms)] + (imu1 + [0.0] * 11)[:11] + (imu2 + [0.0] * 11)[:11]
                    layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='fields', size=len(data), stride=len(data))],
                        data_offset=0
                    )
                    self.pub_parsed.publish(Float32MultiArray(layout=layout, data=data))

        except Exception as e:
            rospy.logwarn(f"Serial read error: {e}")
            self._close_serial()


def main():
    rospy.init_node('dongle_node')
    node = DongleNode()
    rospy.spin()


if __name__ == '__main__':
    main()