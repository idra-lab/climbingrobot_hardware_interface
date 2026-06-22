#!/usr/bin/env python3

import csv
import math
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Wrench


G = 9.80665


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def parse_values(s):
    return [float(x.strip()) for x in str(s).replace(";", ",").split(",") if x.strip()]


def measurement_to_newton(value, unit):
    unit = unit.lower().strip()

    if unit in ("g", "gram", "grams", "grammi"):
        return float(value) / 1000.0 * G

    if unit in ("kg", "kilogram", "kilograms"):
        return float(value) * G

    if unit in ("n", "newton", "newtons"):
        return float(value)

    raise ValueError("measurement_unit must be grams, kg, or N")


class ForceCalibration:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "lat_left_thr")
        self.values = parse_values(rospy.get_param(
            "~values",
            "0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90,1.0"
        ))

        self.measurement_unit = rospy.get_param("~measurement_unit", "grams")
        self.hold_rate_hz = float(rospy.get_param("~hold_rate_hz", 20.0))
        self.settle_s = float(rospy.get_param("~settle_s", 1.0))
        self.zero_s = float(rospy.get_param("~zero_s", 0.5))
        self.lever_arm_m = float(rospy.get_param("~lever_arm_m", 0.0))

        default_out = f"/root/ros_ws/prop_force_calib_{self.mode}_{int(time.time())}.csv"
        self.out_csv = Path(rospy.get_param("~out_csv", default_out))
        self.out_yaml = self.out_csv.with_suffix(".yaml")

        self.pub_raw = rospy.Publisher("/alpine_body/cmd_raw", String, queue_size=1)
        self.pub_wrench = rospy.Publisher("/alpine_body/wrench_cmd", Wrench, queue_size=1)

        self._lock = threading.Lock()
        self._active = True
        self._cmd_type = "raw"
        self._raw_cmd = "THR,0,0,0,0,0,0"
        self._wrench = (0.0, 0.0, 0.0)

        rospy.sleep(0.5)

        self._thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self._thread.start()

    def _publisher_loop(self):
        rate = rospy.Rate(self.hold_rate_hz)

        while not rospy.is_shutdown() and self._active:
            with self._lock:
                cmd_type = self._cmd_type
                raw_cmd = self._raw_cmd
                wrench = self._wrench

            if cmd_type == "raw":
                self.pub_raw.publish(String(data=raw_cmd))

            elif cmd_type == "wrench":
                msg = Wrench()
                msg.force.x = float(wrench[0])
                msg.force.y = float(wrench[1])
                msg.force.z = 0.0
                msg.torque.x = 0.0
                msg.torque.y = 0.0
                msg.torque.z = float(wrench[2])
                self.pub_wrench.publish(msg)

            rate.sleep()

    def set_raw(self, cmd):
        with self._lock:
            self._cmd_type = "raw"
            self._raw_cmd = cmd

    def set_wrench(self, fx, fy, mz):
        with self._lock:
            self._cmd_type = "wrench"
            self._wrench = (float(fx), float(fy), float(mz))

    def stop(self):
        self.set_raw("THR,0,0,0,0,0,0")
        rospy.sleep(0.1)
        self.pub_raw.publish(String(data="stop"))

    def command_for_mode(self, u):
        """
        Returns:
          cmd_type, command, direction_label, signed_axis

        signed_axis is only a convention label for the result table.
        Magnitude calibration uses abs(force).
        """

        u = clamp(float(u), 0.0, 1.0)
        m = self.mode.strip().lower()

        # Direct single thrusters
        if m == "t1":
            return "raw", f"THR,{u:.4f},0,0,0,0,0", "T1", "+T1"
        if m == "t2":
            return "raw", f"THR,0,{u:.4f},0,0,0,0", "T2", "+T2"
        if m == "t3":
            return "raw", f"THR,0,0,{u:.4f},0,0,0", "T3", "+T3"
        if m == "t4":
            return "raw", f"THR,0,0,0,{u:.4f},0,0", "T4", "+T4"
        if m == "t5":
            return "raw", f"THR,0,0,0,0,{u:.4f},0", "T5 / pitch down", "pitch_down"
        if m == "t6":
            return "raw", f"THR,0,0,0,0,0,{u:.4f}", "T6 / pitch up", "pitch_up"

        # Physical pairs, direct THR
        if m == "yaw_right_thr":
            return "raw", f"THR,{u:.4f},0,{u:.4f},0,0,0", "yaw right: T1+T3", "+Mz_right"
        if m == "yaw_left_thr":
            return "raw", f"THR,0,{u:.4f},0,{u:.4f},0,0", "yaw left: T2+T4", "-Mz_left"

        if m == "lat_left_thr":
            return "raw", f"THR,{u:.4f},{u:.4f},0,0,0,0", "lateral left: T1+T2", "+Fy_left"
        if m == "lat_right_thr":
            return "raw", f"THR,0,0,{u:.4f},{u:.4f},0,0", "lateral right: T3+T4", "-Fy_right"

        if m == "pitch_down":
            return "raw", f"pth {u:.4f}", "pitch down: T5", "pitch_down"
        if m == "pitch_up":
            return "raw", f"pth {-u:.4f}", "pitch up: T6", "pitch_up"

        # WRC path, same as high level. Requires firmware mapping to be correct.
        if m == "wrc_yaw_right":
            return "wrench", (0.0, 0.0, +u), "WRC yaw right: +Mz", "+Mz_right"
        if m == "wrc_yaw_left":
            return "wrench", (0.0, 0.0, -u), "WRC yaw left: -Mz", "-Mz_left"

        if m == "wrc_lat_left":
            return "wrench", (0.0, +u, 0.0), "WRC lateral left: +Fy", "+Fy_left"
        if m == "wrc_lat_right":
            return "wrench", (0.0, -u, 0.0), "WRC lateral right: -Fy", "-Fy_right"

        raise RuntimeError(
            "Unknown mode. Use: t1..t6, yaw_right_thr, yaw_left_thr, "
            "lat_left_thr, lat_right_thr, pitch_down, pitch_up, "
            "wrc_yaw_right, wrc_yaw_left, wrc_lat_left, wrc_lat_right"
        )

    def fit_and_write_yaml(self, rows):
        usable = [
            r for r in rows
            if math.isfinite(r["force_N_abs"]) and r["force_N_abs"] >= 0.0
        ]

        if len(usable) < 2:
            print("Not enough points for fit.")
            return

        u = np.array([r["u"] for r in usable], dtype=float)
        f = np.array([r["force_N_abs"] for r in usable], dtype=float)

        # Linear through zero: F = k*u
        k = float(np.sum(u * f) / max(np.sum(u * u), 1e-12))

        # Quadratic: F = a*u^2 + b*u + c
        if len(usable) >= 3:
            a, b, c = np.polyfit(u, f, 2)
            a, b, c = float(a), float(b), float(c)
        else:
            a, b, c = 0.0, k, 0.0

        with self.out_yaml.open("w") as fp:
            fp.write("# ALPINE propeller force calibration\n")
            fp.write(f"mode: {self.mode}\n")
            fp.write(f"measurement_unit: {self.measurement_unit}\n")
            fp.write(f"lever_arm_m: {self.lever_arm_m:.6f}\n")
            fp.write("fit:\n")
            fp.write(f"  linear_through_zero_N_per_cmd: {k:.9f}\n")
            fp.write("  quadratic_force_N:\n")
            fp.write(f"    a_u2: {a:.9f}\n")
            fp.write(f"    b_u: {b:.9f}\n")
            fp.write(f"    c: {c:.9f}\n")
            if self.lever_arm_m > 0.0:
                fp.write(f"  linear_through_zero_Nm_per_cmd: {k * self.lever_arm_m:.9f}\n")
            fp.write("samples:\n")
            for r in rows:
                fp.write(
                    f"  - u: {r['u']:.6f}, force_N_abs: {r['force_N_abs']:.9f}, "
                    f"moment_Nm_abs: {r['moment_Nm_abs']:.9f}, label: \"{r['direction_label']}\"\n"
                )

        print("")
        print("=== FIT RESULT ===")
        print(f"F_abs[N] ≈ {k:.4f} * u")
        print(f"F_abs[N] ≈ {a:.4f} * u^2 + {b:.4f} * u + {c:.4f}")
        if self.lever_arm_m > 0.0:
            print(f"Moment_abs[Nm] ≈ {k * self.lever_arm_m:.4f} * u")
        print(f"Wrote: {self.out_yaml}")

    def run(self):
        print("")
        print("=== ALPINE propeller force calibration ===")
        print(f"mode={self.mode}")
        print(f"values={self.values}")
        print(f"measurement_unit={self.measurement_unit}")
        print(f"out_csv={self.out_csv}")
        print("")
        print("Measure force with scale/load-cell.")
        print("Input measured value when stable. Empty input skips point. 'q' quits.")
        print("")

        self.pub_raw.publish(String(data="attoff"))
        rospy.sleep(0.05)
        self.pub_raw.publish(String(data="proff"))
        rospy.sleep(0.05)
        self.stop()
        rospy.sleep(0.3)

        rows = []

        try:
            for u in self.values:
                cmd_type, command, direction_label, signed_axis = self.command_for_mode(u)

                print("")
                print(f"Command u={u:.3f} | {direction_label}")

                if cmd_type == "raw":
                    print(f"RAW: {command}")
                    self.set_raw(command)
                else:
                    print(f"WRENCH: fx={command[0]:.3f}, fy={command[1]:.3f}, mz={command[2]:.3f}")
                    self.pub_raw.publish(String(data="pron"))
                    rospy.sleep(0.05)
                    self.set_wrench(*command)

                rospy.sleep(self.settle_s)

                s = input(f"Measured force [{self.measurement_unit}] for u={u:.3f}: ").strip()

                if s.lower() in ("q", "quit", "exit"):
                    break

                if s == "":
                    force_N_abs = float("nan")
                    meas = float("nan")
                else:
                    meas = float(s)
                    force_N_abs = abs(measurement_to_newton(meas, self.measurement_unit))

                moment_Nm_abs = force_N_abs * self.lever_arm_m if self.lever_arm_m > 0.0 else float("nan")

                rows.append({
                    "stamp": time.time(),
                    "mode": self.mode,
                    "u": float(u),
                    "direction_label": direction_label,
                    "signed_axis": signed_axis,
                    "measurement": meas,
                    "measurement_unit": self.measurement_unit,
                    "force_N_abs": force_N_abs,
                    "moment_Nm_abs": moment_Nm_abs,
                    "cmd_type": cmd_type,
                    "command": str(command),
                })

                self.stop()
                rospy.sleep(self.zero_s)

        finally:
            self._active = False
            self.stop()

        self.out_csv.parent.mkdir(parents=True, exist_ok=True)

        with self.out_csv.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=[
                "stamp",
                "mode",
                "u",
                "direction_label",
                "signed_axis",
                "measurement",
                "measurement_unit",
                "force_N_abs",
                "moment_Nm_abs",
                "cmd_type",
                "command",
            ])
            writer.writeheader()
            for r in rows:
                writer.writerow(r)

        print(f"Wrote: {self.out_csv}")
        self.fit_and_write_yaml(rows)


if __name__ == "__main__":
    rospy.init_node("propeller_force_calibration")
    ForceCalibration().run()
