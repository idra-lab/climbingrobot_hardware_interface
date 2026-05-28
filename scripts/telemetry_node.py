#!/usr/bin/env python3

import json
import math
import time
import threading
import serial
import csv
from pathlib import Path
from typing import Optional, List, Dict
from collections import deque

import rospy
from std_msgs.msg import String
from climbingrobot_hardware_interface.srv import RopeControlMode, RopeControlModeResponse
from std_srvs.srv import Trigger, TriggerResponse

from climbingrobot_hardware_interface.msg import RopeCommand
from climbingrobot_hardware_interface.msg import DebugMessage
from climbingrobot_hardware_interface.msg import RopeTelemetry


class TelemetryNode:
    """
    Reads CSV telemetry from MCU, republishes raw lines, computes robust velocities.
    ROS 1 (rospy) port of the original ROS 2 (rclpy) node.
    """

    def __init__(self):

        # ── Parameters ─────────────────────────────────────────────────────────
        self.port           = rospy.get_param('~serial_port',   '/dev/ttyUSB0')
        self.baud           = int(rospy.get_param('~baud',      1_000_000))
        self.poll_rate_hz   = float(rospy.get_param('~poll_rate_hz', 200.0))

        side_in   = str(rospy.get_param('~side', 'left')).strip().lower()
        self.side = side_in if side_in in ('left', 'right') else 'left'

        default_cfg = str(
            Path(__file__).resolve().parent.parent / 'config' / 'arganelloTelemetry.json'
        )
        self.config_path        = rospy.get_param('~config_path',   default_cfg)
        self.expect_header      = bool(rospy.get_param('~csv_expect_header',                  True))
        self.replace_col0_with_unix = bool(rospy.get_param('~replace_first_column_with_unix_time', False))
        self.debug_mode         = bool(rospy.get_param('~debug_mode', False))

        self.send_sync_on_start = bool(rospy.get_param('~send_sync_on_start', True))
        self.resync_interval_s  = float(rospy.get_param('~resync_interval_s', 0.0))
        self.sync_epoch_unit    = str(rospy.get_param('~sync_epoch_unit', 'ms')).strip().lower()
        self.append_pc_time_ns  = bool(rospy.get_param('~append_pc_time_ns', False))

        self.sync_roller_radius_m   = float(rospy.get_param('~sync_roller_radius_m',   0.025))
        self.sync_roller_cpr        = int(rospy.get_param('~sync_roller_cpr',          2400))
        self.rope_diameter_m        = float(rospy.get_param('~rope_diameter_m',        0.005))
        self.rope_position_calibration = float(rospy.get_param('~rope_position_calibration', 17.0))

        self.rope_position_outer_loop_enabled = bool(rospy.get_param('~rope_position_outer_loop_enabled', True))
        self.rope_position_kp               = float(rospy.get_param('~rope_position_kp',               6.0))
        self.rope_position_max_vel_m_s      = float(rospy.get_param('~rope_position_max_vel_m_s',      0.10))
        self.rope_position_deadband_m       = float(rospy.get_param('~rope_position_deadband_m',       0.0015))
        self.rope_direction_sign            = float(rospy.get_param('~rope_direction_sign',            -1.0))
        self.torque_dir_right               = -1.0
        self.torque_dir_left                =  1.0
        self.rope_position_motor_vel_scale  = float(rospy.get_param('~rope_position_motor_vel_scale',  0.75))
        self.rope_position_up_near_vel_m_s  = float(rospy.get_param('~rope_position_up_near_vel_m_s',  0.008))
        self.rope_position_up_far_vel_m_s   = float(rospy.get_param('~rope_position_up_far_vel_m_s',   0.040))
        self.rope_position_down_near_vel_m_s= float(rospy.get_param('~rope_position_down_near_vel_m_s',0.110))
        self.rope_position_down_far_vel_m_s = float(rospy.get_param('~rope_position_down_far_vel_m_s', 0.180))
        self.rope_position_profile_zone_m   = float(rospy.get_param('~rope_position_profile_zone_m',   0.004))

        self.max_motor_rps      = float(rospy.get_param('~max_motor_rps',      20.0))
        self.max_roller_rps     = float(rospy.get_param('~max_roller_rps',     20.0))
        self.count_deadband     = int(rospy.get_param('~count_deadband',        2))
        self.lpf_fc_pos_motor   = float(rospy.get_param('~lpf_fc_pos_motor',   5.0))
        self.lpf_fc_pos_roller  = float(rospy.get_param('~lpf_fc_pos_roller',  5.0))
        self.brake_zero_rps     = float(rospy.get_param('~brake_zero_rps',     0.2))
        self.phys_max_rope_m_s  = float(rospy.get_param('~phys_max_rope_m_s',  5.0))

        self.gear_ratio_nominal         = float(rospy.get_param('~gear_ratio_nominal',         2.31))
        self.gear_ratio_update_enabled  = bool(rospy.get_param('~gear_ratio_update_enabled',   False))
        self.gear_ratio_min             = float(rospy.get_param('~gear_ratio_min',             0.5))
        self.gear_ratio_max             = float(rospy.get_param('~gear_ratio_max',             6.0))
        self.gear_ratio_alpha           = float(rospy.get_param('~gear_ratio_alpha',           0.02))
        self.gear_ratio_motor_eps       = float(rospy.get_param('~gear_ratio_motor_eps',       0.20))
        self.gear_ratio_roller_eps      = float(rospy.get_param('~gear_ratio_roller_eps',      0.05))
        self.gear_ratio_max_step        = float(rospy.get_param('~gear_ratio_max_step',        0.15))
        self.gear_ratio_log_period      = int(rospy.get_param('~gear_ratio_log_period',        200))

        # ── Serial ─────────────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.005)
            rospy.loginfo(f"✅ Serial opened: {self.port} @ {self.baud}")
            self.ser.reset_input_buffer()
        except Exception as e:
            rospy.logfatal(f"❌ Failed to open serial: {e}")
            raise

        # ── Publishers ────────────────────────────────────────────────────────
        base = f"/winch/{self.side}"

        self.pub_csv   = rospy.Publisher(f"{base}/telemetry/csv",   String,       queue_size=10)
        self.pub_debug = rospy.Publisher(f"{base}/telemetry/debug", DebugMessage, queue_size=10)
        self.pub_rope  = rospy.Publisher(f"{base}/telemetry",       RopeTelemetry,queue_size=10)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber(f"{base}/set_motor_mode", String,      self._set_motor_mode_cb, queue_size=10)
        rospy.Subscriber(f"{base}/command",        RopeCommand, self._rope_command_cb,   queue_size=10)

        # ── Services ─────────────────────────────────────────────────────────
        rospy.Service(f"{base}/brake_engage",    Trigger, self._srv_brake_engage)
        rospy.Service(f"{base}/brake_disengage", Trigger, self._srv_brake_disengage)
        rospy.Service(f"{base}/sync_now",        Trigger, self._srv_sync_now)
        rospy.Service(f"{base}/rope_zero",       Trigger, self._srv_rope_zero)

        # Compatibility with climbingrobot_controller2_real.py
        # The real controller expects a service /winch/<side>/set_control_mode.
        # Internally we reuse the same mode handling as /winch/<side>/set_motor_mode.
        rospy.Service(f"{base}/set_control_mode", RopeControlMode, self._srv_set_control_mode)

        # ── TX queue & CSV header tracking ────────────────────────────────────
        self.tx_queue: deque = deque()
        self.header: List[str] = []
        self.name_to_idx: Dict[str, int] = {}
        self._stop = False

        # ── State holders ─────────────────────────────────────────────────────
        self._last_valid_motor_pos_norm: Optional[float] = None
        self._last_valid_sync_raw: Optional[int] = None

        self._motor_unwrapped_rev: Optional[float] = None
        self._roller_unwrapped_counts: Optional[float] = None

        self._hist_pos_motor_filt: deque = deque(maxlen=8)
        self._hist_pos_roller_filt: deque = deque(maxlen=8)

        self.motor_speed: float = 0.0
        self.sync_roller_speed: float = 0.0

        self.brake_status: bool = False
        self.current = None
        self.motor_torque = None
        self.tau_motor: float = float('nan')
        self.syncronous_roller_raw_wrapped = None
        self.motor_position = None

        self._motor_rev_zero: Optional[float] = None
        self._roller_counts_zero: Optional[float] = None
        self.rope_length_m: float = 0.0

        self.active_rope_control_mode = 'idle'
        self.rope_position_ref_m = None
        self._last_position_loop_log_time = 0.0
        self._last_position_ref_log_time = 0.0

        self.variable_gear_ratio_g: float = self.gear_ratio_nominal
        self._freeze_g = not self.gear_ratio_update_enabled
        self._gear_print_counter = 0

        rospy.loginfo(
            f"[gear] initial G={self.variable_gear_ratio_g:.3f}, "
            f"update_enabled={self.gear_ratio_update_enabled}, "
            f"limits=[{self.gear_ratio_min:.2f}, {self.gear_ratio_max:.2f}]"
        )

        # ── SYNC & CONFIG ─────────────────────────────────────────────────────
        if self.send_sync_on_start:
            self._send_sync()

        if self.resync_interval_s > 0:
            rospy.Timer(rospy.Duration(self.resync_interval_s), lambda e: self._send_sync())

        cfg = self._load_config(self.config_path)
        if cfg:
            self.send_cmd('CONFIG ' + json.dumps(cfg, separators=(',', ':')))
        else:
            rospy.logwarn("No CONFIG JSON found; device may not stream.")

        # ── Rope position outer-loop timer ────────────────────────────────────
        rospy.Timer(rospy.Duration(0.02), self._rope_position_outer_loop_cb)

        # ── Start IO threads ──────────────────────────────────────────────────
        threading.Thread(target=self._serial_reader_loop, name='serial-reader', daemon=True).start()
        threading.Thread(target=self._serial_writer_loop, name='serial-writer', daemon=True).start()

        if self.debug_mode:
            threading.Thread(target=self._stdin_loop, name='stdin', daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    # Public: enqueue raw line to MCU
    # ─────────────────────────────────────────────────────────────────────────
    def send_cmd(self, line: str) -> None:
        if line:
            self.tx_queue.append(line if line.endswith('\n') else (line + '\n'))

    # ─────────────────────────────────────────────────────────────────────────
    # Brake / sync / zero services  (ROS 1: single req arg, return TriggerResponse)
    # ─────────────────────────────────────────────────────────────────────────
    def _srv_brake_engage(self, req):
        cmd = 'set_brake 1'
        rospy.loginfo(f'→ {cmd}')
        self.send_cmd(cmd)
        return TriggerResponse(success=True, message='sent: set_brake 1')

    def _srv_brake_disengage(self, req):
        cmd = 'set_brake 0'
        rospy.loginfo(f'→ {cmd}')
        self.send_cmd(cmd)
        return TriggerResponse(success=True, message='sent: set_brake 0')

    def _srv_sync_now(self, req):
        epoch = self._send_sync()
        return TriggerResponse(success=True, message=f'sent: sync {epoch}')

    def _srv_rope_zero(self, req):
        if getattr(self, '_pos_roller_filt', None) is None:
            return TriggerResponse(success=False, message='No roller data yet; cannot zero.')

        self._roller_counts_zero = float(self._pos_roller_filt)
        self.rope_length_m = 0.0

        motor_now = getattr(self, '_pos_motor_filt', None)
        if motor_now is None:
            motor_now = getattr(self, '_motor_unwrapped_rev', None)
        if motor_now is not None:
            self._motor_rev_zero = float(motor_now)

        rospy.loginfo(
            f"✔ Rope zeroed: rope_length_m=0.0; "
            f"motor_ref={getattr(self, '_motor_rev_zero', float('nan'))}"
        )
        return TriggerResponse(success=True, message='Rope zeroed and motor reference aligned.')

    # ─────────────────────────────────────────────────────────────────────────
    # Motor mode callback
    # ─────────────────────────────────────────────────────────────────────────
    def _normalize_control_mode(self, mode: str) -> str:
        # Historical compatibility:
        # old real controller uses close_loop_*, hardware node uses closed_loop_*.
        aliases = {
            "close_loop_torque": "closed_loop_torque",
            "close_loop_position": "closed_loop_position",
            "close_loop_velocity": "closed_loop_velocity",
            "closed_loop_torque": "closed_loop_torque",
            "closed_loop_position": "closed_loop_position",
            "closed_loop_velocity": "closed_loop_velocity",
            "idle": "idle",
        }
        key = str(mode).strip()
        return aliases.get(key, key)

    def _srv_set_control_mode(self, req):
        mode = self._normalize_control_mode(req.message)
        try:
            # Reuse exactly the same logic as the /set_motor_mode topic callback.
            self._set_motor_mode_cb(String(data=mode))
            rospy.loginfo(f"[{self.side}] set_control_mode service -> {mode}")
            return RopeControlModeResponse(success=True)
        except Exception as e:
            rospy.logerr(f"[{self.side}] set_control_mode failed: {e}")
            return RopeControlModeResponse(success=False)

    def _set_motor_mode_cb(self, msg) -> None:
        m = (msg.data or '').strip().lower()

        if m == 'idle':
            self.active_rope_control_mode = 'idle'
            self.rope_position_ref_m = None
            self.send_cmd('send_odrive w axis0.requested_state 1')

        elif m in ('closed_loop_torque', 'close_loop_torque'):
            self.active_rope_control_mode = 'closed_loop_torque'
            self.rope_position_ref_m = None
            self.send_cmd('send_odrive w axis0.controller.config.input_mode 1')
            self.send_cmd('send_odrive w axis0.controller.config.control_mode 1')
            self.send_cmd('send_odrive w axis0.requested_state 8')

        elif m in ('closed_loop_velocity', 'close_loop_velocity',
                   'closed_loop_velocoty', 'close_loop_velocoty'):
            self.active_rope_control_mode = 'closed_loop_velocity'
            self.rope_position_ref_m = None
            self.send_cmd('send_odrive w axis0.controller.config.input_mode 1')
            self.send_cmd('send_odrive w axis0.controller.config.control_mode 2')
            self.send_cmd('send_odrive w axis0.requested_state 8')

        elif m in ('closed_loop_position', 'close_loop_position'):
            self.active_rope_control_mode = 'closed_loop_position'

            # Do not jump when entering position mode:
            # hold current measured/estimated rope length until a new command arrives.
            self.rope_position_ref_m = float(getattr(self, 'rope_length_m', 0.0))

            if self.rope_position_outer_loop_enabled:
                self.send_cmd('send_odrive w axis0.controller.input_vel 0.000000')
                self.send_cmd('send_odrive w axis0.controller.config.input_mode 1')
                self.send_cmd('send_odrive w axis0.controller.config.control_mode 2')
                self.send_cmd('send_odrive w axis0.requested_state 8')
                self.send_cmd('send_odrive w axis0.controller.input_vel 0.000000')
                rospy.logwarn(
                    'closed_loop_position requested: holding current rope_length until new position command'
                )
                rospy.logwarn(
                    'closed_loop_position requested: using ROS outer loop on rope_length + ODrive velocity mode'
                )
            else:
                self.send_cmd('send_odrive w axis0.controller.config.input_mode 1')
                self.send_cmd('send_odrive w axis0.controller.config.control_mode 3')
                self.send_cmd('send_odrive w axis0.requested_state 8')
        else:
            rospy.logwarn(
                'Unknown motor mode. Use: idle, closed_loop_torque, '
                'closed_loop_velocity, closed_loop_position'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Rope command callback
    # ─────────────────────────────────────────────────────────────────────────
    def _rope_command_cb(self, msg) -> None:
        rospy.loginfo(
            f"[command] force={msg.rope_force:.3f} N, "
            f"vel={msg.rope_velocity:.3f} m/s, "
            f"pos={msg.rope_position:.3f} m"
        )

        r0     = float(self.sync_roller_radius_m)
        d      = float(self.rope_diameter_m)
        r_eff  = r0 + 0.5 * d
        two_pi = 2.0 * math.pi

        G = float(self.variable_gear_ratio_g)
        if not (math.isfinite(G) and G > 1e-6):
            G = self.gear_ratio_nominal if self.gear_ratio_nominal > 1e-6 else 1.0
            rospy.logwarn(f'Gear ratio invalid; using fallback G={G:.3f}')

        torque_dir = self.torque_dir_right if self.side == 'right' else self.torque_dir_left

        if math.isfinite(msg.rope_force):
            tau_motor = float(msg.rope_force) * G * r_eff * torque_dir
            rospy.loginfo(
                f"[torque_dir] side={self.side}, torque_dir={torque_dir:+.1f}, "
                f"F_rope={float(msg.rope_force):.3f} N, tau_motor={tau_motor:.6f} N·m"
            )
            self.send_cmd(f'send_odrive w axis0.controller.input_torque {tau_motor:.6f}')
        else:
            tau_motor = float('nan')

        if math.isfinite(msg.rope_velocity):
            motor_vel_turns_s = (
                float(msg.rope_velocity) / max(r_eff, 1e-9)
            ) / (two_pi * max(G, 1e-9))
            self.send_cmd(f'send_odrive w axis0.controller.input_vel {motor_vel_turns_s:.6f}')
        else:
            motor_vel_turns_s = float('nan')

        if math.isfinite(msg.rope_position):
            if self.rope_position_outer_loop_enabled and self.active_rope_control_mode == 'closed_loop_position':
                self.rope_position_ref_m = float(msg.rope_position)
                motor_pos_turns_abs = float('nan')

                now = time.time()
                if now - self._last_position_ref_log_time > 0.5:
                    self._last_position_ref_log_time = now
                    rospy.loginfo(
                        f'→ rope position reference stored: {self.rope_position_ref_m:.4f} m'
                    )
            else:
                base_turns = (
                    float(self._motor_rev_zero)
                    if getattr(self, '_motor_rev_zero', None) is not None
                    else 0.0
                )
                cal = max(abs(float(self.rope_position_calibration)), 1e-9)
                delta_turns_raw = (
                    float(msg.rope_position) / max(r_eff, 1e-9)
                ) / (two_pi * max(G, 1e-9))
                delta_turns = delta_turns_raw / cal
                motor_pos_turns_abs = base_turns + delta_turns
                self.send_cmd(f'send_odrive w axis0.controller.input_pos {motor_pos_turns_abs:.6f}')
        else:
            motor_pos_turns_abs = float('nan')

        rospy.loginfo(
            f"→ mapped: τ={tau_motor if math.isfinite(tau_motor) else float('nan'):.3f} N·m, "
            f"vel={motor_vel_turns_s if math.isfinite(motor_vel_turns_s) else float('nan'):.3f} trn/s, "
            f"pos_abs={motor_pos_turns_abs if math.isfinite(motor_pos_turns_abs) else float('nan'):.6f} trn "
            f"(G={G:.3f}, r_eff={r_eff:.4f} m, "
            f"cal_pos={getattr(self, 'rope_position_calibration', float('nan')):.3f}, "
            f"θ0={getattr(self, '_motor_rev_zero', float('nan'))})"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Rope position outer-loop controller  (ROS 1 timer: receives event arg)
    # ─────────────────────────────────────────────────────────────────────────
    def _rope_position_outer_loop_cb(self, event=None):
        if not getattr(self, 'rope_position_outer_loop_enabled', False):
            return
        if getattr(self, 'active_rope_control_mode', 'idle') != 'closed_loop_position':
            return
        if self.rope_position_ref_m is None:
            return
        if bool(getattr(self, 'brake_status', False)):
            return

        # In ROS 1 there is no live parameter server update via get_parameter(),
        # but rospy.get_param() can be called at runtime to pick up rosparam changes.
        try:
            self.rope_position_kp               = float(rospy.get_param('~rope_position_kp',               self.rope_position_kp))
            self.rope_position_max_vel_m_s      = float(rospy.get_param('~rope_position_max_vel_m_s',      self.rope_position_max_vel_m_s))
            self.rope_position_deadband_m       = float(rospy.get_param('~rope_position_deadband_m',       self.rope_position_deadband_m))
            self.rope_direction_sign            = float(rospy.get_param('~rope_direction_sign',            self.rope_direction_sign))
            self.rope_position_motor_vel_scale  = float(rospy.get_param('~rope_position_motor_vel_scale',  self.rope_position_motor_vel_scale))
            self.rope_position_up_near_vel_m_s  = float(rospy.get_param('~rope_position_up_near_vel_m_s',  self.rope_position_up_near_vel_m_s))
            self.rope_position_up_far_vel_m_s   = float(rospy.get_param('~rope_position_up_far_vel_m_s',   self.rope_position_up_far_vel_m_s))
            self.rope_position_down_near_vel_m_s= float(rospy.get_param('~rope_position_down_near_vel_m_s',self.rope_position_down_near_vel_m_s))
            self.rope_position_down_far_vel_m_s = float(rospy.get_param('~rope_position_down_far_vel_m_s', self.rope_position_down_far_vel_m_s))
            self.rope_position_profile_zone_m   = float(rospy.get_param('~rope_position_profile_zone_m',   self.rope_position_profile_zone_m))
        except Exception as e:
            rospy.logwarn(f'Could not refresh rope position params: {e}')

        actual   = float(getattr(self, 'rope_length_m', 0.0))
        ref      = float(self.rope_position_ref_m)
        error    = ref - actual
        abs_err  = abs(error)

        deadband = abs(float(self.rope_position_deadband_m))
        vmax     = abs(float(self.rope_position_max_vel_m_s))
        zone     = abs(float(self.rope_position_profile_zone_m))

        if abs_err <= deadband:
            vel_cmd_m_s = 0.0
            local_vmax  = 0.0
            phase       = 'stop'
        else:
            if error > 0.0:
                near  = abs(float(self.rope_position_up_near_vel_m_s))
                far   = abs(float(self.rope_position_up_far_vel_m_s))
                phase = 'up_near' if abs_err <= zone else 'up_far'
            else:
                near  = abs(float(self.rope_position_down_near_vel_m_s))
                far   = abs(float(self.rope_position_down_far_vel_m_s))
                phase = 'down_near' if abs_err <= zone else 'down_far'

            local_vmax  = min(vmax, near if abs_err <= zone else far)
            vel_cmd_m_s = float(self.rope_position_kp) * error
            vel_cmd_m_s = max(-local_vmax, min(local_vmax, vel_cmd_m_s))

            if error > 0.0:
                vel_cmd_m_s =  abs(vel_cmd_m_s)
            elif error < 0.0:
                vel_cmd_m_s = -abs(vel_cmd_m_s)

        r0     = float(self.sync_roller_radius_m)
        d      = float(self.rope_diameter_m)
        r_eff  = r0 + 0.5 * d
        two_pi = 2.0 * math.pi

        G = float(self.variable_gear_ratio_g)
        if not (math.isfinite(G) and G > 1e-6):
            G = self.gear_ratio_nominal if self.gear_ratio_nominal > 1e-6 else 1.0

        motor_vel_turns_s  = (vel_cmd_m_s / max(r_eff, 1e-9)) / (two_pi * max(G, 1e-9))
        motor_vel_turns_s *= float(self.rope_direction_sign)
        motor_vel_turns_s *= float(self.rope_position_motor_vel_scale)

        self.send_cmd(f'send_odrive w axis0.controller.input_vel {motor_vel_turns_s:.6f}')

        now = time.time()
        if now - self._last_position_loop_log_time > 0.5:
            self._last_position_loop_log_time = now
            rospy.loginfo(
                f"[rope_pos_loop] ref={ref:.4f} m, actual={actual:.4f} m, "
                f"err={error:.4f} m, kp={self.rope_position_kp:.2f}, "
                f"vmax={vmax:.3f}, zone={zone:.4f}, phase={phase}, "
                f"local_vmax={local_vmax:.3f}, scale={self.rope_position_motor_vel_scale:.2f}, "
                f"sign={self.rope_direction_sign:+.0f}, v_cmd={vel_cmd_m_s:.4f} m/s, "
                f"motor_vel={motor_vel_turns_s:.5f} turn/s"
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Variable gear ratio estimator
    # ─────────────────────────────────────────────────────────────────────────
    def _update_variable_gear_ratio(self):
        if getattr(self, '_freeze_g', True):
            return
        try:
            motor_sp  = float(self.motor_speed)
            roller_sp = float(self.sync_roller_speed)

            if not math.isfinite(motor_sp) or not math.isfinite(roller_sp):
                return
            if bool(getattr(self, 'brake_status', False)):
                return
            if abs(motor_sp)  < self.gear_ratio_motor_eps:
                return
            if abs(roller_sp) < self.gear_ratio_roller_eps:
                return

            G_meas = abs(roller_sp / motor_sp)
            if not math.isfinite(G_meas):
                return
            if G_meas < self.gear_ratio_min or G_meas > self.gear_ratio_max:
                return

            G_old = float(self.variable_gear_ratio_g)
            if not math.isfinite(G_old) or G_old <= 0.0:
                self.variable_gear_ratio_g = G_meas
                return

            if abs(G_meas - G_old) > self.gear_ratio_max_step:
                return

            alpha = max(0.0, min(1.0, float(self.gear_ratio_alpha)))
            self.variable_gear_ratio_g = (1.0 - alpha) * G_old + alpha * G_meas

            self._gear_print_counter += 1
            if self._gear_print_counter >= self.gear_ratio_log_period:
                self._gear_print_counter = 0
                rospy.loginfo(
                    f"[gear] G={self.variable_gear_ratio_g:.3f}, "
                    f"G_meas={G_meas:.3f}, "
                    f"motor_sp={motor_sp:.3f} rev/s, "
                    f"roller_sp={roller_sp:.3f} rev/s"
                )
        except Exception as e:
            rospy.logwarn(f'Gear ratio update failed: {e}')

    # ─────────────────────────────────────────────────────────────────────────
    # Reader / writer threads
    # ─────────────────────────────────────────────────────────────────────────
    def _serial_reader_loop(self) -> None:
        while not self._stop:
            try:
                if not self.ser.in_waiting:
                    time.sleep(0.0005)
                    continue
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors='ignore').strip()
                if not line:
                    continue
                self.process_csv(line)
            except Exception as e:
                rospy.logwarn(f'Serial read error: {e}')
                time.sleep(0.01)

    def _serial_writer_loop(self) -> None:
        while not self._stop:
            try:
                if self.tx_queue and not self.ser.in_waiting:
                    for _ in range(4):
                        if not self.tx_queue or self.ser.in_waiting:
                            break
                        out = self.tx_queue.popleft()
                        self.ser.write(out.encode('utf-8'))
                else:
                    time.sleep(0.0005)
            except Exception as e:
                rospy.logwarn(f'Serial write error: {e}')
                time.sleep(0.01)

    def _safe_int(self, value, default=0):
        try:
            x = float(value)
            if not math.isfinite(x):
                return default
            return int(round(x))
        except Exception:
            return default

    # ─────────────────────────────────────────────────────────────────────────
    # Process CSV
    # ─────────────────────────────────────────────────────────────────────────
    def process_csv(self, line: str) -> None:
        self.pub_csv.publish(String(data=line))

        try:
            row = next(csv.reader([line]))
        except Exception:
            return
        if not row:
            return

        if self.expect_header and not self.header:
            self.header = [h.strip() for h in row]
            self.name_to_idx = {n: i for i, n in enumerate(self.header)}
            return

        def idx(name: str, fallback: int):
            return self.name_to_idx.get(name, fallback)

        def cell(name: str, pos: int):
            i = idx(name, pos)
            return row[i].strip() if 0 <= i < len(row) else None

        t_s = None
        s = cell('epoch_ms', 0)
        if s not in (None, ''):
            try:
                t_s = float(s) / 1_000.0
            except Exception:
                t_s = None

        if t_s is None:
            s = cell('micros', 0)
            if s not in (None, ''):
                try:
                    t_s = float(s) / 1_000_000.0
                except Exception:
                    t_s = None

        if t_s is None:
            t_s = time.time()

        brake_status = cell('brake', 1) in ('1', 'true', 'True', 'TRUE')

        s = cell('ibus', 2)
        try:
            current = float(s) if s not in (None, '') else None
        except Exception:
            current = None

        s = cell('motor_torque', 3)
        try:
            motor_torque = float(s) if s not in (None, '') else None
        except Exception:
            motor_torque = None

        s = cell('syncronous_roller_raw_wrapped', 4)
        try:
            sync_raw = int(float(s)) if s not in (None, '') else None
        except Exception:
            sync_raw = None

        sr = sync_raw
        if sr is None or sr == 0:
            if self._last_valid_sync_raw is not None:
                sr = self._last_valid_sync_raw
        else:
            self._last_valid_sync_raw = sr

        sync_raw = sr
        sync_raw = self.glitch_outlier_filter(
            sync_raw,
            name='sync_raw',
            window=15,
            k=3.5,
            max_step=self.max_roller_rps * self.sync_roller_cpr / self.poll_rate_hz,
            persist=3,
        )

        if sync_raw is not None:
            try:
                if not math.isfinite(float(sync_raw)):
                    sync_raw = self._last_valid_sync_raw
            except Exception:
                sync_raw = self._last_valid_sync_raw

        if sync_raw is not None:
            sync_raw = self._safe_int(sync_raw, default=0)

        s = cell('motor_raw_wrapped', 5)
        try:
            motor_pos_norm = float(s) if s not in (None, '') else None
        except Exception:
            motor_pos_norm = None

        mp = motor_pos_norm
        if mp is None or mp == 0.0:
            if self._last_valid_motor_pos_norm is not None:
                mp = self._last_valid_motor_pos_norm
        else:
            self._last_valid_motor_pos_norm = mp

        motor_pos_norm = self.moving_average_filter(mp, window=25, name='motor_pos')

        fixed_dt        = 1.0 / max(1e-6, self.poll_rate_hz)
        cpr_i           = int(self.sync_roller_cpr)
        cpr             = float(cpr_i)
        radius_m        = float(self.sync_roller_radius_m)
        max_motor_rps   = self.max_motor_rps
        max_roller_rps  = self.max_roller_rps
        count_deadband  = self.count_deadband
        lpf_fc_pos_motor  = self.lpf_fc_pos_motor
        lpf_fc_pos_roller = self.lpf_fc_pos_roller
        brake_zero_rps  = self.brake_zero_rps
        phys_max_rope_m_s = self.phys_max_rope_m_s
        tau2pi          = 2.0 * math.pi
        r_eff           = radius_m + 0.5 * float(self.rope_diameter_m)

        def lpf_pos(prev_val, x, dt, fc):
            if x is None:
                return prev_val
            if prev_val is None or fc <= 0 or dt <= 0:
                return x if prev_val is None else prev_val
            tau = 1.0 / (2.0 * math.pi * fc)
            a   = dt / (tau + dt)
            return prev_val + a * (x - prev_val)

        # Unwrap motor
        motor_unwrapped_rev = self._motor_unwrapped_rev
        if motor_pos_norm is not None:
            if motor_unwrapped_rev is None:
                motor_unwrapped_rev = float(motor_pos_norm)
            else:
                frac_prev = motor_unwrapped_rev - math.floor(motor_unwrapped_rev)
                dm = motor_pos_norm - frac_prev
                if dm > 0.5:
                    dm -= 1.0
                elif dm < -0.5:
                    dm += 1.0
                motor_unwrapped_rev = motor_unwrapped_rev + dm

        if motor_unwrapped_rev is None:
            motor_unwrapped_rev = self._motor_unwrapped_rev
        self._motor_unwrapped_rev = motor_unwrapped_rev

        # Unwrap roller
        roller_unwrapped_counts = self._roller_unwrapped_counts
        if sync_raw is not None:
            if roller_unwrapped_counts is None:
                roller_unwrapped_counts = float(sync_raw)
            else:
                prev_wrap = int(round(roller_unwrapped_counts)) % cpr_i
                dr = int(sync_raw) - prev_wrap
                half = cpr / 2.0
                if dr > half:
                    dr -= cpr_i
                elif dr < -half:
                    dr += cpr_i
                if abs(dr) < count_deadband:
                    dr = 0
                roller_unwrapped_counts = roller_unwrapped_counts + float(dr)

        if roller_unwrapped_counts is None:
            roller_unwrapped_counts = self._roller_unwrapped_counts
        self._roller_unwrapped_counts = roller_unwrapped_counts

        # LPF positions
        if not hasattr(self, '_pos_motor_filt'):
            self._pos_motor_filt = motor_unwrapped_rev
        else:
            self._pos_motor_filt = lpf_pos(self._pos_motor_filt, motor_unwrapped_rev, fixed_dt, lpf_fc_pos_motor)

        if not hasattr(self, '_pos_roller_filt'):
            self._pos_roller_filt = roller_unwrapped_counts
        else:
            self._pos_roller_filt = lpf_pos(self._pos_roller_filt, roller_unwrapped_counts, fixed_dt, lpf_fc_pos_roller)

        self._hist_pos_motor_filt.append(self._pos_motor_filt)
        self._hist_pos_roller_filt.append(self._pos_roller_filt)

        # Windowed derivative
        motor_sp  = float(self.motor_speed)
        roller_sp = float(self.sync_roller_speed)

        if len(self._hist_pos_motor_filt) >= 2:
            k   = min(5, len(self._hist_pos_motor_filt) - 1)
            dtw = k * fixed_dt

            p_now = self._hist_pos_motor_filt[-1]
            p_old = self._hist_pos_motor_filt[-1 - k]
            if p_now is not None and p_old is not None and dtw > 0:
                cand = (p_now - p_old) / dtw
                if abs(cand) <= max_motor_rps * 1.5:
                    motor_sp = cand

            r_now = self._hist_pos_roller_filt[-1]
            r_old = self._hist_pos_roller_filt[-1 - k]
            if r_now is not None and r_old is not None and dtw > 0:
                dr_counts = r_now - r_old
                cand = (dr_counts / cpr) / dtw
                if abs(cand) <= max_roller_rps * 1.5:
                    roller_sp = cand

        # Brake gating
        if brake_status and abs(motor_sp) < brake_zero_rps and abs(roller_sp) < brake_zero_rps:
            motor_sp  = 0.0
            roller_sp = 0.0
            self._pos_motor_filt  = motor_unwrapped_rev
            self._pos_roller_filt = roller_unwrapped_counts

        self.motor_speed       = float(motor_sp)
        self.sync_roller_speed = float(roller_sp)

        self.brake_status                  = bool(brake_status)
        self.current                       = current
        self.motor_torque                  = motor_torque
        self.tau_motor                     = self.motor_torque
        self.syncronous_roller_raw_wrapped = sync_raw
        self.motor_position                = motor_pos_norm

        motor_speed_rad_s  = self.motor_speed  * tau2pi
        roller_speed_rad_s = self.sync_roller_speed * tau2pi
        rope_speed_m_s     = roller_speed_rad_s * r_eff

        if abs(rope_speed_m_s) > phys_max_rope_m_s:
            rope_speed_m_s     = 0.0
            roller_speed_rad_s = 0.0
            self.sync_roller_speed = 0.0

        self._update_variable_gear_ratio()

        # Publish DebugMessage
        def to_f(x):
            try:
                return float(x)
            except Exception:
                return float('nan')

        m = DebugMessage()
        m.header.stamp                    = rospy.Time.now()
        m.header.frame_id                 = f'winch_{self.side}'
        m.brake                           = bool(self.brake_status)
        m.current                         = to_f(self.current) if self.current is not None else float('nan')
        m.motor_torque                    = to_f(self.motor_torque) if self.motor_torque is not None else float('nan')
        m.syncronous_roller_raw_wrapped   = self._safe_int(self.syncronous_roller_raw_wrapped, default=0)
        m.motor_position                  = to_f(self.motor_position) if self.motor_position is not None else float('nan')
        m.motor_speed_rev_s               = to_f(self.motor_speed)
        m.motor_speed_rad_s               = to_f(motor_speed_rad_s)
        m.sync_roller_speed_rev_s         = to_f(self.sync_roller_speed)
        m.sync_roller_speed_rad_s         = to_f(roller_speed_rad_s)
        m.rope_speed_m_s                  = to_f(rope_speed_m_s)
        self.pub_debug.publish(m)

        # Rope length from roller
        # Rope length from roller, fallback from motor if roller is not changing
        rope_length_m = getattr(self, 'rope_length_m', 0.0)

        roller_ok = getattr(self, '_pos_roller_filt', None) is not None
        motor_ok = getattr(self, '_pos_motor_filt', None) is not None

        if roller_ok:
            zero = getattr(self, '_roller_counts_zero', None)
            if zero is None:
                self._roller_counts_zero = float(self._pos_roller_filt)
                zero = self._roller_counts_zero

            counts_rel = float(self._pos_roller_filt) - float(zero)
            rope_length_from_roller = (counts_rel / cpr) * tau2pi * r_eff

            # If the roller actually moved, trust it.
            # If it is stuck at zero, fall back to motor.
            if abs(counts_rel) > 2.0:
                rope_length_m = rope_length_from_roller

        if abs(float(rope_length_m)) < 1e-6 and motor_ok:
            motor_zero = getattr(self, '_motor_rev_zero', None)
            if motor_zero is None:
                self._motor_rev_zero = float(self._pos_motor_filt)
                motor_zero = self._motor_rev_zero

            motor_rel_rev = float(self._pos_motor_filt) - float(motor_zero)

            G = float(self.variable_gear_ratio_g)
            if not math.isfinite(G) or G <= 1e-9:
                G = self.gear_ratio_nominal if self.gear_ratio_nominal > 1e-9 else 1.0

            # Estimate rope length from motor turns.
            # Positive rope_length must correspond to positive rope_position command.
            motor_to_rope_sign = float(self.rope_direction_sign)

            rope_length_from_motor = (
                    motor_rel_rev
                    * motor_to_rope_sign
                    * tau2pi
                    * r_eff
                    * G
            )

            rope_length_m = rope_length_from_motor

        self.rope_length_m = float(rope_length_m)

        # Rope force
        G = float(self.variable_gear_ratio_g)
        if not math.isfinite(G) or G <= 1e-9:
            G = self.gear_ratio_nominal if self.gear_ratio_nominal > 1e-9 else 1.0

        den        = max(r_eff * G, 1e-9)
        rope_force = float('nan')
        if self.tau_motor is not None and math.isfinite(self.tau_motor):
            rope_force = float(self.tau_motor) / den

        # Publish RopeTelemetry
        rt = RopeTelemetry()
        rt.header.stamp  = rospy.Time.now()
        rt.rope_force    = to_f(rope_force)
        rt.rope_length   = to_f(rope_length_m)
        rt.rope_velocity = to_f(rope_speed_m_s)
        rt.current       = to_f(self.current) if self.current is not None else float('nan')
        rt.brake_status  = bool(self.brake_status)
        self.pub_rope.publish(rt)

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _load_config(self, path: str) -> Optional[dict]:
        try:
            p = Path(path)
            if not p.exists():
                alt = Path(__file__).resolve().parent.parent / 'config' / 'arganelloTelemetry.json'
                p = alt if alt.exists() else p
            if not p.exists():
                return None
            with p.open('r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f'Failed to load CONFIG: {e}')
            return None

    def _stdin_loop(self) -> None:
        import sys
        import select

        node_name = rospy.get_name()
        prompt = f'[{node_name}:{self.side}]> '
        sys.stdout.write(prompt)
        sys.stdout.flush()

        while not self._stop:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            line = sys.stdin.readline()
            if not line:
                continue
            line = line.strip()
            if not line:
                sys.stdout.write(prompt)
                sys.stdout.flush()
                continue
            if line in ('quit', 'exit'):
                rospy.loginfo('Exiting debug console…')
                break
            self.send_cmd(line)
            sys.stdout.write(prompt)
            sys.stdout.flush()

    def _send_sync(self):
        if self.sync_epoch_unit == 'ns':
            epoch = time.time_ns()
        else:
            epoch = int(time.time_ns() // 1_000_000)
        cmd = f'sync {epoch}'
        rospy.loginfo(f'→ {cmd}')
        self.send_cmd(cmd)
        return epoch

    def shutdown(self):
        self._stop = True
        time.sleep(0.02)
        try:
            self.ser.close()
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Filter helpers  (unchanged logic)
    # ─────────────────────────────────────────────────────────────────────────
    def moving_average_filter(self, value, window=5, name='default'):
        hist_name = f'_ma_hist_{name}'
        last_name = f'_ma_last_{name}'

        if not hasattr(self, hist_name):
            setattr(self, hist_name, deque(maxlen=max(1, int(window))))
        if not hasattr(self, last_name):
            setattr(self, last_name, None)

        hist = getattr(self, hist_name)
        last = getattr(self, last_name)

        if window < 1:
            return value
        if value is None:
            return last

        hist.append(float(value))
        avg = sum(hist) / len(hist)
        filtered = int(round(avg)) if isinstance(value, int) else avg
        setattr(self, last_name, filtered)
        return filtered

    def glitch_outlier_filter(self, value, *, name='enc', window=15, k=3.5,
                               max_step=None, persist=2, tol_accept=None):
        hist_name = f'_gf_hist_{name}'
        last_name = f'_gf_last_{name}'
        pend_name = f'_gf_pending_{name}'
        is_int    = isinstance(value, int)

        if value is None:
            return getattr(self, last_name, None)

        x = float(value)
        if not math.isfinite(x):
            return getattr(self, last_name, None)

        if not hasattr(self, hist_name):
            setattr(self, hist_name, deque(maxlen=max(3, window)))

        hist = getattr(self, hist_name)

        if not hasattr(self, last_name):
            setattr(self, last_name, x)
        if not hasattr(self, pend_name):
            setattr(self, pend_name, (None, 0))

        last = float(getattr(self, last_name))
        pending_val, pending_cnt = getattr(self, pend_name)

        def med_mad(values):
            vals = list(values)
            if not vals:
                return x, 0.0
            vs  = sorted(vals)
            n   = len(vs)
            med = vs[n // 2] if n % 2 == 1 else 0.5 * (vs[n // 2 - 1] + vs[n // 2])
            abs_dev = sorted([abs(v - med) for v in vs])
            mad = abs_dev[n // 2] if n % 2 == 1 else 0.5 * (abs_dev[n // 2 - 1] + abs_dev[n // 2])
            return med, 1.4826 * (mad if mad > 0 else 0.0)

        if len(hist) < 3:
            hist.append(x)
            setattr(self, last_name, x)
            return int(round(x)) if is_int else x

        med, s = med_mad(hist)

        if tol_accept is None:
            tol_accept = max(0.01 * max(abs(med), 1.0), 1.0)

        is_hampel_outlier = abs(x - med) > k * max(s, 1e-9)
        is_step_outlier   = (max_step is not None) and (abs(x - last) > max_step)

        if not (is_hampel_outlier or is_step_outlier):
            hist.append(x)
            setattr(self, pend_name, (None, 0))
            setattr(self, last_name, x)
            return int(round(x)) if is_int else x

        if pending_val is None:
            setattr(self, pend_name, (x, 1))
            return int(round(last)) if is_int else last

        if abs(x - pending_val) <= tol_accept:
            pending_cnt += 1
            setattr(self, pend_name, (pending_val, pending_cnt))
        else:
            setattr(self, pend_name, (x, 1))
            pending_cnt = 1
            pending_val = x

        if pending_cnt >= persist:
            hist.append(pending_val)
            setattr(self, last_name, pending_val)
            setattr(self, pend_name, (None, 0))
            return int(round(pending_val)) if is_int else pending_val

        return int(round(last)) if is_int else last


def main():
    rospy.init_node('telemetry_node')
    node = TelemetryNode()

    rospy.on_shutdown(node.shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down…')


if __name__ == '__main__':
    main()