#!/usr/bin/env python3

import math
import threading
from collections import deque

import rospy

from std_srvs.srv import Trigger, TriggerResponse
from climbingrobot_hardware_interface.srv import AlpineBodyCommand, AlpineBodyCommandResponse
from std_msgs.msg import Float32, String
from climbingrobot_hardware_interface.msg import RopeCommand, AlpineBodyTelemetry


class JumpNode:
    """
    Low-level jump interface for ALPINE.

    Two different entry points are intentionally kept separated:

    1) /alpine/jump
       Manual standalone test.
       jump.py commands valves + local hardcoded rope forces.
       In this mode ropes start after the piston thrust phase.

    2) /alpine_body/command
       Called by climbingrobot_controller2_real.py.
       This is the optimized-control path:
         - leg_force = Fleg, single impulse-like command
         - Fr_l(t), Fr_r(t) must be published elsewhere as RopeCommand.rope_force
       In this mode jump.py commands ONLY the body valves and never publishes rope commands.
    """

    def __init__(self):
        # ── Publishers ───────────────────────────────────────────────────
        self.pub_s1 = rospy.Publisher(
            '/alpine/dongle/servoValve1',
            Float32,
            queue_size=10
        )
        self.pub_s2 = rospy.Publisher(
            '/alpine/dongle/servoValve2',
            Float32,
            queue_size=10
        )

        self.pub_left = rospy.Publisher(
            '/winch/left/command',
            RopeCommand,
            queue_size=10
        )
        self.pub_right = rospy.Publisher(
            '/winch/right/command',
            RopeCommand,
            queue_size=10
        )

        self.pub_left_mode = rospy.Publisher(
            '/winch/left/set_motor_mode',
            String,
            queue_size=10
        )
        self.pub_right_mode = rospy.Publisher(
            '/winch/right/set_motor_mode',
            String,
            queue_size=10
        )

        # ── Services ─────────────────────────────────────────────────────
        rospy.Service('/alpine/jump', Trigger, self.handle_jump)
        rospy.Service('/alpine/jump_abort', Trigger, self.handle_abort)
        rospy.Service('/alpine/jump_stop', Trigger, self.handle_stop)

        rospy.Service(
            '/alpine_body/command',
            AlpineBodyCommand,
            self.handle_alpine_body_command
        )
        
        # Direct inlet-valve calibration mode.
        # Publishes an inlet angle directly, bypassing Fleg -> pressure mapping.
        # Used to collect data: inlet_deg -> IMU delta_v_x.
        rospy.Subscriber(
            '/alpine_body/calib_inlet_deg',
            Float32,
            self.handle_calib_inlet_deg,
            queue_size=10
        )


        # Desired impulse command.
        # Input: J_des [N*s]
        # Mapping:
        #   J_des -> delta_v_des = J_des / robot_mass_kg
        #   delta_v_des -> inlet_deg using experimental lookup table
        rospy.Subscriber(
            '/alpine_body/jump_impulse_des',
            Float32,
            self.handle_jump_impulse_des,
            queue_size=10
        )
        
        
        # ─────────────────────────────────────────────────────────────────
        # Manual safe jump parameters.
        # Used only when calling /alpine/jump manually.
        # ─────────────────────────────────────────────────────────────────
        self.manual_up_force = rospy.get_param('~manual_up_force', 25.0)
        self.manual_mid_force = rospy.get_param('~manual_mid_force', 18.0)
        self.manual_hold_force = rospy.get_param('~manual_hold_force', 15.0)

        self.manual_phase1_ms = rospy.get_param('~manual_phase1_ms', 220.0)
        self.manual_phase2_ms = rospy.get_param('~manual_phase2_ms', 650.0)
        self.manual_phase3_ms = rospy.get_param('~manual_phase3_ms', 400.0)
        self.manual_phase4_ms = rospy.get_param('~manual_phase4_ms', 1500.0)

        # New manual rope timing:
        # If true, the manual piston phase uses body_thrust_ms and ropes start after it.
        self.manual_rope_start_after_thrust = rospy.get_param(
            '~manual_rope_start_after_thrust',
            True
        )
        self.manual_rope_extra_delay_ms = float(rospy.get_param(
            '~manual_rope_extra_delay_ms',
            0.0
        ))

        # ─────────────────────────────────────────────────────────────────
        # Fleg -> pressure -> valve feedforward parameters.
        #
        # P_des = Fleg / A_piston
        #
        # Without pressure sensor feedback this is open-loop:
        # P_des / P_supply is mapped to a servo-valve opening.
        # ─────────────────────────────────────────────────────────────────
        self.piston_diameter_m = rospy.get_param('~piston_diameter_m', 0.032)
        self.piston_area_m2 = math.pi * (0.5 * self.piston_diameter_m) ** 2

        self.supply_pressure_bar = rospy.get_param('~supply_pressure_bar', 4.5)
        self.supply_pressure_pa = self.supply_pressure_bar * 1e5

        self.body_thrust_ms = rospy.get_param('~body_thrust_ms', 50.0)

        self.body_inlet_min_deg = rospy.get_param('~body_inlet_min_deg', 0.0)
        self.body_inlet_max_deg = rospy.get_param('~body_inlet_max_deg', 90.0)

        self.body_exhaust_ms = rospy.get_param('~body_exhaust_ms', 900.0)
        self.body_exhaust_opening_deg = rospy.get_param('~body_exhaust_opening_deg', 90.0)

        self.body_final_close_ms = rospy.get_param('~body_final_close_ms', 200.0)

        self.min_valid_fleg_n = rospy.get_param('~min_valid_fleg_n', 1.0)

        # ─────────────────────────────────────────────────────────────────
        # IMU diagnostic / calibration.
        # This is NOT a closed-loop controller.
        # It only estimates delta_v_x after each jump.
        # ─────────────────────────────────────────────────────────────────
        self.imu_calib_enabled = rospy.get_param('~imu_calib_enabled', True)
        self.imu_baseline_window_s = float(rospy.get_param('~imu_baseline_window_s', 0.50))
        self.imu_post_window_s = float(rospy.get_param('~imu_post_window_s', 0.25))
        self.imu_valve_threshold_deg = float(rospy.get_param('~imu_valve_threshold_deg', 1.0))


        # ─────────────────────────────────────────────────────────────────
        # Experimental J_des -> inlet_deg lookup.
        #
        # Point B:
        #   desired impulse -> desired delta_v -> inlet valve opening.
        #
        # J_des [N*s] = m_robot [kg] * delta_v_des [m/s]
        # delta_v_des is mapped to inlet_deg by a piecewise-linear lookup.
        # ─────────────────────────────────────────────────────────────────
        self.robot_mass_kg = float(rospy.get_param('~robot_mass_kg', 5.0))

        self.jreal_delta_v_table = list(rospy.get_param(
            '~jreal_delta_v_table',
            [0.25, 1.45, 1.75, 1.85]
        ))

        self.jreal_inlet_deg_table = list(rospy.get_param(
            '~jreal_inlet_deg_table',
            [20.0, 30.0, 40.0, 90.0]
        ))

        self.jreal_min_impulse_ns = float(rospy.get_param('~jreal_min_impulse_ns', 0.0))
        self.jreal_max_inlet_deg = float(rospy.get_param('~jreal_max_inlet_deg', 90.0))

        self._imu_lock = threading.Lock()
        self._imu_samples = deque(maxlen=20000)

        self.imu_sub = rospy.Subscriber(
            '/alpine_body/telemetry',
            AlpineBodyTelemetry,
            self._body_telemetry_cb,
            queue_size=500
        )

        # State used to detect valve1 open interval online.
        self._imu_valve1_active = False
        self._imu_valve1_start_s = None
        self._imu_valve1_end_s = None
        self._imu_fleg_cmd_n = float('nan')
        self._imu_inlet_deg = float('nan')
        self._imu_sequence_name = 'none'

        # ── State ────────────────────────────────────────────────────────
        self.sequence_running = False
        self.sequence_start_ms = 0.0
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None
        self.current_mode = None

        self.command_ropes_during_sequence = True

        # Active sequence entries:
        # (duration_ms, mode, left_force, right_force, left_velocity, right_velocity, servo1, servo2)
        self.active_timeline = []

        # Debug values for last Fleg mapping.
        self.last_body_leg_force_n = float('nan')
        self.last_body_pressure_pa = float('nan')
        self.last_body_pressure_bar = float('nan')
        self.last_body_pressure_ratio = float('nan')
        self.last_body_inlet_opening_deg = float('nan')

        self.manual_sequence = self.build_manual_sequence()
        self.active_timeline = self.build_timeline(self.manual_sequence)

        # ── Timer: 100 Hz state machine ──────────────────────────────────
        self.timer = rospy.Timer(rospy.Duration(0.01), self.tick)

        rospy.logwarn(
            "jump_node started: no valve/winch command is sent until "
            "/alpine/jump or /alpine_body/command is called"
        )
        rospy.loginfo(
            "Manual jump: up=%.3f N, mid=%.3f N, hold=%.3f N",
            self.manual_up_force,
            self.manual_mid_force,
            self.manual_hold_force,
        )
        rospy.loginfo(
            "Manual rope timing: start_after_thrust=%s, extra_delay=%.1f ms",
            str(self.manual_rope_start_after_thrust),
            self.manual_rope_extra_delay_ms,
        )
        rospy.loginfo(
            "Fleg mapping: piston_diameter=%.4f m, area=%.6e m^2, "
            "supply=%.3f bar, thrust=%.1f ms, inlet=[%.1f, %.1f] deg",
            self.piston_diameter_m,
            self.piston_area_m2,
            self.supply_pressure_bar,
            self.body_thrust_ms,
            self.body_inlet_min_deg,
            self.body_inlet_max_deg,
        )
        rospy.loginfo(
            "IMU calibration: enabled=%s, baseline=%.2f s, post_window=%.2f s",
            str(self.imu_calib_enabled),
            self.imu_baseline_window_s,
            self.imu_post_window_s,
        )

        rospy.loginfo(
            "J_des lookup: mass=%.3f kg, delta_v_table=%s, inlet_table=%s",
            self.robot_mass_kg,
            str(self.jreal_delta_v_table),
            str(self.jreal_inlet_deg_table),
        )

    # ────────────────────────────────────────────────────────────────────
    # Generic helpers
    # ────────────────────────────────────────────────────────────────────

    def now_ms(self) -> float:
        return rospy.Time.now().to_sec() * 1000.0

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def safe_round(x, ndigits: int = 3):
        try:
            xf = float(x)
        except Exception:
            return str(x)

        if math.isnan(xf):
            return 'nan'
        return round(xf, ndigits)

    @staticmethod
    def _trapz(t, y):
        if len(t) < 2:
            return 0.0

        acc = 0.0
        for i in range(1, len(t)):
            dt = t[i] - t[i - 1]
            if dt < 0.0:
                continue
            acc += 0.5 * (y[i] + y[i - 1]) * dt
        return acc

    def build_timeline(self, sequence):
        timeline = []
        acc = 0.0

        for dur, mode, lf, rf, lv, rv, s1, s2 in sequence:
            acc += float(dur)
            timeline.append((acc, mode, lf, rf, lv, rv, s1, s2))

        return timeline

    # ────────────────────────────────────────────────────────────────────
    # IMU diagnostic helpers
    # ────────────────────────────────────────────────────────────────────

    def _body_telemetry_cb(self, msg):
        if not self.imu_calib_enabled:
            return

        try:
            t = msg.header.stamp.to_sec()
            if t <= 0.0:
                t = rospy.Time.now().to_sec()

            ax = float(msg.body_imu_acceleration.x)
            ay = float(msg.body_imu_acceleration.y)
            az = float(msg.body_imu_acceleration.z)

            with self._imu_lock:
                self._imu_samples.append((t, ax, ay, az))

        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "[IMU_CALIB] telemetry callback error: %s",
                str(e)
            )

    def _reset_imu_sequence_state(self, fleg_cmd_n, inlet_deg, sequence_name):
        self._imu_valve1_active = False
        self._imu_valve1_start_s = None
        self._imu_valve1_end_s = None
        self._imu_fleg_cmd_n = float(fleg_cmd_n)
        self._imu_inlet_deg = float(inlet_deg)
        self._imu_sequence_name = str(sequence_name)

    def _fleg_equivalent_from_inlet_deg(self, inlet_deg):
        """
        Estimate equivalent Fleg from an inlet valve angle using the same open-loop mapping.
        Useful only for manual jump diagnostics.
        """
        span = self.body_inlet_max_deg - self.body_inlet_min_deg
        if abs(span) < 1e-9:
            return 0.0

        ratio = (float(inlet_deg) - self.body_inlet_min_deg) / span
        ratio = self.clamp(ratio, 0.0, 1.0)

        p_equiv_pa = ratio * self.supply_pressure_pa
        f_equiv_n = p_equiv_pa * self.piston_area_m2
        return f_equiv_n

    def _update_imu_valve_tracking(self, s1_deg):
        """
        Detect servoValve1 open/close from the actual sequence commands.
        When the valve closes, schedule IMU metric computation after imu_post_window_s.
        """
        if not self.imu_calib_enabled or not self.sequence_running:
            return

        now_s = rospy.Time.now().to_sec()
        is_open = float(s1_deg) > self.imu_valve_threshold_deg

        if is_open and not self._imu_valve1_active:
            self._imu_valve1_active = True
            self._imu_valve1_start_s = now_s
            self._imu_valve1_end_s = None

            rospy.loginfo(
                "[IMU_CALIB] valve1 opened: sequence=%s, t=%.3f, inlet=%.1f deg",
                self._imu_sequence_name,
                self._imu_valve1_start_s,
                float(s1_deg),
            )

        elif (not is_open) and self._imu_valve1_active:
            self._imu_valve1_active = False
            self._imu_valve1_end_s = now_s

            t_start = self._imu_valve1_start_s
            t_end = self._imu_valve1_end_s
            fleg = self._imu_fleg_cmd_n
            inlet = self._imu_inlet_deg
            seq_name = self._imu_sequence_name

            rospy.loginfo(
                "[IMU_CALIB] valve1 closed: sequence=%s, t=%.3f, duration=%.3f s",
                seq_name,
                t_end,
                t_end - t_start if t_start is not None else -1.0,
            )

            rospy.Timer(
                rospy.Duration(self.imu_post_window_s + 0.05),
                lambda event: self._compute_imu_jump_metrics(
                    t_start,
                    t_end,
                    fleg,
                    inlet,
                    seq_name
                ),
                oneshot=True
            )

    def _compute_imu_jump_metrics(self, t_start, t_end, fleg_cmd, inlet_deg, sequence_name):
        """
        Compute delta_v_x from body_imu_acceleration.x.

        Windows:
          - open_only: servoValve1 open interval
          - open_plus: servoValve1 open interval + imu_post_window_s

        This is diagnostic only. It does not change the valve command.
        """
        if not self.imu_calib_enabled:
            return

        if t_start is None or t_end is None or t_end <= t_start:
            rospy.logwarn("[IMU_CALIB] invalid valve timing, cannot compute metrics.")
            return

        with self._imu_lock:
            samples = list(self._imu_samples)

        if len(samples) < 5:
            rospy.logwarn("[IMU_CALIB] not enough IMU samples.")
            return

        base_t0 = t_start - self.imu_baseline_window_s
        base = [s for s in samples if base_t0 <= s[0] < t_start]

        if len(base) < 3:
            rospy.logwarn(
                "[IMU_CALIB] not enough baseline samples: got %d",
                len(base)
            )
            return

        bx = sum(s[1] for s in base) / len(base)
        by = sum(s[2] for s in base) / len(base)
        bz = sum(s[3] for s in base) / len(base)

        def compute_window(w0, w1):
            win = [s for s in samples if w0 <= s[0] <= w1]
            if len(win) < 2:
                return None

            tt = [s[0] for s in win]
            ax = [s[1] - bx for s in win]
            ay = [s[2] - by for s in win]
            az = [s[3] - bz for s in win]

            dvx = self._trapz(tt, ax)
            dvy = self._trapz(tt, ay)
            dvz = self._trapz(tt, az)

            mag = []
            for i in range(len(ax)):
                mag.append(math.sqrt(ax[i] ** 2 + ay[i] ** 2 + az[i] ** 2))

            return {
                'n': len(win),
                'dvx': dvx,
                'dvy': dvy,
                'dvz': dvz,
                'dvmag': self._trapz(tt, mag),
                'ax_mean': sum(ax) / len(ax),
                'ax_peak': max(ax),
                'ax_min': min(ax),
            }

        open_only = compute_window(t_start, t_end)
        open_plus = compute_window(t_start, t_end + self.imu_post_window_s)

        if open_only is None or open_plus is None:
            rospy.logwarn("[IMU_CALIB] could not compute all IMU windows.")
            return

        rospy.logwarn(
            "[IMU_CALIB] sequence=%s, Fleg=%.1f N, inlet=%.1f deg, "
            "valve_open=%.1f ms, baseline_ax=%.4f, "
            "dvx_open=%.4f m/s, ax_peak_open=%.4f m/s^2, "
            "dvx_plus_%.2fs=%.4f m/s, ax_peak_plus=%.4f m/s^2, "
            "samples_open=%d, samples_plus=%d",
            sequence_name,
            float(fleg_cmd),
            float(inlet_deg),
            1000.0 * (t_end - t_start),
            bx,
            open_only['dvx'],
            open_only['ax_peak'],
            self.imu_post_window_s,
            open_plus['dvx'],
            open_plus['ax_peak'],
            open_only['n'],
            open_plus['n'],
        )


    # ────────────────────────────────────────────────────────────────────
    # J_des -> inlet lookup helpers
    # ────────────────────────────────────────────────────────────────────

    def lookup_inlet_from_delta_v(self, delta_v_des):
        """
        Experimental inverse map:

            delta_v_des -> inlet_deg

        Uses piecewise-linear interpolation on the calibrated table.
        Values outside the table are saturated.

        The table should be monotonic in delta_v. If the raw experimental
        data is not monotonic because of pneumatic pressure drift/noise, use
        a conservative monotonic table in YAML.
        """
        try:
            dv = float(delta_v_des)
        except Exception:
            rospy.logwarn("[J_DES] Invalid delta_v_des. Using 0.")
            dv = 0.0

        if not math.isfinite(dv):
            rospy.logwarn("[J_DES] Non-finite delta_v_des. Using 0.")
            dv = 0.0

        dv_table = [float(x) for x in self.jreal_delta_v_table]
        inlet_table = [float(x) for x in self.jreal_inlet_deg_table]

        if len(dv_table) != len(inlet_table) or len(dv_table) < 2:
            rospy.logwarn("[J_DES] Invalid lookup table. Falling back to inlet=0.")
            return 0.0

        pairs = sorted(zip(dv_table, inlet_table), key=lambda p: p[0])
        dv_table = [p[0] for p in pairs]
        inlet_table = [p[1] for p in pairs]

        if dv <= dv_table[0]:
            return self.clamp(
                inlet_table[0],
                self.body_inlet_min_deg,
                self.jreal_max_inlet_deg,
            )

        if dv >= dv_table[-1]:
            return self.clamp(
                inlet_table[-1],
                self.body_inlet_min_deg,
                self.jreal_max_inlet_deg,
            )

        for i in range(len(dv_table) - 1):
            dv0 = dv_table[i]
            dv1 = dv_table[i + 1]

            if dv0 <= dv <= dv1:
                inlet0 = inlet_table[i]
                inlet1 = inlet_table[i + 1]

                if abs(dv1 - dv0) < 1e-9:
                    inlet = inlet0
                else:
                    alpha = (dv - dv0) / (dv1 - dv0)
                    inlet = inlet0 + alpha * (inlet1 - inlet0)

                return self.clamp(
                    inlet,
                    self.body_inlet_min_deg,
                    self.jreal_max_inlet_deg,
                )

        return self.clamp(
            inlet_table[-1],
            self.body_inlet_min_deg,
            self.jreal_max_inlet_deg,
        )

    def lookup_inlet_from_impulse(self, impulse_des_ns):
        """
        Convert desired impulse J_des [N*s] to inlet angle.

            J_des = m_robot * delta_v_des
            delta_v_des = J_des / m_robot
            inlet_deg = lookup(delta_v_des)
        """
        try:
            j_des = float(impulse_des_ns)
        except Exception:
            rospy.logwarn("[J_DES] Invalid J_des. Using 0.")
            j_des = 0.0

        if not math.isfinite(j_des):
            rospy.logwarn("[J_DES] Non-finite J_des. Using 0.")
            j_des = 0.0

        j_des = max(j_des, self.jreal_min_impulse_ns)

        if self.robot_mass_kg <= 1e-6:
            rospy.logwarn(
                "[J_DES] Invalid robot_mass_kg=%.3f. Falling back to inlet=0.",
                self.robot_mass_kg,
            )
            return 0.0, 0.0

        delta_v_des = j_des / self.robot_mass_kg
        inlet_deg = self.lookup_inlet_from_delta_v(delta_v_des)

        return inlet_deg, delta_v_des

    # ────────────────────────────────────────────────────────────────────
    # Sequence builders
    # ────────────────────────────────────────────────────────────────────

    def build_manual_sequence(self):
        """
        Local standalone test sequence.

        This is NOT the optimized-control path.
        It is only useful to test the hardware without the high-level controller.

        Important:
        In manual mode, ropes start AFTER the piston thrust phase.
        """
        nan = float('nan')

        # Use calibrated body_thrust_ms for manual piston thrust too,
        # if manual_rope_start_after_thrust is enabled.
        if self.manual_rope_start_after_thrust:
            piston_thrust_ms = float(self.body_thrust_ms)
        else:
            piston_thrust_ms = float(self.manual_phase1_ms)

        seq = []

        # Phase 1: piston thrust only, ropes idle.
        seq.append((
            piston_thrust_ms,
            'torque',
            0.0,
            0.0,
            nan,
            nan,
            90.0,
            0.0,
        ))

        # Optional delay after thrust before ropes pull.
        # During this delay, ropes remain idle and valves are closed.
        if self.manual_rope_start_after_thrust and self.manual_rope_extra_delay_ms > 0.0:
            seq.append((
                self.manual_rope_extra_delay_ms,
                'torque',
                0.0,
                0.0,
                nan,
                nan,
                0.0,
                0.0,
            ))

        # Ropes pull while airborne; exhaust opens.
        seq.append((
            self.manual_phase2_ms,
            'torque',
            -abs(self.manual_up_force),
            -abs(self.manual_up_force),
            nan,
            nan,
            0.0,
            self.body_exhaust_opening_deg,
        ))

        # Lower pull, exhaust still open.
        seq.append((
            self.manual_phase3_ms,
            'torque',
            -abs(self.manual_mid_force),
            -abs(self.manual_mid_force),
            nan,
            nan,
            0.0,
            self.body_exhaust_opening_deg,
        ))

        # Final hold, valves closed.
        seq.append((
            self.manual_phase4_ms,
            'torque',
            -abs(self.manual_hold_force),
            -abs(self.manual_hold_force),
            nan,
            nan,
            0.0,
            0.0,
        ))

        return seq

    def build_body_sequence_from_fleg(self, leg_force_n):
        """
        Optimized jump path.

        The high-level controller gives Fleg once through /alpine_body/command.

        Here we map:

            Fleg [N] -> P_des [Pa] = Fleg / piston_area

        Then, since the current low-level interface commands only servo angle,
        we map:

            P_des / P_supply -> inlet valve opening

        This is open-loop pressure feedforward. It is NOT a real pressure loop.
        Rope commands are NOT published in this mode.
        """
        nan = float('nan')

        try:
            fleg = abs(float(leg_force_n))
        except Exception:
            rospy.logwarn("Invalid Fleg received. Falling back to zero.")
            fleg = 0.0

        if not math.isfinite(fleg):
            rospy.logwarn("Non-finite Fleg received. Falling back to zero.")
            fleg = 0.0

        if fleg < self.min_valid_fleg_n:
            p_des_pa = 0.0
            ratio = 0.0
            inlet_opening = 0.0
        else:
            p_des_pa = fleg / self.piston_area_m2

            if self.supply_pressure_pa <= 1e-6:
                rospy.logwarn("Invalid supply pressure. Saturating pressure ratio to 1.0.")
                ratio = 1.0
            else:
                ratio = self.clamp(p_des_pa / self.supply_pressure_pa, 0.0, 1.0)

            inlet_opening = self.body_inlet_min_deg + ratio * (
                self.body_inlet_max_deg - self.body_inlet_min_deg
            )

        self.last_body_leg_force_n = fleg
        self.last_body_pressure_pa = p_des_pa
        self.last_body_pressure_bar = p_des_pa / 1e5
        self.last_body_pressure_ratio = ratio
        self.last_body_inlet_opening_deg = inlet_opening

        if self.supply_pressure_pa > 1e-6 and p_des_pa > self.supply_pressure_pa:
            rospy.logwarn(
                "Requested Fleg %.3f N requires %.3f bar, above supply %.3f bar. "
                "Inlet valve command is saturated.",
                fleg,
                self.last_body_pressure_bar,
                self.supply_pressure_bar,
            )

        return [
            # Phase 1: thrust.
            (
                self.body_thrust_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                inlet_opening,
                0.0,
            ),

            # Phase 2: exhaust / damping.
            (
                self.body_exhaust_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                0.0,
                self.body_exhaust_opening_deg,
            ),

            # Phase 3: close both valves.
            (
                self.body_final_close_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                0.0,
                0.0,
            ),
        ]
        
    def build_body_sequence_from_inlet_deg(self, inlet_deg):
        """
        Direct valve calibration path.

        This bypasses the theoretical Fleg -> pressure -> valve mapping.
        It commands servoValve1 directly with inlet_deg.

        Used to collect experimental data:

            inlet_deg -> body_imu_acceleration.x -> delta_v_x -> J_real

        Rope commands are NOT published in this mode.
        """
        nan = float('nan')

        try:
            inlet = float(inlet_deg)
        except Exception:
            rospy.logwarn("Invalid inlet_deg received. Falling back to zero.")
            inlet = 0.0

        if not math.isfinite(inlet):
            rospy.logwarn("Non-finite inlet_deg received. Falling back to zero.")
            inlet = 0.0

        inlet = self.clamp(
            inlet,
            self.body_inlet_min_deg,
            self.body_inlet_max_deg
        )

        # Equivalent Fleg only for logging.
        # It is NOT used to compute the valve command here.
        fleg_equiv = self._fleg_equivalent_from_inlet_deg(inlet)

        self.last_body_inlet_opening_deg = inlet
        self.last_body_leg_force_n = fleg_equiv

        if self.supply_pressure_pa > 1e-6:
            ratio = (inlet - self.body_inlet_min_deg) / (
                self.body_inlet_max_deg - self.body_inlet_min_deg
            )
            ratio = self.clamp(ratio, 0.0, 1.0)
            p_equiv_pa = ratio * self.supply_pressure_pa
        else:
            ratio = 0.0
            p_equiv_pa = 0.0

        self.last_body_pressure_ratio = ratio
        self.last_body_pressure_pa = p_equiv_pa
        self.last_body_pressure_bar = p_equiv_pa / 1e5

        return [
            # Phase 1: direct inlet thrust.
            (
                self.body_thrust_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                inlet,
                0.0,
            ),

            # Phase 2: exhaust / damping.
            (
                self.body_exhaust_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                0.0,
                self.body_exhaust_opening_deg,
            ),

            # Phase 3: close both valves.
            (
                self.body_final_close_ms,
                'none',
                nan,
                nan,
                nan,
                nan,
                0.0,
                0.0,
            ),
        ]   

    # ────────────────────────────────────────────────────────────────────
    # Winch mode helpers
    # ────────────────────────────────────────────────────────────────────

    def set_torque_mode(self):
        msg = String(data='closed_loop_torque')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'torque'
        rospy.loginfo("Published closed_loop_torque to both winches")

    def set_velocity_mode(self):
        msg = String(data='closed_loop_velocity')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'velocity'
        rospy.loginfo("Published closed_loop_velocity to both winches")

    def set_idle_mode(self):
        msg = String(data='idle')
        self.pub_left_mode.publish(msg)
        self.pub_right_mode.publish(msg)
        self.current_mode = 'idle'
        rospy.logwarn("Published idle to both winches")

    def set_mode(self, mode: str):
        if mode == self.current_mode:
            return

        if mode == 'torque':
            self.set_torque_mode()
        elif mode == 'velocity':
            self.set_velocity_mode()
        elif mode == 'idle':
            self.set_idle_mode()
        elif mode == 'none':
            return
        else:
            rospy.logwarn("Unknown mode requested: %s", str(mode))

    # ────────────────────────────────────────────────────────────────────
    # Sequence start / publish helpers
    # ────────────────────────────────────────────────────────────────────

    def start_sequence(self, sequence, command_ropes, sequence_name='unknown', imu_fleg_cmd=None, imu_inlet_deg=None):
        self.sequence_running = True
        self.sequence_start_ms = self.now_ms()
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.command_ropes_during_sequence = bool(command_ropes)
        self.active_timeline = self.build_timeline(sequence)

        # Estimate IMU log metadata if not explicitly provided.
        first_inlet = 0.0
        if len(sequence) > 0:
            first_inlet = float(sequence[0][6])

        if imu_inlet_deg is None:
            imu_inlet_deg = first_inlet

        if imu_fleg_cmd is None:
            imu_fleg_cmd = self._fleg_equivalent_from_inlet_deg(imu_inlet_deg)

        self._reset_imu_sequence_state(
            fleg_cmd_n=imu_fleg_cmd,
            inlet_deg=imu_inlet_deg,
            sequence_name=sequence_name
        )

        if self.command_ropes_during_sequence:
            self.set_torque_mode()
            rospy.loginfo(
                "Starting jump sequence: VALVES + LOCAL ROPE COMMANDS. "
                "Manual ropes start after piston thrust."
            )
        else:
            rospy.logwarn(
                "Starting jump sequence: VALVES ONLY. "
                "Rope forces Fr_l(t), Fr_r(t) must come from the high-level controller."
            )

    def publish_all(self, lf, rf, lv, rv, s1, s2):
        # Always publish valve commands.
        self.pub_s1.publish(Float32(data=float(s1)))
        self.pub_s2.publish(Float32(data=float(s2)))

        # Track valve1 timing for IMU diagnostic.
        self._update_imu_valve_tracking(float(s1))

        # In optimized /alpine_body/command mode, never overwrite Fr_l(t), Fr_r(t).
        if not self.command_ropes_during_sequence:
            return

        now = rospy.Time.now()

        left = RopeCommand()
        left.header.stamp = now
        left.rope_force = float(lf)
        left.rope_velocity = float(lv)
        left.rope_position = float('nan')

        right = RopeCommand()
        right.header.stamp = now
        right.rope_force = float(rf)
        right.rope_velocity = float(rv)
        right.rope_position = float('nan')

        self.pub_left.publish(left)
        self.pub_right.publish(right)

    def send_if_changed(self, mode, lf, rf, lv, rv, s1, s2):
        cmd = (
            mode,
            self.safe_round(lf),
            self.safe_round(rf),
            self.safe_round(lv),
            self.safe_round(rv),
            self.safe_round(s1),
            self.safe_round(s2),
            self.command_ropes_during_sequence,
        )

        if cmd == self.last_sent:
            return

        if self.command_ropes_during_sequence:
            self.set_mode(mode)

        self.publish_all(lf, rf, lv, rv, s1, s2)
        self.last_sent = cmd

    def refresh_current(self, lf, rf, lv, rv, s1, s2):
        self.publish_all(lf, rf, lv, rv, s1, s2)

    def publish_valves_zero(self):
        self.pub_s1.publish(Float32(data=0.0))
        self.pub_s2.publish(Float32(data=0.0))
        self._update_imu_valve_tracking(0.0)

    def publish_hold_light(self):
        nan = float('nan')
        self.command_ropes_during_sequence = True
        self.send_if_changed(
            'torque',
            -abs(self.manual_hold_force),
            -abs(self.manual_hold_force),
            nan,
            nan,
            0.0,
            0.0,
        )

    def publish_zero_force(self):
        nan = float('nan')
        self.command_ropes_during_sequence = True
        self.send_if_changed(
            'torque',
            0.0,
            0.0,
            nan,
            nan,
            0.0,
            0.0,
        )

    # ────────────────────────────────────────────────────────────────────
    # 100 Hz tick
    # ────────────────────────────────────────────────────────────────────

    def tick(self, event):
        if not self.sequence_running:
            return

        elapsed_ms = self.now_ms() - self.sequence_start_ms

        nan = float('nan')
        phase_index = None
        mode = 'none'
        lf, rf = nan, nan
        lv, rv = nan, nan
        s1, s2 = 0.0, 0.0

        for i, (limit_ms, mode_v, lf_v, rf_v, lv_v, rv_v, s1_v, s2_v) in enumerate(self.active_timeline):
            if elapsed_ms < limit_ms:
                phase_index = i
                mode = mode_v
                lf, rf = lf_v, rf_v
                lv, rv = lv_v, rv_v
                s1, s2 = s1_v, s2_v
                break

        if phase_index is None:
            self.sequence_running = False
            self.current_phase_index = -1
            self.phase_refresh_div = 0
            self.last_sent = None

            if self.command_ropes_during_sequence:
                rospy.loginfo("Manual jump sequence completed -> light hold")
                self.publish_hold_light()
            else:
                rospy.loginfo("Optimized Fleg valve sequence completed -> valves closed")
                self.publish_valves_zero()

            return

        if phase_index != self.current_phase_index:
            self.current_phase_index = phase_index
            self.phase_refresh_div = 0

            rospy.loginfo(
                "Phase %d/%d -> mode=%s, lf=%s, rf=%s, lv=%s, rv=%s, "
                "servoValve1=%.1f, servoValve2=%.1f, command_ropes=%s",
                phase_index + 1,
                len(self.active_timeline),
                mode,
                str(self.safe_round(lf)),
                str(self.safe_round(rf)),
                str(self.safe_round(lv)),
                str(self.safe_round(rv)),
                float(s1),
                float(s2),
                str(self.command_ropes_during_sequence),
            )

            self.send_if_changed(mode, lf, rf, lv, rv, s1, s2)
            return

        # Refresh every 3 ticks, approximately 30 Hz.
        self.phase_refresh_div += 1
        if self.phase_refresh_div >= 3:
            self.phase_refresh_div = 0
            self.refresh_current(lf, rf, lv, rv, s1, s2)

    # ────────────────────────────────────────────────────────────────────
    # Service handlers
    # ────────────────────────────────────────────────────────────────────
    
    def handle_calib_inlet_deg(self, msg):
        """
        Direct valve calibration callback.

        Command an inlet valve angle directly through:

            /alpine_body/calib_inlet_deg

        Example:
            rostopic pub -1 /alpine_body/calib_inlet_deg std_msgs/Float32 "data: 30.0"

        This mode is valves-only and never commands ropes.
        """
        try:
            inlet_deg = float(msg.data)

            body_sequence = self.build_body_sequence_from_inlet_deg(inlet_deg)

            rospy.logwarn(
                "[/alpine_body/calib_inlet_deg] Direct inlet command: "
                "inlet=%.1f deg, equivalent_Fleg=%.3f N, "
                "P_equiv=%.3f bar, ratio=%.3f, thrust=%.1f ms. "
                "Starting VALVES-ONLY calibration sequence.",
                self.last_body_inlet_opening_deg,
                self.last_body_leg_force_n,
                self.last_body_pressure_bar,
                self.last_body_pressure_ratio,
                self.body_thrust_ms,
            )

            self.start_sequence(
                body_sequence,
                command_ropes=False,
                sequence_name='calib_inlet_deg',
                imu_fleg_cmd=self.last_body_leg_force_n,
                imu_inlet_deg=self.last_body_inlet_opening_deg,
            )

        except Exception as e:
            rospy.logerr("[/alpine_body/calib_inlet_deg] failed: %s", str(e))
            self.publish_valves_zero()
            

    def handle_jump_impulse_des(self, msg):
        """
        Desired impulse command.

        Topic:
            /alpine_body/jump_impulse_des

        Type:
            std_msgs/Float32

        Input:
            data = J_des [N*s]

        Mapping:
            J_des -> delta_v_des = J_des / robot_mass_kg
            delta_v_des -> inlet_deg by experimental lookup table

        This mode is valves-only and never commands ropes.
        """
        try:
            j_des = float(msg.data)

            inlet_deg, delta_v_des = self.lookup_inlet_from_impulse(j_des)

            body_sequence = self.build_body_sequence_from_inlet_deg(inlet_deg)

            rospy.logwarn(
                "[/alpine_body/jump_impulse_des] J_des=%.3f N*s, "
                "mass=%.3f kg, delta_v_des=%.3f m/s -> inlet=%.1f deg. "
                "thrust=%.1f ms. Starting VALVES-ONLY J lookup sequence.",
                j_des,
                self.robot_mass_kg,
                delta_v_des,
                self.last_body_inlet_opening_deg,
                self.body_thrust_ms,
            )

            self.start_sequence(
                body_sequence,
                command_ropes=False,
                sequence_name='j_des_lookup',
                imu_fleg_cmd=self.last_body_leg_force_n,
                imu_inlet_deg=self.last_body_inlet_opening_deg,
            )

        except Exception as e:
            rospy.logerr("[/alpine_body/jump_impulse_des] failed: %s", str(e))
            self.publish_valves_zero()

    def handle_alpine_body_command(self, req):
        """
        Optimized jump path called by climbingrobot_controller2_real.py.

        Fleg:
            single force command for the piston impulse.
            Here it is mapped to desired piston pressure and then to valve opening.

        Fropes:
            NOT handled here.
            Fr_l(t), Fr_r(t) must be published continuously to /winch/left/command
            and /winch/right/command by the high-level controller.

        Important:
            The high-level controller should avoid pulling ropes during the Fleg thrust.
            Rope-force pattern should start after t_th or be near zero during t_th.
        """
        try:
            leg_force_n = float(req.leg_force)
            body_sequence = self.build_body_sequence_from_fleg(leg_force_n)

            rospy.logwarn(
                "[/alpine_body/command] Fleg received: %.3f N, "
                "contact_normal=(%.3f, %.3f, %.3f). "
                "Mapped to P_des=%.3f bar, ratio=%.3f, inlet=%.1f deg, thrust=%.1f ms. "
                "Starting VALVES-ONLY sequence.",
                self.last_body_leg_force_n,
                float(req.contact_normal.x),
                float(req.contact_normal.y),
                float(req.contact_normal.z),
                self.last_body_pressure_bar,
                self.last_body_pressure_ratio,
                self.last_body_inlet_opening_deg,
                self.body_thrust_ms,
            )

            self.start_sequence(
                body_sequence,
                command_ropes=False,
                sequence_name='optimized_fleg',
                imu_fleg_cmd=self.last_body_leg_force_n,
                imu_inlet_deg=self.last_body_inlet_opening_deg,
            )

            return AlpineBodyCommandResponse(ack=True)

        except Exception as e:
            rospy.logerr("[/alpine_body/command] failed: %s", str(e))
            self.publish_valves_zero()
            return AlpineBodyCommandResponse(ack=False)

    def handle_jump(self, req):
        """
        Manual standalone safe jump.

        This mode commands both:
          - servo valves
          - local hardcoded rope forces

        Manual rope timing:
          - first phase: piston thrust only, ropes = 0
          - after thrust: ropes start pulling
        """
        self.manual_sequence = self.build_manual_sequence()

        manual_inlet = 90.0
        manual_equiv_fleg = self._fleg_equivalent_from_inlet_deg(manual_inlet)

        self.start_sequence(
            self.manual_sequence,
            command_ropes=True,
            sequence_name='manual_jump',
            imu_fleg_cmd=manual_equiv_fleg,
            imu_inlet_deg=manual_inlet,
        )

        return TriggerResponse(
            success=True,
            message="Manual jump sequence started: piston first, then local rope commands"
        )

    def handle_abort(self, req):
        self.sequence_running = False
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.publish_valves_zero()
        self.set_torque_mode()
        self.publish_hold_light()

        rospy.logwarn("Jump aborted -> valves closed, light hold torque sent")
        return TriggerResponse(
            success=True,
            message="Jump aborted: valves closed, light hold sent"
        )

    def handle_stop(self, req):
        self.sequence_running = False
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.publish_valves_zero()
        self.set_torque_mode()
        self.publish_zero_force()

        rospy.logwarn("Jump stopped -> valves closed, zero rope force sent")
        return TriggerResponse(
            success=True,
            message="Jump stopped: valves closed, zero force sent"
        )


def main():
    rospy.init_node('jump_node')
    node = JumpNode()
    rospy.spin()


if __name__ == '__main__':
    main()
