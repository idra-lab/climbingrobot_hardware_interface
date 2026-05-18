#!/usr/bin/env python3

import math
import rospy

from std_srvs.srv import Trigger, TriggerResponse
from climbingrobot_hardware_interface.srv import AlpineBodyCommand, AlpineBodyCommandResponse
from std_msgs.msg import Float32, String
from climbingrobot_hardware_interface.msg import RopeCommand


class JumpNode:
    """
    Low-level jump interface for ALPINE.

    Two different entry points are intentionally kept separated:

    1) /alpine/jump
       Manual standalone test.
       jump.py commands valves + local hardcoded rope forces.

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

        # Called by climbingrobot_controller2_real.py.
        # The request contains optimized Fleg as leg_force.
        rospy.Service(
            '/alpine_body/command',
            AlpineBodyCommand,
            self.handle_alpine_body_command
        )

        # ── Timer: 100 Hz state machine ──────────────────────────────────
        self.timer = rospy.Timer(rospy.Duration(0.01), self.tick)

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

        # ─────────────────────────────────────────────────────────────────
        # Fleg -> pressure -> valve feedforward parameters.
        # Used only when /alpine_body/command is called by the real controller.
        #
        # P_des = Fleg / A_piston
        #
        # Without pressure sensor feedback this is open-loop:
        # P_des / P_supply is mapped to a servo-valve opening.
        # ─────────────────────────────────────────────────────────────────
        self.piston_diameter_m = rospy.get_param('~piston_diameter_m', 0.032)
        self.piston_area_m2 = math.pi * (0.5 * self.piston_diameter_m) ** 2

        # Regulated supply pressure after the reducer.
        # Thesis reference: 4.5 bar nominal.
        self.supply_pressure_bar = rospy.get_param('~supply_pressure_bar', 4.5)
        self.supply_pressure_pa = self.supply_pressure_bar * 1e5

        # This must be coherent with the optimization parameter t_th.
        # Paper/simulation default is often 50 ms.
        self.body_thrust_ms = rospy.get_param('~body_thrust_ms', 50.0)

        # Servo command range for inlet valve.
        # 0 generally means closed, 90 means fully open in your current convention.
        self.body_inlet_min_deg = rospy.get_param('~body_inlet_min_deg', 0.0)
        self.body_inlet_max_deg = rospy.get_param('~body_inlet_max_deg', 90.0)

        # Exhaust/damping after thrust.
        self.body_exhaust_ms = rospy.get_param('~body_exhaust_ms', 900.0)
        self.body_exhaust_opening_deg = rospy.get_param('~body_exhaust_opening_deg', 90.0)

        # Final closing phase.
        self.body_final_close_ms = rospy.get_param('~body_final_close_ms', 200.0)

        # Optional minimum force threshold.
        # Below this, keep inlet closed.
        self.min_valid_fleg_n = rospy.get_param('~min_valid_fleg_n', 1.0)

        # ── State ────────────────────────────────────────────────────────
        self.sequence_running = False
        self.sequence_start_ms = 0.0
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None
        self.current_mode = None

        # If True, this node publishes RopeCommand during the sequence.
        # Manual mode only.
        #
        # If False, this node publishes valves only.
        # Optimized /alpine_body/command mode.
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
            "Fleg mapping: piston_diameter=%.4f m, area=%.6e m^2, "
            "supply=%.3f bar, thrust=%.1f ms, inlet=[%.1f, %.1f] deg",
            self.piston_diameter_m,
            self.piston_area_m2,
            self.supply_pressure_bar,
            self.body_thrust_ms,
            self.body_inlet_min_deg,
            self.body_inlet_max_deg,
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

    def build_timeline(self, sequence):
        timeline = []
        acc = 0.0

        for dur, mode, lf, rf, lv, rv, s1, s2 in sequence:
            acc += float(dur)
            timeline.append((acc, mode, lf, rf, lv, rv, s1, s2))

        return timeline

    # ────────────────────────────────────────────────────────────────────
    # Sequence builders
    # ────────────────────────────────────────────────────────────────────

    def build_manual_sequence(self):
        """
        Local standalone test sequence.

        This is NOT the optimized-control path.
        It is only useful to test the hardware without the high-level controller.
        """
        nan = float('nan')

        return [
            # Piston only, ropes idle.
            (
                self.manual_phase1_ms,
                'torque',
                0.0,
                0.0,
                nan,
                nan,
                90.0,
                0.0,
            ),

            # Ropes pull while airborne.
            (
                self.manual_phase2_ms,
                'torque',
                -abs(self.manual_up_force),
                -abs(self.manual_up_force),
                nan,
                nan,
                0.0,
                90.0,
            ),

            # Lower pull, exhaust still open.
            (
                self.manual_phase3_ms,
                'torque',
                -abs(self.manual_mid_force),
                -abs(self.manual_mid_force),
                nan,
                nan,
                0.0,
                90.0,
            ),

            # Final hold.
            (
                self.manual_phase4_ms,
                'torque',
                -abs(self.manual_hold_force),
                -abs(self.manual_hold_force),
                nan,
                nan,
                0.0,
                0.0,
            ),
        ]

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
            # servoValve1 = inlet/thrust valve
            # servoValve2 = closed
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
            # The exact landing profile is still open-loop here.
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

    def start_sequence(self, sequence, command_ropes):
        self.sequence_running = True
        self.sequence_start_ms = self.now_ms()
        self.current_phase_index = -1
        self.phase_refresh_div = 0
        self.last_sent = None

        self.command_ropes_during_sequence = bool(command_ropes)
        self.active_timeline = self.build_timeline(sequence)

        if self.command_ropes_during_sequence:
            self.set_torque_mode()
            rospy.loginfo("Starting jump sequence: VALVES + LOCAL ROPE COMMANDS")
        else:
            rospy.logwarn(
                "Starting jump sequence: VALVES ONLY. "
                "Rope forces Fr_l(t), Fr_r(t) must come from the high-level controller."
            )

    def publish_all(self, lf, rf, lv, rv, s1, s2):
        # Always publish valve commands.
        self.pub_s1.publish(Float32(data=float(s1)))
        self.pub_s2.publish(Float32(data=float(s2)))

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

            self.start_sequence(body_sequence, command_ropes=False)
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

        Do not use this mode when testing optimized Fleg/Fropes from
        climbingrobot_controller2_real.py, otherwise local rope commands will override
        the optimized rope-force pattern.
        """
        self.manual_sequence = self.build_manual_sequence()
        self.start_sequence(self.manual_sequence, command_ropes=True)

        return TriggerResponse(
            success=True,
            message="Manual jump sequence started: valves + local rope commands"
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