#!/usr/bin/env python3

import time

import rospy

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from climbingrobot_hardware_interface.msg import RopeCommand
from climbingrobot_hardware_interface.srv import RopeControlMode, RopeControlModeRequest
from termcolor import colored


class WinchStartupSequence:

    def __init__(self):
        self.step_delay = rospy.get_param('~step_delay', 1.0)

        # Hardcoded real homing.
        self.homing_left_force_n = -37.0
        self.homing_right_force_n = -20.0

        # Smooth homing profile.
        self.homing_left_start_force_n = -22.0
        self.homing_right_start_force_n = -22.0

        # Do not go below the stable holding force near the winch.
        self.homing_left_final_force_n = -26.0
        self.homing_right_final_force_n = -20.0

        self.homing_pull_duration_s = 14.0
        self.homing_command_rate_hz = 20.0
        self.homing_pre_brake_command_s = 0.5

        self.homing_ramp_up_s = 1.5
        self.homing_soft_land_s = 11.0

        # ── Publishers ──────────────────────────────────────────────────
        self.left_mode_pub  = rospy.Publisher('/winch/left/set_motor_mode',  String,      queue_size=1)
        self.right_mode_pub = rospy.Publisher('/winch/right/set_motor_mode', String,      queue_size=1)
        self.left_cmd_pub   = rospy.Publisher('/winch/left/command',         RopeCommand, queue_size=1)
        self.right_cmd_pub  = rospy.Publisher('/winch/right/command',        RopeCommand, queue_size=1)

        # ── Service proxies ─────────────────────────────────────────────
        # wait_for_service is called here once at startup (blocking)
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('/winch/left/brake_disengage')
        rospy.wait_for_service('/winch/right/brake_disengage')
        rospy.wait_for_service('/winch/left/rope_zero')
        rospy.wait_for_service('/winch/right/rope_zero')
        rospy.wait_for_service('/winch/left/set_control_mode')
        rospy.wait_for_service('/winch/right/set_control_mode')

        self.left_brake_srv  = rospy.ServiceProxy('/winch/left/brake_disengage',  Trigger)
        self.right_brake_srv = rospy.ServiceProxy('/winch/right/brake_disengage', Trigger)
        self.left_zero_srv   = rospy.ServiceProxy('/winch/left/rope_zero',         Trigger)
        self.right_zero_srv  = rospy.ServiceProxy('/winch/right/rope_zero',        Trigger)
        self.left_mode_srv   = rospy.ServiceProxy('/winch/left/set_control_mode',  RopeControlMode)
        self.right_mode_srv  = rospy.ServiceProxy('/winch/right/set_control_mode', RopeControlMode)

        rospy.loginfo("All services available.")
        time.sleep(1.0)

    # ────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────

    def sleep_step(self, delay=1.0):
        rospy.sleep(delay)          # respects ROS time (sim or wall)

    def publish_mode(self, mode: str):
        rospy.loginfo(f"Setting motor mode: {mode}")
        msg = String(data=mode)

        # Old-style robust topic command.
        for _ in range(10):
            self.left_mode_pub.publish(msg)
            self.right_mode_pub.publish(msg)
            rospy.sleep(0.05)

    def call_trigger(self, proxy: rospy.ServiceProxy, service_name: str):
        rospy.loginfo(f"Calling service: {service_name}")
        try:
            resp = proxy(TriggerRequest())
            if not resp.success:
                rospy.logwarn(f"{service_name} returned false: {resp.message}")
            else:
                rospy.loginfo(f"{service_name} succeeded: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"{service_name} call failed: {e}")

    def publish_command(
        self,
        side: str,
        rope_force: float,
        rope_velocity: float = 0.0,
        rope_position: float = 0.0,
    ):
        msg = RopeCommand()
        msg.header.stamp = rospy.Time.now()
        msg.rope_force    = float(rope_force)
        msg.rope_velocity = float(rope_velocity)
        msg.rope_position = float(rope_position)

        if side == 'left':
            self.left_cmd_pub.publish(msg)
        elif side == 'right':
            self.right_cmd_pub.publish(msg)
        else:
            raise ValueError("side must be 'left' or 'right'")

        rospy.loginfo(
            f"Commanded {side} winch: "
            f"force={rope_force}, "
            f"velocity={rope_velocity}, "
            f"position={rope_position}"
        )

    def publish_homing_forces_continuous(self, duration_s, smooth=True):
        """
        Keep publishing torque commands during homing.

        Smooth profile:
          1) ramp from low force to target force
          2) hold target force
          3) reduce force near the end to avoid hitting the top too aggressively
        """
        duration_s = max(0.0, float(duration_s))
        rate_hz = max(1.0, float(self.homing_command_rate_hz))
        rate = rospy.Rate(rate_hz)

        t_start = rospy.Time.now()
        t_end = t_start + rospy.Duration(duration_s)

        def clamp01(x):
            return max(0.0, min(1.0, float(x)))

        def lerp(a, b, u):
            u = clamp01(u)
            return float(a) + (float(b) - float(a)) * u

        def smooth_force(elapsed_s, side):
            if side == "left":
                f_start = self.homing_left_start_force_n
                f_target = self.homing_left_force_n
                f_final = self.homing_left_final_force_n
            else:
                f_start = self.homing_right_start_force_n
                f_target = self.homing_right_force_n
                f_final = self.homing_right_final_force_n

            if not smooth:
                return f_target

            ramp_up_s = max(0.0, float(self.homing_ramp_up_s))
            soft_land_s = max(0.0, float(self.homing_soft_land_s))
            remaining_s = max(0.0, duration_s - elapsed_s)

            # Initial ramp: low force -> target force.
            if ramp_up_s > 1e-6 and elapsed_s < ramp_up_s:
                return lerp(f_start, f_target, elapsed_s / ramp_up_s)

            # Final soft landing: target force -> final low force.
            if soft_land_s > 1e-6 and remaining_s < soft_land_s:
                return lerp(f_final, f_target, remaining_s / soft_land_s)

            return f_target

        last_log_s = -1

        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            elapsed_s = (rospy.Time.now() - t_start).to_sec()

            left_force = smooth_force(elapsed_s, "left")
            right_force = smooth_force(elapsed_s, "right")

            self.publish_command("left", rope_force=left_force)
            self.publish_command("right", rope_force=right_force)

            # Log only once per second, otherwise it spams at 20 Hz.
            elapsed_int = int(elapsed_s)
            if elapsed_int != last_log_s:
                last_log_s = elapsed_int
                rospy.logwarn(
                    "HOMING smooth forces: t=%.1f/%.1f s, left=%.1f N, right=%.1f N",
                    elapsed_s,
                    duration_s,
                    left_force,
                    right_force,
                )

            rate.sleep()


    # ────────────────────────────────────────────────────────────────────
    # Main sequence
    # ────────────────────────────────────────────────────────────────────

    def run_sequence(self):

        # 1) set torque control
        print(colored("homing:closed_loop_torque", "red"))
        self.publish_mode("closed_loop_torque")
        self.sleep_step(delay=0.30)

        # 2) preload torque commands BEFORE removing brakes
        print(colored("homing: continuous rope forces preload", "red"))
        rospy.logwarn(
            "HOMING forces: left=%.1f N, right=%.1f N, duration=%.1f s, rate=%.1f Hz",
            self.homing_left_force_n,
            self.homing_right_force_n,
            self.homing_pull_duration_s,
            self.homing_command_rate_hz,
        )

        self.publish_homing_forces_continuous(self.homing_pre_brake_command_s)

        # 3) disengage brakes
        print(colored("homing: remove brakes", "red"))
        self.call_trigger(self.left_brake_srv,  '/winch/left/brake_disengage')
        self.call_trigger(self.right_brake_srv, '/winch/right/brake_disengage')

        # 4) keep pulling continuously during homing
        print(colored("homing: pulling continuously", "red"))
        self.publish_homing_forces_continuous(self.homing_pull_duration_s)

        # 5) zero encoders at the reached high/stable point
        print(colored("homing: zero encoders", "red"))
        self.call_trigger(self.left_zero_srv, '/winch/left/rope_zero')
        self.call_trigger(self.right_zero_srv, '/winch/right/rope_zero')

        print(colored("Homing:Winch startup sequence complete.", "red"))



def main():
    rospy.init_node('winch_startup_sequence')
    homingProcedure = WinchStartupSequence()

    try:
        homingProcedure.run_sequence()
        rospy.loginfo("Node is idle. Press Ctrl+C to exit.")
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()