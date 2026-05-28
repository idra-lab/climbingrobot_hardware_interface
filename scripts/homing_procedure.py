#!/usr/bin/env python3

import time

import rospy

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from climbingrobot_hardware_interface.msg import RopeCommand
from termcolor import colored


class WinchStartupSequence:

    def __init__(self):
        self.step_delay = rospy.get_param('~step_delay', 1.0)

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

        self.left_brake_srv  = rospy.ServiceProxy('/winch/left/brake_disengage',  Trigger)
        self.right_brake_srv = rospy.ServiceProxy('/winch/right/brake_disengage', Trigger)
        self.left_zero_srv   = rospy.ServiceProxy('/winch/left/rope_zero',         Trigger)
        self.right_zero_srv  = rospy.ServiceProxy('/winch/right/rope_zero',        Trigger)

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
        self.left_mode_pub.publish(msg)
        self.right_mode_pub.publish(msg)

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

    # ────────────────────────────────────────────────────────────────────
    # Main sequence
    # ────────────────────────────────────────────────────────────────────

    def run_sequence(self):
        # 1) set torque control
        print(colored("homing:closed_loop_torque", "red"))
        self.publish_mode("closed_loop_torque")

        # 2) pull left winch up
        print(colored("homing: set rope forces for winch up", "red"))
        self.publish_command("left", rope_force=-60)
        self.publish_command("right", rope_force=-15)

        # 3) disengage brakes
        print(colored("homing: remove brakes", "red"))
        self.call_trigger(self.left_brake_srv,  '/winch/left/brake_disengage')
        self.call_trigger(self.right_brake_srv, '/winch/right/brake_disengage')
        self.sleep_step(delay=10.0)

        # 4) zero encoders
        print(colored("homing: zero encoders", "red"))
        self.call_trigger(self.left_zero_srv, '/winch/left/rope_zero')
        self.call_trigger(self.right_zero_srv, '/winch/right/rope_zero')
        # TODO: add to readings 0.7 for sx and 0.63 for dc

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