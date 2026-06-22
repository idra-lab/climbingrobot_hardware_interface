#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Wrench


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PropellerSineTask:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "yaw_thr")
        self.amp = float(rospy.get_param("~amp", 0.08))
        self.freq_hz = float(rospy.get_param("~freq_hz", 0.5))
        self.duration_s = float(rospy.get_param("~duration_s", 20.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))

        # Optional: start with lower amplitude and ramp to amp
        self.ramp_s = float(rospy.get_param("~ramp_s", 2.0))

        # Safety limits
        self.amp = clamp(self.amp, 0.0, 1.0)
        self.freq_hz = max(0.01, self.freq_hz)
        self.rate_hz = clamp(self.rate_hz, 5.0, 100.0)

        self.pub_raw = rospy.Publisher("/alpine_body/cmd_raw", String, queue_size=1)
        self.pub_wrench = rospy.Publisher("/alpine_body/wrench_cmd", Wrench, queue_size=1)

        # For rosbag/debug
        self.pub_u = rospy.Publisher("/alpine_body/propeller_sine/u", Float32, queue_size=1)

        rospy.sleep(0.5)

    def raw(self, cmd):
        self.pub_raw.publish(String(data=cmd))

    def wrench(self, fx=0.0, fy=0.0, mz=0.0):
        msg = Wrench()
        msg.force.x = float(fx)
        msg.force.y = float(fy)
        msg.force.z = 0.0
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = float(mz)
        self.pub_wrench.publish(msg)

    def send_stop(self):
        self.wrench(0.0, 0.0, 0.0)
        self.raw("THR,0,0,0,0,0,0")
        rospy.sleep(0.05)
        self.raw("stop")

    def send_yaw_thr(self, u):
        """
        Direct yaw pair test.
        Firmware mapping:
          yaw + -> T1 + T3
          yaw - -> T2 + T4
        """
        u = clamp(u, -1.0, 1.0)

        if u >= 0.0:
            t1 = u
            t2 = 0.0
            t3 = u
            t4 = 0.0
        else:
            t1 = 0.0
            t2 = -u
            t3 = 0.0
            t4 = -u

        self.raw(f"THR,{t1:.4f},{t2:.4f},{t3:.4f},{t4:.4f},0,0")

    def send_pitch_pth(self, u):
        """
        Direct pitch pair test.
          u > 0 -> T5
          u < 0 -> T6
        """
        u = clamp(u, -1.0, 1.0)
        self.raw(f"pth {u:.4f}")

    def send_pitch_thr(self, u):
        """
        Same as pth, but using explicit THR command.
        """
        u = clamp(u, -1.0, 1.0)

        if u >= 0.0:
            t5 = u
            t6 = 0.0
        else:
            t5 = 0.0
            t6 = -u

        self.raw(f"THR,0,0,0,0,{t5:.4f},{t6:.4f}")
    def send_lateral_thr(self, u):
        """
        Direct lateral translation test:
          u > 0 -> left  -> T1 + T2
          u < 0 -> right -> T3 + T4
        """
        u = clamp(u, -1.0, 1.0)

        if u >= 0.0:
            t1 = u
            t2 = u
            t3 = 0.0
            t4 = 0.0
        else:
            t1 = 0.0
            t2 = 0.0
            t3 = -u
            t4 = -u

        self.raw(f"THR,{t1:.4f},{t2:.4f},{t3:.4f},{t4:.4f},0,0")

    def send_yaw_wrc(self, u):
        """
        Test through WRC path instead of direct THR.
        This uses firmware setLateralThrustersFromWrench().
        """
        u = clamp(u, -1.0, 1.0)
        self.wrench(0.0, 0.0, u)

    def run(self):
        rospy.logwarn(
            "Propeller sine test: mode=%s amp=%.3f freq=%.3fHz duration=%.1fs rate=%.1fHz",
            self.mode, self.amp, self.freq_hz, self.duration_s, self.rate_hz
        )

        # Disable attitude hold so IMU PID does not fight the sine test.
        self.raw("attoff")
        rospy.sleep(0.05)
        self.raw("proff")
        rospy.sleep(0.05)
        self.send_stop()
        rospy.sleep(0.2)

        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()

        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                t = now - t0

                if t > self.duration_s:
                    break

                ramp = 1.0
                if self.ramp_s > 1e-6:
                    ramp = clamp(t / self.ramp_s, 0.0, 1.0)

                u = ramp * self.amp * math.sin(2.0 * math.pi * self.freq_hz * t)
                self.pub_u.publish(Float32(data=float(u)))

                if self.mode == "yaw_thr":
                    self.send_yaw_thr(u)

                elif self.mode == "pitch_pth":
                    self.send_pitch_pth(u)

                elif self.mode == "pitch_thr":
                    self.send_pitch_thr(u)

                elif self.mode == "yaw_wrc":
                    self.send_yaw_wrc(u)
                    
                elif self.mode == "lateral_thr":
                    self.send_lateral_thr(u)

                else:
                    rospy.logerr("Unknown mode '%s'. Use yaw_thr, pitch_pth, pitch_thr, yaw_wrc.", self.mode)
                    break

                rate.sleep()

        finally:
            rospy.logwarn("Stopping propellers.")
            self.send_stop()


if __name__ == "__main__":
    rospy.init_node("propeller_sine_task")
    PropellerSineTask().run()