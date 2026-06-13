#!/usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rospy

from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf

from climbingrobot_hardware_interface.msg import RopeTelemetry, AlpineBodyTelemetry


def quat_to_rot(qxyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = qxyzw
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def normalize(v: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return fallback.copy() if fallback is not None else np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


class AlpineOdometryNode:

    def __init__(self):
        self.world_frame     = rospy.get_param('~world_frame', 'world')
        self.base_frame      = rospy.get_param('~base_frame', 'base_link')
        self.publish_tf      = rospy.get_param('~publish_tf', False)
        self.publish_rate_hz = rospy.get_param('~publish_rate_hz', 100.0)

        self.anchor_left  = np.array(rospy.get_param('~anchor_left_xyz',  [0.0, 0.0, 0.0]), dtype=float)
        self.anchor_right = np.array(rospy.get_param('~anchor_right_xyz', [1.30, 0.0, 0.0]), dtype=float)

        self.right_attachment_from_left_body = np.array(
            rospy.get_param('~right_attachment_from_left_body_xyz', [-0.55, 0.0, 0.0]), dtype=float
        )
        self.body_origin_from_left_attachment = np.array(
            rospy.get_param('~body_origin_from_left_attachment_xyz', [-0.275, 0.0, 0.0]), dtype=float
        )

        self.left_home_offset_m  = float(rospy.get_param('~left_home_offset_m',  0.0))
        self.right_home_offset_m = float(rospy.get_param('~right_home_offset_m', 0.53))
        self.left_rope_sign      = float(rospy.get_param('~left_rope_sign', 1.0))
        self.right_rope_sign     = float(rospy.get_param('~right_rope_sign', -1.0))
        self.left_rope_axis      = str(rospy.get_param('~left_rope_axis', '-x'))

        left_rope_topic  = rospy.get_param('~left_rope_topic',  '/winch/left/telemetry')
        right_rope_topic = rospy.get_param('~right_rope_topic', '/winch/right/telemetry')
        dongle_topic     = rospy.get_param('~dongle_topic',     '/alpine/dongle/telemetry')

        self.left_rope_msg:  Optional[RopeTelemetry] = None
        self.right_rope_msg: Optional[RopeTelemetry] = None
        self.epoch_ms:       Optional[float]         = None

        self.body_quat_xyzw: Optional[np.ndarray] = None
        self.body_gyro:      Optional[np.ndarray] = None
        self.body_acc:       Optional[np.ndarray] = None
        self.rope_quat_xyzw: Optional[np.ndarray] = None
        self.rope_gyro:      Optional[np.ndarray] = None
        self.rope_acc:       Optional[np.ndarray] = None

        self.last_body_pos:  Optional[np.ndarray] = None
        self.last_stamp_s:   Optional[float]      = None

        rospy.Subscriber(left_rope_topic,  RopeTelemetry,    self._cb_left_rope,  queue_size=10)
        rospy.Subscriber(right_rope_topic, RopeTelemetry,    self._cb_right_rope, queue_size=10)
        rospy.Subscriber(dongle_topic,     Float32MultiArray, self._cb_dongle,    queue_size=10)

        self.pub_odom  = rospy.Publisher('/odom',                    Odometry,         queue_size=10)
        self.pub_pose  = rospy.Publisher('/alpine/odometry/pose',    PoseStamped,      queue_size=10)
        self.pub_debug = rospy.Publisher('/alpine/odometry/debug',   Float32MultiArray, queue_size=10)
        self.pub_body_telemetry = rospy.Publisher('/alpine_body/telemetry', AlpineBodyTelemetry, queue_size=10)

        #self.tf_broadcaster = tf.TransformBroadcaster() if self.publish_tf else None

        dt = max(1.0 / max(self.publish_rate_hz, 1.0), 0.001)
        rospy.Timer(rospy.Duration(dt), self._update)

        span = np.linalg.norm(self.right_attachment_from_left_body)
        rospy.loginfo(
            f'ODOMETRY NEW VERSION | left_rope_axis={self.left_rope_axis} | '
            f'left_offset={self.left_home_offset_m:.3f} sign={self.left_rope_sign:+.1f} | '
            f'right_offset={self.right_home_offset_m:.3f} sign={self.right_rope_sign:+.1f} | '
            f'right_attachment_from_left_body={self.right_attachment_from_left_body.tolist()} | '
            f'attachment_span={span:.3f}'
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _cb_left_rope(self, msg: RopeTelemetry):
        self.left_rope_msg = msg

    def _cb_right_rope(self, msg: RopeTelemetry):
        self.right_rope_msg = msg

    def _parse_imu_block(self, vals: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        q_wxyz = vals[0:4].astype(float)
        acc    = vals[4:7].astype(float)
        gyro   = vals[7:10].astype(float)
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)

        n = np.linalg.norm(q_xyzw)
        if np.isfinite(n) and n > 1e-9:
            q_xyzw = q_xyzw / n
        else:
            q_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        return q_xyzw, acc, gyro

    def _vec3_msg(self, v: np.ndarray) -> Vector3:
        return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))

    def _quat_msg(self, q_xyzw: np.ndarray) -> Quaternion:
        return Quaternion(
            x=float(q_xyzw[0]),
            y=float(q_xyzw[1]),
            z=float(q_xyzw[2]),
            w=float(q_xyzw[3]),
        )

    def _rpy_from_quat(self, q_xyzw: np.ndarray) -> Vector3:
        r, p, y = tf.transformations.euler_from_quaternion([
            float(q_xyzw[0]),
            float(q_xyzw[1]),
            float(q_xyzw[2]),
            float(q_xyzw[3]),
        ])
        return Vector3(x=float(r), y=float(p), z=float(y))

    def _publish_alpine_body_telemetry(self):
        if (
            self.body_quat_xyzw is None or self.body_gyro is None or self.body_acc is None or
            self.rope_quat_xyzw is None or self.rope_gyro is None or self.rope_acc is None
        ):
            return

        msg = AlpineBodyTelemetry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame

        msg.rope_imu_orientation = self._quat_msg(self.rope_quat_xyzw)
        msg.rope_imu_angular_velocity = self._vec3_msg(self.rope_gyro)
        msg.rope_imu_rpy = self._rpy_from_quat(self.rope_quat_xyzw)
        msg.rope_imu_rpy_derivatives = self._vec3_msg(self.rope_gyro)

        # Compatibility with climbingrobot_controller2_real.py:
        # older real controller reads msg.rope_imu_rpy_d.
        if hasattr(msg, "rope_imu_rpy_d"):
            msg.rope_imu_rpy_d = self._vec3_msg(self.rope_gyro)

        msg.rope_imu_acceleration = self._vec3_msg(self.rope_acc)

        msg.body_imu_orientation = self._quat_msg(self.body_quat_xyzw)
        msg.body_imu_angular_velocity = self._vec3_msg(self.body_gyro)
        msg.body_imu_rpy = self._rpy_from_quat(self.body_quat_xyzw)
        msg.body_imu_rpy_derivatives = self._vec3_msg(self.body_gyro)
        msg.body_imu_acceleration = self._vec3_msg(self.body_acc)

        self.pub_body_telemetry.publish(msg)

    def _cb_dongle(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size < 23:
            return
        self.epoch_ms = float(data[0])
        imu1 = data[1:12]
        imu2 = data[12:23]

        # Real robot mapping:
        # IMU1 = rope / cable IMU
        # IMU2 = body IMU
        self.rope_quat_xyzw, self.rope_acc, self.rope_gyro = self._parse_imu_block(imu1)
        self.body_quat_xyzw, self.body_acc, self.body_gyro = self._parse_imu_block(imu2)

        self._publish_alpine_body_telemetry()

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _effective_lengths(self) -> Tuple[float, float]:
        # Convert raw post-homing winch telemetry into physical absolute lengths.
        # This must stay consistent with climbingrobot_controller2_real.py.
        l1_raw = 0.0 if self.left_rope_msg  is None else float(self.left_rope_msg.rope_length)
        l2_raw = 0.0 if self.right_rope_msg is None else float(self.right_rope_msg.rope_length)
        l1_eff = self.left_home_offset_m + self.left_rope_sign * l1_raw
        l2_eff = self.right_home_offset_m + self.right_rope_sign * l2_raw
        return l1_eff, l2_eff

    def _axis_vector_from_rot(self, R: np.ndarray, axis_name: str) -> np.ndarray:
        axis_name = axis_name.strip().lower()
        mapping = {
            'x':  R[:, 0], '-x': -R[:, 0],
            'y':  R[:, 1], '-y': -R[:, 1],
            'z':  R[:, 2], '-z': -R[:, 2],
        }
        return normalize(mapping.get(axis_name, R[:, 0]))

    # ------------------------------------------------------------------ #
    # Timer callback  (ROS 1: receives a TimerEvent argument)
    # ------------------------------------------------------------------ #

    def _update(self, event):
        if (
            self.left_rope_msg  is None or
            self.right_rope_msg is None or
            self.body_quat_xyzw is None or
            self.rope_quat_xyzw is None
        ):
            return

        l1_eff, l2_eff = self._effective_lengths()

        R_body = quat_to_rot(self.body_quat_xyzw)
        R_rope = quat_to_rot(self.rope_quat_xyzw)

        rope_dir_l     = self._axis_vector_from_rot(R_rope, self.left_rope_axis)
        left_attachment  = self.anchor_left + rope_dir_l * l1_eff
        right_attachment = left_attachment  + (R_body @ self.right_attachment_from_left_body)
        body_pos         = left_attachment  + (R_body @ self.body_origin_from_left_attachment)

        stamp_s = (self.epoch_ms or 0.0) * 1e-3
        lin_vel = np.zeros(3, dtype=float)
        if self.last_body_pos is not None and self.last_stamp_s is not None:
            dt = stamp_s - self.last_stamp_s
            if dt > 1e-4:
                lin_vel = (body_pos - self.last_body_pos) / dt

        self.last_body_pos  = body_pos.copy()
        self.last_stamp_s   = stamp_s

        now = rospy.Time.now()

        # ---- PoseStamped ------------------------------------------------
        pose = PoseStamped()
        pose.header.stamp    = now
        pose.header.frame_id = self.world_frame
        pose.pose.position   = Point(x=float(body_pos[0]), y=float(body_pos[1]), z=float(body_pos[2]))
        pose.pose.orientation.x = float(self.body_quat_xyzw[0])
        pose.pose.orientation.y = float(self.body_quat_xyzw[1])
        pose.pose.orientation.z = float(self.body_quat_xyzw[2])
        pose.pose.orientation.w = float(self.body_quat_xyzw[3])
        self.pub_pose.publish(pose)

        # ---- Odometry ---------------------------------------------------
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose       = pose.pose
        odom.twist.twist.linear = Vector3(
            x=float(lin_vel[0]), y=float(lin_vel[1]), z=float(lin_vel[2])
        )
        if self.body_gyro is not None:
            odom.twist.twist.angular = Vector3(
                x=float(self.body_gyro[0]),
                y=float(self.body_gyro[1]),
                z=float(self.body_gyro[2])
            )
        self.pub_odom.publish(odom)

        # ---- TF ---------------------------------------------------------
        # if self.tf_broadcaster is not None:
        #     self.tf_broadcaster.sendTransform(
        #         (float(body_pos[0]), float(body_pos[1]), float(body_pos[2])),
        #         (
        #             float(self.body_quat_xyzw[0]),
        #             float(self.body_quat_xyzw[1]),
        #             float(self.body_quat_xyzw[2]),
        #             float(self.body_quat_xyzw[3]),
        #         ),
        #         now,
        #         self.base_frame,
        #         self.world_frame,
        #     )

        # ---- Debug ------------------------------------------------------
        attachment_dist        = float(np.linalg.norm(right_attachment - left_attachment))
        nominal_span           = float(np.linalg.norm(self.right_attachment_from_left_body))
        attachment_dist_error  = attachment_dist - nominal_span
        modeled_l2             = float(np.linalg.norm(right_attachment - self.anchor_right))
        modeled_l2_error_signed = modeled_l2 - l2_eff
        right_anchor_error     = abs(modeled_l2_error_signed)

        dbg = Float32MultiArray()
        dbg.data = [
            float(self.epoch_ms or 0.0),
            float(l1_eff), float(l2_eff),
            float(left_attachment[0]),  float(left_attachment[1]),  float(left_attachment[2]),
            float(right_attachment[0]), float(right_attachment[1]), float(right_attachment[2]),
            float(body_pos[0]),         float(body_pos[1]),         float(body_pos[2]),
            float(attachment_dist),
            float(attachment_dist_error),
            float(right_anchor_error),
            float(modeled_l2_error_signed),
        ]
        self.pub_debug.publish(dbg)


def main():
    rospy.init_node('alpine_odometry_node')
    node = AlpineOdometryNode()
    rospy.spin()


if __name__ == '__main__':
    main()