#!/usr/bin/env python3

"""
RadioMaster left-stick planar-force command for the real ALPINE robot.

The node publishes the high-level body-frame EDF bias through
``/alpine_body/wrench_cmd``. Values are force requests in newtons per active
EDF; PWM and the T1--T4 allocation remain inside the body firmware.

Default USB-HID mapping:

  left stick up     (axes[1] = +1) -> negative body X
  left stick down   (axes[1] = -1) -> positive body X
  left stick left   (axes[0] = +1) -> positive body Y
  left stick right  (axes[0] = -1) -> negative body Y

The requested planar vector is radially limited, so diagonal stick commands
cannot exceed ``~max_force_n``. After startup, input loss, or an E-stop, the
stick must pass through the centre before non-zero output is accepted.

The characterised EDF map starts at 4.81 N. Requests between zero and that
minimum would be raised to 4.81 N by the current firmware, so this node holds
the real output at zero until ``~min_active_force_n`` is requested. This
avoids a hidden thrust jump after a very small stick displacement.
"""

import math
import threading
import time
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def apply_rescaled_deadzone(value: float, deadzone: float) -> float:
    """Map [-1, 1] to [-1, 1], with a continuous zero region."""
    value = clamp(float(value), -1.0, 1.0)
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0

    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled, value)


class RadioMasterPropellerForceNode:
    def __init__(self):
        self.joy_topic = str(rospy.get_param("~joy_topic", "/joy"))
        self.wrench_topic = str(
            rospy.get_param("~wrench_topic", "/alpine_body/wrench_cmd")
        )
        self.raw_command_topic = str(
            rospy.get_param("~raw_command_topic", "/alpine_body/cmd_raw")
        )
        self.estop_latched_topic = str(
            rospy.get_param(
                "~estop_latched_topic",
                "/radiomaster_estop/latched",
            )
        )

        self.vertical_axis_index = int(
            rospy.get_param("~vertical_axis_index", 1)
        )
        self.horizontal_axis_index = int(
            rospy.get_param("~horizontal_axis_index", 0)
        )
        self.fx_from_vertical_sign = float(
            rospy.get_param("~fx_from_vertical_sign", -1.0)
        )
        self.fy_from_horizontal_sign = float(
            rospy.get_param("~fy_from_horizontal_sign", 1.0)
        )

        self.deadzone = float(rospy.get_param("~deadzone", 0.08))
        self.center_rearm_threshold = float(
            rospy.get_param("~center_rearm_threshold", 0.12)
        )
        self.max_force_n = float(rospy.get_param("~max_force_n", 14.03))
        self.min_active_force_n = float(
            rospy.get_param("~min_active_force_n", 4.81)
        )
        self.hard_max_force_n = float(
            rospy.get_param("~hard_max_force_n", 14.03)
        )
        self.publish_rate_hz = float(
            rospy.get_param("~publish_rate_hz", 20.0)
        )
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50)
        )

        self.output_enabled = bool(
            rospy.get_param("~output_enabled", False)
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.require_estop_state = bool(
            rospy.get_param("~require_estop_state", True)
        )
        self.manage_open_loop_mode = bool(
            rospy.get_param("~manage_open_loop_mode", True)
        )
        self.open_loop_enable_settle_s = float(
            rospy.get_param("~open_loop_enable_settle_s", 0.10)
        )

        self._validate_parameters()

        self._lock = threading.RLock()
        self._last_joy_monotonic: Optional[float] = None
        self._vertical_raw = 0.0
        self._horizontal_raw = 0.0
        self._mapping_valid = False
        self._estop_state_received = False
        self._estop_latched = True
        self._centre_required = True
        self._open_loop_enabled_by_us = False
        self._open_loop_enabled_monotonic: Optional[float] = None
        self._last_state: Optional[str] = None
        self._last_active: Optional[bool] = None

        self._wrench_pub = rospy.Publisher(
            self.wrench_topic,
            Wrench,
            queue_size=5,
        )
        self._raw_command_pub = rospy.Publisher(
            self.raw_command_topic,
            String,
            queue_size=5,
        )
        self._requested_wrench_pub = rospy.Publisher(
            "~requested_wrench",
            Wrench,
            queue_size=5,
        )
        self._active_pub = rospy.Publisher(
            "~active",
            Bool,
            queue_size=1,
            latch=True,
        )
        self._state_pub = rospy.Publisher(
            "~state",
            String,
            queue_size=1,
            latch=True,
        )

        self._joy_sub = rospy.Subscriber(
            self.joy_topic,
            Joy,
            self._joy_cb,
            queue_size=20,
        )
        self._estop_sub = rospy.Subscriber(
            self.estop_latched_topic,
            Bool,
            self._estop_cb,
            queue_size=5,
        )

        self._publish_active(False)
        self._publish_state("STARTING")
        rospy.on_shutdown(self.shutdown)

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz),
            self._publish_tick,
        )

        rospy.loginfo(
            "RadioMaster propeller-force control ready: joy=%s "
            "vertical=axes[%d] horizontal=axes[%d] range=%.3f..%.3f N "
            "deadzone=%.3f output_enabled=%s dry_run=%s",
            self.joy_topic,
            self.vertical_axis_index,
            self.horizontal_axis_index,
            self.min_active_force_n,
            self.max_force_n,
            self.deadzone,
            self.output_enabled,
            self.dry_run,
        )

    def _validate_parameters(self):
        if self.vertical_axis_index < 0 or self.horizontal_axis_index < 0:
            raise ValueError("joystick axis indices must be zero or greater")
        if self.vertical_axis_index == self.horizontal_axis_index:
            raise ValueError("vertical and horizontal axes must be different")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("~deadzone must be in [0, 1)")
        if not self.deadzone <= self.center_rearm_threshold <= 1.0:
            raise ValueError(
                "~center_rearm_threshold must be between ~deadzone and 1"
            )
        if self.hard_max_force_n <= 0.0:
            raise ValueError("~hard_max_force_n must be positive")
        if not 0.0 < self.max_force_n <= self.hard_max_force_n:
            raise ValueError(
                "~max_force_n must be positive and no greater than "
                "~hard_max_force_n"
            )
        if not 0.0 <= self.min_active_force_n <= self.max_force_n:
            raise ValueError(
                "~min_active_force_n must be in [0, ~max_force_n]"
            )
        if not 1.0 <= self.publish_rate_hz <= 100.0:
            raise ValueError("~publish_rate_hz must be in [1, 100]")
        if self.input_timeout_s <= 0.0:
            raise ValueError("~input_timeout_s must be positive")
        if self.open_loop_enable_settle_s < 0.0:
            raise ValueError("~open_loop_enable_settle_s cannot be negative")
        if abs(self.fx_from_vertical_sign) < 1e-9:
            raise ValueError("~fx_from_vertical_sign cannot be zero")
        if abs(self.fy_from_horizontal_sign) < 1e-9:
            raise ValueError("~fy_from_horizontal_sign cannot be zero")

        self.fx_from_vertical_sign = math.copysign(
            1.0,
            self.fx_from_vertical_sign,
        )
        self.fy_from_horizontal_sign = math.copysign(
            1.0,
            self.fy_from_horizontal_sign,
        )

    @staticmethod
    def _make_wrench(fx_n: float, fy_n: float) -> Wrench:
        message = Wrench()
        message.force.x = float(fx_n)
        message.force.y = float(fy_n)
        message.force.z = 0.0
        message.torque.x = 0.0
        message.torque.y = 0.0
        message.torque.z = 0.0
        return message

    def _joy_cb(self, message: Joy):
        required_size = max(
            self.vertical_axis_index,
            self.horizontal_axis_index,
        ) + 1
        if len(message.axes) < required_size:
            with self._lock:
                self._mapping_valid = False
                self._centre_required = True
            rospy.logerr_throttle(
                2.0,
                "RadioMaster propeller mapping invalid: axes[%d] and "
                "axes[%d] requested, but /joy contains only %d axes",
                self.vertical_axis_index,
                self.horizontal_axis_index,
                len(message.axes),
            )
            return

        with self._lock:
            self._vertical_raw = clamp(
                float(message.axes[self.vertical_axis_index]),
                -1.0,
                1.0,
            )
            self._horizontal_raw = clamp(
                float(message.axes[self.horizontal_axis_index]),
                -1.0,
                1.0,
            )
            self._last_joy_monotonic = time.monotonic()
            self._mapping_valid = True

    def _estop_cb(self, message: Bool):
        latched = bool(message.data)
        with self._lock:
            self._estop_state_received = True
            self._estop_latched = latched
            if latched:
                self._centre_required = True

        if latched:
            self._publish_real_zero()
            self._disable_open_loop()

    def _requested_force(
        self,
        vertical_raw: float,
        horizontal_raw: float,
    ) -> Tuple[float, float]:
        vertical = apply_rescaled_deadzone(vertical_raw, self.deadzone)
        horizontal = apply_rescaled_deadzone(horizontal_raw, self.deadzone)

        fx_unit = self.fx_from_vertical_sign * vertical
        fy_unit = self.fy_from_horizontal_sign * horizontal

        magnitude = math.hypot(fx_unit, fy_unit)
        if magnitude > 1.0:
            fx_unit /= magnitude
            fy_unit /= magnitude

        return (
            self.max_force_n * fx_unit,
            self.max_force_n * fy_unit,
        )

    def _publish_tick(self, _event):
        now = time.monotonic()

        with self._lock:
            last_joy = self._last_joy_monotonic
            mapping_valid = self._mapping_valid
            vertical_raw = self._vertical_raw
            horizontal_raw = self._horizontal_raw
            estop_received = self._estop_state_received
            estop_latched = self._estop_latched
            centre_required = self._centre_required

        input_alive = (
            mapping_valid
            and last_joy is not None
            and (now - last_joy) <= self.input_timeout_s
        )

        fx_n, fy_n = self._requested_force(
            vertical_raw,
            horizontal_raw,
        )
        self._requested_wrench_pub.publish(
            self._make_wrench(fx_n, fy_n)
        )

        requested_magnitude_n = math.hypot(fx_n, fy_n)
        below_active_floor = (
            requested_magnitude_n > 1e-6
            and requested_magnitude_n < self.min_active_force_n
        )
        if below_active_floor:
            fx_n = 0.0
            fy_n = 0.0

        if not input_alive:
            with self._lock:
                self._centre_required = True
            self._safe_output("JOY_TIMEOUT")
            return

        if self.require_estop_state and not estop_received:
            with self._lock:
                self._centre_required = True
            self._safe_output("WAITING_FOR_ESTOP")
            return

        if estop_latched:
            with self._lock:
                self._centre_required = True
            self._safe_output("ESTOP_LATCHED")
            return

        raw_magnitude = math.hypot(vertical_raw, horizontal_raw)
        if centre_required:
            if raw_magnitude <= self.center_rearm_threshold:
                with self._lock:
                    self._centre_required = False
                fx_n = 0.0
                fy_n = 0.0
            else:
                self._safe_output("CENTRE_REQUIRED")
                return

        requested_active = math.hypot(fx_n, fy_n) > 1e-6

        if not self.output_enabled:
            self._publish_active(False)
            self._publish_state(
                "OUTPUT_DISABLED: fx={:.3f} N fy={:.3f} N".format(
                    fx_n,
                    fy_n,
                )
            )
            return

        if self.dry_run:
            self._publish_active(False)
            if below_active_floor:
                self._publish_state(
                    "DRY_RUN_BELOW_MIN: requested={:.3f} N, output=0 N".format(
                        requested_magnitude_n
                    )
                )
            else:
                self._publish_state(
                    "DRY_RUN: fx={:.3f} N fy={:.3f} N".format(
                        fx_n,
                        fy_n,
                    )
                )
            return

        if requested_active and not self._ensure_open_loop(now):
            self._publish_real_zero()
            self._publish_active(False)
            self._publish_state("ENABLING_OPEN_LOOP")
            return

        self._wrench_pub.publish(self._make_wrench(fx_n, fy_n))
        self._publish_active(requested_active)
        if requested_active:
            self._publish_state(
                "ACTIVE: fx={:.3f} N fy={:.3f} N".format(fx_n, fy_n)
            )
        elif below_active_floor:
            self._publish_state(
                "BELOW_MIN_ACTIVE_FORCE: requested={:.3f} N".format(
                    requested_magnitude_n
                )
            )
        else:
            self._publish_state("READY")

    def _ensure_open_loop(self, now: float) -> bool:
        if not self.manage_open_loop_mode:
            return True

        with self._lock:
            enabled = self._open_loop_enabled_by_us
            enabled_at = self._open_loop_enabled_monotonic

        if not enabled:
            self._raw_command_pub.publish(String(data="pron"))
            with self._lock:
                self._open_loop_enabled_by_us = True
                self._open_loop_enabled_monotonic = now
            rospy.logwarn(
                "RadioMaster propeller force requested: enabling firmware "
                "open-loop bias with 'pron'."
            )
            return self.open_loop_enable_settle_s <= 0.0

        if enabled_at is None:
            return False
        return (now - enabled_at) >= self.open_loop_enable_settle_s

    def _disable_open_loop(self):
        if not self.output_enabled or self.dry_run:
            return
        if not self.manage_open_loop_mode:
            return

        with self._lock:
            if not self._open_loop_enabled_by_us:
                return
            self._open_loop_enabled_by_us = False
            self._open_loop_enabled_monotonic = None

        self._raw_command_pub.publish(String(data="proff"))
        rospy.logwarn(
            "RadioMaster propeller control interlocked: sent 'proff'."
        )

    def _safe_output(self, state: str):
        self._publish_real_zero()
        self._disable_open_loop()
        self._publish_active(False)
        self._publish_state(state)

    def _publish_real_zero(self):
        if self.output_enabled and not self.dry_run:
            self._wrench_pub.publish(self._make_wrench(0.0, 0.0))

    def _publish_active(self, active: bool):
        active = bool(active)
        if self._last_active != active:
            self._last_active = active
            self._active_pub.publish(Bool(data=active))

    def _publish_state(self, state: str):
        if self._last_state == state:
            return
        self._last_state = state
        self._state_pub.publish(String(data=state))

        if state.startswith("ACTIVE") or state == "READY":
            rospy.loginfo("RadioMaster propeller control: %s", state)
        elif state.startswith("DRY_RUN") or state.startswith(
            "OUTPUT_DISABLED"
        ):
            rospy.loginfo_throttle(
                1.0,
                "RadioMaster propeller control: %s",
                state,
            )
        else:
            rospy.logwarn("RadioMaster propeller control: %s", state)

    def shutdown(self):
        try:
            if hasattr(self, "_timer"):
                self._timer.shutdown()

            if self.output_enabled and not self.dry_run:
                zero = self._make_wrench(0.0, 0.0)
                for _ in range(3):
                    self._wrench_pub.publish(zero)
                    rospy.sleep(0.02)
                self._disable_open_loop()
        except Exception as exc:
            rospy.logwarn(
                "Could not complete RadioMaster propeller shutdown: %s",
                exc,
            )


def main():
    rospy.init_node("radiomaster_propeller_force")
    RadioMasterPropellerForceNode()
    rospy.spin()


if __name__ == "__main__":
    main()