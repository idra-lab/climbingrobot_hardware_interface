#!/usr/bin/env python3

"""
USB-joystick rope emergency stop for the ALPINE RadioMaster Boxer.

The Boxer is connected directly to the ROS computer in USB Joystick (HID)
mode. ``joy_node`` publishes ``sensor_msgs/Joy`` and this node watches the
configured SE input. Activating SE latches the stop sequence:

  1. engage both winch brakes immediately;
  2. wait six wall-clock seconds;
  3. continuously publish ``idle`` to both winches while latched.

Returning SE to the safe position does not clear the latch. The ``~reset``
service only removes this node's idle override; it deliberately does not
disengage the mechanical brakes.
"""

import threading
import time
from typing import Optional

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class RadioMasterEstopNode:
    def __init__(self):
        self.joy_topic = str(rospy.get_param("~joy_topic", "/joy"))
        self.input_kind = str(rospy.get_param("~input_kind", "axis")).lower()
        self.input_index = int(rospy.get_param("~input_index", 4))
        self.trigger_below = bool(rospy.get_param("~trigger_below", True))
        self.trigger_threshold = float(
            rospy.get_param("~trigger_threshold", -0.50)
        )
        self.release_threshold = float(
            rospy.get_param("~release_threshold", 0.50)
        )

        if self.input_kind not in ("axis", "button"):
            raise ValueError("~input_kind must be either 'axis' or 'button'")
        if self.input_index < 0:
            raise ValueError("~input_index must be zero or greater")
        if (
            self.trigger_below
            and self.release_threshold <= self.trigger_threshold
        ):
            raise ValueError(
                "~release_threshold must be greater than ~trigger_threshold "
                "when ~trigger_below is true"
            )
        if (
            not self.trigger_below
            and self.release_threshold >= self.trigger_threshold
        ):
            raise ValueError(
                "~release_threshold must be less than ~trigger_threshold "
                "when ~trigger_below is false"
            )

        self.idle_delay_s = float(rospy.get_param("~idle_delay_s", 6.0))
        self.idle_publish_rate_hz = float(
            rospy.get_param("~idle_publish_rate_hz", 10.0)
        )
        self.brake_refresh_s = float(rospy.get_param("~brake_refresh_s", 1.0))
        self.service_wait_s = float(rospy.get_param("~service_wait_s", 0.25))

        self.failsafe_on_input_loss = bool(
            rospy.get_param("~failsafe_on_input_loss", True)
        )
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50)
        )
        self.startup_grace_s = float(
            rospy.get_param("~startup_grace_s", 3.0)
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.persist_latch_param = str(
            rospy.get_param(
                "~persist_latch_param",
                "/alpine/radiomaster_estop_latched",
            )
        )

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._started_monotonic = time.monotonic()
        persisted_latch = bool(rospy.get_param(self.persist_latch_param, False))
        self._last_joy_monotonic: Optional[float] = None
        self._last_input_value: Optional[float] = None
        self._input_alive = False
        self._latched = persisted_latch
        self._latched_monotonic: Optional[float] = (
            self._started_monotonic - self.idle_delay_s
            if persisted_latch
            else None
        )
        self._last_brake_refresh_monotonic = float("-inf")
        self._last_idle_publish_monotonic = float("-inf")
        self._brake_refresh_running = False

        self._left_idle_pub = rospy.Publisher(
            "/winch/left/set_motor_mode", String, queue_size=10
        )
        self._right_idle_pub = rospy.Publisher(
            "/winch/right/set_motor_mode", String, queue_size=10
        )

        self._input_value_pub = rospy.Publisher(
            "~input_value", Float32, queue_size=5
        )
        self._link_pub = rospy.Publisher(
            "~link_alive", Bool, queue_size=1, latch=True
        )
        self._latched_pub = rospy.Publisher(
            "~latched", Bool, queue_size=1, latch=True
        )
        self._event_pub = rospy.Publisher(
            "~event", String, queue_size=5, latch=True
        )

        self._joy_sub = rospy.Subscriber(
            self.joy_topic,
            Joy,
            self._joy_cb,
            queue_size=20,
        )
        rospy.Service("~trigger", Trigger, self._manual_trigger_cb)
        rospy.Service("~reset", Trigger, self._reset_cb)

        self._link_pub.publish(Bool(data=False))
        self._latched_pub.publish(Bool(data=persisted_latch))
        self._event_pub.publish(
            String(
                data=(
                    "RESTORED LATCH: enforcing brakes and idle"
                    if persisted_latch
                    else "starting"
                )
            )
        )

        rospy.on_shutdown(self.shutdown)

        threading.Thread(
            target=self._supervisor_loop,
            name="radiomaster-estop-supervisor",
            daemon=True,
        ).start()

        if persisted_latch:
            rospy.logerr(
                "Restored persistent RadioMaster E-stop latch; "
                "enforcing brakes and idle."
            )
            self._start_brake_refresh()

        rospy.loginfo(
            "RadioMaster USB E-stop ready: topic=%s %s[%d] trigger=%s %.3f "
            "release=%.3f idle_delay=%.2fs dry_run=%s",
            self.joy_topic,
            self.input_kind,
            self.input_index,
            "below" if self.trigger_below else "above",
            self.trigger_threshold,
            self.release_threshold,
            self.idle_delay_s,
            self.dry_run,
        )

    def shutdown(self):
        self._stop_event.set()

    def _joy_cb(self, message: Joy):
        values = message.axes if self.input_kind == "axis" else message.buttons
        if self.input_index >= len(values):
            rospy.logerr_throttle(
                2.0,
                "RadioMaster mapping invalid: %s[%d] requested, but /joy "
                "contains only %d %s values",
                self.input_kind,
                self.input_index,
                len(values),
                self.input_kind,
            )
            return

        now = time.monotonic()
        input_value = float(values[self.input_index])

        with self._lock:
            self._last_joy_monotonic = now
            self._last_input_value = input_value

        self._input_value_pub.publish(Float32(data=input_value))
        self._set_input_alive(True)

        if self._input_requests_estop(input_value):
            self._trigger_estop(
                "SE active: {}[{}]={:.3f}".format(
                    self.input_kind,
                    self.input_index,
                    input_value,
                )
            )

    def _input_requests_estop(self, value: float) -> bool:
        if self.trigger_below:
            return value <= self.trigger_threshold
        return value >= self.trigger_threshold

    def _input_is_safe_for_reset(self, value: Optional[float]) -> bool:
        if value is None:
            return False
        if self.trigger_below:
            return value >= self.release_threshold
        return value <= self.release_threshold

    def _set_input_alive(self, alive: bool):
        publish = False
        with self._lock:
            if self._input_alive != bool(alive):
                self._input_alive = bool(alive)
                publish = True
        if publish:
            self._link_pub.publish(Bool(data=bool(alive)))
            rospy.loginfo(
                "RadioMaster USB joystick input %s",
                "alive" if alive else "lost",
            )

    def _supervisor_loop(self):
        period_s = 0.02
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            now = time.monotonic()

            with self._lock:
                last_joy = self._last_joy_monotonic
                latched = self._latched
                latched_at = self._latched_monotonic
                last_brake = self._last_brake_refresh_monotonic
                last_idle = self._last_idle_publish_monotonic

            input_alive = (
                last_joy is not None
                and (now - last_joy) <= self.input_timeout_s
            )
            self._set_input_alive(input_alive)

            startup_elapsed = now - self._started_monotonic
            if (
                self.failsafe_on_input_loss
                and not input_alive
                and startup_elapsed >= self.startup_grace_s
            ):
                self._trigger_estop(
                    "USB joystick timeout: no valid /joy input for "
                    "{:.2f} s".format(self.input_timeout_s)
                )

            if latched and latched_at is not None:
                if now - last_brake >= max(0.10, self.brake_refresh_s):
                    with self._lock:
                        self._last_brake_refresh_monotonic = now
                    self._start_brake_refresh()

                idle_period_s = 1.0 / max(1.0, self.idle_publish_rate_hz)
                if (
                    now - latched_at >= self.idle_delay_s
                    and now - last_idle >= idle_period_s
                ):
                    with self._lock:
                        self._last_idle_publish_monotonic = now
                    self._publish_idle_override()

            self._stop_event.wait(period_s)

    def _trigger_estop(self, reason: str) -> bool:
        now = time.monotonic()
        with self._lock:
            if self._latched:
                return False
            self._latched = True
            self._latched_monotonic = now
            self._last_brake_refresh_monotonic = float("-inf")
            self._last_idle_publish_monotonic = float("-inf")

        rospy.logerr(
            "REMOTE ROPE E-STOP LATCHED: %s. Engaging brakes now; idle in %.2f s.",
            reason,
            self.idle_delay_s,
        )
        rospy.set_param(self.persist_latch_param, True)
        self._latched_pub.publish(Bool(data=True))
        self._event_pub.publish(String(data="LATCHED: " + reason))
        self._start_brake_refresh()
        return True

    def _start_brake_refresh(self):
        with self._lock:
            if self._brake_refresh_running:
                return
            self._brake_refresh_running = True

        threading.Thread(
            target=self._engage_both_brakes,
            name="radiomaster-brake-engage",
            daemon=True,
        ).start()

    def _engage_both_brakes(self):
        try:
            if self.dry_run:
                rospy.logwarn_throttle(
                    1.0,
                    "[DRY RUN] Would call both brake_engage services",
                )
                return

            calls = []
            for side in ("left", "right"):
                thread = threading.Thread(
                    target=self._call_brake_engage,
                    args=(side,),
                    name="brake-engage-" + side,
                    daemon=True,
                )
                calls.append(thread)
                thread.start()

            for thread in calls:
                thread.join(timeout=max(0.5, self.service_wait_s + 0.5))
        finally:
            with self._lock:
                self._brake_refresh_running = False

    def _call_brake_engage(self, side: str):
        service_name = "/winch/{}/brake_engage".format(side)
        try:
            rospy.wait_for_service(service_name, timeout=self.service_wait_s)
            response = rospy.ServiceProxy(service_name, Trigger)(TriggerRequest())
            if not response.success:
                rospy.logerr(
                    "%s returned failure: %s", service_name, response.message
                )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr_throttle(
                1.0,
                "Cannot engage %s brake through %s: %s",
                side,
                service_name,
                exc,
            )

    def _publish_idle_override(self):
        if self.dry_run:
            rospy.logwarn_throttle(
                1.0,
                "[DRY RUN] Would publish idle to both winches",
            )
            return

        idle = String(data="idle")
        self._left_idle_pub.publish(idle)
        self._right_idle_pub.publish(idle)
        rospy.logerr_throttle(
            2.0,
            "REMOTE ROPE E-STOP: enforcing idle on both winches",
        )

    def _manual_trigger_cb(self, _request):
        newly_latched = self._trigger_estop("manual ROS trigger service")
        if newly_latched:
            return TriggerResponse(success=True, message="Remote rope E-stop latched.")
        return TriggerResponse(success=True, message="Remote rope E-stop already latched.")

    def _reset_cb(self, _request):
        with self._lock:
            if not self._latched:
                return TriggerResponse(success=True, message="E-stop is not latched.")

            if not self._input_alive:
                return TriggerResponse(
                    success=False,
                    message="Reset rejected: USB joystick input is not healthy.",
                )

            if not self._input_is_safe_for_reset(self._last_input_value):
                return TriggerResponse(
                    success=False,
                    message="Reset rejected: put SE in the normal/up position first.",
                )

            self._latched = False
            self._latched_monotonic = None

        rospy.set_param(self.persist_latch_param, False)
        self._latched_pub.publish(Bool(data=False))
        self._event_pub.publish(
            String(
                data=(
                    "RESET: idle override removed; mechanical brakes remain engaged"
                )
            )
        )
        rospy.logwarn(
            "Remote rope E-stop reset. Idle override removed; "
            "mechanical brakes remain engaged."
        )
        return TriggerResponse(
            success=True,
            message=(
                "E-stop override reset. Brakes remain engaged; use the normal "
                "safe startup/homing procedure to resume."
            ),
        )


def main():
    rospy.init_node("radiomaster_estop")
    RadioMasterEstopNode()
    rospy.spin()


if __name__ == "__main__":
    main()
