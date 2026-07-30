#!/usr/bin/env python3

"""
Trigger the existing ALPINE manual jump service from a RadioMaster button.

The node is intentionally only an input adapter: it does not duplicate or
change the jump sequence implemented by ``jump.py``.  A jump is requested once
on the rising edge of the configured button.  The button must be released
before another request can be generated.
"""

import threading
import time
from typing import Optional

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger, TriggerRequest


class RadioMasterJumpNode:
    def __init__(self):
        self.joy_topic = str(rospy.get_param("~joy_topic", "/joy"))
        self.button_index = int(rospy.get_param("~button_index", 1))
        self.trigger_threshold = float(
            rospy.get_param("~trigger_threshold", 0.75)
        )
        self.release_threshold = float(
            rospy.get_param("~release_threshold", 0.25)
        )
        self.jump_service = str(
            rospy.get_param("~jump_service", "/alpine/jump")
        )
        self.service_wait_s = float(
            rospy.get_param("~service_wait_s", 1.0)
        )
        self.input_timeout_s = float(
            rospy.get_param("~input_timeout_s", 0.50)
        )
        self.cooldown_s = float(rospy.get_param("~cooldown_s", 2.0))
        self.estop_latched_topic = str(
            rospy.get_param(
                "~estop_latched_topic",
                "/radiomaster_estop/latched",
            )
        )
        self.require_estop_state = bool(
            rospy.get_param("~require_estop_state", True)
        )
        self.dry_run = bool(rospy.get_param("~dry_run", False))

        if self.button_index < 0:
            raise ValueError("~button_index must be zero or greater")
        if self.release_threshold >= self.trigger_threshold:
            raise ValueError(
                "~release_threshold must be lower than ~trigger_threshold"
            )
        if self.input_timeout_s <= 0.0:
            raise ValueError("~input_timeout_s must be greater than zero")
        if self.cooldown_s < 0.0:
            raise ValueError("~cooldown_s must be zero or greater")

        self._lock = threading.RLock()
        self._last_joy_monotonic: Optional[float] = None
        self._last_request_monotonic = float("-inf")
        self._pressed = False
        self._armed = False
        self._request_running = False
        self._estop_state_received = False
        self._estop_latched = True

        self._button_pub = rospy.Publisher(
            "~button_value", Float32, queue_size=5
        )
        self._armed_pub = rospy.Publisher(
            "~armed", Bool, queue_size=1, latch=True
        )
        self._state_pub = rospy.Publisher(
            "~state", String, queue_size=5, latch=True
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

        self._armed_pub.publish(Bool(data=False))
        self._publish_state("WAITING_FOR_SF_RELEASE")

        rospy.loginfo(
            "RadioMaster jump control ready: joy=%s SF=buttons[%d] "
            "service=%s cooldown=%.2fs dry_run=%s",
            self.joy_topic,
            self.button_index,
            self.jump_service,
            self.cooldown_s,
            self.dry_run,
        )

    def _publish_state(self, state: str):
        self._state_pub.publish(String(data=state))

    def _estop_cb(self, message: Bool):
        with self._lock:
            self._estop_state_received = True
            self._estop_latched = bool(message.data)

        if message.data:
            self._publish_state("BLOCKED_ESTOP_LATCHED")
        else:
            with self._lock:
                ready = self._armed and not self._request_running
            if ready:
                self._publish_state("READY")

    def _joy_cb(self, message: Joy):
        if self.button_index >= len(message.buttons):
            self._disarm("MAPPING_INVALID")
            rospy.logerr_throttle(
                2.0,
                "RadioMaster jump mapping invalid: buttons[%d] requested, "
                "but /joy contains only %d buttons",
                self.button_index,
                len(message.buttons),
            )
            return

        now = time.monotonic()
        value = float(message.buttons[self.button_index])
        self._button_pub.publish(Float32(data=value))

        with self._lock:
            connection_gap = (
                self._last_joy_monotonic is None
                or now - self._last_joy_monotonic > self.input_timeout_s
            )
            self._last_joy_monotonic = now

            if connection_gap:
                # A reconnect while SF is already held must never create a jump.
                self._armed = False
                self._pressed = value >= self.trigger_threshold

            if value <= self.release_threshold:
                self._pressed = False
                if not self._armed:
                    self._armed = True
                    self._armed_pub.publish(Bool(data=True))
                if not self._request_running:
                    if (
                        self.require_estop_state
                        and not self._estop_state_received
                    ):
                        self._publish_state("WAITING_FOR_ESTOP_STATE")
                    elif self._estop_latched:
                        self._publish_state("BLOCKED_ESTOP_LATCHED")
                    else:
                        self._publish_state("READY")
                return

            if value < self.trigger_threshold:
                return

            rising_edge = not self._pressed
            self._pressed = True
            if not rising_edge or not self._armed:
                return

            # Consume this edge. A physical release is required to re-arm.
            self._armed = False
            self._armed_pub.publish(Bool(data=False))

            if self._request_running:
                self._publish_state("BLOCKED_REQUEST_RUNNING")
                return
            if (
                self.require_estop_state
                and not self._estop_state_received
            ):
                self._publish_state("BLOCKED_NO_ESTOP_STATE")
                rospy.logwarn("SF jump rejected: E-stop state is not available.")
                return
            if self._estop_latched:
                self._publish_state("BLOCKED_ESTOP_LATCHED")
                rospy.logwarn("SF jump rejected: RadioMaster E-stop is latched.")
                return
            if now - self._last_request_monotonic < self.cooldown_s:
                self._publish_state("BLOCKED_COOLDOWN")
                rospy.logwarn("SF jump rejected: jump-command cooldown is active.")
                return

            self._request_running = True
            self._last_request_monotonic = now

        if self.dry_run:
            rospy.logwarn(
                "[DRY RUN] SF / buttons[%d] would call %s",
                self.button_index,
                self.jump_service,
            )
            self._publish_state("DRY_RUN_JUMP_REQUEST")
            with self._lock:
                self._request_running = False
            return

        self._publish_state("CALLING_JUMP_SERVICE")
        threading.Thread(
            target=self._call_jump_service,
            name="radiomaster-jump-service",
            daemon=True,
        ).start()

    def _disarm(self, state: str):
        with self._lock:
            self._armed = False
            self._pressed = False
        self._armed_pub.publish(Bool(data=False))
        self._publish_state(state)

    def _call_jump_service(self):
        try:
            rospy.wait_for_service(
                self.jump_service,
                timeout=self.service_wait_s,
            )
            response = rospy.ServiceProxy(
                self.jump_service,
                Trigger,
            )(TriggerRequest())

            if response.success:
                self._publish_state("JUMP_ACCEPTED")
                rospy.logwarn(
                    "SF jump command accepted by %s: %s",
                    self.jump_service,
                    response.message,
                )
            else:
                self._publish_state("JUMP_REJECTED")
                rospy.logerr(
                    "SF jump command rejected by %s: %s",
                    self.jump_service,
                    response.message,
                )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self._publish_state("JUMP_SERVICE_ERROR")
            rospy.logerr(
                "Cannot call jump service %s from SF: %s",
                self.jump_service,
                exc,
            )
        finally:
            with self._lock:
                self._request_running = False


def main():
    rospy.init_node("radiomaster_jump")
    RadioMasterJumpNode()
    rospy.spin()


if __name__ == "__main__":
    main()