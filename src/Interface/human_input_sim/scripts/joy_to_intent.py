#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class JoyToIntent:
    def __init__(self):
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.intent_topic = rospy.get_param("~intent_topic", "/human_intent")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.forward_axis = int(rospy.get_param("~forward_axis", 1))
        self.lateral_axis = int(rospy.get_param("~lateral_axis", 0))
        self.forward_sign = float(rospy.get_param("~forward_sign", 1.0))
        self.lateral_sign = float(rospy.get_param("~lateral_sign", 1.0))
        self.deadzone = float(rospy.get_param("~deadzone", 0.15))
        self.max_speed = float(rospy.get_param("~max_speed", 1.0))
        self.intent_rate_hz = float(rospy.get_param("~intent_rate_hz", 1.0))
        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.5))

        self._validate_parameters()

        self.lock = threading.Lock()
        self.latest_axes = None
        self.latest_receive_time = None
        self.sequence = 0
        self.stale_stop_sent = True
        self.received_first_joy = False

        self.intent_pub = rospy.Publisher(
            self.intent_topic, TwistStamped, queue_size=1, latch=True
        )
        self.joy_sub = rospy.Subscriber(
            self.joy_topic, Joy, self._joy_callback, queue_size=1
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.intent_rate_hz), self._publish_timer_callback
        )
        watchdog_period = min(0.1, max(0.02, self.joy_timeout / 5.0))
        self.watchdog_timer = rospy.Timer(
            rospy.Duration(watchdog_period), self._watchdog_callback
        )
        rospy.on_shutdown(self._publish_stop)

        self._publish_velocity(0.0, 0.0)
        rospy.loginfo(
            "[human_input] %s -> %s at %.2f Hz, axes forward=%d lateral=%d",
            self.joy_topic,
            self.intent_topic,
            self.intent_rate_hz,
            self.forward_axis,
            self.lateral_axis,
        )

    def _validate_parameters(self):
        if self.forward_axis < 0 or self.lateral_axis < 0:
            raise ValueError("axis indices must be non-negative")
        if self.forward_axis == self.lateral_axis:
            raise ValueError("forward_axis and lateral_axis must be different")
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        if self.max_speed < 0.0 or not math.isfinite(self.max_speed):
            raise ValueError("max_speed must be finite and non-negative")
        if self.intent_rate_hz <= 0.0 or not math.isfinite(self.intent_rate_hz):
            raise ValueError("intent_rate_hz must be finite and positive")
        if self.joy_timeout <= 0.0 or not math.isfinite(self.joy_timeout):
            raise ValueError("joy_timeout must be finite and positive")
        if not math.isfinite(self.forward_sign) or not math.isfinite(self.lateral_sign):
            raise ValueError("axis signs must be finite")

    def _joy_callback(self, msg):
        now = rospy.Time.now()
        with self.lock:
            publish_immediately = not self.received_first_joy
            self.latest_axes = list(msg.axes)
            self.latest_receive_time = now
            self.stale_stop_sent = False
            self.received_first_joy = True

        if publish_immediately:
            self._publish_current(now)

    def _publish_timer_callback(self, event):
        self._publish_current(event.current_real)

    def _watchdog_callback(self, event):
        with self.lock:
            last_receive = self.latest_receive_time
            stop_already_sent = self.stale_stop_sent

        if last_receive is None or stop_already_sent:
            return
        if (event.current_real - last_receive).to_sec() <= self.joy_timeout:
            return

        with self.lock:
            if self.stale_stop_sent:
                return
            self.stale_stop_sent = True
        rospy.logwarn("[human_input] joystick input timed out; publishing stop")
        self._publish_velocity(0.0, 0.0)

    def _publish_current(self, now):
        with self.lock:
            axes = None if self.latest_axes is None else list(self.latest_axes)
            last_receive = self.latest_receive_time

        if axes is None or last_receive is None:
            return
        if (now - last_receive).to_sec() > self.joy_timeout:
            return

        velocity = self._map_axes(axes)
        if velocity is None:
            with self.lock:
                self.stale_stop_sent = True
            self._publish_velocity(0.0, 0.0)
            return

        with self.lock:
            self.stale_stop_sent = False
        self._publish_velocity(*velocity)

    def _map_axes(self, axes):
        required_size = max(self.forward_axis, self.lateral_axis) + 1
        if len(axes) < required_size:
            rospy.logwarn_throttle(
                1.0,
                "[human_input] Joy has %d axes; at least %d are required",
                len(axes),
                required_size,
            )
            return None

        forward = self.forward_sign * float(axes[self.forward_axis])
        lateral = self.lateral_sign * float(axes[self.lateral_axis])
        if not math.isfinite(forward) or not math.isfinite(lateral):
            rospy.logwarn_throttle(1.0, "[human_input] Joy contains non-finite axes")
            return None

        forward = max(-1.0, min(1.0, forward))
        lateral = max(-1.0, min(1.0, lateral))
        magnitude = math.hypot(forward, lateral)
        if magnitude <= self.deadzone:
            return 0.0, 0.0

        limited_magnitude = min(1.0, magnitude)
        scaled_magnitude = (limited_magnitude - self.deadzone) / (1.0 - self.deadzone)
        scale = self.max_speed * scaled_magnitude / magnitude
        return forward * scale, lateral * scale

    def _publish_velocity(self, forward_velocity, lateral_velocity):
        msg = TwistStamped()
        msg.header.seq = self.sequence
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = forward_velocity
        msg.twist.linear.y = lateral_velocity
        self.sequence += 1
        self.intent_pub.publish(msg)

    def _publish_stop(self):
        try:
            self._publish_velocity(0.0, 0.0)
        except rospy.ROSException:
            pass


if __name__ == "__main__":
    rospy.init_node("joy_to_intent")
    try:
        JoyToIntent()
        rospy.spin()
    except (ValueError, rospy.ROSException) as exc:
        rospy.logfatal("[human_input] joy_to_intent failed: %s", exc)
