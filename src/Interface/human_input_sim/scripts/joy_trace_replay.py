#!/usr/bin/env python3

import math

import rospy
import yaml
from sensor_msgs.msg import Joy


class JoyTraceReplay:
    def __init__(self):
        trace_file = rospy.get_param("~trace_file")
        with open(trace_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        if not isinstance(config, dict):
            raise ValueError("trace root must be a mapping")

        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.publish_rate_hz = float(
            rospy.get_param("~publish_rate_hz", config.get("publish_rate_hz", 20.0))
        )
        self.axis_count = int(config.get("axis_count", 8))
        self.button_count = int(config.get("button_count", 11))
        self.forward_axis = int(config.get("forward_axis", 1))
        self.lateral_axis = int(config.get("lateral_axis", 0))
        self.post_hold_sec = float(config.get("post_hold_sec", 2.0))
        self.events = self._load_events(config.get("events"))
        self._validate_configuration()

        self.publisher = rospy.Publisher(self.joy_topic, Joy, queue_size=1)
        self.sequence = 0

    @staticmethod
    def _load_events(raw_events):
        if not isinstance(raw_events, list) or not raw_events:
            raise ValueError("events must be a non-empty list")

        events = []
        previous_time = -1.0
        for index, raw_event in enumerate(raw_events):
            if not isinstance(raw_event, dict):
                raise ValueError("event {} must be a mapping".format(index))
            event_time = float(raw_event["at"])
            forward = float(raw_event["forward"])
            lateral = float(raw_event["lateral"])
            values = (event_time, forward, lateral)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("event {} contains non-finite values".format(index))
            if event_time < 0.0 or event_time <= previous_time:
                raise ValueError("event times must be strictly increasing and non-negative")
            if abs(forward) > 1.0 or abs(lateral) > 1.0:
                raise ValueError("event {} axes must be in [-1, 1]".format(index))
            events.append(values)
            previous_time = event_time

        if abs(events[-1][1]) > 1e-9 or abs(events[-1][2]) > 1e-9:
            raise ValueError("the final event must return the joystick to zero")
        return events

    def _validate_configuration(self):
        if self.publish_rate_hz <= 0.0 or not math.isfinite(self.publish_rate_hz):
            raise ValueError("publish_rate_hz must be finite and positive")
        if self.axis_count <= 0 or self.button_count < 0:
            raise ValueError("axis_count must be positive and button_count non-negative")
        if not 0 <= self.forward_axis < self.axis_count:
            raise ValueError("forward_axis is outside the configured axes")
        if not 0 <= self.lateral_axis < self.axis_count:
            raise ValueError("lateral_axis is outside the configured axes")
        if self.forward_axis == self.lateral_axis:
            raise ValueError("forward_axis and lateral_axis must be different")
        if self.post_hold_sec < 0.0 or not math.isfinite(self.post_hold_sec):
            raise ValueError("post_hold_sec must be finite and non-negative")

    def run(self):
        wait_rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            rospy.loginfo_throttle(2.0, "[human_input] waiting for a /joy subscriber")
            wait_rate.sleep()
        if rospy.is_shutdown():
            return

        start_time = rospy.Time.now()
        event_index = 0
        current_event = self.events[0]
        end_time = self.events[-1][0] + self.post_hold_sec
        rate = rospy.Rate(self.publish_rate_hz)

        rospy.loginfo(
            "[human_input] replaying %d joystick events from %s at %.2f Hz",
            len(self.events),
            rospy.get_param("~trace_file"),
            self.publish_rate_hz,
        )

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            while event_index + 1 < len(self.events) and elapsed >= self.events[event_index + 1][0]:
                event_index += 1
                current_event = self.events[event_index]
                rospy.loginfo(
                    "[human_input] event %d at %.2f s: forward=%.2f lateral=%.2f",
                    event_index,
                    current_event[0],
                    current_event[1],
                    current_event[2],
                )

            self._publish(current_event[1], current_event[2])
            if elapsed >= end_time:
                break
            rate.sleep()

        rospy.loginfo("[human_input] baseline replay complete")

    def _publish(self, forward, lateral):
        msg = Joy()
        msg.header.seq = self.sequence
        msg.header.stamp = rospy.Time.now()
        msg.axes = [0.0] * self.axis_count
        msg.buttons = [0] * self.button_count
        msg.axes[self.forward_axis] = forward
        msg.axes[self.lateral_axis] = lateral
        self.sequence += 1
        self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("joy_trace_replay")
    try:
        JoyTraceReplay().run()
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        rospy.logfatal("[human_input] joystick replay failed: %s", exc)
        raise SystemExit(1)
