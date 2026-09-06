#!/usr/bin/env python3

import argparse
import csv
import math
import threading

import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Float64MultiArray, String


FIELDS = ["time", "x", "y", "z", "vx", "vy", "vz", "speed"]
INTENT_FIELDS = ["time", "vx", "vy", "vz", "speed"]
FORCE_FIELDS = ["time", "fx", "fy", "fz", "magnitude"]
EVENT_FIELDS = ["time", "event"]
FIELD_DIAGNOSTIC_FIELDS = [
    "time", "x", "y", "z", "ux", "uy", "uz", "potential",
    "field_valid", "potential_valid", "clearance", "intent_vx",
    "intent_vy", "intent_vz", "anchor_x", "anchor_y", "anchor_z",
    "anchor_heading", "reanchor_count"
    , "streamline_eta", "streamline_valid", "streamline_mode", "streamline_end_cap",
    "streamline_version", "streamline_update_disp_m",
    "projection_x", "projection_y", "projection_z",
    "e_perp_x", "e_perp_y", "e_perp_z", "eta_xy", "eta_3d",
    "streamline_clearance_min_m", "projection_clearance_m", "d_v_mps"
]
REANCHOR_FIELDS = ["time", "count", "x", "y", "z", "heading"]


class TrajectoryRecorder:
    def __init__(self, actual_path, planned_path, intent_path, force_path, event_path,
                 field_path, reanchor_path):
        self.lock = threading.Lock()
        self.closed = False
        self.actual_file = open(actual_path, "w", newline="", encoding="utf-8")
        self.planned_file = open(planned_path, "w", newline="", encoding="utf-8")
        self.intent_file = open(intent_path, "w", newline="", encoding="utf-8")
        self.force_file = open(force_path, "w", newline="", encoding="utf-8")
        self.event_file = open(event_path, "w", newline="", encoding="utf-8")
        self.field_file = open(field_path, "w", newline="", encoding="utf-8")
        self.reanchor_file = open(reanchor_path, "w", newline="", encoding="utf-8")
        self.actual_writer = csv.writer(self.actual_file)
        self.planned_writer = csv.writer(self.planned_file)
        self.intent_writer = csv.writer(self.intent_file)
        self.force_writer = csv.writer(self.force_file)
        self.event_writer = csv.writer(self.event_file)
        self.field_writer = csv.writer(self.field_file)
        self.reanchor_writer = csv.writer(self.reanchor_file)
        self.actual_writer.writerow(FIELDS)
        self.planned_writer.writerow(FIELDS)
        self.intent_writer.writerow(INTENT_FIELDS)
        self.force_writer.writerow(FORCE_FIELDS)
        self.event_writer.writerow(EVENT_FIELDS)
        self.field_writer.writerow(FIELD_DIAGNOSTIC_FIELDS)
        self.reanchor_writer.writerow(REANCHOR_FIELDS)
        self.actual_file.flush()
        self.planned_file.flush()
        self.intent_file.flush()
        self.force_file.flush()
        self.event_file.flush()
        self.field_file.flush()
        self.reanchor_file.flush()

        rospy.Subscriber("/sim/odom", Odometry, self._odom_callback, queue_size=100)
        rospy.Subscriber(
            "/position_cmd", PositionCommand, self._command_callback, queue_size=100
        )
        rospy.Subscriber(
            "/human_intent", TwistStamped, self._intent_callback, queue_size=100
        )
        rospy.Subscriber(
            "/quadrotor_simulator_so3/force_disturbance", Vector3,
            self._force_callback, queue_size=100
        )
        rospy.Subscriber(
            "/paper/scenario_events", String, self._event_callback, queue_size=100
        )
        rospy.Subscriber(
            "/paper/gvf_field_diagnostics", Float64MultiArray,
            self._field_callback, queue_size=100
        )
        rospy.Subscriber(
            "/paper/gvf_reanchor_events", Float64MultiArray,
            self._reanchor_callback, queue_size=100
        )
        rospy.on_shutdown(self.close)

    @staticmethod
    def _value(value):
        value = float(value)
        if not math.isfinite(value):
            return ""
        return "{:.9f}".format(value)

    def _write(self, writer, stream, stamp, position, velocity):
        values = [float(velocity.x), float(velocity.y), float(velocity.z)]
        speed = math.sqrt(sum(value * value for value in values))
        row = [
            self._value(stamp.to_sec()),
            self._value(position.x),
            self._value(position.y),
            self._value(position.z),
            self._value(values[0]),
            self._value(values[1]),
            self._value(values[2]),
            self._value(speed),
        ]
        with self.lock:
            if self.closed:
                return
            writer.writerow(row)
            stream.flush()

    def _odom_callback(self, msg):
        self._write(
            self.actual_writer,
            self.actual_file,
            msg.header.stamp,
            msg.pose.pose.position,
            msg.twist.twist.linear,
        )

    def _command_callback(self, msg):
        self._write(
            self.planned_writer,
            self.planned_file,
            msg.header.stamp,
            msg.position,
            msg.velocity,
        )

    def _intent_callback(self, msg):
        values = [
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        ]
        speed = math.sqrt(sum(value * value for value in values))
        row = [
            self._value(msg.header.stamp.to_sec()),
            self._value(values[0]),
            self._value(values[1]),
            self._value(values[2]),
            self._value(speed),
        ]
        with self.lock:
            if self.closed:
                return
            self.intent_writer.writerow(row)
            self.intent_file.flush()

    def _force_callback(self, msg):
        values = [float(msg.x), float(msg.y), float(msg.z)]
        magnitude = math.sqrt(sum(value * value for value in values))
        row = [self._value(rospy.Time.now().to_sec())] + \
            [self._value(value) for value in values] + [self._value(magnitude)]
        with self.lock:
            if self.closed:
                return
            self.force_writer.writerow(row)
            self.force_file.flush()

    def _event_callback(self, msg):
        with self.lock:
            if self.closed:
                return
            self.event_writer.writerow([self._value(rospy.Time.now().to_sec()), msg.data])
            self.event_file.flush()

    def _field_callback(self, msg):
        if len(msg.data) < len(FIELD_DIAGNOSTIC_FIELDS):
            return
        with self.lock:
            if self.closed:
                return
            self.field_writer.writerow([self._value(value) for value in msg.data[:len(FIELD_DIAGNOSTIC_FIELDS)]])
            self.field_file.flush()

    def _reanchor_callback(self, msg):
        if len(msg.data) < len(REANCHOR_FIELDS):
            return
        with self.lock:
            if self.closed:
                return
            self.reanchor_writer.writerow([self._value(value) for value in msg.data[:len(REANCHOR_FIELDS)]])
            self.reanchor_file.flush()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.actual_file.close()
            self.planned_file.close()
            self.intent_file.close()
            self.force_file.close()
            self.event_file.close()
            self.field_file.close()
            self.reanchor_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--actual", required=True)
    parser.add_argument("--planned", required=True)
    parser.add_argument("--intent", required=True)
    parser.add_argument("--force", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--field", required=True)
    parser.add_argument("--reanchors", required=True)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("benchmark_trajectory_recorder")
    TrajectoryRecorder(args.actual, args.planned, args.intent, args.force, args.events,
                       args.field, args.reanchors)
    rospy.spin()


if __name__ == "__main__":
    main()
