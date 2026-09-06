#!/usr/bin/env python3
"""Replay staged point-cloud maps and deterministic simulator force inputs.

This node is intentionally independent of the planner. It publishes the same
world-frame map topic consumed by local_sensing and the existing simulator's
private force-disturbance topic. Each event is held until the next event.

The actual applied force is published on the simulator topic and rendered as
a marker attached to the latest odometry position, so the on-screen arrow and
the recorded force topic always agree. The `--force-event` steps are held
piecewise-constant until the next event. `--open-force-event` is a deterministic
variant that keeps a fixed magnitude but chooses its horizontal direction at
the event time by looking for the clearest local sector in an ASCII PCD map.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker


def read_ascii_pcd(path: Path):
    lines = path.read_text(encoding="ascii").splitlines()
    data_index = None
    for index, line in enumerate(lines):
        if line.strip().upper() == "DATA ASCII":
            data_index = index + 1
            break
    if data_index is None:
        raise ValueError(f"{path} is not an ASCII PCD")
    points = []
    for line in lines[data_index:]:
        fields = line.split()
        if len(fields) < 3:
            continue
        x, y, z = (float(fields[0]), float(fields[1]), float(fields[2]))
        if all(math.isfinite(value) for value in (x, y, z)):
            points.append((x, y, z))
    if not points:
        raise ValueError(f"{path} contains no points")
    return points


def parse_map_stage(value):
    stamp, path = value.split(":", 1)
    return float(stamp), Path(path).resolve()


def parse_force_event(value):
    stamp, vector = value.split(":", 1)
    values = tuple(float(item) for item in vector.split(","))
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError(f"invalid force event: {value}")
    return float(stamp), values


def parse_open_force_event(value):
    stamp, magnitude = value.split(":", 1)
    magnitude = float(magnitude)
    if not math.isfinite(magnitude) or magnitude < 0.0:
        raise ValueError(f"invalid open-area force event: {value}")
    return float(stamp), magnitude


def zero_force(force):
    return all(abs(component) < 1e-9 for component in force)


class ForceMarker:
    """Display the currently applied world-frame external force at the UAV."""

    def __init__(self, topic, scale, odom_state):
        self._scale = scale
        self._odom_state = odom_state
        self._visible = False
        self._publisher = rospy.Publisher(topic, Marker, queue_size=1, latch=True)

    @staticmethod
    def _base_marker(stamp, marker_id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "external_force_disturbance"
        marker.id = marker_id
        marker.pose.orientation.w = 1.0
        return marker

    def publish(self, force):
        stamp = rospy.Time.now()
        position = self._odom_state.position
        if position is None or zero_force(force):
            if self._visible:
                marker = self._base_marker(stamp, 0)
                marker.action = Marker.DELETE
                self._publisher.publish(marker)
                self._visible = False
            return

        start = Point(position.x, position.y, position.z + 0.35)
        magnitude = math.sqrt(sum(component * component for component in force))
        # The arrow is deliberately scaled in metres per Newton, rather than
        # normalized, so the magnitude is visually comparable across trials.
        draw_length = max(0.18, min(1.8, self._scale * magnitude))
        end = Point(start.x + draw_length * force[0] / magnitude,
                    start.y + draw_length * force[1] / magnitude,
                    start.z + draw_length * force[2] / magnitude)

        arrow = self._base_marker(stamp, 0)
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.055
        arrow.scale.y = 0.13
        arrow.scale.z = 0.18
        arrow.color.r = 1.0
        arrow.color.g = 0.58
        arrow.color.b = 0.05
        arrow.color.a = 0.98
        arrow.points = [start, end]
        self._publisher.publish(arrow)
        self._visible = True


class OdomState:
    """Latest odometry snapshot shared by the force selector and marker."""

    def __init__(self):
        self.position = None
        self.velocity = None
        self._odom_sub = rospy.Subscriber("/sim/odom", Odometry, self._callback,
                                          queue_size=1)

    def _callback(self, msg):
        self.position = msg.pose.pose.position
        self.velocity = msg.twist.twist.linear


class OpenAreaForceSelector:
    """Choose a deterministic horizontal force direction with local clearance.

    The generated map contains sampled obstacle surfaces rather than a signed
    distance field.  For each candidate heading we therefore measure the
    nearest point in a forward cone, ignore the floor samples, and also treat
    the PCD's horizontal extent as a soft virtual boundary.  A preferred
    direction (the current cruise direction in the paper scene) only breaks
    near-ties, so a genuinely more open side still wins.
    """

    def __init__(self, points, ray_count=32, max_range=5.0,
                 cone_half_angle_deg=35.0, floor_z=0.15,
                 vertical_window=0.65, preferred=None, grid_size=0.5):
        self._ray_count = int(ray_count)
        self._max_range = float(max_range)
        self._cone_tan = math.tan(math.radians(float(cone_half_angle_deg)))
        self._floor_z = float(floor_z)
        self._vertical_window = float(vertical_window)
        self._grid_size = float(grid_size)
        self._preferred = self._normalize(preferred)
        self._grid = {}
        self._point_count = 0
        self._bounds = None
        self.set_points(points)

    @staticmethod
    def _normalize(direction):
        if direction is None:
            return None
        dx, dy = float(direction[0]), float(direction[1])
        norm = math.hypot(dx, dy)
        if not math.isfinite(norm) or norm < 1e-9:
            return None
        return dx / norm, dy / norm

    def set_points(self, points):
        # Keep only finite, non-floor samples.  Floor points are not useful for
        # selecting a horizontal side and can otherwise dominate every ray.
        if self._grid_size <= 0.0 or not math.isfinite(self._grid_size):
            raise ValueError("grid_size must be finite and positive")
        filtered = [(float(x), float(y), float(z))
                    for x, y, z in points
                    if all(math.isfinite(value) for value in (x, y, z))
                    and float(z) >= self._floor_z]
        self._point_count = len(filtered)
        self._grid = {}
        for point in filtered:
            cell = (math.floor(point[0] / self._grid_size),
                    math.floor(point[1] / self._grid_size))
            self._grid.setdefault(cell, []).append(point)
        if filtered:
            self._bounds = (min(point[0] for point in filtered),
                            max(point[0] for point in filtered),
                            min(point[1] for point in filtered),
                            max(point[1] for point in filtered))
        else:
            self._bounds = None

    def _nearby_points(self, position):
        """Return only map samples in the selector's local search disk.

        The map is static, so a uniform grid is enough here and avoids an
        external dependency such as scipy.  The old implementation visited
        every PCD point for every ray (about 2.7 million comparisons per
        event); this query usually returns a few thousand points instead.
        """
        radius = self._max_range
        cell_min_x = math.floor((position.x - radius) / self._grid_size)
        cell_max_x = math.floor((position.x + radius) / self._grid_size)
        cell_min_y = math.floor((position.y - radius) / self._grid_size)
        cell_max_y = math.floor((position.y + radius) / self._grid_size)
        points = []
        radius_sq = radius * radius
        for cell_x in range(cell_min_x, cell_max_x + 1):
            for cell_y in range(cell_min_y, cell_max_y + 1):
                for px, py, pz in self._grid.get((cell_x, cell_y), ()):
                    rel_x, rel_y = px - position.x, py - position.y
                    if rel_x * rel_x + rel_y * rel_y <= radius_sq:
                        if abs(pz - position.z) <= self._vertical_window:
                            points.append((rel_x, rel_y))
        return points

    def _boundary_clearance(self, position, direction):
        if self._bounds is None:
            return self._max_range
        x, y = position.x, position.y
        dx, dy = direction
        distances = []
        if dx > 1e-9:
            distances.append((self._bounds[1] - x) / dx)
        elif dx < -1e-9:
            distances.append((self._bounds[0] - x) / dx)
        if dy > 1e-9:
            distances.append((self._bounds[3] - y) / dy)
        elif dy < -1e-9:
            distances.append((self._bounds[2] - y) / dy)
        positive = [distance for distance in distances if distance > 0.0]
        if not positive:
            return self._max_range
        # Leave a small margin from the edge; this is only a tie-breaker for
        # open sectors, not a replacement for the simulator's collision model.
        return max(0.0, min(self._max_range, min(positive) - 0.8))

    def choose(self, position, velocity=None):
        if position is None:
            fallback = self._preferred or (1.0, 0.0)
            return fallback, self._max_range, 0

        headings = [(2.0 * math.pi * index / self._ray_count)
                    for index in range(self._ray_count)]
        clearances = []
        nearby_points = self._nearby_points(position)
        for heading in headings:
            direction = (math.cos(heading), math.sin(heading))
            nearest = self._boundary_clearance(position, direction)
            for rel_x, rel_y in nearby_points:
                forward = rel_x * direction[0] + rel_y * direction[1]
                if forward <= 0.0 or forward >= nearest:
                    continue
                lateral = abs(-rel_y * direction[0] + rel_x * direction[1])
                # Use a small absolute width near the vehicle and a cone farther
                # out so sparse PCD sampling cannot create a false open ray.
                if lateral <= max(0.45, forward * self._cone_tan):
                    nearest = forward
            clearances.append(nearest)

        best_clearance = max(clearances)
        # Select the genuinely clearest sector.  A tiny deterministic angular
        # tie-break keeps repeated runs reproducible without biasing the force
        # toward the vehicle's cruise direction; consequently the two pulse
        # forces can point to different open sides as the local scene changes.
        best_index = max(range(len(clearances)),
                         key=lambda index: (clearances[index], -index))
        direction = (math.cos(headings[best_index]), math.sin(headings[best_index]))
        return direction, clearances[best_index], best_index


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-stage", action="append", default=[],
                        help="time:path.pcd; repeat for each replacement map")
    parser.add_argument("--force-event", action="append", default=[],
                        help="time:fx,fy,fz; repeat for each force change")
    parser.add_argument("--open-force-event", action="append", default=[],
                        help="time:magnitude; choose a horizontal direction with the "
                             "largest local PCD clearance at the event")
    parser.add_argument("--obstacle-pcd", default="",
                        help="ASCII PCD used by --open-force-event")
    parser.add_argument("--open-force-ray-count", type=int, default=32,
                        help="number of horizontal sectors sampled for open-area forces")
    parser.add_argument("--open-force-max-range", type=float, default=5.0,
                        help="maximum local clearance range in metres")
    parser.add_argument("--open-force-cone-half-angle", type=float, default=35.0,
                        help="half-angle of each horizontal clearance cone in degrees")
    parser.add_argument("--open-force-vertical-window", type=float, default=0.65,
                        help="vertical distance from the UAV used for horizontal clearance")
    parser.add_argument("--open-force-floor-z", type=float, default=0.15,
                        help="ignore PCD samples below this z when selecting a side")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--map-topic", default="/mock_map")
    parser.add_argument("--force-topic",
                        default="/quadrotor_simulator_so3/force_disturbance")
    parser.add_argument("--force-marker-topic", default="/paper/force_disturbance_marker")
    parser.add_argument("--force-marker-scale", type=float, default=0.45,
                        help="RViz arrow length in metres per Newton")
    args = parser.parse_args()

    map_stages = sorted(parse_map_stage(value) for value in args.map_stage)
    fixed_force_events = [(stamp, values, "fixed")
                          for stamp, values in
                          (parse_force_event(value) for value in args.force_event)]
    open_force_events = [(stamp, magnitude, "open")
                         for stamp, magnitude in
                         (parse_open_force_event(value)
                          for value in args.open_force_event)]
    force_events = sorted(fixed_force_events + open_force_events, key=lambda event: event[0])
    if not map_stages and not force_events:
        raise SystemExit("at least one map stage or force event is required")
    if open_force_events and not args.obstacle_pcd:
        raise SystemExit("--obstacle-pcd is required with --open-force-event")
    if args.open_force_ray_count < 8:
        raise SystemExit("--open-force-ray-count must be at least 8")
    for value, name in ((args.open_force_max_range, "--open-force-max-range"),
                        (args.open_force_cone_half_angle, "--open-force-cone-half-angle"),
                        (args.open_force_vertical_window, "--open-force-vertical-window"),
                        (args.open_force_floor_z, "--open-force-floor-z")):
        if not math.isfinite(value) or value <= 0.0:
            raise SystemExit(f"{name} must be finite and positive")
    event_times = [event[0] for event in force_events]
    if any(not math.isfinite(stamp) or stamp < 0.0 for stamp in event_times):
        raise SystemExit("force event times must be finite and non-negative")
    if len(set(event_times)) != len(event_times):
        raise SystemExit("force event times must be unique")
    for previous, current in zip(force_events, force_events[1:]):
        if current[0] <= previous[0]:
            raise SystemExit("force events must be strictly increasing")
    if map_stages and map_stages[0][0] > 0.25:
        raise SystemExit("the first map stage must start at t=0")
    if args.duration <= 0.0 or not math.isfinite(args.duration):
        raise SystemExit("--duration must be finite and positive")
    if not math.isfinite(args.force_marker_scale) or args.force_marker_scale <= 0.0:
        raise SystemExit("--force-marker-scale must be finite and positive")

    map_points = [(stamp, read_ascii_pcd(path), path)
                  for stamp, path in map_stages]
    rospy.init_node("paper_scenario_replay")
    map_pub = rospy.Publisher(args.map_topic, PointCloud2, queue_size=1, latch=True)
    force_pub = rospy.Publisher(args.force_topic, Vector3, queue_size=1)
    event_pub = rospy.Publisher("/paper/scenario_events", String, queue_size=10)
    odom_state = OdomState()
    force_marker = ForceMarker(args.force_marker_topic, args.force_marker_scale, odom_state)
    selector = None
    if open_force_events:
        selector = OpenAreaForceSelector(
            read_ascii_pcd(Path(args.obstacle_pcd).resolve()),
            ray_count=args.open_force_ray_count,
            max_range=args.open_force_max_range,
            cone_half_angle_deg=args.open_force_cone_half_angle,
            floor_z=args.open_force_floor_z,
            vertical_window=args.open_force_vertical_window,
            preferred=(0.0, -1.0),
        )
    cached_maps = [(stamp, point_cloud2.create_cloud_xyz32(
        Header(stamp=rospy.Time.now(), frame_id="world"), points), path)
        for stamp, points, path in map_points]

    # Wait briefly for local_sensing/simulator subscriptions without delaying
    # the scenario clock indefinitely when a topic is intentionally unused.
    rospy.sleep(0.2)
    start = rospy.Time.now()
    map_index = -1
    force_index = -1
    rate = rospy.Rate(20.0)
    zero = Vector3()
    last_force = (0.0, 0.0, 0.0)
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed > args.duration:
            break

        while map_index + 1 < len(cached_maps) and elapsed >= cached_maps[map_index + 1][0]:
            map_index += 1
            stamp, msg, path = cached_maps[map_index]
            map_pub.publish(msg)
            rospy.loginfo("[paper_replay] map_update t=%.3f stage=%.3f path=%s",
                          elapsed, stamp, path)
            event_pub.publish(String(data=f"map_update t={elapsed:.3f} stage={stamp:.3f} path={path}"))

        while force_index + 1 < len(force_events) and elapsed >= force_events[force_index + 1][0]:
            force_index += 1
            event_stamp, event_value, event_kind = force_events[force_index]
            if event_kind == "open":
                direction, clearance, sector = selector.choose(
                    odom_state.position, odom_state.velocity)
                last_force = (event_value * direction[0],
                              event_value * direction[1], 0.0)
                rospy.loginfo(
                    "[paper_replay] open_force_update t=%.3f magnitude=%.3f "
                    "direction=(%.3f, %.3f) clearance=%.3f sector=%d",
                    elapsed, event_value, direction[0], direction[1], clearance, sector)
                event_pub.publish(String(
                    data=(f"open_force_update t={elapsed:.3f} stage={event_stamp:.3f} "
                          f"force=({last_force[0]:.6f},{last_force[1]:.6f},0.000000) "
                          f"clearance={clearance:.3f} sector={sector}")))
            else:
                last_force = event_value
                rospy.loginfo("[paper_replay] force_update t=%.3f value=%s",
                              elapsed, last_force)
                event_pub.publish(String(
                    data=f"force_update t={elapsed:.3f} stage={event_stamp:.3f} "
                         f"value={last_force}"))

        if map_index >= 0:
            stamp, msg, path = cached_maps[map_index]
            map_pub.publish(msg)

        total_force = last_force
        force_msg = zero if zero_force(total_force) else Vector3(*total_force)
        force_pub.publish(force_msg)
        force_marker.publish(total_force)
        rate.sleep()

    force_pub.publish(zero)
    force_marker.publish((0.0, 0.0, 0.0))
    rospy.loginfo("[paper_replay] complete duration=%.2fs maps=%d force_events=%d",
                  args.duration, len(cached_maps), len(force_events))


if __name__ == "__main__":
    main()
