#!/usr/bin/env python3
"""Check a completed default benchmark, including the platform actually recorded.

Run inside the ROS container:
  python3 scripts/tests/check_benchmark_run.py 2d logs/sim_paper/2d/<run>
"""
import argparse
import csv
import json
import math
from pathlib import Path

import rosbag
import yaml


def check(mode, run):
    metadata = dict(line.split("=", 1) for line in
                    (run / "metadata.txt").read_text().splitlines() if "=" in line)
    assert metadata["mode"] == mode, metadata
    assert metadata["exit_code"] == metadata["plot_exit_code"] == "0", metadata
    planner = "test_gvf_diff_drive.launch" if mode == "2d" else "test_gvf_3d.launch"
    assert metadata["planner"] == planner, metadata
    for filename in ("benchmark_plot.png", "metrics.json", "summary.txt", "benchmark.bag",
                     "config/all_rosparams.yaml"):
        assert (run / filename).stat().st_size > 0, filename
    data = {}
    for name in ("actual_trajectory", "planned_command", "human_intent"):
        with (run / "data" / (name + ".csv")).open() as stream:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        assert len(rows) > 20, (name, len(rows))
        assert all(math.isfinite(v) for row in rows for v in row.values()), name
        assert max(row["speed"] for row in rows) > 0.1, name
        data[name] = rows
    metrics = json.loads((run / "metrics.json").read_text())
    assert metrics["actual_path_length_m"] > 1.0, metrics["actual_path_length_m"]
    assert max(row["speed"] for row in data["planned_command"][-10:]) < 0.01, "release did not stop commands"
    params = yaml.safe_load((run / "config/rosparams.yaml").read_text())
    assert bool(params["gvf"].get("fluid_solver_3d", False)) == (mode == "3d")
    with rosbag.Bag(str(run / "benchmark.bag")) as bag:
        topics = bag.get_type_and_topic_info().topics
        for topic in ("/sim/odom", "/human_intent", "/position_cmd", "/sim/local_map"):
            assert topic in topics and topics[topic].message_count > 20, topic
        if mode == "2d":
            assert max(abs(row["z"]) for row in data["actual_trajectory"]) < 1e-6
            commands = [msg for _, msg, _ in bag.read_messages(topics=["/cmd_vel"])]
            assert len(commands) > 20, "missing differential-drive bridge output"
            assert max(abs(msg.linear.x) for msg in commands) > 0.1
            assert max(abs(msg.angular.z) for msg in commands) > 0.1
            assert max(abs(msg.linear.x) for msg in commands) <= 1.15 + 1e-6
            assert max(abs(msg.angular.z) for msg in commands) <= 1.0 + 1e-6
        else:
            assert max(row["z"] for row in data["actual_trajectory"]) > 0.5
            assert max(abs(row["vz"]) for row in data["planned_command"]) > 0.01
    print("PASS {}: {:.2f} m, {} odometry samples, complete artifacts".format(
        mode, metrics["actual_path_length_m"], len(data["actual_trajectory"])))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("2d", "3d"))
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    check(args.mode, args.run_dir)
