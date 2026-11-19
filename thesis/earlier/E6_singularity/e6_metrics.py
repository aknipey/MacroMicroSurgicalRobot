#!/usr/bin/env python3
"""E6 metrics: one line per run, from the rosbag and CSVs, the same way guard on and off.

    source /opt/ros/jazzy/setup.bash
    python3 thesis/earlier/E6_singularity/e6_metrics.py logs/V9b/Singularity/*

c is computed from /joint_states with the D-H model, because the teleop only publishes
~/condition_number while the guard is on. Writes METRICS.csv next to this file.
"""
import csv
import glob
import importlib.util
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
spec = importlib.util.spec_from_file_location("singularity_input", os.path.join(PKG, "scripts/lab/singularity_input.py"))
si = importlib.util.module_from_spec(spec)
spec.loader.exec_module(si)
MOVING = 0.002      # m/s, tool speed above this counts as moving


def joint_states(run):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(run, "rosbag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/joint_states"]))
    t, q, qd = [], [], []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        m = deserialize_message(data, JointState)
        index = {n: i for i, n in enumerate(m.name)}
        t.append(stamp * 1e-9)
        q.append([m.position[index[j]] for j in si.JOINTS])
        qd.append([m.velocity[index[j]] for j in si.JOINTS])
    return np.array(t) - t[0], np.array(q), np.array(qd)


def metrics(run):
    t, q, qd = joint_states(run)
    step = max(1, int(0.01 / np.median(np.diff(t))))
    t, q, qd = t[::step], q[::step], qd[::step]
    tips = []
    conds = []
    for row in q:
        J, tip = si.jacobian(row)
        conds.append(np.linalg.cond(J))
        tips.append(tip)
    c, tips = np.array(conds), np.array(tips)
    dt = np.gradient(t)
    # Over 50 ms from the actual stamps: bag stamps bunch up, and differencing neighbours
    # turned a few ms of bunching into false 200 mm/s peaks
    later = np.minimum(np.searchsorted(t, t + 0.05), len(t) - 1)
    speed = np.linalg.norm(tips[later] - tips, axis=1) / np.maximum(t[later] - t, 1e-3)
    # Motion phases: first motion, furthest point, and how far back it came
    moving = np.where(speed > MOVING)[0]
    begin, end = moving[0], moving[-1]
    far = np.argmax(c)
    start_tip, end_tip = tips[begin], tips[end]
    w2 = np.degrees(np.abs(np.arctan2(np.sin(q[:, 4]), np.cos(q[:, 4]))))
    return {"run": "/".join(run.rstrip("/").split("/")[-2:]),
            "peak_c": round(float(c.max()), 1),
            "time_c_over_30_s": round(float(dt[c > 30].sum()), 1),
            "time_c_over_60_s": round(float(dt[c > 60].sum()), 1),
            "min_wrist2_deg": round(float(w2.min()), 2),
            "peak_joint_speed_deg_s": round(float(np.degrees(np.abs(qd).max())), 1),
            "peak_wrist_speed_joint": si.JOINTS[int(np.abs(qd).max(0).argmax())].replace("_joint", ""),
            "peak_tool_speed_mm_s": round(float(speed.max() * 1000), 1),
            "travel_in_mm": round(float(np.linalg.norm(tips[far] - start_tip) * 1000), 1),
            "return_gap_mm": round(float(np.linalg.norm(end_tip - start_tip) * 1000), 1),
            "height_drop_mm": round(float((start_tip[2] - tips[:, 2].min()) * 1000), 1),
            "tool_motion_s": round(float(dt[speed > MOVING].sum()), 1)}


def main():
    runs = [r for a in sys.argv[1:] for r in glob.glob(a) if os.path.isdir(os.path.join(r, "rosbag"))]
    rows = [metrics(r) for r in runs]
    for r in rows:
        print(r)
    if rows:
        with open(os.path.join(HERE, "METRICS.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, rows[0].keys())
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
