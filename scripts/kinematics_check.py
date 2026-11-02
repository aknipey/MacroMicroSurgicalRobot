#!/usr/bin/env python3
"""How well a kinematics file reproduces the UR controller's own tool pose, from a run's rosbag.

    python3 scripts/kinematics_check.py logs/V8/CalibratedKinematics/FastCheckingRCMSimpleHolder_Replay1
    python3 scripts/kinematics_check.py <run> --kinematics config/calibrations/ur16e_kinematics_2026-11-03.yaml \
        /opt/ros/jazzy/share/ur_description/config/ur16e/default_kinematics.yaml

/joint_states and /tcp_pose_broadcaster/pose come from the same 500 Hz RTDE packet (stamped
~0.04 ms apart), so tool0 computed from the joint angles through a kinematics file can be compared
with the controller's TCP sample by sample, with no TF timing involved. Unlike META's
tcp_offset.check_max_mm, which compares against TF interpolated from ~20 Hz and so mostly measures
timing error while moving (FastChecking: 1.1-1.4 mm with either model).

The pendant TCP is a fixed tool-frame offset, so it shows as a constant distance. What matters is the
*spread* (max - min) of the distance and the rotation error: ~0 when the file matches the controller.
First check, 2026-11-03 FastChecking replay: calibrated spread 0.000 mm / 0.000 deg, nominal UR16e
0.847 mm / 0.374 deg (docs/UR_CALIBRATION_FIX.md).
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LINKS = ["shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"]
JOINTS = [f"{j}_joint" for j in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
PAIR_TOLERANCE_NS = 500_000     # joint state and TCP sample from the same RTDE packet
EVERY = 5                       # use every 5th TCP sample (100 Hz is plenty)


def transform(rpy=(0.0, 0.0, 0.0), xyz=(0.0, 0.0, 0.0)):
    """4x4 from URDF <origin rpy xyz>: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
    m[:3, 3] = xyz
    return m


# Fixed joints of ur_description's ur_macro.xacro around the six kinematics links
BASE_LINK_TO_INERTIA = transform((0, 0, np.pi))
BASE_LINK_TO_BASE = transform((0, 0, np.pi))           # 'base' = the controller's base frame
WRIST3_TO_TOOL0 = transform((0, -np.pi / 2, -np.pi / 2)) @ transform((np.pi / 2, 0, np.pi / 2))  # via flange


def tool0_in_base(kinematics, q):
    """tool0 in the controller's 'base' frame, built the way ur_description builds the URDF."""
    m = BASE_LINK_TO_INERTIA.copy()
    for name, angle in zip(LINKS, q):
        link = kinematics[name]
        m = m @ transform((link["roll"], link["pitch"], link["yaw"]), (link["x"], link["y"], link["z"]))
        m = m @ transform((0, 0, angle))
    return np.linalg.inv(BASE_LINK_TO_BASE) @ m @ WRIST3_TO_TOOL0


def read_bag(path):
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions("", ""))
    joints, tcp = {}, {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            positions = dict(zip(msg.name, msg.position))
            if all(j in positions for j in JOINTS):
                joints[msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec] = [positions[j] for j in JOINTS]
        elif topic == "/tcp_pose_broadcaster/pose":
            msg = deserialize_message(data, PoseStamped)
            p, o = msg.pose.position, msg.pose.orientation
            tcp[msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec] = (
                np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w]))
    return joints, tcp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="run folder (with rosbag/) or a rosbag folder")
    ap.add_argument("--kinematics", nargs="+",
                    help="kinematics YAMLs to compare (default: the run's META kinematics file + nominal UR16e)")
    args = ap.parse_args()

    bag = os.path.join(args.run, "rosbag") if os.path.isdir(os.path.join(args.run, "rosbag")) else args.run
    files = args.kinematics
    if not files:
        files = []
        meta = os.path.join(args.run, "META.yaml")
        if os.path.exists(meta):
            used = (yaml.safe_load(open(meta)).get("kinematics") or {}).get("file")
            if used:
                files.append(used)
        nominal = "/opt/ros/jazzy/share/ur_description/config/ur16e/default_kinematics.yaml"
        if nominal not in files:
            files.append(nominal)

    joints, tcp = read_bag(bag)
    if not joints or not tcp:
        sys.exit(f"{bag}: needs /joint_states and /tcp_pose_broadcaster/pose")
    stamps = np.array(sorted(joints))
    pairs = []
    for stamp in sorted(tcp)[::EVERY]:
        i = np.searchsorted(stamps, stamp)
        nearest = min((stamps[k] for k in (i - 1, i) if 0 <= k < len(stamps)), key=lambda s: abs(s - stamp))
        if abs(nearest - stamp) < PAIR_TOLERANCE_NS:
            pairs.append((stamp, nearest))
    print(f"{bag}\n  {len(pairs)} joint-state / controller-TCP pairs")

    for path in files:
        kinematics = yaml.safe_load(open(path))["kinematics"]
        dist, rot = [], []
        for stamp, joint_stamp in pairs:
            m = tool0_in_base(kinematics, joints[joint_stamp])
            position, rotation = tcp[stamp]
            dist.append(1e3 * np.linalg.norm(m[:3, 3] - position))
            rot.append(np.degrees((Rotation.from_matrix(m[:3, :3]).inv() * rotation).magnitude()))
        dist, rot = np.array(dist), np.array(rot)
        print(f"  {os.path.basename(path):34s} spread {dist.max() - dist.min():6.3f} mm, rotation error "
              f"max {rot.max():.3f} deg  (constant part {np.median(dist):.1f} mm = pendant TCP)")


if __name__ == "__main__":
    main()
