#!/usr/bin/env python3
"""Records UR16e trajectories (joint angles + tool pose) while you teleoperate.

Services (std_srvs/Trigger):
    /trajectory_recorder/start   begin a new recording
    /trajectory_recorder/stop    stop and save it to <output_dir>/<name>.yaml (+ .csv)

The keyboard teleop ('[' / ']') and Touch teleop (grey button) call these for you.
The tool path being recorded is published as nav_msgs/Path on
/trajectory_recorder/path for RViz. Recording state is published latched on
/trajectory_recorder/recording (std_msgs/String, JSON: {"recording": bool,
"run_dir": path or null, "start_unix": s}) so other recorders (e.g. the NDI tracker,
src/ndi_tracking) can write into the same run folder on the same time base.
With LOG_REQUIRE_NDI a recording is refused unless the NDI tracker follows this topic;
a tracker that drops out mid-run, or a run without NDI_PATH.csv, is logged as an error
and noted in META.yaml (`ndi`).
Saved files can be replayed with trajectory_player.py.

While recording, full-rate path logs are also written to <log_dir>/NEW_RUN/
(see LOG_DIR in teleop_settings.py). Every sample is stamped on arrival with
this node's clock (`t`, s since start); the sender's own stamp is kept as `stamp`.

    TOUCH_PATH.csv   Touch target in the robot frame (base_link, tool0), one row
                     per stylus sample while WHITE is held (from the teleop node)
    STYLUS_RAW.csv   raw stylus pose as published (mm, device frame) + buttons
    UR_PATH.csv      the UR's own TCP pose (tcp_pose_broadcaster, RTDE, ~500 Hz),
                     converted to base_link and from the pendant TCP back to
                     tool0 (offset measured against TF at the start of each
                     recording, saved in META.yaml); TF tool0 at
                     LOG_TF_FALLBACK_RATE if the broadcaster is not running
                     (mock hardware)
    TRAJECTORY.yaml  the replayable trajectory (+ .csv), same as in output_dir
    META.yaml        date, code version, settings, row counts
    rosbag/          ros2 bag (mcap) of LOG_BAG_TOPICS, for replaying raw topics
                     (ros2 bag play logs/NEW_RUN/rosbag); its output is in rosbag.log

NEW_RUN is deleted when the next recording starts - rename the folder to keep
it. With LOG_KEEP_ALL_RUNS each run is also copied to <log_dir>/runs/<date_time>/.
Plot with thesis/run_tools/plot_paths.py.
"""

import csv
import hashlib
import json
import os
import shutil
import signal
import subprocess
from collections import deque
from datetime import datetime

import numpy as np

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from omni_msgs.msg import OmniState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from macro_micro_robot import teleop_settings as cfg
from macro_micro_robot.run_logger import CsvStream
from macro_micro_robot.ur16e_common import ndi_tracker_listening

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

POSE_COLUMNS = ["x", "y", "z", "qx", "qy", "qz", "qw"]
# RTDE samples compared with TF to measure the pendant TCP offset at recording start
TCP_OFFSET_SAMPLES = 50
# Afterwards every TCP_CHECK_EVERY-th RTDE sample is compared again: the offset is rigid,
# so drifting by more than TCP_CHECK_TOLERANCE means TF's kinematics are not this arm's
# (e.g. launched with the wrong ur_type)
TCP_CHECK_EVERY = 25
TCP_CHECK_TOLERANCE = 0.005     # m
# RTDE samples wait this long (s) for TF to reach their stamp before being dropped
TCP_QUEUE_AGE = 1.0
RUN_FOLDER = "NEW_RUN"


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def short_path(path):
    home = os.path.expanduser("~")
    return "~" + path[len(home):] if path.startswith(home) else path


def git_version(path):
    """'<commit>' or '<commit>-dirty' for the repo containing path, or 'unknown'."""
    def git(*args):
        return subprocess.run(["git", "-C", path, *args], capture_output=True, text=True,
                              timeout=2.0, check=True).stdout.strip()
    try:
        commit = git("rev-parse", "--short", "HEAD")
        return commit + ("-dirty" if git("status", "--porcelain", "--untracked-files=no") else "")
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def kinematics_summary(path):
    """Which kinematic model TF used: the kinematics_params_file, its sha256 and calibration hash, and
    whether it is a nominal (ur_description default) or a robot calibration (docs/UR_CALIBRATION_FIX.md)."""
    if not path:
        return {"file": None, "model": "unknown (launch did not pass kinematics_params_file)"}
    summary = {"file": path, "model": "nominal" if "default_kinematics" in path else "calibrated"}
    try:
        with open(path, "rb") as f:
            summary["sha256"] = hashlib.sha256(f.read()).hexdigest()
        with open(path) as f:
            summary["hash"] = (yaml.safe_load(f).get("kinematics") or {}).get("hash")
    except (OSError, yaml.YAMLError, AttributeError) as e:
        summary["error"] = str(e)
    return summary


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self.declare_parameter("output_dir", os.path.expanduser("~/ros2_ws/trajectories"))
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("sample_rate", cfg.RECORDER_SAMPLE_RATE)
        self.declare_parameter("log_dir", cfg.LOG_DIR)
        self.declare_parameter("keep_all_runs", cfg.LOG_KEEP_ALL_RUNS)
        self.declare_parameter("stylus_topic", "/phantom/state")
        self.declare_parameter("touch_target_topic", "/ur16e_touch_teleop/target_pose")
        self.declare_parameter("ur_tcp_topic", "/tcp_pose_broadcaster/pose")
        # Only recorded in META (factory vs calibrated runs must be told apart)
        self.declare_parameter("kinematics_params_file", "")

        self.output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        self.log_dir = os.path.expanduser(self.get_parameter("log_dir").value)
        self.keep_all_runs = self.get_parameter("keep_all_runs").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_positions = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "~/path", latched)
        self.recording_pub = self.create_publisher(String, "~/recording", latched)

        # Path logs (open only while recording)
        self.logs = None
        self.run_dir = None
        self.run_start_wall = None
        self.ur_frame_to_base = {}      # frame_id -> (R, t) into base_frame, looked up once
        self.last_tcp_msg = None        # host time of the last tcp_pose_broadcaster message
        # ee_frame -> pendant TCP (Rotation, translation), measured per recording; RTDE
        # rows wait in pending_tcp_rows until it is known
        self.tcp_offset = None
        self.tcp_offset_samples = []
        self.tcp_check_errors = []      # m, offset drift at each later check
        self.tcp_queue = deque()        # (stamp, values) RTDE samples waiting for TF
        self.tcp_messages = 0
        self.pending_tcp_rows = []
        self.bag = None                 # `ros2 bag record` process while recording
        self.create_subscription(OmniState, self.get_parameter("stylus_topic").value,
                                 self.on_stylus, 100)
        self.create_subscription(PoseStamped, self.get_parameter("touch_target_topic").value,
                                 self.on_touch_target, 100)
        self.create_subscription(PoseStamped, self.get_parameter("ur_tcp_topic").value,
                                 self.on_ur_tcp, 100)
        # Teleop's active tool (tool library) at start, and any switch during a recording
        self.active_tool = None
        self.tool_log = None
        self.create_subscription(String, "/ur16e_touch_teleop/active_tool", self.on_active_tool, latched)
        self.create_timer(1.0 / cfg.LOG_TF_FALLBACK_RATE, self.log_tf_fallback)

        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)

        self.recording = False
        self.samples = []
        self.start_time = None
        self.path = Path()
        self.path.header.frame_id = self.base_frame
        self.publish_recording(False)

        self.create_timer(1.0 / self.get_parameter("sample_rate").value, self.sample)
        self.ndi = None     # NDI tracker status for the current recording (META.yaml)
        self.create_timer(1.0, self.check_ndi)
        self.get_logger().info(f"Recorder ready: runs go to {short_path(os.path.join(self.log_dir, RUN_FOLDER))}")

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in UR_JOINTS):
            self.joint_positions = [positions[j] for j in UR_JOINTS]

    def on_start(self, _request, response):
        if self.recording:
            response.success = False
            response.message = "Already recording"
            return response
        ndi_ok = ndi_tracker_listening(self)
        if not ndi_ok and cfg.LOG_REQUIRE_NDI:
            response.success = False
            response.message = ("NOT RECORDING: the NDI tracker is not running. Start "
                                "`python3 -m ndi_tracking run` (same ROS_DOMAIN_ID), or set "
                                "LOG_REQUIRE_NDI = False to record without it")
            self.get_logger().error(response.message)
            return response
        self.ndi = {"tracker_at_start": ndi_ok, "lost_at": None, "files": None}
        self.tool_log = {"at_start": self.active_tool, "changes": []}
        self.samples = []
        self.path.poses = []
        self.start_time = self.get_clock().now()
        try:
            self.open_logs()
        except OSError as exc:
            self.get_logger().error(f"Could not open path logs in {self.log_dir}: {exc}")
            self.logs = None
        self.recording = True
        self.publish_recording(True)
        response.success = True
        response.message = "Recording..." if self.logs else "Recording trajectory only (path logs failed)..."
        if not ndi_ok:
            response.message += " - WARNING: NDI tracker not running, this run has NO NDI data"
            self.get_logger().error("NDI tracker not running: this run has NO NDI data")
        return response

    def check_ndi(self):
        """Shout as soon as the NDI tracker drops out of a recording it started with."""
        if self.recording and self.ndi["tracker_at_start"] and self.ndi["lost_at"] is None \
                and not ndi_tracker_listening(self):
            self.ndi["lost_at"] = round(self.host_time(), 1)
            self.get_logger().error(f"NDI tracker STOPPED {self.ndi['lost_at']} s into the recording: "
                                    "NDI data is missing from here on")

    def on_stop(self, _request, response):
        if not self.recording:
            response.success = False
            response.message = "Not recording"
            return response
        self.recording = False
        trajectory_path, problems = None, []
        if len(self.samples) < 2:
            problems.append("no replayable trajectory (did the robot move?)")
        else:
            try:
                trajectory_path = self.save()
            except OSError as exc:
                problems.append(f"trajectory not saved: {exc}")
        saved = f"Saved {self.host_time():.1f} s"
        if self.logs:
            # The tracker opens NDI_PATH.csv when it starts recording into run_dir
            self.ndi["files"] = os.path.exists(os.path.join(self.run_dir, "NDI_PATH.csv"))
            ndi_problem = ("NO NDI DATA in this run (no NDI_PATH.csv)" if not self.ndi["files"] else
                           f"NDI tracker stopped {self.ndi['lost_at']} s in: NDI data incomplete"
                           if self.ndi["lost_at"] is not None else None)
            if ndi_problem:
                self.get_logger().error(ndi_problem)
                problems.append(ndi_problem)
        # Before close_logs (bag shutdown takes a moment, LOG_KEEP_ALL_RUNS copies the
        # folder): gives other recorders time to finish their files in run_dir
        self.publish_recording(False)
        if self.logs:
            try:
                saved += " to " + self.close_logs(trajectory_path)
            except OSError as exc:
                problems.append(f"path logs not written: {exc}")
        elif trajectory_path:
            saved += f" trajectory to {short_path(trajectory_path)}"
        response.success = trajectory_path is not None
        response.message = saved + "".join(f" - WARNING: {p}" for p in problems)
        return response

    def publish_recording(self, recording):
        self.recording_pub.publish(String(data=json.dumps({
            "recording": recording,
            "run_dir": self.run_dir if self.logs else None,
            "start_unix": self.start_time.nanoseconds * 1e-9 if self.start_time else None,
        })))

    # ------------------------------------------------------------ path logs
    def host_time(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def on_active_tool(self, msg):
        self.active_tool = json.loads(msg.data)
        if self.recording and self.tool_log is not None:
            self.tool_log["changes"].append({"t": round(self.host_time(), 3), **self.active_tool})

    def open_logs(self):
        self.run_start_wall = datetime.now()
        self.run_dir = os.path.join(self.log_dir, RUN_FOLDER)
        if os.path.isdir(self.run_dir):
            shutil.rmtree(self.run_dir)     # previous run was not renamed: overwrite it
        os.makedirs(self.run_dir)
        self.ur_frame_to_base = {}
        # Counts as a fresh RTDE message so the TF fallback waits 0.5 s for the first real one
        self.last_tcp_msg = self.get_clock().now()
        self.tcp_offset = None
        self.tcp_offset_samples = []
        self.tcp_check_errors = []
        self.tcp_queue = deque()
        self.tcp_messages = 0
        self.pending_tcp_rows = []
        stamped = ["t", "stamp"]
        self.start_bag()
        self.logs = {
            "touch": CsvStream(os.path.join(self.run_dir, "TOUCH_PATH.csv"),
                               stamped + ["frame"] + POSE_COLUMNS),
            "stylus": CsvStream(os.path.join(self.run_dir, "STYLUS_RAW.csv"),
                                stamped + POSE_COLUMNS + ["vx", "vy", "vz", "white", "grey"]),
            "ur": CsvStream(os.path.join(self.run_dir, "UR_PATH.csv"),
                            stamped + ["source", "frame"] + POSE_COLUMNS),
        }

    def close_logs(self, trajectory_path):
        if self.pending_tcp_rows:
            # Offset never measured (no TF?): keep the rows, marked as the raw pendant TCP
            self.get_logger().warn("Pendant TCP offset not measured - UR_PATH rows are the TCP, not tool0")
            for row, values in self.pending_tcp_rows:
                self.logs["ur"].write(row[:2] + ["rtde_tcp", self.base_frame] + values)
            self.pending_tcp_rows = []
        drift = max(self.tcp_check_errors, default=0.0)
        if drift > TCP_CHECK_TOLERANCE:
            self.get_logger().error(
                f"Pendant TCP vs TF {self.ee_frame} drifted up to {drift * 1000:.1f} mm during the run: "
                "TF's kinematics are not this arm's (wrong ur_type or kinematics_params_file?), "
                "so the teleop and UR_PATH are both off")
        logs, self.logs = self.logs, None
        for stream in logs.values():
            stream.close()
        rows = {name: stream.rows for name, stream in logs.items()}
        bag_status = self.stop_bag()

        if trajectory_path:
            stem = os.path.splitext(trajectory_path)[0]
            shutil.copyfile(stem + ".yaml", os.path.join(self.run_dir, "TRAJECTORY.yaml"))
            shutil.copyfile(stem + ".csv", os.path.join(self.run_dir, "TRAJECTORY.csv"))

        package_dir = os.path.dirname(os.path.realpath(cfg.__file__))
        settings = {k: v for k, v in vars(cfg).items() if k.isupper()}
        with open(os.path.join(self.run_dir, "META.yaml"), "w") as f:
            yaml.safe_dump({
                "date": self.run_start_wall.isoformat(timespec="seconds"),
                "duration": round(self.host_time(), 3),
                "code_version": git_version(package_dir),
                "base_frame": self.base_frame,
                "ee_frame": self.ee_frame,
                "rows": rows,
                "rosbag": bag_status,
                "tcp_offset": self.tcp_offset_summary(),
                "kinematics": kinematics_summary(
                    self.get_parameter("kinematics_params_file").value),
                "ndi": self.ndi,
                "tool": self.tool_log,
                "trajectory_source": trajectory_path,
                "notes": {
                    "t": "host receive time, s since recording start (same clock for every file)",
                    "stamp": "sender's header stamp, s",
                    "TOUCH_PATH": "teleop target for ee_frame in base_frame, only while WHITE is held",
                    "STYLUS_RAW": "omni_state pose as published (position mm, device frame)",
                    "UR_PATH": "source rtde = tcp_pose_broadcaster (pendant TCP, tool0_controller) "
                               "moved back to ee_frame by tcp_offset, tf = TF ee_frame fallback, "
                               "rtde_tcp = pendant TCP (offset not measured); all in base_frame",
                    "ndi": "tracker_at_start = NDI tracker following the recording when it started, "
                           "lost_at = s when it stopped following (null = never), "
                           "files = NDI_PATH.csv in this folder at stop",
                    "tool": "teleop RCM tool (config/tools.yaml): at_start = active when recording "
                            "started (null = teleop not running), changes = switches during the run "
                            "(t as above); settings' TOUCH_RCM_SHAFT_* are only the no-library fallback",
                    "kinematics": "kinematics_params_file TF was built from: model nominal = "
                                  "ur_description default, calibrated = this arm's controller "
                                  "calibration (ur_calibration_extract); hash = its calibration hash",
                    "tcp_offset": "ee_frame -> pendant TCP, measured against TF at recording start; "
                                  "check_max_mm = largest drift from it in later checks during the "
                                  "run (should be ~0: more means TF uses the wrong arm kinematics)",
                },
                "settings": settings,
            }, f, default_flow_style=None, sort_keys=False)

        if self.keep_all_runs:
            shutil.copytree(self.run_dir, os.path.join(
                self.log_dir, "runs", self.run_start_wall.strftime("%Y%m%d_%H%M%S")))
        bag = "" if bag_status in ("ok", "off") else f", rosbag {bag_status}"
        return (f"{short_path(self.run_dir)} (Touch {rows['touch']}, UR {rows['ur']} rows{bag}) "
                "- rename it to keep it")

    def start_bag(self):
        if not cfg.LOG_BAG_TOPICS:
            return
        output = open(os.path.join(self.run_dir, "rosbag.log"), "w")
        # Own session, so Ctrl-C in the launch terminal does not cut the bag off
        # before stop_bag() closes it properly
        self.bag = subprocess.Popen(
            ["ros2", "bag", "record", "-s", "mcap", "-o", os.path.join(self.run_dir, "rosbag"),
             "--topics", *cfg.LOG_BAG_TOPICS],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        output.close()

    def stop_bag(self):
        """SIGINT the bag recorder so it writes its metadata; returns a status string."""
        bag, self.bag = self.bag, None
        if bag is None:
            return "off"
        if bag.poll() is not None:
            return f"failed (exit {bag.returncode}, see rosbag.log)"
        bag.send_signal(signal.SIGINT)
        try:
            bag.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            bag.kill()
            return "killed - may be incomplete (see rosbag.log)"
        return "ok"


    def on_stylus(self, msg):
        if self.logs:
            pose = msg.pose
            v = msg.velocity
            self.logs["stylus"].write(
                [self.host_time(), stamp_to_sec(msg.header.stamp)] + pose_values(pose)
                + [v.x, v.y, v.z, int(msg.locked), int(msg.close_gripper)])

    def on_touch_target(self, msg):
        if self.logs:
            self.logs["touch"].write([self.host_time(), stamp_to_sec(msg.header.stamp),
                                      msg.header.frame_id] + pose_values(msg.pose))

    def on_ur_tcp(self, msg):
        if not self.logs:
            return
        self.last_tcp_msg = self.get_clock().now()
        values = self.to_base_frame(msg.header.frame_id, pose_values(msg.pose))
        if values is None:
            return
        row = [self.host_time(), stamp_to_sec(msg.header.stamp), "rtde", self.base_frame]
        if self.tcp_offset is None:
            self.pending_tcp_rows.append((row, values))
        self.tcp_messages += 1
        if self.tcp_offset is None or self.tcp_messages % TCP_CHECK_EVERY == 0:
            self.tcp_queue.append((Time.from_msg(msg.header.stamp), values))
        self.process_tcp_queue()
        if self.tcp_offset is None:
            return
        if self.pending_tcp_rows:   # holds this row too
            for pending_row, pending_values in self.pending_tcp_rows:
                self.logs["ur"].write(pending_row + self.tcp_to_ee(pending_values))
            self.pending_tcp_rows = []
            return
        self.logs["ur"].write(row + self.tcp_to_ee(values))

    def process_tcp_queue(self):
        """Compare queued RTDE samples with TF ee_frame at their stamps, oldest first.

        robot_state_publisher's TF trails the 500 Hz RTDE stream (it publishes at ~20 Hz),
        so a lookup at a fresh RTDE stamp is always in TF's future; samples wait here until
        TF has caught up, or are dropped after TCP_QUEUE_AGE."""
        while self.tcp_queue:
            stamp, values = self.tcp_queue[0]
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, stamp)
            except TransformException:
                if (self.tcp_queue[-1][0] - stamp).nanoseconds * 1e-9 <= TCP_QUEUE_AGE:
                    return      # TF not there yet - retry on the next RTDE message
                self.tcp_queue.popleft()
                continue
            self.tcp_queue.popleft()
            self.add_tcp_offset_sample(tf, values)

    def add_tcp_offset_sample(self, tf, tcp_values):
        """Pendant TCP relative to TF ee_frame: averaged over the first TCP_OFFSET_SAMPLES
        into tcp_offset, later ones only checked against it."""
        t, r = tf.transform.translation, tf.transform.rotation
        ee_rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])
        tcp_rotation = Rotation.from_quat(tcp_values[3:])
        translation = ee_rotation.inv().apply(np.array(tcp_values[:3]) - [t.x, t.y, t.z])
        if self.tcp_offset is not None:
            error = float(np.linalg.norm(translation - self.tcp_offset[1]))
            self.tcp_check_errors.append(error)
            if error > TCP_CHECK_TOLERANCE:
                self.get_logger().warn(
                    f"Pendant TCP vs TF {self.ee_frame} offset moved {error * 1000:.1f} mm - "
                    "wrong ur_type or kinematics file?", throttle_duration_sec=2.0)
            return
        self.tcp_offset_samples.append((ee_rotation.inv() * tcp_rotation, translation))
        if len(self.tcp_offset_samples) < TCP_OFFSET_SAMPLES:
            return
        rotations, translations = zip(*self.tcp_offset_samples)
        self.tcp_offset = (Rotation.concatenate(rotations).mean(), np.median(translations, axis=0))
        summary = self.tcp_offset_summary()
        self.get_logger().info(f"Pendant TCP offset from {self.ee_frame}: {summary['xyz_mm']} mm, "
                               f"{summary['rotvec_deg']} deg (UR_PATH logged as {self.ee_frame})")

    def tcp_to_ee(self, tcp_values):
        """Pendant TCP pose [x y z qx qy qz qw] -> ee_frame pose, using tcp_offset."""
        offset_rotation, offset_translation = self.tcp_offset
        ee_rotation = Rotation.from_quat(tcp_values[3:]) * offset_rotation.inv()
        ee_position = np.array(tcp_values[:3]) - ee_rotation.apply(offset_translation)
        return [float(v) for v in ee_position] + [float(v) for v in ee_rotation.as_quat()]

    def tcp_offset_summary(self):
        if self.tcp_offset is None:
            return None
        rotation, translation = self.tcp_offset
        return {"xyz_mm": [round(float(v) * 1000, 2) for v in translation],
                "rotvec_deg": [round(float(v), 3) for v in np.degrees(rotation.as_rotvec())],
                "checks": len(self.tcp_check_errors),
                "check_max_mm": round(max(self.tcp_check_errors, default=0.0) * 1000, 2)}

    def to_base_frame(self, frame, values):
        """Pose [x y z qx qy qz qw] in `frame` -> base_frame (static transform, looked up once)."""
        if frame == self.base_frame:
            return values
        if frame not in self.ur_frame_to_base:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except TransformException as exc:
                self.get_logger().warn(f"No {self.base_frame}->{frame} transform: {exc}",
                                       throttle_duration_sec=1.0)
                return None
            t, r = tf.transform.translation, tf.transform.rotation
            self.ur_frame_to_base[frame] = (Rotation.from_quat([r.x, r.y, r.z, r.w]),
                                            np.array([t.x, t.y, t.z]))
        rotation, translation = self.ur_frame_to_base[frame]
        position = rotation.apply(values[:3]) + translation
        orientation = (rotation * Rotation.from_quat(values[3:])).as_quat()
        return list(position) + list(orientation)

    def log_tf_fallback(self):
        """UR path from TF when tcp_pose_broadcaster is silent (e.g. mock hardware)."""
        if not self.logs:
            return
        now = self.get_clock().now()
        if self.last_tcp_msg is not None and (now - self.last_tcp_msg).nanoseconds < 5e8:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except TransformException:
            return
        t, r = tf.transform.translation, tf.transform.rotation
        self.logs["ur"].write([self.host_time(), stamp_to_sec(tf.header.stamp), "tf", self.base_frame,
                               t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    def sample(self):
        if not self.recording or self.joint_positions is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f"No {self.base_frame}->{self.ee_frame} transform: {exc}",
                                   throttle_duration_sec=1.0)
            return

        now = self.get_clock().now()
        t, r = tf.transform.translation, tf.transform.rotation
        self.samples.append({
            "time": (now - self.start_time).nanoseconds * 1e-9,
            "positions": list(self.joint_positions),
            "ee_pose": [t.x, t.y, t.z, r.x, r.y, r.z, r.w],
        })

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = now.to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = r
        self.path.poses.append(pose)
        self.path.header.stamp = now.to_msg()
        self.path_pub.publish(self.path)

    def save(self):
        os.makedirs(self.output_dir, exist_ok=True)
        name = datetime.now().strftime("trajectory_%Y%m%d_%H%M%S")
        yaml_path = os.path.join(self.output_dir, name + ".yaml")
        csv_path = os.path.join(self.output_dir, name + ".csv")

        # Re-base time so the first saved sample is t = 0
        t0 = self.samples[0]["time"]
        samples = [dict(s, time=round(s["time"] - t0, 4)) for s in self.samples]

        with open(yaml_path, "w") as f:
            yaml.safe_dump({
                "joint_names": UR_JOINTS,
                "base_frame": self.base_frame,
                "ee_frame": self.ee_frame,
                "duration": samples[-1]["time"],
                "samples": samples,
            }, f, default_flow_style=None, sort_keys=False)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time"] + UR_JOINTS + ["x", "y", "z", "qx", "qy", "qz", "qw"])
            for s in samples:
                writer.writerow([s["time"]] + s["positions"] + s["ee_pose"])

        return yaml_path


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.logs:      # shut down mid-recording: still close the logs and bag properly
            node.get_logger().info(node.close_logs(None))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
