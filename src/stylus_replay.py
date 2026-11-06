#!/usr/bin/env python3
"""Replays the Touch stylus from a logged run into the teleop node, so every version
of the stack can be tested with exactly the same hand motion.

    # Stack without the Touch driver (teleop node listens, nothing else publishes)
    ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay
    # Second terminal, as often as you like:
    ros2 run macro_micro_robot stylus_replay.py V2/Up_10CM            # run folder, relative to logs/
    ros2 run macro_micro_robot stylus_replay.py ~/some/other/run      # or any run folder

1. Reads the stylus pose and buttons from the run's STYLUS_RAW.csv (full rate, from
   the start of the recording - the rosbag misses up to ~2 s while `ros2 bag record`
   subscribes), the robot's joint angles at the start from TRAJECTORY.yaml, and the
   motion scale from the rosbag.
2. Sets the teleop node to the same motion scale (double clicks with the stylus held
   still) and moves the robot to the start joint angles over --approach-time s (asks
   first unless --yes): the teleop target is relative to where the robot was when
   WHITE went down.
3. Starts trajectory_recorder, plays the stylus back with its original timing
   (restamped to now; GREY on its own is masked out so it cannot toggle the
   recording, but GREY pressed while WHITE is held - the RCM chord - is replayed),
   releases WHITE, then stops the recorder. Refuses to start with RCM mode on.
   --rcm: for runs recorded with RCM mode already on, turn it on at the start pose
   (WHITE + GREY with the stylus still, before the recorder starts), so the RCM point
   is the tip there. If RCM mode is still on from the last replay (it ended with the
   tool in), the robot moves straight to the start pose and leaves RCM mode there.
4. Writes REPLAY.yaml into logs/NEW_RUN saying which run it replayed.

Replays of replays work too (they log STYLUS_RAW.csv like any run). The stylus
driver must not be running (launch with input:=replay), otherwise its messages
would mix with the replayed ones.
"""

import argparse
import csv
import os
import sys
import time

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from omni_msgs.msg import OmniButtonEvent, OmniState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from macro_micro_robot.ur16e_common import (
    RECORDER_START_SERVICE,
    RECORDER_STOP_SERVICE,
    TELEOP_CONTROLLER,
    TRAJECTORY_ACTION,
    activate_controller,
    ndi_tracker_listening,
    servo_controller,
    call_service,
    set_servo_paused,
)
from macro_micro_robot import teleop_settings as cfg

STATE_TOPIC = "/phantom/state"
BUTTON_TOPIC = "/phantom/button"
SCALE_TOPIC = "/ur16e_touch_teleop/motion_scale"
RCM_ACTIVE_TOPIC = "/ur16e_touch_teleop/rcm_active"
# Bags recorded before the 2026-11-02 rename have the teleop's topics under this node name.
# Reading only the new name made every older run look like "no motion_scale = 1:1": the V7
# RCMOnly original (1:5) was replayed at 1:1 on 2026-11-03.
OLD_TELEOP_NODE = "/ur5e_touch_teleop/"
PROGRAM_RUNNING_TOPIC = "/io_and_status_controller/robot_program_running"
UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


def resolve_run(name):
    """Run folder, absolute or relative to cwd or LOG_DIR (a rosbag/ path is taken as its run)."""
    log_dir = os.path.expanduser(cfg.LOG_DIR)
    for candidate in (os.path.expanduser(name), os.path.join(log_dir, name)):
        candidate = os.path.normpath(os.path.abspath(candidate))
        if os.path.basename(candidate) == "rosbag":
            candidate = os.path.dirname(candidate)
        if os.path.isfile(os.path.join(candidate, "STYLUS_RAW.csv")):
            return candidate
    sys.exit(f"No STYLUS_RAW.csv found for {name!r} (looked in it and under {log_dir})")


def read_run(run_dir):
    """-> dict(events=[(t, topic, msg or (WHITE, GREY) bools)] sorted, t s from the first
               stylus row, white=WHITE held at the start, joints=start joint positions or
               None, scale=motion_scale value in the bag or None)

    GREY is only kept while it is an RCM chord (pressed with WHITE held); on its own it
    toggled the recording, which the replay owns."""
    with open(os.path.join(run_dir, "STYLUS_RAW.csv"), newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{run_dir}/STYLUS_RAW.csv has no rows")
    t0 = float(rows[0]["t"])
    events, white = [], rows[0]["white"] == "1"
    previous = (white, False)
    grey_was = rows[0]["grey"] == "1"
    chord = False
    for row in rows:
        t = float(row["t"]) - t0
        pressed = row["white"] == "1"           # recorded from OmniState.locked = WHITE
        grey = row["grey"] == "1"               # recorded from OmniState.close_gripper = GREY
        if grey and not grey_was:
            chord = pressed
        elif not grey:
            chord = False
        grey_was = grey
        if (pressed, chord) != previous:
            events.append((t, BUTTON_TOPIC, (pressed, chord)))
            previous = (pressed, chord)
        msg = OmniState(locked=pressed)
        msg.header.frame_id = "world"
        p, q, v = msg.pose.position, msg.pose.orientation, msg.velocity
        p.x, p.y, p.z = (float(row[c]) for c in ("x", "y", "z"))
        q.x, q.y, q.z, q.w = (float(row[c]) for c in ("qx", "qy", "qz", "qw"))
        v.x, v.y, v.z = (float(row[c]) for c in ("vx", "vy", "vz"))
        events.append((t, STATE_TOPIC, msg))

    # The recorder samples joints from the start of the recording (first sample within
    # 1 / RECORDER_SAMPLE_RATE); the robot is still until WHITE goes down
    joints = None
    try:
        with open(os.path.join(run_dir, "TRAJECTORY.yaml")) as f:
            samples = (yaml.safe_load(f) or {}).get("samples") or []
        joints = samples[0]["positions"] if samples else None
    except OSError:
        pass
    return {"events": events, "white": white, "joints": joints,
            "scale": bag_first(run_dir, SCALE_TOPIC, Float64),
            "rcm": bag_first(run_dir, RCM_ACTIVE_TOPIC, Bool)}


def renamed(topic):
    """The same teleop topic as recorded before the 2026-11-02 rename."""
    return topic.replace("/ur16e_touch_teleop/", OLD_TELEOP_NODE)


def bag_first(run_dir, topic, msg_type):
    """The first value of a (latched) topic in the run's bag, or None: the teleop's
    motion_scale, or whether RCM mode was on when the recording started."""
    bag_dir = os.path.join(run_dir, "rosbag")
    if not os.path.isfile(os.path.join(bag_dir, "metadata.yaml")):
        return None
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    reader.set_filter(StorageFilter(topics=[topic, renamed(topic)]))
    if not reader.has_next():
        return None
    return deserialize_message(reader.read_next()[1], msg_type).data


def scale_step(scale, run_dir):
    """motion_scale value from a bag -> scale step (it is TOUCH_TRANSLATION_SCALE * step).

    Runs from before motion scaling (no motion_scale in the bag) were all 1:1."""
    if scale is None:
        return 1.0
    try:
        with open(os.path.join(run_dir, "META.yaml")) as f:
            base = (yaml.safe_load(f) or {}).get("settings", {}).get("TOUCH_TRANSLATION_SCALE", 1.0)
    except OSError:
        base = cfg.TOUCH_TRANSLATION_SCALE
    return scale / base


class StylusReplay(Node):
    def __init__(self):
        super().__init__("stylus_replay")
        self.state_pub = self.create_publisher(OmniState, STATE_TOPIC, 10)
        self.button_pub = self.create_publisher(OmniButtonEvent, BUTTON_TOPIC, 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.program_running = None
        self.teleop_scale = None
        self.rcm_active = None
        self.create_subscription(Bool, PROGRAM_RUNNING_TOPIC, self.on_program_running, latched)
        self.create_subscription(Float64, SCALE_TOPIC, self.on_scale, latched)
        self.create_subscription(Bool, RCM_ACTIVE_TOPIC, self.on_rcm_active, latched)
        self.buttons = (False, False)

    def on_program_running(self, msg):
        self.program_running = msg.data

    def on_scale(self, msg):
        self.teleop_scale = msg.data

    def on_rcm_active(self, msg):
        self.rcm_active = msg.data

    def check_topics(self, timeout=5.0):
        """Exit if the real stylus driver is publishing, or nobody is listening."""
        end = time.monotonic() + timeout
        while not self.get_subscriptions_info_by_topic(STATE_TOPIC) and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)     # discovery
        others = [i for i in self.get_publishers_info_by_topic(STATE_TOPIC) if i.node_name != self.get_name()]
        if others:
            sys.exit(f"{others[0].node_name} is publishing {STATE_TOPIC} - stop the Touch driver "
                     "(launch with input:=replay) so the replay is the only stylus")
        if not self.get_subscriptions_info_by_topic(STATE_TOPIC):
            sys.exit(f"Nothing subscribes to {STATE_TOPIC} - is ur16e_teleop.launch.py input:=replay running?")

    def wait_for_ndi_tracker(self, timeout=5.0):
        """True once the NDI tracker follows the recorder (discovery of a fresh process
        can take a few seconds to find it)."""
        end = time.monotonic() + timeout
        while not ndi_tracker_listening(self) and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return ndi_tracker_listening(self)

    def wait_for_status(self, timeout=5.0):
        """Spin until the latched program-running, motion-scale and RCM topics have arrived."""
        end = time.monotonic() + timeout
        while (self.program_running is None or self.teleop_scale is None
               or self.rcm_active is None) and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def scale_clicks(self, step):
        """Double clicks needed to bring the teleop node from its current scale to `step`."""
        steps = cfg.TOUCH_SCALE_STEPS
        current = (self.teleop_scale or cfg.TOUCH_TRANSLATION_SCALE) / cfg.TOUCH_TRANSLATION_SCALE

        def index(s):
            return min(range(len(steps)), key=lambda i: abs(steps[i] - s))

        if abs(steps[index(step)] - step) > 1e-6:
            self.get_logger().warn(f"Recorded scale step {step:g} is not in TOUCH_SCALE_STEPS {steps} "
                                   f"- using {steps[index(step)]:g}")
        return (index(step) - index(current)) % len(steps)

    def follow(self, points, what):
        """Run [(time_from_start s, joint positions)] on the trajectory controller (stopping
        at the last point); True when it got there."""
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server {TRAJECTORY_ACTION} not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = UR_JOINTS
        for k, (t, positions) in enumerate(points):
            point = JointTrajectoryPoint(positions=[float(p) for p in positions])
            if k == len(points) - 1:
                point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            goal.trajectory.points.append(point)
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{what} was rejected")
            return False
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=2.0)
            raise
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"{what} failed: {result.error_string}")
            return False
        return True

    def run_on_trajectory_controller(self, points, what):
        """follow() with Servo paused and its controller swapped out, then back."""
        if not set_servo_paused(self, True, timeout=2.0):
            self.get_logger().warn("Could not pause MoveIt Servo")
        ok = activate_controller(self, TELEOP_CONTROLLER) and self.follow(points, what)
        activate_controller(self, servo_controller(self))
        set_servo_paused(self, False, timeout=2.0)
        return ok

    def recorder(self, service):
        response = call_service(self, Trigger, service, Trigger.Request(), timeout=15.0)
        if response is None:
            return None
        log = self.get_logger().info if response.success else self.get_logger().warn
        log(response.message)
        return response

    def publish_state(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def publish_button(self, white, grey=False):
        self.buttons = (bool(white), bool(grey))
        self.button_pub.publish(OmniButtonEvent(white_button=int(white), grey_button=int(grey)))

    def play(self, events, rate=1000.0):
        """Publish (t, topic, msg) events at their times; between stylus samples the last
        one is repeated at `rate` (the teleop stops on stale stylus data). Button events
        carry (WHITE, GREY); GREY is only ever pressed with WHITE held (the RCM chord),
        the replay owns recording."""
        start = time.monotonic()
        state = next(msg for _, topic, msg in events if topic == STATE_TOPIC)
        for t, topic, msg in events:
            while (delay := start + t - time.monotonic()) > 0:
                time.sleep(min(delay, 1.0 / rate))
                if delay > 1.0 / rate:
                    self.publish_state(state)
            if topic == STATE_TOPIC:
                state = msg
                self.publish_state(state)
            elif topic == BUTTON_TOPIC and msg != self.buttons:
                self.publish_button(*msg)

    def hold(self, state, seconds, buttons=()):
        """Hold the stylus still at `state` for `seconds`, with WHITE events [(t, white)]."""
        self.play([(0.0, STATE_TOPIC, state)] + [(t, BUTTON_TOPIC, (w, False)) for t, w in buttons]
                  + [(seconds, STATE_TOPIC, state)])

    def double_clicks(self, state, count):
        """Cycle the teleop motion scale with the stylus held still (engaging moves nothing)."""
        click = [(0.0, True), (0.05, False), (0.1, True), (0.15, False)]
        gap = cfg.TOUCH_DOUBLE_CLICK_TIME + 0.2
        for _ in range(count):
            self.hold(state, gap, click)

    def rcm_chord(self, state):
        """Hold WHITE and click GREY with the stylus still: RCM mode on at the tip as it is,
        or off (refused unless the tool is retracted). True if it switched.
        WHITE is held longer than a click, so the next press is not a double click."""
        hold = cfg.TOUCH_DOUBLE_CLICK_TIME + 0.2
        self.play([(0.0, STATE_TOPIC, state), (0.0, BUTTON_TOPIC, (True, False)),
                   (hold, BUTTON_TOPIC, (True, True)), (hold + 0.1, BUTTON_TOPIC, (True, False)),
                   (2 * hold, BUTTON_TOPIC, (False, False)), (3 * hold, STATE_TOPIC, state)])
        on = not self.rcm_active
        end = time.monotonic() + 2.0
        while self.rcm_active != on and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.rcm_active == on


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run folder (e.g. V2/Up_10CM, relative to logs/)")
    parser.add_argument("--approach-time", type=float, default=cfg.PLAYER_APPROACH_TIME,
                        help=f"seconds to move to the start pose (default {cfg.PLAYER_APPROACH_TIME:g})")
    parser.add_argument("--no-approach", action="store_true",
                        help="start from wherever the robot is (the run will not match the original)")
    parser.add_argument("--no-record", action="store_true", help="don't start trajectory_recorder")
    parser.add_argument("--rcm", action="store_true",
                        help="start in RCM mode, latched at the start pose (for runs recorded with RCM already on)")
    parser.add_argument("--yes", action="store_true", help="don't ask for confirmation")
    parser.add_argument("--log-dir", default=cfg.LOG_DIR,
                        help="trajectory_recorder's log_dir, for REPLAY.yaml (default LOG_DIR)")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    run_dir = resolve_run(args.run)
    recorded = read_run(run_dir)
    events = recorded["events"]
    states = [msg for _, topic, msg in events if topic == STATE_TOPIC]
    buttons = [(False, False)] + [b for _, topic, b in events if topic == BUTTON_TOPIC]
    presses = sum(1 for a, b in zip(buttons, buttons[1:]) if b[0] and not a[0])
    chords = sum(1 for a, b in zip(buttons, buttons[1:]) if b[1] and not a[1])
    step = scale_step(recorded["scale"], run_dir)
    print(f"Replaying {run_dir}\n  {len(states)} stylus samples, {events[-1][0]:.1f} s, scale 1:{1.0 / step:g}, "
          f"WHITE {'held at the start + ' if recorded['white'] else ''}{presses} presses, "
          f"{chords} RCM chords (WHITE + GREY){', starting in RCM mode' if args.rcm else ''}")
    if recorded["rcm"] and not args.rcm:
        print("  WARNING: RCM mode was on when this run started - add --rcm to start the replay in it")
    elif recorded["rcm"] is False and args.rcm:
        print("  WARNING: RCM mode was off when this run started, but --rcm turns it on")
    if recorded["joints"] is None and not args.no_approach:
        sys.exit("No TRAJECTORY.yaml to take the start pose from - use --no-approach")

    # Keep the context alive on Ctrl+C so WHITE can be released and the recorder stopped
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StylusReplay()
    recording = finished = False
    try:
        time.sleep(1.0)     # let discovery find the teleop node and any other stylus publisher
        node.check_topics()
        # Before anything moves: the recorder would refuse to start without the tracker anyway
        if not args.no_record and cfg.LOG_REQUIRE_NDI and not node.wait_for_ndi_tracker():
            sys.exit("NDI tracker not running - start `python3 -m ndi_tracking run` (same ROS_DOMAIN_ID) "
                     "first, or set LOG_REQUIRE_NDI = False to replay without NDI data")
        node.wait_for_status()
        if node.program_running is False:
            sys.exit("Robot program not running - press play on the teach pendant")
        if node.rcm_active:
            print("  RCM mode is still on from the last replay (it ended with the tool in): the arm moves "
                  "straight to the start pose with the tool in, then RCM mode is left there")
        if not args.yes:
            what = "move to the start pose and replay" if not args.no_approach else "replay from where it is"
            if input(f"The robot will {what}. Proceed? [y/N] ").strip().lower() != "y":
                print("Aborted.")
                return

        node.publish_button(False)
        node.hold(states[0], 0.5)
        node.double_clicks(states[0], node.scale_clicks(step))
        if not args.no_approach:
            if not node.run_on_trajectory_controller([(args.approach_time, recorded["joints"])],
                                                     "Move to the start pose"):
                return

        node.hold(states[0], 0.5)
        # The recorded chords toggle RCM mode from off. Left on by the last replay of this
        # run, the tip is back at its RCM point here, so the teleop lets it be left
        if node.rcm_active:
            if not node.rcm_chord(states[0]):
                sys.exit("RCM mode is still on from the last replay and can't be left here (the tip is "
                         "not at its RCM point): restart the stack (terminal 1), then replay again")
            node.get_logger().info("Left RCM mode from the last replay")
        if args.rcm and not node.rcm_chord(states[0]):
            node.get_logger().error("RCM mode did not turn on - not replaying")
            return
        if not args.no_record:
            response = node.recorder(RECORDER_START_SERVICE)
            if response is None or not response.success:
                return
            recording = True
            node.hold(states[0], cfg.REPLAY_LEAD_IN)    # ros2 bag record needs a moment to subscribe

        node.play(([(0.0, BUTTON_TOPIC, (True, False))] if recorded["white"] else []) + events)
        node.publish_button(False)
        node.hold(states[-1], cfg.REPLAY_TAIL)          # let the robot settle into the logs
        for _ in range(10):                             # play() doesn't spin: take in rcm_active
            rclpy.spin_once(node, timeout_sec=0.05)
        finished = True
    except KeyboardInterrupt:
        node.get_logger().warn("Replay interrupted")
    finally:
        if rclpy.ok():
            node.publish_button(False)
            if recording and node.recorder(RECORDER_STOP_SERVICE) is not None:
                write_replay_info(run_dir, args, step)
            # Without the Touch, RCM mode could only be left by restarting the stack:
            # leave it here, after the recording, so the next replay starts from off
            if finished and node.rcm_active:
                if node.rcm_chord(states[-1]):
                    node.get_logger().info("Left RCM mode after the replay")
                else:
                    node.get_logger().info("RCM mode stays on (the tool is in, as at the end of the run): "
                                           "the next replay leaves it at the start pose")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def write_replay_info(source_run, args, step):
    new_run = os.path.join(os.path.expanduser(args.log_dir), "NEW_RUN")
    if not os.path.isdir(new_run):
        return
    source_meta = {}
    try:
        with open(os.path.join(source_run, "META.yaml")) as f:
            source_meta = yaml.safe_load(f) or {}
    except OSError:
        pass
    with open(os.path.join(new_run, "REPLAY.yaml"), "w") as f:
        yaml.safe_dump({
            "replay_of": source_run,
            "source_date": source_meta.get("date"),
            "source_code_version": source_meta.get("code_version"),
            "scale_step": float(step),
            "approach": not args.no_approach,
        }, f, sort_keys=False)


if __name__ == "__main__":
    main()
