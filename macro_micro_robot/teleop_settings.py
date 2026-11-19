"""Tuning constants for the teleoperation, recorder and player nodes.

Each value is the default of a ROS parameter (override with `--ros-args -p name:=value`).
With a --symlink-install build, edits take effect the next time the nodes start.
Why each value is what it is: docs/VERSIONS.md.

Stylus speed chain: displacement x TOUCH_TRANSLATION_SCALE = target offset,
error x TOUCH_POSITION_GAIN = velocity, capped at TOUCH_MAX_LINEAR_SPEED, then MoveIt Servo,
then the pendant speed slider. The teleop nodes overwrite Servo's own Cartesian caps with
the max speeds below.
"""

# ---------------------------------------------------------------- Touch stylus
# Robot metres per stylus metre.
TOUCH_TRANSLATION_SCALE = 1.0
# Motion scales cycled by double-clicking WHITE (multiplied into the scale above).
TOUCH_SCALE_STEPS = [1.0, 0.2, 0.05]
# Rotation is scaled by step ** this: 1 = same as translation, 0 = never scaled.
TOUCH_SCALE_ROTATION_EXPONENT = 0.5
# Two WHITE presses this close (s) are a double click.
TOUCH_DOUBLE_CLICK_TIME = 0.3
# Hold GREY this long (s) to switch between arm (macro) and micro module; a short click records.
TOUCH_MODE_SWITCH_HOLD_TIME = 1.0
# Robot radians per stylus radian.
TOUCH_ROTATION_SCALE = 1.0
TOUCH_ENABLE_ROTATION = True
# Pen twist turns wrist 3 directly (negative scale reverses it).
TOUCH_ENABLE_TWIST = True
TOUCH_TWIST_SCALE = 1.0

# Hard caps on tool speed.
TOUCH_MAX_LINEAR_SPEED = 1.2    # m/s
TOUCH_MAX_ANGULAR_SPEED = 3.0    # rad/s

# Proportional gains on the tool-to-target error (1/s).
TOUCH_POSITION_GAIN = 8.0
TOUCH_ROTATION_GAIN = 8.0
# Cap on what the gains may add on top of feedforward (0 = no cap); slows catch-up surges.
TOUCH_MAX_CORRECTION_SPEED = 0.05           # m/s
TOUCH_MAX_CORRECTION_ANGULAR_SPEED = 0.5    # rad/s

# Fraction of the stylus velocity fed forward (0 = off, 1 = full).
TOUCH_FEEDFORWARD = 1.0
# Low-pass time constant on the stylus velocity estimate (s).
TOUCH_FEEDFORWARD_FILTER = 0.02
# 2nd-order Butterworth cutoff (Hz) on the twist sent to Servo; below half TOUCH_PUBLISH_RATE. 0 = off.
TOUCH_COMMAND_FILTER = 5.0
# Constant-velocity Kalman filter on the raw stylus pose (uses only real encoder updates).
TOUCH_KALMAN = True
# Process noise: lower = smoother but overshoots a sudden stop.
TOUCH_KALMAN_ACCEL_NOISE = 2.0           # (m/s^2)^2 s
TOUCH_KALMAN_ANGULAR_ACCEL_NOISE = 30.0  # (rad/s^2)^2 s
# Measurement noise (1 sigma).
TOUCH_KALMAN_POSITION_NOISE = 5e-4       # m
TOUCH_KALMAN_ROTATION_NOISE = 0.01       # rad
# Command-to-pose delay (s): the gains compare the robot with the target this long ago.
TOUCH_LOOP_DELAY = 0.05

# Singularity guard (ur16e_common.SingularityGuard), on the Jacobian condition number c.
# From SLOW, motion into the singularity is cut (all of it at STOP); past STOP the arm is
# pushed back out while WHITE is held. Needs TOUCH_POSE_SOURCE "joint_states".
TOUCH_SINGULARITY_GUARD = True
TOUCH_SINGULARITY_SLOW = 30.0
TOUCH_SINGULARITY_STOP = 60.0
# Largest damped-least-squares damping (~m).
TOUCH_SINGULARITY_DAMPING = 0.05
# Back-off joint speed (rad/s) per unit of c / STOP - 1, and its cap.
TOUCH_SINGULARITY_ESCAPE_GAIN = 1.0
TOUCH_SINGULARITY_ESCAPE_SPEED = 0.2

# RCM mode: hold WHITE + click GREY to latch the tool tip as the pivot; again (retracted) to leave.
# Fallback shaft in tool0 (start, unit direction, length), used only when TOOL_LIBRARY is empty.
TOUCH_RCM_SHAFT_START = [0.00008, 0.00699, 0.20827]     # m
TOUCH_RCM_SHAFT_DIRECTION = [-0.00309, 0.01798, 0.99983]
TOUCH_RCM_SHAFT_LENGTH = 0.1586               # m
# Calibrated tools (macro_micro_robot/tools.py); switch with a number in the launch_system.sh
# terminal or `ros2 param set /ur16e_touch_teleop tool N`.
TOOL_LIBRARY = "~/ros2_ws/src/MacroMicroSurgicalRobot/config/tools.yaml"
# Tool switches need every joint below this speed (rad/s) for this long (s).
TOOL_SWITCH_MAX_JOINT_SPEED = 0.01
TOOL_SWITCH_STILL_TIME = 0.5
# Closest the shaft start may come to the RCM point (m).
TOUCH_RCM_DEPTH_MARGIN = 0.02
# Depth-invariant law: sideways stylus motion moves tool0 by gain x stylus motion at any depth. 0 = off.
TOUCH_RCM_TOOL0_GAIN = 1.0
# Deepest insertion (m) the law holds to; deeper, the tip keeps its share from here.
TOUCH_RCM_TOOL0_GAIN_DEPTH = 0.13
# Optional sideways fade near the RCM point (m, 0 = off) and its exponent. Keep off with the tool0 gain.
TOUCH_RCM_PIVOT_FADE_DEPTH = 0.0
TOUCH_RCM_PIVOT_FADE_STRENGTH = 1.0
# How far (m) the tip may be pulled back out past the RCM point.
TOUCH_RCM_RETRACT_DEPTH = 0.005
# RCM mode can be left only with the tip within this depth (m) of the point.
TOUCH_RCM_EXIT_DEPTH = 0.003
# Gain (1/s) pulling the shaft back onto the RCM point.
TOUCH_RCM_GAIN = 8.0
# Past this shaft-to-point error (m) the robot holds still and RCM may be left at any depth.
TOUCH_RCM_MAX_ERROR = 0.005

# Haptic cues (macro_micro_robot/haptics.py) while WHITE is held. Distances are in stylus metres.
# A force or stiffness of 0 turns that cue off; TOUCH_HAPTICS = False turns all off.
TOUCH_HAPTICS = True
# Cap on total force (N); the Touch gives 3.3 N peak, 0.88 N continuous.
TOUCH_HAPTIC_MAX_FORCE = 0.01
# Max force slew rate (N/s), so steps do not trip the max-impulse check.
TOUCH_HAPTIC_SLEW_RATE = 2.0           # 0.01 N in 5 ms
# One-sided RCM depth walls, rendered by touch_server at 1 kHz (N/m). Off: they buzz on this Touch.
TOUCH_HAPTIC_WALL_STIFFNESS = 0.0       # N/m
# 40 ms tap (N) when the tip target crosses the RCM point, with hysteresis (m) and hold-off (s).
TOUCH_HAPTIC_CROSSING_FORCE = 0.002
TOUCH_HAPTIC_CROSSING_HYSTERESIS = 0.001
TOUCH_HAPTIC_CROSSING_HOLDOFF = 0.3
# Tap (N) at the deepest insertion and at the retract limit.
TOUCH_HAPTIC_LIMIT_FORCE = 0.002
# Tap (N) when RCM turns on or off (two when leaving is refused); frequency > 0 makes it a buzz.
TOUCH_HAPTIC_PULSE_FORCE = 0.002
TOUCH_HAPTIC_PULSE_FREQUENCY = 0.0
# Push (N) and damping (N s/m) out of a singularity, ramped from SLOW to STOP.
TOUCH_HAPTIC_SINGULARITY_FORCE = 0.005
TOUCH_HAPTIC_SINGULARITY_DAMPING = 0.015
# Low-pass time constant on the singularity push (s).
TOUCH_HAPTIC_SINGULARITY_FILTER = 0.03

# Position deadband (m at 1:1; scales with the motion scale).
TOUCH_DEADBAND = 3e-4
# Stop if no stylus data for this long (s).
TOUCH_TIMEOUT = 0.1
# Twist publish rate (Hz); match Servo's cycle.
TOUCH_PUBLISH_RATE = 500.0
# Tool pose source: "tf" (20 Hz) or "joint_states" (500 Hz FK from /robot_description).
TOUCH_POSE_SOURCE = "joint_states"
# omni_state publishes millimetres.
TOUCH_UNITS_TO_M = 1e-3

# "toward_base" or "fixed" (fixed: TOUCH_AXIS_MAP gives the stylus axis per robot X, Y, Z).
TOUCH_MAPPING = "toward_base"
TOUCH_AXIS_MAP = ["y", "-x", "z"]

# Seconds the launch file waits for the driver and Servo before starting the Touch node.
TOUCH_STARTUP_DELAY = 8.0

# ---------------------------------------------------------------- MoveIt Servo
# Servo command lookahead (s); minimum 0.012. Needs a relaunch.
SERVO_LOOKAHEAD = 0.05
# "trajectory" (scaled_joint_trajectory_controller), "velocity" (forward_velocity_controller, speedj)
# or "position" (joint_velocity_limiter -> forward_position_controller, servoj). Needs a relaunch.
SERVO_OUTPUT = "position"
# Servo's own singularity thresholds; the _GUARDED pair applies while the guard above runs.
SERVO_SINGULARITY_SLOWDOWN = 100.0
SERVO_SINGULARITY_STOP = 200.0
SERVO_SINGULARITY_SLOWDOWN_GUARDED = 1e4
SERVO_SINGULARITY_STOP_GUARDED = 1e6

# ------------------------------------------------------- Velocity mode (only)
# Servo cycle in velocity and position mode (Hz); 500 = the UR's 2 ms cycle. Needs a relaunch.
SERVO_VELOCITY_RATE = 500.0
# Use config/external_control_smooth_speedj.urscript (ramps between speedj set points). Real arm only.
SERVO_SMOOTH_SPEEDJ = False
# Run joint_velocity_limiter between Servo and the controller.
VELOCITY_LIMITER = True
# Joint speed cap as a fraction of each URDF limit (all joints scaled together).
VELOCITY_LIMITER_SPEED_FRACTION = 0.8
# Joint acceleration caps (rad/s^2), shoulder pan to wrist 3.
VELOCITY_LIMITER_MAX_ACCELERATION = [8.0, 8.0, 15.0, 25.0, 25.0, 25.0]
# Two first-order smoothing stages after the caps (s, each). 0 = off.
VELOCITY_LIMITER_SMOOTHING = 0.003

# ------------------------------------------------------- Position mode (only)
# Lead (s) added to the integrated position (q + qd * lead) to cancel servoj lag.
SERVO_POSITION_LEAD = 0.03
# Furthest (rad) the integrated position may run ahead of the measured joints.
SERVO_POSITION_MAX_ERROR = 0.15

# -------------------------------------------------------------------- Keyboard
KEYBOARD_LINEAR_SPEED = 0.05     # m/s, starting speed
KEYBOARD_ANGULAR_SPEED = 0.3     # rad/s, starting speed
KEYBOARD_MAX_LINEAR_SPEED = 0.25     # m/s
KEYBOARD_MAX_ANGULAR_SPEED = 1.0     # rad/s
# Factor per + press (- divides).
KEYBOARD_SPEED_STEP = 1.25
# Seconds a key keeps moving after an auto-repeat press, and after the first press.
KEYBOARD_HOLD_TIME = 0.15
KEYBOARD_FIRST_PRESS_HOLD = 0.6
KEYBOARD_PUBLISH_RATE = 50.0     # Hz

# -------------------------------------------------------------------- Recorder
RECORDER_SAMPLE_RATE = 25.0      # Hz

# Runs are written to LOG_DIR/NEW_RUN/ (replaced by the next recording; rename to keep).
# LOG_KEEP_ALL_RUNS also copies each run to LOG_DIR/runs/<date_time>/.
LOG_DIR = "~/ros2_ws/src/MacroMicroSurgicalRobot/logs"
LOG_KEEP_ALL_RUNS = False
# Refuse to record unless the NDI tracker is running (False: warn only).
LOG_REQUIRE_NDI = False
# TF fallback rate for the UR path when tcp_pose_broadcaster is absent (Hz).
LOG_TF_FALLBACK_RATE = 200.0
# Topics recorded to LOG_DIR/NEW_RUN/rosbag; missing topics are skipped, [] = no bag.
LOG_BAG_TOPICS = [
    "/phantom/state",
    "/phantom/button",
    "/phantom/joint_states",
    "/phantom/force_feedback",
    "/phantom/applied_force",
    "/ur16e_touch_teleop/target_pose",
    "/ur16e_touch_teleop/motion_scale",
    "/ur16e_touch_teleop/condition_number",
    "/ur16e_touch_teleop/rcm_active",
    "/ur16e_touch_teleop/rcm_point",
    "/ur16e_touch_teleop/rcm_error",
    "/ur16e_touch_teleop/insertion_depth",
    "/ur16e_touch_teleop/tool_tip",
    "/servo_node/delta_twist_cmds",
    "/servo_node/status",
    "/scaled_joint_trajectory_controller/joint_trajectory",
    "/joint_velocity_limiter/commands",
    "/forward_velocity_controller/commands",
    "/tcp_pose_broadcaster/pose",
    "/joint_states",
    "/speed_scaling_state_broadcaster/speed_scaling",
    "/tf",
    "/tf_static",
]

# ---------------------------------------------------------------------- Player
PLAYER_DEFAULT_SPEED = 1.0       # playback speed factor
PLAYER_MAX_SPEED = 2.0           # --speed is refused above this
PLAYER_APPROACH_TIME = 5.0       # s to move to the first recorded pose
# Recorded samples closer together than this are dropped (s).
PLAYER_MIN_POINT_SPACING = 0.04
# 'home' pose: elbow up, tool pointing down.
HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# --------------------------------------------------------------- Stylus replay
# Still time (s) before and after a stylus replay, so the bag and the settling are captured.
REPLAY_LEAD_IN = 1.5
REPLAY_TAIL = 1.5
