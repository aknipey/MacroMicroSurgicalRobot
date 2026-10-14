"""Tuning constants for the UR teleoperation / trajectory nodes - edit these.

Every value here is the *default* of a ROS parameter, so a one-off run can still
override it with `--ros-args -p name:=value`. With a --symlink-install build,
edits to this file take effect the next time the nodes start (no colcon build).

Speed chain for the Touch stylus (each stage can be the bottleneck):

    stylus displacement * TOUCH_TRANSLATION_SCALE = target offset
    target error * TOUCH_POSITION_GAIN           = commanded velocity
    capped at TOUCH_MAX_LINEAR_SPEED             -> MoveIt Servo -> robot
    ...then scaled again by the teach pendant speed slider.

MoveIt Servo also caps Cartesian speed (scale.linear / scale.rotational, 0.6 m/s
and 0.3 rad/s in ur_servo.yaml). The teleop nodes overwrite those caps with the
max speeds below when they start, so these values are the only limits that matter
(apart from the robot's joint velocity limits, which Servo also enforces).
"""

# ---------------------------------------------------------------- Touch stylus
# Robot metres per stylus metre. 2.0 = robot moves twice as far as your hand.
TOUCH_TRANSLATION_SCALE = 1.0
# Motion scales cycled by double-clicking WHITE (robot metres per stylus metre,
# multiplied into TOUCH_TRANSLATION_SCALE). 0.2 = 1:5, the robot moves a fifth as
# far as your hand. Starts on the first entry.
TOUCH_SCALE_STEPS = [1.0, 0.2, 0.05]
# Tilt and twist are scaled by step ** this, on top of TOUCH_ROTATION_SCALE /
# TOUCH_TWIST_SCALE. 0.5 = sqrt (1:5 -> rotation 1:2.24, 1:20 -> 1:4.47),
# 1 = same as translation, 0 = rotation never scaled.
TOUCH_SCALE_ROTATION_EXPONENT = 0.5
# Two WHITE presses this close together (s, press to press) are a double click.
# The robot still follows the stylus during both presses - nothing waits for this.
TOUCH_DOUBLE_CLICK_TIME = 0.3
# Robot radians per stylus radian.
TOUCH_ROTATION_SCALE = 1.0
TOUCH_ENABLE_ROTATION = True
# Twisting the pen about its own axis turns the last joint (wrist 3) instead of
# being mixed into the tilt. Scale is robot radians per pen radian; make it
# negative if the wrist turns the opposite way to the pen.
TOUCH_ENABLE_TWIST = True
TOUCH_TWIST_SCALE = 1.0

# Top tool speeds. These are the hard caps: if the robot lags your hand on fast
# moves, raise these first.
TOUCH_MAX_LINEAR_SPEED = 1.2    # m/s
TOUCH_MAX_ANGULAR_SPEED = 6.0    # rad/s

# How hard the tool is pulled towards the stylus target (1/s). Commanded speed =
# gain * error, so with gain 4 a 1 cm error gives 4 cm/s. Higher = snappier and
# less lag on small moves, but too high oscillates.
TOUCH_POSITION_GAIN = 8.0
TOUCH_ROTATION_GAIN = 8.0

# Velocity feedforward: also command the stylus's own velocity, so the gains above
# only correct the remaining error instead of producing all of the motion. Roughly
# halves the lag on mock hardware. 0 = off, 1 = full stylus velocity.
TOUCH_FEEDFORWARD = 1.0
# Low-pass time constant on the stylus velocity estimate (s). Larger = smoother but
# more lag; smaller = jitter from the stylus encoders reaches the robot.
TOUCH_FEEDFORWARD_FILTER = 0.02
# Time (s) between a command and the robot pose showing it (Servo lookahead + driver
# + robot). The gains compare the robot with where the target was this long ago,
# so they don't keep pushing for motion that is already on its way (overshoot).
# 0 = compare with the current target, as before.
TOUCH_LOOP_DELAY = 0.05

# Position errors smaller than this are ignored (m, at 1:1 - shrinks with the
# double-click motion scale so fine moves are not swallowed).
TOUCH_DEADBAND = 3e-4
# Stop if no stylus data arrives for this long (s).
TOUCH_TIMEOUT = 0.1
# Rate twist commands are sent to Servo (Hz).
TOUCH_PUBLISH_RATE = 100.0
# Where the robot's tool pose comes from: "tf" = robot_state_publisher's TF (20 Hz,
# so up to 50 ms old) or "joint_states" = computed from each /joint_states message
# (500 Hz) with the URDF on /robot_description.
TOUCH_POSE_SOURCE = "tf"
# omni_state publishes millimetres.
TOUCH_UNITS_TO_M = 1e-3

# "toward_base" or "fixed" (fixed uses TOUCH_AXIS_MAP: stylus axis per robot X, Y, Z).
TOUCH_MAPPING = "toward_base"
TOUCH_AXIS_MAP = ["y", "-x", "z"]

# Seconds the launch file waits for the driver + Servo before starting the Touch node.
TOUCH_STARTUP_DELAY = 8.0

# ---------------------------------------------------------------- MoveIt Servo
# How far ahead (s) each Servo command is scheduled. Servo builds every step on its
# own previous command while that is still in the future, so this must be longer
# than one Servo cycle (4 ms), but it is also latency the robot adds to every move.
# Servo refuses to start below 3 x its 4 ms publish period (0.012).
# Read when Servo starts: needs a relaunch, not just a node restart.
SERVO_LOOKAHEAD = 0.05
# What Servo sends the robot: "trajectory" = joint trajectories to
# scaled_joint_trajectory_controller (the UR tracks positions, adding SERVO_LOOKAHEAD
# and its own servoj lag), "velocity" = joint velocities to forward_velocity_controller
# (the UR runs speedj, no lookahead). Read when Servo starts: needs a relaunch.
SERVO_OUTPUT = "trajectory"

# -------------------------------------------------------------------- Keyboard
KEYBOARD_LINEAR_SPEED = 0.05     # m/s, starting speed
KEYBOARD_ANGULAR_SPEED = 0.3     # rad/s, starting speed
# Ceilings for the +/- speed keys.
KEYBOARD_MAX_LINEAR_SPEED = 0.25     # m/s
KEYBOARD_MAX_ANGULAR_SPEED = 1.0     # rad/s
# Factor applied per + press (- divides by it).
KEYBOARD_SPEED_STEP = 1.25
# Seconds a key keeps moving after an auto-repeat press.
KEYBOARD_HOLD_TIME = 0.15
# Seconds a key keeps moving after the first press (covers the OS auto-repeat delay).
KEYBOARD_FIRST_PRESS_HOLD = 0.6
KEYBOARD_PUBLISH_RATE = 50.0     # Hz

# -------------------------------------------------------------------- Recorder
RECORDER_SAMPLE_RATE = 25.0      # Hz

# Path logs written while GREY / '[' recording is on: Touch target + raw stylus and
# the UR's own (RTDE) TCP pose, at full rate, into LOG_DIR/NEW_RUN/. NEW_RUN is
# deleted when the next recording starts, so rename it to keep a run. With
# LOG_KEEP_ALL_RUNS every run is also copied to LOG_DIR/runs/<date_time>/.
LOG_DIR = "~/ros2_ws/src/MacroMicroSurgicalRobot/logs"
LOG_KEEP_ALL_RUNS = False
# Without tcp_pose_broadcaster (mock hardware) the UR path falls back to TF
# base_link -> tool0, sampled at this rate (Hz).
LOG_TF_FALLBACK_RATE = 200.0
# Topics also recorded into LOG_DIR/NEW_RUN/rosbag (ros2 bag, mcap) during a
# recording; an empty list turns the bag off. Topics that do not exist (e.g.
# the Touch ones in a keyboard run) are simply skipped.
LOG_BAG_TOPICS = [
    "/phantom/state",
    "/phantom/button",
    "/ur5e_touch_teleop/target_pose",
    "/ur5e_touch_teleop/motion_scale",
    "/servo_node/delta_twist_cmds",
    "/servo_node/status",
    "/scaled_joint_trajectory_controller/joint_trajectory",
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
# 'home' pose: elbow up, tool pointing down - away from singularities.
HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# --------------------------------------------------------------- Stylus replay
# stylus_replay.py: seconds the stylus is held still after the recording starts
# (ros2 bag record takes up to ~1 s to subscribe) and after the replay ends (so the
# robot settling is in the logs).
REPLAY_LEAD_IN = 1.5
REPLAY_TAIL = 1.5
