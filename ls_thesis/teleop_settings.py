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
# In velocity mode joint_velocity_limiter caps each joint, so the angular cap only
# needs to bound the tool: 2.0 clipped 16-26% of the V5 benchmarks; 3.0 cut their
# angle error (lag removed) from 15.8 to 9.6 deg in the sim.
TOUCH_MAX_ANGULAR_SPEED = 3.0    # rad/s

# How hard the tool is pulled towards the stylus target (1/s). Commanded speed =
# gain * error, so with gain 4 a 1 cm error gives 4 cm/s. Higher = snappier and
# less lag on small moves, but too high oscillates.
TOUCH_POSITION_GAIN = 8.0
TOUCH_ROTATION_GAIN = 8.0
# Caps on what the gains alone may add (m/s, rad/s; 0 = no cap). When the robot falls
# behind - a speed cap or the joint velocity limiter held it back during a fast pen
# twist - the error grows, and gain x error then asks for a surge to catch up (up to
# 1.7 rad/s extra on the real arm), which feels jerky. Normal tracking needs far less
# (99% of linear corrections < 0.015 m/s, 90% of angular < 0.11 rad/s), so only the
# catch-up is slowed.
TOUCH_MAX_CORRECTION_SPEED = 0.05           # m/s
TOUCH_MAX_CORRECTION_ANGULAR_SPEED = 0.5    # rad/s

# Velocity feedforward: also command the stylus's own velocity, so the gains above
# only correct the remaining error instead of producing all of the motion. Roughly
# halves the lag on mock hardware. 0 = off, 1 = full stylus velocity.
TOUCH_FEEDFORWARD = 1.0
# Low-pass time constant on the stylus velocity estimate (s). Larger = smoother but
# more lag; smaller = jitter from the stylus encoders reaches the robot.
TOUCH_FEEDFORWARD_FILTER = 0.02
# Low-pass cutoff (Hz) on the whole twist sent to Servo (feedforward + gains), as a
# 2nd-order Butterworth. In velocity mode Servo's own smoothing filter only touches the
# positions it no longer sends, so without this the stylus and loop jitter (~20 Hz)
# reaches the arm unfiltered and shakes it. Lower = smoother but more lag. Sim, on a
# real 1:1 hand run: 8 Hz -> 6 Hz cut tool jerk by a quarter for ~2-4 ms more lag;
# 5 Hz cut it by a half for ~5 ms more; on the V5 benchmarks 6 -> 5 Hz cut the 5-15 Hz
# jitter by 13%. Must be below half TOUCH_PUBLISH_RATE. 0 = off.
TOUCH_COMMAND_FILTER = 5.0
# Kalman filter on the raw stylus pose (constant-velocity model; the orientation is an
# extended/error-state version). omni_state repeats each pose for ~4 ms between real
# updates, which makes the feedforward velocity jump; the filter uses only the real
# updates and predicts in between. On the V3 benchmarks it cuts the feedforward noise
# by 17-42% without adding lag (it is 0.5-2 ms ahead), unlike TOUCH_COMMAND_FILTER.
# True = on.
TOUCH_KALMAN = True
# How quickly the hand's velocity may change (process noise). Lower = smoother, but the
# target runs on past a sudden stop: at 1:1 a stop from 200 mm/s overshoots 0.13 mm
# with 2.0 but 1.2 mm with 0.1; from 3 rad/s 0.6 deg with 30 (1.0 deg with 10).
TOUCH_KALMAN_ACCEL_NOISE = 2.0           # (m/s^2)^2 s
TOUCH_KALMAN_ANGULAR_ACCEL_NOISE = 30.0  # (rad/s^2)^2 s
# How much a single stylus reading is trusted (measurement noise, 1 sigma). Higher =
# smoother but follows each reading less closely (and overshoots more).
TOUCH_KALMAN_POSITION_NOISE = 5e-4       # m
TOUCH_KALMAN_ROTATION_NOISE = 0.01       # rad
# Time (s) between a command and the robot pose showing it (Servo lookahead + driver
# + robot). The gains compare the robot with where the target was this long ago,
# so they don't keep pushing for motion that is already on its way (overshoot).
# 0 = compare with the current target, as before.
TOUCH_LOOP_DELAY = 0.05

# Singularity guard (SingularityGuard in ur5e_common): works on each twist before Servo,
# using the condition number c of the arm Jacobian (the number Servo uses; ~8 at HOME).
# On a UR5e/UR16e, c = 30 / 60 is wrist 2 about 15 / 8 deg from 0 (or 180), or the
# elbow about 18 / 11 deg from straight. From SLOW, motion *into* the singularity is
# slowed (all of it gone at STOP) and joint speeds are damped; along or away from it is
# still free, so the tool slides along the edge instead of stalling. Past STOP the arm
# is pushed back out (at most ESCAPE_SPEED rad/s), as long as WHITE is held. While it is
# on, Servo's own all-or-nothing check is moved out of the way (SERVO_SINGULARITY_*_GUARDED).
# Needs TOUCH_POSE_SOURCE "joint_states". False = off (Servo's check only).
TOUCH_SINGULARITY_GUARD = True
TOUCH_SINGULARITY_SLOW = 30.0
TOUCH_SINGULARITY_STOP = 60.0
# Largest damped-least-squares damping (in Jacobian singular value units, ~m). Caps the
# joint speed at 1 / (2 x this) rad/s per m/s (or rad/s) of tool speed. In a kinematic
# sim of random strokes into singularities, 0.03 -> 0.05 cut the biggest cycle-to-cycle
# twist step from 0.12 to 0.03 and followed the stylus more closely (stronger damping
# needs less of the blunt toward-the-singularity cut).
TOUCH_SINGULARITY_DAMPING = 0.05
# Back-off joint speed (rad/s) per unit of c / STOP - 1, and its cap.
TOUCH_SINGULARITY_ESCAPE_GAIN = 1.0
TOUCH_SINGULARITY_ESCAPE_SPEED = 0.2

# Position errors smaller than this are ignored (m, at 1:1 - shrinks with the
# double-click motion scale so fine moves are not swallowed).
TOUCH_DEADBAND = 3e-4
# Stop if no stylus data arrives for this long (s).
TOUCH_TIMEOUT = 0.1
# Rate twist commands are sent to Servo (Hz). Match Servo's cycle (SERVO_VELOCITY_RATE
# in velocity mode): slower, Servo holds each command for several cycles and the arm
# feels every velocity step as a kick (a ~100 Hz buzz at 100 Hz, ~250 Hz at 250 Hz).
TOUCH_PUBLISH_RATE = 500.0
# Where the robot's tool pose comes from: "tf" = robot_state_publisher's TF (20 Hz,
# so up to 50 ms old) or "joint_states" = computed from each /joint_states message
# (500 Hz) with the URDF on /robot_description. Use "joint_states" in velocity mode:
# the 20 Hz TF staircase puts a 20 Hz sawtooth (+ 40/60 Hz) into the correction term,
# which velocity mode passes straight to the arm. (In trajectory mode "tf" lagged a
# little less, see VERSIONS.md V4.)
TOUCH_POSE_SOURCE = "joint_states"
# omni_state publishes millimetres.
TOUCH_UNITS_TO_M = 1e-3

# "toward_base" or "fixed" (fixed uses TOUCH_AXIS_MAP: stylus axis per robot X, Y, Z).
TOUCH_MAPPING = "toward_base"
TOUCH_AXIS_MAP = ["y", "-x", "z"]

# Seconds the launch file waits for the driver + Servo before starting the Touch node.
# The node switches on Servo's controller at startup: at 0 the controllers are not
# loaded yet, so in velocity mode no controller is active and the robot never moves.
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
SERVO_OUTPUT = "velocity"
# Servo's own singularity check (condition number thresholds): it scales the whole
# twist down from SLOWDOWN and halts at STOP - and above STOP it halts *every* command,
# even one leading back out (seen on the mock arm at c = 554 with a twist straight out),
# so once there the arm is stuck. The keyboard node sets these (ur_servo.yaml's values);
# the Touch node raises them to the _GUARDED pair while its singularity guard runs, so
# the guard decides (joint_velocity_limiter still caps every joint). Set at node start,
# no relaunch needed.
SERVO_SINGULARITY_SLOWDOWN = 100.0
SERVO_SINGULARITY_STOP = 200.0
SERVO_SINGULARITY_SLOWDOWN_GUARDED = 1e4
SERVO_SINGULARITY_STOP_GUARDED = 1e6

# ------------------------------------------------------- Velocity mode (only)
# Servo's cycle in velocity mode (Hz). 500 = the UR's own 2 ms control cycle, so the
# joint speed set point changes every robot cycle in small steps. At 250 the set point
# held for 2 cycles and then jumped, and the wrists chattered at 250 Hz (torque noise
# 15-20x trajectory mode, highest right before every joint fault). Trajectory mode
# keeps ur_servo.yaml's 250 Hz. Read when Servo starts: needs a relaunch.
SERVO_VELOCITY_RATE = 500.0
# joint_velocity_limiter.py between Servo and forward_velocity_controller: smooths and
# limits each joint's speed and acceleration. True = on.
VELOCITY_LIMITER = True
# Joint speed cap, as a fraction of each joint's URDF limit (UR16e: 120 deg/s base and
# shoulder, 180 deg/s elbow and wrists). Servo itself scales to exactly 1.0, and the
# benchmark faults happened with a wrist right at its limit. All joints are scaled
# together, so the tool keeps its direction.
VELOCITY_LIMITER_SPEED_FRACTION = 0.8
# Joint acceleration caps (rad/s^2), in UR joint order: shoulder pan, shoulder lift,
# elbow, wrist 1, wrist 2, wrist 3. The UR's speedj ramps at 40 rad/s^2; every
# velocity-mode fault happened with a wrist at 28-43 rad/s^2, i.e. switching between
# full acceleration and full deceleration. The wrist cap is near the 99th percentile
# of the V3 benchmarks (25-33 rad/s^2), so only the hardest reversals are slowed.
VELOCITY_LIMITER_MAX_ACCELERATION = [8.0, 8.0, 15.0, 25.0, 25.0, 25.0]
# Smoothing after the limits (s): two first-order stages with this time constant each
# (critically damped, ~2x this in lag). It rounds the corners the caps make when they
# engage and the steps left when a command arrives late (the laptop is not real-time),
# and, unlike a Butterworth, it can never take the output past the speed or
# acceleration caps. 0 = off.
VELOCITY_LIMITER_SMOOTHING = 0.003

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
    "/phantom/joint_states",    # Touch encoder angles: shows a sticking/skipping joint directly
    "/ur5e_touch_teleop/target_pose",
    "/ur5e_touch_teleop/motion_scale",
    "/ur5e_touch_teleop/condition_number",     # singularity guard: arm Jacobian condition number
    "/servo_node/delta_twist_cmds",
    "/servo_node/status",
    "/scaled_joint_trajectory_controller/joint_trajectory",
    "/joint_velocity_limiter/commands",         # Servo's joint velocities (velocity mode)
    "/forward_velocity_controller/commands",    # what the arm is sent (velocity mode)
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
