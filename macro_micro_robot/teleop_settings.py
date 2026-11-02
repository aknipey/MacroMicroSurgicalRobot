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
# TOUCH_TWIST_SCALE. 1 = the same as translation (1:5 -> rotation 1:5, 1:20 -> 1:20);
# 0.5 = square root (1:2.24 / 1:4.47, before 2026-10-30), 0 = rotation never scaled.
TOUCH_SCALE_ROTATION_EXPONENT = 1.0
# Two WHITE presses this close together (s, press to press) are a double click.
# The robot still follows the stylus during both presses - nothing waits for this.
TOUCH_DOUBLE_CLICK_TIME = 0.3
# Holding GREY this long (s) without touching WHITE switches the stylus between the
# macro (UR arm) and micro (micro-manipulator) module. A shorter GREY click still
# starts / stops recording, on release.
TOUCH_MODE_SWITCH_HOLD_TIME = 1.0
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

# Singularity guard (SingularityGuard in ur16e_common): works on each twist before Servo,
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

# RCM (remote centre of motion) mode: hold WHITE and click GREY to latch the tool tip
# as a fixed pivot point (the port / incision); do it again (tool retracted) to leave.
# While on, the stylus drives the tool tip, the arm pivots the shaft about the RCM
# point to put it there, and pen twist rolls the tool about the shaft (pen tilt is
# ignored). The tool is described here only - keep the pendant TCP at zero.
# Shaft geometry in the tool0 frame (z out of the flange): the shaft starts at
# SHAFT_START (the end at the holder), runs along SHAFT_DIRECTION (normalised) and
# ends at the tip SHAFT_LENGTH further on. Only points on that segment can be the RCM.
# Measure your tool.
# Tool = NDI wand 8700340, calibrated 2026-10-26 (wand re-seated in the holder):
# tool0 -> wand hand-eye fit (wrist joints jogged, no divot) + the wand's own markers ->
# tip calibration (2026-10-20). Tip at (-0.41, 9.84, 366.87) mm in tool0, shaft 1.05 deg
# off tool0 z, tip checked against the camera to 0.82 mm rms. LENGTH = tip back to the
# wand's first marker along the shaft.
# src/ndi_tracking/calibrations/robot_tcp_handeye_2026-10-26.json. 2026-10-22 (old seat):
# [-0.0006, 0.00662, 0.20624] / [-0.00607, 0.01481, 0.99987], tip 2.5 mm from today's.
# Only used when TOOL_LIBRARY has no tools: the library (below) is where tools live now.
TOUCH_RCM_SHAFT_START = [0.00008, 0.00699, 0.20827]     # m
TOUCH_RCM_SHAFT_DIRECTION = [-0.00309, 0.01798, 0.99983]
TOUCH_RCM_SHAFT_LENGTH = 0.1586               # m
# Numbered library of calibrated tools (macro_micro_robot/tools.py). The teleop starts on the
# library's `default` tool; type a number in the launch_system.sh terminal (or
# `ros2 param set /ur16e_touch_teleop tool N`) to switch. Add tools with
# `ros2 run macro_micro_robot tool_library.py add ...`. The file is re-read on every switch.
TOOL_LIBRARY = "~/ros2_ws/src/MacroMicroSurgicalRobot/config/tools.yaml"
# A switch is refused in RCM mode, while WHITE drives the arm, or unless every joint
# has stayed under this speed (rad/s) for TOOL_SWITCH_STILL_TIME (s).
TOOL_SWITCH_MAX_JOINT_SPEED = 0.01
TOOL_SWITCH_STILL_TIME = 0.5
# Closest the shaft start (holder / collar) may come to the RCM point (m), so the
# deepest insertion is SHAFT_LENGTH - this.
TOUCH_RCM_DEPTH_MARGIN = 0.02
# Sideways stylus motion moves tool0 (not the tip) by this many times the (scaled)
# stylus motion, the same at every depth: the tip gets gain * depth / lever, lever =
# tool0 -> RCM point. The tip's share goes to 0 at the RCM point by itself, so the
# pivot per mm stays finite (gain / lever) right down to depth 0, and going in or out
# with the stylus held to one side does not swing tool0 sideways.
# 0 = off (the tip follows the stylus 1:1, the arm's swing grows as 1 / depth).
TOUCH_RCM_TOOL0_GAIN = 1.0
# Deepest insertion (m) the constant-tool0 law holds to. Deeper, the tip keeps the
# share it had here: the lever gets short, so the pivot per mm would keep growing.
TOUCH_RCM_TOOL0_GAIN_DEPTH = 0.13
# Optional extra fade (m) of the sideways motion near the RCM point (smootherstep,
# 0 at depth 0, full at this depth), raised to TOUCH_RCM_PIVOT_FADE_STRENGTH (>= 1).
# 0 = off. With the tool0 gain on, keep it off: sideways stylus motion held back by
# the fade comes back as the tool goes deeper, so moving in and out across the fade
# swung tool0 sideways at ~4x the stylus speed (V7 real arm run).
TOUCH_RCM_PIVOT_FADE_DEPTH = 0.0
TOUCH_RCM_PIVOT_FADE_STRENGTH = 1.0
# How far (m) the tip may be pulled back out past the RCM point, so the tool can come
# fully out of the port before leaving RCM mode. The shaft line still runs through
# the point out there.
TOUCH_RCM_RETRACT_DEPTH = 0.005
# RCM mode can only be left with the tip within this depth (m) of the RCM point
# (anywhere outside it, up to TOUCH_RCM_RETRACT_DEPTH out, also counts).
TOUCH_RCM_EXIT_DEPTH = 0.003
# How hard the shaft is pulled back onto the RCM point when it drifts off (1/s),
# capped at TOUCH_MAX_CORRECTION_SPEED like the position correction.
TOUCH_RCM_GAIN = 8.0
# If the shaft is ever this far (m) from the RCM point, the robot holds still and
# RCM mode may be left at any depth.
TOUCH_RCM_MAX_ERROR = 0.005

# Haptic cues (force feedback on the stylus, macro_micro_robot/haptics.py), all while WHITE is
# held: a soft wall at the deepest RCM insertion and at the retract limit, a click as
# the tip target crosses the RCM point, a small tap when RCM mode turns on or off (two
# when it refuses to turn off), and a push out of a singularity.
# Distances are in *stylus* metres, so the cues feel the same at every motion scale.
# Each cue's force / stiffness = 0 turns just that one off. False = no forces at all.
TOUCH_HAPTICS = True
# Hard cap on the total force (N). The Touch gives 3.3 N peak, 0.88 N continuous.
# Strengths are set from bare-OpenHaptics tests on SN 22129000559 (2026-10-19, VERSIONS.md
# V7 part 2): nothing is felt below ~0.0008 N, 0.001 N is soft but felt, 0.01 N is about as
# much as is comfortable (0.1 N felt huge). Its force output does not follow the command
# well (the same kick for 0.001-0.01 N steps), so cues stay just past the threshold.
TOUCH_HAPTIC_MAX_FORCE = 0.01
# Fastest the force may change (N/s): rounds off every step (release, entering a wall),
# as a jump in force can trip the Touch's max-force-impulse check.
TOUCH_HAPTIC_SLEW_RATE = 2.0           # 0.01 N in 5 ms
# RCM depth walls: N per m of stylus past the limit, one sided (like TouchCheckup's Box,
# which feels smooth: free inside, pushed back only when pressing into a wall). They are
# rendered by touch_server in the Touch's own 1 kHz loop (needs macro_micro_robot
# touch_driver.launch.py): computed here and sent over ROS they arrived tens of ms late
# and shook the pen violently (2026-10-19). Only the total force cap above applies.
# 0 = off. On SN 22129000559 any spring wall buzzed at its edge: its force passes the
# ~0.0008 N threshold where the Touch's fixed kick switches on (logs NEW_RUN 17:28, ~2 mm
# shake at 0.0005-0.0015 N). The limits are marked by a click instead (below).
TOUCH_HAPTIC_WALL_STIFFNESS = 0.0       # N/m (2.0 put the 0.01 N cap 5 mm into the wall)
# Click when the tip target crosses the RCM point: one 40 ms tap (N) against the direction
# of travel. It replaces a detent spring, which pulled the pen towards the point like
# HelloHapticDevice's gravity well and, held in the hand, shook it (2026-10-19). Taps are
# open loop and cannot. The side changes only once the target is this far (m of stylus)
# past the point, so hovering at the point does not click over and over.
TOUCH_HAPTIC_CROSSING_FORCE = 0.002
TOUCH_HAPTIC_CROSSING_HYSTERESIS = 0.001
# Minimum time (s) between two crossing clicks. On the real Touch each click shook the pen
# ~10 mm, which carried it back across the point and fired the next click 40-60 ms later,
# a self-sustaining buzz (logs NEW_RUN, 2026-10-19). The side is still tracked meanwhile.
TOUCH_HAPTIC_CROSSING_HOLDOFF = 0.3
# Click (N) when the tip target reaches the deepest insertion or the retract limit, pushing
# back out of the limit. Open loop like the crossing click, with the same hysteresis and
# hold-off, so it cannot chatter the way a wall did. The arm already stops at the limit.
TOUCH_HAPTIC_LIMIT_FORCE = 0.002
# RCM on / off tap: one small upward push (60 ms) when RCM mode turns on or off, two
# when leaving is refused. Force (N); frequency (Hz) > 0 makes each tap a buzz instead
# (0.4 N at 40 Hz was far too much on the real Touch).
TOUCH_HAPTIC_PULSE_FORCE = 0.002
TOUCH_HAPTIC_PULSE_FREQUENCY = 0.0
# Singularity: from TOUCH_SINGULARITY_SLOW (0) to TOUCH_SINGULARITY_STOP (full), push the
# stylus with up to this force (N) in the direction that moves the arm out of it, plus
# damping (N s/m) on stylus motion heading in. Only the stylus translation is pushed
# (the Touch cannot twist the pen), so a singularity reached by tilting the pen is
# pushed back by moving the pen sideways. Needs TOUCH_SINGULARITY_GUARD.
TOUCH_HAPTIC_SINGULARITY_FORCE = 0.005
TOUCH_HAPTIC_SINGULARITY_DAMPING = 0.015
# Low-pass time constant (s) on the singularity push: the condition number and the
# direction out jitter with the arm.
TOUCH_HAPTIC_SINGULARITY_FILTER = 0.03

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
# (the UR runs speedj, no lookahead), "position" = Servo's joint velocities through
# joint_velocity_limiter, which integrates them into joint positions for
# forward_position_controller (the UR runs servoj, its lag cancelled by
# SERVO_POSITION_LEAD). speedj buzzed / ground in some poses on the UR16e (a kick
# every 2 ms set point); servoj never did. Read when Servo starts: needs a relaunch.
SERVO_OUTPUT = "position"
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
# Run config/external_control_smooth_speedj.urscript on the robot instead of the
# driver's stock script. The stock speed thread reaches every 2 ms joint speed set point
# at a fixed 40 rad/s^2, i.e. in a fraction of the step: a torque kick per step. In the
# V7 runs the big joints' current then carried 0.3-1.0 A RMS at 120-250 Hz while the
# arm accelerated (the buzz / grind; 0.04 A in a slow run), with no such content in the
# command itself. The copy spreads each change over the whole step (as the driver's own
# spline mode does), so the joint speed ramps between set points. Real arm only (mock
# hardware runs no URScript). Read at launch: needs a relaunch.
SERVO_SMOOTH_SPEEDJ = True
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

# ------------------------------------------------------- Position mode (only)
# Position mode always runs joint_velocity_limiter (it does the integrating), with the
# VELOCITY_LIMITER_* caps and smoothing above and Servo at SERVO_VELOCITY_RATE.
# How far ahead (s) of the integrated position each command is sent, at the current
# joint speed: q + qd * lead. The UR's servoj (driver: lookahead 0.03 s, the minimum,
# gain 2000) reaches a set point ~16 ms dead time + ~15 ms lag later (ur_emulator's
# fit to the V3 logs), so 0.03 cancels that at steady speed. 0 = plain servoj lag.
SERVO_POSITION_LEAD = 0.03
# Furthest (rad, per joint) the integrated position may run ahead of the measured
# joints. Normal tracking at the speed caps is ~0.08 rad (2.5 rad/s x ~31 ms); past
# this the arm is not following (stopped, blocked) and the command is held back
# instead of winding up.
SERVO_POSITION_MAX_ERROR = 0.15

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
# True = refuse to start a recording (GREY button or replay) unless the NDI tracker
# (`python3 -m ndi_tracking run`, src/ndi_tracking) is following it, so no run ends
# up without NDI data. False = record anyway, with a warning at start and stop and
# META.yaml ndi.tracker_at_start: false. Off since 2026-10-30 (user's choice); it was
# added 2026-10-26 after replays were recorded without NDI unnoticed.
LOG_REQUIRE_NDI = False
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
    "/phantom/force_feedback",  # haptic cues sent to the stylus (N)
    "/phantom/applied_force",   # force touch_server actually rendered, incl. RCM walls / detent (N)
    "/ur16e_touch_teleop/target_pose",
    "/ur16e_touch_teleop/motion_scale",
    "/ur16e_touch_teleop/condition_number",     # singularity guard: arm Jacobian condition number
    "/ur16e_touch_teleop/rcm_active",           # RCM mode on / off
    "/ur16e_touch_teleop/rcm_point",            # latched RCM point (base frame)
    "/ur16e_touch_teleop/rcm_error",            # distance shaft line -> RCM point (m)
    "/ur16e_touch_teleop/insertion_depth",      # tip depth past the RCM point (m)
    "/ur16e_touch_teleop/tool_tip",             # tool tip pose (shaft axis = its z)
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
