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
# Robot radians per stylus radian.
TOUCH_ROTATION_SCALE = 1.0
TOUCH_ENABLE_ROTATION = True

# Top tool speeds. These are the hard caps: if the robot lags your hand on fast
# moves, raise these first.
TOUCH_MAX_LINEAR_SPEED = 1.2    # m/s
TOUCH_MAX_ANGULAR_SPEED = 15    # rad/s

# How hard the tool is pulled towards the stylus target (1/s). Commanded speed =
# gain * error, so with gain 4 a 1 cm error gives 4 cm/s. Higher = snappier and
# less lag on small moves, but too high oscillates.
TOUCH_POSITION_GAIN = 8.0
TOUCH_ROTATION_GAIN = 5.0

# Position errors smaller than this are ignored (m).
TOUCH_DEADBAND = 3e-4
# Stop if no stylus data arrives for this long (s).
TOUCH_TIMEOUT = 0.1
# Rate twist commands are sent to Servo (Hz).
TOUCH_PUBLISH_RATE = 100.0
# omni_state publishes millimetres.
TOUCH_UNITS_TO_M = 1e-3

# "toward_base" or "fixed" (fixed uses TOUCH_AXIS_MAP: stylus axis per robot X, Y, Z).
TOUCH_MAPPING = "toward_base"
TOUCH_AXIS_MAP = ["y", "-x", "z"]

# Seconds the launch file waits for the driver + Servo before starting the Touch node.
TOUCH_STARTUP_DELAY = 8.0

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

# ---------------------------------------------------------------------- Player
PLAYER_DEFAULT_SPEED = 1.0       # playback speed factor
PLAYER_MAX_SPEED = 2.0           # --speed is refused above this
PLAYER_APPROACH_TIME = 5.0       # s to move to the first recorded pose
# Recorded samples closer together than this are dropped (s).
PLAYER_MIN_POINT_SPACING = 0.04
# 'home' pose: elbow up, tool pointing down - away from singularities.
HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
