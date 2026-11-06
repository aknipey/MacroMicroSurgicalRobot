# Touch force diagnostics

Used on 2026-10-19 to find why no haptic force reached the Touch (see claude-notes/VERSIONS.md,
V7 part 2, "No force on the real Touch"). Both measure how far a **free-resting pen** (out of the
inkwell, nobody touching it) rises under a ramped upward force, from the encoders, so the
result does not depend on what anyone feels.

- `lifttest.c`: OpenHaptics only, no ROS. Stop every Touch driver first (one program per device).
  ```bash
  g++ -std=gnu++17 -o lifttest lifttest.c -lHD -lHDU -lrt -lpthread
  ./lifttest demo 1.0        # HelloHapticDevice's call order, 1 N up
  ./lifttest driver 1.0      # omni_state's exact calls and order
  ```
  Linking it with ROS (`-Wl,--no-as-needed -lrcl`) or `LD_PRELOAD`ing libnuma gives 0 mm:
  that is the bug touch_server works around. Must be `gnu++17` (the HD headers need `linux`).
- `ros_lift.py`: the same through ROS (`/phantom/force_feedback` -> `/phantom/state`), with
  `ros2 launch macro_micro_robot touch_driver.launch.py` running. Pass the force in N.

A healthy Touch lifts the pen ~150-175 mm at 1 N in both.
