# A Low-Cost Teleoperable Surgical Robot with a Macro-Micro Structure and a Continuum Tip for Open-Source Research

ROS 2 package providing simultaneous teleoperation of a custom micro-manipulator and a Universal Robots UR5e robotic arm via a 3D Systems Touch haptic stylus.

> **Note:** This package was originally written for **ROS 1 Noetic (catkin)** and has been migrated to **ROS 2 Jazzy (ament_cmake / colcon)**. The control logic is unchanged; only the ROS plumbing and build system were ported. See _ROS 2 dependencies & known external gaps_ below for packages you must obtain separately.

Additional dependencies and their installation instructions can be found in these repositories:

[https://github.com/LMBScott/motor_angles_msg/](https://github.com/LMBScott/motor_angles_msg/)

[https://github.com/LMBScott/Serial_Control/](https://github.com/LMBScott/Serial_Control/)

## Package contents

| Node                        | Language | Purpose                                                                                                                   |
| --------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------- |
| `ur5e_control`              | C++      | Touch → UR5e cartesian teleoperation (tf2 + controller-manager service load/switch + `FollowCartesianTrajectory` action). |
| `micro_module_control`      | C++      | Touch → micro-manipulator Jacobian control (Eigen, `motor_angles_msg`).                                                   |
| `android_pose_subscriber`   | C++      | Touch → rviz marker visualisation.                                                                                        |
| `android_pose_publisher.py` | Python   | Android-phone websocket → `PoseStamped`.                                                                                  |

## ROS 2 dependencies & known external gaps

These are **not** part of this repository and must be present in your colcon workspace before the package will build/run:

- **`omni_msgs`** — provides `OmniState` / `OmniButtonEvent`. Supplied by the ROS 2 Geomagic/3D-Systems Touch driver. Source-build it into your workspace.
- **`cartesian_control_msgs`** — provides the `FollowCartesianTrajectory` action. Ships with the ROS 2 UR driver / `cartesian_controllers`. Try `sudo apt install ros-jazzy-cartesian-control-msgs`, otherwise source-build.
- **`motor_angles_msg`** — the previous student's _custom_ message package (link above) is **ROS 1 only**. A small ROS 2 `rosidl` port (one `MotorAngles.msg` with the four `*_angle` fields) is required before `micro_module_control` will build.
- Runtime drivers: the **ROS 2 UR driver**, a **ROS 2 Touch driver**, and a replacement for `rosserial` (e.g. **micro-ROS** or a serial bridge) for the micro-module Arduino.

## Installation

1. Install **Ubuntu 24.04 LTS**.
2. (For real UR5e control) Apply a real-time kernel patch — see the [UR ROS 2 driver real-time docs](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
3. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):
   ```
   sudo apt install ros-jazzy-desktop
   ```
4. Install build tooling and core dependencies:
   ```
   sudo apt install python3-colcon-common-extensions python3-rosdep \
       ros-jazzy-tf2-geometry-msgs ros-jazzy-controller-manager-msgs \
       ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
       ros-jazzy-xacro libeigen3-dev python3-websocket
   sudo rosdep init   # skip if already initialised
   rosdep update
   ```
5. Create a colcon workspace and clone this repository into it:
   ```
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone <this-repo-url> ls_thesis
   ```
6. Clone/source-build the external packages listed in _ROS 2 dependencies & known external gaps_ into `~/ros2_ws/src` as well (`omni_msgs`, `cartesian_control_msgs`, a ROS 2 `motor_angles_msg`).
7. Install the [Geomagic Touch driver](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers) (ROS 2 fork) and the [OpenHaptics SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US). For a 64-bit system, create the symbolic links:
   ```
   sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
   sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
   sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
   sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0
   ```
   Run `./Touch_Setup` and `./Touch_Diagnostic` (with the Touch connected) to verify communication.
8. Resolve dependencies and build:
   ```
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

> Thanks to Mustafa Sevinc for writing the original installation instructions.

## ROS 2 Startup Commands

> ROS 2 has no `roscore`; nodes discover each other automatically. `source ~/ros2_ws/install/setup.bash` in every terminal first.

### Visualisation only (no hardware)

```
ros2 launch ls_thesis display.launch.py
```

### Touch Stylus

1. Add permissions for the touch USB device: `sudo chmod 777 /dev/ttyACM0`
2. Start the Touch driver (command depends on your ROS 2 Touch driver package, e.g.):
   `ros2 launch omni_common omni_state.launch.py`

### UR5e

1. Start the UR ROS 2 driver:
   ```
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
       robot_ip:=192.168.0.100 \
       kinematics_params_file:=/home/lachlan/lab_ur5e_1_calibration.yaml
   ```
2. Start the LS_ROS_CONTROL program on the UR5e Teach Pendant.
3. Start the control node: `ros2 run ls_thesis ur5e_control`

### Micro Module

1. Add permissions for the Arduino device: `sudo chmod 777 /dev/ttyACM1`
2. Start the serial bridge for the micro-module microcontroller (micro-ROS agent or serial bridge replacing `rosserial`).
3. Start the control node: `ros2 run ls_thesis micro_module_control`

NOTE: Be sure to manually disable power to the servo motors via the switch on the micro module before unplugging the control USB cable.

📌 UR5e + LS Thesis System — Installation & Startup Notes
🟡 Current Status (as of last session)
ROS 2 Jazzy installed and working
ls_thesis workspace builds successfully with colcon
External dependencies partially installed:
omni_msgs (Geomagic Touch interface)
cartesian_control_msgs (built from source)
motor_angles_msg (local source build)
ur_robot_driver (installed from source — now available)
❌ Current Missing / Incomplete Setup

1. UR5e Robot Setup (NOT DONE YET)
   Robot has not been powered on
   Teach pendant has not been configured
   Remote control mode not enabled
   External Control URCap may not be installed
   Robot IP not confirmed
   No verified network connection to robot
2. Touch Device (NOT REQUIRED FOR FIRST TEST)
   Geomagic Touch driver currently unstable
   USB “resource busy” errors present
   Not required for initial UR5e bring-up
3. UR ROS Control Runtime (NOT RUNNING YET)
   ur_robot_driver installed but not launched in real mode
   Controller manager not active
   No active /controller_manager services
   🚀 Full System Startup Procedure (REAL ROBOT MODE)
   🔴 STEP 1 — Power on UR5e robot
   Turn on UR control box
   Wait for teach pendant to boot
   Press ON / Start Robot
   Wait until robot is fully initialized (motors enabled)
   🔵 STEP 2 — Configure teach pendant
   Ensure robot is not running any program
   Switch to:
   Remote Control mode
   Ensure system is idle (no active motion program)
   🟡 STEP 3 — Network verification

On laptop:

ping <robot_ip>

Expected:

stable replies from robot IP (e.g. 192.168.0.100)
🟢 STEP 4 — Start UR ROS 2 driver (REAL MODE)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
 ur_type:=ur5e \
 robot_ip:=<robot_ip>

Expected logs:

UR hardware interface initialized
controller_manager active
joint_state_broadcaster running
🔵 STEP 5 — Verify ROS control is active
ros2 service list | grep controller

Expected:

/controller_manager/load_controller
/controller_manager/list_controllers
🟣 STEP 6 — Run thesis control node
ros2 run ls_thesis ur5e_control

System should now:

connect to controller manager
load required controllers
begin teleoperation pipeline
⚠️ Safety Notes
Robot may move immediately when control node starts
Ensure workspace is clear before enabling control
Keep emergency stop accessible at all times
Start with low-speed / small motion tests
🧠 System Architecture (final intended state)
Touch device (optional, later)
↓
ls_thesis teleop nodes
↓
ur_robot_driver (ROS 2 control)
↓
UR5e robot (real hardware)
🟡 Next Steps After Basic Setup Works
Validate controller loading sequence
Test 1 cm Cartesian motion command
Re-enable Touch device input
Integrate full teleoperation loop (Touch → UR5e)


🧠 Categories of Errors
1. ❌ Runtime / system configuration errors (NOT code fixes)

These are environment/robot issues:

🚫 UR5e not connected / controller manager missing

Error:

Failed to call load_controller service
Could not contact /controller_manager

Cause:

UR driver not running
robot not in Remote mode
External Control program not active
wrong IP / network

Fix (NOT code):

start UR driver first
ensure robot is running + remote mode enabled
🚫 UR driver cannot connect to robot

Error:

Failed to connect to robot on IP ...:30001/30002/30004

Cause:

wrong IP
robot off / not booted
no network access
missing UR External Control program

Fix (NOT code):

network setup + robot startup procedure
🚫 Touch device failure

Error:

Failed to initialize haptic device
LibUSB Error: Resource busy

Cause:

USB already claimed by another process
driver conflict
hardware issue

Fix (NOT code):

kill other processes
restart driver / unplug device
2. ⚠️ Build system / dependency issues (PARTLY code-related)
⚠️ Missing ROS packages (APT / rosdep failures)

Example:

Unable to locate package ros-jazzy-cartesian-control-msgs

Meaning:

dependency not available in binary repo

Fix options:

✔ correct: build from source (you did this)
✔ OR remove dependency if unused

Code-side improvement:

document clearly in README
optionally wrap dependency as optional build
⚠️ Incorrect dependency declarations in package.xml
Example issue:
<depend>eigen</depend>

Problem:

eigen is not a ROS package

Fix:
Replace with:

<build_depend>eigen3_cmake_module</build_depend>

or better:

remove entirely if only used via CMake find_package(Eigen3)
3. ⚠️ Code-level issues in your thesis package (REAL IMPROVEMENTS)

These are the most important “fix in code” items.

🧩 Issue A — Hard dependency on controller manager at startup
Problem:

Your node:

immediately tries to call load_controller
exits if service is unavailable
Why this is fragile:
requires strict launch order
breaks in simulation or partial startup
causes immediate failure like you saw
✔ Fix (recommended)

Add service waiting + retry loop:

if (!client->wait_for_service(10s)) {
    RCLCPP_WARN(node->get_logger(),
        "Controller manager not ready, retrying...");
    return;
}

or better:

async retry timer instead of exit
🧩 Issue B — No “simulation-safe mode”
Problem:

ur5e_control assumes:

real controller manager exists
robot is present
Fix:

Add parameter:

declare_parameter("use_real_robot", true);

Then:

if (!use_real_robot) {
    RCLCPP_INFO(..., "Skipping controller setup (simulation mode)");
    return;
}
🧩 Issue C — Tight coupling between Touch + robot startup
Problem:

System expects both:

/omni/state
UR controller manager

to exist at the same time.

This caused cascading failure.

✔ Fix:

Split responsibilities:

Touch node = independent publisher
UR node = independent consumer
teleop node = bridge only

Add checks:

if (!topic_exists("/omni/state")) {
    RCLCPP_WARN(..., "Touch not available, running idle mode");
}
🧩 Issue D — No graceful degradation on missing inputs
Problem:

Nodes crash or fail when:

topic missing
service missing
robot absent
✔ Fix:

Use “soft failure” pattern:

never exit immediately
instead:
log warning
keep node alive
retry periodically
🧩 Issue E — Hardcoded IP / robot assumptions
Problem:

Robot IP is often embedded or assumed.

✔ Fix:

Move to parameters:

declare_parameter("robot_ip", "192.168.0.100");

Launch file sets it.

🧩 Issue F — Controller loading not abstracted
Problem:

Direct calls like:

load_controller()
switch_controller()

inside main logic.

✔ Fix:

Encapsulate:

class ControllerInterface {
public:
    bool load();
    bool activate();
    bool ready();
};

This makes:

simulation support easier
debugging cleaner
startup order safer