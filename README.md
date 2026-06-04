# A Low-Cost Teleoperable Surgical Robot with a Macro-Micro Structure and a Continuum Tip for Open-Source Research
ROS 2 package providing simultaneous teleoperation of a custom micro-manipulator and a Universal Robots UR5e robotic arm via a 3D Systems Touch haptic stylus.

> **Note:** This package was originally written for **ROS 1 Noetic (catkin)** and has been migrated to **ROS 2 Jazzy (ament_cmake / colcon)**. The control logic is unchanged; only the ROS plumbing and build system were ported. See *ROS 2 dependencies & known external gaps* below for packages you must obtain separately.

Additional dependencies and their installation instructions can be found in these repositories:

[https://github.com/LMBScott/motor_angles_msg/](https://github.com/LMBScott/motor_angles_msg/)

[https://github.com/LMBScott/Serial_Control/](https://github.com/LMBScott/Serial_Control/)

## Package contents

| Node | Language | Purpose |
| --- | --- | --- |
| `ur5e_control` | C++ | Touch → UR5e cartesian teleoperation (tf2 + controller-manager service load/switch + `FollowCartesianTrajectory` action). |
| `micro_module_control` | C++ | Touch → micro-manipulator Jacobian control (Eigen, `motor_angles_msg`). |
| `android_pose_subscriber` | C++ | Touch → rviz marker visualisation. |
| `android_pose_publisher.py` | Python | Android-phone websocket → `PoseStamped`. |

## ROS 2 dependencies & known external gaps

These are **not** part of this repository and must be present in your colcon workspace before the package will build/run:

- **`omni_msgs`** — provides `OmniState` / `OmniButtonEvent`. Supplied by the ROS 2 Geomagic/3D-Systems Touch driver. Source-build it into your workspace.
- **`cartesian_control_msgs`** — provides the `FollowCartesianTrajectory` action. Ships with the ROS 2 UR driver / `cartesian_controllers`. Try `sudo apt install ros-jazzy-cartesian-control-msgs`, otherwise source-build.
- **`motor_angles_msg`** — the previous student's *custom* message package (link above) is **ROS 1 only**. A small ROS 2 `rosidl` port (one `MotorAngles.msg` with the four `*_angle` fields) is required before `micro_module_control` will build.
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
6. Clone/source-build the external packages listed in *ROS 2 dependencies & known external gaps* into `~/ros2_ws/src` as well (`omni_msgs`, `cartesian_control_msgs`, a ROS 2 `motor_angles_msg`).
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
