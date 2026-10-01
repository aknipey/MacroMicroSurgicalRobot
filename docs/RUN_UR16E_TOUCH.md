# How to run the UR16e with the Touch stylus

Simple step-by-step guide. Do the steps in order.

---

## Safety first

- Someone must be holding the **emergency stop** (red button on the teach pendant) at all times.
- Nobody inside the robot's reach while it is running.
- Start slow. Only speed up once everything moves the way you expect.
- **Let go of the WHITE button** on the stylus and the robot stops.

---

## Part A — One-time setup

Skip Part A if this computer has already been set up.

### A1. Connect the cable

Plug an Ethernet cable from the robot's control box into the computer (the USB Ethernet adapter).

### A2. Give the computer an IP address

Open a terminal (`Ctrl+Alt+T`) and run:

```bash
nmcli con add type ethernet ifname enx6c6e0740fc7e con-name ur16e ipv4.method manual ipv4.addresses 192.168.0.10/24
nmcli con up ur16e
```

### A3. Set the robot's IP address (on the teach pendant)

1. ☰ menu (top right) → **Settings** → **System** → **Network**
2. Choose **Static**
3. IP address: `192.168.0.100`
4. Subnet mask: `255.255.255.0`

### A4. Set up External Control (on the teach pendant)

1. ☰ → **Settings** → **System** → **URCaps**. Check that **External Control** is in the list.
2. Go to the **Installation** tab → **URCaps** → **External Control**:
   - Host IP: `192.168.0.10`
   - Custom port: `50002`
3. **Save** → **Save Installation As…** → name it `ros_control`.

### A5. Make the robot program (on the teach pendant)

1. **Program** tab → **New** → **Program**
2. Tap **Robot Program** in the tree on the left
3. **URCaps** → **External Control**
4. **Save** → **Save Program As…** → name it `ros_control` (it saves as `ros_control.urp`)

### A6. Set up the Touch stylus

Plug in the Touch by USB, then run:

```bash
~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/bin/Touch_HeadlessSetup
```

It should say `Success. Configured Device : Default Device`.
Run this again if you ever plug in a different Touch.

---

## Part B — Every time

You will use **3 terminals**. Open a new one with `Ctrl+Alt+T`, or with `Ctrl+Shift+T` inside a terminal for a new tab.

### B1. Turn on the robot (teach pendant)

1. Press the power button on the pendant and wait for it to boot.
2. Tap the red **Power off** status in the bottom-left corner.
3. Press **ON**, then **START**. The brakes release with a click.
4. **Open** → **Program** → `ros_control`. Don't press Play yet.

### B2. Check the connection

In any terminal:

```bash
ping -c 3 192.168.0.100
```

- You see `64 bytes from 192.168.0.100` → connected. Go on.
- You see `Destination Host Unreachable` → check the cable, then run `nmcli con up ur16e`.

### B3. Put the stylus in its inkwell

Put the pen in the holder on the Touch base. It calibrates there when the driver starts.

### B4. Terminal 1 — start the robot driver

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ls_thesis ur5e_teleop.launch.py ur_type:=ur16e robot_ip:=192.168.0.100 input:=none
```

Wait until you see **`System successfully started!`**. RViz will also open.

- A red message `calibration parameters ... don't match` is **normal**. Ignore it.
- If you see `Could not get configuration package` → press `Ctrl+C`, wait 5 seconds, and run the last command again.

### B5. Press Play (teach pendant)

Press **▶ Play** at the bottom of the pendant.

Terminal 1 should say **`Robot connected to reverse interface. Ready to receive control commands.`**

### B6. Terminal 2 — start the Touch driver

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch omni_common omni_state.launch.py
```

You should see `Found Touch.` and `Publishing omni state on: phantom/state`.

### B7. Terminal 3 — start the Touch control

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ls_thesis ur5e_touch_teleop.py --ros-args -p max_linear_speed:=0.05 -p translation_scale:=0.5
```

Wait for **`Touch teleop ready`**.

### B8. Drive the robot

1. Take the stylus out of the inkwell.
2. **Hold the WHITE button** → the robot follows your hand.
3. **Let go of WHITE** → the robot stops. Move your hand back to the middle and hold WHITE again.
4. **GREY button** → start or stop recording the path. Recordings are saved in `~/ros2_ws/trajectories`.

The directions assume you stand **in front of the robot**. Pushing the pen away from you moves the robot away from you.

---

## Part C — Turning it off

1. Let go of WHITE.
2. Terminal 3: `Ctrl+C`
3. Pendant: press **■ Stop**
4. Terminal 2: `Ctrl+C`
5. Terminal 1: `Ctrl+C`
6. Put the stylus back in the inkwell.
7. Pendant: tap the green status in the bottom-left corner → **OFF**, then shut down if you're finished for the day.

---

## Something went wrong?

| What you see | What to do |
|---|---|
| Pendant: *"The connection could not be established"* | Terminal 1 isn't ready. Wait for `System successfully started!` before pressing Play. |
| Terminal 1: `Could not get configuration package` | `Ctrl+C`, wait 5 seconds, start Terminal 1 again. |
| Terminal 3: `switch_controller not available` | Terminal 1 crashed or isn't running. Restart it (B4), press Play (B5), then restart Terminal 3. |
| Terminal 3: `Cannot engage: no fresh Touch state` | Terminal 2 (Touch driver) isn't running. Start it (B6). |
| Terminal 2: `stty: /dev/ttyACM300` / `Failed to initialize haptic device` | Run the Touch setup again (A6). |
| Robot moves the wrong way | Let go of WHITE. Check you're standing in front of the robot. If it's still wrong, the `axis_map` setting needs changing. |
| `ros2: command not found` | You forgot to source. Run the two `source` lines. |

---

## Practise without the robot (simulation)

Same as Part B, but **no robot needed, nothing on the pendant**. The arm only moves in RViz.

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ls_thesis ur5e_teleop.launch.py ur_type:=ur16e use_mock_hardware:=true input:=touch

# Terminal 2 (the fake arm starts in a bad pose, so move it home first)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ls_thesis trajectory_player.py home
```

`input:=touch` starts the Touch driver and the Touch control for you.

---

## All commands and their options

Run the two `source` lines in every new terminal before any `ros2` command.

### Sourcing

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Main launch (robot driver + MoveIt Servo + recorder + RViz)

```bash
ros2 launch ls_thesis ur5e_teleop.launch.py [options]
```

| Option | Default | What it does |
|---|---|---|
| `ur_type:=` | `ur5e` | Robot model. **Use `ur16e`.** Also: `ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur20`, `ur30` |
| `robot_ip:=` | `192.168.0.100` | The robot's IP address |
| `use_mock_hardware:=` | `false` | `true` = simulated robot, no real robot needed |
| `input:=` | `keyboard` | `touch` = also start the Touch driver + Touch control (default speeds). `keyboard` or `none` = start neither |
| `launch_rviz:=` | `true` | `false` = don't open RViz |
| `kinematics_params_file:=` | UR16e factory values | This robot's own calibration file (see "Robot calibration" below) |
| `trajectory_dir:=` | `~/ros2_ws/trajectories` | Folder where recordings are saved |

Show all options: `ros2 launch ls_thesis ur5e_teleop.launch.py --show-args`

### Touch driver

```bash
ros2 launch omni_common omni_state.launch.py
```

No options. It publishes the stylus on `/phantom/state` and the buttons on `/phantom/button`.

### Touch control

```bash
ros2 run ls_thesis ur5e_touch_teleop.py --ros-args -p option:=value -p option:=value
```

| Option | Default | What it does |
|---|---|---|
| `max_linear_speed` | `0.15` | Top speed of the robot tool (metres/second). **Start with `0.05`** |
| `translation_scale` | `1.0` | Robot movement per stylus movement. `0.5` = robot moves half as far as your hand |
| `enable_rotation` | `false` | `true` = twisting the pen also rotates the robot tool |
| `rotation_scale` | `1.0` | Robot rotation per stylus rotation |
| `max_angular_speed` | `0.6` | Top rotation speed (radians/second) |
| `axis_map` | `['y','-x','z']` | Which stylus direction drives robot X, Y, Z. Change it if directions are wrong |
| `position_gain` | `4.0` | How hard it pulls toward the target position. Higher = snappier |
| `rotation_gain` | `3.0` | Same, for rotation |
| `deadband` | `0.0003` | Ignore stylus movements smaller than this (metres) |
| `base_frame` | `base_link` | Robot frame that motion is measured in |
| `ee_frame` | `tool0` | Robot tool frame |
| `state_topic` | `/phantom/state` | Where the stylus position comes from |
| `button_topic` | `/phantom/button` | Where the button presses come from |
| `touch_units_to_m` | `0.001` | Stylus units to metres (driver gives millimetres) |
| `touch_timeout` | `0.1` | Stop if no stylus data for this long (seconds) |
| `publish_rate` | `100.0` | Commands per second |
| `setup_servo` | `true` | Switch the robot into Servo mode on start |

Example with several options:

```bash
ros2 run ls_thesis ur5e_touch_teleop.py --ros-args -p max_linear_speed:=0.05 -p translation_scale:=0.5 -p enable_rotation:=true
```

Example changing `axis_map`:

```bash
ros2 run ls_thesis ur5e_touch_teleop.py --ros-args -p "axis_map:=['x','y','z']"
```

### Keyboard control (instead of the Touch)

```bash
ros2 run ls_thesis ur5e_keyboard_teleop.py --ros-args -p option:=value
```

| Option | Default | What it does |
|---|---|---|
| `linear_speed` | `0.05` | Move speed (metres/second) |
| `angular_speed` | `0.3` | Rotation speed (radians/second) |
| `hold_time` | `0.15` | How long a key press keeps moving (seconds) |
| `base_frame` | `base_link` | Robot base frame |
| `tool_frame` | `tool0` | Robot tool frame |
| `publish_rate` | `50.0` | Commands per second |
| `setup_servo` | `true` | Switch the robot into Servo mode on start |

Keys: `w/s` `a/d` `r/f` = move X/Y/Z · `u/o` `i/k` `j/l` = rotate · `SPACE` = stop · `+/-` = faster/slower · `t` = switch between base and tool frame · `[` = start recording · `]` = stop and save recording · `h` = help · `Ctrl+C` = quit

### Replay a recording / go home

⚠️ **This moves the real robot on its own.** Keep the area clear and a hand on the emergency stop.

```bash
ros2 run ls_thesis trajectory_player.py <file> [options]
```

| Option | Default | What it does |
|---|---|---|
| `<file>` | — | A recording `.yaml` file, `latest` (newest recording), or `home` (ready pose) |
| `--speed` | `1.0` | Playback speed, between 0 and 2. `0.5` = half speed |
| `--approach-time` | `5.0` | Seconds to move to the start of the path |
| `--yes` | off | Don't ask "are you sure?" first |
| `--no-restore-teleop` | off | Don't switch back to Touch/keyboard control afterwards |

Examples:

```bash
ros2 run ls_thesis trajectory_player.py latest --speed 0.5
ros2 run ls_thesis trajectory_player.py ~/ros2_ws/trajectories/trajectory_XXXX.yaml
ros2 run ls_thesis trajectory_player.py home
```

### Recorder settings (starts automatically with the main launch)

| Option | Default | What it does |
|---|---|---|
| `output_dir` | `~/ros2_ws/trajectories` | Where recordings go (set it with `trajectory_dir:=` on the main launch) |
| `sample_rate` | `25.0` | Points saved per second |
| `base_frame` | `base_link` | Robot base frame |
| `ee_frame` | `tool0` | Robot tool frame |

### Robot calibration (optional, makes positions more accurate)

```bash
ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.0.100 target_filename:=$HOME/ur16e_calibration.yaml
```

Then add `kinematics_params_file:=$HOME/ur16e_calibration.yaml` to the main launch.

### Network

```bash
ping -c 3 192.168.0.100          # is the robot reachable?
nmcli con up ur16e               # turn the robot network connection on
nmcli con down ur16e             # turn it off
ip -brief addr                   # show this computer's IP addresses
```

### Touch setup

```bash
~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/bin/Touch_HeadlessSetup   # set up the stylus
~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/bin/TouchCheckup          # test the stylus (window app)
~/Downloads/TouchDriver_2025_12_10+1/TouchDriver_2025_12_10/ListCOMPortHapticDevices  # which port is the stylus on?
```

### Useful checks

```bash
ros2 node list                          # what's running
ros2 topic echo /phantom/state --once   # is the stylus sending data?
ros2 control list_controllers           # robot controllers (needs Terminal 1 running)
```
