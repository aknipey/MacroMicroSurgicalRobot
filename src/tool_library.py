#!/usr/bin/env python3
"""Manage the numbered tool library (cfg.TOOL_LIBRARY) and switch the teleop's tool.

    tool_library.py list
    tool_library.py add 2 "Needle driver" --from ~/ros2_ws/src/ndi_tracking/calibrations/X.json --length 180
    tool_library.py add 3 "Probe" --tip X Y Z --direction X Y Z --length 120
    tool_library.py remove 3
    tool_library.py default 2
    tool_library.py console      # type a number + Enter to switch (launch_system.sh runs this)

`add` takes the tip and shaft direction (tool0 frame) from an ndi_tracking robot-tcp
JSON (tip_tool0_mm / direction_tool0) or as numbers; the shaft length (tip back to
the holder end, mm) is measured. Re-adding an existing number replaces it (--force).
The console sets the teleop's `tool` parameter, which refuses the switch in RCM mode,
while WHITE is held or while the robot moves, and prints why.
"""
import argparse
import json
import os
import sys
import threading
from datetime import date

from macro_micro_robot import teleop_settings as cfg
from macro_micro_robot import tools as tool_library

TELEOP_NODE = "/ur16e_touch_teleop"


def cmd_list(args):
    default, tools = tool_library.load_library(args.library)
    if not tools:
        print(f"No tools in {tool_library.library_path(args.library)}")
        return
    for number in sorted(tools):
        mark = "*" if number == default else " "
        print(f"{mark} {tool_library.describe(number, tools[number])}")
    print("(* = default at startup)")


def cmd_add(args):
    default, tools = tool_library.load_library(args.library)
    if args.number in tools and not args.force:
        sys.exit(f"Tool {args.number} exists ({tools[args.number]['name']}) - use --force to replace it")
    if args.calibration:
        tip, direction, calibrated = tool_library.from_calibration(args.calibration)
        source = os.path.abspath(os.path.expanduser(args.calibration))
    elif args.tip and args.direction:
        tip, direction, calibrated, source = args.tip, args.direction, date.today().isoformat(), "manual"
    else:
        sys.exit("Give --from <robot-tcp JSON>, or --tip and --direction")
    entry = {"name": args.name,
             "tip_tool0_mm": [round(float(v), 4) for v in tip],
             "direction_tool0": [round(float(v), 6) for v in direction],
             "shaft_length_mm": float(args.length),
             "calibration": source,
             "date": calibrated,
             "notes": args.notes}
    tool_library.check_entry(args.number, entry)
    tools[args.number] = entry
    tool_library.save_library(args.library, default if default is not None else args.number, tools)
    print(f"Saved {tool_library.describe(args.number, entry)}")


def cmd_remove(args):
    default, tools = tool_library.load_library(args.library)
    if args.number not in tools:
        sys.exit(f"No tool {args.number}")
    removed = tools.pop(args.number)
    if default == args.number:
        default = min(tools) if tools else None
        print(f"Default is now {default}")
    tool_library.save_library(args.library, default, tools)
    print(f"Removed {args.number}: {removed['name']}")


def cmd_default(args):
    _, tools = tool_library.load_library(args.library)
    if args.number not in tools:
        sys.exit(f"No tool {args.number}")
    tool_library.save_library(args.library, args.number, tools)
    print(f"Default tool is now {args.number}: {tools[args.number]['name']}")


def cmd_console(args):
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("tool_console")
    client = node.create_client(SetParameters, f"{TELEOP_NODE}/set_parameters")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_active(msg):
        tool = json.loads(msg.data)
        print(f"[tools] active tool: {tool['number']} - {tool['name']}", flush=True)

    node.create_subscription(String, f"{TELEOP_NODE}/active_tool", on_active, latched)

    def switch(number):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[tools] {TELEOP_NODE} is not running", flush=True)
            return
        value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=number)
        request = SetParameters.Request(parameters=[Parameter(name="tool", value=value)])

        def done(future):
            result = future.result().results[0]
            if not result.successful:
                print(f"[tools] switch to {number} REFUSED: {result.reason}", flush=True)

        client.call_async(request).add_done_callback(done)

    def read_input():
        print("[tools] type a tool number + Enter to switch, l + Enter to list", flush=True)
        for line in sys.stdin:
            line = line.strip()
            if line.isdigit():
                switch(int(line))
            elif line in ("l", "list"):
                try:
                    cmd_list(args)
                except ValueError as exc:
                    print(f"[tools] {exc}")
            elif line:
                print("[tools] type a tool number, or l to list", flush=True)
        # stdin closed (not a terminal): keep running so the launcher does not stop

    threading.Thread(target=read_input, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", default=cfg.TOOL_LIBRARY)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("list").set_defaults(func=cmd_list)
    p = sub.add_parser("add")
    p.add_argument("number", type=int)
    p.add_argument("name")
    p.add_argument("--from", dest="calibration", help="ndi_tracking robot-tcp JSON")
    p.add_argument("--tip", type=float, nargs=3, metavar=("X", "Y", "Z"), help="tip in tool0, mm")
    p.add_argument("--direction", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="shaft direction holder -> tip, tool0 frame")
    p.add_argument("--length", type=float, required=True, help="shaft length, tip back to holder end, mm")
    p.add_argument("--notes", default="")
    p.add_argument("--force", action="store_true", help="replace an existing number")
    p.set_defaults(func=cmd_add)
    for name, func in (("remove", cmd_remove), ("default", cmd_default)):
        p = sub.add_parser(name)
        p.add_argument("number", type=int)
        p.set_defaults(func=func)
    sub.add_parser("console").set_defaults(func=cmd_console)
    args = ap.parse_args()
    try:
        args.func(args)
    except ValueError as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    main()
