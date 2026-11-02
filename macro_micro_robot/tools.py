"""Numbered library of calibrated robot tools (cfg.TOOL_LIBRARY, a YAML file).

Each tool is a straight shaft fixed to tool0, as RcmTool needs it: the tip and the
shaft direction (holder -> tip) in the tool0 frame, and the shaft length back from
the tip to the holder end. The tip
and direction come from a calibration (ndi_tracking robot-tcp writes both into its
JSON as tip_tool0_mm / direction_tool0); the length is measured.

    default: 1
    tools:
      1:
        name: NDI wand 8700340
        tip_tool0_mm: [x, y, z]
        direction_tool0: [x, y, z]      # holder -> tip, normalised on load
        shaft_length_mm: 158.6
        calibration: <file it came from>
        date: '2026-10-26'
        notes: ''
"""
import json
import os

import numpy as np
import yaml

from macro_micro_robot.ur16e_common import RcmTool

HEADER = """\
# Tool library for the UR16e Touch teleop (macro_micro_robot/tools.py).
# Number -> calibrated tool. tip_tool0_mm and direction_tool0 (holder -> tip) are in
# the tool0 frame; shaft_length_mm runs back from the tip to the holder end (only
# that segment can be the RCM point). Edit by hand or with
#   ros2 run macro_micro_robot tool_library.py add|remove|default ...
# The teleop re-reads this file on every tool switch.
"""


def library_path(path):
    return os.path.expanduser(path)


def load_library(path):
    """(default number or None, {number: entry}) from the YAML file; (None, {}) if missing."""
    path = library_path(path)
    if not os.path.exists(path):
        return None, {}
    with open(path) as f:
        try:
            data = yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise ValueError(f"{path} is not valid YAML: {exc}") from exc
    tools = {}
    for number, entry in (data.get("tools") or {}).items():
        number = int(number)
        check_entry(number, entry)
        tools[number] = entry
    default = data.get("default")
    if default is not None and int(default) not in tools:
        raise ValueError(f"{path}: default tool {default} is not in the library")
    return (int(default) if default is not None else None), tools


def save_library(path, default, tools):
    path = library_path(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"default": default, "tools": {n: tools[n] for n in sorted(tools)}}
    with open(path, "w") as f:
        f.write(HEADER)
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)


def check_entry(number, entry):
    if number < 1:
        raise ValueError(f"tool numbers start at 1, not {number}")
    for key in ("name", "tip_tool0_mm", "direction_tool0", "shaft_length_mm"):
        if key not in entry:
            raise ValueError(f"tool {number} has no {key}")
    rcm_tool(entry)     # raises on a bad shape / zero direction / length


def rcm_tool(entry):
    """RcmTool (metres) for a library entry."""
    tip = np.asarray(entry["tip_tool0_mm"], dtype=float) * 1e-3
    direction = np.asarray(entry["direction_tool0"], dtype=float)
    length = float(entry["shaft_length_mm"]) * 1e-3
    if tip.shape != (3,) or direction.shape != (3,) or not np.linalg.norm(direction) > 1e-9:
        raise ValueError(f"bad tip {entry['tip_tool0_mm']} / direction {entry['direction_tool0']}")
    direction = direction / np.linalg.norm(direction)
    return RcmTool(tip - length * direction, direction, length)


def from_calibration(json_path):
    """(tip_tool0_mm, direction_tool0, date) from an ndi_tracking robot-tcp JSON."""
    with open(os.path.expanduser(json_path)) as f:
        data = json.load(f)
    if "tip_tool0_mm" not in data or "direction_tool0" not in data:
        raise ValueError(f"{json_path} has no tip_tool0_mm / direction_tool0 - not a robot-tcp result")
    return data["tip_tool0_mm"], data["direction_tool0"], str(data.get("date", ""))[:10]


def describe(number, entry):
    tool = rcm_tool(entry)
    tip = 1e3 * tool.tip_offset
    tilt = np.degrees(np.arccos(np.clip(tool.direction[2], -1.0, 1.0)))
    return (f"{number}: {entry['name']}  tip ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) mm, "
            f"shaft {entry['shaft_length_mm']:.1f} mm, {tilt:.2f} deg off tool0 z"
            + (f", calibrated {entry['date']}" if entry.get("date") else ""))


def active_tool_message(number, entry):
    """JSON for the teleop's latched ~/active_tool topic (also logged by the recorder)."""
    return json.dumps({"number": number, "name": entry["name"],
                       "tip_tool0_mm": [float(v) for v in entry["tip_tool0_mm"]],
                       "direction_tool0": [float(v) for v in entry["direction_tool0"]],
                       "shaft_length_mm": float(entry["shaft_length_mm"]),
                       "calibration": entry.get("calibration", ""),
                       "date": entry.get("date", "")})
