# Lift test for the Touch: see README.md in this folder.
"""Lift test through the real omni_state driver: ramps force.z (up in /phantom/state axes)
to the given N, holds 1.2 s, and reports how far the pen rose."""
import sys
import time

import rclpy
from omni_msgs.msg import OmniFeedback, OmniState
peak = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
rclpy.init()
n = rclpy.create_node("ros_lift")
pub = n.create_publisher(OmniFeedback, "/phantom/force_feedback", 10)
z = []
n.create_subscription(OmniState, "/phantom/state", lambda m: z.append(m.pose.position.z), 10)


def spin(dt):
    end = time.monotonic() + dt
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.001)


t0 = time.monotonic()
while (pub.get_subscription_count() == 0 or not z) and time.monotonic() - t0 < 20:
    spin(0.1)
if not z:
    print("RESULT: no /phantom/state - driver not running?")
    raise SystemExit
spin(1.0)
start = z[-1]
top = start
t0 = time.monotonic()
while (t := time.monotonic() - t0) < 1.5:
    m = OmniFeedback()
    m.force.z = peak * min(t / 0.3, 1.0)
    pub.publish(m)
    spin(0.002)
    top = max(top, z[-1])
for i in range(30, -1, -1):
    m = OmniFeedback()
    m.force.z = peak * i / 30
    pub.publish(m)
    spin(0.01)
for _ in range(20):
    pub.publish(OmniFeedback())
    spin(0.005)
print(f"RESULT: ROS driver, {peak} N up: pen z {start:.1f} -> highest {top:.1f} mm, rose {top - start:.1f} mm")
