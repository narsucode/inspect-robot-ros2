#!/usr/bin/env python3
"""Send one NavigateToPose goal 0.6 m ahead and report progress."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__("nav_test")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def pose(self):
        for _ in range(30):
            try:
                tr = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                t, r = tr.transform.translation, tr.transform.rotation
                return t.x, t.y, yaw_of(r)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None


def main():
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
    rclpy.init()
    n = T()
    p = n.pose()
    if p is None:
        print("ERROR: no map->base_footprint TF")
        return 1
    x, y, th = p
    print("start pose: x=%.3f y=%.3f yaw=%.1f deg" % (x, y, math.degrees(th)))
    gx, gy = x + dist * math.cos(th), y + dist * math.sin(th)
    print("goal: x=%.3f y=%.3f" % (gx, gy))

    if not n.cli.wait_for_server(timeout_sec=5.0):
        print("ERROR: navigate_to_pose action not available")
        return 2

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = n.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    goal.pose.pose.orientation.z = math.sin(th / 2)
    goal.pose.pose.orientation.w = math.cos(th / 2)

    last = {"d": None, "t": 0.0}

    def on_fb(fb):
        d = fb.feedback.distance_remaining
        now = time.time()
        if last["d"] is None or abs(d - last["d"]) > 0.03 or now - last["t"] > 2.0:
            print("  remaining=%.2fm  nav=%.1fs" % (d, fb.feedback.navigation_time.sec))
            last["d"] = d
            last["t"] = now

    send = n.cli.send_goal_async(goal, feedback_callback=on_fb)
    rclpy.spin_until_future_complete(n, send, timeout_sec=10)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("goal rejected")
        return 3
    print("goal accepted")
    t0 = time.time()
    res = handle.get_result_async()
    while not res.done() and time.time() - t0 < 90:
        rclpy.spin_once(n, timeout_sec=0.2)
        now = n.pose()
        if now and int(time.time() - t0) % 5 == 0:
            pass
    if not res.done():
        print("TIMEOUT after 90s")
        return 4
    print("result status=%d (4=SUCCEEDED), elapsed=%.1fs" % (res.result().status, time.time() - t0))
    now = n.pose()
    if now:
        print("end pose: x=%.3f y=%.3f yaw=%.1f deg" % (now[0], now[1], math.degrees(now[2])))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
