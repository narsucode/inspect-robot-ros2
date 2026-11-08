#!/usr/bin/env python3
"""Judge whether /scan matches the saved map, without eyeballing RViz.

It subscribes to /scan and TF, publishes an initial pose to /initialpose,
then measures how far each scan endpoint is from the nearest occupied map cell.
It scores the scan twice: as-is, and left-right mirrored.  A clearly better
mirrored score means the firmware index mapping is flipped.

Usage (on the VM, after bringup + nav2 are running):
  source /opt/ros/humble/setup.bash
  source ~/narsu/ros2/patrol_ws/install/setup.bash
  python3 ~/narsu/ros2/tools/scan_map_check.py --yaw 3.129 --x 0 --y 0
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = np.array(Image.open(img_path).convert("L"))
    occ_prob = (255.0 - img.astype(np.float32)) / 255.0
    occupied = occ_prob > float(meta.get("occupied_thresh", 0.65))
    free = occ_prob < float(meta.get("free_thresh", 0.25))
    res = float(meta["resolution"])
    ox, oy, _ = meta["origin"]
    # distance (metres) from any cell to the nearest occupied cell
    dist = distance_transform_edt(~occupied).astype(np.float32) * res
    return dist, occupied, free, res, float(ox), float(oy)


def global_search(pts, dist, free, res, ox, oy, mirror=False, stride=4, yaw_step_deg=3.0):
    """Brute-force where in the map this scan fits best (coarse-to-fine)."""
    rows, cols = dist.shape
    p = pts.copy()
    if mirror:
        p[:, 1] = -p[:, 1]
    if len(p) > 64:
        p = p[:: max(1, len(p) // 64)]

    ys = np.deg2rad(np.arange(0.0, 360.0, yaw_step_deg))
    cos, sin = np.cos(ys), np.sin(ys)
    # (Y, N, 2)
    rot = np.stack(
        [
            p[None, :, 0] * cos[:, None] - p[None, :, 1] * sin[:, None],
            p[None, :, 0] * sin[:, None] + p[None, :, 1] * cos[:, None],
        ],
        axis=-1,
    )

    rr, cc = np.nonzero(free[::stride, ::stride])
    rr, cc = rr * stride, cc * stride
    best = (1e9, 0.0, 0.0, 0.0)
    for r0, c0 in zip(rr, cc):
        x0 = ox + c0 * res
        y0 = oy + (rows - 1 - r0) * res
        mx = rot[..., 0] + x0
        my = rot[..., 1] + y0
        ci = ((mx - ox) / res).astype(np.int32)
        ri = (rows - 1 - (my - oy) / res).astype(np.int32)
        np.clip(ci, 0, cols - 1, out=ci)
        np.clip(ri, 0, rows - 1, out=ri)
        cost = dist[ri, ci].mean(axis=1)  # per yaw
        k = int(np.argmin(cost))
        if cost[k] < best[0]:
            best = (float(cost[k]), float(ys[k]), x0, y0)

    # local refinement
    _, yaw0, x0, y0 = best
    for dx in np.arange(-0.15, 0.151, 0.05):
        for dy in np.arange(-0.15, 0.151, 0.05):
            for dyaw in np.arange(-0.05, 0.051, 0.01):
                yaw = yaw0 + dyaw
                R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                q = p @ R.T + np.array([x0 + dx, y0 + dy])
                ci = ((q[:, 0] - ox) / res).astype(int)
                ri = (rows - 1 - (q[:, 1] - oy) / res).astype(int)
                if (ci < 0).any() or (ci >= cols).any() or (ri < 0).any() or (ri >= rows).any():
                    continue
                cost = float(dist[ri, ci].mean())
                if cost < best[0]:
                    best = (cost, float(yaw), float(x0 + dx), float(y0 + dy))
    return best


class ScanMapChecker(Node):
    def __init__(self, scan_topic):
        super().__init__("scan_map_check")
        self.scan = None
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("waiting for %s ..." % scan_topic)

    def on_scan(self, msg):
        self.scan = msg

    def send_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.2)).to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        # DDS discovery needs a moment before the first publish actually lands.
        for _ in range(30):
            if self.pose_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(3):
            self.pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)
        self.get_logger().info("initial pose sent: x=%.2f y=%.2f yaw=%.3f" % (x, y, yaw))

    def wait_scan(self, timeout):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None:
                return True
        return False

    def lookup(self, stamp):
        # The ESP32 stamps scans with its own clock, which can sit a little
        # ahead of the TF buffer.  Try the scan stamp, then fall back to the
        # latest transform (the robot barely moves during the check).
        for target in (Time.from_msg(stamp), Time()):
            for _ in range(30):
                try:
                    return self.tf_buffer.lookup_transform(
                        "map", self.scan.header.frame_id, target, timeout=rclpy.duration.Duration(seconds=0.2)
                    )
                except (LookupException, ConnectivityException, ExtrapolationException):
                    rclpy.spin_once(self, timeout_sec=0.1)
        return None


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def to_map(points, tf):
    """points: Nx2 in laser frame -> Nx2 in map frame"""
    R = quat_to_rot(tf.transform.rotation)
    t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
    p3 = np.hstack([points, np.zeros((points.shape[0], 1))])
    return (p3 @ R.T + t)[:, :2]


def score(points_map, dist, res, ox, oy):
    rows, cols = dist.shape
    c = ((points_map[:, 0] - ox) / res).astype(int)
    r = (rows - 1 - (points_map[:, 1] - oy) / res).astype(int)
    ok = (c >= 0) & (c < cols) & (r >= 0) & (r < rows)
    if ok.sum() == 0:
        return None
    d = dist[r[ok], c[ok]]
    return {
        "n": int(ok.sum()),
        "median": float(np.median(d)),
        "p90": float(np.percentile(d, 90)),
        "within_10cm": float((d <= 0.10).mean()),
        "within_20cm": float((d <= 0.20).mean()),
    }


def extract(scan):
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    good = np.isfinite(ranges) & (ranges >= max(scan.range_min, 0.05)) & (ranges <= scan.range_max - 1e-3)
    r, a = ranges[good], angles[good]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="/home/ros/narsu/ros2/patrol_ws/src/youdaobot_navigation/maps/room.yaml")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=3.129)
    ap.add_argument("--wait", type=float, default=15.0)
    ap.add_argument("--no-initial-pose", action="store_true")
    ap.add_argument("--search", action="store_true", help="brute-force search the map for the scan's pose")
    args = ap.parse_args()

    dist, occupied, free, res, ox, oy = load_map(args.map)
    print("map: %dx%d cells, res=%.3f m, origin=(%.2f, %.2f)" % (dist.shape[1], dist.shape[0], res, ox, oy))

    rclpy.init()
    node = ScanMapChecker(args.scan_topic)
    try:
        if not node.wait_scan(args.wait):
            print("ERROR: no %s data within %.0fs" % (args.scan_topic, args.wait))
            return 2
        scan = node.scan
        pts = extract(scan)
        print("scan: %d points, frame=%s, angle=[%.2f, %.2f]" % (len(pts), scan.header.frame_id, scan.angle_min, scan.angle_max))

        if args.search:
            best = {}
            for name, mir in (("as-is", False), ("mirrored", True)):
                s = global_search(pts, dist, free, res, ox, oy, mirror=mir)
                best[name] = s
                print("%s best: cost=%.3fm pose=(%.2f, %.2f, %.3f rad)" % (name, s[0], s[2], s[3], s[1]))
            normal, mirrored = best["as-is"], best["mirrored"]
            use = normal if normal[0] <= mirrored[0] else mirrored
            print("VERDICT: %s fits better by %.3fm" % (
                "as-is" if normal[0] <= mirrored[0] else "mirrored", abs(normal[0] - mirrored[0])))
            args.x, args.y, args.yaw = use[2], use[3], use[1]
            print("using pose (%.2f, %.2f, %.3f) as initial pose" % (args.x, args.y, args.yaw))

        if not args.no_initial_pose:
            node.send_initial_pose(args.x, args.y, args.yaw)
            end = time.time() + 2.0
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            # let the scan refresh so it matches the pose we just set
            old = scan.header.stamp
            end = time.time() + 4.0
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.scan is not None and node.scan.header.stamp != old:
                    scan = node.scan
                    pts = extract(scan)
                    break
        tf = node.lookup(scan.header.stamp)
        if tf is None:
            print("ERROR: no map -> %s TF" % scan.header.frame_id)
            return 3

        normal = score(to_map(pts, tf), dist, res, ox, oy)
        mirrored_pts = np.stack([pts[:, 0], -pts[:, 1]], axis=1)
        mirrored = score(to_map(mirrored_pts, tf), dist, res, ox, oy)

        def show(name, s):
            if s is None:
                print("%s: no points inside map" % name)
                return
            print(
                "%s: median=%.3fm p90=%.3fm <=10cm=%.0f%% <=20cm=%.0f%% (n=%d)"
                % (name, s["median"], s["p90"], 100 * s["within_10cm"], 100 * s["within_20cm"], s["n"])
            )

        show("as-is   ", normal)
        show("mirrored", mirrored)

        if normal and mirrored:
            if normal["median"] + 0.05 < mirrored["median"]:
                print("VERDICT: as-is fits the map better -> firmware indexing is probably correct")
            elif mirrored["median"] + 0.05 < normal["median"]:
                print("VERDICT: mirrored fits better -> left/right swap in firmware (line 517)")
            else:
                print("VERDICT: inconclusive - check initial pose / localization first")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
