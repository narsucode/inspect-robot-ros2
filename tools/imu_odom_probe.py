#!/usr/bin/env python3
"""Drive straight, compare wheel-odom yaw against IMU gyro integration."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__("imu_odom_probe")
        self.odom = None
        self.imu_yaw = 0.0
        self.last_imu_t = None
        self.imu_gyro_yaw = 0.0
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 20)
        self.create_subscription(Imu, "/imu", self.cb_imu, qos_profile_sensor_data)

    def cb_odom(self, m):
        self.odom = m

    def cb_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_imu_t is not None:
            dt = t - self.last_imu_t
            if 0 < dt < 0.5:
                self.imu_gyro_yaw += m.angular_velocity.z * dt
        self.last_imu_t = t
        self.imu_yaw = yaw_of(m.orientation)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "fwd"
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else (1.0 if mode == "rot" else 0.1)
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else (1.5 if mode == "rot" else 3.0)
    rclpy.init()
    n = P()
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    th0 = yaw_of(n.odom.pose.pose.orientation) if n.odom else 0.0
    gyro0 = n.imu_gyro_yaw
    imu0 = n.imu_yaw
    cmd = Twist()
    if mode == "rot":
        cmd.angular.z = vx
    else:
        cmd.linear.x = vx
    end = time.time() + secs
    while time.time() < end and rclpy.ok():
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.02)
    n.pub.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)

    th1 = yaw_of(n.odom.pose.pose.orientation)
    print("odom dyaw     = %+.1f deg" % math.degrees(th1 - th0))
    print("imu gyro dyaw = %+.1f deg (integrated)" % math.degrees(n.imu_gyro_yaw - gyro0))
    print("imu abs dyaw  = %+.1f deg (Madgwick yaw, may drift)" % math.degrees(n.imu_yaw - imu0))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
