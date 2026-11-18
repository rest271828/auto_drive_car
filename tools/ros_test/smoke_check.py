#!/usr/bin/env python3
"""ROS 2 闭环冒烟测试: 订阅 /planning/local_trajectory 并监听 tf, 统计
规划频率、主车行驶距离、与障碍物的碰撞和最小间距。

用法(先启动 planning_launch.py):
    python3 smoke_check.py --config planning_static_obs_config.yaml --duration 40 --min-travel 150
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from base_msgs.msg import LocalTrajectory
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def corners(x, y, th, length, width):
    out = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        lx, ly = sx * length / 2, sy * width / 2
        out.append((x + lx * math.cos(th) - ly * math.sin(th), y + lx * math.sin(th) + ly * math.cos(th)))
    return out


def overlap(c1, c2):
    """分离轴定理: 两个凸四边形是否重叠"""
    for poly in (c1, c2):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            ax, ay = -ey, ex
            p1 = [ax * p[0] + ay * p[1] for p in c1]
            p2 = [ax * p[0] + ay * p[1] for p in c2]
            if max(p1) < min(p2) or max(p2) < min(p1):
                return False
    return True


def seg_dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy)))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def poly_dist(c1, c2):
    return min(min(seg_dist(c1[i], c2[j], c2[(j + 1) % 4]), seg_dist(c2[i], c1[j], c1[(j + 1) % 4]))
               for i in range(4) for j in range(4))


class SmokeChecker(Node):
    def __init__(self, cfg):
        super().__init__("smoke_checker")
        veh = cfg["vehicle"]
        self.map_frame = cfg["pnc_map"]["frame"]
        self.car = veh["main_car"]
        self.obses = [veh[f"obs_car{i}"] for i in (1, 2, 3)]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(LocalTrajectory, "/planning/local_trajectory", self.traj_cb, 10)
        self.create_timer(0.1, self.tick)
        self.traj_times = []
        self.traj_empty = 0
        self.traj_speeds = []
        self.samples = []  # (t, car(x,y,yaw), [obs(x,y,yaw) or None])
        self.tf_fail = 0

    def traj_cb(self, msg):
        self.traj_times.append(time.monotonic())
        if not msg.local_trajectory:
            self.traj_empty += 1
            return
        self.traj_speeds.append(msg.local_trajectory[0].speed_point.speed)

    def lookup(self, frame):
        try:
            ts = self.buffer.lookup_transform(self.map_frame, frame, Time())
        except Exception:  # noqa: BLE001  (tf2 的各种异常)
            return None
        tr = ts.transform
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def tick(self):
        car = self.lookup(self.car["frame"])
        if car is None:
            self.tf_fail += 1
            return
        obs = [self.lookup(o["frame"]) for o in self.obses]
        self.samples.append((time.monotonic(), car, obs))

    def report(self, min_travel):
        lines = []
        ok = True

        # 规划频率
        n = len(self.traj_times)
        rate = (n - 1) / (self.traj_times[-1] - self.traj_times[0]) if n >= 2 else 0.0
        lines.append(f"trajectory msgs: {n}, rate: {rate:.1f} Hz, empty: {self.traj_empty}")
        if rate < 8.0:
            ok = False
            lines.append("  FAIL: planning rate < 8 Hz (or no trajectory at all)")

        # 行驶情况
        if len(self.samples) < 2:
            lines.append(f"FAIL: no tf map->{self.car['frame']} samples (tf lookup failed {self.tf_fail} times)")
            return False, lines
        travelled = 0.0
        max_v = 0.0
        stall = 0.0
        longest_stall = 0.0
        for (t0, c0, _), (t1, c1, _) in zip(self.samples, self.samples[1:]):
            d = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
            travelled += d
            if t1 > t0:
                max_v = max(max_v, d / (t1 - t0))
            stall = stall + (t1 - t0) if d < 0.01 else 0.0
            longest_stall = max(longest_stall, stall)
        first, last = self.samples[0][1], self.samples[-1][1]
        lines.append(f"car start: ({first[0]:.1f}, {first[1]:.1f})  end: ({last[0]:.1f}, {last[1]:.1f}, yaw {last[2]:.2f})")
        lines.append(f"travelled: {travelled:.1f} m, max tf speed: {max_v:.1f} m/s, longest stop: {longest_stall:.1f} s")
        if self.traj_speeds:
            lines.append(f"planned speed at t=0: min {min(self.traj_speeds):.2f}, max {max(self.traj_speeds):.2f} m/s")
        if travelled < min_travel:
            ok = False
            lines.append(f"  FAIL: travelled < {min_travel} m")

        # 碰撞与最小间距
        collisions = 0
        min_clear = float("inf")
        for t, c, obs in self.samples:
            cc = corners(c[0], c[1], c[2], self.car["length"], self.car["width"])
            for o, conf in zip(obs, self.obses):
                if o is None:
                    continue
                oc = corners(o[0], o[1], o[2], conf["length"], conf["width"])
                if overlap(cc, oc):
                    collisions += 1
                    if collisions <= 5:
                        lines.append(f"  COLLISION with obs{conf['id']}: car ({c[0]:.1f},{c[1]:.1f}) obs ({o[0]:.1f},{o[1]:.1f})")
                else:
                    min_clear = min(min_clear, poly_dist(cc, oc))
        lines.append(f"collision samples: {collisions}, min clearance: {min_clear:.2f} m")
        if collisions:
            ok = False
            lines.append("  FAIL: collision")
        return ok, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="planning_static_obs_config.yaml")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--min-travel", type=float, default=100.0)
    args = ap.parse_args()

    cfg_path = os.path.join(get_package_share_directory("planning"), "config", args.config)
    with open(cfg_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = SmokeChecker(cfg)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok, lines = node.report(args.min_travel)
    print(f"===== {args.config}: {'PASS' if ok else 'FAIL'} =====")
    for line in lines:
        print(line)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
