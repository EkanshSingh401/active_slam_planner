#!/usr/bin/env python3
"""Record /openvins/joint_covariance to a pickle for offline eigen analysis.

ANALYSIS TOOL. Read-only on the ROS side -- it subscribes and writes a file,
and touches neither the estimator nor the publisher.

Each sample keeps the raw dim x dim matrix plus the full StateBlock metadata,
so the offline script can slice blocks without ever hardcoding an offset.
That matters: OpenVINS re-indexes its covariance on every marginalization.

Usage:
    python3 record_joint_covariance.py --seconds 60 --out results/run.pkl
"""

import argparse
import pickle
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from active_slam_msgs.msg import JointCovariance


class Recorder(Node):
    def __init__(self, seconds, out, wait_features=0):
        super().__init__("joint_cov_recorder")
        self.seconds = seconds
        self.out = out
        self.wait_features = wait_features
        self.armed = wait_features <= 0
        self.samples = []
        self.pose = None
        self.t0 = None
        self.create_subscription(JointCovariance, "/openvins/joint_covariance", self._cb, 50)
        self.create_subscription(Odometry, "/odomimu", self._odom, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"recording {seconds:.0f}s -> {out}")

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def _cb(self, msg):
        if not self.armed:
            if msg.num_slam_features < self.wait_features:
                return
            self.armed = True
            self.get_logger().info(
                f"ARMED: {msg.num_slam_features} SLAM features in state -- recording now")
        if self.t0 is None:
            self.t0 = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cov = np.asarray(msg.covariance, dtype=float).reshape(msg.dim, msg.dim)
        blocks = [
            dict(
                type=b.type, index=b.index, size=b.size, state_id=b.state_id,
                clone_timestamp=b.clone_timestamp, feature_id=b.feature_id,
                camera_id=b.camera_id, parameterization=b.parameterization,
                is_anchored=b.is_anchored,
                anchor_clone_timestamp=b.anchor_clone_timestamp,
            )
            for b in msg.blocks
        ]
        self.samples.append(dict(
            stamp=stamp, wall=time.time(), dim=msg.dim, cov=cov, blocks=blocks,
            full_state_dim=msg.full_state_dim, num_clones=msg.num_clones,
            num_slam_features=msg.num_slam_features,
            includes_imu=msg.includes_imu, includes_clones=msg.includes_clones,
            includes_slam_features=msg.includes_slam_features,
            includes_calibration=msg.includes_calibration,
            pose=self.pose,
        ))

    def _tick(self):
        if self.t0 is None:
            if self.armed:
                self.get_logger().warn(
                    "no /openvins/joint_covariance yet -- is OpenVINS initialized?")
            else:
                self.get_logger().info(
                    f"waiting for >= {self.wait_features} SLAM features before recording...")
            return
        el = time.time() - self.t0
        if int(el) % 10 == 0 and self.samples:
            s = self.samples[-1]
            self.get_logger().info(
                f"{el:5.1f}s  n={len(self.samples)}  dim={s['dim']}  "
                f"feats={s['num_slam_features']}  clones={s['num_clones']}"
            )
        if el > self.seconds:
            raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="results/joint_cov_run.pkl")
    # Don't start the clock until the filter has actually filled up. Recording
    # from the instant of initialisation would capture the feature build-up
    # rather than the converged filter the diagnostic is about.
    ap.add_argument("--wait-features", type=int, default=0,
                    help="hold off recording until this many SLAM features are in state")
    a = ap.parse_args()

    rclpy.init()
    node = Recorder(a.seconds, a.out, a.wait_features)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass

    s = node.samples
    if not s:
        print("NO SAMPLES -- OpenVINS almost certainly never initialized.")
        return
    feats = [x["num_slam_features"] for x in s]
    dims = [x["dim"] for x in s]
    poses = [x["pose"] for x in s if x["pose"]]
    with open(a.out, "wb") as f:
        pickle.dump(s, f)
    print(f"wrote {len(s)} samples -> {a.out}")
    print(f"  dim   {min(dims)}..{max(dims)}")
    print(f"  feats {min(feats)}..{max(feats)}  (mean {np.mean(feats):.1f})")
    if poses:
        r = [np.linalg.norm(p) for p in poses]
        print(f"  |p|   {min(r):.3f}..{max(r):.3f} m  -- {'BOUNDED' if max(r) < 50 else 'DIVERGED'}")


if __name__ == "__main__":
    main()
