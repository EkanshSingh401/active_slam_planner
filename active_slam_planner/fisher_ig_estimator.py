#!/usr/bin/env python3
"""Node 1 of 3: candidate viewpoint generation + information-gain scoring.

INTERFACES ARE REAL. THE SCORING IS NOT. See the PLACEHOLDER banner below.

Subscribes:
    /openvins/joint_covariance   (active_slam_msgs/JointCovariance)
    <esdf_topic>                 (sensor_msgs/PointCloud2)  nvblox ESDF slice
    /odomimu                     (nav_msgs/Odometry)        current pose

Publishes:
    ~/scored_viewpoints          (active_slam_msgs/ScoredViewpointArray)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from active_slam_msgs.msg import JointCovariance, ScoredViewpoint, ScoredViewpointArray


class FisherIGEstimator(Node):
    def __init__(self):
        super().__init__("fisher_ig_estimator")

        self.declare_parameter("esdf_topic", "/nvblox_node/static_esdf_pointcloud")
        self.declare_parameter("odom_topic", "/odomimu")
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("num_candidates", 24)
        self.declare_parameter("candidate_radius", 2.0)
        self.declare_parameter("candidate_z_offsets", [0.0, 0.5])

        esdf_topic = self.get_parameter("esdf_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        rate = float(self.get_parameter("rate").value)

        self._cov_msg = None
        self._cov = None
        self._pose = None
        self._esdf_seen = 0

        self.create_subscription(JointCovariance, "/openvins/joint_covariance", self._cb_cov, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self.create_subscription(PointCloud2, esdf_topic, self._cb_esdf, qos_profile_sensor_data)

        self._pub = self.create_publisher(ScoredViewpointArray, "~/scored_viewpoints", 10)
        self.create_timer(1.0 / max(rate, 1e-3), self._tick)

        self.get_logger().info(f"fisher_ig_estimator up: esdf={esdf_topic} odom={odom_topic} rate={rate}Hz")
        self.get_logger().warn("SCORING IS A PLACEHOLDER - scores carry no information-theoretic meaning")

    # ------------------------------------------------------------------ subs
    def _cb_cov(self, msg: JointCovariance):
        """Unpack the joint covariance. This part is REAL and worth keeping."""
        self._cov_msg = msg
        try:
            self._cov = np.asarray(msg.covariance, dtype=float).reshape(msg.dim, msg.dim)
        except ValueError:
            self.get_logger().error(f"covariance length {len(msg.covariance)} != dim^2 ({msg.dim}^2)")
            self._cov = None

    def _cb_odom(self, msg: Odometry):
        self._pose = msg.pose.pose

    def _cb_esdf(self, msg: PointCloud2):
        self._esdf_seen += 1

    # ------------------------------------------------------- real helpers
    def block(self, name, feature_id=None):
        """Slice a named block out of the latest joint covariance.

        REAL utility - this is the intended way to consume the message, and it
        re-reads offsets every time because OpenVINS re-indexes its covariance
        whenever a clone or feature is marginalized.
        """
        if self._cov is None or self._cov_msg is None:
            return None, None
        for b in self._cov_msg.blocks:
            if b.type != name:
                continue
            if feature_id is not None and b.feature_id != feature_id:
                continue
            return self._cov[b.index:b.index + b.size, b.index:b.index + b.size], b
        return None, None

    def cross_block(self, b1, b2):
        """Cross-covariance between two blocks. REAL utility."""
        if self._cov is None:
            return None
        return self._cov[b1.index:b1.index + b1.size, b2.index:b2.index + b2.size]

    # ------------------------------------------------------------------ tick
    def _tick(self):
        if self._pose is None:
            return

        cands = self._generate_candidates(self._pose)

        msg = ScoredViewpointArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "global"
        msg.reference_pose = self._pose
        msg.viewpoints = [self._score(p) for p in cands]
        msg.scoring_method = "PLACEHOLDER_constant"
        msg.is_placeholder = True
        msg.covariance_trace = float(np.trace(self._cov)) if self._cov is not None else -1.0
        self._pub.publish(msg)

    def _generate_candidates(self, pose):
        """Candidates on a ring (optionally several heights) around current pose.

        PLACEHOLDER SAMPLING STRATEGY. A real implementation would sample the
        reachable free space from the ESDF and respect the sensor frustum,
        rather than assuming a ring of fixed radius is meaningful.
        """
        n = int(self.get_parameter("num_candidates").value)
        r = float(self.get_parameter("candidate_radius").value)
        zs = list(self.get_parameter("candidate_z_offsets").value)
        out = []
        for dz in zs:
            for i in range(n):
                th = 2.0 * math.pi * i / max(n, 1)
                p = Pose()
                p.position.x = pose.position.x + r * math.cos(th)
                p.position.y = pose.position.y + r * math.sin(th)
                p.position.z = pose.position.z + float(dz)
                # face back toward the reference pose
                yaw = math.atan2(pose.position.y - p.position.y, pose.position.x - p.position.x)
                p.orientation.z = math.sin(0.5 * yaw)
                p.orientation.w = math.cos(0.5 * yaw)
                out.append(p)
        return out

    # =====================================================================
    # ############  PLACEHOLDER - NOT A REAL INFORMATION GAIN  ############
    #
    # This returns a trivial constant-ish score. It is NOT a Fisher
    # information gain and must not be interpreted as one.
    #
    # A real implementation needs to decide, at minimum:
    #   * What the information measure is over -- pose only, map only, or the
    #     joint pose+map block. The joint block including IMU<->feature
    #     cross-covariance is available via block()/cross_block() above; that
    #     availability is the entire reason Part 1 exists.
    #   * How to predict the measurement Jacobian H for a candidate viewpoint
    #     without actually going there (which features would be visible, at
    #     what image coordinates, under what visibility/occlusion model from
    #     the ESDF).
    #   * Which scalarization of the resulting information matrix to optimize
    #     (A-opt trace, D-opt log-det, E-opt min-eigenvalue). These give
    #     genuinely different exploration behaviour and the choice is open.
    #   * !! FRAME !! SLAM feature blocks arrive in ANCHORED_MSCKF_INVERSE_DEPTH,
    #     NOT global XYZ. Any metric reasoning about map uncertainty must first
    #     propagate them through the anchored->XYZ Jacobian, which depends on
    #     the anchor clone pose. The anchor clone's block and its cross-terms
    #     are in the same message (see StateBlock.anchor_clone_timestamp), so
    #     it is doable from one message -- but it is NOT done anywhere yet.
    #
    # DO NOT build downstream logic that assumes these numbers mean anything.
    # =====================================================================
    def _score(self, pose):
        vp = ScoredViewpoint()
        vp.pose = pose
        vp.information_gain = 1.0   # PLACEHOLDER
        vp.coverage_gain = 0.0      # PLACEHOLDER
        if self._pose is not None:
            d = math.dist(
                (pose.position.x, pose.position.y, pose.position.z),
                (self._pose.position.x, self._pose.position.y, self._pose.position.z),
            )
        else:
            d = 0.0
        vp.travel_cost = d
        vp.is_reachable = True      # PLACEHOLDER - real version queries the ESDF
        vp.clearance = -1.0         # PLACEHOLDER - real version reads ESDF distance
        vp.score = vp.information_gain
        return vp


def main():
    rclpy.init()
    node = FisherIGEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
