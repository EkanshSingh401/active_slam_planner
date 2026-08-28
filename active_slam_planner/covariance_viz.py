#!/usr/bin/env python3
"""Turn /openvins/joint_covariance into things RViz and rqt_plot can draw.

VISUALIZATION ONLY. This node reads the joint covariance and the current pose
and emits markers + scalars. It does not touch the estimator, and nothing
downstream should treat its output as an estimate.

Subscribes:
    /openvins/joint_covariance   (active_slam_msgs/JointCovariance)
    /odomimu                     (nav_msgs/Odometry)   -- where to draw the ellipsoid

Publishes:
    ~/uncertainty_ellipsoid   visualization_msgs/Marker  SPHERE, 1-sigma position
    ~/uncertainty_text        visualization_msgs/Marker  TEXT_VIEW_FACING readout
    ~/pos_cov_trace           std_msgs/Float64  trace of the IMU position block [m^2]
    ~/pos_sigma_max           std_msgs/Float64  largest 1-sigma semi-axis [m]
    ~/state_dim               std_msgs/Float64  dim of the published covariance
    ~/num_slam_features       std_msgs/Float64  SLAM features in the state

The scalars are Float64 (not a custom message) purely so rqt_plot and
PlotJuggler can pick them up with no plugin.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker

from active_slam_msgs.msg import JointCovariance

# Layout inside the OpenVINS 15x15 IMU block:
#   [0:3] JPL quaternion error, [3:6] position, [6:9] velocity,
#   [9:12] gyro bias, [12:15] accel bias
IMU_POS_OFFSET = 3
IMU_BLOCK_SIZE = 15


def rotation_to_quaternion(rot):
    """3x3 rotation matrix -> (x, y, z, w). Shepperd's method, branch on trace."""
    m = rot
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


class CovarianceViz(Node):
    def __init__(self):
        super().__init__("covariance_viz")

        self.declare_parameter("cov_topic", "/openvins/joint_covariance")
        self.declare_parameter("odom_topic", "/odomimu")
        self.declare_parameter("marker_frame", "global")
        # 1-sigma on a healthy filter is millimetres, which is invisible next to
        # a metre-scale mesh. This exaggerates it. It is a DISPLAY GAIN ONLY --
        # the numbers in the text marker and on the Float64 topics are raw.
        self.declare_parameter("sigma_scale", 50.0)
        # Floor so the marker never vanishes entirely, in metres of diameter.
        self.declare_parameter("min_diameter", 0.02)
        self.declare_parameter("alpha", 0.45)
        self.declare_parameter("publish_text", True)

        self._odom = None
        self._warned_no_imu = False
        self._logged_param = False

        cov_topic = self.get_parameter("cov_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.create_subscription(JointCovariance, cov_topic, self._cb_cov, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)

        self._pub_marker = self.create_publisher(Marker, "~/uncertainty_ellipsoid", 10)
        self._pub_text = self.create_publisher(Marker, "~/uncertainty_text", 10)
        self._pub_trace = self.create_publisher(Float64, "~/pos_cov_trace", 10)
        self._pub_sigma = self.create_publisher(Float64, "~/pos_sigma_max", 10)
        self._pub_dim = self.create_publisher(Float64, "~/state_dim", 10)
        self._pub_nfeat = self.create_publisher(Float64, "~/num_slam_features", 10)

        self.get_logger().info(
            f"covariance_viz up: cov={cov_topic} odom={odom_topic} "
            f"frame={self.get_parameter('marker_frame').value} "
            f"sigma_scale={self.get_parameter('sigma_scale').value}x"
        )

    def _cb_odom(self, msg: Odometry):
        self._odom = msg

    def _cb_cov(self, msg: JointCovariance):
        try:
            cov = np.asarray(msg.covariance, dtype=float).reshape(msg.dim, msg.dim)
        except ValueError:
            self.get_logger().error(
                f"covariance length {len(msg.covariance)} != dim^2 ({msg.dim}^2)"
            )
            return

        # Offsets shift whenever OpenVINS marginalizes, so re-read them every
        # message rather than caching. See StateBlock.msg.
        imu_blk = None
        for b in msg.blocks:
            if b.type == "imu":
                imu_blk = b
                break

        self._pub_dim.publish(Float64(data=float(msg.dim)))
        self._pub_nfeat.publish(Float64(data=float(msg.num_slam_features)))
        self._log_landmark_parameterization(msg)

        if imu_blk is None or imu_blk.size != IMU_BLOCK_SIZE:
            if not self._warned_no_imu:
                self.get_logger().warn(
                    "no 15x15 IMU block in JointCovariance -- "
                    "nothing to draw (is joint_cov include_imu on?)"
                )
                self._warned_no_imu = True
            return

        i = imu_blk.index + IMU_POS_OFFSET
        pos_cov = cov[i:i + 3, i:i + 3]
        # Symmetrize before eigh: the wire matrix is symmetric to ~1e-18 but
        # eigh only reads one triangle, so make the choice explicit.
        pos_cov = 0.5 * (pos_cov + pos_cov.T)

        evals, evecs = np.linalg.eigh(pos_cov)
        # eigh can return small negatives on a near-singular block; clamp.
        evals = np.clip(evals, 0.0, None)
        sigmas = np.sqrt(evals)

        trace = float(np.trace(pos_cov))
        self._pub_trace.publish(Float64(data=trace))
        self._pub_sigma.publish(Float64(data=float(sigmas.max())))

        if self._odom is None:
            return

        self._publish_ellipsoid(msg, sigmas, evecs)
        if self.get_parameter("publish_text").value:
            self._publish_text(msg, sigmas, trace)

    def _publish_ellipsoid(self, msg, sigmas, evecs):
        gain = float(self.get_parameter("sigma_scale").value)
        floor = float(self.get_parameter("min_diameter").value)

        # eigh's eigenvector matrix is orthonormal but may be left-handed;
        # a Marker orientation must be a proper rotation.
        rot = np.array(evecs, dtype=float)
        if np.linalg.det(rot) < 0.0:
            rot[:, 0] = -rot[:, 0]
        qx, qy, qz, qw = rotation_to_quaternion(rot)

        p = self._odom.pose.pose.position

        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.get_parameter("marker_frame").value
        m.ns = "position_uncertainty"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = p.x
        m.pose.position.y = p.y
        m.pose.position.z = p.z
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        # Marker scale is a full diameter; sigma is a semi-axis.
        m.scale.x = max(2.0 * sigmas[0] * gain, floor)
        m.scale.y = max(2.0 * sigmas[1] * gain, floor)
        m.scale.z = max(2.0 * sigmas[2] * gain, floor)
        m.color.r = 1.0
        m.color.g = 0.35
        m.color.b = 0.0
        m.color.a = float(self.get_parameter("alpha").value)
        self._pub_marker.publish(m)

    def _publish_text(self, msg, sigmas, trace):
        p = self._odom.pose.pose.position
        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.get_parameter("marker_frame").value
        m.ns = "position_uncertainty_text"
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = p.x
        m.pose.position.y = p.y
        m.pose.position.z = p.z + 0.35
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.9
        # Raw numbers, not multiplied by sigma_scale.
        m.text = (
            f"1sigma {sigmas[2] * 1e3:.1f}/{sigmas[1] * 1e3:.1f}/{sigmas[0] * 1e3:.1f} mm\n"
            f"tr(Pxyz) {trace:.3e} m^2\n"
            f"dim {msg.dim}  feats {msg.num_slam_features}  clones {msg.num_clones}"
        )
        self._pub_text.publish(m)

    def _log_landmark_parameterization(self, msg):
        """Say once, out loud, why there are no per-landmark ellipsoids.

        Part (c) of the ask is deliberately NOT implemented, for two
        independent reasons -- neither of which is fixable inside this node:

        1. The landmark 3x3 blocks are ANCHORED_MSCKF_INVERSE_DEPTH. They are
           two bearing angles and an inverse depth relative to an anchor clone,
           NOT metric XYZ. Drawing them as world-frame ellipsoids would be a
           lie, and their trace mixes rad^2 with m^-2.
        2. Even with feat_rep_slam: GLOBAL_3D, this message carries covariance
           only -- no landmark POSITIONS. /points_slam has the positions but is
           a bare XYZ cloud with no feature ids, so there is no way to say which
           3x3 belongs at which point.

        Doing it properly means propagating through the anchored->XYZ Jacobian
        (which needs the anchor clone's pose, also absent here). That belongs
        in the planner, which has the estimator handle -- not in a viz node.
        """
        if self._logged_param or not msg.blocks:
            return
        for b in msg.blocks:
            if b.type == "slam_feature":
                self._logged_param = True
                self.get_logger().warn(
                    f"landmark blocks are '{b.parameterization}' "
                    f"(is_anchored={b.is_anchored}) -- per-landmark ellipsoids "
                    "SKIPPED on purpose: not metric XYZ, and this message "
                    "carries no landmark positions to place them at. "
                    "Only the IMU position block is drawn."
                )
                return


def main():
    rclpy.init()
    node = CovarianceViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
