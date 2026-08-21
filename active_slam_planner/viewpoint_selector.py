#!/usr/bin/env python3
"""Node 2 of 3: pick one goal pose from the scored candidates.

INTERFACES ARE REAL. THE SELECTION POLICY IS NOT.

Subscribes:
    /fisher_ig_estimator/scored_viewpoints  (active_slam_msgs/ScoredViewpointArray)

Publishes:
    ~/goal_pose   (geometry_msgs/PoseStamped)
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from active_slam_msgs.msg import ScoredViewpointArray


class ViewpointSelector(Node):
    def __init__(self):
        super().__init__("viewpoint_selector")

        self.declare_parameter("input_topic", "/fisher_ig_estimator/scored_viewpoints")
        self.declare_parameter("min_score", -1e30)
        self.declare_parameter("require_reachable", True)
        # Present so the shape of the real decision is visible, but NOT yet
        # used in any meaningful way -- see the PLACEHOLDER banner.
        self.declare_parameter("w_information", 1.0)
        self.declare_parameter("w_coverage", 0.0)
        self.declare_parameter("w_travel_cost", 0.0)

        topic = self.get_parameter("input_topic").value
        self.create_subscription(ScoredViewpointArray, topic, self._cb, 10)
        self._pub = self.create_publisher(PoseStamped, "~/goal_pose", 10)

        self._warned = False
        self.get_logger().info(f"viewpoint_selector up: input={topic}")
        self.get_logger().warn("SELECTION POLICY IS A PLACEHOLDER - plain argmax, no tradeoff logic")

    def _cb(self, msg: ScoredViewpointArray):
        if msg.is_placeholder and not self._warned:
            self._warned = True
            self.get_logger().warn(
                f"incoming scores are flagged is_placeholder=True "
                f"(method='{msg.scoring_method}') - selection output is not meaningful yet"
            )

        cands = list(msg.viewpoints)
        if self.get_parameter("require_reachable").value:
            cands = [c for c in cands if c.is_reachable]
        min_score = float(self.get_parameter("min_score").value)
        cands = [c for c in cands if c.score >= min_score]
        if not cands:
            return

        best = self._select(cands)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose = best.pose
        self._pub.publish(out)

    # =====================================================================
    # ############  PLACEHOLDER - THIS IS WHERE THE REAL TRADEOFF GOES  ####
    #
    # Currently: plain argmax over the score the estimator already computed.
    #
    # This node is deliberately the place where the coverage-vs-uncertainty
    # balance will live, and that decision HAS NOT BEEN MADE. Open questions:
    #   * How to trade expected information gain against coverage of unknown
    #     space against travel cost. The w_* parameters above are declared to
    #     show the intended shape, but a fixed linear weighting may well be
    #     the wrong model.
    #   * Whether selection should be greedy per-cycle (as here) or over a
    #     receding horizon of several viewpoints.
    #   * Hysteresis / commitment: naive per-cycle argmax will dither between
    #     near-equal candidates and produce useless oscillating goals. Some
    #     commitment mechanism is required before this drives a real vehicle.
    #   * Whether to fall back to a frontier-style objective when the
    #     information gain is flat (e.g. staring at a textureless wall).
    #
    # DO NOT treat the current output as a planner.
    # =====================================================================
    def _select(self, cands):
        return max(cands, key=lambda c: c.score)


def main():
    rclpy.init()
    node = ViewpointSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
