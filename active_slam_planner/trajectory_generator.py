#!/usr/bin/env python3
"""Node 3 of 3: turn a selected goal pose into a streamed trajectory setpoint.

INTERFACES ARE REAL. THE TRAJECTORY IS NOT.

Subscribes:
    /viewpoint_selector/goal_pose   (geometry_msgs/PoseStamped)
    /odomimu                        (nav_msgs/Odometry)

Publishes:
    ~/setpoint_pose   (geometry_msgs/PoseStamped)   ROS-frame (FLU/ENU) setpoint
    ~/trajectory      (nav_msgs/Path)               full sampled path, for RViz

NOTE ON THE PX4 BOUNDARY: this node deliberately publishes a ROS-frame
PoseStamped, NOT a px4_msgs/TrajectorySetpoint. Frame conversion and anything
that touches the vehicle lives in openvins_to_px4.py behind a disabled-by-
default guard. Keeping the conversion in exactly one place is the point.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("trajectory_generator")

        self.declare_parameter("goal_topic", "/viewpoint_selector/goal_pose")
        self.declare_parameter("odom_topic", "/odomimu")
        self.declare_parameter("cruise_speed", 0.5)     # m/s
        self.declare_parameter("setpoint_rate", 20.0)   # Hz
        self.declare_parameter("num_samples", 40)
        self.declare_parameter("goal_tolerance", 0.10)  # m

        goal_topic = self.get_parameter("goal_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        rate = float(self.get_parameter("setpoint_rate").value)

        self._goal = None
        self._start = None
        self._pose = None
        self._t_start = None

        self.create_subscription(PoseStamped, goal_topic, self._cb_goal, 10)
        self.create_subscription(Odometry, odom_topic, self._cb_odom, 10)
        self._pub_sp = self.create_publisher(PoseStamped, "~/setpoint_pose", 10)
        self._pub_path = self.create_publisher(Path, "~/trajectory", 1)
        self.create_timer(1.0 / max(rate, 1e-3), self._tick)

        self.get_logger().info(f"trajectory_generator up: goal={goal_topic} rate={rate}Hz")
        self.get_logger().warn("TRAJECTORY IS A PLACEHOLDER - straight-line lerp, not minimum-snap")

    def _cb_odom(self, msg: Odometry):
        self._pose = msg.pose.pose

    def _cb_goal(self, msg: PoseStamped):
        if self._pose is None:
            self.get_logger().warn("goal received before any odometry - ignoring")
            return
        self._goal = msg
        self._start = self._pose
        self._t_start = self.get_clock().now()
        self._publish_path()

    def _duration(self):
        if self._goal is None or self._start is None:
            return 0.0
        d = math.dist(
            (self._start.position.x, self._start.position.y, self._start.position.z),
            (self._goal.pose.position.x, self._goal.pose.position.y, self._goal.pose.position.z),
        )
        v = max(float(self.get_parameter("cruise_speed").value), 1e-3)
        return d / v

    def _sample(self, s):
        """Linear interpolation at normalized time s in [0,1]. PLACEHOLDER."""
        s = min(max(s, 0.0), 1.0)
        p = PoseStamped()
        p.header.frame_id = self._goal.header.frame_id
        a, b = self._start, self._goal.pose
        p.pose.position.x = a.position.x + s * (b.position.x - a.position.x)
        p.pose.position.y = a.position.y + s * (b.position.y - a.position.y)
        p.pose.position.z = a.position.z + s * (b.position.z - a.position.z)
        # orientation: hold the goal attitude (PLACEHOLDER - no slerp)
        p.pose.orientation = b.orientation
        return p

    def _publish_path(self):
        n = int(self.get_parameter("num_samples").value)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._goal.header.frame_id
        path.poses = [self._sample(i / max(n - 1, 1)) for i in range(n)]
        for ps in path.poses:
            ps.header = path.header
        self._pub_path.publish(path)

    def _tick(self):
        if self._goal is None or self._start is None or self._t_start is None:
            return
        T = self._duration()
        if T <= 0.0:
            return
        el = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
        sp = self._sample(el / T)
        sp.header.stamp = self.get_clock().now().to_msg()
        self._pub_sp.publish(sp)

    # =====================================================================
    # ############  PLACEHOLDER - NOT A REAL TRAJECTORY GENERATOR  ########
    #
    # Currently: constant-speed straight-line interpolation from the pose at
    # goal-receipt to the goal, with the goal attitude held constant.
    #
    # This is NOT flyable as-is. It has, deliberately, none of:
    #   * Continuity. Velocity is discontinuous at both endpoints and every
    #     new goal restarts the lerp from the current pose, so the setpoint
    #     jumps. A quadrotor tracking this would jerk at every replan.
    #   * Dynamic feasibility. No velocity/acceleration/jerk limits are
    #     enforced and no thrust or attitude-rate feasibility is checked.
    #   * Minimum-snap. The real version is a polynomial (typically degree 7,
    #     minimizing snap) over a sequence of waypoints, which is what makes
    #     it differentially flat and trackable by a PX4-style controller.
    #   * Obstacle awareness. The straight line is NOT checked against the
    #     nvblox ESDF. It will happily route through a wall.
    #   * Attitude interpolation. Orientation should be slerped, and yaw
    #     usually wants to point along velocity or at the region being mapped.
    #
    # DO NOT connect this to a vehicle.
    # =====================================================================


def main():
    rclpy.init()
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
