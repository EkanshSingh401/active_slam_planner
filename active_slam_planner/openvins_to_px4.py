#!/usr/bin/env python3
"""OpenVINS odometry -> PX4 VehicleOdometry bridge.

STATUS: frame conversion is REAL and unit-tested. Publishing to the vehicle is
STUBBED OFF behind `enabled` (default False) and cannot fire accidentally.

Direction matters. multi_drone_nvblox/scripts/px4_odom_republisher.py already
goes PX4 -> ROS (NED/FRD -> ENU/FLU) for RViz. This node is the INBOUND leg,
ROS -> PX4. They are complementary, not duplicates; that node is untouched.
Conveniently it already subscribes to /fmu/in/vehicle_visual_odometry, so once
this node is enabled it will visualise exactly what we send.

Subscribes:
    /odomimu   (nav_msgs/Odometry)   OpenVINS, ENU-ish world / FLU-ish body

Publishes:
    ~/vehicle_odometry_ned  (geometry_msgs/PoseStamped)  ALWAYS - converted
                            pose for inspection without px4_msgs installed
    /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)
                            ONLY when enabled=true AND px4_msgs is importable

FRAME CONVENTIONS
    ROS  : world ENU (x East, y North, z Up),  body FLU (x Fwd, y Left, z Up)
    PX4  : world NED (x North, y East, z Down), body FRD (x Fwd, y Right, z Dn)

    position/velocity  (E,N,U) -> (N,E,D) = (y, x, -z)
    orientation        q_ned_frd = q_ned_enu (x) q_enu_flu (x) q_flu_frd
    body rates         (x,y,z)_FLU -> (x,-y,-z)_FRD

    Both static quaternions below are 180-degree rotations, hence their own
    inverses, which is why px4_odom_republisher.py can use the same constants
    for the opposite direction.

CAVEAT - OpenVINS "global" is gravity-aligned z-up but its yaw is arbitrary
(yaw is unobservable and gets fixed at init). So it is ENU-*like*, not true
East/North. PX4 EKF2 tolerates this for visual odometry, but the yaw offset is
real and must not be mistaken for a heading reference.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from active_slam_planner.frame_conversions import (
    enu_flu_to_ned_frd_quat,
    enu_to_ned_position,
    flu_to_frd_body,
)


class OpenVinsToPX4(Node):
    def __init__(self):
        super().__init__("openvins_to_px4")

        # ###############################################################
        # SAFETY: default False. Nothing is sent to the flight controller
        # unless this is explicitly turned on. Do not default this to True.
        # ###############################################################
        self.declare_parameter("enabled", False)
        self.declare_parameter("odom_topic", "/odomimu")
        self.declare_parameter("px4_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("pose_variance", [0.01, 0.01, 0.01])
        self.declare_parameter("orientation_variance", [0.01, 0.01, 0.01])
        self.declare_parameter("velocity_variance", [0.05, 0.05, 0.05])

        self.enabled = bool(self.get_parameter("enabled").value)
        odom_topic = self.get_parameter("odom_topic").value
        self.px4_topic = self.get_parameter("px4_topic").value

        self._pub_dbg = self.create_publisher(PoseStamped, "~/vehicle_odometry_ned", 10)
        self._pub_px4 = None
        self._VehicleOdometry = None

        if self.enabled:
            try:
                from px4_msgs.msg import VehicleOdometry  # noqa: F401
                self._VehicleOdometry = VehicleOdometry
                self._pub_px4 = self.create_publisher(VehicleOdometry, self.px4_topic, 10)
                self.get_logger().warn(
                    f"ENABLED - publishing VehicleOdometry to {self.px4_topic}. "
                    "This feeds a flight controller."
                )
            except ImportError:
                self.get_logger().error(
                    "enabled=true but px4_msgs is not importable; staying inert. "
                    "Build px4_msgs into the workspace first."
                )
                self.enabled = False
        else:
            self.get_logger().info(
                "inert (enabled=false): converting and publishing "
                "~/vehicle_odometry_ned only, nothing sent to PX4"
            )

        self.create_subscription(Odometry, odom_topic, self._cb, 10)
        self.get_logger().info(f"openvins_to_px4 up: odom={odom_topic}")

    def _cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        nx, ny, nz = enu_to_ned_position(p.x, p.y, p.z)
        nq = enu_flu_to_ned_frd_quat(q.w, q.x, q.y, q.z)
        # OpenVINS reports twist.linear in the LOCAL (body) frame -- see
        # ROS2Visualizer.cpp, "vel in local frame" -- so this is FLU->FRD,
        # not ENU->NED.
        vx, vy, vz = flu_to_frd_body(v.x, v.y, v.z)
        wx, wy, wz = flu_to_frd_body(w.x, w.y, w.z)

        dbg = PoseStamped()
        dbg.header.stamp = msg.header.stamp
        dbg.header.frame_id = "ned"
        dbg.pose.position.x, dbg.pose.position.y, dbg.pose.position.z = nx, ny, nz
        dbg.pose.orientation.w = nq[0]
        dbg.pose.orientation.x = nq[1]
        dbg.pose.orientation.y = nq[2]
        dbg.pose.orientation.z = nq[3]
        self._pub_dbg.publish(dbg)

        if not self.enabled or self._pub_px4 is None:
            return

        VO = self._VehicleOdometry
        out = VO()
        stamp_us = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec * 1e-3)
        out.timestamp = stamp_us
        out.timestamp_sample = stamp_us
        out.pose_frame = VO.POSE_FRAME_NED
        out.position = [float(nx), float(ny), float(nz)]
        out.q = [float(c) for c in nq]
        out.velocity_frame = VO.VELOCITY_FRAME_BODY_FRD
        out.velocity = [float(vx), float(vy), float(vz)]
        out.angular_velocity = [float(wx), float(wy), float(wz)]
        out.position_variance = [float(x) for x in self.get_parameter("pose_variance").value]
        out.orientation_variance = [float(x) for x in self.get_parameter("orientation_variance").value]
        out.velocity_variance = [float(x) for x in self.get_parameter("velocity_variance").value]
        # TODO: OpenVINS resets are not currently detected; PX4 uses
        # reset_counter to know the estimate jumped. See README.
        out.reset_counter = 0
        out.quality = 0
        self._pub_px4.publish(out)


def main():
    rclpy.init()
    node = OpenVinsToPX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
