#!/usr/bin/env python3
"""Wire the three active-SLAM planner nodes together (+ optional PX4 bridge).

SCAFFOLD. The nodes run and the topics connect end to end, but the algorithms
inside nodes 1-3 are placeholders. See each node's PLACEHOLDER banner.

Chain:
    /openvins/joint_covariance ──┐
    <esdf_topic> ────────────────┼─> fisher_ig_estimator
    /odomimu ────────────────────┘        │ ~/scored_viewpoints
                                          v
                                   viewpoint_selector
                                          │ ~/goal_pose
                                          v
                                  trajectory_generator
                                          │ ~/setpoint_pose
                                          v
                                   (PX4 leg -- NOT wired, see README)

This does NOT launch OpenVINS, the RealSense driver, or nvblox. Bring those up
separately; it deliberately does not touch the cuVSLAM multi_drone_nvblox path.

Usage:
    ros2 launch active_slam_planner active_slam.launch.py
    ros2 launch active_slam_planner active_slam.launch.py enable_px4_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esdf_topic = LaunchConfiguration('esdf_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    enable_px4_bridge = LaunchConfiguration('enable_px4_bridge')
    px4_publish = LaunchConfiguration('px4_publish')

    return LaunchDescription([
        DeclareLaunchArgument(
            'esdf_topic', default_value='/nvblox_node/static_esdf_pointcloud',
            description='nvblox ESDF pointcloud topic'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odomimu',
            description='OpenVINS odometry topic'),
        DeclareLaunchArgument(
            'enable_px4_bridge', default_value='false',
            description='Run the openvins_to_px4 node at all'),
        DeclareLaunchArgument(
            # SAFETY: even with the bridge node running, this must be set true
            # before anything is sent to a flight controller. Two separate
            # switches on purpose.
            'px4_publish', default_value='false',
            description='DANGER: actually publish VehicleOdometry to /fmu/in/...'),

        Node(
            package='active_slam_planner', executable='fisher_ig_estimator',
            name='fisher_ig_estimator', output='screen',
            parameters=[{
                'esdf_topic': esdf_topic,
                'odom_topic': odom_topic,
                'rate': 2.0,
                'num_candidates': 24,
                'candidate_radius': 2.0,
            }],
        ),
        Node(
            package='active_slam_planner', executable='viewpoint_selector',
            name='viewpoint_selector', output='screen',
            parameters=[{
                'input_topic': '/fisher_ig_estimator/scored_viewpoints',
                'require_reachable': True,
            }],
        ),
        Node(
            package='active_slam_planner', executable='trajectory_generator',
            name='trajectory_generator', output='screen',
            parameters=[{
                'goal_topic': '/viewpoint_selector/goal_pose',
                'odom_topic': odom_topic,
                'cruise_speed': 0.5,
                'setpoint_rate': 20.0,
            }],
        ),
        GroupAction(
            condition=IfCondition(enable_px4_bridge),
            actions=[
                Node(
                    package='active_slam_planner', executable='openvins_to_px4',
                    name='openvins_to_px4', output='screen',
                    parameters=[{
                        'odom_topic': odom_topic,
                        'enabled': px4_publish,
                    }],
                ),
            ],
        ),
    ])
