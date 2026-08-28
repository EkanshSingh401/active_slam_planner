#!/usr/bin/env python3
"""One-shot bringup of the whole live demo: camera -> OpenVINS -> nvblox -> RViz.

    ros2 launch active_slam_planner demo.launch.py

Brings up, in order:
    realsense2_camera            D455, IR stereo + IMU + depth + color
    run_subscribe_msckf          OpenVINS VIO           -> /odomimu /pathimu /trackhist
    ov_odom_to_tf                /odomimu -> TF odom->camera_link
    nvblox_node                  TF-localized mapping   -> mesh + ESDF slice
    odom->global static TF       joins the two TF trees (see below)
    covariance_viz               joint covariance -> markers + Float64 scalars
    rviz2                        with rviz/demo.rviz

The OpenVINS + bridge + nvblox trio is NOT duplicated here -- it is included
from multi_drone_nvblox/launch/nvblox_openvins.launch.py, which already wires
them. This file adds the camera, the frame join, the covariance viz and RViz.

WHY THE STATIC odom->global TF
    nvblox's global_frame is `odom`; OpenVINS stamps /odomimu, /pathimu,
    /points_slam and its own TF in `global`. Those are two disconnected trees,
    so RViz can only ever resolve one of them against a single fixed frame.
    The bridge derives odom->camera_link straight from the global->imu pose, so
    odom and global are numerically the same frame -- publishing identity
    between them is exact, not an approximation. It makes the tree
    odom -> global -> imu -> cam0/cam1 alongside odom -> camera_link -> ...

CAMERA PARAMETER FIXES (fix_camera_params:=true, default)
    Two D455 gotchas that cost a lot of debugging time and do NOT survive being
    passed as launch arguments -- they have to be set on the running node:
      * depth_module.emitter_enabled must be 0. The IR projector's dot pattern
        is a field of fake, world-locked "features" that wrecks VIO.
      * IR auto-exposure latches at its 33 ms maximum, which fills the 33.3 ms
        frame period, so the IR streams silently drop to 15 Hz and every frame
        is motion-blurred. That is what made VIO diverge to kilometres. It does
        not recover on its own; toggling AE off/on unsticks it.
    Verify by measuring `ros2 topic hz /camera/infra1/image_rect_raw` (expect
    ~30). Reading back depth_module.exposure proves nothing -- it reports the
    last value *set*, not what AE actually chose.

RMW
    rmw_fastrtps_cpp is broken on this machine: publishers report subscribers
    but zero messages are delivered. CycloneDDS is forced below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERA_NODE = '/camera/camera'   # rs_launch.py: namespace=camera_name, name=camera_name


def generate_launch_description():
    run_camera = LaunchConfiguration('run_camera')
    run_rviz = LaunchConfiguration('run_rviz')
    fix_camera_params = LaunchConfiguration('fix_camera_params')
    sigma_scale = LaunchConfiguration('sigma_scale')
    ov_config = LaunchConfiguration('ov_config')
    rviz_config = LaunchConfiguration('rviz_config')

    default_rviz = PathJoinSubstitution(
        [FindPackageShare('active_slam_planner'), 'rviz', 'demo.rviz'])

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    nvblox_openvins_launch = os.path.join(
        get_package_share_directory('multi_drone_nvblox'), 'launch', 'nvblox_openvins.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_camera', default_value='true',
            description='Launch the RealSense driver (false if it is already up)'),
        DeclareLaunchArgument(
            'run_rviz', default_value='true',
            description='Launch RViz2 with rviz/demo.rviz'),
        DeclareLaunchArgument(
            'fix_camera_params', default_value='true',
            description='Force emitter off and unstick the IR auto-exposure latch'),
        DeclareLaunchArgument(
            'sigma_scale', default_value='50.0',
            description='DISPLAY GAIN on the uncertainty ellipsoid. 1-sigma is '
                        'millimetres and invisible at map scale; this only '
                        'scales the marker, never the published numbers.'),
        DeclareLaunchArgument(
            'ov_config',
            default_value='/workspaces/isaac_ros-dev/src/open_vins/config/rs_d455/estimator_config.yaml',
            description='OpenVINS estimator_config.yaml (container path)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz config file'),

        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # -- D455 ------------------------------------------------------------
        # Defaults put the node at /camera/camera and its topics under
        # /camera/..., which is exactly what open_vins/config/rs_d455 and
        # nvblox_openvins.launch.py already expect. Do not add a namespace.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(run_camera),
            launch_arguments={
                'enable_infra1': 'true',     # OpenVINS cam0
                'enable_infra2': 'true',     # OpenVINS cam1
                'enable_depth': 'true',      # nvblox
                'enable_color': 'true',      # nvblox mesh colouring
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',     # publish one /camera/imu at gyro rate
                'depth_module.profile': '848x480x30',
                'rgb_camera.profile': '848x480x30',
            }.items(),
        ),

        # -- camera parameter fixes (see docstring) --------------------------
        # MUST run before OpenVINS starts. Toggling auto-exposure re-enables the
        # IR stream, and a stream restart in the middle of static init starves
        # the filter of features at the exact moment it can least afford it.
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                condition=IfCondition(fix_camera_params),
                name='d455_param_fix', output='screen', shell=True,
                cmd=[
                    'ros2 param set ', CAMERA_NODE, ' depth_module.emitter_enabled 0 && ',
                    'ros2 param set ', CAMERA_NODE, ' depth_module.enable_auto_exposure false && ',
                    'ros2 param set ', CAMERA_NODE, ' depth_module.exposure 5000 && ',
                    'ros2 param set ', CAMERA_NODE, ' depth_module.enable_auto_exposure true && ',
                    'echo "[d455_param_fix] emitter off, IR auto-exposure re-armed"',
                ],
            ),
        ]),

        # -- OpenVINS + /odomimu->TF bridge + nvblox -------------------------
        # Starts only after the camera params have settled, so the imagery
        # OpenVINS initializes on is the same imagery it will keep receiving.
        TimerAction(period=14.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nvblox_openvins_launch),
                launch_arguments={
                    'run_openvins': 'true',
                    'ov_config': ov_config,
                }.items(),
            ),
        ]),

        # -- join the OpenVINS and nvblox TF trees (identity, see docstring) --
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='odom_to_global_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'global'],
            output='log',
        ),

        # -- covariance visualization ----------------------------------------
        TimerAction(period=14.0, actions=[
            Node(
                package='active_slam_planner', executable='covariance_viz',
                name='covariance_viz', output='screen',
                parameters=[{
                    'cov_topic': '/openvins/joint_covariance',
                    'odom_topic': '/odomimu',
                    'marker_frame': 'global',
                    'sigma_scale': sigma_scale,
                    'min_diameter': 0.02,
                    'alpha': 0.45,
                    'publish_text': True,
                }],
            ),
        ]),

        # -- RViz ------------------------------------------------------------
        TimerAction(period=16.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                condition=IfCondition(run_rviz),
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ]),
    ])
