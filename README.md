# active_slam_planner

Scaffold for the Fisher-information-based active SLAM pipeline, plus the
OpenVINS → PX4 visual-odometry bridge.

**Read this first:** interfaces, topics, message types, launch wiring and the
frame conversions are real and working. The *algorithms* in nodes 1–3 are
explicitly marked placeholders. Every one carries a `PLACEHOLDER` banner and
logs a warning on startup. `ScoredViewpointArray.is_placeholder` is `true` on
the wire so a consumer can tell programmatically.

## What is real vs. what is stubbed

| Component | Status |
|---|---|
| `active_slam_msgs` message definitions | **real** |
| OpenVINS joint covariance publisher (Part 1) | **real** |
| Topic wiring / launch / parameters | **real** |
| Joint-covariance unpacking (`block()`, `cross_block()`) | **real** |
| ENU/FLU → NED/FRD conversion (`frame_conversions.py`) | **real, unit-tested** |
| Fisher information gain scoring | **PLACEHOLDER** — constant |
| Candidate viewpoint sampling | **PLACEHOLDER** — fixed-radius ring |
| Viewpoint selection policy | **PLACEHOLDER** — plain argmax |
| Trajectory generation | **PLACEHOLDER** — straight-line lerp |
| PX4 publish path | **stubbed off**, `enabled:=false` by default |

## Nodes

### `fisher_ig_estimator`
Subscribes `/openvins/joint_covariance`, the nvblox ESDF, and `/odomimu`;
publishes `~/scored_viewpoints`.

The covariance-unpacking helpers are the genuinely useful part. `block(type,
feature_id)` and `cross_block(b1, b2)` slice named blocks out of the joint
matrix, **re-reading offsets every message** — mandatory, because OpenVINS
re-indexes its covariance whenever a clone or feature is marginalized.

### `viewpoint_selector`
Subscribes the scored viewpoints, publishes `~/goal_pose`. This is where the
coverage-vs-uncertainty tradeoff will live; that decision is **not made**. The
`w_information` / `w_coverage` / `w_travel_cost` parameters exist to show the
intended shape only.

### `trajectory_generator`
Subscribes `~/goal_pose`, publishes `~/setpoint_pose` and `~/trajectory`.
Deliberately emits a **ROS-frame** `PoseStamped`, not a PX4 message, so frame
conversion lives in exactly one place.

### `openvins_to_px4`
Converts `/odomimu` to PX4 NED/FRD. Always publishes
`~/vehicle_odometry_ned` (a `PoseStamped`) so the conversion is inspectable
without `px4_msgs` installed. Only publishes real `px4_msgs/VehicleOdometry`
when `enabled:=true` **and** `px4_msgs` imports.

## Relationship to `px4_odom_republisher.py`

`multi_drone_nvblox/scripts/px4_odom_republisher.py` goes **PX4 → ROS**
(NED/FRD → ENU/FLU) to visualise the flight controller's estimate in RViz.
`openvins_to_px4` is the **opposite direction**, ROS → PX4.

They are **complementary, not duplicates.** The existing script is unmodified
and should stay that way — it is part of the working cuVSLAM path. Usefully, it
already subscribes to `/fmu/in/vehicle_visual_odometry`, so once our bridge is
enabled it will visualise exactly what we send, which is a free correctness
check.

## Frame conventions

```
ROS : world ENU (x East,  y North, z Up)    body FLU (x Fwd, y Left,  z Up)
PX4 : world NED (x North, y East,  z Down)  body FRD (x Fwd, y Right, z Down)

position/velocity  (E,N,U) -> (N,E,D) = (y, x, -z)
orientation        q_ned_frd = q_ned_enu ⊗ q_enu_flu ⊗ q_flu_frd
body rates         (x,y,z)_FLU -> (x,-y,-z)_FRD
```

Both static quaternions are 180° rotations and hence self-inverse, which is why
the same constants serve both directions.

Run the offline tests (no ROS, no px4_msgs, no vehicle needed):

```bash
python3 test/test_frame_conversion.py
```

⚠️ **OpenVINS yaw is arbitrary.** Its `global` frame is gravity-aligned z-up,
but yaw is unobservable and gets frozen at initialization. So it is ENU-*like*,
not true East/North. PX4 EKF2 tolerates this for visual odometry, but do not
mistake it for a heading reference.

## TODO: what remains for the PX4 control leg

None of this is implemented. Listed so the gap is explicit.

**1. `px4_msgs` is not in this workspace.** It is not currently built. Add it
(`git clone https://github.com/PX4/px4_msgs`, matching your PX4 firmware
release — message definitions change between releases and a mismatch fails
silently or deserializes garbage).

**2. uXRCE-DDS transport.** PX4 v1.14+ uses `uxrce_dds_client` on the vehicle
and `MicroXRCEAgent` on the companion. Neither is set up here. Without the
agent running, `/fmu/in/...` and `/fmu/out/...` simply do not exist.

**3. EKF2 parameters for external vision** — must be set on the flight
controller, not here:

| Param | Purpose |
|---|---|
| `EKF2_EV_CTRL` | bitmask enabling EV horizontal position / vertical position / velocity / yaw fusion |
| `EKF2_HGT_REF` | set to Vision if EV should be the primary height reference |
| `EKF2_EV_DELAY` | EV measurement delay (ms) — must reflect the real OpenVINS latency |
| `EKF2_EV_POS_X/Y/Z` | camera/IMU offset from vehicle body origin |
| `EKF2_GPS_CTRL` | usually 0 indoors, so EV is not fighting GPS |
| `EKF2_BARO_CTRL` | consider disabling if EV height is trusted |

**4. Offboard mode + arming.** PX4 requires `OffboardControlMode` streaming at
**> 2 Hz before** the mode switch is accepted, and it must keep streaming or
PX4 fails safe. Then `VehicleCommand` `DO_SET_MODE` → offboard, then arm. None
of this exists yet — `trajectory_generator` publishes a ROS `PoseStamped`, not
`TrajectorySetpoint` or `OffboardControlMode`.

**5. Setpoint streaming rates.** `VehicleVisualOdometry` at roughly
30–50 Hz (OpenVINS runs ~30 Hz, which is adequate);
`OffboardControlMode` + `TrajectorySetpoint` at ≥ 10 Hz, 20–50 Hz typical.

**6. `reset_counter`.** Currently hardcoded to 0. PX4 uses it to know the
external estimate jumped (e.g. OpenVINS re-initialized). Leaving it at 0 across
a re-init will make EKF2 fuse a discontinuity as if it were real motion. This
needs wiring to actual OpenVINS reset detection before flight.

**7. Timestamp domain.** PX4 wants microseconds on the **vehicle's** clock.
This node currently forwards the ROS stamp. Without clock alignment (the
uXRCE-DDS agent can provide offset estimation) `EKF2_EV_DELAY` compensation is
meaningless.

**8. Safety.** Nothing here has been flight-tested. The publish path is behind
two independent switches (`enable_px4_bridge` to run the node,
`px4_publish` to let it transmit) and both default to false. Keep it that way
until the above is done and bench-tested with props off.
