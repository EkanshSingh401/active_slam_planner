#!/usr/bin/env python3
"""ROS(ENU/FLU) <-> PX4(NED/FRD) frame conversions.

Deliberately has NO ROS imports so it can be unit-tested offline, without a
workspace, without px4_msgs and without a vehicle. This is the error-prone part
of the PX4 bridge, so it is isolated and pinned down by tests
(see test/test_frame_conversion.py).

FRAME CONVENTIONS
    ROS : world ENU (x East, y North, z Up),   body FLU (x Fwd, y Left, z Up)
    PX4 : world NED (x North, y East, z Down), body FRD (x Fwd, y Right, z Dn)

Both static quaternions below are 180-degree rotations and therefore their own
inverses, which is why the same constants serve both directions (and why
multi_drone_nvblox/scripts/px4_odom_republisher.py can reuse them going the
other way).
"""

import math

_S = 1.0 / math.sqrt(2.0)

# [w, x, y, z]
Q_NED_ENU = [0.0, _S, _S, 0.0]    # 180 deg about (1,1,0)/sqrt(2): ENU <-> NED
Q_FLU_FRD = [0.0, 1.0, 0.0, 0.0]  # 180 deg about x: FLU <-> FRD


def qmul(a, b):
    """Hamilton product, [w, x, y, z] order."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def enu_to_ned_position(x, y, z):
    """(East, North, Up) -> (North, East, Down). Self-inverse."""
    return (y, x, -z)


def flu_to_frd_body(x, y, z):
    """Body-frame vector FLU -> FRD (body velocity, or body rates). Self-inverse."""
    return (x, -y, -z)


def enu_flu_to_ned_frd_quat(qw, qx, qy, qz):
    """Orientation of FLU body in ENU world -> FRD body in NED world.

    Input and output are [w, x, y, z]. The result is normalised and
    sign-canonicalised to w >= 0, so repeated calls are directly comparable
    (q and -q represent the same rotation).
    """
    q = qmul(Q_NED_ENU, qmul([qw, qx, qy, qz], Q_FLU_FRD))
    n = math.sqrt(sum(c * c for c in q))
    if n > 0.0:
        q = [c / n for c in q]
    if q[0] < 0.0:
        q = [-c for c in q]
    return q
