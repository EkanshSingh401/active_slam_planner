#!/usr/bin/env python3
"""Offline unit tests for the ENU/FLU -> NED/FRD conversion.

This is the part of the PX4 bridge that is REAL, so it is the part that gets
tested. Runs without ROS, without px4_msgs, and without a vehicle:

    python3 test/test_frame_conversion.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from active_slam_planner.frame_conversions import (  # noqa: E402
    enu_flu_to_ned_frd_quat,
    enu_to_ned_position,
    flu_to_frd_body,
    qmul,
)

FAIL = []


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def check(cond, label):
    print(("  PASS  " if cond else "  FAIL  ") + label)
    if not cond:
        FAIL.append(label)


def quat_to_R(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def col(R, i):
    return [R[0][i], R[1][i], R[2][i]]


print("=== position: ENU -> NED ===")
check(enu_to_ned_position(1.0, 0.0, 0.0) == (0.0, 1.0, -0.0), "1m East  -> NED (0,1,0)")
check(enu_to_ned_position(0.0, 1.0, 0.0) == (1.0, 0.0, -0.0), "1m North -> NED (1,0,0)")
check(enu_to_ned_position(0.0, 0.0, 1.0) == (0.0, 0.0, -1.0), "1m Up    -> NED (0,0,-1) i.e. Down=-1")
check(enu_to_ned_position(3.0, -2.0, 5.0) == (-2.0, 3.0, -5.0), "general (3,-2,5) -> (-2,3,-5)")

print("\n=== body vectors: FLU -> FRD ===")
check(flu_to_frd_body(1.0, 0.0, 0.0) == (1.0, -0.0, -0.0), "Forward preserved")
check(flu_to_frd_body(0.0, 1.0, 0.0) == (0.0, -1.0, -0.0), "Left -> -Right")
check(flu_to_frd_body(0.0, 0.0, 1.0) == (0.0, -0.0, -1.0), "Up   -> -Down")

print("\n=== orientation: identity ENU/FLU ===")
# Body aligned with ENU: Forward=East, Left=North, Up=Up.
# In NED/FRD that is Forward=East=(0,1,0), Right=South=(-1,0,0), Down=(0,0,1).
# That is a +90 deg yaw about NED Down -> q = (cos45, 0, 0, sin45).
q = enu_flu_to_ned_frd_quat(1.0, 0.0, 0.0, 0.0)
check(close(q, [_S := math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]),
      f"identity -> +90deg yaw in NED, got {[round(c,6) for c in q]}")
R = quat_to_R(q)
check(close(col(R, 0), [0.0, 1.0, 0.0], 1e-9), "body Forward maps to NED East (0,1,0)")
check(close(col(R, 1), [-1.0, 0.0, 0.0], 1e-9), "body Right   maps to NED South (-1,0,0)")
check(close(col(R, 2), [0.0, 0.0, 1.0], 1e-9), "body Down    maps to NED Down (0,0,1)")

print("\n=== orientation: 90 deg yaw in ENU (nose from East to North) ===")
qy = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]  # +90 about ENU Up
q2 = enu_flu_to_ned_frd_quat(*qy)
R2 = quat_to_R(q2)
# Nose now points North = NED (1,0,0)
check(close(col(R2, 0), [1.0, 0.0, 0.0], 1e-9), "body Forward maps to NED North (1,0,0)")
check(close(col(R2, 2), [0.0, 0.0, 1.0], 1e-9), "body Down still NED Down")

print("\n=== algebraic properties ===")
for name, qin in [
    ("identity", [1.0, 0.0, 0.0, 0.0]),
    ("yaw45", [math.cos(math.pi / 8), 0.0, 0.0, math.sin(math.pi / 8)]),
    ("roll30", [math.cos(math.pi / 12), math.sin(math.pi / 12), 0.0, 0.0]),
    ("mixed", [0.5, 0.5, 0.5, 0.5]),
]:
    n = math.sqrt(sum(c * c for c in qin))
    qin = [c / n for c in qin]
    out = enu_flu_to_ned_frd_quat(*qin)
    norm = math.sqrt(sum(c * c for c in out))
    check(abs(norm - 1.0) < 1e-12, f"{name}: output is unit norm ({norm:.15f})")
    Rr = quat_to_R(out)
    det = (Rr[0][0] * (Rr[1][1] * Rr[2][2] - Rr[1][2] * Rr[2][1])
           - Rr[0][1] * (Rr[1][0] * Rr[2][2] - Rr[1][2] * Rr[2][0])
           + Rr[0][2] * (Rr[1][0] * Rr[2][1] - Rr[1][1] * Rr[2][0]))
    check(abs(det - 1.0) < 1e-9, f"{name}: rotation det == +1 (no mirror), det={det:.12f}")

print("\n=== involution: applying the transform twice returns the original ===")
# Both static quaternions are 180-deg rotations, so the map is its own inverse
# (up to sign). This is exactly why px4_odom_republisher.py reuses them for the
# opposite direction, and it is worth pinning down as a test.
for name, qin in [("yaw45", [math.cos(math.pi / 8), 0.0, 0.0, math.sin(math.pi / 8)]),
                  ("mixed", [0.5, 0.5, 0.5, 0.5])]:
    n = math.sqrt(sum(c * c for c in qin))
    qin = [c / n for c in qin]
    once = enu_flu_to_ned_frd_quat(*qin)
    twice = enu_flu_to_ned_frd_quat(*once)
    same = close(twice, qin, 1e-12) or close([-c for c in twice], qin, 1e-12)
    check(same, f"{name}: round trip returns original")

check(enu_to_ned_position(*enu_to_ned_position(1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0),
      "position round trip is identity")
check(flu_to_frd_body(*flu_to_frd_body(1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0),
      "body vector round trip is identity")

print("\n=== quaternion helper sanity ===")
check(close(qmul([1, 0, 0, 0], [0.5, 0.5, 0.5, 0.5]), [0.5, 0.5, 0.5, 0.5]), "qmul identity")
check(close(qmul([0, 1, 0, 0], [0, 1, 0, 0]), [-1, 0, 0, 0]), "i*i = -1")

print("\nRESULT: " + ("ALL PASSED" if not FAIL else f"{len(FAIL)} FAILED"))
for f in FAIL:
    print("  - " + f)
sys.exit(1 if FAIL else 0)
