#!/usr/bin/env python3
"""Offline self-test for eigen_diagnostic.py. No ROS, no hardware.

Builds a synthetic joint covariance whose near-null directions are planted
ENTIRELY INSIDE THE CLONE BLOCK, then checks the diagnostic localises them
there. If null_energy_by_block cannot find a degeneracy we put in on purpose,
it cannot be trusted on real data either.

    python3 test_eigen_diagnostic.py
"""
import pickle
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import eigen_diagnostic as ed

N_FEAT = 30
N_NULL = 5
rng = np.random.default_rng(7)


def make_blocks():
    blocks, i = [], 0
    blocks.append(dict(type="imu", index=i, size=15, state_id=0, clone_timestamp=0.0,
                       feature_id=-1, camera_id=-1, parameterization="", is_anchored=False,
                       anchor_clone_timestamp=0.0)); i += 15
    blocks.append(dict(type="calib_dt", index=i, size=1, state_id=0, clone_timestamp=0.0,
                       feature_id=-1, camera_id=-1, parameterization="", is_anchored=False,
                       anchor_clone_timestamp=0.0)); i += 1
    for k in range(11):
        blocks.append(dict(type="clone", index=i, size=6, state_id=0,
                           clone_timestamp=1000.0 + 0.04 * k, feature_id=-1, camera_id=-1,
                           parameterization="", is_anchored=False,
                           anchor_clone_timestamp=0.0)); i += 6
    for k in range(N_FEAT):
        blocks.append(dict(type="slam_feature", index=i, size=3, state_id=0,
                           clone_timestamp=0.0, feature_id=k, camera_id=0,
                           parameterization="ANCHORED_MSCKF_INVERSE_DEPTH",
                           is_anchored=True, anchor_clone_timestamp=1000.0)); i += 3
    return blocks, i


def synth(dim, blocks, seed):
    r = np.random.default_rng(seed)
    clone_idx = np.array(
        [j for b in blocks if b["type"] == "clone" for j in range(b["index"], b["index"] + b["size"])])
    # N_NULL orthonormal vectors supported ONLY on the clone block
    M = np.zeros((dim, N_NULL))
    M[clone_idx, :] = r.normal(size=(clone_idx.size, N_NULL))
    Qn, _ = np.linalg.qr(M)
    # complete to a full orthonormal basis
    R = r.normal(size=(dim, dim - N_NULL))
    R -= Qn @ (Qn.T @ R)
    Qr, _ = np.linalg.qr(R)
    V = np.hstack([Qn, Qr])
    ev = np.concatenate([np.full(N_NULL, 1e-20), 10 ** r.uniform(-6, -2, dim - N_NULL)])
    cov = V @ np.diag(ev) @ V.T
    return 0.5 * (cov + cov.T)


def main():
    blocks, dim = make_blocks()
    samples = []
    for k in range(30):
        samples.append(dict(stamp=1000.0 + k * 0.2, wall=0.0, dim=dim,
                            cov=synth(dim, blocks, 100 + k), blocks=blocks,
                            full_state_dim=dim, num_clones=11, num_slam_features=N_FEAT,
                            includes_imu=True, includes_clones=True,
                            includes_slam_features=True, includes_calibration=True,
                            pose=(0.1, 0.1, 0.0)))
    ok = True

    # 1. null energy must land on the clone block
    ne = ed.null_energy_by_block(samples[0])
    assert ne is not None, "diagnostic found no near-null directions in a planted-null matrix"
    print(f"  planted {N_NULL} null dirs in clone block; found {ne['n_null']} of {ne['dim']}")
    for k, v in sorted(ne["share"].items(), key=lambda x: -x[1]["mean"]):
        print(f"    {k:<16} energy {v['mean']:6.1%}")
    clone_share = ne["share"]["clone"]["mean"]
    ok &= ne["n_null"] == N_NULL
    ok &= clone_share > 0.99
    print(f"  -> localisation {'PASS' if clone_share > 0.99 else 'FAIL'} "
          f"(clone share {clone_share:.3%}, want >99%)")

    # 2. full matrix must be rank deficient by exactly N_NULL; IMU block must be clean
    full = ed.spectrum(samples[0]["cov"])
    imu = ed.spectrum(ed.submatrix(samples[0]["cov"],
                                   [b for b in blocks if b["type"] == "imu"]))
    print(f"  full: dim {full['dim']} rank {full['rank']} deficit {full['rank_deficit']}")
    print(f"  imu : dim {imu['dim']} rank {imu['rank']} cond {imu['cond_eig']:.2e}")
    ok &= full["rank_deficit"] == N_NULL
    ok &= imu["rank_deficit"] == 0

    # 3. Cholesky must fail on the full matrix, succeed on the IMU block
    cf = ed.cholesky_check(samples[0]["cov"])
    ci = ed.cholesky_check(ed.submatrix(samples[0]["cov"],
                                        [b for b in blocks if b["type"] == "imu"]))
    print(f"  chol full={cf['ok']} (want False)   chol imu={ci['ok']} (want True)")
    ok &= (not cf["ok"]) and ci["ok"]

    # 4. cond(C) ~ sqrt(cond(A)) on a well-conditioned block
    if ci["ok"]:
        ratio = ci["cond_factor"] / np.sqrt(imu["cond_eig"])
        print(f"  cond(C)/sqrt(cond A) = {ratio:.3f} (want ~1)")
        ok &= 0.5 < ratio < 2.0

    # 5. end-to-end: the CLI must run and emit all three artefacts
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "s.pkl"
        p.write_bytes(pickle.dumps(samples))
        r = subprocess.run([sys.executable, str(Path(__file__).parent / "eigen_diagnostic.py"),
                            "--in", str(p), "--outdir", d, "--steps", "10"],
                           capture_output=True, text=True)
        made = [f for f in ("eigen_report.txt", "eigen_results.json",
                            "eigen_condition_vs_time.png") if (Path(d) / f).exists()]
        print(f"  CLI exit {r.returncode}, artefacts {made}")
        ok &= r.returncode == 0 and len(made) == 3
        if r.returncode != 0:
            print(r.stdout[-1500:], r.stderr[-1500:])

    print("\nSELF-TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
