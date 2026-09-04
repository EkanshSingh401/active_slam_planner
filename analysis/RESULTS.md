# Eigenspectrum diagnostic — results

Recorded 2026-09-04 on prometheus (Jetson Orin NX), D455 stereo + IMU,
`open_vins` branch `openvins-integration`, config `rs_d455`.

Numbers only. No conclusion is drawn here about which planning criterion to use.

## Two runs

| | `moving_camera` (**primary**) | `static_camera` (corroborating) |
|---|---|---|
| samples | 439 over 90.8 s | 292 over 60.4 s |
| timesteps analysed | 25 | 25 |
| dim | 208 … 220 | 85 … 217 |
| SLAM features | **42 … 46 (mean 43.1)** | 1 … 45 (mean 4.9) |
| \|p\| | 0.234 … 0.238 m — bounded | 0.014 … 0.052 m — bounded |
| IR | 30.0 Hz (33.3 ms) | 30.0 Hz (33.3 ms) |
| OpenVINS lag | ~1.6 ms (real-time) | ~1.2 ms (real-time) |

The static run's feature population collapsed 45 → 2 within ~10 s: with no
parallax OpenVINS marginalises SLAM features it cannot triangulate. It is kept
because **every structural result below is identical across the two runs**,
across dim 85→220 and 2→46 features, which is itself the strongest evidence
that the finding is structural rather than data-dependent.

All figures below are from `moving_camera` unless stated.

## Per-block spectrum (mean over 25 timesteps)

| block | dim | cond (eig) | rank | deficit | lambda_min | Cholesky |
|---|---|---|---|---|---|---|
| FULL joint | 208–220 | **inf** | 202–214 | **6** | −4.26e−18 | **0/25** |
| IMU (15) | 15 | 1.01e6 | 15 | 0 | 2.61e−09 | 25/25, cond(C) 1.00e3 |
| calib_dt (1) | 1 | 1.00 | 1 | 0 | 7.86e−09 | 25/25, cond(C) 1.00 |
| clones only | 66 | 6.70e9 | 66 | 0 | 4.25e−12 | 25/25, cond(C) 8.17e4 |
| SLAM feats only | 126–138 | 2.21e5 | 126–138 | 0 | 3.12e−09 | 25/25, cond(C) 4.64e2 |
| IMU + feats | 141–153 | 1.43e6 | 141–153 | 0 | 2.12e−09 | 25/25, cond(C) 1.17e3 |
| IMU + clones | 81 | **inf** | 75–76 | **5–6** | −3.59e−18 | 1/25 |
| IMU+clones+feats | 207–219 | 1.34e17* | 201–213 | **5–6** | −3.28e−18 | 2/25 |
| FULL minus calib | 207–219 | 1.34e17* | 201–213 | **5–6** | −3.28e−18 | 2/25 |

`*` finite at only the 2/25 timesteps that were PD; averaged over those.

Full-matrix SVD condition number ranged **1e17 … 4e18**, consistent with the
previously measured 5.6e17. `lambda_min` is *negative*, so the matrix is not
PSD to working precision and the eigenvalue ratio is undefined.

## Where the degeneracy lives

Near-null eigenvectors — 6.0 of 211 dims — energy by block type:

| block type | share |
|---|---|
| clone | **50.00 %** |
| imu | **50.00 %** |
| calib_dt | 0.00 % |
| slam_feature | **0.00 %** |

Resolving the imu/clone split:

| component | share |
|---|---|
| IMU pose (quat, pos) | **50.00 %** |
| IMU vel / gyro-bias / accel-bias | 0.00 % |
| **newest clone** | **50.00 %** |
| all older clones | 0.00 % |
| **IMU pose + newest clone** | **100.00 %** |

`<v_imu_pose, v_newest_clone> = −0.5000`, newest-clone age `+0.0000 s`.

An inner product of exactly −0.5 between unit-norm 6-vectors is the signature
of the pairing `v = (u, −u)/sqrt(2)`. With a clone age of zero this says the six
near-null directions **are `IMU_pose − newest_clone`** — identically zero,
because OpenVINS creates each clone as a bit-exact copy of the current IMU
pose. The degeneracy is structural and exactly predictable. It involves neither
the velocity/bias states, the older clones, nor the map.

(`static_camera` gives 99.85 % and −0.4992 — the same result, marginally
noisier because some sampled timesteps caught a clone mid-update.)

## Corroboration from the submatrix table

Every block set that **fails** Cholesky contains **both** the IMU and the clone
block. Every set that **succeeds 25/25** contains at most one of them:

| succeeds 25/25 | cond | | fails | Cholesky |
|---|---|---|---|---|
| calib_dt | 1.00 | | FULL joint | 0/25 |
| SLAM feats only | 2.21e5 | | IMU + clones | 1/25 |
| IMU (15) | 1.01e6 | | IMU+clones+feats | 2/25 |
| IMU + feats | 1.43e6 | | FULL minus calib | 2/25 |
| clones only | 6.70e9 | | | |

## Clone conditioning vs sliding-window length

| #clones | dim | span (s) | cond | rank | deficit | lambda_min | Cholesky |
|---|---|---|---|---|---|---|---|
| 2 | 12 | 0.05 | 1.24e7 | 12 | 0 | 6.32e−10 | 25/25 |
| 4 | 24 | 0.13 | 8.65e8 | 24 | 0 | 1.46e−11 | 25/25 |
| 6 | 36 | 0.22 | 2.48e9 | 36 | 0 | 7.07e−12 | 25/25 |
| 8 | 48 | 0.31 | 4.06e9 | 48 | 0 | 5.60e−12 | 25/25 |
| 11 | 66 | 0.42 | 6.70e9 | 66 | 0 | 4.25e−12 | 25/25 |

Conditioning degrades ~540x from a 2-clone (0.05 s) to an 11-clone (0.42 s)
window, consistent with consecutive poses over the window being increasingly
near-linearly-dependent. **Rank stays full at every window length** — this is
ill-conditioning with window span, not rank deficiency.

## Cholesky factor vs sqrt(matrix condition)

| block | cond(A) | sqrt(cond A) | cond(C) measured | ratio |
|---|---|---|---|---|
| IMU (15) | 1.01e6 | 1.01e3 | 1.00e3 | 1.00 |
| calib_dt (1) | 1.00 | 1.00 | 1.00 | 1.00 |
| clones only | 6.70e9 | 8.19e4 | 8.17e4 | 1.00 |
| SLAM feats only | 2.21e5 | 4.70e2 | 4.64e2 | 0.99 |
| IMU + feats | 1.43e6 | 1.20e3 | 1.17e3 | 0.98 |
| IMU+clones+feats | 1.34e17 | 3.66e8 | 1.21e10 | **33.13** |

`cond(C) ~ sqrt(cond(A))` holds to within 2 % on every well-conditioned block.
It breaks down (ratio 33) only on `IMU+clones+feats`, which is barely PD at the
2/25 timesteps where Cholesky succeeded at all — the relation assumes a
comfortably positive-definite matrix.

## Log-det numerical viability (statement of fact)

Meaningful only where the matrix is PD with condition number well inside double
precision (~1e16).

**Well-posed** — PD at 25/25 timesteps: `calib_dt` (1.00), `SLAM feats only`
(2.21e5), `IMU (15)` (1.01e6), `IMU + feats` (1.43e6), `clones only` (6.70e9).

**Not viable** — not PD: `FULL joint` (0/25), `IMU + clones` (1/25),
`IMU+clones+feats` (2/25), `FULL minus calib` (2/25).

## Reproduce

```bash
cd analysis
python3 test_eigen_diagnostic.py                  # offline self-test, no hardware
python3 record_joint_covariance.py --seconds 90 --wait-features 25 \
        --out results/joint_cov_moving.pkl        # camera must be MOVING
python3 eigen_diagnostic.py --in results/joint_cov_moving.pkl \
        --outdir results/moving_camera
```

Raw `.pkl` files are gitignored; the report, JSON and figure for each run are
committed under `results/moving_camera/` and `results/static_camera/`.
