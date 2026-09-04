# Eigenspectrum diagnostic — results

Run recorded 2026-09-04 on prometheus (Jetson Orin NX), D455 stereo + IMU,
`open_vins` branch `openvins-integration`, config `rs_d455`.

Numbers only. No conclusion is drawn here about which planning criterion to use.

## Run conditions

| | |
|---|---|
| samples | 292 over 60.4 s (5 Hz publisher) |
| timesteps analysed | 25, evenly spaced across the run |
| dim | 85 … 217 |
| SLAM features | 1 … 45 (**mean 4.9**) |
| \|p\| | 0.014 … 0.052 m — **bounded** |
| IR rate | 30.0 Hz (header period 33.3 ms) |
| OpenVINS lag | ~1.2 ms (real-time, not CPU-starved) |
| rank tolerance | `lambda_max * 1e-12` |

**Caveat on feature count.** The recording armed at 45 SLAM features but the
camera was static for most of the 60 s, and OpenVINS marginalises SLAM features
it cannot triangulate without parallax, so the population fell to 2 and the
mean over the run is 4.9 rather than the 25+ requested. The feature-dependent
rows (`SLAM feats only`, `IMU + feats`) are therefore computed on a small block
for most timesteps. **The headline result is unaffected** — the rank deficit is
exactly 6 at every timestep from dim 88 to dim 205 and from 2 features to 41,
i.e. it is independent of how many features are in the state. A re-record while
the camera is moving would tighten the feature-block numbers only.

## Per-block spectrum (mean over 25 timesteps)

| block | dim | cond (eig) | rank | deficit | lambda_min | Cholesky |
|---|---|---|---|---|---|---|
| FULL joint | 88–205 | **inf** | 82–199 | **6** | −3.67e−18 | **0/25** |
| IMU (15) | 15 | 9.07e5 | 15 | 0 | 3.21e−09 | 25/25, cond(C) 9.41e2 |
| calib_dt (1) | 1 | 1.00 | 1 | 0 | 1.68e−08 | 25/25, cond(C) 1.00 |
| clones only | 66 | 6.34e9 | 66 | 0 | 4.62e−12 | 25/25, cond(C) 7.92e4 |
| SLAM feats only | 6–123 | 9.00e4 | 6–123 | 0 | 7.86e−08 | 25/25, cond(C) 2.12e2 |
| IMU + feats | 21–138 | 1.93e6 | 21–138 | 0 | 3.21e−09 | 25/25, cond(C) 1.20e3 |
| IMU + clones | 81 | **inf** | 75–76 | **5–6** | −3.03e−18 | 3/25 |
| IMU+clones+feats | 87–204 | **inf** | 81–199 | **5–6** | −3.21e−18 | 5/25 |
| FULL minus calib | 87–204 | **inf** | 81–199 | **5–6** | −3.21e−18 | 5/25 |

Full-matrix SVD condition number ranged **1.7e17 … 1.6e20** across the run
(consistent with the previously measured 5.6e17). `lambda_min` is *negative*,
around −1e−18, so the eigenvalue ratio is meaningless and the matrix is not
positive semidefinite to working precision.

## Where the degeneracy lives

Near-null eigenvectors (6.0 of ~99 dims), energy by block type:

| block type | mean share |
|---|---|
| clone | 49.9 % |
| imu | 49.9 % |
| calib_dt | 0.2 % |
| slam_feature | **0.0 %** |

Resolving the imu/clone split further:

| component | share of null energy |
|---|---|
| IMU pose (quat, pos) | 49.92 % |
| IMU vel / gyro-bias / accel-bias | 0.00 % |
| newest clone | 49.92 % |
| all older clones | 0.00 % |
| **IMU pose + newest clone** | **99.85 %** |

`<v_imu_pose, v_newest_clone> = −0.4992` and the newest clone's age is
`+0.0000 s`.

An inner product of exactly −0.5 between two unit-norm 6-vectors is the
signature of the pairing `v = (u, −u)/sqrt(2)`. Combined with a clone age of
zero, this says the six near-null directions **are the difference between the
IMU pose and the most recent clone** — which OpenVINS creates as a bit-exact
copy of the current IMU pose. The degeneracy is structural and exactly
predictable, not an estimation pathology, and it does not involve the
velocity/bias states, the older clones, or the map at all.

## Clone conditioning vs sliding-window length

| #clones | dim | span (s) | cond | rank | deficit | lambda_min | Cholesky |
|---|---|---|---|---|---|---|---|
| 2 | 12 | 0.06 | 6.15e6 | 12 | 0 | 1.17e−09 | 25/25 |
| 4 | 24 | 0.14 | 8.81e8 | 24 | 0 | 1.49e−11 | 25/25 |
| 6 | 36 | 0.22 | 2.47e9 | 36 | 0 | 6.61e−12 | 25/25 |
| 8 | 48 | 0.30 | 4.08e9 | 48 | 0 | 5.26e−12 | 25/25 |
| 11 | 66 | 0.43 | 6.34e9 | 66 | 0 | 4.62e−12 | 25/25 |

Conditioning degrades ~1000x from a 2-clone to an 11-clone window (0.06 s to
0.43 s), consistent with consecutive poses over the window being increasingly
near-linearly-dependent. **Rank stays full at every window length** — this is
ill-conditioning, not rank deficiency.

## Cholesky factor vs sqrt(matrix condition)

| block | cond(A) | sqrt(cond A) | cond(C) measured | ratio |
|---|---|---|---|---|
| IMU (15) | 9.07e5 | 9.52e2 | 9.41e2 | 0.99 |
| calib_dt (1) | 1.00 | 1.00 | 1.00 | 1.00 |
| clones only | 6.34e9 | 7.96e4 | 7.92e4 | 0.99 |
| SLAM feats only | 9.00e4 | 3.00e2 | 2.12e2 | 0.71 |
| IMU + feats | 1.93e6 | 1.39e3 | 1.20e3 | 0.86 |

`cond(C) ~ sqrt(cond(A))` holds on every block where Cholesky succeeds.

## Log-det numerical viability (statement of fact)

A log-det is numerically meaningful only where the matrix is positive definite
with a condition number well inside double precision (~1e16).

**Well-posed** (PD at 25/25 timesteps):

| block | cond |
|---|---|
| IMU (15) | 9.07e5 |
| calib_dt (1) | 1.00 |
| SLAM feats only | 9.00e4 |
| IMU + feats | 1.93e6 |
| clones only | 6.34e9 |

**Not viable** (not PD; Cholesky fails at most or all timesteps):

| block | Cholesky |
|---|---|
| FULL joint | 0/25 |
| IMU + clones | 3/25 |
| IMU + clones + feats | 5/25 |
| FULL minus calib | 5/25 |

Every failing set is exactly a set that contains **both** the IMU block and the
clone block. Every succeeding set contains at most one of them.

## Reproduce

```bash
cd analysis
python3 test_eigen_diagnostic.py                 # offline self-test, no hardware
python3 record_joint_covariance.py --seconds 60 --wait-features 25 \
        --out results/joint_cov_run.pkl
python3 eigen_diagnostic.py --in results/joint_cov_run.pkl --outdir results
```

Raw `.pkl` (23 MB) is gitignored; `eigen_report.txt`, `eigen_results.json` and
`eigen_condition_vs_time.png` are committed.
