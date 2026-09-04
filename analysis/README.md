# Joint-covariance eigenspectrum diagnostic

Analysis only. Nothing here modifies the estimator, the covariance publisher,
or any planner node. The recorder subscribes; the diagnostic reads a file.

## Why

The full joint covariance was previously measured at condition number ~5.6e17
with a minimum eigenvalue of ~2.5e-18 on a healthy converged filter -- i.e.
numerically rank-deficient. That number alone does not say *which subspace*
carries the degeneracy, which is what decides whether a subspace-restricted
D-optimality (log-det) criterion is numerically viable or whether one is
limited to trace / square-root formulations.

These scripts report the numbers. They deliberately draw no conclusion about
which criterion to use.

## Usage

```bash
# 1. record from a healthy run (OpenVINS initialised, features present)
python3 record_joint_covariance.py --seconds 60 --wait-features 25 \
        --out results/joint_cov_run.pkl

# 2. analyse
python3 eigen_diagnostic.py --in results/joint_cov_run.pkl --outdir results

# 3. self-test the analysis on planted data (no ROS, no hardware)
python3 test_eigen_diagnostic.py
```

`--wait-features` holds off the clock until that many SLAM features are
actually in the state, so the recording captures the converged filter rather
than the feature build-up right after initialisation.

## What is computed

Per sampled timestep, using the `StateBlock` metadata on the wire to slice
blocks (offsets are re-read every message -- OpenVINS re-indexes its
covariance on every marginalisation, so cached offsets are always wrong):

1. **Full matrix** -- spectrum, condition number, numerical rank at
   `tol = lambda_max * 1e-12`, rank deficit.
2. **Principal submatrices** -- IMU 15x15, calib_dt 1x1, clones only, SLAM
   features only, IMU+features, IMU+clones, IMU+clones+features, full minus
   calibration.
3. **Null-space localisation** -- near-null eigenvectors of the full matrix
   projected onto each block type, reported as an energy share. This answers
   "where does the degeneracy live" directly, rather than inferring it from
   submatrix conditioning.
4. **Clone-window sweep** -- conditioning of the clone block using the k most
   recent consecutive clones (k = 2,4,6,8,11), ordered by `clone_timestamp`,
   testing whether degeneracy grows with sliding-window length.
5. **Cholesky** -- attempted on every block; where it succeeds, `cond(C)` is
   compared against `sqrt(cond(A))`, the numerical argument for a square-root
   information form.

## Numerical conventions

* Every matrix is symmetrised (`0.5*(A+A.T)`) before decomposition. The wire
  matrix is symmetric to ~1e-18, but `eigh` reads a single triangle, so the
  choice is explicit rather than implicit.
* Condition number is `lambda_max/lambda_min` from `eigvalsh`. When
  `lambda_min <= 0` that ratio is meaningless, so the SVD-based
  `np.linalg.cond` is reported alongside; the two agree for a positive-definite
  matrix and diverge exactly when definiteness has been lost. Table entries
  marked `*` had an infinite condition number at one or more timesteps and are
  averaged over the finite ones only.

## Self-test

`test_eigen_diagnostic.py` builds a synthetic covariance whose near-null
directions are planted entirely inside the clone block and asserts that the
diagnostic (a) finds exactly that many null directions, (b) localises ~100% of
their energy to the clone block, (c) fails Cholesky on the full matrix while
succeeding on the IMU block, and (d) recovers `cond(C)/sqrt(cond(A)) ~ 1`.
It runs offline with no ROS and no hardware.

## Outputs

* `results/eigen_report.txt` -- the tables
* `results/eigen_results.json` -- machine-readable
* `results/eigen_condition_vs_time.png` -- conditioning, eigenvalue extremes,
  and rank vs time across the run
