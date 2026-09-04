#!/usr/bin/env python3
"""Eigenspectrum diagnostic on the OpenVINS joint covariance.

ANALYSIS ONLY. Reads a pickle from record_joint_covariance.py. Reports numbers;
draws no conclusion about which planning criterion to use.

Question it answers: WHICH SUBSPACE carries the near-zero eigenvalues.

Method notes that matter for reading the output:
  * Every matrix is symmetrised (0.5*(A+A.T)) before decomposition. The wire
    matrix is symmetric to ~1e-18, but eigh reads one triangle, so the choice
    is made explicit rather than left to the library.
  * Spectra come from `eigvalsh` (symmetric). Condition number is reported as
    lambda_max/lambda_min. When lambda_min <= 0 that ratio is meaningless, so
    the SVD-based sigma_max/sigma_min from np.linalg.cond is reported too --
    they agree for a PD matrix and diverge exactly when positive-definiteness
    has been lost.
  * Numerical rank counts eigenvalues > lambda_max * RTOL (default 1e-12),
    the same tolerance used to select "near-null" eigenvectors.

Usage:
    python3 eigen_diagnostic.py --in results/joint_cov_run.pkl --outdir results
"""

import argparse
import json
import pickle
from collections import defaultdict

import numpy as np

RTOL = 1e-12


# ----------------------------------------------------------------- helpers
def sym(a):
    return 0.5 * (a + a.T)


def spectrum(a):
    """min/max eigenvalue, condition number, numerical rank, tiny-eig count."""
    a = sym(a)
    n = a.shape[0]
    if n == 0:
        return None
    ev = np.linalg.eigvalsh(a)
    lmax, lmin = float(ev[-1]), float(ev[0])
    tol = abs(lmax) * RTOL
    rank = int((ev > tol).sum())
    cond_eig = float(lmax / lmin) if lmin > 0 else float("inf")
    try:
        cond_svd = float(np.linalg.cond(a))
    except np.linalg.LinAlgError:
        cond_svd = float("nan")
    return dict(
        dim=n, lmin=lmin, lmax=lmax, cond_eig=cond_eig, cond_svd=cond_svd,
        rank=rank, rank_deficit=n - rank, n_below_tol=int((ev <= tol).sum()),
    )


def cholesky_check(a):
    """Try Cholesky. On success also report cond(C) to compare against sqrt(cond(A)).

    The square-root-information argument is that cond(C) ~ sqrt(cond(A)), i.e.
    the factor is representable in far fewer digits than the matrix itself.
    """
    a = sym(a)
    if a.shape[0] == 0:
        return dict(ok=False, reason="empty", cond_factor=None)
    try:
        c = np.linalg.cholesky(a)
    except np.linalg.LinAlgError as e:
        return dict(ok=False, reason=str(e).strip() or "not positive definite",
                    cond_factor=None)
    s = np.linalg.svd(c, compute_uv=False)
    cond_c = float(s[0] / s[-1]) if s[-1] > 0 else float("inf")
    return dict(ok=True, reason="", cond_factor=cond_c)


def blocks_of(sample, *types):
    return [b for b in sample["blocks"] if b["type"] in types]


def idx_of(blocks):
    """Row/col indices covered by a list of blocks."""
    out = []
    for b in blocks:
        out.extend(range(b["index"], b["index"] + b["size"]))
    return np.array(sorted(out), dtype=int)


def submatrix(cov, blocks):
    i = idx_of(blocks)
    if i.size == 0:
        return np.zeros((0, 0))
    return cov[np.ix_(i, i)]


# -------------------------------------------------------------- block sets
def block_sets(sample):
    """Named principal submatrices requested by the diagnostic."""
    imu = blocks_of(sample, "imu")
    dt = blocks_of(sample, "calib_dt")
    clones = blocks_of(sample, "clone")
    feats = blocks_of(sample, "slam_feature")
    calib = blocks_of(sample, "calib_dt", "cam_extrinsics", "cam_intrinsics",
                      "imu_intrinsics")
    return [
        ("FULL joint", sample["blocks"]),
        ("IMU (15)", imu),
        ("calib_dt (1)", dt),
        ("clones only", clones),
        ("SLAM feats only", feats),
        ("IMU + feats", imu + feats),
        ("IMU + clones", imu + clones),
        ("IMU+clones+feats", imu + clones + feats),
        ("FULL minus calib", [b for b in sample["blocks"] if b not in calib]),
    ]


# --------------------------------------------- where does the degeneracy live
def null_energy_by_block(sample):
    """Project near-null eigenvectors onto each block; report energy share.

    This is the direct answer to "where does the degeneracy live", as opposed
    to inferring it from submatrix conditioning. For each eigenvector with
    eigenvalue <= lambda_max*RTOL we compute ||v_block||^2 (the vectors are
    unit norm, so the shares sum to 1) and average over all such vectors.
    """
    cov = sym(sample["cov"])
    ev, evec = np.linalg.eigh(cov)
    tol = abs(ev[-1]) * RTOL
    null = np.where(ev <= tol)[0]
    if null.size == 0:
        return None
    groups = defaultdict(list)
    for b in sample["blocks"]:
        groups[b["type"]].extend(range(b["index"], b["index"] + b["size"]))
    share = {}
    for name, idx in groups.items():
        idx = np.array(idx, dtype=int)
        # energy of each null vector inside this block, averaged over vectors
        e = (evec[idx][:, null] ** 2).sum(axis=0)
        share[name] = dict(mean=float(e.mean()), min=float(e.min()), max=float(e.max()))
    return dict(n_null=int(null.size), dim=int(cov.shape[0]), share=share)


def clone_window_sweep(sample, counts=(2, 4, 6, 8, 11)):
    """Conditioning of the clone block vs how many clones are included.

    Clones are ordered by their timestamp so that "k clones" means the k most
    recent consecutive sliding-window poses, which is what the
    near-linear-dependence hypothesis is actually about.
    """
    clones = sorted(blocks_of(sample, "clone"), key=lambda b: b["clone_timestamp"])
    out = {}
    for k in counts:
        if k > len(clones):
            continue
        sel = clones[-k:]
        s = spectrum(submatrix(sample["cov"], sel))
        ch = cholesky_check(submatrix(sample["cov"], sel))
        s["chol_ok"] = ch["ok"]
        s["cond_factor"] = ch["cond_factor"]
        s["span_s"] = float(sel[-1]["clone_timestamp"] - sel[0]["clone_timestamp"])
        out[k] = s
    return out


# ------------------------------------------------------------------- driver
def analyse(samples, n_steps):
    # well-separated timesteps across the whole run, not just the end
    idx = np.unique(np.linspace(0, len(samples) - 1, n_steps).astype(int))
    per_block = defaultdict(list)
    per_chol = defaultdict(list)
    null_rows = []
    sweep_rows = []
    timeline = []

    for i in idx:
        s = samples[i]
        cov = s["cov"]
        for name, blks in block_sets(s):
            sub = submatrix(cov, blks)
            sp = spectrum(sub)
            if sp is None:
                continue
            per_block[name].append(sp)
            per_chol[name].append(cholesky_check(sub))
        ne = null_energy_by_block(s)
        if ne:
            null_rows.append(ne)
        sweep_rows.append(clone_window_sweep(s))
        full = spectrum(cov)
        timeline.append(dict(
            t=s["stamp"], dim=s["dim"], feats=s["num_slam_features"],
            clones=s["num_clones"], cond=full["cond_eig"], cond_svd=full["cond_svd"],
            lmin=full["lmin"], lmax=full["lmax"], rank=full["rank"],
            deficit=full["rank_deficit"],
        ))
    return idx, per_block, per_chol, null_rows, sweep_rows, timeline


def fmt(x, e=True):
    if x is None:
        return "-"
    if isinstance(x, float) and not np.isfinite(x):
        return "inf"
    return f"{x:.3e}" if e else f"{x}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default="results/joint_cov_run.pkl")
    ap.add_argument("--outdir", default="results")
    ap.add_argument("--steps", type=int, default=25)
    a = ap.parse_args()

    with open(a.inp, "rb") as f:
        samples = pickle.load(f)

    idx, per_block, per_chol, null_rows, sweep_rows, timeline = analyse(samples, a.steps)

    L = []
    def w(s=""):
        L.append(s)
        print(s)

    feats = [s["num_slam_features"] for s in samples]
    dims = [s["dim"] for s in samples]
    poses = [s["pose"] for s in samples if s["pose"]]
    span = samples[-1]["stamp"] - samples[0]["stamp"]

    w("=" * 100)
    w("EIGENSPECTRUM DIAGNOSTIC -- OpenVINS joint covariance")
    w("=" * 100)
    w(f"samples in log      : {len(samples)} over {span:.1f} s")
    w(f"timesteps analysed  : {len(idx)} (evenly spaced across the run)")
    w(f"dim                 : {min(dims)}..{max(dims)}")
    w(f"SLAM features       : {min(feats)}..{max(feats)} (mean {np.mean(feats):.1f})")
    if poses:
        r = [np.linalg.norm(p) for p in poses]
        w(f"|p|                 : {min(r):.3f}..{max(r):.3f} m "
          f"({'BOUNDED' if max(r) < 50 else 'DIVERGED -- NOT a healthy run'})")
    w(f"rank tolerance      : lambda_max * {RTOL:g}")
    w("")

    # ---------------------------------------------------------------- table
    w("-" * 100)
    w("PER-BLOCK SPECTRUM  (mean across sampled timesteps, [min..max] where it varies)")
    w("-" * 100)
    hdr = f"{'block':<20}{'dim':>7}{'cond (eig)':>13}{'rank':>10}{'deficit':>9}{'lambda_min':>13}{'lambda_max':>13}  {'chol':<22}"
    w(hdr)
    w("-" * 100)
    table = {}
    for name in [n for n, _ in block_sets(samples[0])]:
        rows = per_block.get(name, [])
        if not rows:
            continue
        ch = per_chol[name]
        dims_ = [r["dim"] for r in rows]
        conds = [r["cond_eig"] for r in rows]
        ranks = [r["rank"] for r in rows]
        defs = [r["rank_deficit"] for r in rows]
        lmins = [r["lmin"] for r in rows]
        lmaxs = [r["lmax"] for r in rows]
        nok = sum(1 for c in ch if c["ok"])
        cfs = [c["cond_factor"] for c in ch if c["ok"] and c["cond_factor"] is not None]
        finite = [c for c in conds if np.isfinite(c)]
        cond_s = fmt(np.mean(finite)) if finite else "inf"
        if finite and len(finite) < len(conds):
            cond_s += "*"
        dim_s = str(dims_[0]) if len(set(dims_)) == 1 else f"{min(dims_)}-{max(dims_)}"
        rank_s = str(ranks[0]) if len(set(ranks)) == 1 else f"{min(ranks)}-{max(ranks)}"
        def_s = str(defs[0]) if len(set(defs)) == 1 else f"{min(defs)}-{max(defs)}"
        chol_s = f"{nok}/{len(ch)} ok"
        if cfs:
            chol_s += f", cond(C)~{np.mean(cfs):.2e}"
        w(f"{name:<20}{dim_s:>7}{cond_s:>13}{rank_s:>10}{def_s:>9}"
          f"{fmt(np.mean(lmins)):>13}{fmt(np.mean(lmaxs)):>13}  {chol_s:<22}")
        table[name] = dict(
            dim=dim_s, cond_mean=(float(np.mean(finite)) if finite else None),
            cond_min=(float(min(finite)) if finite else None),
            cond_max=(float(max(finite)) if finite else None),
            rank=rank_s, deficit=def_s,
            lmin_mean=float(np.mean(lmins)), lmax_mean=float(np.mean(lmaxs)),
            chol_ok=nok, chol_n=len(ch),
            chol_fail_reason=next((c["reason"] for c in ch if not c["ok"]), ""),
            cond_factor_mean=(float(np.mean(cfs)) if cfs else None),
        )
    w("  * = condition number infinite (lambda_min <= 0) at some timesteps; mean over finite ones only")
    w("")

    # sqrt(cond) check
    w("-" * 100)
    w("CHOLESKY FACTOR vs sqrt(matrix condition)   [square-root-form numerical argument]")
    w("-" * 100)
    w(f"{'block':<20}{'cond(A)':>14}{'sqrt(cond A)':>16}{'cond(C) meas':>16}{'ratio':>10}")
    for name, t in table.items():
        if t["cond_factor_mean"] and t["cond_mean"]:
            sq = np.sqrt(t["cond_mean"])
            w(f"{name:<20}{t['cond_mean']:>14.3e}{sq:>16.3e}"
              f"{t['cond_factor_mean']:>16.3e}{t['cond_factor_mean']/sq:>10.2f}")
    w("")

    # ------------------------------------------------- null-space localisation
    w("-" * 100)
    w("WHERE THE DEGENERACY LIVES -- energy of near-null eigenvectors per block type")
    w("-" * 100)
    if null_rows:
        w(f"near-null directions: {np.mean([r['n_null'] for r in null_rows]):.1f} of "
          f"{np.mean([r['dim'] for r in null_rows]):.0f} dims (mean over timesteps)")
        w(f"{'block type':<22}{'mean energy share':>20}{'min':>12}{'max':>12}")
        keys = sorted({k for r in null_rows for k in r["share"]})
        for k in keys:
            m = [r["share"][k]["mean"] for r in null_rows if k in r["share"]]
            lo = [r["share"][k]["min"] for r in null_rows if k in r["share"]]
            hi = [r["share"][k]["max"] for r in null_rows if k in r["share"]]
            w(f"{k:<22}{np.mean(m):>19.1%}{np.mean(lo):>12.1%}{np.mean(hi):>12.1%}")
        w("(shares sum to 1 across block types; eigenvectors are unit norm)")
    else:
        w("no eigenvalues below tolerance at any sampled timestep")
    w("")

    # -------------------------------------------------- clone window sweep
    w("-" * 100)
    w("CLONE BLOCK CONDITIONING vs SLIDING-WINDOW LENGTH")
    w("-" * 100)
    w(f"{'#clones':>8}{'dim':>7}{'span_s':>9}{'cond':>14}{'rank':>7}{'deficit':>9}{'lambda_min':>13}{'chol':>7}")
    ks = sorted({k for r in sweep_rows for k in r})
    sweep_out = {}
    for k in ks:
        rows = [r[k] for r in sweep_rows if k in r]
        if not rows:
            continue
        cf = [r["cond_eig"] for r in rows if np.isfinite(r["cond_eig"])]
        w(f"{k:>8}{rows[0]['dim']:>7}{np.mean([r['span_s'] for r in rows]):>9.2f}"
          f"{(np.mean(cf) if cf else float('inf')):>14.3e}"
          f"{int(np.mean([r['rank'] for r in rows])):>7}"
          f"{int(np.mean([r['rank_deficit'] for r in rows])):>9}"
          f"{np.mean([r['lmin'] for r in rows]):>13.3e}"
          f"{sum(1 for r in rows if r['chol_ok']):>4}/{len(rows):<3}")
        sweep_out[k] = dict(dim=rows[0]["dim"],
                            span_s=float(np.mean([r["span_s"] for r in rows])),
                            cond=(float(np.mean(cf)) if cf else None),
                            rank=float(np.mean([r["rank"] for r in rows])),
                            lmin=float(np.mean([r["lmin"] for r in rows])),
                            chol_ok=sum(1 for r in rows if r["chol_ok"]), n=len(rows))
    w("")

    # ------------------------------------------------------------ timeline
    w("-" * 100)
    w("FULL-MATRIX CONDITIONING OVER THE RUN")
    w("-" * 100)
    w(f"{'t_rel_s':>9}{'dim':>6}{'feats':>7}{'clones':>8}{'cond':>13}{'rank':>7}{'deficit':>9}{'lambda_min':>13}")
    t0 = timeline[0]["t"]
    for r in timeline:
        w(f"{r['t']-t0:>9.1f}{r['dim']:>6}{r['feats']:>7}{r['clones']:>8}"
          f"{r['cond']:>13.3e}{r['rank']:>7}{r['deficit']:>9}{r['lmin']:>13.3e}")
    w("")

    # ---------------------------------------------------------- log-det note
    w("-" * 100)
    w("LOG-DET NUMERICAL VIABILITY (statement of fact, not a recommendation)")
    w("-" * 100)
    w("A log-det is numerically meaningful only where the matrix is positive definite")
    w("with a condition number well inside double precision (~1e16).")
    for name, t in table.items():
        pd = t["chol_ok"] == t["chol_n"] and t["chol_n"] > 0
        c = t["cond_mean"]
        if pd and c is not None and c < 1e12:
            verdict = "OK      log-det well-posed"
        elif pd and c is not None:
            verdict = "MARGINAL PD but cond > 1e12"
        else:
            verdict = "NO      not PD at all sampled timesteps"
        cs = f"{c:.2e}" if c else "inf"
        w(f"  {name:<20} cond={cs:>11}  chol {t['chol_ok']}/{t['chol_n']}  -> {verdict}")
    w("=" * 100)

    with open(f"{a.outdir}/eigen_report.txt", "w") as f:
        f.write("\n".join(L) + "\n")
    with open(f"{a.outdir}/eigen_results.json", "w") as f:
        json.dump(dict(table=table, timeline=timeline, clone_sweep=sweep_out,
                       null_energy=null_rows, rtol=RTOL,
                       n_samples=len(samples), n_analysed=len(idx)), f, indent=2)

    # ------------------------------------------------------------- plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [r["t"] - t0 for r in timeline]
        fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
        ax[0].semilogy(t, [r["cond"] for r in timeline], "o-", color="#c0392b", label="cond (eig)")
        ax[0].semilogy(t, [r["cond_svd"] for r in timeline], "s--", ms=3, color="#7f8c8d", label="cond (SVD)")
        ax[0].axhline(1e16, ls=":", c="k", lw=1, label="double precision ~1e16")
        ax[0].set_ylabel("condition number")
        ax[0].set_title("Full joint covariance conditioning over the run")
        ax[0].legend(fontsize=8); ax[0].grid(alpha=.3, which="both")
        ax[1].semilogy(t, [abs(r["lmin"]) for r in timeline], "o-", color="#2980b9", label="|lambda_min|")
        ax[1].semilogy(t, [r["lmax"] for r in timeline], "o-", color="#27ae60", label="lambda_max")
        ax[1].set_ylabel("eigenvalue"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3, which="both")
        ax[2].plot(t, [r["dim"] for r in timeline], "o-", label="dim")
        ax[2].plot(t, [r["rank"] for r in timeline], "s-", label="numerical rank")
        ax[2].plot(t, [r["deficit"] for r in timeline], "^-", label="rank deficit")
        ax[2].plot(t, [r["feats"] for r in timeline], ".--", label="SLAM features")
        ax[2].set_xlabel("time since first sample (s)"); ax[2].set_ylabel("count")
        ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
        fig.tight_layout()
        fig.savefig(f"{a.outdir}/eigen_condition_vs_time.png", dpi=130)
        print(f"\nplot -> {a.outdir}/eigen_condition_vs_time.png")
    except Exception as e:
        print(f"plot skipped: {e}")

    print(f"report -> {a.outdir}/eigen_report.txt")
    print(f"json   -> {a.outdir}/eigen_results.json")


if __name__ == "__main__":
    main()
