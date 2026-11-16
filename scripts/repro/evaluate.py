"""One evaluator for every method: xrtslam-metrics (the paper's / branch CI's ATE code).

For each trajectory it reports, on the same GT and the same time association
(xrtslam-metrics ``get_sanitized_trajectories``: overlap trim + evo
``associate_trajectories``):

- ``ate_se3_cm``: RMSE ATE after SE(3) Umeyama alignment, evo ``main_ape.ape``
  exactly as ``tracking.py ate`` calls it (the paper's metric);
- ``atec_cm``: the C++ ``atec`` metric the CI report (``batch.py``) uses, as a
  cross-check of the SE(3) number;
- ``ate_sim3_cm``: the same evo call with ``correct_scale=True``;
- ``rte_cm``: RMSE RTE with delta = 6 frames, like ``tracking.py rte``;
- ``completion``: tracked fraction of the GT time span.

Usage:
    python scripts/repro/evaluate.py <run_dir> [<run_dir> ...]
    python scripts/repro/evaluate.py --gt GT.csv --est name=path.csv ... --out out.json
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import warnings
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
XRTMET = ROOT / "thirdparty" / "xrtslam-metrics"
sys.path[:0] = [str(XRTMET), str(XRTMET / "cpp")]

import numpy as np  # noqa: E402
from evo.core.metrics import PoseRelation, Unit  # noqa: E402
from evo.main_ape import ape  # noqa: E402
from evo.main_rpe import rpe  # noqa: E402

from alignment import compute_ates  # noqa: E402  (xrtslam-metrics C++ module)
from tracking import get_sanitized_trajectories  # noqa: E402

DATA = Path(os.environ.get("DATA_ROOT", ROOT / "datasets"))  # same layout as run_sequence.sh
DEVICE_ROOTS = {
    "euroc": DATA / "euroc",
    "tumvi": DATA / "tum-vi",
    "msdmg": DATA / "monado-slam-datasets/M_monado_datasets/MG_reverb_g2",
    "msdmi": DATA / "monado-slam-datasets/M_monado_datasets/MI_valve_index",
    "msdmo": DATA / "monado-slam-datasets/M_monado_datasets/MO_odyssey_plus",
}
GT_RELPATHS = ["mav0/gt/data.csv", "mav0/state_groundtruth_estimate0/data.csv", "mav0/mocap0/data.csv"]
# Trajectory files written by run_sequence.sh, in table order.
METHODS = {
    "bsl": "Basalt VIO (BSL, causal)",
    "lc_rt": "Basalt + LC, real-time output (causal)",
    "lc": "+LC (keyframes after PGO, per frame)",
    "ba": "+GBA (rootba float, no pose graph)",
    "bapgd": "RootSLAM, GBA double",
    "bapgf": "RootSLAM (paper RS: GBA float + pose graph)",
}


@dataclass
class Score:
    """Metrics for one trajectory against one GT."""

    ate_se3_cm: float
    atec_cm: float
    ate_sim3_cm: float
    sim3_scale: float
    rte_cm: float
    completion: float
    n_matched: int


def gt_path_for(seq: str) -> Path:
    """GT csv of an evaluation.json sequence key, from the raw dataset under DATA_ROOT."""
    ev = json.loads((ROOT / ".ci" / "evaluation.json").read_text())
    s = ev["sequences"][seq]
    base = DEVICE_ROOTS[s["device"]] / s["path"]
    for rel in GT_RELPATHS:
        if (base / rel).exists():
            return base / rel
    raise FileNotFoundError(f"no GT under {base}")


def score(est_csv: Path, gt_csv: Path) -> Score:
    """Score one trajectory with xrtslam-metrics association and evo metrics."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        traj_est, traj_ref = get_sanitized_trajectories(est_csv, gt_csv, silence=True)
    n = traj_est.num_poses
    # Completion: estimate's time span overlapping GT / GT time span.
    gt_ts = np.loadtxt(gt_csv, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    est_ts = np.loadtxt(est_csv, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    ov = min(est_ts[-1], gt_ts[-1]) - max(est_ts[0], gt_ts[0])
    completion = float(max(ov, 0) / (gt_ts[-1] - gt_ts[0]))

    common = dict(pose_relation=PoseRelation.translation_part, align=True, n_to_align=-1, align_origin=False)
    r_se3 = ape(traj_ref=traj_ref, traj_est=copy.deepcopy(traj_est), correct_scale=False, **common)
    # Sim(3): same Umeyama call evo's ape makes, keeping the scale it returns.
    est_sim3 = copy.deepcopy(traj_est)
    _, _, scale = est_sim3.align(traj_ref, correct_scale=True)
    ate_sim3 = float(np.sqrt(np.mean(np.sum((est_sim3.positions_xyz - traj_ref.positions_xyz) ** 2, axis=1))))
    r_rte = rpe(
        traj_ref=traj_ref,
        traj_est=copy.deepcopy(traj_est),
        pose_relation=PoseRelation.translation_part,
        delta=6,
        delta_unit=Unit.frames,
        rel_delta_tol=0.1,
        all_pairs=False,
        align=True,
        correct_scale=False,
        n_to_align=-1,
        align_origin=False,
        support_loop=False,
    )
    atec = float(compute_ates(gt_csv, [est_csv])[0])
    return Score(
        ate_se3_cm=100 * r_se3.stats["rmse"],
        atec_cm=100 * atec,
        ate_sim3_cm=100 * ate_sim3,
        sim3_scale=float(scale),
        rte_cm=100 * r_rte.stats["rmse"],
        completion=completion,
        n_matched=n,
    )


def evaluate_run(run_dir: Path) -> dict:
    """Score every method trajectory in <run_dir>/eval and write eval/metrics.json."""
    seq = run_dir.name
    gt = gt_path_for(seq)
    out: dict = {"seq": seq, "gt": str(gt), "methods": {}}
    for key in METHODS:
        csv = run_dir / "eval" / f"{key}.csv"
        if not csv.exists():
            continue
        try:
            out["methods"][key] = asdict(score(csv, gt))
        except Exception as e:  # noqa: BLE001 - record the failure in the table
            out["methods"][key] = {"error": repr(e)}
    (run_dir / "eval" / "metrics.json").write_text(json.dumps(out, indent=2))
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dirs", nargs="*", type=Path)
    p.add_argument("--gt", type=Path)
    p.add_argument("--est", action="append", default=[], help="name=path.csv")
    p.add_argument("--out", type=Path)
    a = p.parse_args()
    if a.gt:
        res = {}
        for item in a.est:
            name, path = item.split("=", 1)
            try:
                res[name] = asdict(score(Path(path), a.gt))
            except Exception as e:  # noqa: BLE001
                res[name] = {"error": repr(e)}
        txt = json.dumps({"gt": str(a.gt), "methods": res}, indent=2)
        if a.out:
            a.out.write_text(txt)
        print(txt)
        return
    for d in a.run_dirs:
        r = evaluate_run(d)
        for k, m in r["methods"].items():
            if "error" in m:
                print(f"{r['seq']:8s} {k:7s} ERROR {m['error']}")
            else:
                print(
                    f"{r['seq']:8s} {k:7s} ATE se3 {m['ate_se3_cm']:7.2f}  atec {m['atec_cm']:7.2f}  "
                    f"sim3 {m['ate_sim3_cm']:7.2f} (s={m['sim3_scale']:.4f})  RTE {m['rte_cm']:5.2f}  "
                    f"compl {m['completion']:.3f}"
                )


if __name__ == "__main__":
    main()
