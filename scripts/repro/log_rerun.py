"""Log the real intermediates of one RootSLAM run to a Rerun 0.42 recording.

Reads what run_sequence.sh wrote (no re-implementation of the algorithm):

- frontend: the keyframe observations of map/map.json on the keyframe images of all
  cameras, with each landmark's track over the previous keyframes;
- per-frame timing: timing.tracking.<seq>.csv (frontend and backend ms per frame);
- VIO keyframes: map.json keyframe poses + Pinhole frusta, and the 7 newest keyframes
  (vio_max_kfs) as the sliding window;
- persistent map: landmarks, and the landmarks each keyframe adds (first observation);
- loop detections: loop_closure edges of map/pose_graph.json, at the time of
  the newer keyframe, with both keyframe images;
- pose graph before / after optimisation: real-time VIO poses at the keyframe
  timestamps (before) vs map/kf_trajectory.csv (after online PGO), with the
  spanning-tree, covisibility and loop edges;
- global BA: ba_log.<variant>.json (cost, iteration time) for the float32 and
  float64 runs, and the keyframe shift between online PGO and global BA;
- final trajectories vs GT: eval/*.csv, each SE(3)-aligned to GT with evo, the
  per-frame position error, and (optional, --compare-dir) the slam_rs / colsfm
  trajectories scored by the same evaluator.

With --blueprint-dir it also writes one .rbl per page figure and figures.json
(timeline + time per figure) for the page's screenshot renderer.

Usage:
    python scripts/repro/log_rerun.py --run-dir runs/MGO07 --out rootslam-MGO07.rrd \
        [--compare-dir <dir with slam_rs.csv / colsfm.csv>] [--blueprint-dir figs/]
"""

from __future__ import annotations

import argparse
import copy
import io
import json
import sys
from collections import defaultdict
from functools import lru_cache
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from evo.core import sync
from evo.tools import file_interface
from PIL import Image
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "repro"))
from evaluate import DEVICE_ROOTS, gt_path_for  # noqa: E402

METHOD_COLORS = {
    "gt": (200, 200, 200),
    "bsl": (230, 85, 70),
    "lc": (240, 180, 60),
    "ba": (120, 170, 250),
    "bapgf": (80, 220, 140),
    "slam_rs": (222, 125, 243),
    "colsfm": (34, 181, 230),
}
METHOD_LABELS = {
    "gt": "ground truth",
    "bsl": "Basalt VIO (BSL)",
    "lc": "+ loop closing (online PGO)",
    "ba": "+ global BA, no pose graph",
    "bapgf": "RootSLAM (GBA float32 + pose graph)",
    "slam_rs": "slam-rs (online VIO)",
    "colsfm": "colsfm (offline, refines slam-rs)",
}
VIO_MAX_KFS = 7  # vio_max_kfs in data/msd/msdmg_config.json
GBA_SHIFT_SCALE = 10.0


def se3(q_wxyz: np.ndarray, t: np.ndarray) -> np.ndarray:
    """4x4 matrix from a wxyz quaternion and a translation."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()
    T[:3, 3] = t
    return T


def load_csv_poses(path: Path) -> dict[int, np.ndarray]:
    """EuRoC trajectory csv -> {t_ns: 4x4}."""
    a = np.loadtxt(path, delimiter=",", comments="#")
    a = np.atleast_2d(a)
    return {int(r[0]): se3(r[4:8], r[1:4]) for r in a}


def umeyama_se3(est_csv: Path, gt_csv: Path) -> np.ndarray:
    """SE(3) alignment est->GT, with the evaluator's association (xrtslam-metrics / evo)."""
    ref = file_interface.read_euroc_csv_trajectory(gt_csv)
    est = file_interface.read_euroc_csv_trajectory(est_csv)
    t0, t1 = max(est.timestamps[0], ref.timestamps[0]), min(est.timestamps[-1], ref.timestamps[-1])
    ref.reduce_to_time_range(t0, t1)
    est.reduce_to_time_range(t0, t1)
    ref, est = sync.associate_trajectories(ref, est)
    r, t, _ = copy.deepcopy(est).align(ref, correct_scale=False)
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def apply(T: np.ndarray, poses: dict[int, np.ndarray]) -> dict[int, np.ndarray]:
    return {k: T @ v for k, v in poses.items()}


def centres(poses: dict[int, np.ndarray], keys=None) -> np.ndarray:
    keys = sorted(poses) if keys is None else keys
    return np.array([poses[k][:3, 3] for k in keys])


def nearest_gt(gt_keys: np.ndarray, gt_xyz: np.ndarray, t_ns: np.ndarray, max_dt_ns: int = 10_000_000) -> tuple[np.ndarray, np.ndarray]:
    """GT position nearest in time to each t_ns, and a mask of matches within max_dt_ns."""
    i = np.clip(np.searchsorted(gt_keys, t_ns), 1, len(gt_keys) - 1)
    left_closer = (t_ns - gt_keys[i - 1]) < (gt_keys[i] - t_ns)
    j = np.where(left_closer, i - 1, i)
    return gt_xyz[j], np.abs(gt_keys[j] - t_ns) <= max_dt_ns


def error_colors(err_cm: np.ndarray, hi_cm: float) -> np.ndarray:
    """Blue (0) -> yellow -> red (hi_cm) for per-frame position error."""
    x = np.clip(err_cm / max(hi_cm, 1e-6), 0, 1)
    r = np.where(x < 0.5, 60 + 2 * x * 190, 250)
    g = np.where(x < 0.5, 130 + 2 * x * 100, 230 - (x - 0.5) * 2 * 170)
    b = np.where(x < 0.5, 240 - 2 * x * 180, 60 - (x - 0.5) * 2 * 20)
    return np.stack([r, g, b], axis=1).astype(np.uint8)


@lru_cache(maxsize=256)
def jpeg(path: Path, quality: int = 75) -> rr.EncodedImage:
    """Keyframe image as JPEG (the PNGs would push the .rrd past the 300 MB page budget)."""
    buf = io.BytesIO()
    Image.open(path).convert("L").save(buf, format="JPEG", quality=quality)
    return rr.EncodedImage(contents=buf.getvalue(), media_type="image/jpeg")


def log_transform(path: str, T: np.ndarray) -> None:
    q = Rotation.from_matrix(T[:3, :3]).as_quat()  # xyzw
    rr.log(path, rr.Transform3D(translation=T[:3, 3], quaternion=rr.Quaternion(xyzw=q)))


def image_index(ds: Path, cam: int) -> tuple[dict[int, Path], np.ndarray]:
    idx = {}
    with open(ds / "mav0" / f"cam{cam}" / "data.csv") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            t, fn = line.strip().split(",")[:2]
            idx[int(t)] = ds / "mav0" / f"cam{cam}" / "data" / fn.strip()
    return idx, np.array(sorted(idx))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--kf-image-stride", type=int, default=1, help="log every Nth keyframe image")
    ap.add_argument("--track-len", type=int, default=6, help="keyframes of history per landmark track")
    ap.add_argument("--compare-dir", type=Path, default=None, help="dir with slam_rs.csv / colsfm.csv (+ scores.json) to add as trajectories")
    ap.add_argument("--blueprint-dir", type=Path, default=None, help="write one .rbl per page figure + figures.json here")
    ap.add_argument("--map-stride", type=int, default=3, help="keyframes between cumulative map snapshots")
    ap.add_argument("--marker-stride", type=int, default=3, help="frames between moving markers on the time timeline")
    a = ap.parse_args()

    run = a.run_dir
    seq = run.name
    app_id = f"rootslam_{seq}"
    ev = json.loads((ROOT / ".ci" / "evaluation.json").read_text())
    s = ev["sequences"][seq]
    ds = DEVICE_ROOTS[s["device"]] / s["path"]
    calib = json.loads((ROOT / ev["calibs"][s["device"]]).read_text())["value0"]
    gt_csv = gt_path_for(seq)
    n_cams = len(calib["intrinsics"])

    rr.init(app_id, recording_id=f"rootslam-{seq}")
    facts: dict = {}
    rr.save(a.out)

    # --- trajectories, aligned to GT -------------------------------------------------
    gt = load_csv_poses(gt_csv)
    t_start = min(gt)
    gt_keys = np.array(sorted(gt))
    gt_xyz = centres(gt, list(gt_keys))
    T_align = umeyama_se3(run / "eval" / "bapgf.csv", gt_csv)  # map / graph / keyframes use RootSLAM's alignment
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/final/gt", rr.LineStrips3D([gt_xyz], colors=[METHOD_COLORS["gt"]], radii=0.004, labels=[METHOD_LABELS["gt"]], show_labels=False), static=True)
    metrics = json.loads((run / "eval" / "metrics.json").read_text())["methods"] if (run / "eval" / "metrics.json").exists() else {}

    sources: dict[str, tuple[Path, float | None]] = {}
    for key in ["bsl", "lc", "ba", "bapgf"]:
        csv = run / "eval" / f"{key}.csv"
        if csv.exists():
            sources[key] = (csv, metrics.get(key, {}).get("ate_se3_cm"))
    compare_keys: list[str] = []
    if a.compare_dir is not None:
        scores_f = a.compare_dir / "scores.json"
        scores = json.loads(scores_f.read_text())["methods"] if scores_f.exists() else {}
        for key in ["slam_rs", "colsfm"]:
            csv = a.compare_dir / f"{key}.csv"
            if csv.exists():
                sources[key] = (csv, scores.get(key, {}).get("ate_se3_cm"))
                compare_keys.append(key)

    aligned: dict[str, dict[int, np.ndarray]] = {}
    for key, (csv, ate) in sources.items():
        P = apply(umeyama_se3(csv, gt_csv), load_csv_poses(csv))
        aligned[key] = P
        label = METHOD_LABELS[key] + (f"  ATE {ate:.2f} cm" if ate is not None else "")
        entity = f"world/compare/{key}" if key in compare_keys else f"world/final/{key}"
        rr.log(entity, rr.LineStrips3D([centres(P)], colors=[METHOD_COLORS[key]], radii=0.004, labels=[label], show_labels=False), static=True)

    # Per-frame position error after alignment, and moving markers on the `time` timeline.
    errors: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for key, P in aligned.items():
        keys = np.array(sorted(P))
        xyz = centres(P, list(keys))
        g, ok = nearest_gt(gt_keys, gt_xyz, keys)
        errors[key] = (keys[ok], xyz[ok], np.linalg.norm(xyz[ok] - g[ok], axis=1) * 100.0)
    if "bapgf" in errors:
        keys, xyz, err = errors["bapgf"]
        hi = float(np.percentile(err, 98))
        cols = error_colors(err, hi)
        segs = [xyz[i:i + 2] for i in range(len(xyz) - 1)]
        rr.log("world/eval/rootslam_error", rr.LineStrips3D(segs, colors=cols[:-1], radii=0.006), static=True)
        rr.log("world/eval/rootslam_error_scale", rr.TextDocument(f"Colour scale: blue 0 cm -> yellow {hi / 2:.1f} cm -> red {hi:.1f} cm (98th percentile)"), static=True)
    for key, (keys, xyz, err) in errors.items():
        base = "compare" if key in compare_keys else "final"
        for i in range(0, len(keys), max(1, a.marker_stride)):
            rr.set_time("time", duration=(int(keys[i]) - t_start) * 1e-9)
            rr.log(f"eval/position_error_cm/{key}", rr.Scalars(float(err[i])))
            rr.log(f"world/{base}_now/{key}", rr.Points3D([xyz[i]], colors=[METHOD_COLORS[key]], radii=0.02))
    for i in range(0, len(gt_keys), 30):  # GT is 1 kHz; markers at ~33 Hz
        rr.set_time("time", duration=(int(gt_keys[i]) - t_start) * 1e-9)
        for base in ["final", "compare"]:
            rr.log(f"world/{base}_now/gt", rr.Points3D([gt_xyz[i]], colors=[(245, 245, 245)], radii=0.02))
    rr.reset_time()
    for key in errors:
        rr.log(f"eval/position_error_cm/{key}", rr.SeriesLines(colors=[METHOD_COLORS[key]], names=[METHOD_LABELS[key]], widths=[1.5]), static=True)

    # --- per-frame timing (frontend / backend) -----------------------------------------
    timing_csv = run / f"timing.tracking.{seq}.csv"
    if timing_csv.exists():
        with open(timing_csv) as f:
            header = f.readline().lstrip("#").strip().split(",")
        tm = np.atleast_2d(np.loadtxt(timing_csv, delimiter=",", comments="#", dtype=np.int64))
        c = {n: i for i, n in enumerate(header)}
        fe = (tm[:, c["frontend_keypoints_pushed"]] - tm[:, c["frontend_frames_received"]]) * 1e-6
        be = (tm[:, c["backend_marginalization_ended"]] - tm[:, c["backend_keypoints_received"]]) * 1e-6
        for i in range(len(tm)):
            rr.set_time("time", duration=(int(tm[i, 0]) - t_start) * 1e-9)
            rr.log("timing/frontend_ms", rr.Scalars(float(fe[i])))
            rr.log("timing/backend_ms", rr.Scalars(float(be[i])))
        rr.reset_time()
        rr.log("timing/frontend_ms", rr.SeriesLines(colors=[(34, 181, 230)], names=["frontend (optical flow) ms"]), static=True)
        rr.log("timing/backend_ms", rr.SeriesLines(colors=[(246, 218, 117)], names=["backend (VIO window solve + marg.) ms"]), static=True)

    # --- map: keyframes, landmarks, observations --------------------------------------
    m = json.loads((run / "map" / "map.json").read_text())
    kf_after = {int(k["id"]): se3(np.array(k["T_w_i"][:4]), np.array(k["T_w_i"][4:])) for k in m["keyframes"]}
    kf_after = apply(T_align, kf_after)
    kf_ids = sorted(kf_after)
    lm_ids = [int(l["id"]) for l in m["landmarks"]] if "id" in m["landmarks"][0] else list(range(len(m["landmarks"])))
    lm_xyz = np.array([l["p_w"] for l in m["landmarks"]])
    lm_xyz = (T_align[:3, :3] @ lm_xyz.T).T + T_align[:3, 3]
    z = lm_xyz[:, 2]
    lo, hi = np.percentile(z, [2, 98])
    tz = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
    lm_col = np.stack([60 + 160 * tz, 120 + 60 * (1 - tz), 230 - 150 * tz], axis=1).astype(np.uint8)
    rr.log("world/map/landmarks", rr.Points3D(lm_xyz, colors=lm_col, radii=0.006), static=True)
    lm_row = {lid: i for i, lid in enumerate(lm_ids)}

    obs_by_kf: dict[int, dict[int, list]] = defaultdict(lambda: defaultdict(list))
    first_kf: dict[int, int] = {}
    for o in m["observations"]:
        k, lm = int(o["kf_id"]), int(o["lm_id"])
        obs_by_kf[k][int(o["cam_id"])].append((lm, o["pos"][0], o["pos"][1]))
        first_kf[lm] = min(first_kf.get(lm, k), k)
    new_lms_by_kf: dict[int, list[int]] = defaultdict(list)
    for lm, k in first_kf.items():
        if lm in lm_row:
            new_lms_by_kf[k].append(lm_row[lm])

    # Before PGO: real-time VIO poses (causal, untouched by loop closing) at keyframe stamps.
    rt = apply(T_align, load_csv_poses(run / "eval" / "lc_rt.csv"))
    after_pgo = apply(T_align, load_csv_poses(run / "map" / "kf_trajectory.csv"))
    gba = apply(T_align, load_csv_poses(run / "map" / "optimized_tracking_pg_float.csv"))
    pg = json.loads((run / "map" / "pose_graph.json").read_text())["edges"]

    def graph(prefix: str, P: dict[int, np.ndarray]) -> None:
        ids = [k for k in kf_ids if k in P]
        rr.log(f"{prefix}/nodes", rr.Points3D(centres(P, ids), radii=0.012, colors=[(235, 235, 235)]), static=True)
        for etype, col, rad in [("spanning_tree", (120, 170, 250), 0.003), ("high_covisibility", (110, 110, 150), 0.0015), ("loop_closure", (255, 80, 200), 0.006)]:
            segs = [[P[e["kf_id_a"]][:3, 3], P[e["kf_id_b"]][:3, 3]] for e in pg if e["type"] == etype and e["kf_id_a"] in P and e["kf_id_b"] in P]
            if segs:
                rr.log(f"{prefix}/{etype}", rr.LineStrips3D(segs, colors=[col], radii=rad), static=True)

    rt_at_kf = {}
    rt_keys = np.array(sorted(rt))
    for k in kf_ids:
        i = int(np.argmin(np.abs(rt_keys - k)))
        rt_at_kf[k] = rt[int(rt_keys[i])]
    graph("world/pose_graph/before", rt_at_kf)
    graph("world/pose_graph/after", after_pgo)
    graph("world/pose_graph/gba", gba)

    # Global BA shift: after online PGO -> after global BA, per keyframe (exaggerated). The global BA
    # fixes no gauge, so remove the best rigid motion first (Kabsch) and show only the change of shape.
    both = [k for k in kf_ids if k in after_pgo and k in gba]
    if both:
        o = centres(after_pgo, both)
        g = centres(gba, both)
        raw = np.linalg.norm(g - o, axis=1) * 100.0
        mo, mg = o.mean(axis=0), g.mean(axis=0)
        U, _, Vt = np.linalg.svd((g - mg).T @ (o - mo))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        g_al = (R @ (g - mg).T).T + mo
        d = g_al - o
        mag = np.linalg.norm(d, axis=1) * 100.0
        rr.log("world/gba_shift/arrows", rr.Arrows3D(origins=o, vectors=d * GBA_SHIFT_SCALE, colors=error_colors(mag, float(np.percentile(mag, 98))), radii=0.003), static=True)
        rr.log("world/gba_shift/nodes", rr.Points3D(o, radii=0.008, colors=[(235, 235, 235)]), static=True)
        facts["gba_shift"] = {"n": len(both), "scale": GBA_SHIFT_SCALE, "median_cm": float(np.median(mag)), "p95_cm": float(np.percentile(mag, 95)),
                              "max_cm": float(mag.max()), "raw_median_cm": float(np.median(raw)), "raw_max_cm": float(raw.max())}
        rr.log("world/gba_shift/stats", rr.TextDocument(
            f"Keyframe shift online PGO -> global BA after removing the best rigid motion (arrows x{GBA_SHIFT_SCALE:.0f}): "
            f"median {np.median(mag):.2f} cm, 95th pct {np.percentile(mag, 95):.2f} cm, max {mag.max():.2f} cm over {len(both)} keyframes "
            f"(raw, with the gauge motion: median {np.median(raw):.2f} cm)"), static=True)

    # Loop drift gate: every "[MDB] Drift" decision of the online run, in order.
    log_f = run / f"output.{seq}.log"
    if log_f.exists():
        import re
        gates = [(float(m.group(1)) * 100.0, float(m.group(2)), m.group(3) == "ACCEPTED")
                 for m in re.finditer(r"\[MDB\] Drift: ([0-9.eE+-]+) m, rot: ([0-9.eE+-]+) deg\. -> (ACCEPTED|REJECTED)", log_f.read_text())]
        for i, (dcm, ddeg, acc) in enumerate(gates):
            rr.set_time("loop_index", sequence=i)
            rr.log("loops/gate/drift_cm", rr.Scalars(dcm))
            rr.log("loops/gate/rotation_deg", rr.Scalars(ddeg))
            rr.log("loops/gate/threshold_10cm", rr.Scalars(10.0))
            rr.log("loops/gate/threshold_2deg", rr.Scalars(2.0))
            rr.log("loops/gate/accepted", rr.Scalars(1.0 if acc else 0.0))
        rr.reset_time()
        rr.log("loops/gate/drift_cm", rr.SeriesPoints(colors=[(255, 80, 200)], names=["drift translation [cm]"], marker_sizes=[5.0]), static=True)
        rr.log("loops/gate/rotation_deg", rr.SeriesPoints(colors=[(34, 181, 230)], names=["drift rotation [deg]"], marker_sizes=[5.0]), static=True)
        rr.log("loops/gate/threshold_10cm", rr.SeriesLines(colors=[(255, 80, 200)], names=["gate 10 cm"], widths=[1.0]), static=True)
        rr.log("loops/gate/threshold_2deg", rr.SeriesLines(colors=[(34, 181, 230)], names=["gate 2 deg"], widths=[1.0]), static=True)
        rr.log("loops/gate/accepted", rr.SeriesPoints(colors=[(246, 218, 117)], names=["PGO triggered (1 = yes)"], marker_sizes=[7.0]), static=True)
        pgo = re.findall(r"Ceres: (\d+) iter, ([0-9.]+) ms, cost ([0-9.eE+-]+) -> ([0-9.eE+-]+)", log_f.read_text())
        facts["loop_gate"] = {"n": len(gates), "accepted": sum(g[2] for g in gates),
                              "drift_cm": [g[0] for g in gates], "rot_deg": [g[1] for g in gates],
                              "pgo": [{"iters": int(i), "ms": float(ms), "cost0": float(c0), "cost1": float(c1)} for i, ms, c0, c1 in pgo]}

    # --- keyframe timeline: cameras, images, observations, tracks, window, loops -------
    T_i_c = []
    for ci in range(n_cams):
        tic = calib["T_imu_cam"][ci]
        T_i_c.append(se3(np.array([tic["qw"], tic["qx"], tic["qy"], tic["qz"]]), np.array([tic["px"], tic["py"], tic["pz"]])))
    cam = calib["intrinsics"][0]["intrinsics"]
    w, h = calib["resolution"][0]
    rr.log("world/vio/cam0", rr.Pinhole(focal_length=[cam["fx"], cam["fy"]], principal_point=[cam["cx"], cam["cy"]],
                                        resolution=[w, h], image_plane_distance=0.15, camera_xyz=rr.ViewCoordinates.RDF), static=True)
    images = [image_index(ds, ci) for ci in range(n_cams)]

    def image_of(t_ns: int, ci: int = 0) -> Path | None:
        idx, keys = images[ci]
        i = int(np.argmin(np.abs(keys - t_ns)))
        return idx[int(keys[i])] if abs(int(keys[i]) - t_ns) < 5_000_000 else None

    loops_by_b = defaultdict(list)
    for e in pg:
        if e["type"] == "loop_closure":
            loops_by_b[max(e["kf_id_a"], e["kf_id_b"])].append(min(e["kf_id_a"], e["kf_id_b"]))

    history: dict[tuple[int, int], list] = defaultdict(list)  # (cam, lm_id) -> [(x, y)] over keyframes
    trail = []
    loop_total = 0
    loop_kf_index: list[int] = []
    loop_segs: list = []
    map_rows: list[int] = []
    for n, k in enumerate(kf_ids):
        rr.set_time("time", duration=(k - t_start) * 1e-9)
        rr.set_time("keyframe", sequence=n)
        log_transform("world/vio/cam0", kf_after[k] @ T_i_c[0])
        trail.append(kf_after[k][:3, 3])
        rr.log("world/vio/keyframes", rr.Points3D(np.array(trail), radii=0.01, colors=[(250, 250, 120)]))
        window = np.array(trail[-VIO_MAX_KFS:])
        rr.log("world/vio/window", rr.Points3D(window, radii=0.022, colors=[(255, 140, 60)]))
        rr.log("world/vio/window_path", rr.LineStrips3D([window], radii=0.004, colors=[(255, 140, 60)]))
        map_rows.extend(new_lms_by_kf.get(k, []))
        if map_rows and (n % a.map_stride == 0 or n == len(kf_ids) - 1):  # cumulative map, every map_stride keyframes
            rr.log("world/map/growing", rr.Points3D(lm_xyz[map_rows], colors=lm_col[map_rows], radii=0.006))
        for ci in range(n_cams):
            obs = obs_by_kf.get(k, {}).get(ci, [])
            for lm, x, y in obs:
                history[(ci, lm)].append((x, y))
                history[(ci, lm)] = history[(ci, lm)][-a.track_len:]
            if obs:
                pts = np.array([[x, y] for _, x, y in obs])
                ids = [lm for lm, _, _ in obs]
                tracks = [np.array(history[(ci, lm)]) for lm in ids if len(history[(ci, lm)]) > 1]
                rr.log(f"frontend/cam{ci}/keypoints", rr.Points2D(pts, radii=3.0, class_ids=[i % 64 for i in ids]))
                rr.log(f"frontend/cam{ci}/tracks", rr.LineStrips2D(tracks, colors=[(80, 220, 140)], radii=1.0))
            if n % a.kf_image_stride == 0:
                p = image_of(k, ci)
                if p is not None:
                    rr.log(f"frontend/cam{ci}", jpeg(p))
                    if ci == 0:
                        rr.log("world/vio/cam0", jpeg(p))
        if k in loops_by_b:
            loop_kf_index.append(n)
            loop_segs += [[kf_after[k][:3, 3], kf_after[j][:3, 3]] for j in loops_by_b[k] if j in kf_after]
            rr.log("world/loops/detected", rr.LineStrips3D(loop_segs, colors=[(255, 80, 200)], radii=0.008))
            loop_total += len(loops_by_b[k])
            q = image_of(k)
            mt = image_of(loops_by_b[k][0])
            if q is not None and mt is not None:
                rr.log("loops/query", jpeg(q))
                rr.log("loops/match", jpeg(mt))
            rr.log("loops/count", rr.Scalars(len(loops_by_b[k])))
            rr.log("loops/cumulative", rr.Scalars(loop_total))

    # --- global BA iterations ---------------------------------------------------------
    gba_names = [("balpgf", "float32"), ("balpgd", "float64"), ("bal", "float32_no_pose_graph")]
    gba_cols = {"float32": (80, 220, 140), "float64": (120, 170, 250), "float32_no_pose_graph": (240, 180, 60)}
    for variant, name in gba_names:
        f = run / f"ba_log.{variant}.json"
        if not f.exists():
            continue
        b = json.loads(f.read_text())
        for i, it in enumerate(b["iteration"]):
            rr.set_time("gba_iteration", sequence=int(it))
            rr.log(f"gba/cost/{name}", rr.Scalars(b["cost_valid"][i]))
            rr.log(f"gba/log10_cost/{name}", rr.Scalars(float(np.log10(b["cost_valid"][i]))))
            rr.log(f"gba/iteration_time_s/{name}", rr.Scalars(b["iteration_time"][i]))
            rr.log(f"gba/cumulative_time_s/{name}", rr.Scalars(b["cumulative_time"][i]))
        rr.reset_time()
        for grp in ["cost", "log10_cost", "iteration_time_s", "cumulative_time_s"]:
            rr.log(f"gba/{grp}/{name}", rr.SeriesLines(colors=[gba_cols[name]], names=[name.replace("_", " ")], widths=[2.0]), static=True)

    # --- blueprints ---------------------------------------------------------------------
    lo3, hi3 = gt_xyz.min(axis=0), gt_xyz.max(axis=0)
    centre = (lo3 + hi3) / 2
    ext = float(np.max(hi3 - lo3))

    def eye(dx: float = -0.55, dy: float = -0.85, dz: float = 0.9, scale: float = 0.8, target=None) -> rrb.EyeControls3D:
        t = centre if target is None else target
        return rrb.EyeControls3D(kind="Orbital", position=t + scale * ext * np.array([dx, dy, dz]), look_target=t, eye_up=[0, 0, 1])

    def v3(name: str, contents: list[str], **kw) -> rrb.Spatial3DView:
        return rrb.Spatial3DView(name=name, origin="world", contents=contents, eye_controls=kw.pop("eye", eye()),
                                 background=(12, 14, 18), line_grid=rrb.LineGrid3D(visible=False), **kw)

    final_now = ["world/final/**", "world/final_now/**"]
    compare_all = ["world/final/gt", "world/final/bapgf", "world/compare/**", "world/compare_now/**", "world/final_now/bapgf"]

    def cams_grid(name: str = "Frontend: 4 cameras, keyframe observations + tracks") -> rrb.Grid:
        return rrb.Grid(*[rrb.Spatial2DView(name=f"cam{ci}", origin=f"frontend/cam{ci}") for ci in range(n_cams)], grid_columns=2, name=name)

    main_bp = rrb.Blueprint(
        rrb.Tabs(
            rrb.Horizontal(
                v3("Final trajectories vs GT (dots = now)", ["world/final/**", "world/compare/**", "world/final_now/**", "world/compare_now/**"]),
                rrb.Vertical(
                    rrb.Spatial2DView(name="Frontend: keyframe tracks (cam0)", origin="frontend/cam0"),
                    rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm"),
                ),
                column_shares=[2, 1],
                name="Overview",
            ),
            rrb.Horizontal(
                cams_grid(),
                v3("VIO keyframes + map", ["world/vio/**", "world/map/growing"]),
                name="Frontend + VIO",
            ),
            rrb.Horizontal(
                v3("Loops", ["world/loops/**", "world/vio/keyframes", "world/final/gt"]),
                rrb.Vertical(rrb.Spatial2DView(name="query keyframe", origin="loops/query"), rrb.Spatial2DView(name="matched keyframe", origin="loops/match")),
                name="Loop detection",
            ),
            rrb.Horizontal(
                v3("Pose graph before PGO (VIO)", ["world/pose_graph/before/**", "world/final/gt"]),
                v3("after online PGO", ["world/pose_graph/after/**", "world/final/gt"]),
                v3("after global BA (RootSLAM)", ["world/pose_graph/gba/**", "world/final/gt"]),
                name="Pose graph",
            ),
            rrb.Grid(
                rrb.TimeSeriesView(name="cost (valid)", origin="gba/cost"),
                rrb.TimeSeriesView(name="iteration time [s]", origin="gba/iteration_time_s"),
                rrb.TimeSeriesView(name="cumulative time [s]", origin="gba/cumulative_time_s"),
                v3(f"keyframe shift PGO -> GBA (x{GBA_SHIFT_SCALE:.0f})", ["world/gba_shift/**"]),
                name="Global BA float32 vs float64",
            ),
            rrb.Horizontal(
                v3("trajectories vs GT", final_now),
                rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                   contents=["eval/position_error_cm/bsl", "eval/position_error_cm/lc", "eval/position_error_cm/bapgf"]),
                name="Evaluation",
            ),
            *([rrb.Horizontal(
                v3("RootSLAM vs slam-rs vs colsfm vs GT", compare_all),
                rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                   contents=["eval/position_error_cm/bapgf", *[f"eval/position_error_cm/{k}" for k in compare_keys]]),
                column_shares=[3, 2],
                name="RootSLAM vs our stack",
            )] if compare_keys else []),
        ),
        collapse_panels=True,
    )
    # Default only: an .rbl opened next to the .rrd (page embeds) must win over it.
    rr.send_blueprint(main_bp, make_active=False, make_default=True)

    if a.blueprint_dir is not None:
        a.blueprint_dir.mkdir(parents=True, exist_ok=True)
        n_kf = len(kf_ids)
        mid_kf = n_kf // 2
        loop_kf = loop_kf_index[len(loop_kf_index) // 2] if loop_kf_index else mid_kf
        t_end_s = (int(gt_keys[-1]) - t_start) * 1e-9
        img_only = lambda ci: [f"frontend/cam{ci}"]  # noqa: E731
        figs: dict[str, tuple[object, str, object]] = {
            # name: (view or container, timeline, time)
            "fe_image": (rrb.Spatial2DView(name="cam0 keyframe image", origin="frontend/cam0", contents=img_only(0)), "keyframe", mid_kf),
            "fe_keypoints": (rrb.Spatial2DView(name="cam0 observations", origin="frontend/cam0", contents=["frontend/cam0", "frontend/cam0/keypoints"]), "keyframe", mid_kf),
            "fe_tracks": (rrb.Spatial2DView(name="cam0 tracks", origin="frontend/cam0"), "keyframe", mid_kf),
            "fe_4cams": (cams_grid("4 cameras"), "keyframe", mid_kf),
            "vio_window": (v3("VIO keyframes + window", ["world/vio/**"], eye=eye(scale=0.4, target=kf_after[kf_ids[mid_kf]][:3, 3])), "keyframe", mid_kf),
            "vio_timing": (rrb.TimeSeriesView(name="per-frame time [ms]", origin="timing"), "time", t_end_s / 2),
            "vio_out": (v3("Basalt VIO vs GT", ["world/final/gt", "world/final/bsl"]), "keyframe", n_kf - 1),
            "map_growing": (v3("persistent map", ["world/map/growing", "world/vio/keyframes"], eye=eye(scale=2.0)), "keyframe", mid_kf),
            "map_full": (v3("persistent map (end)", ["world/map/landmarks", "world/vio/keyframes"], eye=eye(scale=2.0)), "keyframe", n_kf - 1),
            "loop_pair": (rrb.Horizontal(rrb.Spatial2DView(name="query keyframe", origin="loops/query"), rrb.Spatial2DView(name="matched old keyframe", origin="loops/match")), "keyframe", loop_kf),
            "loop_edges": (v3("loop edges", ["world/loops/**", "world/vio/keyframes", "world/final/gt"]), "keyframe", n_kf - 1),
            "pg_before": (v3("pose graph before PGO", ["world/pose_graph/before/**", "world/final/gt"]), "keyframe", n_kf - 1),
            "pg_after": (v3("pose graph after online PGO", ["world/pose_graph/after/**", "world/final/gt"]), "keyframe", n_kf - 1),
            "pgo_out": (v3("+LC vs GT", ["world/final/gt", "world/final/lc"]), "keyframe", n_kf - 1),
            "gba_in": (v3("map + pose graph into GBA", ["world/map/landmarks", "world/pose_graph/after/**"], eye=eye(scale=1.6)), "keyframe", n_kf - 1),
            "gba_cost": (rrb.TimeSeriesView(name="GBA log10 cost (valid) per iteration", origin="gba/log10_cost"), "gba_iteration", 0),
            "gba_time": (rrb.TimeSeriesView(name="GBA cumulative time [s]", origin="gba/cumulative_time_s"), "gba_iteration", 0),
            "gba_shift": (v3(f"keyframe shift PGO -> GBA (x{GBA_SHIFT_SCALE:.0f})", ["world/gba_shift/arrows", "world/gba_shift/nodes"]), "keyframe", n_kf - 1),
            "loop_gate": (rrb.TimeSeriesView(name="loop drift gate: PnP drift per detected loop", origin="loops/gate"), "loop_index", 0),
            "gba_out": (v3("RootSLAM vs GT", ["world/final/gt", "world/final/bapgf"]), "keyframe", n_kf - 1),
            "eval_all": (v3("all variants vs GT", ["world/final/**"]), "keyframe", n_kf - 1),
            "eval_error": (v3("RootSLAM coloured by position error", ["world/eval/rootslam_error", "world/final/gt"]), "keyframe", n_kf - 1),
            "eval_series": (rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                               contents=["eval/position_error_cm/bsl", "eval/position_error_cm/lc", "eval/position_error_cm/bapgf"]), "time", t_end_s),
        }
        if compare_keys:
            figs["compare_all"] = (v3("RootSLAM vs slam-rs vs colsfm vs GT", ["world/final/gt", "world/final/bapgf", "world/compare/**"]), "keyframe", n_kf - 1)
            figs["compare_series"] = (rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                                         contents=["eval/position_error_cm/bapgf", *[f"eval/position_error_cm/{k}" for k in compare_keys]]), "time", t_end_s)
        # Embeds: richer layouts, swept for the video; the live viewer opens with the same .rbl.
        embeds = {
            "emb_frontend": (rrb.Horizontal(cams_grid(), v3("VIO keyframes + growing map", ["world/vio/**", "world/map/growing"], eye=eye(scale=1.6)), column_shares=[1, 1]), "keyframe", None),
            "emb_loops": (rrb.Horizontal(v3("loops + keyframes", ["world/loops/**", "world/vio/keyframes", "world/final/gt"]),
                                         rrb.Vertical(rrb.Spatial2DView(name="query keyframe", origin="loops/query"), rrb.Spatial2DView(name="matched old keyframe", origin="loops/match")),
                                         column_shares=[3, 2]), "keyframe", None),
            "emb_eval": (rrb.Horizontal(v3("trajectories vs GT (dots = now)", final_now),
                                        rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                                           contents=["eval/position_error_cm/bsl", "eval/position_error_cm/lc", "eval/position_error_cm/bapgf"]),
                                        column_shares=[3, 2]), "time", None),
        }
        if compare_keys:
            embeds["emb_compare"] = (rrb.Horizontal(v3("RootSLAM vs slam-rs vs colsfm vs GT (dots = now)", compare_all),
                                                    rrb.TimeSeriesView(name="position error [cm]", origin="eval/position_error_cm",
                                                                       contents=["eval/position_error_cm/bapgf", *[f"eval/position_error_cm/{k}" for k in compare_keys]]),
                                                    column_shares=[3, 2]), "time", None)
        manifest = {"app_id": app_id, "n_keyframes": n_kf, "t_end_s": t_end_s, "loop_keyframes": loop_kf_index, "figures": {}, "embeds": {}}
        for group, items in [("figures", figs), ("embeds", embeds)]:
            for name, (view, timeline, t) in items.items():
                bp = rrb.Blueprint(view, rrb.TimePanel(state="hidden" if group == "figures" else "collapsed", timeline=timeline), rrb.BlueprintPanel(state="collapsed"), rrb.SelectionPanel(state="collapsed"))
                bp.save(app_id, a.blueprint_dir / f"{name}.rbl")
                manifest[group][name] = {"rbl": f"{name}.rbl", "timeline": timeline, "time": t}
        manifest["main_rbl"] = "main.rbl"
        main_bp.save(app_id, a.blueprint_dir / "main.rbl")
        manifest["facts"] = facts
        (a.blueprint_dir / "figures.json").write_text(json.dumps(manifest, indent=1))
        print(f"wrote {len(figs)} figure + {len(embeds)} embed blueprints to {a.blueprint_dir}")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
