"""Aggregate per-sequence metrics into the "ours vs paper" tables (Table I / II of RootSLAM).

Reads <runs>/<SEQ>/eval/metrics.json and run_info.txt (written by run_sequence.sh +
evaluate.py) and writes <out>/summary.json and <out>/summary.md.

Usage: python scripts/repro/summarize.py [--runs $RUNS_ROOT (default <repo>/runs)] [--out results]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "thirdparty" / "xrtslam-metrics"))
from timing import TimingStats  # noqa: E402  (xrtslam-metrics, as the CI report: -fc 4 -lc -2)


def frame_ms(csv: Path) -> float | None:
    """Mean per-frame time [ms], frontend_frames_received -> backend_state_pushed."""
    try:
        return float(TimingStats(csv_fn=csv, cols=("4", "-2"), units="ms").mean)
    except Exception:  # noqa: BLE001
        return None

# Paper Table I, RMSE ATE [cm] (group averages): BSL, +LC, +GBA, RS.
PAPER_ATE = {
    "MIO": (33.0, 9.3, 8.3, 7.0), "MIP": (23.6, 4.0, 2.6, 2.3), "MG": (21.9, 4.5, 4.1, 3.4),
    "MO": (19.0, 5.7, 5.7, 4.6), "EMH": (9.1, 8.1, 8.1, 7.0), "EV1": (5.1, 5.9, 6.2, 3.8),
    "EV2": (7.8, 6.9, 7.9, 5.8), "TC": (39.7, 11.1, 9.4, 8.4), "TM": (183.1, 93.4, 91.2, 90.7),
    "TR": (8.7, 3.7, 0.9, 0.9), "TS": (102.5, 83.2, 82.4, 82.2),
}
PAPER_RTE = {
    "MIO": (1.38, 1.48, 1.88, 1.26), "MIP": (0.62, 0.79, 0.94, 0.70), "MG": (1.35, 1.47, 1.73, 1.21),
    "MO": (1.15, 0.98, 1.30, 0.89), "EMH": (0.89, 1.03, 0.91, 0.88), "EV1": (1.22, 1.33, 2.11, 1.32),
    "EV2": (1.23, 1.63, 2.91, 1.48), "TC": (6.77, 2.20, 1.55, 1.41), "TM": (18.95, 10.93, 10.55, 10.52),
    "TR": (0.67, 0.80, 0.45, 0.45), "TS": (11.79, 10.38, 10.01, 10.01),
}
# Paper Table II: GBA double vs float, ATE [cm] and time [s] (i5-13600).
PAPER_T2 = {
    "MIO": (7.0, 7.0, 4.59, 2.03), "MIP": (2.3, 2.3, 15.19, 7.74), "MG": (3.5, 3.4, 2.85, 1.39),
    "MO": (4.9, 4.6, 3.68, 2.07), "EMH": (7.0, 7.0, 2.69, 1.29), "EV1": (3.8, 3.8, 2.34, 1.20),
    "EV2": (5.7, 5.8, 2.70, 1.38),
}
GROUP_SIZE = {"MIO": 16, "MIP": 17, "MG": 15, "MO": 16, "EMH": 5, "EV1": 3, "EV2": 3, "TC": 5, "TM": 6, "TR": 6, "TS": 3}
COLS = ["bsl", "lc", "ba", "bapgf"]  # paper columns BSL, +LC, +GBA, RS


def group_of(seq: str) -> str:
    for p, g in [("MIP", "MIP"), ("MIO", "MIO"), ("MGO", "MG"), ("MOO", "MO"), ("EMH", "EMH"), ("EV1", "EV1"), ("EV2", "EV2"),
                 ("TC", "TC"), ("TM", "TM"), ("TR", "TR"), ("TS", "TS"), ("TO", "TO")]:
        if seq.startswith(p):
            return g
    return "?"


def parse_info(path: Path) -> dict:
    info = {}
    if path.exists():
        for m in re.finditer(r"(\w+_wall_s)=([\d.]+)", path.read_text()):
            info[m.group(1)] = float(m.group(2))
        m = re.search(r"load_start=([\d.]+)", path.read_text())
        if m:
            info["load_start"] = float(m.group(1))
    return info


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, default=Path(os.environ.get("RUNS_ROOT", Path(__file__).resolve().parents[2] / "runs")))
    ap.add_argument("--out", type=Path, default=Path("results"))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    seqs = {}
    for mf in sorted(a.runs.glob("*/eval/metrics.json")):
        d = json.loads(mf.read_text())
        seq = d["seq"]
        info = parse_info(mf.parent.parent / "run_info.txt")
        logs = {}
        for v in ["balpgf", "balpgd", "bal"]:
            f = mf.parent.parent / f"ba_log.{v}.json"
            if f.exists():
                logs[v] = json.loads(f.read_text())["_static"]["timing"]["optimize"]
        rd = mf.parent.parent
        info["frame_ms_rs"] = frame_ms(rd / f"timing.tracking.{seq}.csv")
        info["frame_ms_bsl"] = frame_ms(rd / "bsl" / f"timing.tracking.{seq}.csv")
        seqs[seq] = {"group": group_of(seq), "methods": d["methods"], "info": info, "gba_optimize_s": logs}

    groups = {}
    for seq, s in seqs.items():
        groups.setdefault(s["group"], []).append(seq)

    def val(seq, col, key="ate_se3_cm"):
        m = seqs[seq]["methods"].get(col, {})
        return m.get(key)

    lines = ["# RootSLAM reproduction: ours vs paper", "",
             "RMSE ATE [cm], SE(3) alignment, per-frame trajectories, xrtslam-metrics association (paper protocol).",
             "Columns as in paper Table I: BSL = Basalt VIO, +LC = loop closing + online PGO, +GBA = global BA without pose graph,",
             "RS = RootSLAM (GBA float32 + pose graph). One deterministic run per sequence, 4 VIO threads, pinned to 8 cores.", "",
             "## Group averages", "",
             "| group | done/total | BSL ours | BSL paper | +LC ours | +LC paper | +GBA ours | +GBA paper | RS ours | RS paper | RS Sim(3) ours | RS RTE ours | RS RTE paper |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    summary_groups = {}
    for g in [g for g in GROUP_SIZE if g in groups]:
        ss = sorted(groups[g])
        row = {}
        for c in COLS + ["sim3", "rte"]:
            key = {"sim3": "ate_sim3_cm", "rte": "rte_cm"}.get(c, "ate_se3_cm")
            col = "bapgf" if c in ("sim3", "rte") else c
            vals = [val(s, col, key) for s in ss]
            vals = [v for v in vals if v is not None]
            row[c] = sum(vals) / len(vals) if vals else None
        summary_groups[g] = {"n": len(ss), "total": GROUP_SIZE[g], "ours": row, "paper_ate": PAPER_ATE.get(g), "paper_rte": PAPER_RTE.get(g)}
        pa = PAPER_ATE.get(g, (None,) * 4)
        f = lambda x: "–" if x is None else f"{x:.1f}"  # noqa: E731
        lines.append(f"| {g} | {len(ss)}/{GROUP_SIZE[g]} | {f(row['bsl'])} | {f(pa[0])} | {f(row['lc'])} | {f(pa[1])} | {f(row['ba'])} | {f(pa[2])} | "
                     f"**{f(row['bapgf'])}** | **{f(pa[3])}** | {f(row['sim3'])} | {'–' if row['rte'] is None else f'{row['rte']:.2f}'} | {PAPER_RTE[g][3]:.2f} |")
    # Paper "All*": mean over all 95 non-TO sequences (Table I / III).
    all_seqs = [q for q in seqs if seqs[q]["group"] in GROUP_SIZE]
    allrow = {}
    for c in COLS:
        v = [val(q, c) for q in all_seqs]
        v = [x for x in v if x is not None]
        allrow[c] = sum(v) / len(v) if v else None
    summary_groups["All*"] = {"n": len(all_seqs), "total": 95, "ours": allrow, "paper_ate": (34.8, 14.1, 13.3, 12.4)}
    f = lambda x: "–" if x is None else f"{x:.1f}"  # noqa: E731
    lines.append(f"| All* | {len(all_seqs)}/95 | {f(allrow['bsl'])} | 34.8 | {f(allrow['lc'])} | 14.1 | {f(allrow['ba'])} | 13.3 | "
                 f"**{f(allrow['bapgf'])}** | **12.4** | – | – | 1.90 |")
    lines += ["", "Partial groups average only the sequences done; the paper averages the full group.",
              "All* = mean over every sequence of the paper groups except TUM-VI outdoors (as in the paper).",
              "MIPT02 and MOO15: run 1 failed (MIPT02 basalt_vio abort in loop closing; MOO15 VIO divergence to km); the table uses one rerun, see REPRO.md.", "",
              "## Per sequence", "",
              "| seq | BSL | +LC | +GBA | RS (f32) | RS (f64) | RS Sim(3) | RS scale | RS RTE | completion | frame ms BSL / RS | VIO wall [s] | GBA f32 / f64 optimize [s] | load |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for seq in sorted(seqs, key=lambda s: (list(GROUP_SIZE).index(seqs[s]["group"]) if seqs[s]["group"] in GROUP_SIZE else 99, s)):
        s = seqs[seq]
        f = lambda x: "err" if x is None else f"{x:.2f}"  # noqa: E731
        go = s["gba_optimize_s"]
        gtxt = f"{go['balpgf']:.2f} / {go['balpgd']:.2f}" if "balpgf" in go and "balpgd" in go else "–"
        m = s["methods"].get("bapgf", {})
        lines.append(f"| {seq} | {f(val(seq, 'bsl'))} | {f(val(seq, 'lc'))} | {f(val(seq, 'ba'))} | **{f(val(seq, 'bapgf'))}** | {f(val(seq, 'bapgd'))} | "
                     f"{f(m.get('ate_sim3_cm'))} | {m.get('sim3_scale', float('nan')):.4f} | {f(m.get('rte_cm'))} | {m.get('completion', 0):.3f} | "
                     f"{(s['info'].get('frame_ms_bsl') or float('nan')):.1f} / {(s['info'].get('frame_ms_rs') or float('nan')):.1f} | {s['info'].get('vio_wall_s', float('nan')):.0f} | {gtxt} | {s['info'].get('load_start', float('nan')):.0f} |")
    (a.out / "summary.md").write_text("\n".join(lines) + "\n")
    (a.out / "summary.json").write_text(json.dumps({"groups": summary_groups, "sequences": seqs, "paper_table2": PAPER_T2}, indent=2))
    print("\n".join(lines[:8 + len(summary_groups) + 2]))


if __name__ == "__main__":
    main()
