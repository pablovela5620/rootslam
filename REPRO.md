# RootSLAM reproduction with pixi

This file reports one full reproduction of Table I of *RootSLAM: Square Root Bundle Adjustment for Visual-Inertial SLAM*
(Santucci, de Mayo, Cremers, Pire, [arXiv 2610.15157](https://arxiv.org/abs/2610.15157)), run with the pixi build of this branch.
The code is the authors' public work branch; the numbers are ours.

## Pins

| component | source | commit |
|---|---|---|
| basalt (VIO, loop closing, map + pose-graph export) | gitlab.freedesktop.org/mateosss/basalt, branch `tsantucci/loop-closure` | `4c696ed8b2c8b74030fc00b855de00b8fa17d872` |
| rootba (`bal`, global square-root BA with pose-graph residuals) | github.com/TomasSantucci/rootba, branch `fixed-intrinsics` (the CI's `ROOTBA_REF`) | `83959b5cc7c80f548fb062ef3de81d7b4a0ee68d` |
| xrtslam-metrics (association, ATE / RTE, frame merge) | gitlab.freedesktop.org/mateosss/xrtslam-metrics | `f5ae56ca09a264e7811e56bb5787bf038fddba6c` |

## Protocol

- The pipeline mirrors the branch CI (`.ci/cieval.template.yaml`): `basalt_vio --deterministic 1 --num-threads 4 --map-export-path map`,
  then three `bal` runs on `map/map.json`, then `merge_frames_keyframes.py` to per-frame trajectories.
- Paper columns map to runs as follows. **BSL**: Basalt VIO with `config.enable_loop_closing=false` (not in the CI; added here).
  **+LC**: keyframes after online PGO, merged into all frames. **+GBA**: `bal` float32, no pose graph, `--min-obs-per-kf 20`.
  **RS (RootSLAM)**: `bal` float32 with pose-graph residuals, `--min-obs-per-kf 10` (CI job `bapgf`; it equals the float column of the paper's Table II).
- Metric: RMSE ATE in cm after SE(3) Umeyama alignment (evo, as `xrtslam-metrics tracking.py ate`; it matches the C++ `atec` the CI report uses),
  with xrtslam-metrics' time association against each dataset's raw GT file. RTE: delta = 6 frames.
- One deterministic run per sequence, as the CI does. Each `basalt_vio` / `bal` process is pinned to 8 cores.
- All* = mean over all 95 sequences of the paper's groups except TUM-VI outdoors (as in the paper). TO was not run.

## Table I: ours vs paper (RMSE ATE [cm], group averages)

1000

**Headline:** RootSLAM 14.1 cm vs the paper's 12.4 cm over all 95 sequences; plain Basalt 33.5 vs 34.8.
TUM-VI magistrale alone adds 1.6 of the 1.7 cm difference ((115.8 - 90.7) x 6 / 95).
MG, MIP, MO and TR match to 0.1 cm; EV1, EV2 and TC are within 0.7 cm; TS is 10 cm better than the paper.

## Gaps and their explanation

All three gaps enter at the loop-closing stage (+LC), and loop closing varies from run to run on our host.

- **EMH (8.7 vs 7.0).** BSL matches (9.4 vs 9.1). With `--deterministic-mapper 1` (synchronous mapper; the CI does not pass it)
  RS per sequence is 6.91 / 4.66 / 5.52 / 11.83 / 9.23 → **7.6** (CI flags: 7.26 / 5.83 / 9.21 / 12.55 / 8.80 → 8.7).
  +LC stays very noisy (EMH04 +LC 19.5 → 35.4 between the two runs); the global BA absorbs most of it. Explained: loop-closing variance plus the asynchronous mapper on a loaded host.
- **MIO (9.1 vs 7.0).** Carried by the sequences where VIO fails (BSL 54-118 cm: MIO01, 02, 13, 15, 16). Synchronous-mapper reruns:
  MIO02 45.9 → 32.4, MIO16 16.6 → 13.4, MIO01 17.6 → 16.2, MIO13 28.8 → 28.6. With these four MIO ≈ 8.0.
  Partly explained; **MIO13 stays at ~29 cm in both modes, unexplained** (the paper gives no per-sequence numbers).
- **TM (115.8 vs 90.7).** Magistrale has mocap GT only at the start and the end, so ATE measures whether the start-to-end loop closed.
  Start-to-end loop edges per run: TM1 4 (RS 7.4 cm), TM2-TM6 0 (RS 91-219 cm). The group average is a handful of binary events.
  Sim(3) numbers mean nothing on TM (two GT clusters).
- **EuRoC scale.** Every EuRoC RS run has Sim(3) scale 1.012-1.017, so Sim(3) ATE is far below SE(3) ATE (EMH 4.4 vs 8.7).
- **Not bit-reproducible.** Even with `--deterministic 1`, two runs of one sequence differ (EMH02 BSL 4.87 vs 5.18).
  Hypothesis, not tested: TBB parallel reductions sum floats in a schedule-dependent order. This is the source of the variance above.

## Rerun policy (2 sequences flagged)

Two first runs failed; the tables use one rerun for each:

| seq | run 1 | rerun used |
|---|---|---|
| MIPT02 | `basalt_vio` aborted (`std::out_of_range`, `map::at`) in loop closing right after an accepted PGO; no map | RS 2.08 cm (no crash) |
| MOO15 | VIO diverged: BSL 1319 m, RS 470.5 cm | BSL 47.4, RS 31.6 cm (low-parallax seated sequence, Sim(3) scale 0.27-0.43) |

With run 1 of MOO15, MO would read 32.1 instead of 4.6. The MIPT02 abort looks like a race in the asynchronous loop closer.

## Float32 vs float64 global BA (paper Table II)

- Optimize time float64 / float32: **median 1.94x** over the 82 sequences with float64 >= 0.5 s, 2.01x on total time
  (1055 s vs 2123 s over 94 sequences). Paper: 2.16x. MGO07: 0.48 s vs 0.94 s, both at final cost 5576.5.
- ATE |float32 - float64|: median 0.004 cm, max 6.0 cm (TM5).
- On 6 of 92 sequences (MOO16, MOO15, TM3, MIO12, MIO13, MIPP06) the final cost differs by more than 1%: both precisions stop at
  the 20-iteration limit without converging after rejected LM steps. MOO16: float32 reaches a lower cost (1505 vs 7212) but a worse ATE (3.26 vs 1.46 cm).

## Speed

Mean frame time (xrtslam-metrics `timing.py -fc 4 -lc -2`, the CI's measure) over 95 sequences: BSL 15.3 ms, RS 15.7 ms.
Paper: 10.6 / 10.8 ms on an Intel Core Ultra 9 285H. Our timings are not comparable to the paper's: see the caveats.

## Hardware and load caveats

- AMD Ryzen 9 9950X3D (16 cores / 32 threads), 186 GB RAM, Linux, conda-forge GCC 15.
- The machine was shared: 1-minute load average 19-80 (mean 43) during the runs. After the first 25 sequences up to 3 sequences ran
  at the same time, all pinned to the same 8 cores. Load affects frame times and, through the asynchronous mapper, loop-closing outcomes.
- The per-sequence table records the load at the start of each run.

## Comparison with two other systems on MSD MGO07

Same evaluator, GT and time association for all methods. slam-rs and colsfm are our own systems (an online Rust VIO and an
offline COLMAP-based refiner of the slam-rs trajectory); they are not part of the paper.

| method | type | SE(3) ATE [cm] | Sim(3) ATE [cm] | runtime |
|---|---|---|---|---|
| Basalt VIO (BSL) | online | 2.10 | 2.09 | 71 s VIO (load ~74) |
| slam-rs | online | 2.11 | 2.01 | 55 s, 10.4 ms/frame, CPU |
| RootSLAM | online + offline GBA | 1.35 | 0.75 | 71 s VIO + 0.48 s GBA |
| colsfm | offline refinement | 1.19 | 0.65 | 289 s, one RTX 5090 |

MGO07 has a ~10 ms camera-to-mocap clock offset that affects all methods; both offline refiners land at Sim(3) scale ~0.986.

## Per sequence

| seq | BSL | +LC | +GBA | RS (f32) | RS (f64) | RS Sim(3) | RS scale | RS RTE | completion | frame ms BSL / RS | VIO wall [s] | GBA f32 / f64 optimize [s] | load |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| MIO01 | 61.91 | 25.17 | 21.15 | **17.64** | 17.84 | 17.45 | 0.9685 | 3.43 | 1.000 | 19.8 / 24.5 | 325 | 3.48 / 7.26 | 47 |
| MIO02 | 118.44 | 47.99 | 45.34 | **45.92** | 45.94 | 38.35 | 0.7434 | 4.45 | 1.000 | 20.8 / 26.2 | 177 | 1.08 / 3.06 | 41 |
| MIO03 | 9.51 | 4.29 | 2.75 | **2.27** | 2.27 | 2.06 | 1.0125 | 0.86 | 1.000 | 18.1 / 26.4 | 228 | 3.02 / 9.98 | 40 |
| MIO04 | 20.64 | 6.68 | 6.94 | **4.12** | 4.11 | 4.12 | 1.0031 | 1.48 | 1.000 | 13.2 / 17.7 | 158 | 2.83 / 5.04 | 54 |
| MIO05 | 3.52 | 8.70 | 1.63 | **1.32** | 1.32 | 1.13 | 1.0092 | 0.28 | 1.000 | 27.4 / 15.4 | 150 | 6.33 / 15.57 | 47 |
| MIO06 | 4.93 | 4.41 | 2.66 | **1.87** | 1.87 | 1.65 | 1.0128 | 0.75 | 1.000 | 27.4 / 30.1 | 212 | 3.36 / 5.50 | 44 |
| MIO07 | 2.28 | 3.76 | 1.67 | **1.57** | 1.57 | 1.51 | 1.0064 | 0.24 | 0.999 | 32.8 / 30.1 | 188 | 3.37 / 15.62 | 51 |
| MIO08 | 5.81 | 2.63 | 7.10 | **3.22** | 3.21 | 2.99 | 1.0139 | 0.83 | 0.999 | 35.8 / 28.9 | 63 | 1.19 / 1.36 | 60 |
| MIO09 | 0.57 | 0.57 | 0.23 | **0.24** | 0.24 | 0.18 | 1.0078 | 0.20 | 0.985 | 41.4 / 34.9 | 19 | 0.09 / 0.08 | 66 |
| MIO10 | 1.53 | 1.61 | 1.73 | **1.37** | 1.37 | 1.20 | 1.0151 | 0.49 | 0.995 | 31.6 / 31.7 | 18 | 0.12 / 0.31 | 61 |
| MIO11 | 2.42 | 2.47 | 2.29 | **2.29** | 2.29 | 2.29 | 0.9997 | 0.62 | 0.996 | 23.2 / 28.1 | 24 | 0.09 / 0.16 | 60 |
| MIO12 | 42.40 | 5.66 | 4.70 | **4.50** | 4.88 | 4.37 | 1.0127 | 0.59 | 1.000 | 23.8 / 28.8 | 962 | 5.34 / 8.72 | 58 |
| MIO13 | 109.39 | 29.01 | 28.99 | **28.78** | 28.80 | 28.05 | 0.8996 | 2.84 | 1.000 | 22.1 / 27.9 | 1010 | 2.00 / 3.30 | 52 |
| MIO14 | 7.91 | 3.38 | 2.55 | **2.50** | 2.49 | 1.70 | 1.0228 | 0.29 | 1.000 | 23.7 / 22.7 | 820 | 16.75 / 39.41 | 43 |
| MIO15 | 72.54 | 13.09 | 11.99 | **11.92** | 11.93 | 11.90 | 0.9910 | 1.66 | 1.000 | 23.7 / 22.4 | 473 | 8.68 / 14.24 | 45 |
| MIO16 | 53.82 | 15.97 | 18.88 | **16.64** | 16.25 | 16.56 | 1.0206 | 1.63 | 1.000 | 21.6 / 24.8 | 537 | 6.66 / 10.10 | 50 |
| MIPB01 | 27.83 | 3.99 | 1.22 | **1.19** | 1.40 | 0.98 | 1.0104 | 0.56 | 1.000 | 24.3 / 23.5 | 417 | 23.18 / 38.90 | 48 |
| MIPB02 | 23.49 | 5.87 | 2.56 | **1.68** | 1.67 | 1.15 | 1.0253 | 0.46 | 1.000 | 26.5 / 27.6 | 504 | 16.18 / 32.36 | 44 |
| MIPB03 | 19.07 | 4.47 | 3.41 | **2.70** | 2.39 | 2.06 | 1.0281 | 0.54 | 1.000 | 25.1 / 26.6 | 857 | 8.26 / 60.50 | 57 |
| MIPB04 | 10.62 | 3.97 | 7.03 | **2.02** | 2.04 | 2.02 | 0.9983 | 0.50 | 1.000 | 20.6 / 20.2 | 339 | 31.33 / 114.03 | 50 |
| MIPB05 | 4.26 | 3.49 | 1.53 | **1.41** | 1.33 | 1.36 | 1.0054 | 0.42 | 1.000 | 13.7 / 24.6 | 350 | 13.12 / 32.82 | 50 |
| MIPB06 | 4.86 | 3.50 | 3.24 | **3.21** | 2.19 | 2.03 | 1.0294 | 0.39 | 1.000 | 14.5 / 15.1 | 214 | 8.50 / 67.67 | 46 |
| MIPB07 | 6.22 | 3.01 | 1.56 | **1.54** | 1.53 | 1.03 | 1.0161 | 0.47 | 1.000 | 12.6 / 17.6 | 214 | 22.95 / 44.76 | 31 |
| MIPB08 | 63.41 | 4.61 | 4.69 | **3.21** | 3.21 | 3.06 | 1.0262 | 0.41 | 1.000 | 10.9 / 22.4 | 3275 | 1.69 / 3.03 | 35 |
| MIPP01 | 45.68 | 4.46 | 3.04 | **2.99** | 2.99 | 2.98 | 1.0026 | 0.98 | 1.000 | 12.7 / 13.7 | 394 | 2.02 / 3.94 | 32 |
| MIPP02 | 24.23 | 4.51 | 4.25 | **3.72** | 3.58 | 3.71 | 1.0057 | 1.45 | 1.000 | 12.8 / 12.4 | 263 | 2.76 / 5.32 | 31 |
| MIPP03 | 26.08 | 4.58 | 4.91 | **3.52** | 3.52 | 2.84 | 1.0375 | 1.26 | 1.000 | 12.5 / 12.4 | 347 | 2.85 / 8.25 | 38 |
| MIPP04 | 28.81 | 4.14 | 2.50 | **2.44** | 2.44 | 2.13 | 1.0199 | 1.00 | 1.000 | 12.4 / 12.4 | 348 | 1.79 / 4.78 | 32 |
| MIPP05 | 18.31 | 3.76 | 2.84 | **1.93** | 1.96 | 1.89 | 1.0065 | 1.02 | 1.000 | 12.2 / 12.4 | 256 | 2.76 / 5.20 | 27 |
| MIPP06 | 28.33 | 3.94 | 2.86 | **2.75** | 2.78 | 2.66 | 1.0138 | 1.03 | 1.000 | 12.0 / 12.6 | 438 | 1.56 / 4.83 | 26 |
| MIPT01 | 10.81 | 3.81 | 1.52 | **1.51** | 1.50 | 0.96 | 1.0194 | 0.39 | 1.000 | 12.8 / 13.3 | 340 | 7.04 / 13.47 | 32 |
| MIPT02 | 19.87 | 3.61 | 2.05 | **2.08** | 2.09 | 1.45 | 1.0273 | 0.42 | 1.000 | 23.8 / 24.3 | 875 | 7.64 / 16.47 | 38 |
| MIPT03 | 40.02 | 4.24 | 2.70 | **2.57** | 2.57 | 2.22 | 1.0144 | 0.53 | 1.000 | 14.0 / 15.1 | 629 | 2.76 / 5.00 | 27 |
| MGO01 | 67.97 | 13.25 | 11.78 | **11.02** | 11.16 | 10.50 | 0.9604 | 2.32 | 1.000 | 20.5 / 19.3 | 149 | 1.26 / 2.14 | 56 |
| MGO02 | 55.49 | 8.35 | 7.52 | **6.12** | 6.12 | 6.06 | 0.9912 | 2.50 | 1.000 | 17.2 / 16.5 | 156 | 1.56 / 1.43 | 43 |
| MGO03 | 14.51 | 4.29 | 1.91 | **1.75** | 1.75 | 1.50 | 0.9916 | 0.99 | 1.000 | 20.2 / 24.5 | 229 | 1.66 / 4.04 | 45 |
| MGO04 | 26.18 | 7.49 | 5.58 | **4.93** | 4.95 | 4.93 | 0.9999 | 2.11 | 1.000 | 16.0 / 15.5 | 157 | 1.40 / 1.76 | 45 |
| MGO05 | 2.99 | 2.97 | 1.56 | **1.55** | 1.53 | 0.63 | 0.9839 | 0.34 | 1.000 | 27.1 / 19.6 | 144 | 3.98 / 6.34 | 35 |
| MGO06 | 11.09 | 3.78 | 2.10 | **1.92** | 1.92 | 1.58 | 0.9848 | 1.41 | 1.000 | 12.4 / 14.6 | 101 | 0.42 / 0.70 | 58 |
| MGO07 | 2.10 | 1.83 | 1.37 | **1.35** | 1.35 | 0.75 | 0.9858 | 0.44 | 0.999 | 30.4 / 27.9 | 71 | 0.48 / 0.94 | 74 |
| MGO08 | 2.75 | 2.77 | 5.97 | **2.37** | 2.37 | 1.89 | 0.9836 | 1.21 | 0.998 | 24.0 / 28.2 | 35 | 0.34 / 0.37 | 77 |
| MGO09 | 0.78 | 0.79 | 1.08 | **1.07** | 1.07 | 0.71 | 0.9726 | 0.69 | 0.988 | 25.6 / 27.3 | 5 | 0.02 / 0.01 | 79 |
| MGO10 | 0.83 | 0.86 | 1.37 | **1.37** | 1.37 | 0.83 | 0.9716 | 0.29 | 0.997 | 21.5 / 22.6 | 14 | 0.16 / 0.19 | 71 |
| MGO11 | 1.68 | 1.69 | 1.03 | **1.01** | 1.01 | 0.76 | 0.9872 | 0.44 | 0.997 | 27.7 / 26.9 | 24 | 0.22 / 0.27 | 68 |
| MGO12 | 60.10 | 6.76 | 6.00 | **5.07** | 5.06 | 5.04 | 0.9942 | 1.72 | 1.000 | 12.4 / 13.2 | 1722 | 3.31 / 25.89 | 28 |
| MGO13 | 68.35 | 8.08 | 8.02 | **6.96** | 6.96 | 6.32 | 0.9634 | 2.39 | 1.000 | 12.8 / 10.8 | 83 | 0.73 / 1.43 | 30 |
| MGO14 | 7.03 | 3.98 | 2.13 | **2.01** | 2.01 | 1.11 | 0.9817 | 0.91 | 1.000 | 19.8 / 16.7 | 80 | 1.54 / 2.10 | 35 |
| MGO15 | 4.16 | 5.92 | 2.64 | **2.61** | 3.13 | 2.53 | 0.9824 | 0.45 | 1.000 | 18.8 / 19.2 | 866 | 2.40 / 17.81 | 43 |
| MOO01 | 28.10 | 4.77 | 3.34 | **2.68** | 2.69 | 2.05 | 1.0200 | 1.31 | 1.000 | 6.5 / 7.4 | 52 | 1.13 / 1.98 | 32 |
| MOO02 | 23.77 | 4.19 | 4.72 | **2.43** | 2.43 | 2.43 | 1.0005 | 1.46 | 1.000 | 6.3 / 6.7 | 56 | 0.99 / 2.51 | 34 |
| MOO03 | 17.64 | 3.63 | 2.94 | **2.42** | 2.46 | 2.41 | 1.0022 | 1.12 | 1.000 | 6.7 / 6.8 | 47 | 0.63 / 1.05 | 37 |
| MOO04 | 6.53 | 4.26 | 4.53 | **3.36** | 3.40 | 3.31 | 1.0059 | 1.19 | 1.000 | 5.9 / 6.5 | 46 | 0.67 / 1.10 | 34 |
| MOO05 | 1.87 | 1.89 | 1.17 | **1.17** | 1.17 | 1.02 | 1.0066 | 0.29 | 1.000 | 6.8 / 7.2 | 34 | 0.68 / 0.96 | 33 |
| MOO06 | 5.63 | 3.08 | 2.31 | **1.57** | 1.57 | 1.38 | 1.0094 | 0.78 | 1.000 | 6.9 / 6.4 | 43 | 1.50 / 2.45 | 35 |
| MOO07 | 1.33 | 1.34 | 1.22 | **1.14** | 1.14 | 1.07 | 0.9950 | 0.31 | 0.999 | 6.1 / 6.1 | 12 | 0.20 / 0.26 | 29 |
| MOO08 | 2.77 | 2.92 | 2.84 | **2.89** | 2.89 | 2.70 | 0.9884 | 1.27 | 0.998 | 5.5 / 5.5 | 5 | 0.03 / 0.05 | 27 |
| MOO09 | 0.37 | 0.39 | 0.48 | **0.42** | 0.42 | 0.38 | 0.9922 | 0.30 | 0.989 | 6.5 / 6.4 | 1 | 0.01 / 0.01 | 30 |
| MOO10 | 0.99 | 1.02 | 1.42 | **1.41** | 1.41 | 1.36 | 0.9903 | 0.37 | 0.995 | 6.4 / 5.9 | 3 | 0.04 / 0.04 | 29 |
| MOO11 | 1.90 | 1.89 | 1.88 | **2.44** | 2.44 | 1.56 | 0.9695 | 0.45 | 0.998 | 5.9 / 6.1 | 4 | 0.08 / 0.09 | 29 |
| MOO12 | 68.91 | 9.28 | 9.84 | **8.78** | 8.78 | 8.72 | 1.0121 | 0.82 | 1.000 | 6.7 / 6.5 | 818 | 2.53 / 6.15 | 30 |
| MOO13 | 50.15 | 6.97 | 7.16 | **6.19** | 6.19 | 6.04 | 1.0175 | 1.73 | 1.000 | 6.3 / 5.9 | 43 | 0.77 / 1.69 | 29 |
| MOO14 | 11.30 | 4.70 | 2.67 | **2.45** | 2.45 | 2.36 | 1.0082 | 0.81 | 1.000 | 6.3 / 6.8 | 73 | 1.33 / 2.24 | 35 |
| MOO15 | 47.40 | 42.31 | 84.51 | **31.55** | 25.86 | 21.18 | 0.4341 | 3.29 | 1.000 | 16.8 / 11.8 | 327 | 3.25 / 14.92 | 42 |
| MOO16 | 2.45 | 2.13 | 1.72 | **3.26** | 1.46 | 2.52 | 0.5797 | 0.27 | 1.000 | 6.2 / 6.3 | 265 | 35.17 / 18.11 | 32 |
| EMH01 | 7.62 | 8.32 | 7.92 | **7.26** | 7.26 | 1.49 | 1.0168 | 0.49 | 1.000 | 25.9 / 18.9 | 79 | – | 38 |
| EMH02 | 4.87 | 11.27 | 5.67 | **5.83** | 5.82 | 1.51 | 1.0123 | 0.42 | 1.000 | 26.8 / 28.0 | 110 | 6.85 / 14.45 | 68 |
| EMH03 | 5.99 | 9.68 | 9.28 | **9.21** | 9.21 | 7.96 | 1.0131 | 1.34 | 1.000 | 25.3 / 20.7 | 75 | 4.73 / 7.37 | 64 |
| EMH04 | 12.05 | 19.53 | 14.81 | **12.55** | 12.56 | 7.13 | 1.0154 | 1.30 | 1.000 | 18.1 / 19.9 | 63 | 3.84 / 10.84 | 68 |
| EMH05 | 16.69 | 10.32 | 10.02 | **8.80** | 8.80 | 3.99 | 1.0115 | 1.04 | 1.000 | 16.4 / 17.1 | 61 | 1.24 / 4.03 | 64 |
| EV101 | 4.47 | 5.23 | 4.19 | **4.21** | 4.21 | 3.16 | 1.0153 | 1.30 | 1.000 | 9.8 / 8.6 | 62 | 2.58 / 8.40 | 53 |
| EV102 | 5.38 | 5.70 | 2.98 | **3.02** | 3.02 | 1.53 | 1.0148 | 1.15 | 1.000 | 7.0 / 6.8 | 20 | 0.48 / 0.82 | 39 |
| EV103 | 5.64 | 7.28 | 12.79 | **5.00** | 4.99 | 4.44 | 1.0149 | 1.60 | 1.000 | 8.8 / 7.9 | 34 | 1.11 / 1.23 | 31 |
| EV201 | 3.60 | 3.00 | 3.39 | **3.75** | 3.75 | 2.05 | 1.0139 | 0.47 | 1.000 | 20.0 / 14.7 | 45 | 4.42 / 13.31 | 34 |
| EV202 | 3.77 | 4.00 | 3.73 | **2.49** | 2.49 | 1.49 | 1.0095 | 0.79 | 1.000 | 7.9 / 8.5 | 29 | 0.98 / 1.90 | 33 |
| EV203 | 15.63 | 8.59 | 10.20 | **8.96** | 7.09 | 8.95 | 0.9979 | 3.24 | 1.000 | 6.9 / 7.3 | 22 | 0.59 / 0.66 | 33 |
| TC1 | 32.11 | 5.96 | 2.99 | **2.08** | 2.08 | 1.90 | 1.0122 | 0.93 | 1.000 | 7.6 / 5.7 | 67 | 8.83 / 20.85 | 29 |
| TC2 | 44.42 | 12.23 | 10.05 | **9.84** | 9.84 | 9.83 | 0.9961 | 3.23 | 1.000 | 7.9 / 7.5 | 90 | 3.88 / 8.68 | 29 |
| TC3 | 61.81 | 5.28 | 6.64 | **6.51** | 6.31 | 6.51 | 0.9996 | 2.67 | 1.000 | 6.7 / 7.0 | 86 | 6.55 / 30.91 | 33 |
| TC4 | 22.26 | 22.27 | 21.73 | **19.13** | 19.07 | 19.12 | 1.0088 | 2.83 | 1.000 | 6.4 / 6.9 | 42 | 4.47 / 7.91 | 36 |
| TC5 | 38.03 | 13.50 | 4.18 | **2.79** | 2.18 | 2.78 | 1.0024 | 0.46 | 1.000 | 6.2 / 8.6 | 91 | 105.44 / 98.73 | 33 |
| TM1 | 185.52 | 7.33 | 7.33 | **7.42** | 7.45 | 7.38 | 0.9917 | 0.81 | 1.000 | 11.7 / 15.3 | 345 | 87.83 / 140.09 | 50 |
| TM2 | 138.36 | 96.92 | 92.73 | **90.99** | 89.71 | 66.05 | 0.5375 | 10.94 | 1.000 | 11.3 / 11.3 | 176 | 104.19 / 163.36 | 44 |
| TM3 | 168.65 | 169.42 | 169.39 | **169.37** | 169.35 | 84.53 | 0.2658 | 20.43 | 1.000 | 9.5 / 10.1 | 155 | 6.28 / 11.62 | 44 |
| TM4 | 133.91 | 108.52 | 106.43 | **106.73** | 106.25 | 69.18 | 0.4368 | 10.23 | 1.000 | 9.5 / 22.5 | 352 | 122.60 / 308.95 | 42 |
| TM5 | 98.16 | 100.57 | 97.13 | **101.70** | 95.73 | 70.03 | 0.4830 | 11.23 | 1.000 | 18.5 / 9.6 | 131 | 89.08 / 225.88 | 43 |
| TM6 | 318.95 | 220.83 | 218.33 | **218.54** | 220.05 | 73.86 | 0.1433 | 21.82 | 1.000 | 6.8 / 15.1 | 235 | 45.50 / 48.79 | 46 |
| TR1 | 11.25 | 3.81 | 0.77 | **0.76** | 0.76 | 0.69 | 1.0032 | 0.51 | 1.000 | 6.4 / 7.3 | 35 | 1.26 / 2.24 | 33 |
| TR2 | 6.96 | 4.02 | 0.86 | **0.91** | 0.91 | 0.76 | 1.0042 | 0.62 | 1.000 | 7.6 / 7.2 | 35 | 1.12 / 3.05 | 33 |
| TR3 | 11.24 | 3.49 | 1.01 | **1.02** | 1.02 | 0.76 | 1.0054 | 0.56 | 1.000 | 5.9 / 6.4 | 32 | 0.98 / 2.70 | 34 |
| TR4 | 4.31 | 4.59 | 1.26 | **1.25** | 1.25 | 1.06 | 1.0062 | 0.40 | 1.000 | 6.2 / 7.4 | 36 | 0.91 / 2.60 | 32 |
| TR5 | 16.62 | 3.56 | 0.85 | **0.85** | 0.85 | 0.78 | 1.0036 | 0.59 | 1.000 | 5.8 / 6.9 | 37 | 0.78 / 1.87 | 33 |
| TR6 | 1.88 | 2.22 | 0.76 | **0.77** | 0.77 | 0.72 | 1.0027 | 0.29 | 1.000 | 6.5 / 7.0 | 30 | 0.67 / 2.25 | 30 |
| TS1 | 36.86 | 2.72 | 1.27 | **1.29** | 1.29 | 1.07 | 1.0075 | 0.51 | 1.000 | 14.8 / 7.4 | 67 | 75.31 / 131.20 | 35 |
| TS2 | 118.48 | 80.19 | 79.69 | **79.65** | 79.64 | 55.03 | 0.5573 | 9.87 | 1.000 | 11.6 / 11.7 | 87 | 11.15 / 20.02 | 48 |
| TS3 | 133.53 | 136.20 | 136.49 | **135.36** | 135.43 | 77.62 | 0.3176 | 15.13 | 1.000 | 12.8 / 11.8 | 128 | 62.12 / 108.36 | 50 |
