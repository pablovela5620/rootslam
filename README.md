# RootSLAM (authors' work branch) + pixi build and reproduction

> **Credit.** RootSLAM is by Tomás Santucci, Mateo de Mayo, Daniel Cremers and Taihú Pire:
> *RootSLAM: Square Root Bundle Adjustment for Visual-Inertial SLAM*, [arXiv 2610.15157](https://arxiv.org/abs/2610.15157).
> The code here is the authors' **public work branch** [`tsantucci/loop-closure`](https://gitlab.freedesktop.org/mateosss/basalt/-/tree/tsantucci/loop-closure)
> of [Basalt for Monado](https://gitlab.freedesktop.org/mateosss/basalt) (itself a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt)),
> with the global optimizer in [TomasSantucci/rootba](https://github.com/TomasSantucci/rootba) (branch `fixed-intrinsics`).
> The paper says the official code comes "upon acceptance". **Prefer the official repository once it exists.**
>
> This branch (`pixi`) only adds a [pixi](https://pixi.sh) build from conda-forge and scripts that reproduce the paper's Table I.
> It does not change the authors' code. The upstream licence (BSD-3-Clause, `LICENSE`) applies unchanged.

## Reproduce with pixi

Linux x86-64 only. `pixi` must be installed; everything else comes from conda-forge.

```bash
git clone --recursive -b pixi https://github.com/pablovela5620/rootslam.git && cd rootslam
pixi run build              # basalt_vio (this repo) + bal (thirdparty/rootba); ~10 min on 8 cores
pixi run build-metrics-cpp  # xrtslam-metrics' C++ alignment module
```

`pixi run build` applies `patches/rootba-pangolin-no-ffmpeg.patch` to the rootba submodule (Pangolin's ffmpeg driver does not build
against conda-forge's ffmpeg 8; `bal` does not use it). rootba's GUI tool `bal_gui` does not link with the conda-forge sysroot; the
pipeline does not need it.

### Data

Put the datasets under `DATA_ROOT` (default `./datasets`) in this layout, each sequence in EuRoC/ASL format (`<seq>/mav0/...`):

```
$DATA_ROOT/euroc/MH_01_easy ... V2_03_difficult
$DATA_ROOT/tum-vi/dataset-room1_512_16 ...             (TUM-VI, 512x512, 16-bit, EuRoC export)
$DATA_ROOT/monado-slam-datasets/M_monado_datasets/{MG_reverb_g2,MI_valve_index,MO_odyssey_plus}/...
```

- **EuRoC**: [ETH Research Collection](https://www.research-collection.ethz.ch/handle/20.500.11850/690084) (`machine_hall.zip`,
  `vicon_room1.zip`, `vicon_room2.zip`, which contain one zip per sequence). A public mirror of the same zips is on Hugging Face:
  `GlowBond/EuRoC_MAV_Dataset`.
- **Monado SLAM datasets (MSD)**: Hugging Face [`collabora/monado-slam-datasets`](https://huggingface.co/datasets/collabora/monado-slam-datasets), e.g.
  `hf download collabora/monado-slam-datasets --repo-type dataset --include "M_monado_datasets/MG_reverb_g2/MGO_others/*" --local-dir $DATA_ROOT/monado-slam-datasets`,
  then unzip each sequence next to its zip. A few long sequences (e.g. MGO12, MOO12) are split zips: join them with `zip -s 0 X.zip --out X-joined.zip` first.
- **TUM-VI**: `https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-<name>_512_16.tar` (room1-6, corridor1-5, magistrale1-6, slides1-3).

Ground truth is read from each sequence (`mav0/state_groundtruth_estimate0`, `mav0/gt`, `mav0/mocap0`).

### Run, score, tabulate

```bash
export DATA_ROOT=/path/to/datasets RUNS_ROOT=/path/to/runs   # RUNS_ROOT default ./runs; use a local disk
pixi run repro-seq EMH01                 # one sequence: VIO+LC, plain Basalt, 3 x bal, per-frame trajectories
pixi run repro-eval $RUNS_ROOT/EMH01     # SE(3) / Sim(3) ATE and RTE for every column
pixi run repro-queue MGO01 MGO02 ...     # many sequences; start it 2-3 times to share one 8-core set
pixi run repro-summary                   # results/summary.md: ours vs paper per group, All*, per sequence
```

The sequence keys are those of `.ci/evaluation.json` (EMH01-05, EV101-203, MGO01-15, MIO01-16, MIPB/MIPP/MIPT, MOO01-16, TC/TM/TR/TS).
`scripts/repro/run_sequence.sh` documents the knobs (`CORES`, `NUM_THREADS`, `VIO_EXTRA`, `SKIP_BSL`).
`scripts/repro/log_rerun.py` logs one run's intermediates (frontend tracks, keyframes, loops, pose graph before/after, BA iterations,
trajectories vs GT) to a Rerun recording (`rerun-sdk` 0.42 is in the pixi env).

### Expected numbers

RMSE ATE [cm], SE(3), one run per sequence (our full run: [REPRO.md](REPRO.md)):

| | BSL (Basalt) | RootSLAM | paper BSL | paper RootSLAM |
|---|---|---|---|---|
| All* (95 sequences) | 33.5 | **14.1** | 34.8 | **12.4** |
| MSD MG / MIP / MO | 21.7 / 23.6 / 16.9 | 3.4 / 2.4 / 4.6 | 21.9 / 23.6 / 19.0 | 3.4 / 2.3 / 4.6 |
| EuRoC EMH / EV1 / EV2 | 9.4 / 5.2 / 7.7 | 8.7 / 4.1 / 5.1 | 9.1 / 5.1 / 7.8 | 7.0 / 3.8 / 5.8 |

Loop closing varies from run to run, so single sequences can move by several cm; [REPRO.md](REPRO.md) explains the gaps (EMH, MIO, TUM-VI magistrale).

---

# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) improved
for tracking XR devices with
[Monado](https://gitlab.freedesktop.org/monado/monado). Many thanks to the
Basalt authors.

## Installation

- **Prebuilt (Ubuntu/Raspberry/Radxa)**: Download [latest .deb](https://gitlab.freedesktop.org/mateosss/basalt/-/releases) and install with

  ```bash
  sudo apt install -y ./basalt-monado-*.deb
  ```

- **From source (Linux)**

  ```bash
  git clone --recursive https://gitlab.freedesktop.org/mateosss/basalt.git
  cd basalt && ./scripts/install_deps.sh
  cmake --preset library # use "development" instead of "library" if you want extra binaries and debug symbols
  sudo cmake --build build --target install
  ```

- **From source (Windows)**: See the [build guide](doc/monado/Windows.md) for Windows.

## Usage

If you want to run OpenXR application with Monado, you need to set the
environment variable `VIT_SYSTEM_LIBRARY_PATH` to the path of the basalt library.

By default, Monado will try to load the library from `/usr/lib/libbasalt.so` if
the environment variable is not set.

If you want to test whether everything is working you can download a short dataset with [EuRoC (ASL) format](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format like [`MOO09_short_1_updown`](https://huggingface.co/datasets/collabora/monado-slam-datasets/resolve/main/M_monado_datasets/MO_odyssey_plus/MOO_others/MOO09_short_1_updown.zip?download=true) from the [Monado SLAM datasets](https://huggingface.co/datasets/collabora/monado-slam-datasets):

```bash
wget https://huggingface.co/datasets/collabora/monado-slam-datasets/resolve/main/M_monado_datasets/MO_odyssey_plus/MOO_others/MOO09_short_1_updown.zip
unzip MOO09_short_1_updown.zip
```

- **Try it standalone with a dataset (requires extra binaries)**

  ```bash
  basalt_vio --show-gui 1 --dataset-path MOO09_short_1_updown/ --dataset-type euroc --cam-calib /usr/share/basalt/msdmo_calib.json --config-path /usr/share/basalt/msdmo_config.json
  ```

- **Use a RealSense camera without Monado (requires extra binaries)**
  You'll need to calibrate your camera if you want the best results but meanwhile you can try with these calibration files instead.

  - RealSense D455 (and maybe also D435)

    ```bash
    basalt_rs_t265_vio --is-d455 --cam-calib /usr/share/basalt/d455_calib.json --config-path /usr/share/basalt/default_config.json
    ```

  - Realsense T265: Get t265_calib.json from [this issue](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and run

    ```bash
    basalt_rs_t265_vio --cam-calib t265_calib.json --config-path /usr/share/basalt/default_config.json
    ```

- **Try it through `monado-cli` with a dataset**

  ```bash
  monado-cli slambatch MOO09_short_1_updown/ /usr/share/basalt/msdmo.toml results
  ```

- **Try it with `monado`, a dataset, and an OpenXR app**

  ```bash
  # Run monado-service with a fake "euroc device" driver
  export EUROC_PATH=MOO09_short_1_updown/ # dataset path
  export EUROC_HMD=false # false for controller tracking
  export EUROC_PLAY_FROM_START=true # produce samples right away
  export SLAM_CONFIG=/usr/share/basalt/msdmo.toml # includes calibration
  export SLAM_SUBMIT_FROM_START=true # consume samples right away
  export XRT_DEBUG_GUI=1 # enable monado debug ui
  monado-service &

  # Get and run a sample OpenXR application
  wget https://gitlab.freedesktop.org/wallbraker/apps/-/raw/main/VirtualGround-x86_64.AppImage
  chmod +x VirtualGround-x86_64.AppImage
  ./VirtualGround-x86_64.AppImage normal
  ```

- **Use a real device in Monado**.

  When using a real device driver you might want to enable the `XRT_DEBUG_GUI=1` and `SLAM_UI=1` environment variables to show debug GUIs of Monado and Basalt respectively.

  Monado has a couple of drivers supporting SLAM tracking (and thus Basalt). Most of them should work without any user input.

  - WMR ([troubleshoot](doc/monado/WMR.md))
  - Rift S (might need to press "Submit to SLAM", like the Vive Driver).
  - Northstar / DepthAI ([This hand-tracking guide](https://monado.freedesktop.org/handtracking) has a depthai section).
  - Vive Driver (Valve Index) ([read before using](doc/monado/Vive.md))
  - RealSense Driver ([setup](doc/monado/Realsense.md)).

## Development

If you want to set up your build environment for developing and iterating on Basalt, see the [development guide](doc/Development.md).
