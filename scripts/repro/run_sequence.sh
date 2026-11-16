#!/usr/bin/env bash
# Run one evaluation sequence exactly like the branch CI job
# (.ci/cieval.template.yaml, ".run-dataset"), without Docker.
#
# Usage: scripts/repro/run_sequence.sh <SEQ_KEY> [OUT_ROOT]
#   SEQ_KEY  key in .ci/evaluation.json, e.g. EMH01, MGO07, MIO03
#   OUT_ROOT defaults to $RUNS_ROOT (default: <repo>/runs). Use a local disk:
#            network file systems can briefly deny access to files just written.
#
# Env:
#   DATA_ROOT      dataset root (default <repo>/datasets), laid out as
#                  euroc/<MH_01_easy...>, tum-vi/<dataset-room1_512_16...>,
#                  monado-slam-datasets/M_monado_datasets/{MG_reverb_g2,MI_valve_index,MO_odyssey_plus}/...
#   RUNS_ROOT      output root (default <repo>/runs)
#   DETERMINISTIC  default 1, NUM_THREADS default 4: as the CI accuracy evaluation
#                  (create-evaluation job in .gitlab-ci.yml)
#   CORES          CPU list for taskset (default 0-7: an 8-core cap)
#   VIO_EXTRA      extra basalt_vio args for ablations, e.g. "--deterministic-mapper 1"
#   SKIP_BSL=1     skip the plain-Basalt (loop closing off) run
# Must run inside the pixi default environment (pixi run -e default ...).
set -euo pipefail

SEQ="$1"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_ROOT="${2:-${RUNS_ROOT:-$ROOT/runs}}"
DATA="${DATA_ROOT:-$ROOT/datasets}"
CORES="${CORES:-0-7}"
DETERMINISTIC="${DETERMINISTIC:-1}"
NUM_THREADS="${NUM_THREADS:-4}"
VIO_EXTRA="${VIO_EXTRA:-}"  # extra basalt_vio args for ablations, e.g. "--deterministic-mapper 1"

EVAL_JSON="$ROOT/.ci/evaluation.json"
XRTMET="$ROOT/thirdparty/xrtslam-metrics"
VIO="$ROOT/build/basalt_vio"
BAL="$ROOT/thirdparty/rootba/bin/bal"

DS_DEV=$(jq -r ".sequences.$SEQ.device" "$EVAL_JSON")
DS_REL_PATH=$(jq -r ".sequences.$SEQ.path" "$EVAL_JSON")
DS_CONFIG="$ROOT/$(jq -r ".configs.$DS_DEV" "$EVAL_JSON")"
DS_CALIB="$ROOT/$(jq -r ".calibs.$DS_DEV" "$EVAL_JSON")"
DS_FORMAT=$(jq -r ".formats.$DS_DEV" "$EVAL_JSON")

# Docker mount points of the CI runner -> dataset roots under DATA_ROOT.
case "$DS_DEV" in
  euroc) DS_ROOT="$DATA/euroc" ;;
  tumvi) DS_ROOT="$DATA/tum-vi" ;;
  msdmg) DS_ROOT="$DATA/monado-slam-datasets/M_monado_datasets/MG_reverb_g2" ;;
  msdmi) DS_ROOT="$DATA/monado-slam-datasets/M_monado_datasets/MI_valve_index" ;;
  msdmo) DS_ROOT="$DATA/monado-slam-datasets/M_monado_datasets/MO_odyssey_plus" ;;
  *) echo "unknown device $DS_DEV" >&2; exit 2 ;;
esac
DS_PATH="$DS_ROOT/$DS_REL_PATH"
[ -d "$DS_PATH/mav0" ] || { echo "missing dataset $DS_PATH" >&2; exit 3; }

OUT="$OUT_ROOT/$SEQ"
rm -rf "$OUT"
mkdir -p "$OUT"
cd "$OUT"

{
  echo "seq=$SEQ dev=$DS_DEV path=$DS_PATH"
  echo "config=$DS_CONFIG calib=$DS_CALIB format=$DS_FORMAT"
  echo "deterministic=$DETERMINISTIC num_threads=$NUM_THREADS vio_extra=$VIO_EXTRA"
  echo "basalt=$(git -C "$ROOT" rev-parse --short HEAD) rootba=$(git -C "$ROOT/thirdparty/rootba" rev-parse --short HEAD)"
  echo "start=$(date -Is) load_start=$(cut -d' ' -f1-3 /proc/loadavg)"
} > run_info.txt

# 1. Online VIO + loop closure, exports map/map.json and map/pose_graph.json.
#    8 cores for the whole process (COMMON-BRIEF), like the upstream Docker example.
T0=$(date +%s.%N)
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c "$CORES" "$VIO" --save-times 1 --show-gui 0 \
  --deterministic "$DETERMINISTIC" --num-threads "$NUM_THREADS" \
  --dataset-path "$DS_PATH" --dataset-type "$DS_FORMAT" \
  --cam-calib "$DS_CALIB" --config-path "$DS_CONFIG" \
  --save-trajectory euroc --save-trajectory-fn "tracking.$SEQ.csv" \
  --map-export-path map $VIO_EXTRA > "output.$SEQ.log" 2>&1 || echo "basalt_vio exit $?" >> run_info.txt
T1=$(date +%s.%N)
echo "vio_wall_s=$(echo "$T1 - $T0" | bc) load_after_vio=$(cut -d' ' -f1-3 /proc/loadavg)" >> run_info.txt

# 1b. (skipped when SKIP_BSL=1, for ablations) Paper's BSL column (Table I): plain Basalt VIO, same config with
#     config.enable_loop_closing=false. Not part of the CI template.
mkdir -p bsl
if [ "${SKIP_BSL:-0}" != 1 ]; then
jq '.value0["config.enable_loop_closing"] = false' "$DS_CONFIG" > bsl/config_bsl.json
T0=$(date +%s.%N)
( cd bsl && OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c "$CORES" "$VIO" --save-times 1 --show-gui 0 \
  --deterministic "$DETERMINISTIC" --num-threads "$NUM_THREADS" \
  --dataset-path "$DS_PATH" --dataset-type "$DS_FORMAT" \
  --cam-calib "$DS_CALIB" --config-path config_bsl.json \
  --save-trajectory euroc --save-trajectory-fn "tracking.$SEQ.csv" > "output.$SEQ.log" 2>&1 ) \
  || echo "basalt_vio bsl exit $?" >> run_info.txt
T1=$(date +%s.%N)
echo "bsl_wall_s=$(echo "$T1 - $T0" | bc) load_after_bsl=$(cut -d' ' -f1-3 /proc/loadavg)" >> run_info.txt
fi

# 2. Global square-root BA (rootba), three CI variants.
run_bal() {  # name, extra args...
  local name="$1"; shift
  local t0 t1
  t0=$(date +%s.%N)
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c "$CORES" "$BAL" --input map/map.json --solver-type SQUARE_ROOT \
    --calibration-file "$DS_CALIB" --no-normalize --residual-robust-norm HUBER \
    --optimized-cost ERROR_VALID "$@" > "output.$SEQ.$name.log" 2>&1 || echo "bal $name exit $?" >> run_info.txt
  t1=$(date +%s.%N)
  [ -f ba_log.json ] && mv ba_log.json "ba_log.$name.json"  # per-iteration cost / time
  echo "${name}_wall_s=$(echo "$t1 - $t0" | bc) load=$(cut -d' ' -f1-3 /proc/loadavg)" >> run_info.txt
}
run_bal bal   --no-use-double --output-optimized-path map/optimized_tracking.csv --min-obs-per-kf 20
run_bal balpgd --use-double   --output-optimized-path map/optimized_tracking_pg_double.csv --min-obs-per-kf 10 --pose-graph-file map/pose_graph.json
run_bal balpgf --no-use-double --output-optimized-path map/optimized_tracking_pg_float.csv --min-obs-per-kf 10 --pose-graph-file map/pose_graph.json

# 3. Per-frame trajectories (keyframe poses interpolated into all frames).
MERGE="python -I $XRTMET/ops/merge_frames_keyframes.py"
mkdir -p eval
cp "bsl/rt_tracking.$SEQ.csv" eval/bsl.csv || true      # BSL: plain Basalt VIO (causal)
cp "rt_tracking.$SEQ.csv" eval/lc_rt.csv || true          # VIO+LC, causal real-time output
$MERGE "rt_tracking.$SEQ.csv" map/kf_trajectory.csv eval/lc.csv || true
$MERGE "rt_tracking.$SEQ.csv" map/optimized_tracking.csv eval/ba.csv || true
$MERGE "rt_tracking.$SEQ.csv" map/optimized_tracking_pg_double.csv eval/bapgd.csv || true
$MERGE "rt_tracking.$SEQ.csv" map/optimized_tracking_pg_float.csv eval/bapgf.csv || true
echo "end=$(date -Is)" >> run_info.txt
echo "done $OUT"
