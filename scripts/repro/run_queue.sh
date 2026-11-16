#!/usr/bin/env bash
# Run run_sequence.sh + evaluate.py over a list of evaluation.json keys.
#
# Usage (inside the pixi default env): scripts/repro/run_queue.sh KEY [KEY ...]
#
# Several workers can share one key list (start the same command 2-3 times):
# each key is claimed atomically with mkdir $RUNS_ROOT/.claims/<KEY>. All
# workers stay pinned to the same CORES (run_sequence.sh), so the CPU cap holds;
# one run keeps ~1.5 cores busy on average.
#
# Env: RUNS_ROOT (default <repo>/runs), DATA_ROOT, CORES and the other
# run_sequence.sh variables. Keys whose dataset is missing are skipped.
# Log: $RUNS_ROOT/queue.log. Stop after the current sequence: touch $RUNS_ROOT/STOP.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNS_ROOT="${RUNS_ROOT:-$ROOT/runs}"
export RUNS_ROOT
CLAIMS="$RUNS_ROOT/.claims"
LOG="$RUNS_ROOT/queue.log"
mkdir -p "$CLAIMS"

for key in "$@"; do
  [ -e "$RUNS_ROOT/STOP" ] && { echo "$(date -Is) stop file, exiting" >> "$LOG"; break; }
  [ -e "$RUNS_ROOT/$key/eval/metrics.json" ] && continue      # done
  mkdir "$CLAIMS/$key" 2>/dev/null || continue                  # another worker has it
  echo "$(date -Is) $key start load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$LOG"
  if bash "$ROOT/scripts/repro/run_sequence.sh" "$key" > "$RUNS_ROOT/$key.out" 2>&1; then
    python "$ROOT/scripts/repro/evaluate.py" "$RUNS_ROOT/$key" >> "$LOG" 2>&1
    echo "$(date -Is) $key done" >> "$LOG"
  else
    echo "$(date -Is) $key FAILED (see $RUNS_ROOT/$key.out; exit 3 = dataset missing)" >> "$LOG"
  fi
done
echo "$(date -Is) queue finished" >> "$LOG"
