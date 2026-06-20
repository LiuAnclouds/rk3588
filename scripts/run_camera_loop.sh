#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
CAMERA_INDEX="${1:-0}"
TIMEOUT_MS="${2:-5000}"
SLEEP_SEC="${3:-0.2}"

FRAME="$PROJECT_DIR/outputs/live_frame.jpg"
RESULT="$PROJECT_DIR/outputs/live_result.jpg"
LOG="$PROJECT_DIR/outputs/live_targets.jsonl"

cd "$PROJECT_DIR"
mkdir -p outputs

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"

echo "camera_index=$CAMERA_INDEX timeout_ms=$TIMEOUT_MS sleep_sec=$SLEEP_SEC"
echo "frame=$FRAME"
echo "result=$RESULT"
echo "log=$LOG"
echo "Press Ctrl+C to stop."

while true; do
  ts="$(date '+%Y-%m-%dT%H:%M:%S%z')"
  "$PROJECT_DIR/bin/mvs_capture" "$FRAME" "$CAMERA_INDEX" "$TIMEOUT_MS" >/tmp/mvs_capture_live.log
  infer_out="$("$PROJECT_DIR/scripts/run_image.sh" "$FRAME" "$RESULT")"
  printf '{"timestamp":"%s","output":%s}\n' "$ts" "$(printf '%s' "$infer_out" | sed -n '/^{ /,/^] }/p' | tr '\n' ' ')" >> "$LOG"
  printf '%s\n' "$infer_out" | tail -n 8
  sleep "$SLEEP_SEC"
done
