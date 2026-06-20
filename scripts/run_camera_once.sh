#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
FRAME="$PROJECT_DIR/outputs/camera_frame.jpg"
RESULT="$PROJECT_DIR/outputs/camera_result.jpg"

cd "$PROJECT_DIR"
mkdir -p outputs

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"

"$PROJECT_DIR/bin/mvs_capture" "$FRAME" "${1:-0}" "${2:-3000}"
"$PROJECT_DIR/scripts/run_image.sh" "$FRAME" "$RESULT"

echo "frame=$FRAME"
echo "result=$RESULT"
