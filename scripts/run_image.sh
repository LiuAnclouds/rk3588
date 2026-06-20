#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <input_image> [output_image]" >&2
  exit 1
fi

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
MODEL="${MODEL:-$PROJECT_DIR/models/current_seg.rknn}"
INPUT="$1"
OUTPUT="${2:-$PROJECT_DIR/outputs/result.jpg}"

cd "$PROJECT_DIR"
mkdir -p "$(dirname "$OUTPUT")"

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"
exec "$PROJECT_DIR/bin/mine_yolov8_seg" "$MODEL" "$INPUT" "$OUTPUT"
