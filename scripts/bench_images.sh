#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
MODEL="${MODEL:-$PROJECT_DIR/models/current_seg.rknn}"
IMAGE_DIR="${1:-$PROJECT_DIR/test_images}"
OUTPUT_DIR="${2:-$PROJECT_DIR/docs/images}"
REPEATS="${3:-1}"

cd "$PROJECT_DIR"
mkdir -p "$OUTPUT_DIR"

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"
exec "$PROJECT_DIR/bin/mine_live_infer" --bench "$MODEL" "$IMAGE_DIR" "$OUTPUT_DIR" "$REPEATS"
