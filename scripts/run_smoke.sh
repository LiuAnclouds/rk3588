#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
cd "$PROJECT_DIR"

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"
exec "$PROJECT_DIR/bin/rknn_smoke" \
  "${MODEL:-$PROJECT_DIR/models/current_seg.rknn}"
