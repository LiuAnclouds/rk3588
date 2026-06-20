#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
cd "$PROJECT_DIR"

mkdir -p web logs

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"
MODEL="${MODEL:-$PROJECT_DIR/models/current_seg.rknn}"
MJPEG_PORT="${MJPEG_PORT:-8090}"
WRITE_SNAPSHOT="${WRITE_SNAPSHOT:-0}"

if pgrep -f "$PROJECT_DIR/bin/mine_live_infer" >/dev/null; then
  echo "mine_live_infer already running"
else
  nohup "$PROJECT_DIR/bin/mine_live_infer" 0 \
    "$PROJECT_DIR/web/live_result.jpg" \
    "$PROJECT_DIR/web/live_targets.json" \
    "$MODEL" \
    "$MJPEG_PORT" \
    "$WRITE_SNAPSHOT" \
    > "$PROJECT_DIR/logs/live_infer.log" 2>&1 < /dev/null &
  live_pid=$!
  disown "$live_pid" 2>/dev/null || true
  echo "started live infer pid=$live_pid"
fi

if pgrep -f "python3 -m http.server 8080" >/dev/null; then
  echo "web server already running"
else
  cd "$PROJECT_DIR/web"
  nohup python3 -m http.server 8080 --bind 0.0.0.0 \
    > "$PROJECT_DIR/logs/web.log" 2>&1 < /dev/null &
  web_pid=$!
  disown "$web_pid" 2>/dev/null || true
  cd "$PROJECT_DIR"
  echo "started web server on :8080 pid=$web_pid"
fi

echo "Open: http://192.168.1.5:8080/"
echo "Stream: http://192.168.1.5:${MJPEG_PORT}/stream"
