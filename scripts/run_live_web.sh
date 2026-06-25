#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
CONFIG_FILE="$PROJECT_DIR/config/mine_config.conf"
cd "$PROJECT_DIR"

mkdir -p web logs

# Load config if exists, else use defaults/env
if [[ -f "$CONFIG_FILE" ]]; then
  source "$CONFIG_FILE"
fi

export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:$PROJECT_DIR/lib:${LD_LIBRARY_PATH:-}"
MODEL="${MODEL:-$PROJECT_DIR/models/current_seg.rknn}"
MJPEG_PORT="${MJPEG_PORT:-8090}"
WEB_PORT="${WEB_PORT:-8080}"
CAMERA_INDEX="${CAMERA_INDEX:-0}"
WRITE_SNAPSHOT="${WRITE_SNAPSHOT:-0}"

if pgrep -f "$PROJECT_DIR/bin/mine_live_infer" >/dev/null; then
  echo "mine_live_infer already running"
else
  nohup "$PROJECT_DIR/bin/mine_live_infer" "$CAMERA_INDEX" \
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

if pgrep -f "python3 -m http.server $WEB_PORT" >/dev/null; then
  echo "web server already running"
else
  cd "$PROJECT_DIR/web"
  nohup python3 -m http.server "$WEB_PORT" --bind 0.0.0.0 \
    > "$PROJECT_DIR/logs/web.log" 2>&1 < /dev/null &
  web_pid=$!
  disown "$web_pid" 2>/dev/null || true
  cd "$PROJECT_DIR"
  echo "started web server on :$WEB_PORT pid=$web_pid"
fi

# Display URLs using configured IP
STREAM_IP="${STREAM_IP:-192.168.1.5}"
echo "Open:     http://${STREAM_IP}:${WEB_PORT}/"
echo "Stream:   http://${STREAM_IP}:${MJPEG_PORT}/stream"
echo "Set IP?   http://${STREAM_IP}:${WEB_PORT}/?ip=${STREAM_IP}&port=${MJPEG_PORT}"
