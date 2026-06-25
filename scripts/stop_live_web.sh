#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
pkill -f "$PROJECT_DIR/bin/mine_live_infer" 2>/dev/null || true
pkill -f "python3 -m http.server" 2>/dev/null || true
pkill -f "mine_rs485.py" 2>/dev/null || true
echo "stopped live inference, web server, and RS485 transmitter"
