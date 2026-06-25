#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/orangepi/moonxkj/mine_pedestrian"
SERVICE_NAME="mine_pedestrian"
SERVICE_FILE="$PROJECT_DIR/scripts/mine_pedestrian.service"
SYSTEMD_PATH="/etc/systemd/system/${SERVICE_NAME}.service"

echo "=== Mine Pedestrian Auto-Start Setup ==="
echo ""

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script needs root to install systemd service."
  echo "Running: sudo $0"
  sudo "$0" "$@"
  exit $?
fi

echo "Installing systemd service..."
cp "$SERVICE_FILE" "$SYSTEMD_PATH"
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"

echo ""
echo "Service installed successfully."
echo ""
echo "Commands:"
echo "  sudo systemctl start  $SERVICE_NAME    # Start now"
echo "  sudo systemctl stop   $SERVICE_NAME    # Stop"
echo "  sudo systemctl status $SERVICE_NAME    # Check status"
echo "  sudo systemctl restart $SERVICE_NAME   # Restart"
echo ""
echo "The service will auto-start on next boot."
echo ""
read -rp "Start the service now? [Y/n] " answer
if [[ "$answer" =~ ^[Yy]?$ ]]; then
  systemctl start "$SERVICE_NAME"
  sleep 2
  systemctl status "$SERVICE_NAME" --no-pager || true
fi
