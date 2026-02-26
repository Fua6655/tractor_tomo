#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="tomo-power-orchestrator.service"
REPO_DIR="${TOMO_REPO_DIR:-$HOME/tractor_tomo}"
SRC="$REPO_DIR/scripts/raspberry/$SERVICE_NAME"
DST="/etc/systemd/system/$SERVICE_NAME"

if [[ ! -f "$SRC" ]]; then
  echo "Missing service file: $SRC" >&2
  exit 1
fi

chmod +x "$REPO_DIR/scripts/raspberry/run_power_orchestrator.sh"
chmod +x "$REPO_DIR/scripts/raspberry/power_orchestrator.py"

echo "Installing $SERVICE_NAME"
sudo cp "$SRC" "$DST"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo "Service status:"
sudo systemctl --no-pager --full status "$SERVICE_NAME" || true
