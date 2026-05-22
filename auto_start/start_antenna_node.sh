#!/usr/bin/env bash
set -e

SCRIPT_DIR=$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd_

while ! ip link show | grep -q "state UP"; do
  echo "[INFO] Waiting for active network interface..."
  sleep 1
done

ehco "[INFO] network interface is up"

echo "[INFO] starting antenna node"

if command -v nixos-rebuild; then
  echo "[INFO] running on NixOS"
else
  echo "[INFO] skill issue"
fi

source $SCRIPT_DIR/../install/setup.bash

ros2 run antenna_pkg antenna
