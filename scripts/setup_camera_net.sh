#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-enP4p65s0}"

sudo ip link set "$IFACE" up

if ! ip -4 addr show dev "$IFACE" | grep -q "192.168.10.1/24"; then
  sudo ip addr add 192.168.10.1/24 dev "$IFACE"
fi

ip -br addr show "$IFACE"
