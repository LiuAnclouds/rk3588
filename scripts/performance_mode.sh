#!/usr/bin/env bash
set -euo pipefail

echo "Setting CPU governors to performance..."
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [ -w "$f" ] && echo performance > "$f" || sudo sh -c "echo performance > '$f'"
done

echo "Setting devfreq governors to performance where supported..."
for d in /sys/class/devfreq/*; do
  [ -d "$d" ] || continue
  [ -f "$d/available_governors" ] || continue
  [ -f "$d/governor" ] || continue
  if grep -qw performance "$d/available_governors"; then
    if [ -w "$d/governor" ]; then
      echo performance > "$d/governor"
    else
      sudo sh -c "echo performance > '$d/governor'"
    fi
  fi
done

echo "Current CPU governors:"
for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  printf "%s=" "$f"
  cat "$f"
done

echo "Current devfreq state:"
for d in /sys/class/devfreq/*; do
  [ -d "$d" ] || continue
  echo "--- $d"
  [ -f "$d/name" ] && cat "$d/name"
  [ -f "$d/governor" ] && cat "$d/governor"
  [ -f "$d/cur_freq" ] && cat "$d/cur_freq"
done
