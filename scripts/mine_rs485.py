#!/usr/bin/env python3
"""
Mine Pedestrian RS485 Transmitter
Reads live_targets.json and sends person detection status via serial (USB-to-485).
Supports simulation mode when no hardware serial device is available.

Protocol: $PED,<frame_id>,<detected>*<checksum>\r\n
  detected = 1 (person present) or 0 (clear)
"""

import argparse
import json
import os
import signal
import sys
import time
from datetime import datetime


def checksum_nmea(data: str) -> str:
    c = 0
    for ch in data:
        c ^= ord(ch)
    return f"{c:02X}"


def format_frame(frame_id: int, detected: bool) -> str:
    d = "1" if detected else "0"
    payload = f"PED,{frame_id},{d}"
    return f"${payload}*{checksum_nmea(payload)}\r\n"


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def open_serial(device: str, baud: int):
    try:
        import serial
        ser = serial.Serial(device, baudrate=baud, timeout=0.5, write_timeout=0.5)
        return ser, None
    except ImportError:
        return None, "pyserial not installed"
    except Exception as e:
        return None, str(e)


def run_loop(json_path: str, device: str, baud: int, interval: float = 0.1):
    print(f"[{timestamp()}] Mine RS485 TX starting: device={device} baud={baud}")
    print(f"[{timestamp()}] JSON source: {json_path}")

    running = True
    last_frame_id = -1

    def handle_signal(signum, frame):
        nonlocal running
        print(f"\n[{timestamp()}] Received signal {signum}, stopping...")
        running = False

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    ser, ser_error = open_serial(device, baud)
    if ser is not None:
        print(f"[{timestamp()}] Serial port {device} opened at {baud} baud")
        mode = "LIVE"
    elif device and os.path.exists(device):
        print(f"[{timestamp()}] Serial open failed: {ser_error}, simulation mode")
        mode = "SIM"
    else:
        print(f"[{timestamp()}] Device {device} not present, simulation mode")
        mode = "SIM"

    reconnect_interval = 5.0
    last_reconnect_attempt = 0.0

    while running:
        # Reconnect serial if needed
        if ser is None and os.path.exists(device):
            now = time.time()
            if now - last_reconnect_attempt >= reconnect_interval:
                ser, ser_error = open_serial(device, baud)
                if ser is not None:
                    print(f"[{timestamp()}] Serial reconnected: {device}")
                    mode = "LIVE"
                last_reconnect_attempt = now

        # Read JSON
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except FileNotFoundError:
            time.sleep(interval)
            continue
        except json.JSONDecodeError:
            time.sleep(interval)
            continue

        frame_id = data.get("frame_id", -1)
        if frame_id == last_frame_id:
            time.sleep(interval)
            continue
        last_frame_id = frame_id

        # Build frame: just detected yes/no
        targets = data.get("targets", [])
        detected = len(targets) > 0
        total = len(targets)
        msg = format_frame(frame_id, detected)

        ts = timestamp()
        if ser is not None:
            try:
                ser.write(msg.encode("ascii", errors="replace"))
                ser.flush()
                print(f"[{ts}] TX -> {msg.strip()}  (targets={total})")
            except Exception as e:
                print(f"[{ts}] Serial write error: {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                mode = "SIM"
        else:
            tag = "SIM" if mode == "SIM" else "TX"
            print(f"[{ts}] {tag} -> {msg.strip()}  (targets={total})")

        time.sleep(interval)

    if ser is not None:
        try:
            ser.close()
            print(f"[{timestamp()}] Serial port closed")
        except Exception:
            pass
    print(f"[{timestamp()}] RS485 TX stopped.")


def main():
    parser = argparse.ArgumentParser(description="Mine Pedestrian RS485 Transmitter")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", required=True)
    parser.add_argument("--interval", type=float, default=0.1)
    args = parser.parse_args()

    run_loop(args.json, args.device, args.baud, args.interval)


if __name__ == "__main__":
    main()
