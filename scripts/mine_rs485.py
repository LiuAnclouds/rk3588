#!/usr/bin/env python3
"""
Mine Pedestrian RS485 Transmitter
Reads live_targets.json and sends person detection data via serial (USB-to-485).
Supports simulation mode when no hardware serial device is available.
"""

import argparse
import json
import os
import signal
import sys
import time
from datetime import datetime

# Track last successful write time for health monitoring
_last_write_ok = time.time()


def checksum_nmea(data: str) -> str:
    """Compute NMEA-style XOR checksum."""
    c = 0
    for ch in data:
        c ^= ord(ch)
    return f"{c:02X}"


def format_pedestrian_frame(target: dict, frame_id: int) -> str:
    """Format a single pedestrian detection into the RS485 protocol frame."""
    bbox = target.get("bbox", [0, 0, 0, 0])
    fp = target.get("footpoint_px", [0, 0])
    payload = (
        f"PED,{frame_id},{target.get('id', 0)},"
        f"{target.get('confidence', 0):.3f},"
        f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]},"
        f"{fp[0]},{fp[1]},"
        f"{target.get('bearing_deg', 0):.1f},"
        f"{target.get('bearing_zone', '')},"
        f"{target.get('range_zone', '')}"
    )
    return f"${payload}*{checksum_nmea(payload)}\r\n"


def format_all_targets(data: dict) -> list:
    """Format all targets from a live_targets.json frame."""
    frames = []
    frame_id = data.get("frame_id", 0)
    for t in data.get("targets", []):
        frames.append(format_pedestrian_frame(t, frame_id))
    if not data.get("targets"):
        ck = checksum_nmea(f"PED,{frame_id},0,0,,,,,,,")
        frames.append(f"$PED,{frame_id},0,0,,,,,,,*{ck}\r\n")
    return frames


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def open_serial(device: str, baud: int):
    """Attempt to open serial port. Returns (serial_obj, error_msg)."""
    try:
        import serial
        ser = serial.Serial(device, baudrate=baud, timeout=0.5, write_timeout=0.5)
        return ser, None
    except ImportError:
        return None, "pyserial not installed"
    except Exception as e:
        return None, str(e)


def run_loop(json_path: str, device: str, baud: int, interval: float = 0.1,
             rate_limit_hz: float = 0):
    """Main loop: poll JSON file and send detections via serial."""
    print(f"[{timestamp()}] Mine RS485 TX starting: device={device} baud={baud}")
    print(f"[{timestamp()}] JSON source: {json_path}")

    running = True
    last_frame_id = -1
    last_send_time = 0.0

    def handle_signal(signum, frame):
        nonlocal running
        print(f"\n[{timestamp()}] Received signal {signum}, stopping...")
        running = False

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    # Open serial (may be None if not available)
    ser, ser_error = open_serial(device, baud)
    if ser is not None:
        print(f"[{timestamp()}] Serial port {device} opened at {baud} baud")
    elif device and os.path.exists(device):
        print(f"[{timestamp()}] Serial device {device} exists but open failed: {ser_error}")
        print(f"[{timestamp()}] Running simulation mode (will retry device)")
    else:
        print(f"[{timestamp()}] Device {device} not present, running simulation mode")

    reconnect_interval = 5.0   # seconds between reconnection attempts
    last_reconnect_attempt = 0.0
    stale_timeout = 5.0        # warn if JSON not updated for this many seconds

    while running:
        # Try to reconnect serial if disconnected
        if ser is None and os.path.exists(device):
            now = time.time()
            if now - last_reconnect_attempt >= reconnect_interval:
                ser, ser_error = open_serial(device, baud)
                if ser is not None:
                    print(f"[{timestamp()}] Serial reconnected: {device}")
                last_reconnect_attempt = now

        # Read JSON
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except FileNotFoundError:
            if time.time() - last_reconnect_attempt > stale_timeout:
                print(f"[{timestamp()}] Waiting for {json_path}...")
                last_reconnect_attempt = time.time()
            time.sleep(interval)
            continue
        except json.JSONDecodeError as e:
            print(f"[{timestamp()}] JSON parse error: {e}")
            time.sleep(interval)
            continue

        frame_id = data.get("frame_id", -1)
        if frame_id == last_frame_id:
            time.sleep(interval)
            continue
        last_frame_id = frame_id

        # Rate limiting
        if rate_limit_hz > 0:
            min_interval = 1.0 / rate_limit_hz
            elapsed = time.time() - last_send_time
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)

        # Format and send
        frames = format_all_targets(data)
        for msg in frames:
            ts = timestamp()
            msg_str = msg.strip()

            if ser is not None:
                try:
                    ser.write(msg.encode("ascii", errors="replace"))
                    ser.flush()
                    print(f"[{ts}] TX -> {msg_str}")
                    global _last_write_ok
                    _last_write_ok = time.time()
                except Exception as e:
                    print(f"[{ts}] Serial write error: {e}")
                    print(f"[{ts}] TX(sim) -> {msg_str}")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
            else:
                print(f"[{ts}] TX(sim) -> {msg_str}")

        last_send_time = time.time()
        time.sleep(interval)

    # Cleanup
    if ser is not None:
        try:
            ser.close()
            print(f"[{timestamp()}] Serial port closed")
        except Exception:
            pass
    print(f"[{timestamp()}] RS485 TX stopped.")


def main():
    parser = argparse.ArgumentParser(description="Mine Pedestrian RS485 Transmitter")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--json", required=True, help="Path to live_targets.json")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="Polling interval in seconds")
    parser.add_argument("--rate-limit", type=float, default=0,
                        help="Max sends per second (0=unlimited)")
    args = parser.parse_args()

    run_loop(args.json, args.device, args.baud, args.interval, args.rate_limit)


if __name__ == "__main__":
    main()
