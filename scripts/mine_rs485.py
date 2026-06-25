#!/usr/bin/env python3
"""
Mine Pedestrian RS485 Transmitter
Reads live_targets.json and sends person detection data via serial (USB-to-485).
Supports simulation mode when no hardware serial device is available.
"""

import argparse
import json
import os
import struct
import sys
import time
from datetime import datetime


def checksum_nmea(data: str) -> str:
    """Compute NMEA-style XOR checksum."""
    c = 0
    for ch in data:
        c ^= ord(ch)
    return f"{c:02X}"


def format_pedestrian_frame(target: dict, frame_id: int) -> str:
    """
    Format a single pedestrian detection into a serial protocol frame.
    Protocol: $PED,<frame_id>,<id>,<conf>,<x1>,<y1>,<x2>,<y2>,<fx>,<fy>,<bearing>,<bearing_zone>,<range_zone>*<cksum>
    """
    bbox = target.get("bbox", [0, 0, 0, 0])
    fp = target.get("footpoint_px", [0, 0])
    payload = (
        f"PED,{frame_id},{target.get('id', 0)},{target.get('confidence', 0):.3f},"
        f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]},"
        f"{fp[0]},{fp[1]},"
        f"{target.get('bearing_deg', 0):.1f},"
        f"{target.get('bearing_zone', '')},"
        f"{target.get('range_zone', '')}"
    )
    cksum = checksum_nmea(payload)
    return f"${payload}*{cksum}\r\n"


def format_all_targets(data: dict) -> list:
    """Format all targets from a live_targets.json frame."""
    frames = []
    frame_id = data.get("frame_id", 0)
    for t in data.get("targets", []):
        frames.append(format_pedestrian_frame(t, frame_id))
    if not data.get("targets"):
        frames.append(f"$PED,{frame_id},0,0,,,,,,,*{checksum_nmea(f'PED,{frame_id},0,0,,,,,,,')}\r\n")
    return frames


def simulate_send(message: str, device: str = "/dev/null"):
    """Simulate or actually send a message via serial port."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    msg_stripped = message.strip()
    print(f"[{timestamp}] TX -> {msg_stripped}")

    if not os.path.exists(device):
        print(f"[{timestamp}] WARNING: device {device} not found, simulation only")
        return False

    try:
        import serial
        ser = serial.Serial(device, baudrate=115200, timeout=0.5)
        ser.write(message.encode("ascii", errors="replace"))
        ser.close()
        return True
    except ImportError:
        print(f"[{timestamp}] WARNING: pyserial not installed, simulation only")
        return False
    except Exception as e:
        print(f"[{timestamp}] ERROR: serial write failed: {e}")
        return False


def run_loop(json_path: str, device: str, baud: int, interval: float = 0.1):
    """Main loop: poll the JSON file and send detections."""
    print(f"Mine RS485 TX started: device={device} baud={baud} json={json_path}")
    last_frame_id = -1
    has_serial = os.path.exists(device)

    if has_serial:
        try:
            import serial
            ser = serial.Serial(device, baudrate=baud, timeout=0.5)
            print(f"Serial port {device} opened at {baud} baud")
        except ImportError:
            print("WARNING: pyserial not available, running simulation mode")
            has_serial = False
            ser = None
        except Exception as e:
            print(f"WARNING: cannot open {device}: {e}, running simulation mode")
            has_serial = False
            ser = None
    else:
        print(f"Device {device} not present, running simulation mode")
        ser = None

    try:
        while True:
            try:
                with open(json_path, "r") as f:
                    data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(interval)
                continue

            frame_id = data.get("frame_id", -1)
            if frame_id == last_frame_id:
                time.sleep(interval)
                continue
            last_frame_id = frame_id

            frames = format_all_targets(data)
            for msg in frames:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] TX -> {msg.strip()}")

                if has_serial and ser is not None:
                    try:
                        ser.write(msg.encode("ascii", errors="replace"))
                    except Exception as e:
                        print(f"[{timestamp}] ERROR: {e}")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nRS485 TX stopped.")
    finally:
        if ser is not None and has_serial:
            ser.close()


def main():
    parser = argparse.ArgumentParser(description="Mine Pedestrian RS485 Transmitter")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--json", required=True, help="Path to live_targets.json")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="Polling interval in seconds")
    args = parser.parse_args()

    run_loop(args.json, args.device, args.baud, args.interval)


if __name__ == "__main__":
    main()
