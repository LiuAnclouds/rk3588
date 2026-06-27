#!/usr/bin/env python3 -u
"""
Mine Pedestrian Modbus RTU Slave (RS485)
Monitors live_targets.json and serves detection status via Modbus RTU.

Protocol:  Modbus RTU
Slave ID:  2
Baud:      115200
Data/Stop/Parity: 8/1/N

Holding Registers (Function Code 03):
  0  (40001): frame_id low 16 bits
  1  (40002): detected flag  (0=clear, 1=person detected)
  2  (40003): target count (number of persons detected)
  3  (40004): frame_id high 16 bits
"""

import argparse
import json
import os
import signal
import struct
import sys
import time
from datetime import datetime

# Modbus RTU constants
MB_READ_HOLDING_REGISTERS = 0x03
MB_SLAVE_ID = 2
MB_REG_FRAME_ID_LO = 0
MB_REG_DETECTED = 1
MB_REG_TARGET_COUNT = 2
MB_REG_FRAME_ID_HI = 3
MB_REG_COUNT = 4  # total registers


def crc16(data: bytes) -> int:
    """Modbus CRC-16 (polynomial 0xA001)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc


def verify_crc(frame: bytes) -> bool:
    """Verify Modbus CRC of a received frame."""
    if len(frame) < 2:
        return False
    expected = crc16(frame[:-2])
    received = frame[-2] | (frame[-1] << 8)
    return expected == received


def build_response(slave_id: int, func_code: int, data: bytes) -> bytes:
    """Build a Modbus RTU response frame with CRC."""
    header = bytes([slave_id, func_code])
    frame = header + data
    crc = crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_exception_response(slave_id: int, func_code: int, exc_code: int) -> bytes:
    """Build a Modbus exception response."""
    return build_response(slave_id, func_code | 0x80, bytes([exc_code]))


def handle_read_holding_registers(slave_id: int, frame: bytes, registers: list) -> bytes:
    """Handle Function Code 03 - Read Holding Registers."""
    if len(frame) < 8:
        return build_exception_response(slave_id, MB_READ_HOLDING_REGISTERS, 0x03)

    start_addr = (frame[2] << 8) | frame[3]
    quantity = (frame[4] << 8) | frame[5]

    if quantity < 1 or quantity > 125:
        return build_exception_response(slave_id, MB_READ_HOLDING_REGISTERS, 0x03)

    end_addr = start_addr + quantity
    if end_addr > len(registers):
        return build_exception_response(slave_id, MB_READ_HOLDING_REGISTERS, 0x02)

    data = bytearray()
    for i in range(start_addr, end_addr):
        val = registers[i] & 0xFFFF
        data.append((val >> 8) & 0xFF)
        data.append(val & 0xFF)

    return build_response(slave_id, MB_READ_HOLDING_REGISTERS,
                          bytes([len(data)]) + bytes(data))


def process_frame(frame: bytes, registers: list) -> bytes:
    """Process a Modbus RTU request frame and return response."""
    if len(frame) < 4:
        return b''

    slave_id = frame[0]
    if slave_id != MB_SLAVE_ID:
        return b''  # not addressed to us

    func_code = frame[1]

    if func_code == MB_READ_HOLDING_REGISTERS:
        return handle_read_holding_registers(slave_id, frame, registers)

    # Unsupported function code
    return build_exception_response(slave_id, func_code, 0x01)


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def load_registers(json_path: str) -> tuple:
    """Read live_targets.json and build register values. Returns (registers, ok)."""
    try:
        with open(json_path, "r") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None, False

    frame_id = data.get("frame_id", 0)
    targets = data.get("targets", [])
    detected = 1 if len(targets) > 0 else 0
    count = len(targets)

    regs = [0] * MB_REG_COUNT
    regs[MB_REG_FRAME_ID_LO] = frame_id & 0xFFFF
    regs[MB_REG_FRAME_ID_HI] = (frame_id >> 16) & 0xFFFF
    regs[MB_REG_DETECTED] = detected
    regs[MB_REG_TARGET_COUNT] = count
    return regs, True


def run_slave(json_path: str, device: str, baud: int):
    """Modbus RTU slave main loop."""
    print(f"[{timestamp()}] Modbus RTU Slave starting")
    print(f"[{timestamp()}] Device={device} Baud={baud} 8/1/N SlaveID={MB_SLAVE_ID}")
    print(f"[{timestamp()}] JSON source: {json_path}")

    running = True

    def handle_signal(signum, frame):
        nonlocal running
        print(f"\n[{timestamp()}] Signal {signum}, stopping...")
        running = False

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    # Initialize registers
    registers, _ = load_registers(json_path)
    if registers is None:
        registers = [0] * MB_REG_COUNT
        print(f"[{timestamp()}] Waiting for {json_path}...")

    # Open serial port
    if not os.path.exists(device):
        print(f"[{timestamp()}] Device {device} not found, running simulation mode")
        ser = None
    else:
        try:
            import serial
            ser = serial.Serial(device, baudrate=baud, bytesize=8,
                                parity='N', stopbits=1, timeout=0.01)
            print(f"[{timestamp()}] Serial {device} opened")
        except ImportError:
            print(f"[{timestamp()}] pyserial not installed, simulation mode")
            ser = None
        except Exception as e:
            print(f"[{timestamp()}] Serial open error: {e}, simulation mode")
            ser = None

    poll_tick = time.time()
    req_count = 0
    buf = b''

    try:
        while running:
            # Periodically refresh registers from JSON
            now = time.time()
            if now - poll_tick >= 0.2:
                new_regs, ok = load_registers(json_path)
                if ok and new_regs is not None:
                    registers = new_regs
                poll_tick = now

            if ser is not None:
                try:
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                        req_count += 1
                except Exception:
                    time.sleep(0.01)
                    continue

                # Parse complete Modbus frames from buffer
                # Modbus RTU frames are separated by >= 3.5 char silence
                # For simplicity, try to parse when buffer has enough data
                while len(buf) >= 4:
                    # Minimum frame: slave(1) + func(1) + addr(2) + qty(2) + crc(2) = 8
                    if len(buf) < 8:
                        break

                    # Check CRC
                    if not verify_crc(buf[:8]):
                        # Shift buffer and retry
                        buf = buf[1:]
                        continue

                    resp = process_frame(buf[:8], registers)
                    if resp:
                        try:
                            ser.write(resp)
                            ser.flush()
                            det = registers[MB_REG_DETECTED]
                            cnt = registers[MB_REG_TARGET_COUNT]
                            fid = registers[MB_REG_FRAME_ID_LO] | (registers[MB_REG_FRAME_ID_HI] << 16)
                            if req_count <= 1 or req_count % 50 == 0:
                                print(f"[{timestamp()}] Modbus: fid={fid} detected={det} count={cnt} reqs={req_count}")
                                req_count = 0
                        except Exception as e:
                            print(f"[{timestamp()}] Write error: {e}")
                            try:
                                ser.close()
                            except Exception:
                                pass
                            ser = None
                            break

                    buf = buf[8:]  # consume frame
            else:
                # Simulation mode: just log register state changes
                new_regs, ok = load_registers(json_path)
                if ok and new_regs is not None and new_regs != registers:
                    registers = new_regs
                    fid = registers[MB_REG_FRAME_ID_LO] | (registers[MB_REG_FRAME_ID_HI] << 16)
                    det = registers[MB_REG_DETECTED]
                    cnt = registers[MB_REG_TARGET_COUNT]
                    print(f"[{timestamp()}] MODBUS(sim) SlaveID={MB_SLAVE_ID} fid={fid} detected={det} count={cnt}")
                time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            try:
                ser.close()
                print(f"[{timestamp()}] Serial closed")
            except Exception:
                pass
        print(f"[{timestamp()}] Modbus RTU Slave stopped.")


def main():
    parser = argparse.ArgumentParser(description="Mine Pedestrian Modbus RTU Slave")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", required=True)
    args = parser.parse_args()

    run_slave(args.json, args.device, args.baud)


if __name__ == "__main__":
    main()
