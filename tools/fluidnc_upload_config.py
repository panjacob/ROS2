#!/usr/bin/env python3
"""Upload a FluidNC config file over USB serial (XMODEM) and optionally restart the board.

  python3 tools/fluidnc_upload_config.py firmware/fluidnc/config.xyzab.yaml --restart

Stop the ROS launch first: only one program can use the serial port.
"""
import argparse
import os
import time

import serial

SOH, EOT, ACK, NAK, CAN, CRC = 0x01, 0x04, 0x06, 0x15, 0x18, ord('C')
BLOCK = 128


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def open_board(port, baud):
    s = serial.Serial(None, baud, timeout=0.1)
    s.dtr = False
    s.rts = False
    s.port = port
    s.open()
    # Opening the CH340 may reboot the ESP32: wait for the banner (or a quiet line).
    deadline, seen = time.time() + 6.0, b''
    while time.time() < deadline and b"for help]" not in seen:
        seen += s.read(4096)
    time.sleep(0.5)
    s.reset_input_buffer()
    return s


def command(s, line, wait=3.0):
    s.write((line + '\n').encode())
    deadline, out = time.time() + wait, b''
    while time.time() < deadline:
        out += s.read(4096)
        if out.rstrip().endswith(b'ok') or b'error:' in out:
            break
    return out.decode(errors='replace')


def xmodem_send(s, data):
    deadline = time.time() + 10.0
    while True:
        c = s.read(1)
        if c and c[0] == CRC:
            break
        if time.time() > deadline:
            raise RuntimeError('Board did not start XMODEM receive')
    # Pad with newlines (harmless in YAML) instead of 0x1A, which would end up in the file.
    if len(data) % BLOCK:
        data += b'\n' * (BLOCK - len(data) % BLOCK)
    for n in range(len(data) // BLOCK):
        chunk = data[n * BLOCK:(n + 1) * BLOCK]
        seq = (n + 1) & 0xFF
        packet = bytes([SOH, seq, 0xFF - seq]) + chunk + crc16(chunk).to_bytes(2, 'big')
        for _ in range(10):
            s.write(packet)
            reply = _read_byte(s, 5.0)
            if reply == ACK:
                break
            if reply == CAN:
                raise RuntimeError('Board cancelled the transfer')
        else:
            raise RuntimeError(f'Block {n + 1} not acknowledged')
    for _ in range(10):
        s.write(bytes([EOT]))
        if _read_byte(s, 5.0) == ACK:
            return
    raise RuntimeError('End of transfer not acknowledged')


def _read_byte(s, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        c = s.read(1)
        if c:
            return c[0]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file')
    ap.add_argument('--name', help='file name on the board (default: same as local)')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--restart', action='store_true', help='restart FluidNC to load the new config')
    args = ap.parse_args()

    name = args.name or os.path.basename(args.file)
    with open(args.file, 'rb') as f:
        data = f.read()

    s = open_board(args.port, args.baud)
    s.write(f'$Xmodem/Receive={name}\n'.encode())
    xmodem_send(s, data)
    time.sleep(0.5)
    print(s.read(4096).decode(errors='replace').strip())
    print(f'Uploaded {len(data)} bytes as {name}')
    print(command(s, '$Config/Filename').strip())
    if args.restart:
        s.write(b'$Bye\n')
        print('Restarting FluidNC...')
    s.close()


if __name__ == '__main__':
    main()
