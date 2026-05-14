"""
Bluetooth packet reader for MazeBot firmware.

Distilled from lidar_visualizer.py's BluetoothReceiver — strips out all the
SLAM / rendering plumbing and just gives you two things:

    - one_full_scan(ser) -> (scan, odom)
          Block until a full 360° lidar rotation is received, return:
            scan = [(angle_deg, distance_m, quality), ...]
            odom = (x_m, y_m, theta_deg)  -- from the last packet of the scan
    - wait_for_ack(ser, timeout) -> str | None
          Reads firmware text replies (MOVE_DONE / TURN_DONE / MOVE_TIMEOUT /
          TURN_TIMEOUT / ESTOP) and returns the first one seen.

Packet formats (from MazeBot_MSD/Core/Src/bt_cmd.c + lidar_visualizer.py):
    Fused  0x55 0xAA + 16 data bytes: <I H B h h i B>
           angle_q8(4) dist_mm(2) quality(1) odom_x_cm(2) odom_y_cm(2)
           odom_theta_q8(4) checksum_xor(1)
    Odom   0x55 0xBB + 9  data bytes: <h h i B>
    SYNC   0xEE 0xEE  (start of a new full rotation)
    Ack    ASCII line "MOVE_DONE\r\n" / "TURN_DONE\r\n" / "ESTOP\r\n" ...

Note: ACKs are text lines mixed with the binary packet stream. When the
byte 'M' (0x4D), 'T' (0x54), or 'E' (0x45) shows up where a header byte
would be expected, we peel off a CRLF-terminated line and return it.

Distance calibration: the firmware's mm value is used raw. We intentionally
do NOT apply the DISTANCE_SCALE_FACTOR / near-distance correction that
lidar_visualizer.py applies — phase 3 works in "virtual units" as the user
wants. If results look off on the physical maze, we can revisit.
"""
from __future__ import annotations
import struct
import time
from dataclasses import dataclass

HEADER_FUSED_B1 = 0x55
HEADER_FUSED_B2 = 0xAA
HEADER_ODOM_B2 = 0xBB
SYNC_BYTE = 0xEE

# ACK lines from firmware (see bt_cmd.c)
ACK_PREFIX_BYTES = {ord('M'), ord('T'), ord('E')}  # Move*, Turn*, Estop
ACK_MESSAGES = {'MOVE_DONE', 'MOVE_TIMEOUT', 'TURN_DONE', 'TURN_TIMEOUT', 'ESTOP'}


@dataclass
class Odom:
    x_m: float
    y_m: float
    theta_deg: float


def _read_exact(ser, n):
    """Read exactly n bytes from serial, or return None if timeout."""
    buf = ser.read(n)
    if len(buf) < n:
        return None
    return buf


def _parse_fused(data: bytes) -> dict | None:
    """Parse 16 data bytes of a fused packet. Returns dict or None on checksum fail."""
    if len(data) != 16:
        return None
    angle_q8, dist_mm, quality, ox_cm, oy_cm, otheta_q8, checksum = \
        struct.unpack('<IHBhhiB', data)
    chk = 0
    for b in data[:-1]:
        chk ^= b
    if chk != checksum:
        return None
    return {
        'type': 'FUSED',
        'angle_deg': angle_q8 / 256.0,
        'distance_m': dist_mm / 1000.0,
        'quality': quality,
        'odom': Odom(ox_cm / 100.0, oy_cm / 100.0, otheta_q8 / 256.0),
    }


def _parse_odom(data: bytes) -> dict | None:
    if len(data) != 9:
        return None
    ox_cm, oy_cm, otheta_q8, checksum = struct.unpack('<hhiB', data)
    chk = 0
    for b in data[:-1]:
        chk ^= b
    if chk != checksum:
        return None
    return {
        'type': 'ODOM',
        'odom': Odom(ox_cm / 100.0, oy_cm / 100.0, otheta_q8 / 256.0),
    }


def _try_read_ack(ser, first_byte: int) -> str | None:
    """
    We saw 'M' / 'T' / 'E' where a 0x55 header was expected. Could be the
    start of an ACK line. Read until \n (or timeout) and see if it parses.

    Returns the ACK keyword (e.g. 'MOVE_DONE') or None if it didn't look like one.
    """
    line = bytearray([first_byte])
    deadline = time.time() + 0.1  # give up quickly if not a real ack
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b'\n':
            break
        line.extend(b)
        if len(line) > 50:  # unreasonably long, bail
            return None
    text = bytes(line).decode('ascii', errors='ignore').strip()
    if text in ACK_MESSAGES:
        return text
    return None


def read_one_packet(ser):
    """
    Read one message from the firmware. Returns one of:
        dict  — a fused or odom packet (see _parse_fused / _parse_odom)
        'SYNC' — a sync marker (start of new rotation)
        str   — an ACK keyword (MOVE_DONE / TURN_DONE / ...)
        None  — timeout / checksum fail / unrecognized byte
    """
    b = ser.read(1)
    if not b:
        return None
    b0 = b[0]

    if b0 == HEADER_FUSED_B1:
        b = ser.read(1)
        if not b:
            return None
        b1 = b[0]
        if b1 == HEADER_FUSED_B2:
            data = _read_exact(ser, 16)
            return _parse_fused(data) if data else None
        if b1 == HEADER_ODOM_B2:
            data = _read_exact(ser, 9)
            return _parse_odom(data) if data else None
        return None

    if b0 == SYNC_BYTE:
        b = ser.read(1)
        if b and b[0] == SYNC_BYTE:
            return 'SYNC'
        return None

    if b0 in ACK_PREFIX_BYTES:
        ack = _try_read_ack(ser, b0)
        if ack:
            return ack
        return None

    return None  # junk byte


def read_full_scan(ser, timeout_s: float = 2.0):
    """
    Collect one complete lidar rotation.

    Returns (scan_points, odom) where
        scan_points = list of (angle_deg, distance_m, quality)
        odom        = Odom from the last packet, or None if no fused packet came
    Returns (None, None) if we didn't get a complete scan within timeout.

    ACK lines that arrive during scan collection are ignored (and lost!). If
    you need to wait for an ACK you should call wait_for_ack() instead, after
    sending a command.
    """
    deadline = time.time() + timeout_s
    scan = []
    last_odom = None
    started = False

    while time.time() < deadline:
        pkt = read_one_packet(ser)
        if pkt is None:
            continue

        if pkt == 'SYNC':
            if started and len(scan) > 20:  # got a full rotation
                return scan, last_odom
            # otherwise we're just entering a fresh rotation
            scan = []
            started = True
            continue

        if isinstance(pkt, dict):
            if pkt['type'] == 'FUSED':
                if started:
                    scan.append((pkt['angle_deg'], pkt['distance_m'], pkt['quality']))
                last_odom = pkt['odom']
            elif pkt['type'] == 'ODOM':
                last_odom = pkt['odom']
            continue

        # ack string — ignore silently
        continue

    return None, None


def wait_for_ack(ser, timeout_s: float = 12.0) -> str | None:
    """
    After sending e.g. 'F70\\n', call this to block until MOVE_DONE (or a
    timeout / failure ACK). Returns the ACK string, or None if timed out.

    While waiting, the lidar is still streaming scan + odom packets. We
    silently skip those — they'll be picked up next time read_full_scan()
    is called. (There's an inherent race where the "freshly sensed" scan
    might actually be from during the motion; the controller should do a
    *fresh* scan after the ack returns, not rely on pre-ack data.)
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        pkt = read_one_packet(ser)
        if isinstance(pkt, str) and pkt in ACK_MESSAGES:
            return pkt
    return None


def drain(ser, duration_s: float = 0.2):
    """Throw away any buffered bytes. Useful before sending a command to clear stale ACKs."""
    deadline = time.time() + duration_s
    while time.time() < deadline:
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        else:
            time.sleep(0.01)
