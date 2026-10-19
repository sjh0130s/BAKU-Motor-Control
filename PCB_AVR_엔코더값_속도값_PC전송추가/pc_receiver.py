"""Receive BAKU motor's measured RPM telemetry: UDP port 5001.

Valid=0 means no reliable fresh RPM feedback, *not* that motor speed is zero.
"""
import socket
import struct
import time

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5001

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(1.0)
    print(f"[BAKU RPM] Listening on UDP {LISTEN_PORT}; Ctrl+C to stop")
    last_received = time.monotonic()
    try:
        while True:
            try:
                packet, address = sock.recvfrom(128)
            except socket.timeout:
                if time.monotonic() - last_received >= 1.0:
                    print("[WARNING] No fresh UDP telemetry received within 1 second")
                    last_received = time.monotonic()
                continue
            last_received = time.monotonic()
            if len(packet) != 5:
                print(f"[DROP] unexpected {len(packet)} bytes from {address}")
                continue
            rpm, valid = struct.unpack("<iB", packet)
            if valid == 1:
                print(f"[RPM] {rpm:+d} rpm (controller feedback) from {address}")
            else:
                print(f"[INVALID] no fresh motor RPM feedback from {address}")
    except KeyboardInterrupt:
        print("\nStopped")
