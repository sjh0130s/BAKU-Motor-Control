"""Golden Motor EZkontrol RPM feedback UDP test (Arduino -> PC).
Packet: signed int32 rpm, uint32 millis_timestamp (little-endian), 8 bytes.
"""
import socket
import struct
import time

PC_LISTEN_IP = "0.0.0.0"
PC_LISTEN_PORT = 5000
ARDUINO_IP = "10.177.21.2"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PC_LISTEN_IP, PC_LISTEN_PORT))
sock.settimeout(2.0)
print(f"Waiting for motor RPM UDP on {PC_LISTEN_IP}:{PC_LISTEN_PORT}")

packet_count = 0
last_report = time.monotonic()
last_motor_ms = None

while True:
    try:
        raw, (src_ip, src_port) = sock.recvfrom(1024)
    except socket.timeout:
        print("No motor RPM UDP packet in 2 seconds")
        continue

    if src_ip != ARDUINO_IP:
        print(f"Ignoring other sender: {src_ip}:{src_port}")
        continue
    if len(raw) != 8:
        print(f"Unexpected packet size ({len(raw)} bytes): {raw.hex()}")
        continue

    rpm, timestamp_ms = struct.unpack("<iI", raw)
    delta_ms = None if last_motor_ms is None else (timestamp_ms - last_motor_ms) & 0xFFFFFFFF
    last_motor_ms = timestamp_ms
    packet_count += 1
    print(f"Actual RPM={rpm:>7}, sample millis={timestamp_ms}, sample dt(ms)={delta_ms}")

    now = time.monotonic()
    if now - last_report >= 1.0:
        print(f"  Received {packet_count} packets / {now - last_report:.2f}s "
              f"({packet_count / (now - last_report):.1f}Hz)")
        packet_count = 0
        last_report = now