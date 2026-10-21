"""Receive measured motor RPM with a CAN-sample timestamp via UDP 5001.

Payload (8 bytes, little endian): <int32 motor_rpm, uint32 sample_timestamp_ms>.
The timestamp uses *PCB uptime milliseconds*, not the PC wall clock.
Same timestamp repeated = retransmission of the same CAN sample, not a new measurement.
No UDP for > 0.5 s = stale/failed telemetry (valid byte deliberately removed).
"""
import socket
import struct
import time

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5001
NO_DATA_TIMEOUT_SEC = 0.5

def decode(packet: bytes):
    if len(packet) != 8:
        raise ValueError(f"expected 8 bytes, got {len(packet)}")
    return struct.unpack("<iI", packet)

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        sock.settimeout(0.1)
        print(f"[BAKU] UDP {LISTEN_PORT} listening; Ctrl+C to stop")
        last_packet_at = None
        last_sample_ts = None
        last_warning = None
        received_packets = 0
        new_samples = 0
        count_start = time.monotonic()
        try:
            while True:
                try:
                    packet, address = sock.recvfrom(128)
                except socket.timeout:
                    now = time.monotonic()
                    if last_packet_at is not None and (now - last_packet_at) > NO_DATA_TIMEOUT_SEC:
                        if last_warning is None or (now - last_warning) >= 1.0:
                            print("[STALE] No RPM UDP received for >500 ms; do NOT interpret as zero RPM")
                            last_warning = now
                    continue
                try:
                    rpm, sample_ts = decode(packet)
                except ValueError as err:
                    print(f"[DROP] {err} from {address}")
                    continue
                now = time.monotonic()
                last_packet_at = now
                last_warning = None
                received_packets += 1
                fresh = last_sample_ts is None or ((sample_ts-last_sample_ts)&0xFFFFFFFF) != 0
                if fresh:
                    new_samples += 1
                last_sample_ts = sample_ts
                print(f"[RPM] {rpm:+d} rpm, PCB CAN sample ms={sample_ts}, "
                      f"{'NEW' if fresh else 'REPEATED'} from {address}")
                if now-count_start>=1.0:
                    period=now-count_start
                    print(f"[RATE] UDP recv={received_packets/period:.1f} Hz, "
                          f"new CAN samples={new_samples/period:.1f} Hz")
                    received_packets = new_samples = 0
                    count_start = now
        except KeyboardInterrupt:
            print("\nStopped")

if __name__ == "__main__":
    main()
