"""Windows motor-only UDP tuner for the 1025_0100 UNO firmware.

Run run.cmd, or python motor_test.py --dry-run for a socket-free simulation.
All control code lives in this file. GUI/dependencies are imported on demand.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parent / "Arduino_UNO/1025_0100_변경코드_PC에_모터값_계속_보내/1025_0100_변경코드_PC에_모터값_계속_보내.ino"
PC_IP, ASMS_IP, BOARD_IP = "10.177.21.1", "10.177.21.5", "10.177.21.2"
COMMAND_PORT, FEEDBACK_PORT = 5000, 5001
RPM_LIMIT, TX_PERIOD, ZERO_SECONDS = 891.387, 0.02, 2.0
UNAVAILABLE = -(1 << 31)
PC_PACKET, ASMS_PACKET, FEEDBACK_PACKET = struct.Struct("<ffB"), struct.Struct("<Bhh"), struct.Struct("<iI")


def finite(value, name="값"):
    if isinstance(value, bool):
        raise ValueError(f"{name}: 숫자를 입력하세요.")
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as exc:
        raise ValueError(f"{name}: 숫자를 입력하세요.") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name}: NaN/무한대는 허용하지 않습니다.")
    return result


def rpm_value(value):
    value = finite(value, "RPM")
    if abs(value) > RPM_LIMIT:
        raise ValueError(f"RPM 범위: {-RPM_LIMIT} ~ {RPM_LIMIT}")
    return value


def joy_value(value):
    number = finite(value, "joy_x")
    if not number.is_integer() or not -2048 <= number <= 2047:
        raise ValueError("joy_x는 -2048 ~ 2047 범위의 정수입니다.")
    return int(number)


def joy_rpm(raw):
    raw = joy_value(raw)
    if abs(raw) <= 120:
        return 0.0
    magnitude = min(1.0, (abs(raw) - 120) / (2048 - 120)) * RPM_LIMIT
    return magnitude if raw < 0 else -magnitude


def pc_packet(rpm):
    return PC_PACKET.pack(0.0, rpm_value(rpm), 0)


def asms_packet(mode, joy=0):
    if mode not in (1, 2, 3):
        raise ValueError("ASMS mode는 1/2/3만 지원합니다.")
    return ASMS_PACKET.pack(mode, joy_value(joy), 0)


@dataclass(frozen=True)
class Stage:
    rpm: float
    at: float
    hold: float = 0.0


@dataclass(frozen=True)
class Scenario:
    stages: tuple[Stage, ...]

    def __post_init__(self):
        if not 1 <= len(self.stages) <= 1000:
            raise ValueError("단계 수는 1 ~ 1000입니다.")
        previous_end = 1.0
        for index, stage in enumerate(self.stages):
            rpm_value(stage.rpm)
            at, hold = finite(stage.at, "도달 시각"), finite(stage.hold, "유지 시간")
            if at < previous_end:
                raise ValueError(f"{index + 1}단계 도달 시각은 {previous_end:g}초 이상이어야 합니다.")
            if hold < 0:
                raise ValueError("유지 시간은 0초 이상이어야 합니다.")
            previous_end = at + hold
            if not math.isfinite(previous_end):
                raise ValueError("시간 합계가 너무 큽니다.")

    def value(self, elapsed):
        if elapsed < 1:
            return 0.0
        previous_end, previous_rpm = 1.0, 0.0
        for stage in self.stages:
            if elapsed < stage.at:
                ratio = (elapsed - previous_end) / (stage.at - previous_end)
                return previous_rpm + (stage.rpm - previous_rpm) * ratio
            if elapsed < stage.at + stage.hold:
                return stage.rpm
            previous_end, previous_rpm = stage.at + stage.hold, stage.rpm
        return previous_rpm

    def points(self, end=None):
        end = max(self.stages[-1].at + self.stages[-1].hold + 2, end or 0)
        xs, ys = [0.0, 1.0], [0.0, 0.0]
        for stage in self.stages:
            xs.append(stage.at)
            ys.append(stage.rpm)
            xs.append(stage.at + stage.hold)
            ys.append(stage.rpm)
        xs.append(end)
        ys.append(self.stages[-1].rpm)
        return xs, ys

    def data(self):
        return {"schema": 1, "rpm_convention": "pc_speed_field", "initial_zero_s": 1,
                "final_behavior": "hold_until_stop", "stages": [asdict(s) for s in self.stages]}

    @classmethod
    def from_data(cls, data):
        if (not isinstance(data, dict) or data.get("schema") != 1 or
                data.get("rpm_convention") != "pc_speed_field" or
                data.get("initial_zero_s") != 1 or data.get("final_behavior") != "hold_until_stop"):
            raise ValueError("지원하지 않는 시나리오 형식/단위입니다.")
        try:
            stages = tuple(Stage(rpm_value(s["rpm"]), finite(s["at"]), finite(s["hold"]))
                           for s in data["stages"])
        except (KeyError, TypeError) as exc:
            raise ValueError("시나리오 단계 형식이 올바르지 않습니다.") from exc
        return cls(stages)


class Feedback:
    """UDP receipt and advancing MCU measurement time are separate clocks."""
    def __init__(self, warning_age=0.5):
        self.warning_age = warning_age
        self.last_udp = self.last_advance = self.stamp = self.raw = None
        self.baseline_at = None
        self.confirmed = False
        self.packets = self.samples = self.bad = self.resets = 0
        self.udp_times, self.sample_times = [], []

    def observe(self, data, address, now):
        if address != (BOARD_IP, FEEDBACK_PORT) or len(data) != FEEDBACK_PACKET.size:
            self.bad += 1
            raise ValueError(f"피드백 송신원/크기 오류: {address}, {len(data)} bytes")
        raw, stamp = FEEDBACK_PACKET.unpack(data)
        self.last_udp, self.raw = now, raw
        self.packets += 1
        self.udp_times.append(now)
        new = False
        if raw == UNAVAILABLE:
            self.stamp, self.last_advance, self.confirmed = None, None, False
            self.baseline_at = None
        else:
            if self.stamp is None:
                self.confirmed = False  # a first packet may already be cached
                self.baseline_at = now
            else:
                delta = (stamp - self.stamp) & 0xFFFFFFFF
                if delta >= 0x80000000:
                    self.resets += 1  # restart or out-of-order packet; re-establish baseline
                    self.last_advance, self.confirmed = None, False
                    self.baseline_at = now
                elif delta:
                    self.last_advance, self.confirmed, new = now, True, True
                    self.samples += 1
                    self.sample_times.append(now)
            self.stamp = stamp
        self.udp_times = [t for t in self.udp_times if t > now - 2]
        self.sample_times = [t for t in self.sample_times if t > now - 2]
        return raw, stamp, new

    def state(self, now):
        udp_age = None if self.last_udp is None else now - self.last_udp
        baseline = self.last_advance if self.last_advance is not None else self.baseline_at
        can_age = None if baseline is None else now - baseline
        if udp_age is None:
            health = "WAIT_UDP"
        elif udp_age > self.warning_age:
            health = "NO_UDP"
        elif self.raw == UNAVAILABLE:
            health = "NO_CAN_SAMPLE"
        elif can_age > self.warning_age:
            health = "STALE_CAN"
        elif not self.confirmed:
            health = "WAIT_CAN_ADVANCE"
        else:
            health = "OK"
        return {"health": health, "udp_age_s": udp_age, "can_age_s": can_age,
                "actual_raw": self.raw,
                "actual_rpm": None if self.raw in (None, UNAVAILABLE) else -self.raw,
                "udp_hz": sum(t > now - 1 for t in self.udp_times),
                "can_hz": sum(t > now - 1 for t in self.sample_times), "bad_packets": self.bad,
                "clock_resets": self.resets}


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def metadata(args):
    def git(*command):
        try:
            p = subprocess.run(["git", "-C", str(HERE.parent), *command], capture_output=True,
                               text=True, encoding="utf-8", errors="replace", timeout=5)
            return p.stdout.strip() if p.returncode == 0 else "unavailable"
        except (OSError, subprocess.SubprocessError):
            return "unavailable"
    packages = {}
    for package in ("matplotlib", "numpy", "prompt_toolkit", "wcwidth"):
        try:
            packages[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            packages[package] = "not installed"
    return {"started_local": datetime.now().astimezone().isoformat(), "python": sys.version,
            "executable": sys.executable, "argv": sys.argv, "packages": packages,
            "git_commit": git("rev-parse", "HEAD"), "git_status": git("status", "--short"),
            "program_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "firmware_source": str(FIRMWARE),
            "firmware_source_sha256": hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() if FIRMWARE.exists() else None,
            "firmware_upload_verified": False, "options": vars(args),
            "rpm_convention": "reference=UDP speed; displayed actual=-raw feedback; positive=forward per source",
            "network": {"pc": PC_IP, "asms": ASMS_IP, "board": BOARD_IP,
                        "command_port": COMMAND_PORT, "feedback_port": FEEDBACK_PORT},
            "tx_period_s": TX_PERIOD, "initial_zero_s": 1, "stop_zero_s": ZERO_SECONDS,
            "docker_image": os.environ.get("MOTOR_TEST_IMAGE", "not used (native application)")}


class Journal:
    HEADERS = {
        "commands": ["source", "mode", "reference_rpm", "wire_rpm", "joy_x", "success", "error", "gap_s"],
        "feedback": ["raw_rpm", "actual_rpm", "mcu_ms", "new_sample", "health", "udp_age_s", "can_age_s"],
        "events": ["event", "detail"],
    }

    def __init__(self, root, name, info, scenario=None):
        self.started = time.monotonic()
        self.path = Path(root) / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + name)
        self.path.mkdir(parents=True, exist_ok=False)
        self.info = dict(info, kind=name, started_local=datetime.now().astimezone().isoformat())
        write_json(self.path / "metadata.json", self.info)
        if scenario:
            write_json(self.path / "scenario.json", scenario.data())
        self.items = queue.Queue(maxsize=20000)
        self.error, self.dropped, self.closed = "", 0, False
        self.thread = threading.Thread(target=self._write, name="csv-writer", daemon=True)
        self.thread.start()

    def record(self, table, now, **values):
        if self.closed:
            return
        try:
            self.items.put_nowait((table, now - self.started, time.time(), values))
        except queue.Full:
            self.dropped += 1
            self.error = f"로그 큐 포화: {self.dropped}개 누락"

    def _write(self):
        handles = {}
        try:
            writers = {}
            for table, fields in self.HEADERS.items():
                handle = (self.path / (table + ".csv")).open("w", encoding="utf-8-sig", newline="")
                handles[table] = handle
                writer = csv.DictWriter(handle, fieldnames=["elapsed_s", "pc_epoch_s", *fields])
                writer.writeheader()
                writers[table] = writer
            last_flush = time.monotonic()
            while True:
                try:
                    item = self.items.get(timeout=0.2)
                except queue.Empty:
                    item = False
                if item is None:
                    break
                if item:
                    table, elapsed, epoch, values = item
                    writers[table].writerow({"elapsed_s": elapsed, "pc_epoch_s": epoch, **values})
                if time.monotonic() - last_flush >= 0.5:
                    for handle in handles.values():
                        handle.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self.error = f"로그 저장 실패: {exc}"
        finally:
            for handle in handles.values():
                try:
                    handle.close()
                except OSError as exc:
                    self.error = str(exc)

    def close(self, **extra):
        if self.closed:
            return
        self.closed = True
        while self.thread.is_alive():
            try:
                self.items.put(None, timeout=0.1)
                break
            except queue.Full:
                pass
        self.thread.join()
        self.info.update(extra, ended_local=datetime.now().astimezone().isoformat(),
                         log_error=self.error, dropped_records=self.dropped)
        try:
            write_json(self.path / "metadata.json", self.info)
        except OSError as exc:
            self.error = f"메타데이터 저장 실패: {exc}"


class UDPTransport:
    def __init__(self):
        self.sockets = []
        try:
            self.pc = self._socket(PC_IP, 0)
            self.asms = self._socket(ASMS_IP, 0)
            self.rx = self._socket(PC_IP, FEEDBACK_PORT)
        except Exception:
            self.close()
            raise

    def _socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockets.append(sock)
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        sock.settimeout(0.05)
        sock.bind((ip, port))
        return sock

    def send(self, source, packet):
        sock = self.pc if source == "pc" else self.asms
        if sock.sendto(packet, (BOARD_IP, COMMAND_PORT)) != len(packet):
            raise OSError("불완전한 UDP 전송")

    def receive(self):
        try:
            return self.rx.recvfrom(1024)
        except socket.timeout:
            return None

    def close(self):
        for sock in self.sockets:
            sock.close()


class Simulation:
    """No sockets. Includes firmware source/mode gating and its one-second timeout."""
    def __init__(self):
        self.started = self.last_rx = self.last_can = self.last_command = time.monotonic()
        self.mode = None
        self.target = self.actual = 0.0
        self.raw, self.stamp, self.fault = 0, 1000, "none"
        self.lock = threading.Lock()

    def send(self, source, packet):
        with self.lock:
            now = time.monotonic()
            if source == "asms":
                mode, joy, _ = ASMS_PACKET.unpack(packet)
                if mode != self.mode:
                    self.target = 0.0
                self.mode = mode
                if mode == 2:
                    self.target, self.last_command = joy_rpm(joy), now
                elif mode == 3:
                    self.target = 0.0
            elif self.mode in (None, 1):
                self.target, self.last_command = PC_PACKET.unpack(packet)[1], now

    def receive(self):
        time.sleep(TX_PERIOD)
        with self.lock:
            now = time.monotonic()
            dt, self.last_rx = now - self.last_rx, now
            if now - self.last_command > 1:
                self.target = 0.0
                if self.mode == 2:
                    self.mode = 0
            self.actual += (self.target - self.actual) * (1 - math.exp(-dt / 0.35))
            if self.fault != "can" and now - self.last_can >= 0.05:
                self.raw = round(-self.actual)
                self.stamp = (1000 + int((now - self.started) * 1000)) & 0xFFFFFFFF
                self.last_can = now - ((now - self.last_can) % 0.05)
            if self.fault == "udp":
                return None
            raw = UNAVAILABLE if self.fault == "sentinel" else self.raw
            return FEEDBACK_PACKET.pack(raw, self.stamp), (BOARD_IP, FEEDBACK_PORT)

    def close(self):
        pass


class Engine:
    def __init__(self, transport, logs, info, warning_age=0.5):
        self.transport, self.logs, self.info = transport, logs, info
        self.feedback = Feedback(warning_age)
        self.lock, self.done = threading.RLock(), threading.Event()
        self.lifecycle = threading.RLock()
        self.session = Journal(logs, "session", info)
        self.trial = None
        self.mode, self.target, self.joy, self.scenario = "idle", 0.0, 0, None
        self.phase, self.run_start, self.requested_asms = "idle", None, None
        self.last_tx, self.last_gap, self.max_gap = None, None, 0.0
        self.last_periodic = None
        self.sent = self.tx_errors = self.missed_deadlines = 0
        self.error, self.threads = "", []

    def start(self):
        for name, work in (("udp-tx", self._tx), ("udp-rx", self._rx), ("health", self._health)):
            thread = threading.Thread(target=work, name=name, daemon=True)
            thread.start()
            self.threads.append(thread)

    def _record(self, table, now, **values):
        self.session.record(table, now, **values)
        if self.trial:
            self.trial.record(table, now, **values)

    def event(self, name, detail=""):
        with self.lock:
            self._record("events", time.monotonic(), event=name, detail=detail)

    def _send(self, source, packet, reference, mode, joy=0, periodic=True):
        now = time.monotonic()
        gap = None
        success, error = True, ""
        try:
            self.transport.send(source, packet)
            sent_at = time.monotonic()
            if periodic:
                gap = None if self.last_periodic is None else sent_at - self.last_periodic
                self.last_periodic, self.last_gap = sent_at, gap
            self.last_tx = sent_at
            self.max_gap = max(self.max_gap, gap or 0)
            self.sent += 1
            self.error = ""
        except OSError as exc:
            success, error = False, str(exc)
            self.tx_errors += 1
            self.error = error
        wire = PC_PACKET.unpack(packet)[1] if source == "pc" else None
        self._record("commands", now, source=source, mode=mode, reference_rpm=reference,
                     wire_rpm=wire, joy_x=joy, success=int(success), error=error, gap_s=gap)
        return success

    def _reference(self, now):
        if self.phase == "stopping":
            return 0.0
        elapsed = 0 if self.run_start is None else now - self.run_start
        if elapsed < 1:
            return 0.0
        return self.scenario.value(elapsed) if self.scenario else self.target

    def _tx(self):
        deadline = time.monotonic()
        try:
            while not self.done.is_set():
                with self.lock:
                    now = time.monotonic()
                    if self.mode == "pc":
                        rpm = self._reference(now)
                        self._send("pc", pc_packet(rpm), rpm, "AUTO")
                    elif self.mode == "asms":
                        joy = 0 if self.phase == "stopping" or now - self.run_start < 1 else self.joy
                        self._send("asms", asms_packet(2, joy), joy_rpm(joy), "MANUAL", joy)
                deadline += TX_PERIOD
                now = time.monotonic()
                if deadline < now:
                    self.missed_deadlines += 1
                    deadline = now + TX_PERIOD  # do not burst old commands after a delay
                self.done.wait(max(0.0, deadline - now))
        except Exception as exc:
            with self.lock:
                self.mode, self.error = "idle", f"송신 작업 중단: {exc}"
                self.event("tx_worker_failed", traceback.format_exc())

    def _rx(self):
        while not self.done.is_set():
            try:
                packet = self.transport.receive()
                if packet is None:
                    continue
                data, address = packet
                with self.lock:
                    now = time.monotonic()
                    try:
                        raw, stamp, new = self.feedback.observe(data, address, now)
                    except ValueError as exc:
                        if self.feedback.bad <= 3 or self.feedback.bad % 50 == 0:
                            self.event("invalid_packet", str(exc))
                        continue
                    state = self.feedback.state(now)
                    self._record("feedback", now, raw_rpm=raw, actual_rpm=state["actual_rpm"],
                                 mcu_ms=stamp, new_sample=int(new), health=state["health"],
                                 udp_age_s=state["udp_age_s"], can_age_s=state["can_age_s"])
            except OSError as exc:
                self.event("rx_error", str(exc))
                self.done.wait(0.2)
            except Exception:
                self.event("rx_worker_failed", traceback.format_exc())
                return

    def _health(self):
        previous = None
        while not self.done.wait(0.2):
            state = self.snapshot()
            if state["health"] != previous:
                self.event("feedback_health", json.dumps(state, ensure_ascii=False))
                previous = state["health"]

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            state = self.feedback.state(now)
            errors = [self.error, self.session.error, self.trial.error if self.trial else ""]
            state.update(mode=self.mode, phase=self.phase, reference_rpm=self._reference(now) if self.mode == "pc" else
                         (joy_rpm(self.joy) if self.mode == "asms" and self.phase != "stopping" and now - self.run_start >= 1 else 0),
                         elapsed_s=0 if self.run_start is None else now - self.run_start,
                         sent=self.sent, tx_errors=self.tx_errors,
                         tx_age_s=None if self.last_tx is None else now - self.last_tx,
                         max_gap_s=self.max_gap, missed_deadlines=self.missed_deadlines,
                         requested_asms=self.requested_asms, error="; ".join(e for e in errors if e))
            return state

    def begin(self, mode, scenario=None):
        with self.lifecycle:
            if self.done.is_set():
                raise ValueError("프로그램이 종료 중입니다.")
            return self._begin(mode, scenario)

    def _begin(self, mode, scenario):
        if mode not in ("pc", "asms"):
            raise ValueError("지원하지 않는 실행 모드")
        with self.lock:
            if self.trial or self.mode != "idle":
                raise ValueError("현재 기록을 먼저 종료하세요.")
            if mode == "pc" and self.requested_asms in (2, 3):
                raise ValueError("ASMS MANUAL/ESTOP 이력: PC 메뉴의 a로 ASMS AUTO를 명시적으로 전송하세요.")
        trial = Journal(self.logs, "auto" if scenario else mode + "_manual", self.info, scenario)
        with self.lock:
            self.trial, self.mode, self.scenario = trial, mode, scenario
            self.target, self.joy = 0.0, 0
            self.phase, self.run_start, self.last_tx = "running", time.monotonic(), None
            self.last_periodic = None
            trial.info["control_start_elapsed_s"] = self.run_start - trial.started
            if mode == "asms":
                self.requested_asms = 2
            self.event("run_start", "initial 1s zero; final target held until explicit stop")
            self.event("feedback_health", json.dumps(self.snapshot(), ensure_ascii=False))
        return trial.path

    def set_target(self, value):
        with self.lock:
            if not self.trial or self.phase != "running" or self.scenario:
                raise ValueError("수동 기록 실행 중에만 값을 바꿀 수 있습니다.")
            if self.mode == "pc":
                self.target = rpm_value(value)
            else:
                self.joy = joy_value(value)
            self.event("target_input", str(value))

    def asms_command(self, mode):
        with self.lifecycle:
            if self.done.is_set():
                raise ValueError("프로그램이 종료 중입니다.")
            return self._asms_command(mode)

    def _asms_command(self, mode):
        if mode not in (1, 3):
            raise ValueError("일회 명령은 AUTO(1)/ESTOP(3)만 지원합니다.")
        with self.lock:
            self.mode, self.target, self.joy, self.scenario = "idle", 0.0, 0, None
            self.last_periodic = None
            self.requested_asms = mode
            self.event("asms_mode_requested", str(mode) + " (not an ACK)")
        results = []
        for _ in range(3):
            with self.lock:
                results.append(self._send("asms", asms_packet(mode), 0, "AUTO_REQUEST" if mode == 1 else "ESTOP_REQUEST", periodic=False))
            time.sleep(TX_PERIOD)
        return all(results)

    def finish(self, estop=False, zero_seconds=ZERO_SECONDS):
        with self.lifecycle:
            return self._finish(estop, zero_seconds)

    def _finish(self, estop, zero_seconds):
        with self.lock:
            if self.trial is None:
                return None
            if not estop:
                self.phase, self.target, self.joy = "stopping", 0.0, 0
                self.event("stop_zero_begin", f"zero for {zero_seconds:g}s, then TX idle; no ESTOP")
        if estop:
            self.asms_command(3)
        else:
            time.sleep(zero_seconds)  # separate from TX/RX threads
        with self.lock:
            self.mode, self.phase, self.scenario = "idle", "idle", None
            self.last_periodic = None
            self.event("run_end", "ESTOP requested" if estop else "TX stopped; firmware timeout HALT expected after >1s")
            trial, self.trial = self.trial, None
            final_state = self.snapshot()
            self.run_start = None
        trial.close(final_state=final_state, termination="estop" if estop else "zero_then_tx_idle")
        return trial

    def close(self):
        with self.lifecycle:
            if self.done.is_set():
                return
            self.finish()
            self.done.set()
            for thread in self.threads:
                thread.join(timeout=2)
            self.transport.close()
            self.session.close(final_state=self.snapshot())


def can_bind(ip):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((ip, 0))
            return True
        except OSError:
            return False


class NetworkSetup:
    """Opt-in, temporary address aliases; never delete pre-existing addresses."""
    def __init__(self, adapter, session):
        self.adapter, self.session, self.added = adapter, session, []

    def netsh(self, action, ip):
        args = ["netsh", "interface", "ipv4", action, "address", f"name={self.adapter}", f"address={ip}"]
        if action == "add":
            args += ["mask=255.255.255.0", "store=active"]
        result = subprocess.run(args, capture_output=True, timeout=15)
        self.session.record("events", time.monotonic(), event="network_" + action,
                            detail=json.dumps({"command": args, "code": result.returncode,
                                               "stdout": result.stdout.decode(errors="replace"),
                                               "stderr": result.stderr.decode(errors="replace")}, ensure_ascii=False))
        return result.returncode

    def ensure(self, automatic):
        if os.name == "nt":
            snapshot = subprocess.run(["netsh", "interface", "ipv4", "dump"], capture_output=True, timeout=15)
            (self.session.path / "network_before.txt").write_bytes(snapshot.stdout + snapshot.stderr)
        for ip in (PC_IP, ASMS_IP):
            if can_bind(ip):
                print(f"[IP] {ip}: 기존 주소 사용")
                continue
            if not automatic or os.name != "nt":
                raise RuntimeError(f"{ip} 바인딩 불가. README의 고정 IP 설정 또는 관리자 CMD에서 --setup-network --adapter 사용")
            if self.netsh("add", ip) != 0:
                raise RuntimeError(f"{ip} 추가 실패. 관리자 권한/어댑터 이름/중복 IP를 확인하세요.")
            self.added.append(ip)
            for _ in range(100):
                if can_bind(ip):
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError(f"{ip} 추가 후 사용 불가. 중복 주소(DAD)와 어댑터를 확인하세요.")

    def close(self):
        for ip in reversed(self.added):
            try:
                if self.netsh("delete", ip):
                    print(f"[IP ERROR] {ip} 정리 실패. README의 수동 삭제 명령을 확인하세요.")
            except (OSError, subprocess.SubprocessError) as exc:
                print(f"[IP ERROR] {ip} 정리 실패: {exc}")
        self.added.clear()


class UI:
    """Tk stays on the main thread; CLI submits window factories via a queue."""
    def __init__(self, root, engine):
        self.root, self.engine, self.requests = root, engine, queue.Queue()
        root.withdraw()
        root.after(40, self._pump)

    def _pump(self):
        try:
            while True:
                work, ready, result = self.requests.get_nowait()
                try:
                    work(lambda value: (result.append(value), ready.set()))
                except Exception as exc:
                    result.append(exc)
                    ready.set()
        except queue.Empty:
            pass
        self.root.after(40, self._pump)

    def call(self, work):
        ready, result = threading.Event(), []
        self.requests.put((work, ready, result))
        while not ready.wait(0.1):
            if self.engine.done.is_set():
                raise EOFError
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]

    def editor(self, initial, done, preview=False):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        from matplotlib.figure import Figure
        win = tk.Toplevel(self.root)
        win.title("Motor RPM reference — 미리보기" if preview else "Motor RPM reference — 표 / 점 끌기")
        win.geometry("1050x760")
        stages, selected, drag = list(initial.stages), [0], [None]
        status = tk.StringVar(value="처음 1초=0 RPM 고정. 원=도달점, 사각형=유지 종료점. 마지막 RPM은 기록 종료까지 유지.")
        ttk.Label(win, textvariable=status, wraplength=1000).pack(fill="x", padx=10, pady=6)
        fig = Figure(figsize=(9, 3.6), dpi=100)
        ax = fig.add_subplot(111)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(canvas, win)
        tree = ttk.Treeview(win, columns=("rpm", "at", "hold"), show="headings", height=5, selectmode="browse")
        for key, title in (("rpm", "목표 RPM"), ("at", "목표 도달 시각 [s]"), ("hold", "유지 시간 [s] / 마지막은 무기한")):
            tree.heading(key, text=title)
        tree.pack(fill="x", padx=10)
        row = ttk.Frame(win)
        row.pack(fill="x", padx=10, pady=6)
        values = [tk.StringVar() for _ in range(3)]
        for label, value in zip(("RPM", "도달 [s]", "유지 [s]"), values):
            ttk.Label(row, text=label).pack(side="left")
            ttk.Entry(row, textvariable=value, width=12).pack(side="left", padx=4)

        def draw():
            scenario = Scenario(tuple(stages))
            ax.clear()
            ax.plot(*scenario.points(), color="#1764b0", label="Reference (PC speed RPM)")
            ax.plot([s.at for s in stages], [s.rpm for s in stages], "o", color="#c34e1f", picker=8)
            ax.plot([s.at + s.hold for s in stages[:-1]], [s.rpm for s in stages[:-1]], "s", color="#308447")
            ax.axvspan(0, 1, color="gray", alpha=0.15)
            ax.set(xlabel="Elapsed [s]", ylabel="RPM (+ forward per firmware)", ylim=(-RPM_LIMIT * 1.1, RPM_LIMIT * 1.1))
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right")
            fig.tight_layout()
            canvas.draw_idle()
            tree.delete(*tree.get_children())
            for i, s in enumerate(stages):
                tree.insert("", "end", iid=str(i), values=(f"{s.rpm:g}", f"{s.at:g}", "기록 종료까지" if i == len(stages) - 1 else f"{s.hold:g}"))
            selected[0] = min(selected[0], len(stages) - 1)
            tree.selection_set(str(selected[0]))

        def pick(_=None):
            ids = tree.selection()
            if ids:
                selected[0] = int(ids[0])
                s = stages[selected[0]]
                for value, number in zip(values, (s.rpm, s.at, s.hold)):
                    value.set(f"{number:g}")

        def change(action):
            old = list(stages)
            try:
                if action == "delete":
                    if len(stages) == 1:
                        raise ValueError("최소 한 단계가 필요합니다.")
                    stages.pop(selected[0])
                elif action == "add":
                    previous = stages[-1]
                    stages.append(Stage(0.0, previous.at + previous.hold + 2, 0))
                    selected[0] = len(stages) - 1
                else:
                    stages[selected[0]] = Stage(rpm_value(values[0].get()), finite(values[1].get()),
                                                0 if selected[0] == len(stages) - 1 else finite(values[2].get()))
                Scenario(tuple(stages))
                draw()
                pick()
            except ValueError as exc:
                stages[:] = old
                messagebox.showerror("입력 오류", str(exc), parent=win)

        def save():
            path = filedialog.asksaveasfilename(parent=win, defaultextension=".json", filetypes=[("Scenario", "*.json")])
            if path:
                try:
                    write_json(Path(path), Scenario(tuple(stages)).data())
                except (OSError, ValueError) as exc:
                    messagebox.showerror("저장 오류", str(exc), parent=win)

        def load():
            path = filedialog.askopenfilename(parent=win, filetypes=[("Scenario", "*.json")])
            if path:
                try:
                    scenario = Scenario.from_data(json.loads(Path(path).read_text(encoding="utf-8-sig")))
                    stages[:] = scenario.stages
                    selected[0] = 0
                    draw()
                    pick()
                except (OSError, ValueError, TypeError) as exc:
                    messagebox.showerror("불러오기 오류", str(exc), parent=win)

        def finish(accept):
            result = Scenario(tuple(stages)) if accept else None
            win.destroy()
            done(result)

        def press(event):
            if preview or event.inaxes != ax or canvas.toolbar.mode or event.xdata is None:
                return
            candidates = [(i, "at", s.at, s.rpm) for i, s in enumerate(stages)]
            candidates += [(i, "end", s.at + s.hold, s.rpm) for i, s in enumerate(stages[:-1])]
            for i, kind, x, y in candidates:
                px, py = ax.transData.transform((x, y))
                if math.hypot(event.x - px, event.y - py) <= 10:
                    drag[0] = (i, kind)
                    selected[0] = i
                    break

        def motion(event):
            if drag[0] is None or event.inaxes != ax or event.xdata is None:
                return
            i, kind = drag[0]
            stage = stages[i]
            rpm = round(max(-RPM_LIMIT, min(RPM_LIMIT, event.ydata)), 3)
            if kind == "at":
                lower = 1 if i == 0 else stages[i - 1].at + stages[i - 1].hold
                upper = stages[i + 1].at - stage.hold if i + 1 < len(stages) else float("inf")
                updated = replace(stage, rpm=rpm, at=max(lower, min(upper, round(event.xdata, 3))))
            else:
                end = max(stage.at, min(stages[i + 1].at, round(event.xdata, 3)))
                updated = replace(stage, rpm=rpm, hold=end - stage.at)
            candidate = list(stages)
            candidate[i] = updated
            try:
                Scenario(tuple(candidate))
            except ValueError:
                return
            stages[:] = candidate
            draw()
            pick()

        canvas.mpl_connect("button_press_event", press)
        canvas.mpl_connect("motion_notify_event", motion)
        canvas.mpl_connect("button_release_event", lambda _: drag.__setitem__(0, None))
        tree.bind("<<TreeviewSelect>>", pick)
        if not preview:
            for label, action in (("선택 수정", "update"), ("단계 추가", "add"), ("선택 삭제", "delete")):
                ttk.Button(row, text=label, command=lambda a=action: change(a)).pack(side="left", padx=3)
        buttons = ttk.Frame(win)
        buttons.pack(fill="x", padx=10, pady=8)
        ttk.Button(buttons, text="JSON 저장", command=save).pack(side="left")
        if not preview:
            ttk.Button(buttons, text="JSON 불러오기", command=load).pack(side="left", padx=5)
        ttk.Button(buttons, text="이 그래프 사용 (실행은 CLI Enter)", command=lambda: finish(True)).pack(side="right")
        ttk.Button(buttons, text="취소", command=lambda: finish(False)).pack(side="right", padx=5)
        win.protocol("WM_DELETE_WINDOW", lambda: finish(False))
        draw()
        pick()

    def results(self, trial, done):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        fig = result_figure(trial.path, self.engine.feedback.warning_age)
        fig.savefig(trial.path / "result.png", dpi=160)
        win = tk.Toplevel(self.root)
        win.title("Motor RPM — reference / actual")
        ttk.Label(win, text=f"저장 위치: {trial.path}", wraplength=1000).pack(fill="x", padx=10, pady=6)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(canvas, win)
        canvas.draw()
        def close():
            win.destroy()
            done(None)
        ttk.Button(win, text="닫고 메뉴로", command=close).pack(pady=6)
        win.protocol("WM_DELETE_WINDOW", close)


def result_figure(path, warning_age=0.5):
    from matplotlib.figure import Figure
    fig = Figure(figsize=(10, 5), dpi=100)
    ax = fig.add_subplot(111)
    def rows(name):
        file = path / (name + ".csv")
        with file.open(encoding="utf-8-sig", newline="") as handle:
            yield from csv.DictReader(handle)
    def stride(name):
        with (path / (name + ".csv")).open(encoding="utf-8-sig") as handle:
            count = sum(1 for _ in handle)
        return max(1, math.ceil(count / 30000))
    tx, refs, actual_t, actual = [], [], [], []
    every = stride("commands")
    end = 0.0
    previous_ref = None
    for i, row in enumerate(rows("commands")):
        t = float(row["elapsed_s"])
        end = max(end, t)
        if row["success"] != "1":
            continue
        ref = float(row["reference_rpm"])
        # Keep step edges in addition to the decimated trace.
        if i % every == 0 or previous_ref is not None and abs(ref - previous_ref) > 20:
            tx.append(t)
            refs.append(ref)
        previous_ref = ref
    every, previous_sample, gap_inserted = stride("feedback"), None, False
    sample_index = 0
    for i, row in enumerate(rows("feedback")):
        t = float(row["elapsed_s"])
        end = max(end, t)
        if row["health"] != "OK":
            if not gap_inserted:
                actual_t.append(t)
                actual.append(float("nan"))
                gap_inserted = True
            continue
        if row["new_sample"] == "1":
            if previous_sample is not None and t - previous_sample > warning_age:
                actual_t.append(t)
                actual.append(float("nan"))
            previous_sample = t
            if sample_index % every == 0:
                actual_t.append(t)
                actual.append(float(row["actual_rpm"]))
                gap_inserted = False
            sample_index += 1
    scenario_file = path / "scenario.json"
    if scenario_file.exists():
        scenario = Scenario.from_data(json.loads(scenario_file.read_text(encoding="utf-8")))
        xs, ys = scenario.points(end)
        info = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
        offset = info.get("control_start_elapsed_s", 0.0)
        xs = [x + offset for x in xs]
        # A schedule may be stopped early; show only the recorded time span.
        ax.plot(xs, ys, "--", alpha=0.6, label="Planned reference")
    ax.step(tx, refs, where="post", label="Sent reference", color="#1764b0")
    ax.plot(actual_t, actual, label="Actual (-UDP RPM)", color="#dd7521")
    stale_start = None
    for row in rows("events"):
        if row["event"] == "feedback_health":
            health = json.loads(row["detail"])["health"]
            t = float(row["elapsed_s"])
            if health != "OK" and stale_start is None:
                stale_start = t
            elif health == "OK" and stale_start is not None:
                ax.axvspan(stale_start, t, color="red", alpha=0.08)
                stale_start = None
    if stale_start is not None:
        ax.axvspan(stale_start, end, color="red", alpha=0.08)
    ax.set(xlabel="Elapsed since recording [s]", ylabel="RPM (+ forward per firmware)",
           title="Reference / actual (red shading = feedback warning)", xlim=(0, max(1, end)))
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    return fig


def status_text(engine):
    s = engine.snapshot()
    def value(number, digits=1):
        return "--" if number is None else f"{number:.{digits}f}"
    return (f"{s['mode'].upper()}/{s['phase']} t={s['elapsed_s']:.1f}s "
            f"REF={s['reference_rpm']:.1f} ACT={value(s['actual_rpm'], 0)} RPM [{s['health']}] "
            f"UDP={s['udp_hz']}Hz CAN={s['can_hz']}Hz "
            f"age UDP/CAN={value(s['udp_age_s'], 2)}/{value(s['can_age_s'], 2)}s "
            f"TX={s['sent']} errors={s['tx_errors']} badRX={s['bad_packets']} "
            f"maxGap={s['max_gap_s'] * 1000:.0f}ms" + (f" ERROR={s['error']}" if s['error'] else ""))


class Console:
    def __init__(self, engine, ui):
        from prompt_toolkit import PromptSession
        self.engine, self.ui = engine, ui
        self.prompt = PromptSession()
        self.last_scenario = Scenario((Stage(500.0, 2.0, 2.0), Stage(0.0, 6.0)))

    def ask(self, text):
        return self.prompt.prompt(text, bottom_toolbar=lambda: status_text(self.engine), refresh_interval=0.2).strip()

    def number(self, text, validator):
        while True:
            try:
                return validator(self.ask(text))
            except ValueError as exc:
                print(f"[입력 오류] {exc}")

    def show_result(self, trial):
        if trial:
            print(f"[저장] {trial.path}")
            if trial.error:
                print(f"[LOG ERROR] {trial.error}")
            try:
                self.ui.call(lambda done: self.ui.results(trial, done))
            except (OSError, ValueError, RuntimeError) as exc:
                print(f"[그래프 오류] 로그는 위 폴더에서 확인하세요: {exc}")

    def wizard(self):
        self.ask("안전을 위해 실행 후 처음 1초는 0 RPM입니다. Enter > ")
        def count(text):
            number = finite(text)
            if not number.is_integer() or not 1 <= number <= 1000:
                raise ValueError("1 ~ 1000 정수를 입력하세요.")
            return int(number)
        count = self.number("목표 RPM 단계 수 > ", count)
        stages, previous_end = [], 1.0
        for index in range(count):
            rpm = self.number(f"{index + 1}번째 목표 RPM > ", rpm_value)
            def arrival(text):
                at = finite(text, "도달 시각")
                if at < previous_end:
                    raise ValueError(f"{previous_end:g}초 이상이어야 합니다.")
                return at
            at = self.number(f"{index + 1}번째 목표 도달 시각 [s] > ", arrival)
            hold = 0.0
            if index < count - 1:
                def duration(text):
                    duration = finite(text, "유지 시간")
                    if duration < 0 or not math.isfinite(at + duration):
                        raise ValueError("유한한 0초 이상 시간을 입력하세요.")
                    return duration
                hold = self.number(f"{index + 1}번째 유지 시간 [s, 0 허용] > ", duration)
            stages.append(Stage(rpm, at, hold))
            previous_end = at + hold
        print("마지막 RPM은 기록 종료까지 유지합니다. 그래프에서 사용/취소를 선택하세요.")
        scenario = Scenario(tuple(stages))
        return self.ui.call(lambda done: self.ui.editor(scenario, done, preview=True))

    def run_trial(self, mode, scenario=None):
        while True:
            confirmation = self.ask("Enter를 누르면 기록/송신 시작 (b=취소) > ").lower()
            if confirmation == "b":
                return
            if confirmation == "":
                break
            print("실행하려면 빈 입력에서 Enter, 취소하려면 b를 입력하세요.")
        self.engine.begin(mode, scenario)
        print("[시작] 처음 1초=0. s=기록 종료(0 명령 2초 후 송신 중단), e=즉시 ASMS ESTOP")
        print("명령/피드백 부호는 REF 기준이며, 송신 성공은 보드 수락 ACK가 아닙니다.")
        try:
            while True:
                text = self.ask("auto [s/e]> " if scenario else ("rpm [s/e]> " if mode == "pc" else "joy_x [s/e]> ")).lower()
                if text in ("s", "stop", "b", "q"):
                    trial = self.engine.finish()
                    break
                if text in ("e", "estop"):
                    trial = self.engine.finish(estop=True)
                    print("[ESTOP 요청] PC 재개 전 ASMS AUTO(a) 필요. 정지 확인 ACK는 없습니다.")
                    break
                if text == "status":
                    print(status_text(self.engine))
                elif text.startswith("fault ") and isinstance(self.engine.transport, Simulation):
                    fault = text.split(maxsplit=1)[1]
                    if fault in ("udp", "can", "sentinel", "none"):
                        with self.engine.transport.lock:
                            self.engine.transport.fault = fault
                        self.engine.event("simulation_fault", fault)
                    else:
                        print("fault udp / can / sentinel / none")
                elif not scenario:
                    try:
                        self.engine.set_target(text)
                    except ValueError as exc:
                        print(f"[입력 오류] {exc}")
                else:
                    print("s=기록 종료 / e=ESTOP / status (모의 실행: fault udp|can|sentinel|none)")
        except (EOFError, KeyboardInterrupt):
            self.engine.finish()
            raise
        self.show_result(trial)

    def pc_menu(self):
        while True:
            text = self.ask("PC: 1=수동 RPM 2=그래프 편집 3=질문식 자동 a=ASMS AUTO b=뒤로 > ").lower()
            try:
                if text == "b":
                    return
                if text == "a":
                    ok = self.engine.asms_command(1)
                    print(f"ASMS AUTO {'송신 완료' if ok else '송신 오류'} (보드 ACK 아님)")
                elif text == "1":
                    self.run_trial("pc")
                elif text in ("2", "3"):
                    scenario = (self.ui.call(lambda done: self.ui.editor(self.last_scenario, done))
                                if text == "2" else self.wizard())
                    if scenario:
                        self.last_scenario = scenario
                        self.run_trial("pc", scenario)
            except (ValueError, OSError) as exc:
                print(f"[오류] {exc}")

    def asms_menu(self):
        while True:
            text = self.ask("ASMS: 1=모터 joy_x 입력 2=AUTO 3=ESTOP b=뒤로 > ").lower()
            if text == "b":
                return
            if text == "1":
                print("joy_x=-2048~2047, 중립=0(±120), 음수=전진. 조향 입력은 없습니다.")
                self.run_trial("asms")
            elif text in ("2", "3"):
                mode = 1 if text == "2" else 3
                ok = self.engine.asms_command(mode)
                print(f"ASMS {'AUTO' if mode == 1 else 'ESTOP'} {'송신 완료' if ok else '송신 오류'} (보드 ACK 아님)")

    def run(self):
        from prompt_toolkit.patch_stdout import patch_stdout
        def reporter():
            while not self.engine.done.wait(1):
                state = self.engine.snapshot()
                # Also prints while a GUI is open and the CLI prompt is not running.
                print(("[상태] " if state["health"] == "OK" else "[피드백 경고] ") + status_text(self.engine))
        report = threading.Thread(target=reporter, name="cli-status", daemon=True)
        with patch_stdout():
            report.start()
            print("Motor-only UDP tuner | " + ("DRY RUN: 소켓/장비 송신 없음" if isinstance(self.engine.transport, Simulation) else "실제 UDP 통신"))
            print(f"RPM ±{RPM_LIMIT}; +입력→보드 내부 -RPM; actual 표시=-raw. 경고는 시험을 자동 종료하지 않습니다.")
            print("초기 보드 모드는 알 수 없습니다. PC 명령이 차단되어 있으면 PC 메뉴 a로 ASMS AUTO를 전송하세요.")
            print(f"전체 세션 기록: {self.engine.session.path}")
            try:
                while True:
                    mode = self.ask("모드 PC / ASMS / QUIT > ").lower()
                    if mode in ("quit", "q"):
                        break
                    if mode == "pc":
                        self.pc_menu()
                    elif mode == "asms":
                        self.asms_menu()
            except (EOFError, KeyboardInterrupt):
                print("입력 종료: 0 명령 2초 후 송신 중단/저장합니다.")
            finally:
                self.engine.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="no sockets: simulated motor feedback")
    parser.add_argument("--setup-network", action="store_true", help="Windows admin: temporarily add missing .1/.5 addresses")
    parser.add_argument("--adapter", default="이더넷", help="exact Windows test Ethernet adapter name")
    parser.add_argument("--warning-age", type=float, default=0.5, help="feedback warning threshold, NOT auto-stop timeout")
    parser.add_argument("--logs", default=str(HERE / "logs"), help="local session/trial directory")
    args = parser.parse_args(argv)
    if not math.isfinite(args.warning_age) or args.warning_age <= 0:
        parser.error("--warning-age must be positive and finite")
    try:
        import tkinter as tk
        import matplotlib
        import prompt_toolkit
    except ImportError as exc:
        print(f"설치가 필요합니다: {exc}\ninstall.cmd 또는 python -m pip install -r requirements.txt")
        return 1
    if os.name == "nt":
        matplotlib.rcParams["font.family"] = "Malgun Gothic"
    matplotlib.rcParams["axes.unicode_minus"] = False
    root = None
    engine = network = transport = startup_log = None
    try:
        root = tk.Tk()
        root.withdraw()
        info = metadata(args)
        if args.dry_run:
            transport = Simulation()
        else:
            startup_log = Journal(args.logs, "network_setup", info)
            network = NetworkSetup(args.adapter, startup_log)
            network.ensure(args.setup_network)
            transport = UDPTransport()
        engine = Engine(transport, args.logs, info, args.warning_age)
        ui = UI(root, engine)
        console = Console(engine, ui)
        engine.start()
        finished = threading.Event()
        def cli():
            try:
                console.run()
            except Exception:
                print(traceback.format_exc())
                engine.event("console_failed", traceback.format_exc())
                engine.close()
            finally:
                finished.set()
        threading.Thread(target=cli, name="cli", daemon=True).start()
        def poll():
            if finished.is_set():
                root.quit()
            else:
                root.after(100, poll)
        root.after(100, poll)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            # Main-thread Ctrl+C while a graph is open still performs normal shutdown.
            engine.close()
        return 0
    except (OSError, RuntimeError, tk.TclError, subprocess.SubprocessError) as exc:
        print(f"[시작 오류] {exc}\n네트워크/설치 README를 확인하세요.")
        return 1
    finally:
        if engine and not engine.done.is_set():
            engine.close()
        elif transport:
            transport.close()
        if network:
            network.close()
        if startup_log:
            startup_log.close()
        if root:
            root.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
