"""Acquisizione multi-VCOM, calibrazione e localizzazione 2D BLE CS."""

import argparse
import csv
import json
import math
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import serial


ANCHOR_IDS = (0, 1, 2)
DEFAULT_ANCHORS = {
    0: (0.0, 0.0),
    1: (1.5, 0.0),
    2: (0.75, 1.2),
}
# Alias mantenuto per compatibilità con eventuali import esterni dello script.
ANCHORS = DEFAULT_ANCHORS

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "rtls_config.json"
DEFAULT_CALIBRATION_DATASET_PATH = SCRIPT_DIR / "calibration_dataset.json"
DEFAULT_MEASUREMENTS_DIR = SCRIPT_DIR / "measurements"

# Valori specifici delle board attualmente assegnate; --port permette l'override.
DEFAULT_PORTS = {
    0: "/dev/cu.usbmodem0010577003343",
    1: "/dev/cu.usbmodem0010577446553",
    2: "/dev/cu.usbmodem0010577541043",
}

DEFAULT_BAUD = 115200
DEFAULT_MAX_AGE_S = 6.0
DEFAULT_MAX_SKEW_S = 4.0
DEFAULT_MIN_DISTANCE_M = 0.05
# Applied to raw serial measurements before calibration; it must include the
# expected uncalibrated anchor bias.
DEFAULT_MAX_DISTANCE_M = 3.0
DEFAULT_STATUS_INTERVAL_S = 5.0
DEFAULT_CALIBRATION_SNAPSHOTS = 100
DEFAULT_MAX_CALIBRATION_RMSE_M = 0.25
# Mode 3/PBR can produce a systematic phase-slope compression which requires
# gains above one (A1 in the controlled three-point campaign is about 2.17).
# Keep the scale positive to reject inverted fits, but let the measured fit and
# its RMSE decide whether the calibration is usable.
MIN_CALIBRATION_SCALE = 0.3
MAX_CALIBRATION_SCALE = 3.0
MAX_ABS_CALIBRATION_OFFSET_M = 3.0
DEFAULT_DISTANCE_FILTER_WINDOW = 5
DEFAULT_OUTLIER_SIGMA = 4.0
DEFAULT_OUTLIER_FLOOR_M = 0.20
DEFAULT_POSITION_ALPHA = 0.35
DEFAULT_VALIDATION_SAMPLES = 200
DEFAULT_VALIDATION_WARMUP = 10
DEFAULT_TOKEN_TIMEOUT_S = 8.0
DEFAULT_TOKEN_RETRY_DELAY_S = 0.25
DEFAULT_PBR_BRANCH_MAX_STEP_M = 0.40

NUMBER_PATTERN = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
DISTANCE_PATTERN = re.compile(
    rf"^DIST_DATA:ANCHOR:(?P<anchor>\d+)"
    rf"\|SEQ:(?P<seq>\d+)"
    rf"\|T_MS:(?P<device_time>\d+)"
    rf"\|METHOD:(?P<method>[A-Z_]+)"
    rf"\|RAW_VAL:(?P<distance>{NUMBER_PATTERN})"
    rf"\|SAMPLES:(?P<samples>\d+)"
    rf"\|QUALITY:(?P<quality>[A-Z_]+)"
    rf"(?P<extras>(?:\|[A-Z0-9_]+:[^|\r\n]+)*)$"
)
CS_STATUS_PATTERN = re.compile(
    r"^CS_STATUS:ANCHOR:(?P<anchor>\d+)"
    r"\|STATE:(?P<state>[A-Z_]+)"
    r"(?:\|CODE:(?P<code>-?\d+))?$"
)


@dataclass(frozen=True)
class DistanceSample:
    anchor_id: int
    sequence: int
    device_time_ms: int
    method: str
    distance_m: float
    sample_count: int
    quality: str
    received_monotonic: float
    pbr_valid: bool = False
    pbr_distance_m: float = None
    pbr_samples: int = 0
    pbr_rmse_rad: float = None
    rtt_valid: bool = False
    rtt_distance_m: float = None
    rtt_samples: int = 0
    rtt_stddev_m: float = None
    rtt_records: int = 0
    rtt_aa_failures: int = 0
    rtt_rssi_missing: int = 0
    rtt_timing_missing: int = 0
    rtt_diagnostic_m: float = None


@dataclass(frozen=True)
class AnchorStatus:
    anchor_id: int
    state: str
    code: int
    received_monotonic: float


@dataclass(frozen=True)
class SerialLinkStatus:
    anchor_id: int
    connected: bool
    received_monotonic: float


@dataclass(frozen=True)
class DistanceCalibration:
    scale: float = 1.0
    offset_m: float = 0.0

    def apply(self, raw_distance_m):
        return self.scale * raw_distance_m + self.offset_m


@dataclass(frozen=True)
class RtlsConfig:
    anchors: dict
    calibrations: dict
    max_rmse_m: float


@dataclass(frozen=True)
class PositionSolution:
    position: tuple
    rmse_m: float
    max_residual_m: float
    residuals_m: tuple
    iterations: int
    converged: bool


def default_rtls_config():
    return RtlsConfig(
        anchors=DEFAULT_ANCHORS.copy(),
        calibrations={anchor_id: DistanceCalibration() for anchor_id in ANCHOR_IDS},
        max_rmse_m=0.25,
    )


def _validate_anchor_geometry(anchors):
    p0, p1, p2 = (np.asarray(anchors[anchor_id], dtype=float) for anchor_id in ANCHOR_IDS)
    geometry = np.vstack((p1 - p0, p2 - p0))
    if np.linalg.matrix_rank(geometry) < 2:
        raise ValueError("le coordinate delle anchor sono collineari o duplicate")


def load_rtls_config(path):
    path = Path(path)
    if not path.exists():
        print(f"[WARN] Configurazione {path} non trovata: uso valori neutri")
        return default_rtls_config()

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        anchor_data = data["anchors_m"]
        calibration_data = data["distance_calibration"]
        max_rmse_m = float(data["quality"]["max_rmse_m"])
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        raise ValueError(f"configurazione RTLS non valida in {path}: {exc}") from exc

    anchors = {}
    calibrations = {}
    for anchor_id in ANCHOR_IDS:
        key = str(anchor_id)
        try:
            coordinates = tuple(float(value) for value in anchor_data[key])
            if len(coordinates) != 2 or not all(math.isfinite(value) for value in coordinates):
                raise ValueError("coordinate non finite o non 2D")
            calibration = calibration_data[key]
            scale = float(calibration["scale"])
            offset_m = float(calibration["offset_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"configurazione incompleta per anchor {anchor_id}: {exc}") from exc

        if not math.isfinite(scale) or scale <= 0 or not math.isfinite(offset_m):
            raise ValueError(f"calibrazione non valida per anchor {anchor_id}")
        anchors[anchor_id] = coordinates
        calibrations[anchor_id] = DistanceCalibration(scale, offset_m)

    if not math.isfinite(max_rmse_m) or max_rmse_m <= 0:
        raise ValueError("quality.max_rmse_m deve essere positivo")

    _validate_anchor_geometry(anchors)
    return RtlsConfig(anchors, calibrations, max_rmse_m)


def save_rtls_config(path, config):
    path = Path(path)
    payload = {
        "version": 1,
        "anchors_m": {
            str(anchor_id): list(config.anchors[anchor_id]) for anchor_id in ANCHOR_IDS
        },
        "distance_calibration": {
            str(anchor_id): {
                "scale": config.calibrations[anchor_id].scale,
                "offset_m": config.calibrations[anchor_id].offset_m,
            }
            for anchor_id in ANCHOR_IDS
        },
        "quality": {"max_rmse_m": config.max_rmse_m},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(path)


def parse_distance_line(line, received_monotonic=None):
    """Converte un record DIST_DATA completo; restituisce None se non valido."""
    marker = line.find("DIST_DATA:")
    if marker < 0:
        return None

    match = DISTANCE_PATTERN.fullmatch(line[marker:].strip())
    if not match:
        return None

    try:
        distance_m = float(match.group("distance"))
        anchor_id = int(match.group("anchor"))
        sequence = int(match.group("seq"))
        device_time_ms = int(match.group("device_time"))
        sample_count = int(match.group("samples"))
    except ValueError:
        return None

    quality = match.group("quality")
    if (
        anchor_id not in ANCHOR_IDS
        or quality != "OK"
        or sample_count <= 0
        or not math.isfinite(distance_m)
    ):
        return None

    if received_monotonic is None:
        received_monotonic = time.monotonic()

    extras = {}
    for field in match.group("extras").split("|"):
        if field:
            key, value = field.split(":", 1)
            extras[key] = value

    extended = "PBR_VALID" in extras or "RTT_VALID" in extras
    try:
        pbr_valid = extras.get("PBR_VALID") == "1"
        pbr_distance_m = (
            float(extras["PBR_VAL"])
            if pbr_valid and "PBR_VAL" in extras
            else None
        )
        pbr_samples = int(extras.get("PBR_SAMPLES", "0"))
        pbr_rmse_rad = (
            float(extras["PBR_RMSE"])
            if pbr_valid and "PBR_RMSE" in extras
            else None
        )
        rtt_valid = extras.get("RTT_VALID") == "1"
        rtt_distance_m = (
            float(extras["RTT_VAL"])
            if rtt_valid and "RTT_VAL" in extras
            else None
        )
        rtt_samples = int(extras.get("RTT_SAMPLES", "0"))
        rtt_stddev_m = (
            float(extras["RTT_STD"])
            if rtt_valid and "RTT_STD" in extras
            else None
        )
        rtt_records = int(extras.get("RTT_RECORDS", "0"))
        rtt_aa_failures = int(extras.get("RTT_AA_FAIL", "0"))
        rtt_rssi_missing = int(extras.get("RTT_RSSI_MISS", "0"))
        rtt_timing_missing = int(extras.get("RTT_TIME_MISS", "0"))
        rtt_diagnostic_m = (
            float(extras["RTT_DIAG_VAL"])
            if "RTT_DIAG_VAL" in extras
            else None
        )
    except (KeyError, ValueError):
        return None

    if not extended and match.group("method") in {"PBR", "PHASE"}:
        pbr_valid = True
        pbr_distance_m = distance_m
        pbr_samples = sample_count

    numeric_metrics = (
        pbr_distance_m,
        pbr_rmse_rad,
        rtt_distance_m,
        rtt_stddev_m,
        rtt_diagnostic_m,
    )
    if any(value is not None and not math.isfinite(value) for value in numeric_metrics):
        return None

    return DistanceSample(
        anchor_id=anchor_id,
        sequence=sequence,
        device_time_ms=device_time_ms,
        method=match.group("method"),
        distance_m=distance_m,
        sample_count=sample_count,
        quality=quality,
        received_monotonic=received_monotonic,
        pbr_valid=pbr_valid,
        pbr_distance_m=pbr_distance_m,
        pbr_samples=pbr_samples,
        pbr_rmse_rad=pbr_rmse_rad,
        rtt_valid=rtt_valid,
        rtt_distance_m=rtt_distance_m,
        rtt_samples=rtt_samples,
        rtt_stddev_m=rtt_stddev_m,
        rtt_records=rtt_records,
        rtt_aa_failures=rtt_aa_failures,
        rtt_rssi_missing=rtt_rssi_missing,
        rtt_timing_missing=rtt_timing_missing,
        rtt_diagnostic_m=rtt_diagnostic_m,
    )


def select_distance_source(sample, source, fusion_guard_m):
    """Seleziona PBR, RTT o una fusione sorvegliata dalla stessa procedura Mode 3."""
    if source == "phase":
        if not sample.pbr_valid:
            return None
        return replace(
            sample,
            method="PHASE",
            distance_m=sample.pbr_distance_m,
            sample_count=sample.pbr_samples,
        )

    if source == "rtt":
        if not sample.rtt_valid:
            return None
        return replace(
            sample,
            method="RTT",
            distance_m=sample.rtt_distance_m,
            sample_count=sample.rtt_samples,
        )

    if source != "fused":
        raise ValueError(f"sorgente distanza non supportata: {source}")

    if sample.pbr_valid and sample.rtt_valid:
        discrepancy_m = abs(sample.pbr_distance_m - sample.rtt_distance_m)
        if discrepancy_m <= fusion_guard_m:
            # PBR ha normalmente una precisione migliore; RTT funge da controllo
            # indipendente e non altera la misura quando le due stime concordano.
            return replace(
                sample,
                method="FUSED_PBR",
                distance_m=sample.pbr_distance_m,
                sample_count=sample.pbr_samples,
            )
        # Le due sorgenti hanno bias diversi e richiedono calibrazioni
        # indipendenti. Prima di averle, una discordanza è un veto su RTT,
        # non un motivo per sostituire una misura PBR altrimenti valida.
        return replace(
            sample,
            method="FUSED_PBR_GUARD",
            distance_m=sample.pbr_distance_m,
            sample_count=sample.pbr_samples,
        )

    if sample.pbr_valid:
        return replace(
            sample,
            method="FUSED_PBR_ONLY",
            distance_m=sample.pbr_distance_m,
            sample_count=sample.pbr_samples,
        )
    if sample.rtt_valid:
        return replace(
            sample,
            method="FUSED_RTT_ONLY",
            distance_m=sample.rtt_distance_m,
            sample_count=sample.rtt_samples,
        )
    return None


def parse_cs_status_line(line, received_monotonic=None):
    """Converte un record CS_STATUS emesso dal firmware comandato."""
    marker = line.find("CS_STATUS:")
    if marker < 0:
        return None

    match = CS_STATUS_PATTERN.fullmatch(line[marker:].strip())
    if not match:
        return None

    anchor_id = int(match.group("anchor"))
    if anchor_id not in ANCHOR_IDS:
        return None

    if received_monotonic is None:
        received_monotonic = time.monotonic()

    code_text = match.group("code")
    return AnchorStatus(
        anchor_id=anchor_id,
        state=match.group("state"),
        code=int(code_text) if code_text is not None else 0,
        received_monotonic=received_monotonic,
    )


class SerialReader(threading.Thread):
    """Reader con riconnessione automatica per una singola VCOM."""

    def __init__(
        self,
        anchor_id,
        port_name,
        baud,
        output_queue,
        stop_event,
        min_distance_m,
        max_distance_m,
    ):
        super().__init__(name=f"anchor-{anchor_id}-serial", daemon=True)
        self.anchor_id = anchor_id
        self.port_name = port_name
        self.baud = baud
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m
        self._last_sequence = None
        self._last_device_time_ms = None
        self._serial = None
        self._serial_lock = threading.Lock()

    def send_command(self, command):
        """Invia una riga al firmware senza interrompere il reader."""
        payload = (command.rstrip("\r\n") + "\n").encode("ascii")
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return False
            try:
                self._serial.write(payload)
                self._serial.flush()
            except (serial.SerialException, OSError):
                return False
        return True

    def _publish_link_status(self, connected):
        try:
            self.output_queue.put(
                SerialLinkStatus(self.anchor_id, connected, time.monotonic()),
                timeout=0.1,
            )
        except queue.Full:
            print(f"[WARN] Coda host piena: stato VCOM A{self.anchor_id} scartato")

    def run(self):
        while not self.stop_event.is_set():
            try:
                with serial.Serial(self.port_name, self.baud, timeout=0.5) as ser:
                    with self._serial_lock:
                        self._serial = ser
                    print(f"[INFO] Anchor {self.anchor_id} connessa a {self.port_name}")
                    self._publish_link_status(True)
                    try:
                        self._read_port(ser)
                    finally:
                        with self._serial_lock:
                            self._serial = None
                        self._publish_link_status(False)
            except (serial.SerialException, OSError, ValueError) as exc:
                with self._serial_lock:
                    self._serial = None
                if not self.stop_event.is_set():
                    print(
                        f"[WARN] Anchor {self.anchor_id}, porta {self.port_name}: "
                        f"{exc}; nuovo tentativo fra 1 s"
                    )
                    self.stop_event.wait(1.0)

    def _read_port(self, ser):
        while not self.stop_event.is_set():
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            received_at = time.monotonic()
            status = parse_cs_status_line(line, received_at)
            if status is not None:
                if status.anchor_id != self.anchor_id:
                    print(
                        f"[WARN] Stato firmware A{status.anchor_id} ricevuto sulla "
                        f"porta di A{self.anchor_id}: ignorato"
                    )
                    continue
                try:
                    self.output_queue.put(status, timeout=0.1)
                except queue.Full:
                    print(
                        f"[WARN] Coda host piena: stato anchor "
                        f"{self.anchor_id} scartato"
                    )
                continue

            sample = parse_distance_line(line, received_at)
            if sample is None:
                continue

            if sample.anchor_id != self.anchor_id:
                print(
                    f"[WARN] ID firmware {sample.anchor_id} ricevuto sulla porta "
                    f"configurata per l'anchor {self.anchor_id}: campione ignorato"
                )
                continue

            if not self.min_distance_m <= sample.distance_m <= self.max_distance_m:
                print(
                    f"[WARN] Distanza fuori range da anchor {self.anchor_id}: "
                    f"{sample.distance_m:.3f} m"
                )
                continue

            if self._is_duplicate_or_stale(sample):
                print(
                    f"[WARN] Record duplicato/fuori sequenza da anchor "
                    f"{self.anchor_id}: SEQ={sample.sequence}"
                )
                continue

            self._last_sequence = sample.sequence
            self._last_device_time_ms = sample.device_time_ms

            try:
                self.output_queue.put(sample, timeout=0.1)
            except queue.Full:
                print(f"[WARN] Coda host piena: campione anchor {self.anchor_id} scartato")

    def _is_duplicate_or_stale(self, sample):
        if self._last_sequence is None:
            return False

        # k_uptime_get() riparte da zero dopo un reboot.
        if sample.device_time_ms < self._last_device_time_ms:
            return False

        return sample.sequence <= self._last_sequence


class CsTokenCoordinator:
    """Serializza le procedure CS sulle tre VCOM mantenendo le letture parallele."""

    def __init__(self, readers, timeout_s, retry_delay_s):
        self.readers = {reader.anchor_id: reader for reader in readers}
        self.timeout_s = timeout_s
        self.retry_delay_s = retry_delay_s
        # Una risposta READY al primo CS_PING può andare persa durante
        # l'apertura/reset della VCOM. Ripetere il probe impedisce che una
        # singola anchor connessa ma senza stato blocchi l'intero coordinatore.
        self.status_probe_interval_s = max(1.0, retry_delay_s)
        self.next_status_probe_at = 0.0
        self.connected = set()
        self.ready = set()
        self.active_anchor = None
        self.awaiting_release = None
        self.deadline = 0.0
        self.next_request_at = 0.0
        self.next_index = 0
        self.completed_cycles = 0
        self.timeouts = 0
        self.errors = 0

    def on_link_status(self, event):
        anchor_id = event.anchor_id
        if event.connected:
            self.connected.add(anchor_id)
            self.readers[anchor_id].send_command("CS_PING")
            self.next_status_probe_at = (
                event.received_monotonic + self.status_probe_interval_s
            )
            return

        self.connected.discard(anchor_id)
        self.ready.discard(anchor_id)
        if self.active_anchor == anchor_id:
            self.active_anchor = None
        if self.awaiting_release == anchor_id:
            self.awaiting_release = None
        self.next_request_at = event.received_monotonic + self.retry_delay_s

    def on_anchor_status(self, status):
        anchor_id = status.anchor_id
        state = status.state

        if state == "READY":
            self.ready.add(anchor_id)
            if self.awaiting_release == anchor_id:
                self.awaiting_release = None
                self.next_request_at = status.received_monotonic + self.retry_delay_s
        elif state in {"QUEUED", "BUSY"}:
            self.ready.discard(anchor_id)
        elif state == "ERROR":
            self.errors += 1
            self.ready.discard(anchor_id)
            if self.active_anchor == anchor_id:
                self.active_anchor = None
                self.awaiting_release = anchor_id
                self.deadline = status.received_monotonic + self.timeout_s
                # Non affamare le altre anchor se questa connessione deve
                # recuperare: al ritorno di tutte le READY il giro riparte
                # dall'anchor successiva.
                self.next_index = (self.next_index + 1) % len(ANCHOR_IDS)
            print(f"[TOKEN] A{anchor_id} misura fallita (codice {status.code})")
        elif state in {
            "BOOT",
            "CONFIGURING",
            "DISCONNECTED",
            "NOT_READY",
            "BAD_COMMAND",
            "RECOVERING",
        }:
            self.ready.discard(anchor_id)

    def accept_sample(self, sample, now):
        """Accetta soltanto il DIST_DATA prodotto dal proprietario del token."""
        if sample.anchor_id != self.active_anchor:
            print(
                f"[TOKEN] DIST_DATA inatteso da A{sample.anchor_id}: "
                "campione ignorato"
            )
            return False

        completed_anchor = self.active_anchor
        self.active_anchor = None
        self.awaiting_release = completed_anchor
        self.deadline = now + self.timeout_s
        self.next_index = (self.next_index + 1) % len(ANCHOR_IDS)
        if self.next_index == 0:
            self.completed_cycles += 1
        return True

    def tick(self, now):
        if self.active_anchor is not None and now >= self.deadline:
            anchor_id = self.active_anchor
            self.timeouts += 1
            self.active_anchor = None
            self.awaiting_release = anchor_id
            self.deadline = now + self.timeout_s
            self.readers[anchor_id].send_command("CS_PING")
            print(f"[TOKEN] Timeout misura A{anchor_id}; attendo rilascio CS")
            return

        if self.awaiting_release is not None:
            if now >= self.deadline:
                anchor_id = self.awaiting_release
                self.deadline = now + self.timeout_s
                self.readers[anchor_id].send_command("CS_PING")
                print(f"[TOKEN] A{anchor_id} non ha confermato READY; richiesto stato")
            return

        if self.active_anchor is not None:
            return

        missing_ready = self.connected - self.ready
        if missing_ready:
            if now >= self.next_status_probe_at:
                for anchor_id in sorted(missing_ready):
                    self.readers[anchor_id].send_command("CS_PING")
                self.next_status_probe_at = now + self.status_probe_interval_s
                anchors = ",".join(f"A{value}" for value in sorted(missing_ready))
                print(f"[TOKEN] Richiesto stato a {anchors}")
            return

        if now < self.next_request_at:
            return

        if self.connected != set(ANCHOR_IDS):
            return

        anchor_id = ANCHOR_IDS[self.next_index]
        if not self.readers[anchor_id].send_command("CS_MEASURE"):
            self.ready.discard(anchor_id)
            self.next_request_at = now + self.retry_delay_s
            return

        self.active_anchor = anchor_id
        self.ready.discard(anchor_id)
        self.deadline = now + self.timeout_s
        print(f"[TOKEN] Concesso ad A{anchor_id}")

    def format_status(self):
        active = "nessuna" if self.active_anchor is None else f"A{self.active_anchor}"
        release = (
            "nessuna" if self.awaiting_release is None else f"A{self.awaiting_release}"
        )
        ready = ",".join(f"A{value}" for value in sorted(self.ready)) or "nessuna"
        return (
            f"[TOKEN] attiva={active} rilascio={release} ready={ready} "
            f"cicli={self.completed_cycles} timeout={self.timeouts} errori={self.errors}"
        )


class SampleSynchronizer:
    """Crea una terna solo dopo un nuovo campione da ciascuna anchor."""

    def __init__(self, anchor_ids):
        self.anchor_ids = tuple(sorted(anchor_ids))
        self.latest = {}
        self.generations = {anchor_id: 0 for anchor_id in self.anchor_ids}
        self.last_used_generations = {anchor_id: 0 for anchor_id in self.anchor_ids}

    def add(self, sample):
        self.latest[sample.anchor_id] = sample
        self.generations[sample.anchor_id] += 1

    def complete_snapshot(self):
        """Restituisce una terna nuova senza imporre vincoli temporali.

        La calibrazione avviene con il tag fermo e le procedure CS sono
        intenzionalmente serializzate. In questo caso lo skew tra A0, A1 e A2
        non indica movimento e non deve impedire la raccolta della terna.
        """
        if any(anchor_id not in self.latest for anchor_id in self.anchor_ids):
            return None

        if any(
            self.generations[anchor_id] <= self.last_used_generations[anchor_id]
            for anchor_id in self.anchor_ids
        ):
            return None

        snapshot = {anchor_id: self.latest[anchor_id] for anchor_id in self.anchor_ids}
        self.last_used_generations = self.generations.copy()
        return snapshot

    def coherent_snapshot(self, now, max_age_s, max_skew_s):
        if any(anchor_id not in self.latest for anchor_id in self.anchor_ids):
            return None

        if any(
            self.generations[anchor_id] <= self.last_used_generations[anchor_id]
            for anchor_id in self.anchor_ids
        ):
            return None

        snapshot = {anchor_id: self.latest[anchor_id] for anchor_id in self.anchor_ids}
        receipt_times = [sample.received_monotonic for sample in snapshot.values()]

        if now - min(receipt_times) > max_age_s:
            return None
        if max(receipt_times) - min(receipt_times) > max_skew_s:
            return None

        self.last_used_generations = self.generations.copy()
        return snapshot


def format_anchor_status(synchronizer, now):
    """Descrive l'ultimo campione grezzo valido ricevuto da ogni anchor."""
    entries = []
    for anchor_id in synchronizer.anchor_ids:
        sample = synchronizer.latest.get(anchor_id)
        if sample is None:
            entries.append(f"A{anchor_id}: nessun DIST_DATA")
            continue

        age_s = max(0.0, now - sample.received_monotonic)
        pbr_text = (
            f"PBR={sample.pbr_distance_m:.3f}m"
            if sample.pbr_valid
            else "PBR=NA"
        )
        rtt_text = (
            f"RTT={sample.rtt_distance_m:.3f}m"
            if sample.rtt_valid
            else (
                "RTT=NA("
                f"rec={sample.rtt_records} cand={sample.rtt_samples} "
                f"aa={sample.rtt_aa_failures} "
                f"rssi={sample.rtt_rssi_missing} "
                f"time={sample.rtt_timing_missing} "
                f"diag={sample.rtt_diagnostic_m:.3f}m)"
                if sample.rtt_records > 0 and sample.rtt_diagnostic_m is not None
                else "RTT=NA"
            )
        )
        entries.append(
            f"A{anchor_id}: raw={sample.distance_m:.3f}m {sample.method} "
            f"{pbr_text} {rtt_text} N={sample.sample_count} "
            f"SEQ={sample.sequence} eta={age_s:.1f}s"
        )

    return "[STATO] " + " | ".join(entries)


def linear_trilaterate(distances, anchors):
    """Calcola una stima lineare usata per inizializzare il solver non lineare."""
    reference_id = ANCHOR_IDS[0]
    p0 = anchors[reference_id]
    d0 = distances[reference_id]
    matrix = []
    vector = []

    for anchor_id in ANCHOR_IDS[1:]:
        point = anchors[anchor_id]
        distance = distances[anchor_id]
        matrix.append([2 * (point[0] - p0[0]), 2 * (point[1] - p0[1])])
        vector.append(
            d0**2
            - distance**2
            - p0[0] ** 2
            + point[0] ** 2
            - p0[1] ** 2
            + point[1] ** 2
        )

    try:
        position, _, rank, _ = np.linalg.lstsq(
            np.asarray(matrix, dtype=float), np.asarray(vector, dtype=float), rcond=None
        )
    except np.linalg.LinAlgError:
        return None

    if rank < 2 or not np.all(np.isfinite(position)):
        return None
    return np.asarray(position, dtype=float)


def trilaterate(d0, d1, d2):
    """Wrapper compatibile con la precedente API lineare."""
    position = linear_trilaterate({0: d0, 1: d1, 2: d2}, DEFAULT_ANCHORS)
    if position is None:
        return None
    return float(position[0]), float(position[1])


def solve_position(distances, anchors, max_iterations=30):
    """Minimizza i residui delle tre circonferenze con Levenberg-Marquardt."""
    position = linear_trilaterate(distances, anchors)
    if position is None:
        return None

    anchor_matrix = np.asarray([anchors[anchor_id] for anchor_id in ANCHOR_IDS], dtype=float)
    measured = np.asarray([distances[anchor_id] for anchor_id in ANCHOR_IDS], dtype=float)
    if np.any(~np.isfinite(measured)) or np.any(measured <= 0):
        return None

    damping = 1e-3
    converged = False
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        offsets = position - anchor_matrix
        predicted = np.linalg.norm(offsets, axis=1)
        predicted = np.maximum(predicted, 1e-9)
        residuals = predicted - measured
        jacobian = offsets / predicted[:, None]

        lhs = jacobian.T @ jacobian + damping * np.eye(2)
        rhs = -(jacobian.T @ residuals)
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            return None

        candidate = position + step
        candidate_residuals = np.linalg.norm(candidate - anchor_matrix, axis=1) - measured
        if np.dot(candidate_residuals, candidate_residuals) <= np.dot(residuals, residuals):
            position = candidate
            damping = max(damping / 3.0, 1e-9)
            if np.linalg.norm(step) < 1e-6:
                converged = True
                break
        else:
            damping = min(damping * 10.0, 1e9)

    final_residuals = np.linalg.norm(position - anchor_matrix, axis=1) - measured
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(final_residuals)):
        return None

    return PositionSolution(
        position=(float(position[0]), float(position[1])),
        rmse_m=float(np.sqrt(np.mean(final_residuals**2))),
        max_residual_m=float(np.max(np.abs(final_residuals))),
        residuals_m=tuple(float(value) for value in final_residuals),
        iterations=iterations,
        converged=converged,
    )


def _load_calibration_dataset(path):
    path = Path(path)
    if not path.exists():
        return {"version": 1, "sessions": []}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"dataset di calibrazione non valido in {path}: {exc}") from exc
    if data.get("version") != 1 or not isinstance(data.get("sessions"), list):
        raise ValueError(f"formato dataset di calibrazione non supportato in {path}")
    return data


def build_calibration_dataset(path, target_position, snapshots):
    """Aggiunge una sessione in memoria senza modificare il file su disco."""
    dataset = _load_calibration_dataset(path)
    session = {
        "position_m": [float(target_position[0]), float(target_position[1])],
        "captured_at_unix": time.time(),
        "raw_distances_m": {
            str(anchor_id): [
                float(snapshot[anchor_id].distance_m) for snapshot in snapshots
            ]
            for anchor_id in ANCHOR_IDS
        },
    }
    dataset["sessions"].append(session)
    return dataset


def save_calibration_dataset(path, dataset):
    """Salva atomicamente un dataset già validato."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(dataset, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(path)


def append_calibration_session(path, target_position, snapshots):
    """API compatibile: crea e salva una nuova sessione di calibrazione."""
    dataset = build_calibration_dataset(path, target_position, snapshots)
    save_calibration_dataset(path, dataset)
    return dataset


def fit_distance_calibrations(dataset, anchors):
    """Fitta true_distance = scale * raw_distance + offset per anchor."""
    calibrations = {}
    statistics = {}

    for anchor_id in ANCHOR_IDS:
        raw_values = []
        true_values = []
        target_distances = []

        for session in dataset["sessions"]:
            target = np.asarray(session["position_m"], dtype=float)
            anchor = np.asarray(anchors[anchor_id], dtype=float)
            true_distance = float(np.linalg.norm(target - anchor))
            samples = [float(value) for value in session["raw_distances_m"][str(anchor_id)]]
            raw_values.extend(samples)
            true_values.extend([true_distance] * len(samples))
            target_distances.append(true_distance)

        if not raw_values:
            raise ValueError(f"nessun campione di calibrazione per anchor {anchor_id}")

        raw_array = np.asarray(raw_values, dtype=float)
        true_array = np.asarray(true_values, dtype=float)
        distinct_distances = len({round(value, 4) for value in target_distances})

        if distinct_distances >= 2 and np.ptp(target_distances) >= 0.20:
            design = np.column_stack((raw_array, np.ones(raw_array.size)))
            coefficients, _, rank, _ = np.linalg.lstsq(design, true_array, rcond=None)
            scale, offset_m = (float(value) for value in coefficients)
            model = "scale+offset"
            if rank < 2:
                raise ValueError(
                    f"fit non identificabile per anchor {anchor_id}: "
                    "servono distanze grezze differenti"
                )
            if not MIN_CALIBRATION_SCALE <= scale <= MAX_CALIBRATION_SCALE:
                raise ValueError(
                    f"scala non plausibile per anchor {anchor_id}: {scale:.3f} "
                    f"(intervallo ammesso {MIN_CALIBRATION_SCALE:.1f}-"
                    f"{MAX_CALIBRATION_SCALE:.1f})"
                )
        else:
            scale = 1.0
            offset_m = float(np.mean(true_array - raw_array))
            model = "offset-only"

        corrected = scale * raw_array + offset_m
        fit_rmse_m = float(np.sqrt(np.mean((corrected - true_array) ** 2)))
        calibrations[anchor_id] = DistanceCalibration(scale, offset_m)
        statistics[anchor_id] = {
            "model": model,
            "samples": raw_array.size,
            "fit_rmse_m": fit_rmse_m,
        }

    return calibrations, statistics


def validate_calibration_fit(calibrations, statistics, max_fit_rmse_m):
    """Rifiuta coefficienti o residui incompatibili con una calibrazione sana."""
    for anchor_id in ANCHOR_IDS:
        calibration = calibrations[anchor_id]
        fit_rmse_m = statistics[anchor_id]["fit_rmse_m"]
        if abs(calibration.offset_m) > MAX_ABS_CALIBRATION_OFFSET_M:
            raise ValueError(
                f"offset non plausibile per anchor {anchor_id}: "
                f"{calibration.offset_m:+.3f} m (massimo ammesso "
                f"±{MAX_ABS_CALIBRATION_OFFSET_M:.1f} m)"
            )
        if fit_rmse_m > max_fit_rmse_m:
            raise ValueError(
                f"fit troppo disperso per anchor {anchor_id}: "
                f"RMSE={fit_rmse_m:.3f} m > {max_fit_rmse_m:.3f} m"
            )


class CalibrationCollector:
    def __init__(self, target_position, required_snapshots):
        self.target_position = target_position
        self.required_snapshots = required_snapshots
        self.snapshots = []

    def add(self, snapshot):
        self.snapshots.append(snapshot)
        count = len(self.snapshots)
        if count == 1 or count % 10 == 0 or self.complete:
            print(f"[CALIBRAZIONE] Raccolte {count}/{self.required_snapshots} terne")

    @property
    def complete(self):
        return len(self.snapshots) >= self.required_snapshots


class RobustDistanceFilter:
    """Filtro mediano con rifiuto Hampel degli outlier per una singola anchor."""

    def __init__(self, window, sigma_threshold, floor_m):
        self.values = deque(maxlen=window)
        self.pending_outliers = deque(maxlen=3)
        self.sigma_threshold = sigma_threshold
        self.floor_m = floor_m
        self.outlier_count = 0

    def add(self, value):
        if len(self.values) >= 3:
            history = np.asarray(self.values, dtype=float)
            median = float(np.median(history))
            mad = float(np.median(np.abs(history - median)))
            robust_sigma = 1.4826 * mad
            threshold = max(self.floor_m, self.sigma_threshold * robust_sigma)
            if abs(value - median) > threshold:
                self.pending_outliers.append(value)
                reacquire_span = max(2.0 * self.floor_m, threshold)
                if (
                    len(self.pending_outliers) == self.pending_outliers.maxlen
                    and np.ptp(np.asarray(self.pending_outliers, dtype=float))
                    <= reacquire_span
                ):
                    self.values.clear()
                    self.values.extend(self.pending_outliers)
                    self.pending_outliers.clear()
                    return float(np.median(np.asarray(self.values, dtype=float)))
                self.outlier_count += 1
                return None

        self.pending_outliers.clear()
        self.values.append(value)
        return float(np.median(np.asarray(self.values, dtype=float)))


class PbrBranchTracker:
    """Mantiene continuo il ramo PBR partendo da una posizione iniziale nota.

    Il phase-slope può cambiare di ramo e produrre un salto di vari metri. Il
    tracker mantiene un offset esclusivamente runtime per ogni anchor: non
    modifica né i campioni grezzi né i coefficienti di calibrazione salvati.
    """

    def __init__(self, anchors, start_position, max_step_m):
        start = np.asarray(start_position, dtype=float)
        self.expected_start = {
            anchor_id: float(
                np.linalg.norm(start - np.asarray(anchors[anchor_id], dtype=float))
            )
            for anchor_id in ANCHOR_IDS
        }
        self.max_step_m = max_step_m
        self.offsets = {}
        self.previous = {}
        self.corrections = {anchor_id: 0 for anchor_id in ANCHOR_IDS}

    def add(self, anchor_id, calibrated_distance_m):
        value = float(calibrated_distance_m)
        if anchor_id not in self.offsets:
            target = self.expected_start[anchor_id]
            self.offsets[anchor_id] = target - value
            self.previous[anchor_id] = target
            return target, True

        aligned = value + self.offsets[anchor_id]
        previous = self.previous[anchor_id]
        if abs(aligned - previous) > self.max_step_m:
            # Un nuovo ramo stabile conserva le variazioni successive, ma la
            # sua origine viene raccordata all'ultima distanza affidabile.
            self.offsets[anchor_id] += previous - aligned
            aligned = previous
            self.corrections[anchor_id] += 1
            return aligned, True

        self.previous[anchor_id] = aligned
        return aligned, False


class PositionFilter:
    """Filtro esponenziale con latenza inferiore alla precedente media mobile."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.position = None

    def add(self, position):
        current = np.asarray(position, dtype=float)
        if self.position is None:
            self.position = current
        else:
            self.position = self.alpha * current + (1.0 - self.alpha) * self.position
        return float(self.position[0]), float(self.position[1])


class MeasurementLogger:
    """CSV incrementale contenente sia misure accettate sia terne scartate."""

    FIELDNAMES = [
        "host_time_utc",
        "status",
        "skew_ms",
        "ground_truth_x_m",
        "ground_truth_y_m",
        "error_m",
        "solver_x_m",
        "solver_y_m",
        "filtered_x_m",
        "filtered_y_m",
        "solver_rmse_m",
        "solver_max_residual_m",
    ] + [
        f"{name}{anchor_id}"
        for anchor_id in ANCHOR_IDS
        for name in (
            "sequence_a",
            "method_a",
            "samples_a",
            "raw_distance_a",
            "pbr_valid_a",
            "pbr_distance_a",
            "pbr_samples_a",
            "pbr_rmse_rad_a",
            "rtt_valid_a",
            "rtt_distance_a",
            "rtt_samples_a",
            "rtt_stddev_a",
            "corrected_distance_a",
            "filtered_distance_a",
        )
    ]

    def __init__(self, path, ground_truth=None):
        self.path = Path(path)
        self.ground_truth = ground_truth
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def log(
        self,
        snapshot,
        status,
        skew_ms,
        raw_distances,
        corrected_distances=None,
        filtered_distances=None,
        solution=None,
        filtered_position=None,
    ):
        row = {field: "" for field in self.FIELDNAMES}
        row.update(
            {
                "host_time_utc": datetime.now(timezone.utc).isoformat(),
                "status": status,
                "skew_ms": f"{skew_ms:.3f}",
            }
        )
        if self.ground_truth is not None:
            row["ground_truth_x_m"] = f"{self.ground_truth[0]:.6f}"
            row["ground_truth_y_m"] = f"{self.ground_truth[1]:.6f}"
        if solution is not None:
            row["solver_x_m"] = f"{solution.position[0]:.6f}"
            row["solver_y_m"] = f"{solution.position[1]:.6f}"
            row["solver_rmse_m"] = f"{solution.rmse_m:.6f}"
            row["solver_max_residual_m"] = f"{solution.max_residual_m:.6f}"
        if filtered_position is not None:
            row["filtered_x_m"] = f"{filtered_position[0]:.6f}"
            row["filtered_y_m"] = f"{filtered_position[1]:.6f}"
            if self.ground_truth is not None:
                row["error_m"] = f"{math.dist(filtered_position, self.ground_truth):.6f}"

        for anchor_id in ANCHOR_IDS:
            sample = snapshot[anchor_id]
            row[f"sequence_a{anchor_id}"] = sample.sequence
            row[f"method_a{anchor_id}"] = sample.method
            row[f"samples_a{anchor_id}"] = sample.sample_count
            row[f"raw_distance_a{anchor_id}"] = f"{raw_distances[anchor_id]:.6f}"
            row[f"pbr_valid_a{anchor_id}"] = int(sample.pbr_valid)
            row[f"rtt_valid_a{anchor_id}"] = int(sample.rtt_valid)
            if sample.pbr_valid:
                row[f"pbr_distance_a{anchor_id}"] = (
                    f"{sample.pbr_distance_m:.6f}"
                )
                row[f"pbr_samples_a{anchor_id}"] = sample.pbr_samples
                if sample.pbr_rmse_rad is not None:
                    row[f"pbr_rmse_rad_a{anchor_id}"] = (
                        f"{sample.pbr_rmse_rad:.6f}"
                    )
            if sample.rtt_valid:
                row[f"rtt_distance_a{anchor_id}"] = (
                    f"{sample.rtt_distance_m:.6f}"
                )
                row[f"rtt_samples_a{anchor_id}"] = sample.rtt_samples
                if sample.rtt_stddev_m is not None:
                    row[f"rtt_stddev_a{anchor_id}"] = (
                        f"{sample.rtt_stddev_m:.6f}"
                    )
            if corrected_distances is not None:
                row[f"corrected_distance_a{anchor_id}"] = (
                    f"{corrected_distances[anchor_id]:.6f}"
                )
            if filtered_distances is not None and anchor_id in filtered_distances:
                row[f"filtered_distance_a{anchor_id}"] = (
                    f"{filtered_distances[anchor_id]:.6f}"
                )

        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        self._file.close()


class ValidationCollector:
    def __init__(self, target_position, required_samples, warmup_samples):
        self.target_position = target_position
        self.required_samples = required_samples
        self.warmup_samples = warmup_samples
        self.total_snapshots = 0
        self.accepted_snapshots = 0
        self.solver_positions = []
        self.filtered_positions = []
        self.rejections = {}

    def reject(self, status):
        self.total_snapshots += 1
        self.rejections[status] = self.rejections.get(status, 0) + 1

    def add(self, solver_position, filtered_position):
        self.total_snapshots += 1
        self.accepted_snapshots += 1
        if self.accepted_snapshots <= self.warmup_samples:
            if self.accepted_snapshots == 1:
                print(
                    f"[VALIDAZIONE] Warm-up filtro: "
                    f"{self.warmup_samples} posizioni"
                )
            return

        self.solver_positions.append(solver_position)
        self.filtered_positions.append(filtered_position)
        count = len(self.filtered_positions)
        if count == 1 or count % 10 == 0 or self.complete:
            print(f"[VALIDAZIONE] Raccolte {count}/{self.required_samples} posizioni")

    @property
    def complete(self):
        return len(self.filtered_positions) >= self.required_samples

    @staticmethod
    def _metrics(positions, target_position):
        values = np.asarray(positions, dtype=float)
        target = np.asarray(target_position, dtype=float)
        errors = np.linalg.norm(values - target, axis=1)
        ddof = 1 if len(values) > 1 else 0
        return {
            "mean_position_m": [float(value) for value in np.mean(values, axis=0)],
            "bias_x_m": float(np.mean(values[:, 0]) - target[0]),
            "bias_y_m": float(np.mean(values[:, 1]) - target[1]),
            "std_x_m": float(np.std(values[:, 0], ddof=ddof)),
            "std_y_m": float(np.std(values[:, 1], ddof=ddof)),
            "mean_error_m": float(np.mean(errors)),
            "rmse_position_m": float(np.sqrt(np.mean(errors**2))),
            "median_error_m": float(np.median(errors)),
            "p95_error_m": float(np.percentile(errors, 95)),
            "max_error_m": float(np.max(errors)),
        }

    def summary(self):
        considered = self.total_snapshots
        availability = self.accepted_snapshots / considered if considered else 0.0
        return {
            "target_position_m": list(self.target_position),
            "requested_samples": self.required_samples,
            "warmup_samples": self.warmup_samples,
            "total_snapshots": self.total_snapshots,
            "accepted_snapshots": self.accepted_snapshots,
            "availability": availability,
            "rejections": self.rejections,
            "solver": self._metrics(self.solver_positions, self.target_position),
            "filtered": self._metrics(self.filtered_positions, self.target_position),
        }


class LivePlot:
    def __init__(self, anchors):
        import matplotlib.pyplot as plt

        self.plt = plt
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

        for anchor_id, (x_pos, y_pos) in anchors.items():
            self.ax.scatter(x_pos, y_pos, color="blue", s=150, marker="s")
            self.ax.text(
                x_pos,
                y_pos + 0.15,
                f"Ancora {anchor_id}",
                fontsize=10,
                ha="center",
                fontweight="bold",
            )

        (self.tag_plot,) = self.ax.plot([], [], "r*", markersize=18, label="Tag Mobile")
        x_values = [point[0] for point in anchors.values()]
        y_values = [point[1] for point in anchors.values()]
        self.ax.set_xlim(min(x_values) - 1.0, max(x_values) + 1.0)
        self.ax.set_ylim(min(y_values) - 1.0, max(y_values) + 1.0)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("X (metri)")
        self.ax.set_ylabel("Y (metri)")
        self.ax.set_title("RTLS - BLE Channel Sounding")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")
        self.fig.canvas.draw()

    def update(self, position):
        self.tag_plot.set_data([position[0]], [position[1]])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def process_events(self):
        self.plt.pause(0.001)


def parse_port_spec(spec):
    anchor_text, separator, port_name = spec.partition("=")
    if not separator or not port_name:
        raise argparse.ArgumentTypeError("formato richiesto: ID=PORTA")

    try:
        anchor_id = int(anchor_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("l'ID anchor deve essere un intero") from exc

    if anchor_id not in ANCHOR_IDS:
        raise argparse.ArgumentTypeError(f"ID anchor non valido: {anchor_id}")

    return anchor_id, port_name


def parse_position_spec(spec):
    parts = spec.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("formato richiesto: X,Y")
    try:
        position = tuple(float(value) for value in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("X e Y devono essere numeri") from exc
    if not all(math.isfinite(value) for value in position):
        raise argparse.ArgumentTypeError("X e Y devono essere finiti")
    return position


def resolve_ports(port_specs, parser):
    if not port_specs:
        return DEFAULT_PORTS.copy()

    ports = dict(port_specs)
    if set(ports) != set(ANCHOR_IDS):
        parser.error("specificare esattamente una --port per ciascuna anchor 0, 1 e 2")
    if len(set(ports.values())) != len(ports):
        parser.error("ogni anchor deve usare una porta seriale differente")
    return ports


def _finalize_calibration(args, config, collector):
    dataset = build_calibration_dataset(
        args.calibration_dataset, collector.target_position, collector.snapshots
    )
    calibrations, statistics = fit_distance_calibrations(dataset, config.anchors)
    validate_calibration_fit(
        calibrations, statistics, args.max_calibration_rmse
    )
    calibrated_config = RtlsConfig(config.anchors, calibrations, config.max_rmse_m)
    # Le due scritture avvengono soltanto dopo la validazione completa.
    save_calibration_dataset(args.calibration_dataset, dataset)
    save_rtls_config(args.config, calibrated_config)

    print(f"[CALIBRAZIONE] Dataset aggiornato: {args.calibration_dataset}")
    print(f"[CALIBRAZIONE] Configurazione aggiornata: {args.config}")
    for anchor_id in ANCHOR_IDS:
        calibration = calibrations[anchor_id]
        stats = statistics[anchor_id]
        print(
            f"[CALIBRAZIONE] A{anchor_id}: modello={stats['model']} "
            f"scale={calibration.scale:.6f} offset={calibration.offset_m:+.4f}m "
            f"fit_RMSE={stats['fit_rmse_m']:.4f}m N={stats['samples']}"
        )


def _store_calibration_session(args, collector):
    """Salva un punto della campagna senza applicare un fit ancora parziale."""
    dataset = build_calibration_dataset(
        args.calibration_dataset, collector.target_position, collector.snapshots
    )
    save_calibration_dataset(args.calibration_dataset, dataset)
    print(f"[CALIBRAZIONE] Sessione salvata: {args.calibration_dataset}")
    print(
        f"[CALIBRAZIONE] Punti raccolti nella campagna: "
        f"{len(dataset['sessions'])}"
    )
    print("[CALIBRAZIONE] Configurazione invariata (modalità solo raccolta)")


def default_validation_log_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_MEASUREMENTS_DIR / f"validation_{timestamp}.csv"


def _finalize_validation(args, config, collector):
    summary = collector.summary()
    summary.update(
        {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "csv_log": str(args.csv_log),
            "anchors_m": {
                str(anchor_id): list(config.anchors[anchor_id])
                for anchor_id in ANCHOR_IDS
            },
            "distance_calibration": {
                str(anchor_id): {
                    "scale": config.calibrations[anchor_id].scale,
                    "offset_m": config.calibrations[anchor_id].offset_m,
                }
                for anchor_id in ANCHOR_IDS
            },
            "max_rmse_m": config.max_rmse_m,
            "distance_source": args.distance_source,
            "fusion_guard_m": args.fusion_guard,
            "filter": {
                "distance_window": args.distance_filter_window,
                "outlier_sigma": args.outlier_sigma,
                "outlier_floor_m": args.outlier_floor,
                "position_alpha": args.position_alpha,
            },
        }
    )
    summary_path = Path(args.csv_log).with_suffix(".summary.json")
    temporary_path = summary_path.with_suffix(summary_path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(summary_path)

    filtered = summary["filtered"]
    solver = summary["solver"]
    print(
        f"[RISULTATO] Errore filtrato: medio={filtered['mean_error_m']:.3f}m "
        f"mediano={filtered['median_error_m']:.3f}m "
        f"P95={filtered['p95_error_m']:.3f}m "
        f"massimo={filtered['max_error_m']:.3f}m"
    )
    print(
        f"[RISULTATO] Precisione filtrata: stdX={filtered['std_x_m']:.3f}m "
        f"stdY={filtered['std_y_m']:.3f}m | "
        f"P95 solver non filtrato={solver['p95_error_m']:.3f}m"
    )
    print(
        f"[RISULTATO] Disponibilita={100.0 * summary['availability']:.1f}% "
        f"({summary['accepted_snapshots']}/{summary['total_snapshots']}) "
        f"scarti={summary['rejections']}"
    )
    print(f"[RISULTATO] CSV: {args.csv_log}")
    print(f"[RISULTATO] Riepilogo JSON: {summary_path}")


def run(args, ports, config):
    sample_queue = queue.Queue(maxsize=256)
    stop_event = threading.Event()
    readers = [
        SerialReader(
            anchor_id,
            port_name,
            args.baud,
            sample_queue,
            stop_event,
            args.min_distance,
            args.max_distance,
        )
        for anchor_id, port_name in sorted(ports.items())
    ]
    coordinator = CsTokenCoordinator(
        readers, args.token_timeout, args.token_retry_delay
    )

    collector = None
    if args.calibration_point is not None:
        collector = CalibrationCollector(
            args.calibration_point, args.calibration_snapshots
        )
        print(
            f"[CALIBRAZIONE] Punto noto X={args.calibration_point[0]:.3f}m "
            f"Y={args.calibration_point[1]:.3f}m"
        )

    validation = None
    if args.validation_point is not None:
        validation = ValidationCollector(
            args.validation_point, args.validation_samples, args.validation_warmup
        )
        print(
            f"[VALIDAZIONE] Ground truth X={args.validation_point[0]:.3f}m "
            f"Y={args.validation_point[1]:.3f}m"
        )

    for reader in readers:
        reader.start()

    print(
        "[INFO] Reader seriali avviati; coordinamento CS a token "
        "A0 -> A1 -> A2"
    )
    synchronizer = SampleSynchronizer(config.anchors)
    position_filter = PositionFilter(args.position_alpha)
    distance_filters = {
        anchor_id: RobustDistanceFilter(
            args.distance_filter_window, args.outlier_sigma, args.outlier_floor
        )
        for anchor_id in ANCHOR_IDS
    }
    branch_tracker = None
    if args.pbr_branch_tracking is not None:
        branch_tracker = PbrBranchTracker(
            config.anchors, args.pbr_branch_tracking, args.pbr_branch_max_step
        )
        print(
            "[TRACKING PBR] Continuita attiva; posizione iniziale "
            f"X={args.pbr_branch_tracking[0]:.3f}m "
            f"Y={args.pbr_branch_tracking[1]:.3f}m"
        )
    logger = (
        MeasurementLogger(args.csv_log, args.validation_point)
        if args.csv_log is not None
        else None
    )
    if logger is not None:
        print(f"[INFO] Logging CSV: {logger.path}")
    plot = None if args.no_plot or collector is not None else LivePlot(config.anchors)
    next_status_at = time.monotonic() + args.status_interval

    exit_code = 0
    try:
        while True:
            try:
                event = sample_queue.get(timeout=0.05)
                if isinstance(event, DistanceSample):
                    if coordinator.accept_sample(event, time.monotonic()):
                        selected = select_distance_source(
                            event, args.distance_source, args.fusion_guard
                        )
                        if selected is None:
                            print(
                                f"[QUALITA] A{event.anchor_id}: sorgente "
                                f"{args.distance_source} non disponibile"
                            )
                        elif not args.min_distance <= selected.distance_m <= args.max_distance:
                            print(
                                f"[QUALITA] A{event.anchor_id}: distanza "
                                f"{selected.method} fuori range "
                                f"({selected.distance_m:.3f}m)"
                            )
                        else:
                            synchronizer.add(selected)
                elif isinstance(event, AnchorStatus):
                    coordinator.on_anchor_status(event)
                elif isinstance(event, SerialLinkStatus):
                    coordinator.on_link_status(event)
            except queue.Empty:
                pass

            now = time.monotonic()
            coordinator.tick(now)
            if collector is not None:
                # Durante la calibrazione il tag resta sul punto noto: basta
                # una nuova misura per anchor, anche se il token rende la
                # terna più lenta delle soglie usate durante il tracking.
                snapshot = synchronizer.complete_snapshot()
            else:
                snapshot = synchronizer.coherent_snapshot(
                    now, args.max_age, args.max_skew
                )
            if snapshot is not None:
                if collector is not None:
                    collector.add(snapshot)
                    if collector.complete:
                        try:
                            if args.calibration_collect_only:
                                _store_calibration_session(args, collector)
                            else:
                                _finalize_calibration(args, config, collector)
                        except ValueError as exc:
                            print(f"[ERRORE CALIBRAZIONE] {exc}")
                            print(
                                "[ERRORE CALIBRAZIONE] Dataset e configurazione "
                                "non sono stati modificati"
                            )
                            exit_code = 1
                        except OSError as exc:
                            print(f"[ERRORE CALIBRAZIONE] Salvataggio fallito: {exc}")
                            print(
                                "[ERRORE CALIBRAZIONE] Controllare i file di "
                                "dataset e configurazione prima di riprovare"
                            )
                            exit_code = 1
                        break
                else:
                    raw_distances = {
                        anchor_id: snapshot[anchor_id].distance_m
                        for anchor_id in ANCHOR_IDS
                    }
                    skew_ms = 1000.0 * (
                        max(item.received_monotonic for item in snapshot.values())
                        - min(item.received_monotonic for item in snapshot.values())
                    )
                    corrected_distances = {
                        anchor_id: config.calibrations[anchor_id].apply(
                            raw_distances[anchor_id]
                        )
                        for anchor_id in ANCHOR_IDS
                    }
                    if branch_tracker is not None:
                        for anchor_id in ANCHOR_IDS:
                            tracked, realigned = branch_tracker.add(
                                anchor_id, corrected_distances[anchor_id]
                            )
                            corrected_distances[anchor_id] = tracked
                            if realigned:
                                print(
                                    f"[TRACKING PBR] A{anchor_id}: ramo "
                                    f"riallineato a {tracked:.3f}m"
                                )

                    if any(value <= 0 for value in corrected_distances.values()):
                        print("[WARN] Terna scartata: calibrazione produce distanza non positiva")
                        if validation:
                            validation.reject("nonpositive_distance")
                        if logger:
                            logger.log(
                                snapshot,
                                "nonpositive_distance",
                                skew_ms,
                                raw_distances,
                                corrected_distances,
                            )
                    else:
                        filtered_distances = {}
                        outlier_ids = []
                        for anchor_id in ANCHOR_IDS:
                            filtered_distance = distance_filters[anchor_id].add(
                                corrected_distances[anchor_id]
                            )
                            if filtered_distance is None:
                                outlier_ids.append(anchor_id)
                            else:
                                filtered_distances[anchor_id] = filtered_distance

                        if outlier_ids:
                            anchors_text = ",".join(str(value) for value in outlier_ids)
                            print(
                                f"[FILTRO] Terna scartata: outlier distanza "
                                f"anchor {anchors_text}"
                            )
                            if validation:
                                validation.reject("distance_outlier")
                            if logger:
                                logger.log(
                                    snapshot,
                                    "distance_outlier",
                                    skew_ms,
                                    raw_distances,
                                    corrected_distances,
                                    filtered_distances,
                                )
                        else:
                            solution = solve_position(filtered_distances, config.anchors)
                            if solution is None:
                                print("[WARN] Solver di posizione non convergente")
                                if validation:
                                    validation.reject("solver_failed")
                                if logger:
                                    logger.log(
                                        snapshot,
                                        "solver_failed",
                                        skew_ms,
                                        raw_distances,
                                        corrected_distances,
                                        filtered_distances,
                                    )
                            elif solution.rmse_m > config.max_rmse_m:
                                print(
                                    f"[QUALITA] Terna scartata: "
                                    f"RMSE={solution.rmse_m:.3f}m "
                                    f"> limite={config.max_rmse_m:.3f}m"
                                )
                                if validation:
                                    validation.reject("rmse_rejected")
                                if logger:
                                    logger.log(
                                        snapshot,
                                        "rmse_rejected",
                                        skew_ms,
                                        raw_distances,
                                        corrected_distances,
                                        filtered_distances,
                                        solution,
                                    )
                            else:
                                filtered_position = position_filter.add(solution.position)
                                print(
                                    f"[POSIZIONE] X:{filtered_position[0]:.3f}m "
                                    f"Y:{filtered_position[1]:.3f}m "
                                    f"| Df0:{filtered_distances[0]:.3f} "
                                    f"Df1:{filtered_distances[1]:.3f} "
                                    f"Df2:{filtered_distances[2]:.3f} "
                                    f"| RMSE:{solution.rmse_m:.3f}m "
                                    f"skew:{skew_ms:.0f}ms"
                                )
                                if logger:
                                    logger.log(
                                        snapshot,
                                        "accepted",
                                        skew_ms,
                                        raw_distances,
                                        corrected_distances,
                                        filtered_distances,
                                        solution,
                                        filtered_position,
                                    )
                                if validation:
                                    validation.add(
                                        solution.position, filtered_position
                                    )
                                    if validation.complete:
                                        _finalize_validation(args, config, validation)
                                        break
                                if plot:
                                    plot.update(filtered_position)

            if plot:
                plot.process_events()

            if args.status_interval > 0 and now >= next_status_at:
                print(coordinator.format_status())
                print(format_anchor_status(synchronizer, now))
                print(
                    "[FILTRO] "
                    + " | ".join(
                        f"A{anchor_id}: outlier={distance_filters[anchor_id].outlier_count}"
                        for anchor_id in ANCHOR_IDS
                    )
                )
                next_status_at = now + args.status_interval

    except KeyboardInterrupt:
        print("\n[INFO] Arresto del sistema di localizzazione")
    finally:
        stop_event.set()
        for reader in readers:
            reader.join(timeout=2.0)
        if logger is not None:
            logger.close()
    return exit_code


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Localizzazione e calibrazione BLE Channel Sounding da tre VCOM"
    )
    parser.add_argument(
        "--port",
        action="append",
        type=parse_port_spec,
        metavar="ID=PORTA",
        help="associazione anchor/porta; ripetere per ID 0, 1 e 2",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--max-age", type=float, default=DEFAULT_MAX_AGE_S)
    parser.add_argument("--max-skew", type=float, default=DEFAULT_MAX_SKEW_S)
    parser.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_M)
    parser.add_argument("--max-distance", type=float, default=DEFAULT_MAX_DISTANCE_M)
    parser.add_argument(
        "--distance-source",
        choices=("phase", "rtt", "fused"),
        default="phase",
        help="stima Mode 3 usata per calibrazione/trilaterazione (default: phase)",
    )
    parser.add_argument(
        "--fusion-guard",
        type=float,
        default=0.75,
        help=(
            "massima differenza PBR-RTT prima che la modalità fused usi RTT, "
            "in metri"
        ),
    )
    parser.add_argument("--max-rmse", type=float, help="override soglia RMSE in metri")
    parser.add_argument(
        "--token-timeout",
        type=float,
        default=DEFAULT_TOKEN_TIMEOUT_S,
        help="timeout in secondi per una misura CS comandata",
    )
    parser.add_argument(
        "--token-retry-delay",
        type=float,
        default=DEFAULT_TOKEN_RETRY_DELAY_S,
        help="pausa in secondi tra il rilascio e il token successivo",
    )
    parser.add_argument(
        "--status-interval",
        type=float,
        default=DEFAULT_STATUS_INTERVAL_S,
        help="secondi tra i messaggi di stato; 0 disabilita lo stato periodico",
    )
    parser.add_argument(
        "--calibration-point",
        type=parse_position_spec,
        metavar="X,Y",
        help="raccoglie campioni con il tag fermo nella posizione nota X,Y",
    )
    parser.add_argument(
        "--calibration-snapshots",
        type=int,
        default=DEFAULT_CALIBRATION_SNAPSHOTS,
        help="numero di terne coerenti da raccogliere nel punto noto",
    )
    parser.add_argument(
        "--calibration-dataset",
        type=Path,
        default=DEFAULT_CALIBRATION_DATASET_PATH,
    )
    parser.add_argument(
        "--calibration-collect-only",
        action="store_true",
        help=(
            "salva il punto nel dataset senza calcolare il fit né modificare "
            "la configurazione; utile per i punti intermedi di una campagna"
        ),
    )
    parser.add_argument(
        "--max-calibration-rmse",
        type=float,
        default=DEFAULT_MAX_CALIBRATION_RMSE_M,
        help="RMSE massimo ammesso per il fit delle distanze, in metri",
    )
    parser.add_argument(
        "--csv-log",
        type=Path,
        help="salva ogni terna accettata o scartata nel CSV indicato",
    )
    parser.add_argument(
        "--validation-point",
        type=parse_position_spec,
        metavar="X,Y",
        help="valuta automaticamente la posizione rispetto al punto noto X,Y",
    )
    parser.add_argument(
        "--validation-samples",
        type=int,
        default=DEFAULT_VALIDATION_SAMPLES,
        help="numero di posizioni valide da usare per le metriche",
    )
    parser.add_argument(
        "--validation-warmup",
        type=int,
        default=DEFAULT_VALIDATION_WARMUP,
        help="posizioni valide iniziali escluse dalle metriche",
    )
    parser.add_argument(
        "--distance-filter-window",
        type=int,
        default=DEFAULT_DISTANCE_FILTER_WINDOW,
        help="finestra mediana dispari; 1 disabilita la memoria del filtro",
    )
    parser.add_argument(
        "--outlier-sigma",
        type=float,
        default=DEFAULT_OUTLIER_SIGMA,
        help="soglia Hampel espressa in deviazioni robuste",
    )
    parser.add_argument(
        "--outlier-floor",
        type=float,
        default=DEFAULT_OUTLIER_FLOOR_M,
        help="variazione minima ammessa dal filtro distanza, in metri",
    )
    parser.add_argument(
        "--position-alpha",
        type=float,
        default=DEFAULT_POSITION_ALPHA,
        help="peso 0-1 della nuova posizione nel filtro esponenziale",
    )
    parser.add_argument(
        "--pbr-branch-tracking",
        type=parse_position_spec,
        metavar="X,Y",
        help=(
            "abilita la continuita PBR partendo dalla posizione nota X,Y; "
            "solo per tracking/demo, non per calibrazione o validazione"
        ),
    )
    parser.add_argument(
        "--pbr-branch-max-step",
        type=float,
        default=DEFAULT_PBR_BRANCH_MAX_STEP_M,
        help="massimo salto PBR per ciclo prima del riallineamento del ramo",
    )
    parser.add_argument("--no-plot", action="store_true")
    return parser


def main():
    parser = build_argument_parser()
    args = parser.parse_args()
    if args.max_age <= 0 or args.max_skew < 0:
        parser.error("--max-age deve essere positivo e --max-skew non negativo")
    if args.min_distance < 0 or args.max_distance <= args.min_distance:
        parser.error("intervallo delle distanze non valido")
    if args.fusion_guard <= 0:
        parser.error("--fusion-guard deve essere positivo")
    if args.status_interval < 0:
        parser.error("--status-interval non può essere negativo")
    if args.max_rmse is not None and args.max_rmse <= 0:
        parser.error("--max-rmse deve essere positivo")
    if args.token_timeout <= 0 or args.token_retry_delay < 0:
        parser.error("parametri del coordinatore token non validi")
    if args.calibration_snapshots <= 0:
        parser.error("--calibration-snapshots deve essere positivo")
    if args.max_calibration_rmse <= 0:
        parser.error("--max-calibration-rmse deve essere positivo")
    if args.calibration_point is not None and args.validation_point is not None:
        parser.error("--calibration-point e --validation-point sono mutuamente esclusivi")
    if args.calibration_collect_only and args.calibration_point is None:
        parser.error("--calibration-collect-only richiede --calibration-point")
    if args.validation_samples <= 0:
        parser.error("--validation-samples deve essere positivo")
    if args.validation_warmup < 0:
        parser.error("--validation-warmup non può essere negativo")
    if (
        args.distance_filter_window <= 0
        or args.distance_filter_window % 2 == 0
    ):
        parser.error("--distance-filter-window deve essere positivo e dispari")
    if args.outlier_sigma <= 0 or args.outlier_floor < 0:
        parser.error("parametri del filtro outlier non validi")
    if not 0 < args.position_alpha <= 1:
        parser.error("--position-alpha deve appartenere all'intervallo (0, 1]")
    if args.pbr_branch_max_step <= 0:
        parser.error("--pbr-branch-max-step deve essere positivo")
    if args.pbr_branch_tracking is not None:
        if args.distance_source != "phase":
            parser.error("--pbr-branch-tracking richiede --distance-source phase")
        if args.calibration_point is not None or args.validation_point is not None:
            parser.error(
                "--pbr-branch-tracking e riservato al tracking: non usarlo "
                "durante calibrazione o validazione"
            )
    if args.validation_point is not None and args.csv_log is None:
        args.csv_log = default_validation_log_path()

    try:
        config = load_rtls_config(args.config)
    except ValueError as exc:
        parser.error(str(exc))

    if args.max_rmse is not None:
        config = RtlsConfig(config.anchors, config.calibrations, args.max_rmse)

    ports = resolve_ports(args.port, parser)
    print(f"[INFO] Configurazione RTLS: {args.config}")
    for anchor_id in ANCHOR_IDS:
        calibration = config.calibrations[anchor_id]
        print(
            f"[INFO] A{anchor_id} coordinate={config.anchors[anchor_id]} "
            f"scale={calibration.scale:.6f} offset={calibration.offset_m:+.4f}m"
        )
    raise SystemExit(run(args, ports, config))


if __name__ == "__main__":
    main()
