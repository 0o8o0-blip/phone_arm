"""BrowserPhone teleoperator: WebXR pose over WebTransport.

The operator's Android phone connects to an HTTPS server hosted on this Pi
(see ./browser/). WebXR streams 6DoF pose at ~30 Hz; the browser ships each
pose as an unreliable WebTransport datagram to a session relay, which
forwards to this Pi where the arm follows the pose. Robot video comes back
via browser-native WHEP through MediaMTX/coturn TURN. This class exposes
the pose stream as the local phone action schema consumed by `follower.main`.

------------------------------------------------------------------------
Process architecture
------------------------------------------------------------------------

`follower.main` holds the motor bus + runs the 60 Hz IK loop. To keep
the loop free of GC pauses and I/O jitter, two subprocesses are spawned during
BrowserPhone.connect():

  1. BrowserPhone HTTPS server subprocess (_browser_phone_process_main)
     - serves the operator page on :8443, handles /webrtc/config and /test_event
     - is the arm-side WebTransport client -- pose datagrams flow relay
       -> here -> _ingest_pose_msg -> IPC to main
     - owns SessionRecorder for pose.csv + debug.csv + events.jsonl

  2. Trajectory writer subprocess (_trajectory_writer_process_main)
     - takes IK-tick trajectory rows over an mp.Queue, appends to
       trajectory.csv. Isolated because trajectory rows come in at 60 Hz
       and the CSV I/O would otherwise stall the IK loop under disk pressure.

_RecorderProxy wires the IPC between them so pose.csv, debug.csv, events.jsonl,
and trajectory.csv land under the same session directory. `follower/run.sh`
supplies that directory.

Each subprocess is started with `mp.get_context("spawn")` which re-imports
this module (~6 s on cold cache). PHONE_ARM_SUBPROC_STARTUP_S (default 30 s)
gives the parent enough headroom to wait; teleop won't hard-fail on a slow
first import.

------------------------------------------------------------------------
Control transport
------------------------------------------------------------------------

Session-relay WebTransport is the only supported control transport. The
arm-side client is _relay_wt_arm_loop; the phone side is app.js's
startControlWebTransport. Datagrams are latest-only (chrome
outgoingHighWaterMark=1 + outgoingMaxAge=75ms; server LatestSlot).

Older transports (direct WebRTC DataChannel, session-relay WebSocket,
session-relay WebRTC PeerConnection) have been removed. See git history if
you need them back.
"""
from __future__ import annotations

import asyncio
import atexit
import csv
import gc as _gc
import io
import json
import os as _os
import queue as _queue
import signal
import shlex
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.parse as _urlparse
from dataclasses import dataclass
from functools import wraps
from pathlib import Path

import multiprocessing as _mp
import threading as _threading
from datetime import datetime as _datetime, timezone as _timezone

import numpy as np
from aiohttp import web
from scipy.spatial.transform import Rotation

from follower.hardware import ROBOT_ADAPTER_SERIAL
from shared.leader_protocol import LEADER_MESSAGE_TYPE, parse_leader_positions


# Per-session recorder. The browser/control server owns pose/events and the
# robot process writes trajectory rows through _RecorderProxy.
_ACTIVE_RECORDER: "SessionRecorder | None" = None


_PHONE_ARM_GC_PAUSE_MS = 50.0


def _record_session_event(event_type: str, details: dict) -> None:
    rec = _ACTIVE_RECORDER
    record_event = getattr(rec, "record_event", None)
    if callable(record_event):
        record_event(event_type, details)


_GC_PROBE_START_S: float | None = None


def _gc_pause_probe(phase: str, info: dict) -> None:
    global _GC_PROBE_START_S
    if phase == "start":
        _GC_PROBE_START_S = time.perf_counter()
        return
    if phase != "stop" or _GC_PROBE_START_S is None:
        return
    pause_ms = (time.perf_counter() - _GC_PROBE_START_S) * 1000.0
    _GC_PROBE_START_S = None
    if pause_ms > _PHONE_ARM_GC_PAUSE_MS:
        _record_session_event("gc_pause", {
            "pause_ms": round(pause_ms, 1),
            "generation": info.get("generation"),
            "collected": info.get("collected"),
            "uncollectable": info.get("uncollectable"),
        })


if _gc_pause_probe not in _gc.callbacks:
    _gc.callbacks.append(_gc_pause_probe)


_RECORDINGS_DIR = Path(__file__).resolve().parent / "teleop_recordings"
_FIXED_RECORDING_DIR_ENV = "PHONE_ARM_RECORDING_DIR"


def _fixed_recording_dir() -> Path | None:
    raw = _os.environ.get(_FIXED_RECORDING_DIR_ENV, "").strip()
    if not raw:
        return None
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path


class SessionRecorder:
    """Per-session recorder. Writes:
      <session-dir>/
        pose.csv          -- one row per accepted B1-held control pose
        debug.csv         -- browser/control diagnostics keyed by last pose seq
        events.jsonl      -- B1 transitions, faults, re-cals

    follower/run.sh sets <session-dir> once per run. Direct follower.main runs fall back to
    teleop_recordings/<token-name>/<utc-ts>/.

    Methods are called from multiple threads. Callers only enqueue bounded work;
    one writer thread owns file IO so recording cannot block control timing.
    """

    _QUEUE_MAX = 4096

    def __init__(self, token_name: str) -> None:
        fixed_dir = _fixed_recording_dir()
        if fixed_dir is not None:
            self.session_dir = fixed_dir
        else:
            # Sanitize token name to a safe directory component.
            safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in token_name) or "unknown"
            ts = _datetime.now(_timezone.utc).strftime("%Y%m%d_%H%M%S")
            self.session_dir = _RECORDINGS_DIR / safe / ts
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._state_lock = _threading.Lock()
        self._pose_csv = (self.session_dir / "pose.csv").open("w", buffering=1)
        self._pose_csv_header_written = False
        self._pose_columns: tuple[str, ...] | None = None
        self._debug_csv = None
        self._debug_csv_header_written = False
        self._debug_columns: tuple[str, ...] | None = None
        self._events_jsonl = (self.session_dir / "events.jsonl").open("w", buffering=1)
        # trajectory.csv: per-IK-tick desired_ee + q_meas + q_goal. Opened
        # lazily on first record_trajectory call so a session that never
        # touches the arm doesn't leave a stub file. Join to pose.csv via
        # nearest t_pi_ms; IK runs at 60Hz vs pose ~30Hz so multiple
        # trajectory rows can map to one pose.
        self._trajectory_csv = None
        self._trajectory_csv_header_written = False
        self._trajectory_columns: tuple[str, ...] | None = None
        self._closed = False
        self._close_reported = False
        self._drop_counts: dict[str, int] = {}
        self._t_start_ms = time.time() * 1000.0
        self._queue: _queue.Queue = _queue.Queue(maxsize=self._QUEUE_MAX)
        self._writer_thread = _threading.Thread(
            target=self._writer_loop,
            name="session-recorder-writer",
            daemon=True,
        )
        self._writer_thread.start()
        # Belt-and-suspenders: if the process exits without going through the
        # normal _shutdown() path, atexit still closes the CSV/JSON handles.
        # close() is idempotent via self._closed.
        atexit.register(self.close)
        print(f"[recorder] session opened at {self.session_dir}")

    def record_pose(self, row: dict) -> None:
        """One row per accepted pose. Caller decides which rows to log (e.g.
        B1-held only); the recorder just writes whatever it's given."""
        self._enqueue("pose", dict(row))

    def record_debug(self, row: dict) -> None:
        """One row per best-effort browser debug datagram."""
        self._enqueue("debug", dict(row))

    def record_trajectory(self, row: dict) -> None:
        """One row per IK tick while robot motion is being commanded or settling.
        Called from follower.main's _TrajectoryLogger. Caller decides the
        key set; the recorder writes whatever it's given. Schema is opened
        lazily from the first row's keys."""
        self._enqueue("trajectory", dict(row))

    def record_event(self, event_type: str, details: dict | None = None) -> None:
        rec = {
            "t_utc": _datetime.now(_timezone.utc).isoformat(timespec="milliseconds"),
            "t_session_s": (time.time() * 1000.0 - self._t_start_ms) / 1000.0,
            "event": event_type,
        }
        if details:
            rec["details"] = dict(details)
        self._enqueue("event", rec)

    def _enqueue(self, kind: str, payload) -> None:
        with self._state_lock:
            if self._closed:
                return
        try:
            self._queue.put_nowait((kind, payload))
        except _queue.Full:
            first_drop = False
            with self._state_lock:
                prev = self._drop_counts.get(kind, 0)
                self._drop_counts[kind] = prev + 1
                first_drop = prev == 0
            if first_drop:
                print(f"[recorder] queue full; dropping {kind} records")

    def _writer_loop(self) -> None:
        try:
            while True:
                try:
                    kind, payload = self._queue.get(timeout=0.5)
                except _queue.Empty:
                    with self._state_lock:
                        if self._closed:
                            break
                    continue
                try:
                    if kind == "close":
                        break
                    if kind == "pose":
                        self._write_pose(payload)
                    elif kind == "debug":
                        self._write_debug(payload)
                    elif kind == "trajectory":
                        self._write_trajectory(payload)
                    elif kind == "event":
                        self._write_event(payload)
                    else:
                        print(f"[recorder] unknown queue item kind={kind!r}")
                except Exception as e:  # noqa: BLE001
                    print(f"[recorder] {kind} write failed: {e}")
                finally:
                    try:
                        self._queue.task_done()
                    except ValueError:
                        pass
        finally:
            self._finalize_outputs()

    def _write_pose(self, row: dict) -> None:
        if not self._pose_csv_header_written:
            self._pose_columns = tuple(row.keys())
            self._pose_csv.write(",".join(self._pose_columns) + "\n")
            self._pose_csv_header_written = True
        assert self._pose_columns is not None
        self._pose_csv.write(self._csv_row(row, self._pose_columns))

    def _write_debug(self, row: dict) -> None:
        if self._debug_csv is None:
            self._debug_csv = (self.session_dir / "debug.csv").open("w", buffering=1)
        if not self._debug_csv_header_written:
            self._debug_columns = tuple(row.keys())
            self._debug_csv.write(",".join(self._debug_columns) + "\n")
            self._debug_csv_header_written = True
        assert self._debug_columns is not None
        self._debug_csv.write(self._csv_row(row, self._debug_columns))

    def _write_trajectory(self, row: dict) -> None:
        if self._trajectory_csv is None:
            self._trajectory_csv = (
                self.session_dir / "trajectory.csv"
            ).open("w", buffering=1)
        if not self._trajectory_csv_header_written:
            self._trajectory_columns = tuple(row.keys())
            self._trajectory_csv.write(",".join(self._trajectory_columns) + "\n")
            self._trajectory_csv_header_written = True
        assert self._trajectory_columns is not None
        self._trajectory_csv.write(self._csv_row(row, self._trajectory_columns))

    def _write_event(self, rec: dict) -> None:
        self._events_jsonl.write(json.dumps(rec) + "\n")

    @staticmethod
    def _csv_row(row: dict, columns: tuple[str, ...]) -> str:
        out = io.StringIO()
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(
            "" if (value := row.get(key)) is None else value
            for key in columns
        )
        return out.getvalue()

    def _finalize_outputs(self) -> None:
        for fh in (self._pose_csv, self._debug_csv, self._events_jsonl, self._trajectory_csv):
            if fh is None:
                continue
            try:
                fh.close()
            except Exception:
                pass

    def close(self) -> None:
        should_signal = False
        with self._state_lock:
            if not self._closed:
                self._closed = True
                should_signal = True
        if should_signal:
            deadline = time.time() + float(_os.environ.get("PHONE_ARM_SUBPROC_STARTUP_S", "30"))
            try:
                while True:
                    try:
                        self._queue.put(("close", None), timeout=0.5)
                        break
                    except _queue.Full:
                        if (not self._writer_thread.is_alive()
                                or time.time() >= deadline):
                            print("[recorder] close could not enqueue sentinel")
                            break
            except Exception as e:  # noqa: BLE001
                print(f"[recorder] close signal failed: {e}")
        if (_threading.current_thread() is not self._writer_thread
                and self._writer_thread.is_alive()):
            self._writer_thread.join(timeout=10.0)
        if self._writer_thread.is_alive():
            print(f"[recorder] writer did not stop cleanly: {self.session_dir}")
            return
        with self._state_lock:
            if self._close_reported:
                return
            self._close_reported = True
            drops = dict(self._drop_counts)
        drop_note = ""
        if drops:
            drop_note = "; dropped=" + ",".join(
                f"{kind}:{count}" for kind, count in sorted(drops.items())
            )
        print(f"[recorder] session closed: {self.session_dir} "
              f"(pose/debug/events{drop_note})")


class _TrajectorySessionRecorder:
    """Dedicated-process writer for trajectory.csv.

    This owns only trajectory.csv.  Pose/debug/events stay with SessionRecorder and
    video stays with the video worker, so a large trajectory file cannot fill
    the browser/control server's recorder queue.
    """

    def __init__(self, session_dir: str | Path) -> None:
        self.session_dir = Path(session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._trajectory_csv = (self.session_dir / "trajectory.csv").open(
            "w", buffering=1
        )
        self._trajectory_columns: tuple[str, ...] | None = None
        self._rows = 0
        print(f"[trajectory-recorder] trajectory recording at {self.session_dir}")

    def record_trajectory(self, row: dict) -> None:
        if self._trajectory_columns is None:
            self._trajectory_columns = tuple(row.keys())
            self._trajectory_csv.write(",".join(self._trajectory_columns) + "\n")
        self._trajectory_csv.write(
            SessionRecorder._csv_row(row, self._trajectory_columns)
        )
        self._rows += 1

    def close(self) -> None:
        try:
            self._trajectory_csv.close()
        except Exception:
            pass
        print(
            f"[trajectory-recorder] session closed: {self.session_dir} "
            f"({self._rows} trajectory rows)"
        )


class _RecorderProxy:
    """Parent-process stand-in for SessionRecorder.

    follower.main records trajectory rows through gateway._ACTIVE_RECORDER.
    After the browser server moves to a child process, pose/events live there
    and trajectory.csv lives in a dedicated writer process.  This proxy sends
    trajectory rows directly to that writer so the browser/control server never
    competes with high-volume CSV output.
    """

    def __init__(self, command_queue, trajectory_queue=None) -> None:
        self._command_queue = command_queue
        self._trajectory_queue = trajectory_queue
        self._drop_counts: dict[str, int] = {}
        self._trajectory_lock = _threading.Lock()
        self._trajectory_pending: list[dict] = []
        self._trajectory_closed = False
        self._trajectory_wakeup = _threading.Event()
        self._trajectory_batch_size = self._env_int(
            "PHONE_ARM_TRAJECTORY_BATCH_SIZE", 256, minimum=1
        )
        self._trajectory_batch_interval_s = self._env_float(
            "PHONE_ARM_TRAJECTORY_BATCH_INTERVAL_S", 0.05, minimum=0.001
        )
        self._trajectory_thread: _threading.Thread | None = None
        if self._trajectory_queue is not None:
            self._trajectory_thread = _threading.Thread(
                target=self._trajectory_feeder_loop,
                name="TrajectoryRecorderProxy",
                daemon=True,
            )
            self._trajectory_thread.start()
            atexit.register(self.close)

    @staticmethod
    def _env_int(name: str, default: int, *, minimum: int) -> int:
        raw = _os.environ.get(name)
        if raw is None:
            return default
        try:
            return max(minimum, int(raw))
        except (TypeError, ValueError):
            print(f"[recorder-proxy] invalid {name}={raw!r}; using {default}")
            return default

    @staticmethod
    def _env_float(name: str, default: float, *, minimum: float) -> float:
        raw = _os.environ.get(name)
        if raw is None:
            return default
        try:
            return max(minimum, float(raw))
        except (TypeError, ValueError):
            print(f"[recorder-proxy] invalid {name}={raw!r}; using {default}")
            return default

    def _send(self, kind: str, payload) -> None:
        if self._command_queue is None:
            return
        try:
            self._command_queue.put_nowait((kind, payload))
        except (_queue.Full, BrokenPipeError, EOFError, OSError):
            prev = self._drop_counts.get(kind, 0)
            self._drop_counts[kind] = prev + 1
            if prev == 0:
                print(f"[recorder-proxy] command queue full/broken; dropping {kind}")

    def record_trajectory(self, row: dict) -> None:
        payload = dict(row)
        q = self._trajectory_queue
        if q is None:
            self._send("trajectory", payload)
            return
        with self._trajectory_lock:
            if self._trajectory_closed:
                return
            self._trajectory_pending.append(payload)
            should_wake = len(self._trajectory_pending) >= self._trajectory_batch_size
        if should_wake:
            self._trajectory_wakeup.set()

    def _take_trajectory_batch(self) -> list[dict]:
        with self._trajectory_lock:
            if not self._trajectory_pending:
                return []
            batch = self._trajectory_pending
            self._trajectory_pending = []
            return batch

    def _send_trajectory_batch(self, batch: list[dict]) -> None:
        if not batch:
            return
        q = self._trajectory_queue
        if q is None:
            for row in batch:
                self._send("trajectory", row)
            return
        msg = ("trajectory", batch[0]) if len(batch) == 1 else ("trajectory_batch", batch)
        try:
            q.put_nowait(msg)
        except _queue.Full:
            # PHONE_ARM_TRAJECTORY_QUEUE_MAX can make this finite. If it fills,
            # preserve the row rather than silently losing training data.
            print("[recorder-proxy] trajectory queue full; feeder blocking to preserve rows")
            q.put(msg)
        except (BrokenPipeError, EOFError, OSError):
            prev = self._drop_counts.get("trajectory", 0)
            self._drop_counts["trajectory"] = prev + 1
            if prev == 0:
                print("[recorder-proxy] trajectory writer unavailable; dropping trajectory")

    def _trajectory_feeder_loop(self) -> None:
        while True:
            self._trajectory_wakeup.wait(self._trajectory_batch_interval_s)
            self._trajectory_wakeup.clear()
            batch = self._take_trajectory_batch()
            if batch:
                self._send_trajectory_batch(batch)
            with self._trajectory_lock:
                if self._trajectory_closed and not self._trajectory_pending:
                    return

    def close(self) -> None:
        if self._trajectory_queue is None:
            return
        with self._trajectory_lock:
            if self._trajectory_closed:
                return
            self._trajectory_closed = True
        self._trajectory_wakeup.set()
        if (self._trajectory_thread is not None
                and self._trajectory_thread is not _threading.current_thread()):
            self._trajectory_thread.join(timeout=10.0)
            if self._trajectory_thread.is_alive():
                print("[recorder-proxy] trajectory feeder did not drain cleanly")

    def record_event(self, event_type: str, details: dict | None = None) -> None:
        self._send("event", (event_type, dict(details or {})))


def _trajectory_writer_process_main(command_queue, status_queue) -> None:
    """Process entry point for high-volume trajectory.csv output."""
    recorder: _TrajectorySessionRecorder | None = None
    pending: list[dict] = []
    try:
        status_queue.put_nowait(("ready", None))
    except Exception:
        pass
    try:
        while True:
            try:
                item = command_queue.get()
            except (EOFError, OSError):
                break
            if not item:
                continue
            kind = item[0]
            payload = item[1] if len(item) > 1 else None
            if kind == "open":
                if recorder is None:
                    recorder = _TrajectorySessionRecorder(payload)
                    if pending:
                        print(
                            "[trajectory-recorder] flushing "
                            f"{len(pending)} rows queued before session open"
                        )
                        for row in pending:
                            recorder.record_trajectory(row)
                        pending.clear()
                elif str(recorder.session_dir) != str(payload):
                    recorder.close()
                    recorder = _TrajectorySessionRecorder(payload)
            elif kind == "trajectory":
                if recorder is None:
                    pending.append(dict(payload))
                else:
                    recorder.record_trajectory(payload)
            elif kind == "trajectory_batch":
                batch = [dict(row) for row in (payload or [])]
                if recorder is None:
                    pending.extend(batch)
                else:
                    for row in batch:
                        recorder.record_trajectory(row)
            elif kind == "shutdown":
                break
    except BaseException as e:  # noqa: BLE001
        try:
            status_queue.put_nowait(("error", repr(e)))
        except Exception:
            pass
        raise
    finally:
        if recorder is None and pending:
            ts = _datetime.now(_timezone.utc).strftime("%Y%m%d_%H%M%S")
            recorder = _TrajectorySessionRecorder(
                _RECORDINGS_DIR / "trajectory_orphan" / ts
            )
            print(
                "[trajectory-recorder] no session opened; writing "
                f"{len(pending)} pending rows to orphan session"
            )
            for row in pending:
                recorder.record_trajectory(row)
            pending.clear()
        if recorder is not None:
            recorder.close()
        try:
            status_queue.put_nowait(("closed", None))
        except Exception:
            pass


@dataclass
class PhoneConfig:
    camera_offset = np.array([0.0, -0.02, 0.04])


def check_if_already_connected(fn):
    @wraps(fn)
    def wrapper(self, *args, **kwargs):
        if self.is_connected:
            raise RuntimeError(f"{self.__class__.__name__} is already connected")
        return fn(self, *args, **kwargs)

    return wrapper

REPO_ROOT = Path(__file__).resolve().parents[1]
CERTS_DIR = Path(__file__).resolve().parent / "certs"
STATIC_DIR = REPO_ROOT / "controllers" / "phone"

DEFAULT_PORT = 8443

# Pinned by by-PATH (physical USB port), NOT by-id: the generic Realtek 0bda:5844
# cams all report the SAME fake serial (200901010001), so their by-id symlink
# collides when two are plugged and flips unpredictably. by-path is unambiguous
# and stable -- as long as each cam stays in its current port. The shell video
# publisher imports VIDEO_DEVICE below to pick the default ffmpeg input.
_REALFLEX_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1.4:1.0-video-index0"   # port 1.4
_CAM2_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1.2:1.0-video-index0"     # port 1.1.2 (added 2026-05-28)
_CAM3_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1.4:1.0-video-index0"     # port 1.1.4 (added 2026-05-29)
_C920_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0"       # port 1.3

_CAMERA_REGISTRY: list[tuple[str, str, str]] = [
    ("realflex", "RealFlex", _REALFLEX_DEVICE),
    ("cam2", "Cam 2", _CAM2_DEVICE),
    ("cam3", "Cam 3", _CAM3_DEVICE),
    ("c920", "C920", _C920_DEVICE),
]


def _present_cameras() -> list[tuple[str, str, str]]:
    return [c for c in _CAMERA_REGISTRY if Path(c[2]).exists()]


def _default_video_device() -> str:
    present = _present_cameras()
    return present[0][2] if present else _REALFLEX_DEVICE


# Robot camera video. This is deliberately the robot camera stream shown
# over the WebXR AR passthrough so the phone camera is not visible to the user.
VIDEO_DEVICE = _default_video_device()
# Browser WHEP media is RELAY-ONLY: it always flows through the VPS TURN relay,
# never a direct LAN/host path or a public srflx path. TURN is therefore
# mandatory for remote video. follower/run.sh supplies these from ~/.turn_secret.
TURN_URL = _os.environ.get("PHONE_ARM_TURN_URL", "")
TURN_USER = _os.environ.get("PHONE_ARM_TURN_USER", "")
TURN_PW = _os.environ.get("PHONE_ARM_TURN_PW", "")
# Phone-facing TURN URL: when the operator should hit a DIFFERENT coturn than
# the Pi (two-edge video routing -- e.g. operator hits SGP coturn near them
# while Pi uses London coturn over LAN), set this. Unset -> phone uses
# TURN_URL too (single-coturn topology).
TURN_URL_PHONE = _os.environ.get("PHONE_ARM_TURN_URL_PHONE", "") or TURN_URL
ALLOW_NO_TURN_FOR_LOCAL_REPRO = (
    _os.environ.get("PHONE_ARM_ALLOW_NO_TURN_FOR_LOCAL_REPRO", "").strip() == "1"
)
TURN_URLS_PHONE = [TURN_URL_PHONE] if TURN_URL_PHONE else []

# Public URL the phone connects to. Caddy on the VPS terminates TLS for
# 188-166-154-201.sslip.io with a Let's Encrypt cert, then reverse-proxies to
# the bore tunnel (localhost:8443 on the VPS) which forwards to this process.
# VPS_HOST is the IP the bore tunnel terminates against; it stays in the Pi-side
# cert SAN but is no longer seen by phones (Caddy proxies with tls_insecure_skip_verify).
VPS_PUBLIC_URL = "https://188-166-154-201.sslip.io"
VPS_HOST = "188.166.154.201"

# Session-relay WebTransport control transport.
# The arm connects OUTBOUND as an "arm" peer to SESSION_RELAY_WT_URL; the
# operator's browser connects as a "phone" peer to SESSION_RELAY_WT_URL_PHONE
# (which may point to a different edge for geo-optimized routing). All pose
# datagrams flow phone -> forwarder(s) -> relay -> arm. Video is separate WHEP
# through MediaMTX/TURN.
SESSION_RELAY_ARM_TOKEN = _os.environ.get("PHONE_ARM_SESSION_RELAY_ARM_TOKEN", "").strip()
SESSION_RELAY_PHONE_TOKEN = _os.environ.get("PHONE_ARM_SESSION_RELAY_PHONE_TOKEN", "").strip()
SESSION_RELAY_SESSION = _os.environ.get("PHONE_ARM_SESSION_RELAY_SESSION", "default").strip() or "default"
SESSION_RELAY_WT_URL = _os.environ.get("PHONE_ARM_SESSION_RELAY_WT_URL", "").strip()
# Phone-facing override: when Pi and operator should hit DIFFERENT WT
# endpoints (Pi connects to a London-local relay for low latency, operator
# connects to a Singapore forwarder near them). Empty -> phone uses
# SESSION_RELAY_WT_URL too (single-relay topology).
SESSION_RELAY_WT_URL_PHONE = _os.environ.get("PHONE_ARM_SESSION_RELAY_WT_URL_PHONE", "").strip()

# MediaMTX SFU for robot video. /webrtc/config advertises this WHEP endpoint
# and subscriber token; app.js uses it as the only robot-video path.
MEDIAMTX_WHEP_URL = _os.environ.get(
    "PHONE_ARM_MEDIAMTX_WHEP_URL", "").strip()
MEDIAMTX_PLAY_TOKEN = _os.environ.get(
    "PHONE_ARM_MEDIAMTX_PLAY_TOKEN", "").strip()


def _session_relay_wt_url(role: str, *, base_override: str | None = None) -> str:
    # Phone role can override to a separate edge URL when a forwarder is in
    # use; arm always uses the canonical SESSION_RELAY_WT_URL so the Pi can
    # connect directly to the real relay.
    base = base_override or (
        SESSION_RELAY_WT_URL_PHONE
        if role == "phone" and SESSION_RELAY_WT_URL_PHONE
        else SESSION_RELAY_WT_URL
    )
    if not base:
        return ""
    if base.startswith("wss://"):
        base = "https://" + base[len("wss://"):]
    base = base.rstrip("/")
    if base.endswith("/phone") or base.endswith("/arm"):
        base = base.rsplit("/", 1)[0]
    elif not base.endswith("/wt"):
        base = base + "/wt"
    qs = {"session": SESSION_RELAY_SESSION}
    token = SESSION_RELAY_ARM_TOKEN if role == "arm" else SESSION_RELAY_PHONE_TOKEN
    if token:
        qs["token"] = token
    return f"{base}/{role}?{_urlparse.urlencode(qs)}"


def _allow_system_dist_packages_for_aioquic() -> None:
    """Allow apt-installed aioquic when the lerobot venv lacks it.

    The arm process runs from a venv, but on the Pi the pragmatic deployment for
    this prototype is `python3-aioquic`. Python versions match on this image, so
    adding the distro dist-packages path is enough and keeps the venv otherwise
    untouched.
    """
    candidates = [
        "/usr/lib/python3/dist-packages",
        f"/usr/local/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages",
        f"/usr/lib/python{sys.version_info.major}/dist-packages",
    ]
    for path in candidates:
        if path not in sys.path and Path(path).exists():
            sys.path.append(path)

# Token-based auth. If this file exists, every request must include a valid
# ?t=TOKEN query param (except /healthz). If it does NOT exist, the server runs
# unauthenticated -- backwards-compatible default for first runs. Tokens are
# managed via follower/mint_token.py; the server reloads on SIGHUP so revocations are
# instant without a teleop restart. File format: a JSON array of
#   {"value": str, "name": str, "expires_at": float|null}
# where expires_at is a unix timestamp (null = never expires).
TOKENS_FILE = Path.home() / ".phone_arm_tokens.json"


def _env_float(name: str, default: float) -> float:
    try:
        return float(_os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


# If no pose message has been received in this many ms, treat the pose as stale
# for recalibration/discontinuity checks. The action path keeps the last target,
# so brief delivery gaps become a hold instead of a clutch reset.
STALE_POSE_TIMEOUT_MS = 250.0
# Drop duplicate/backwards pose datagrams, but do not reject otherwise-ordered
# poses just because transport latency is high. Operators can adapt to steady
# lag; the unsafe case is replaying an older command after a newer one landed.
POSE_ORDER_DROP_LOG_PERIOD_S = 1.0
INCIDENT_MIN_INTERVAL_S = _env_float("PHONE_ARM_INCIDENT_MIN_INTERVAL_S", 2.0)
INCIDENT_RELAY_SEGMENT_MS = _env_float(
    "PHONE_ARM_INCIDENT_RELAY_SEGMENT_MS",
    STALE_POSE_TIMEOUT_MS,
)
CONTROL_OWNER_TIMEOUT_S = _env_float("PHONE_ARM_CONTROL_OWNER_TIMEOUT_S", 15.0)
# ARCore/WebXR can relocalize and jump the reported pose while still claiming
# pose_valid=true. If that happens with B1 held, freeze phone motion so we don't
# command a huge target. The operator must release and reengage B1 to resume.
# (A previous POSE_MAX_SPEED_MPS=1.0 gate was removed 2026-06-13: per-session
# analysis of the johnreyrio run showed ~2,300 trips, of which ~51% were jitter
# artifacts (back-to-back packet arrivals after a network gap inflated apparent
# speed) and ~49% were real operator hand motion above 1 m/s. The handful of
# real ARCore catches it claimed were actually pre-emptions of the step gate
# (gate already latched by a false-positive when the real jump arrived). The
# step gate alone covers the safety case.)
POSE_JUMP_MAX_M = 0.12
# Speed threshold that must ALSO be exceeded (in addition to POSE_JUMP_MAX_M)
# for a position sample to be treated as an ARCore glitch rather than real
# fast motion or a long-time-delta artifact. Rationale from the 2026-07-02
# S22 session analysis: 17 position faults broke down as
#   - ~5 real fast intentional hand motion    (speeds 3-8 m/s)
#   - ~2 very fast but plausible motion       (speeds 8-11 m/s)
#   - ~5 low-speed big-step time-delay art.   (speeds < 1 m/s, big step from
#                                              a pose-gap accumulating real
#                                              motion at normal speed)
#   - ~5 genuine ARCore tracking glitches     (speeds 13-47 m/s -- physically
#                                              impossible for a hand)
# Pure step check catches everything the same way. Adding a "step AND speed"
# criterion at 12 m/s lets normal fast motion through, ignores slow moves
# over long delays, and still catches the tracking glitches (all at 13+ m/s).
# Human hand peak speeds observed in sports (cricket bowling, table-tennis
# wrist snap) are 8-12 m/s at the extreme; 12 m/s gives us a small safety
# margin over that.
POSE_SPEED_MAX_MPS = 12.0
# Orientation discontinuity guard. The final-jump log from 2026-06-10 showed a
# ~40deg yaw jump while pose_valid stayed true; if accepted, that yaw directly
# becomes shoulder_pan motion. Treat implausible orientation steps the same way
# as position relocalization: hold until the operator reengages B1.
ORIENT_YAW_JUMP_MAX_DEG = 25.0
ORIENT_YAW_MAX_SPEED_DPS = 360.0
ORIENT_ROT_JUMP_MAX_DEG = 35.0
ORIENT_ROT_MAX_SPEED_DPS = 720.0

# Rate-limit per-frame debug prints to ~2Hz.
DEBUG_PRINT_PERIOD_S = 0.5

# Above this pitch from horizontal (~60°, cos = 0.5), the phone's forward axis
# is too close to vertical for yaw/roll to be numerically well-defined --
# arctan2(fwd[0], -fwd[2]) flips wildly on tiny noise. In that region we hold
# the previous yaw_delta and phone_roll instead of letting them ricochet.
ORIENT_FREEZE_COS_THRESHOLD = 0.5


@dataclass
class _FaultRecovery:
    """Continuity-fault detection state, scoped to a B1-held run.

    Decoupled from the rest of BrowserPhone state so it can be reset as a unit
    on re-cal / B1 release without spelling out four fields every time."""
    # Last calibrated pose that passed the continuity gate while B1 was held.
    last_accepted_pos_cal: np.ndarray | None = None
    last_accepted_pose_t_ms: float | None = None
    last_accepted_yaw_delta_deg: float | None = None
    last_accepted_rot: Rotation | None = None
    last_accepted_orient_t_ms: float | None = None
    # True after a step/speed fault; arm holds until B1 release/reengage.
    latched: bool = False
    # Details for the active latch, used to correlate fault_latch/fault_recover.
    latch_id: int = 0
    latch_kind: str | None = None
    latch_details: dict | None = None

    def reset(self) -> None:
        self.last_accepted_pos_cal = None
        self.last_accepted_pose_t_ms = None
        self.last_accepted_yaw_delta_deg = None
        self.last_accepted_rot = None
        self.last_accepted_orient_t_ms = None
        self.latched = False
        self.latch_id = 0
        self.latch_kind = None
        self.latch_details = None


def _ensure_cert() -> tuple[Path, Path]:
    cert = CERTS_DIR / "server.crt"
    key = CERTS_DIR / "server.key"
    if cert.exists() and key.exists():
        return cert, key
    CERTS_DIR.mkdir(parents=True, exist_ok=True)
    hostname = socket.gethostname()
    # The phone connects to the VPS public IP (TLS terminates here behind the
    # bore tunnel), so that IP must be in the SANs. No LAN IPs: we are not
    # reachable on the LAN anymore -- everything goes through the VPS.
    sans = ["DNS:localhost", f"DNS:{hostname}", "IP:127.0.0.1", f"IP:{VPS_HOST}"]
    san_str = ",".join(sans)
    subprocess.run(
        [
            "openssl", "req", "-x509", "-nodes", "-newkey", "rsa:2048",
            "-keyout", str(key), "-out", str(cert),
            "-days", "3650", "-subj", f"/CN={hostname}",
            "-addext", f"subjectAltName={san_str}",
        ],
        check=True,
    )
    return cert, key


class BrowserPhone:
    """Browser-backed phone teleoperator.

    Runs the browser/control aiohttp server in a subprocess. Last-received pose
    is held under a lock and surfaced via get_action() to the teleop loop.

    On B1 rising edge (operator first presses the hold button), we re-zero
    the calibration (the phone's current pose becomes neutral). This is
    the same UX as HEBI Mobile I/O.
    """

    name = "browser_phone"
    config_class = PhoneConfig

    def __init__(self, config: PhoneConfig, port: int = DEFAULT_PORT):
        self.config = config
        self.port = port

        self._state_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._latest_msg: dict | None = None
        self._latest_robot_feedback: dict = {}
        # Cached gated action result. Populated by _process_pose_state() at
        # pose-msg ingest time; read by get_action() at IK-tick time. This is
        # the linchpin of the single-timeline architecture: the gate processes
        # every msg exactly once, in the order it arrived, and the IK loop
        # just surfaces the latest cached decision. No IK-tick interleaving,
        # no msg skipping, no gate state that diverges from the recorder.
        self._latest_action_state: dict | None = None
        self._n_msgs = 0
        # WT arrival-pattern counters reset every 2s by _relay_stats_loop.
        # Burst count = consecutive msgs arriving within 1ms of the prior --
        # under normal network delivery, msgs are spaced ~33ms apart (30 Hz).
        # Sub-ms inter-arrivals mean the kernel UDP buffer flushed a backlog,
        # which means the asyncio loop was busy and missed reads. Max gap is
        # the largest dt between consecutive arrivals -- the duration of the
        # stall that produced the burst.
        self._dc_window_count = 0
        self._dc_window_bursts = 0
        self._dc_window_max_gap_ms = 0.0
        self._dc_window_out_of_order_drops = 0
        self._dc_window_max_source_age_ms = 0.0
        self._dc_window_client_skipped = 0
        self._dc_window_client_buffered_max = 0
        self._dc_window_first_seq: int | None = None
        self._dc_window_last_seq: int | None = None
        self._dc_window_seq_gap_count = 0
        self._dc_window_seq_missing = 0
        self._dc_window_seq_max_gap = 0
        self._dc_window_seq_backwards = 0
        self._dc_window_seq_duplicates = 0
        self._dc_window_phone_to_relay_sum_ms = 0.0
        self._dc_window_phone_to_relay_max_ms = 0.0
        self._dc_window_phone_to_relay_count = 0
        self._dc_window_browser_queue_sum_ms = 0.0
        self._dc_window_browser_queue_max_ms = 0.0
        self._dc_window_browser_queue_count = 0
        self._dc_window_browser_write_to_edge_sum_ms = 0.0
        self._dc_window_browser_write_to_edge_max_ms = 0.0
        self._dc_window_browser_write_to_edge_count = 0
        self._dc_window_browser_write_to_relay_sum_ms = 0.0
        self._dc_window_browser_write_to_relay_max_ms = 0.0
        self._dc_window_browser_write_to_relay_count = 0
        self._dc_window_edge_queue_sum_ms = 0.0
        self._dc_window_edge_queue_max_ms = 0.0
        self._dc_window_edge_queue_count = 0
        self._dc_window_edge_to_relay_sum_ms = 0.0
        self._dc_window_edge_to_relay_max_ms = 0.0
        self._dc_window_edge_to_relay_count = 0
        self._dc_window_relay_queue_sum_ms = 0.0
        self._dc_window_relay_queue_max_ms = 0.0
        self._dc_window_relay_queue_count = 0
        self._dc_window_relay_to_pi_sum_ms = 0.0
        self._dc_window_relay_to_pi_max_ms = 0.0
        self._dc_window_relay_to_pi_count = 0
        self._dc_last_client_ctrl: dict = {}
        self._dc_last_arrival_ms: float | None = None
        self._dc_seq_stream_id: str | None = None
        self._dc_last_seq: int | None = None
        self._dc_last_browser_t_ms: float | None = None

        self._loop: asyncio.AbstractEventLoop | None = None
        self._runner: web.AppRunner | None = None
        self._site: web.TCPSite | None = None
        self._ready_event = threading.Event()
        # Process isolation is permanent: the browser/control server and
        # trajectory writer both run outside the robot-control loop.
        self._server_process = None
        self._action_ipc_queue = None
        self._command_ipc_queue = None
        self._status_ipc_queue = None
        self._ipc_command_thread: threading.Thread | None = None
        # Trajectory CSV is high-volume and should not share the browser/control
        # server's recorder queue. In process-split mode the robot process sends
        # trajectory rows directly to this writer process; the browser server
        # only tells it which session directory to use.
        self._trajectory_process = None
        self._trajectory_ipc_queue = None
        self._trajectory_status_queue = None

        # Token auth -- populated by _load_tokens() at server startup and via
        # SIGHUP. Empty list + _auth_enabled=False = no token file = no auth.
        self._tokens: list[dict] = []
        self._auth_enabled = False
        self._control_reacquire_required = False
        self._control_reacquire_reason: str | None = None
        self._relay_task: asyncio.Task | None = None
        self._relay_stats_task: asyncio.Task | None = None
        self._relay_epoch_seen: int | None = None
        # Browser-side control ownership. Video is multi-subscriber via
        # MediaMTX, but only one browser should ever receive the WebTransport
        # phone-role URL. Same page can reconnect; a new page can claim only
        # after the active owner lease goes quiet.
        self._control_owner_page_id: str | None = None
        self._control_owner_name: str | None = None
        self._control_owner_claim_ms = 0.0
        self._control_owner_seen_ms = 0.0

        # Calibration baseline. Updated on B1 rising edge.
        self._calib_pos = np.zeros(3)
        # Yaw-only inverse for the POSITION mapping: the operator holds the phone
        # tilted back to view the screen, and rotating the hand-translation by the
        # full tilt leaks vertical motion into EE-reach ("can't lift up"). Using
        # only the clutch yaw keeps world-vertical -> EE-up at any tilt.
        self._calib_yaw_inv = Rotation.identity()
        self._phone_roll_deg = 0.0  # gravity-referenced phone twist (for wrist_roll)
        # Gimbal-guard release re-anchoring (see investigation 2026-06-05): when
        # the guard freezes yaw_delta/phone_roll because pitch is near vertical,
        # the operator can rotate their hand during the freeze. Releasing the
        # guard naively makes yaw/roll snap to the full accumulated rotation in
        # one frame, swinging the IK target ~1m and slamming the arm. We track
        # whether the guard was active LAST frame; on the active->released edge
        # we re-anchor _clutch_yaw and offset _phone_roll so the reported values
        # stay continuous across the transition.
        self._orient_was_frozen = False
        self._phone_roll_offset = 0.0
        self._phone_pitch_deg = 0.0  # gravity-referenced phone pitch (for wrist_flex experiment)
        self._last_yaw_delta_deg = 0.0  # held when phone pitch is near vertical (gimbal-prone)
        # Clutch heading (rad), for cylindrical yaw->shoulder_pan mapping.
        self._clutch_yaw = 0.0
        self._b1_prev = False  # raw B1, used for rising-edge re-cal
        # B1 was pressed while tracking was not fresh. Do not silently resume
        # when tracking recovers; require a release and fresh B1 press.
        self._b1_reengage_required = False
        self._b1_reengage_reason: str | None = None

        # Per-session recorder. Opened lazily on first authenticated browser
        # activity (so the token name is known); closed in _shutdown.
        self._session_recorder: SessionRecorder | None = None

        # Rate-limiting for debug prints (wallclock-based, NOT frame-counted,
        # so the rate is independent of control-loop hz).
        self._last_debug_print_t = 0.0
        self._last_wrtc_print_t = 0.0
        # Track whether we already announced "stale" so it logs once per gap,
        # not 60 times a second.
        self._stale_logged = False
        self._stale_started_ms: float | None = None
        # Track ARKit/WebXR pose validity transitions separately from B1 so we
        # can correlate "phone got weird" moments with tracking loss.
        self._last_pose_valid: bool | None = None
        # Cached last-good pose. On a malformed quaternion or a message that's
        # missing the pos/rot_xyzw field we hold these instead of snapping to
        # origin / identity (which would whip wrist_flex toward 0°, bypass the
        # |fwd|-vertical orient-freeze guard, and -- with B1 held -- shove the
        # desired EE by a calibration-offset's worth in one frame).
        self._last_raw_pos: np.ndarray = np.zeros(3)
        self._last_raw_rot: Rotation = Rotation.identity()
        # Pose-jump/orientation fault detection (see _FaultRecovery).
        self._fault = _FaultRecovery()
        self._fault_latch_seq = 0
        self._ws_out_of_order_drops = 0
        self._last_ws_order_drop_log_t = 0.0
        self._last_incident_log_t: dict[str, float] = {}

    # --- Teleoperator interface -----------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._server_process is not None and self._server_process.is_alive()

    @property
    def is_calibrated(self) -> bool:
        return True  # Always usable; first B1 press performs in-place re-cal

    @check_if_already_connected
    def connect(self) -> None:
        self._connect_process()

    def _connect_process(self) -> None:
        global _ACTIVE_RECORDER
        self._ready_event.clear()
        ctx = _mp.get_context("spawn")
        self._action_ipc_queue = ctx.Queue(maxsize=8)
        self._command_ipc_queue = ctx.Queue(maxsize=2048)
        self._status_ipc_queue = ctx.Queue(maxsize=16)
        self._start_trajectory_writer_process(ctx)
        self._server_process = ctx.Process(
            target=_browser_phone_process_main,
            name="BrowserPhoneServerProcess",
            args=(
                self.config,
                self.port,
                self._action_ipc_queue,
                self._command_ipc_queue,
                self._status_ipc_queue,
                self._trajectory_ipc_queue,
            ),
            daemon=True,
        )
        self._server_process.start()
        deadline = time.time() + float(_os.environ.get("PHONE_ARM_SUBPROC_STARTUP_S", "30"))
        while time.time() < deadline:
            if self._server_process is not None and not self._server_process.is_alive():
                self._stop_trajectory_writer_process()
                raise RuntimeError(
                    f"BrowserPhone server process exited early with "
                    f"code {self._server_process.exitcode}"
                )
            try:
                kind, payload = self._status_ipc_queue.get(timeout=0.1)
            except _queue.Empty:
                continue
            if kind == "ready":
                self._ready_event.set()
                _ACTIVE_RECORDER = _RecorderProxy(
                    self._command_ipc_queue,
                    self._trajectory_ipc_queue,
                )
                self._install_sighup_handler()
                print(f"[browser_phone] server process pid={self._server_process.pid} ready")
                return
            if kind == "error":
                self._stop_trajectory_writer_process()
                raise RuntimeError(f"BrowserPhone server process failed: {payload}")
        self._stop_trajectory_writer_process()
        raise RuntimeError("BrowserPhone server process failed to start within startup window")

    def _trajectory_queue_max(self) -> int:
        raw = _os.environ.get("PHONE_ARM_TRAJECTORY_QUEUE_MAX", "0")
        try:
            return max(0, int(raw))
        except (TypeError, ValueError):
            print(
                "[browser_phone] invalid PHONE_ARM_TRAJECTORY_QUEUE_MAX="
                f"{raw!r}; using unbounded trajectory queue"
            )
            return 0

    def _start_trajectory_writer_process(self, ctx) -> None:
        if self._trajectory_process is not None:
            return
        self._trajectory_ipc_queue = ctx.Queue(maxsize=self._trajectory_queue_max())
        self._trajectory_status_queue = ctx.Queue(maxsize=16)
        self._trajectory_process = ctx.Process(
            target=_trajectory_writer_process_main,
            name="PhoneArmTrajectoryWriter",
            args=(self._trajectory_ipc_queue, self._trajectory_status_queue),
        )
        self._trajectory_process.daemon = False
        self._trajectory_process.start()
        deadline = time.time() + float(_os.environ.get("PHONE_ARM_SUBPROC_STARTUP_S", "30"))
        while time.time() < deadline:
            if (self._trajectory_process is not None
                    and not self._trajectory_process.is_alive()):
                raise RuntimeError(
                    "trajectory writer exited early with code "
                    f"{self._trajectory_process.exitcode}"
                )
            try:
                kind, payload = self._trajectory_status_queue.get(timeout=0.1)
            except _queue.Empty:
                continue
            if kind == "ready":
                print(
                    f"[browser_phone] trajectory writer pid="
                    f"{self._trajectory_process.pid} ready"
                )
                return
            if kind == "error":
                raise RuntimeError(f"trajectory writer failed: {payload}")
        self._stop_trajectory_writer_process()
        raise RuntimeError("trajectory writer failed to start within startup window")

    def _stop_trajectory_writer_process(self) -> None:
        proc = self._trajectory_process
        if proc is None:
            return
        q = self._trajectory_ipc_queue
        if q is not None:
            try:
                q.put(("shutdown", None), timeout=2.0)
            except Exception as e:  # noqa: BLE001
                print(f"[browser_phone] trajectory writer shutdown signal failed: {e}")
        raw_timeout = _os.environ.get("PHONE_ARM_TRAJECTORY_DRAIN_TIMEOUT_S", "120")
        try:
            timeout_s = max(1.0, float(raw_timeout))
        except (TypeError, ValueError):
            timeout_s = 120.0
        proc.join(timeout=timeout_s)
        if proc.is_alive():
            print(
                "[browser_phone] trajectory writer did not drain within "
                f"{timeout_s:.0f}s; terminating (trajectory.csv may be incomplete)"
            )
            proc.terminate()
            proc.join(timeout=5.0)
        self._trajectory_process = None
        self._trajectory_ipc_queue = None
        self._trajectory_status_queue = None

    def _install_sighup_handler(self) -> None:
        # Register SIGHUP -> reload tokens from the MAIN thread (the only place
        # signal.signal works). mint_token.py signals the parent follower.main
        # process, so we forward reloads to the child over IPC. Without this the
        # default SIGHUP action would terminate teleop.
        def _on_sighup(_signum, _frame):
            if self._command_ipc_queue is not None:
                try:
                    self._command_ipc_queue.put_nowait(("reload_tokens", None))
                except (_queue.Full, BrokenPipeError, EOFError, OSError):
                    print("[browser_phone] token reload signal dropped; command queue unavailable")
        try:
            signal.signal(signal.SIGHUP, _on_sighup)
        except (ValueError, OSError) as e:
            # ValueError if connect() ever ends up on a non-main thread.
            print(f"[browser_phone] could not install SIGHUP handler ({e}); "
                  "token edits will require restart")

    def calibrate(self) -> None:
        # No-op; first B1 rising edge calibrates in place.
        pass

    def configure(self) -> None:
        pass

    def _recalibrate(self, raw_pos, raw_rot) -> None:
        """Anchor the position + yaw-only control frame to the current phone pose
        (the clutch reference). Used on B1 rising edge.
        Yaw-only frame: project forward onto the horizontal plane (WebXR +Y up) and
        undo heading via R_y(+yaw), so the control frame is heading-independent."""
        self._calib_pos = raw_pos.copy()
        _fwd = raw_rot.apply([0.0, 0.0, -1.0])
        _yaw = float(np.arctan2(_fwd[0], -_fwd[2]))
        self._calib_yaw_inv = Rotation.from_euler('y', _yaw)
        self._clutch_yaw = _yaw
        # Re-clutch clears the gimbal-guard re-anchor state too: any prior
        # accumulated phone-roll offset belonged to the OLD clutch reference.
        # _last_yaw_delta_deg also has to reset to 0 -- if the next frame falls
        # in the frozen zone (pitch past ~60deg, common right after a clutch
        # where the operator is still tilted), the frozen branch reuses
        # _last_yaw_delta_deg verbatim. Without this reset that's the old
        # absolute yaw against the OLD clutch frame, causing a one-frame
        # EE-target swing of ~200mm (investigated 2026-06-06).
        self._orient_was_frozen = False
        self._phone_roll_offset = 0.0
        self._last_yaw_delta_deg = 0.0
        self._fault.reset()

    @staticmethod
    def _rotation_step_deg(prev: Rotation | None, cur: Rotation) -> float:
        if prev is None:
            return 1e9
        return float(np.degrees((prev.inv() * cur).magnitude()))

    @staticmethod
    def _source_age_ms(msg: dict, t_recv_ms: float) -> float | None:
        """Estimate how old a client pose was when it reached this process."""
        if not bool(msg.get("off_valid", False)):
            return None
        try:
            t_client_ms = float(msg["t"])
            clock_offset_ms = float(msg.get("off", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        if not (np.isfinite(t_client_ms) and np.isfinite(clock_offset_ms)):
            return None
        return t_recv_ms - (t_client_ms + clock_offset_ms)

    @staticmethod
    def _finite_float(value) -> float | None:
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        if not np.isfinite(out):
            return None
        return out

    @staticmethod
    def _note_window_value(value: float | None, total: float, max_value: float, count: int):
        if value is None:
            return total, max_value, count
        return total + value, max(max_value, value), count + 1

    def _incident_snapshot(self) -> dict:
        now_ms = time.time() * 1000.0
        last_arrival_age_ms = None
        if self._dc_last_arrival_ms is not None:
            last_arrival_age_ms = round(now_ms - self._dc_last_arrival_ms, 1)
        gate = self._operator_gate_status()
        return {
            "last_seq": self._dc_last_seq,
            "ws_msg_count": self._n_msgs,
            "last_arrival_age_ms": last_arrival_age_ms,
            "window_count": self._dc_window_count,
            "window_max_gap_ms": round(self._dc_window_max_gap_ms, 1),
            "window_out_of_order_drops": self._dc_window_out_of_order_drops,
            "window_max_source_age_ms": round(self._dc_window_max_source_age_ms, 1),
            "window_client_buffered_max": self._dc_window_client_buffered_max,
            "window_client_skipped": self._dc_window_client_skipped,
            "window_seq_missing": self._dc_window_seq_missing,
            "window_seq_max_gap": self._dc_window_seq_max_gap,
            "control_reacquire_required": int(self._control_reacquire_required),
            "control_reacquire_reason": self._control_reacquire_reason,
            "b1_reengage_required": int(self._b1_reengage_required),
            "b1_reengage_reason": self._b1_reengage_reason,
            "operator_gate_state": gate["state"],
            "operator_gate_reason": gate["reason"],
            "out_of_order_drops_total": self._ws_out_of_order_drops,
            "client_ctrl_state": self._dc_last_client_ctrl.get("ctrl_state"),
            "client_ctrl_ready": self._dc_last_client_ctrl.get("ctrl_ready"),
            "client_ctrl_conn_seq": self._dc_last_client_ctrl.get("ctrl_conn_seq"),
            "client_ctrl_reconnect_count": self._dc_last_client_ctrl.get(
                "ctrl_reconnect_count"
            ),
            "client_ctrl_last_reconnect_reason": self._dc_last_client_ctrl.get(
                "ctrl_last_reconnect_reason"
            ),
        }

    def _operator_gate_status(self, data: dict | None = None) -> dict:
        """Short operator-facing state for the phone UI.

        This mirrors the same gates that decide whether phone motion can affect
        the arm; it is diagnostic/user feedback only and must not mutate state.
        """
        if data is not None:
            b1_held = bool(data.get("enabled", self._b1_prev))
        else:
            b1_held = self._b1_prev
        pose_valid = bool(data.get("pose_valid", True)) if data is not None else True
        if self._control_reacquire_required:
            reason = self._control_reacquire_reason or "control_reconnect"
            return {
                "state": "reengage_required",
                "severity": "bad",
                "message": (
                    "RELEASE, THEN REENGAGE B1 TO CONTINUE"
                    if b1_held else
                    "REENGAGE B1 TO CONTINUE"
                ),
                "requires_reengage": True,
                "reason": reason,
            }
        if self._fault.latched:
            return {
                "state": "tracking_hold",
                "severity": "bad",
                "message": "RELEASE, THEN REENGAGE B1 TO CONTINUE",
                "requires_reengage": True,
                "reason": self._fault.latch_kind or "tracking_fault",
            }
        if self._b1_reengage_required:
            return {
                "state": "b1_reengage_required",
                "severity": "bad",
                "message": (
                    "RELEASE, THEN REENGAGE B1 TO CONTINUE"
                    if b1_held else
                    "REENGAGE B1 TO CONTINUE"
                ),
                "requires_reengage": True,
                "reason": self._b1_reengage_reason or "b1_without_fresh_tracking",
            }
        if not pose_valid:
            return {
                "state": "no_tracking",
                "severity": "warn",
                "message": "NO AR TRACKING",
                "requires_reengage": False,
                "reason": "pose_invalid",
            }
        return {
            "state": "ok",
            "severity": "ok",
            "message": "",
            "requires_reengage": False,
            "reason": None,
        }

    def _record_incident(
        self,
        event_type: str,
        details: dict,
        *,
        min_interval_s: float = INCIDENT_MIN_INTERVAL_S,
    ) -> None:
        now_t = time.time()
        last_t = self._last_incident_log_t.get(event_type, 0.0)
        if now_t - last_t < min_interval_s:
            return
        self._last_incident_log_t[event_type] = now_t
        payload = self._incident_snapshot()
        payload.update(details)
        _record_session_event(event_type, payload)

    def _update_relay_timing_stats(self, data: dict, t_recv_ms: float) -> None:
        """Preserve relay hop timing on each accepted relay pose.

        Phone clock is browser time plus the browser's estimated offset to this
        process. Relay/Pi clock comparisons assume NTP is close enough for
        attribution; raw timestamps are also recorded so we can spot skew.
        """
        relay_recv_ms = self._finite_float(data.get("_relay_recv_ms"))
        relay_send_ms = self._finite_float(data.get("_relay_send_ms"))
        relay_queue_ms = self._finite_float(data.get("_relay_queue_age_ms"))
        if relay_queue_ms is None and relay_recv_ms is not None and relay_send_ms is not None:
            relay_queue_ms = max(0.0, relay_send_ms - relay_recv_ms)
            data["_relay_queue_age_ms"] = relay_queue_ms

        relay_to_pi_ms = None
        if relay_send_ms is not None:
            relay_to_pi_ms = t_recv_ms - relay_send_ms
            data["_relay_to_pi_ms"] = relay_to_pi_ms

        edge_recv_ms = self._finite_float(data.get("_edge_recv_ms"))
        edge_send_ms = self._finite_float(data.get("_edge_send_ms"))
        if edge_send_ms is not None and relay_recv_ms is not None:
            data["_edge_to_relay_ms"] = relay_recv_ms - edge_send_ms

        t_client_ms = self._finite_float(data.get("t"))
        t_browser_write_ms = self._finite_float(data.get("t_browser_write_ms"))
        if t_client_ms is not None and t_browser_write_ms is not None:
            data["_browser_queue_ms"] = t_browser_write_ms - t_client_ms

        phone_to_relay_ms = None
        relay_total_age_ms = None
        if bool(data.get("off_valid", False)):
            clock_offset_ms = self._finite_float(data.get("off"))
            if t_client_ms is not None and clock_offset_ms is not None:
                phone_send_pi_clock_ms = t_client_ms + clock_offset_ms
                if relay_recv_ms is not None:
                    phone_to_relay_ms = relay_recv_ms - phone_send_pi_clock_ms
                    data["_phone_to_relay_ms"] = phone_to_relay_ms
                if relay_send_ms is not None:
                    relay_total_age_ms = relay_send_ms - phone_send_pi_clock_ms
                    data["_relay_total_age_ms"] = relay_total_age_ms
                if t_browser_write_ms is not None:
                    browser_write_pi_clock_ms = t_browser_write_ms + clock_offset_ms
                    if edge_recv_ms is not None:
                        data["_browser_write_to_edge_ms"] = (
                            edge_recv_ms - browser_write_pi_clock_ms
                        )
                    if relay_recv_ms is not None:
                        data["_browser_write_to_relay_ms"] = (
                            relay_recv_ms - browser_write_pi_clock_ms
                        )

        (
            self._dc_window_phone_to_relay_sum_ms,
            self._dc_window_phone_to_relay_max_ms,
            self._dc_window_phone_to_relay_count,
        ) = self._note_window_value(
            phone_to_relay_ms,
            self._dc_window_phone_to_relay_sum_ms,
            self._dc_window_phone_to_relay_max_ms,
            self._dc_window_phone_to_relay_count,
        )
        (
            self._dc_window_browser_queue_sum_ms,
            self._dc_window_browser_queue_max_ms,
            self._dc_window_browser_queue_count,
        ) = self._note_window_value(
            self._finite_float(data.get("_browser_queue_ms")),
            self._dc_window_browser_queue_sum_ms,
            self._dc_window_browser_queue_max_ms,
            self._dc_window_browser_queue_count,
        )
        (
            self._dc_window_browser_write_to_edge_sum_ms,
            self._dc_window_browser_write_to_edge_max_ms,
            self._dc_window_browser_write_to_edge_count,
        ) = self._note_window_value(
            self._finite_float(data.get("_browser_write_to_edge_ms")),
            self._dc_window_browser_write_to_edge_sum_ms,
            self._dc_window_browser_write_to_edge_max_ms,
            self._dc_window_browser_write_to_edge_count,
        )
        (
            self._dc_window_browser_write_to_relay_sum_ms,
            self._dc_window_browser_write_to_relay_max_ms,
            self._dc_window_browser_write_to_relay_count,
        ) = self._note_window_value(
            self._finite_float(data.get("_browser_write_to_relay_ms")),
            self._dc_window_browser_write_to_relay_sum_ms,
            self._dc_window_browser_write_to_relay_max_ms,
            self._dc_window_browser_write_to_relay_count,
        )
        (
            self._dc_window_edge_queue_sum_ms,
            self._dc_window_edge_queue_max_ms,
            self._dc_window_edge_queue_count,
        ) = self._note_window_value(
            self._finite_float(data.get("_edge_q_ms")),
            self._dc_window_edge_queue_sum_ms,
            self._dc_window_edge_queue_max_ms,
            self._dc_window_edge_queue_count,
        )
        (
            self._dc_window_edge_to_relay_sum_ms,
            self._dc_window_edge_to_relay_max_ms,
            self._dc_window_edge_to_relay_count,
        ) = self._note_window_value(
            self._finite_float(data.get("_edge_to_relay_ms")),
            self._dc_window_edge_to_relay_sum_ms,
            self._dc_window_edge_to_relay_max_ms,
            self._dc_window_edge_to_relay_count,
        )
        (
            self._dc_window_relay_queue_sum_ms,
            self._dc_window_relay_queue_max_ms,
            self._dc_window_relay_queue_count,
        ) = self._note_window_value(
            relay_queue_ms,
            self._dc_window_relay_queue_sum_ms,
            self._dc_window_relay_queue_max_ms,
            self._dc_window_relay_queue_count,
        )
        (
            self._dc_window_relay_to_pi_sum_ms,
            self._dc_window_relay_to_pi_max_ms,
            self._dc_window_relay_to_pi_count,
        ) = self._note_window_value(
            relay_to_pi_ms,
            self._dc_window_relay_to_pi_sum_ms,
            self._dc_window_relay_to_pi_max_ms,
            self._dc_window_relay_to_pi_count,
        )
        for segment, value in (
            ("phone_to_relay", phone_to_relay_ms),
            ("relay_queue", relay_queue_ms),
            ("relay_to_pi", relay_to_pi_ms),
        ):
            if value is not None and value > INCIDENT_RELAY_SEGMENT_MS:
                self._record_incident("incident_relay_latency", {
                    "segment": segment,
                    "latency_ms": round(value, 1),
                    "threshold_ms": INCIDENT_RELAY_SEGMENT_MS,
                    "seq": data.get("seq"),
                    "source_age_ms": data.get("_source_age_ms"),
                    "phone_to_relay_ms": phone_to_relay_ms,
                    "browser_queue_ms": data.get("_browser_queue_ms"),
                    "browser_write_to_edge_ms": data.get("_browser_write_to_edge_ms"),
                    "browser_write_to_relay_ms": data.get("_browser_write_to_relay_ms"),
                    "relay_queue_ms": relay_queue_ms,
                    "relay_to_pi_ms": relay_to_pi_ms,
                    "edge_queue_ms": data.get("_edge_q_ms"),
                    "edge_to_relay_ms": data.get("_edge_to_relay_ms"),
                    "client_buffered": data.get("ws_buffered"),
                    "client_skipped": data.get("ws_skipped"),
                })
                break

    def _maybe_log_out_of_order_drop(self, msg: dict, details: dict) -> None:
        now_t = time.time()
        if now_t - self._last_ws_order_drop_log_t <= POSE_ORDER_DROP_LOG_PERIOD_S:
            return
        self._last_ws_order_drop_log_t = now_t
        print(
            "[browser_phone] dropped out-of-order pose "
            f"reason={details.get('reason')} seq={msg.get('seq', '?')} "
            f"last_seq={details.get('last_seq')} drops={self._ws_out_of_order_drops} "
            f"src_age={msg.get('_source_age_ms', '?')}ms "
            f"client_skipped={msg.get('ws_skipped', '?')} "
            f"client_buffered={msg.get('ws_buffered', '?')}"
        )
        self._record_incident("incident_out_of_order_pose_drop", {
            **details,
            "source_age_ms": msg.get("_source_age_ms"),
            "phone_to_relay_ms": msg.get("_phone_to_relay_ms"),
            "browser_queue_ms": msg.get("_browser_queue_ms"),
            "browser_write_to_edge_ms": msg.get("_browser_write_to_edge_ms"),
            "browser_write_to_relay_ms": msg.get("_browser_write_to_relay_ms"),
            "relay_queue_ms": msg.get("_relay_queue_age_ms"),
            "relay_to_pi_ms": msg.get("_relay_to_pi_ms"),
            "edge_queue_ms": msg.get("_edge_q_ms"),
            "edge_to_relay_ms": msg.get("_edge_to_relay_ms"),
            "client_buffered": msg.get("ws_buffered"),
            "client_skipped": msg.get("ws_skipped"),
        })

    def _pose_link_event_details(self, cached: dict, pose_age_ms: float) -> dict:
        raw_inputs = dict(cached.get("phone.raw_inputs", {}))
        b1_held = bool(raw_inputs.get("b1", False) or cached.get("phone.enabled", False))
        details = {
            "age_ms": round(float(pose_age_ms), 1),
            "last_seq": raw_inputs.get("_seq"),
            "b1_held": int(b1_held),
            "phone_enabled": int(bool(cached.get("phone.enabled", False))),
            "pose_valid": int(bool(cached.get("_pose_valid", True))),
            "ws_msg_count": raw_inputs.get("_ws_msg_count"),
            "source_age_ms": raw_inputs.get("_source_age_ms"),
            "phone_to_relay_ms": raw_inputs.get("_phone_to_relay_ms"),
            "browser_queue_ms": raw_inputs.get("_browser_queue_ms"),
            "browser_write_to_edge_ms": raw_inputs.get("_browser_write_to_edge_ms"),
            "browser_write_to_relay_ms": raw_inputs.get("_browser_write_to_relay_ms"),
            "edge_queue_ms": raw_inputs.get("_edge_q_ms"),
            "edge_to_relay_ms": raw_inputs.get("_edge_to_relay_ms"),
            "relay_queue_age_ms": raw_inputs.get("_relay_queue_age_ms"),
            "relay_to_pi_ms": raw_inputs.get("_relay_to_pi_ms"),
            "relay_total_age_ms": raw_inputs.get("_relay_total_age_ms"),
            "client_buffered": raw_inputs.get("_client_buffered"),
            "client_skipped": raw_inputs.get("_client_skipped"),
            "ctrl_state": raw_inputs.get("_client_ctrl_state"),
            "ctrl_ready": raw_inputs.get("_client_ctrl_ready"),
            "control_reacquire_required": int(
                bool(raw_inputs.get("_control_reacquire_required", 0))
            ),
        }
        return details

    @staticmethod
    def _pose_stream_id(msg: dict) -> str | None:
        page_id = msg.get("page_id")
        if page_id is None:
            return None
        return str(page_id)

    def _update_dc_seq_stats(self, msg: dict) -> tuple[bool, dict | None]:
        try:
            seq = int(msg.get("seq"))
        except (TypeError, ValueError):
            return False, None
        stream_id = self._pose_stream_id(msg)
        t_client_ms = self._finite_float(msg.get("t"))
        if (
            t_client_ms is not None
            and self._dc_last_browser_t_ms is not None
            and t_client_ms < self._dc_last_browser_t_ms
        ):
            self._dc_window_seq_backwards += 1
            return True, {
                "reason": "backwards_timestamp",
                "seq": seq,
                "last_seq": self._dc_last_seq,
                "stream_id": stream_id,
                "t_browser_ms": t_client_ms,
                "last_t_browser_ms": self._dc_last_browser_t_ms,
            }
        if stream_id != self._dc_seq_stream_id:
            self._dc_seq_stream_id = stream_id
            self._dc_last_seq = None
        if self._dc_window_first_seq is None:
            self._dc_window_first_seq = seq
        self._dc_window_last_seq = seq
        if self._dc_last_seq is not None:
            delta = seq - self._dc_last_seq
            if delta > 1:
                missing = delta - 1
                self._dc_window_seq_gap_count += 1
                self._dc_window_seq_missing += missing
                if missing > self._dc_window_seq_max_gap:
                    self._dc_window_seq_max_gap = missing
            elif delta == 0:
                self._dc_window_seq_duplicates += 1
                return True, {
                    "reason": "duplicate_seq",
                    "seq": seq,
                    "last_seq": self._dc_last_seq,
                    "stream_id": stream_id,
                }
            elif delta < 0:
                self._dc_window_seq_backwards += 1
                return True, {
                    "reason": "backwards_seq",
                    "seq": seq,
                    "last_seq": self._dc_last_seq,
                    "stream_id": stream_id,
                }
        self._dc_last_seq = seq
        if t_client_ms is not None:
            self._dc_last_browser_t_ms = t_client_ms
        return False, None

    def _publish_action_state(self, action_state: dict) -> None:
        q = self._action_ipc_queue
        if q is None:
            return
        try:
            pos = action_state["phone.pos"]
            pos_list = pos.tolist() if hasattr(pos, "tolist") else list(pos)
            msg = {
                "pos": pos_list,
                "raw_inputs": dict(action_state["phone.raw_inputs"]),
                "enabled": bool(action_state["phone.enabled"]),
                "t_recv_ms": float(action_state["_t_recv_ms"]),
                "pose_valid": bool(action_state.get("_pose_valid", True)),
                "control_source": action_state.get("control.source", "phone"),
                "leader_positions": action_state.get("leader.positions"),
                "leader_enabled": bool(action_state.get("leader.enabled", False)),
                "leader_source_id": action_state.get("leader.source_id"),
            }
        except Exception as e:  # noqa: BLE001
            print(f"[browser_phone] action IPC serialize failed: {e}")
            return
        try:
            q.put_nowait(msg)
        except _queue.Full:
            try:
                q.get_nowait()
            except _queue.Empty:
                pass
            try:
                q.put_nowait(msg)
            except (_queue.Full, BrokenPipeError, EOFError, OSError):
                pass
        except (BrokenPipeError, EOFError, OSError):
            pass

    def _store_robot_feedback(self, feedback) -> None:
        if not isinstance(feedback, dict):
            return
        clean = {}
        for key, value in feedback.items():
            if not isinstance(key, str):
                continue
            if isinstance(value, bool) or value is None or isinstance(value, str):
                clean[key] = value
            elif isinstance(value, int):
                clean[key] = value
            elif isinstance(value, float):
                try:
                    clean[key] = value if np.isfinite(value) else None
                except (TypeError, ValueError):
                    clean[key] = None
        with self._feedback_lock:
            self._latest_robot_feedback = clean

    def _robot_feedback_snapshot(self) -> dict:
        with self._feedback_lock:
            feedback = dict(self._latest_robot_feedback)
        t_feedback_ms = self._finite_float(feedback.get("robot_tracking_t_pi_ms"))
        if t_feedback_ms is not None:
            feedback["robot_tracking_age_ms"] = max(
                0.0, time.time() * 1000.0 - t_feedback_ms
            )
        return feedback

    def _store_action_ipc_msg(self, msg: dict) -> bool:
        try:
            action_state = {
                "phone.pos": np.asarray(msg["pos"], dtype=float),
                "phone.raw_inputs": dict(msg.get("raw_inputs", {})),
                "phone.enabled": bool(msg.get("enabled", False)),
                "_t_recv_ms": float(msg["t_recv_ms"]),
                "_pose_valid": bool(msg.get("pose_valid", True)),
                "control.source": str(msg.get("control_source", "phone")),
                "leader.positions": msg.get("leader_positions"),
                "leader.enabled": bool(msg.get("leader_enabled", False)),
                "leader.source_id": msg.get("leader_source_id"),
            }
        except (KeyError, TypeError, ValueError) as e:
            print(f"[browser_phone] action IPC parse failed: {e}")
            return False
        with self._state_lock:
            self._latest_action_state = action_state
        return True

    def _drain_action_ipc(self) -> bool:
        q = self._action_ipc_queue
        if q is None:
            return False
        latest = None
        while True:
            try:
                latest = q.get_nowait()
            except _queue.Empty:
                break
            except (BrokenPipeError, EOFError, OSError):
                return False
        if latest is None:
            return False
        return self._store_action_ipc_msg(latest)

    def wait_for_action_update(self, timeout_s: float) -> bool:
        """Block until a new pose action arrives, then keep only the newest one."""
        q = self._action_ipc_queue
        if q is None:
            if timeout_s > 0:
                time.sleep(timeout_s)
            return False
        latest = None
        try:
            latest = q.get(timeout=max(0.0, timeout_s))
        except _queue.Empty:
            return False
        except (BrokenPipeError, EOFError, OSError):
            return False
        while True:
            try:
                latest = q.get_nowait()
            except _queue.Empty:
                break
            except (BrokenPipeError, EOFError, OSError):
                break
        return self._store_action_ipc_msg(latest)

    def _require_control_reacquire(self, reason: str) -> None:
        """After a control-link outage, require B1 release before motion resumes.

        During the outage get_action() keeps returning the cached pose so the
        arm holds the last connected target. The first fresh pose after a
        reconnect may be far away because the operator moved the phone while
        waiting; if B1 is still held we suppress that pose until they release
        and press B1 again, which creates an explicit new clutch edge.
        """
        if self._control_reacquire_required:
            return
        self._control_reacquire_required = True
        self._control_reacquire_reason = reason
        self._b1_reengage_required = False
        self._b1_reengage_reason = None
        self._fault.reset()
        print(f"[browser_phone] control link {reason} -> release and reengage B1")
        rec = self._session_recorder
        if rec is not None:
            rec.record_event("control_reacquire_required", {"reason": reason})

    def _latch_fault(self, kind: str, details: dict) -> None:
        self._fault_latch_seq += 1
        event_details = {
            "fault_id": self._fault_latch_seq,
            "fault_kind": kind,
            **details,
        }
        self._fault.latched = True
        self._fault.latch_id = self._fault_latch_seq
        self._fault.latch_kind = kind
        self._fault.latch_details = event_details
        rec = self._session_recorder
        if rec is not None:
            rec.record_event("fault_latch", event_details)

    @staticmethod
    def _b1_edge_details(raw_inputs: dict | None) -> dict:
        if not raw_inputs:
            return {}
        details = {}
        if "_b1_edge_seq" in raw_inputs:
            try:
                details["edge_seq"] = int(raw_inputs["_b1_edge_seq"])
            except (TypeError, ValueError):
                details["edge_seq"] = raw_inputs["_b1_edge_seq"]
        if "_b1_edge_reason" in raw_inputs:
            details["edge_reason"] = str(raw_inputs["_b1_edge_reason"])
        if "_b1_edge_t_ms" in raw_inputs:
            try:
                details["edge_t_ms"] = float(raw_inputs["_b1_edge_t_ms"])
            except (TypeError, ValueError):
                details["edge_t_ms"] = raw_inputs["_b1_edge_t_ms"]
        return details

    def _handle_b1_edges(self, b1_raw: bool, raw_pos: np.ndarray,
                         raw_rot: Rotation, pose_valid: bool,
                         is_stale: bool,
                         raw_inputs: dict | None = None) -> None:
        """B1 rising edge recalibrates only if tracking is fresh.

        Any bad B1 press, control reacquire, or latched continuity fault must
        pass through release -> fresh press before motion can affect the arm.
        Fresh tracking alone never resumes control while B1 remains held.
        """
        b1_rising = b1_raw and not self._b1_prev
        b1_falling = (not b1_raw) and self._b1_prev
        rec = self._session_recorder
        if b1_rising:
            edge_details = self._b1_edge_details(raw_inputs)
            if pose_valid and not is_stale:
                self._recalibrate(raw_pos, raw_rot)
                self._b1_reengage_required = False
                self._b1_reengage_reason = None
                print("[browser_phone] B1 rising edge: re-calibrated")
                if rec is not None:
                    rec.record_event("b1_press", {
                        "recalibrated": True,
                        **edge_details,
                    })
                    rec.record_event("recal", {"trigger": "b1_rising"})
            else:
                reason = "pose_invalid" if not pose_valid else "pose_stale"
                self._b1_reengage_required = True
                self._b1_reengage_reason = reason
                print(
                    "[browser_phone] B1 rising edge without fresh tracking "
                    "-> release and reengage"
                )
                if rec is not None:
                    rec.record_event("b1_press", {
                        "recalibrated": False,
                        "pose_valid": pose_valid,
                        "is_stale": is_stale,
                        "requires_reengage": True,
                        "reason": reason,
                        **edge_details,
                    })
                    rec.record_event("b1_reengage_required", {
                        "reason": reason,
                        **edge_details,
                    })
        elif b1_falling:
            edge_details = self._b1_edge_details(raw_inputs)
            if rec is not None:
                rec.record_event("b1_release", edge_details)
            if self._b1_reengage_required:
                ready_details = {
                    "trigger": "b1_release",
                    "reason": self._b1_reengage_reason,
                    **edge_details,
                }
                self._b1_reengage_required = False
                self._b1_reengage_reason = None
                print("[browser_phone] B1 reengage ready after release")
                if rec is not None:
                    rec.record_event("b1_reengage_ready", ready_details)
            if self._fault.latched:
                recover_details = {
                    "trigger": "b1_release",
                    "fault_id": self._fault.latch_id,
                    "fault_kind": self._fault.latch_kind,
                    **edge_details,
                }
                self._fault.reset()
                print("[browser_phone] fault cleared by B1 release; waiting for reengage")
                if rec is not None:
                    rec.record_event("fault_recover", recover_details)
        # IMPORTANT: no recalibration while B1 remains held. Real delivery gaps
        # and tracking dropouts must be resolved by a deliberate reengage.
        self._b1_prev = b1_raw

    def _check_pose_discontinuity(self, b1_raw: bool, pos_cal: np.ndarray,
                                   pose_valid: bool, is_stale: bool,
                                   t_motion_ms: float) -> tuple[float, float]:
        """Latch a fault if pos_cal jumps more than POSE_JUMP_MAX_M from the
        last accepted sample (an ARCore relocalization signature). Returns
        (jump_m, speed_mps) for diagnostic logging; speed is no longer a fault
        criterion. Rejected samples never update the baseline -- they don't
        become acceptable just because wall-clock time passed.

        t_motion_ms is the BROWSER SEND timestamp of the pose, not the Pi's
        receive timestamp. Chrome's WT queue occasionally delivers two
        datagrams within 1-2 ms of each other after a brief network stall
        (QUIC congestion control, HWM=1). Using receive-time for dt_s made
        speed = motion/dt explode -- a 6 mm normal step in a 2 ms burst
        became 3 m/s. Browser send-time reflects actual physical motion
        timing regardless of network bursting. See 2026-07-09 session
        20260709_155118 fault at 15:53:16.282 for the reference incident."""
        if not b1_raw:
            self._fault.reset()
            return 0.0, 0.0
        if not (pose_valid and not is_stale and not self._fault.latched):
            return 0.0, 0.0
        if self._fault.last_accepted_pos_cal is None or self._fault.last_accepted_pose_t_ms is None:
            self._fault.last_accepted_pos_cal = pos_cal.copy()
            self._fault.last_accepted_pose_t_ms = t_motion_ms
            return 0.0, 0.0
        pose_jump_m = float(np.linalg.norm(pos_cal - self._fault.last_accepted_pos_cal))
        dt_s = max((t_motion_ms - self._fault.last_accepted_pose_t_ms) / 1000.0, 1e-6)
        pose_speed_mps = pose_jump_m / dt_s
        # Fault only if BOTH the step exceeds POSE_JUMP_MAX_M AND the implied
        # speed exceeds POSE_SPEED_MAX_MPS. Pure step-based check false-
        # positived on (a) real fast intentional motion where 4-8 m/s hand
        # motion produced 150-250 mm steps at 30 Hz, and (b) time-delay
        # artifacts where a pose gap accumulated a big step at normal speeds.
        # Adding the speed criterion cuts fault rate roughly in half while
        # still catching genuine ARCore tracking glitches (which uniformly
        # show speeds >13 m/s -- physically impossible for a hand).
        step_fault = (
            POSE_JUMP_MAX_M > 0.0
            and pose_jump_m > POSE_JUMP_MAX_M
            and pose_speed_mps > POSE_SPEED_MAX_MPS
        )
        if step_fault:
            print(
                "[browser_phone] pose discontinuity while B1 held: "
                f"step {pose_jump_m*1000:.0f}mm > {POSE_JUMP_MAX_M*1000:.0f}mm AND "
                f"speed {pose_speed_mps:.1f}m/s > {POSE_SPEED_MAX_MPS}m/s; "
                "release and reengage B1 to resume"
            )
            self._latch_fault("position", {
                "step_mm": round(pose_jump_m * 1000, 1),
                "speed_mps": round(pose_speed_mps, 2),
                "step_threshold_mm": round(POSE_JUMP_MAX_M * 1000, 1),
                "speed_threshold_mps": round(POSE_SPEED_MAX_MPS, 2),
            })
        else:
            self._fault.last_accepted_pos_cal = pos_cal.copy()
            self._fault.last_accepted_pose_t_ms = t_motion_ms
        return pose_jump_m, pose_speed_mps

    def _check_orientation_discontinuity(
        self,
        b1_raw: bool,
        raw_rot: Rotation,
        yaw_delta_deg: float,
        pose_valid: bool,
        is_stale: bool,
        t_motion_ms: float,
        orient_frozen: bool,
    ) -> tuple[float, float, float, float]:
        """Reject implausible orientation jumps while B1 is held.

        The shoulder-pan command consumes yaw_delta directly, so a WebXR yaw
        relocalization is just as dangerous as a position relocalization. We
        check both reported yaw_delta continuity and full raw-rotation
        continuity. The gimbal guard's frozen region is excluded because yaw is
        intentionally held there and re-anchored on release.

        t_motion_ms is the BROWSER SEND timestamp of the pose (same as in
        _check_pose_discontinuity). Chrome's WT queue occasionally delivers
        two datagrams within 1-2 ms after a brief network stall; using
        receive-time made a normal 6 deg yaw between frames explode to
        3000+ deg/s and false-latch an orientation fault. Browser send-time
        reflects actual physical rotation timing.
        """
        if not b1_raw:
            return 0.0, 0.0, 0.0, 0.0
        if (not pose_valid or is_stale or self._fault.latched or orient_frozen):
            return 0.0, 0.0, 0.0, 0.0
        if (self._fault.last_accepted_yaw_delta_deg is None
                or self._fault.last_accepted_rot is None
                or self._fault.last_accepted_orient_t_ms is None):
            self._fault.last_accepted_yaw_delta_deg = yaw_delta_deg
            self._fault.last_accepted_rot = raw_rot
            self._fault.last_accepted_orient_t_ms = t_motion_ms
            return 0.0, 0.0, 0.0, 0.0

        yaw_jump_deg = abs(
            (yaw_delta_deg - self._fault.last_accepted_yaw_delta_deg + 180.0) % 360.0
            - 180.0
        )
        rot_jump_deg = self._rotation_step_deg(self._fault.last_accepted_rot, raw_rot)
        dt_s = max((t_motion_ms - self._fault.last_accepted_orient_t_ms) / 1000.0, 1e-6)
        yaw_speed_dps = yaw_jump_deg / dt_s
        rot_speed_dps = rot_jump_deg / dt_s

        yaw_step_fault = (
            ORIENT_YAW_JUMP_MAX_DEG > 0.0
            and yaw_jump_deg > ORIENT_YAW_JUMP_MAX_DEG
        )
        yaw_speed_fault = (
            ORIENT_YAW_MAX_SPEED_DPS > 0.0
            and yaw_speed_dps > ORIENT_YAW_MAX_SPEED_DPS
        )
        rot_step_fault = (
            ORIENT_ROT_JUMP_MAX_DEG > 0.0
            and rot_jump_deg > ORIENT_ROT_JUMP_MAX_DEG
        )
        rot_speed_fault = (
            ORIENT_ROT_MAX_SPEED_DPS > 0.0
            and rot_speed_dps > ORIENT_ROT_MAX_SPEED_DPS
        )
        if yaw_step_fault or yaw_speed_fault or rot_step_fault or rot_speed_fault:
            reasons = []
            if yaw_step_fault:
                reasons.append(
                    f"yaw step {yaw_jump_deg:.0f}deg > {ORIENT_YAW_JUMP_MAX_DEG:.0f}deg"
                )
            if yaw_speed_fault:
                reasons.append(
                    f"yaw speed {yaw_speed_dps:.0f}deg/s > {ORIENT_YAW_MAX_SPEED_DPS:.0f}deg/s"
                )
            if rot_step_fault:
                reasons.append(
                    f"rot step {rot_jump_deg:.0f}deg > {ORIENT_ROT_JUMP_MAX_DEG:.0f}deg"
                )
            if rot_speed_fault:
                reasons.append(
                    f"rot speed {rot_speed_dps:.0f}deg/s > {ORIENT_ROT_MAX_SPEED_DPS:.0f}deg/s"
                )
            print(
                "[browser_phone] orientation discontinuity while B1 held: "
                f"{', '.join(reasons)}; release and reengage B1 to resume"
            )
            self._latch_fault("orientation", {
                "reasons": reasons,
                "yaw_jump_deg": round(yaw_jump_deg, 1),
                "yaw_speed_dps": round(yaw_speed_dps, 1),
                "rot_jump_deg": round(rot_jump_deg, 1),
                "rot_speed_dps": round(rot_speed_dps, 1),
                "yaw_step_threshold_deg": round(ORIENT_YAW_JUMP_MAX_DEG, 1),
                "yaw_speed_threshold_dps": round(ORIENT_YAW_MAX_SPEED_DPS, 1),
                "rot_step_threshold_deg": round(ORIENT_ROT_JUMP_MAX_DEG, 1),
                "rot_speed_threshold_dps": round(ORIENT_ROT_MAX_SPEED_DPS, 1),
            })
        else:
            self._fault.last_accepted_yaw_delta_deg = yaw_delta_deg
            self._fault.last_accepted_rot = raw_rot
            self._fault.last_accepted_orient_t_ms = t_motion_ms
        return rot_jump_deg, rot_speed_dps, yaw_jump_deg, yaw_speed_dps

    @staticmethod
    def _remap_phone_to_robot_frame(pos_cal: np.ndarray) -> np.ndarray:
        """Axis remap WebXR -> robot operator frame.

        WebXR local frame (phone held screen-toward-operator): +X right, +Y up,
        +Z toward operator (-Z forward/away). Downstream wants:
            pos[0]=lateral, pos[1]=back (reach = -pos[1] = forward), pos[2]=up.
        Map phone (X,Y,Z) -> robot (-X, Z, Y). The old (negate X & Y only) left
        reach<-up and up<-back, so retracting the phone made the wrist climb.
        """
        return np.array([-pos_cal[0], pos_cal[2], pos_cal[1]], dtype=float)

    def _compute_phone_roll(self, raw_rot: Rotation, cur_fwd: np.ndarray) -> float:
        """Phone twist about the viewing/forward axis, gravity-referenced and
        decoupled from yaw/pitch (tilting the screen to see it doesn't read as
        roll). 0 = screen upright. Degenerate when phone points straight up/down
        (rare in use) -> hold last value. Feeds wrist_roll."""
        ref_up = np.array([0.0, 1.0, 0.0]) - np.dot([0.0, 1.0, 0.0], cur_fwd) * cur_fwd
        n = float(np.linalg.norm(ref_up))
        if n <= 1e-3:
            return self._phone_roll_deg
        ref_up /= n
        ph_up = raw_rot.apply([0.0, 1.0, 0.0])
        return float(np.degrees(np.arctan2(
            float(np.dot(np.cross(ref_up, ph_up), cur_fwd)),
            float(np.dot(ref_up, ph_up)))))

    async def _loop_lag_probe(self) -> None:
        """Detect asyncio-loop blocking. We schedule a wake every 100 ms;
        if the actual wake is more than LAG_THRESHOLD_MS later than intended,
        the loop was busy (the typical cause of pose-stream stalls -- pose
        msgs sit in the kernel UDP buffer until the loop is free, then drain
        in a sub-ms burst). Records 'loop_lag' events only on incidents to
        avoid flooding.
        """
        PROBE_INTERVAL_S = 0.1
        LAG_THRESHOLD_MS = 50.0
        loop = asyncio.get_event_loop()
        while True:
            intended_wake = loop.time() + PROBE_INTERVAL_S
            try:
                await asyncio.sleep(PROBE_INTERVAL_S)
            except asyncio.CancelledError:
                return
            actual_wake = loop.time()
            lag_ms = (actual_wake - intended_wake) * 1000.0
            if lag_ms > LAG_THRESHOLD_MS:
                rec = self._session_recorder
                if rec is not None:
                    rec.record_event("loop_lag", {"lag_ms": round(lag_ms, 1)})

    async def _relay_wt_arm_loop(self) -> None:
        """Arm-side WebTransport datagram client for the session relay."""
        url = _session_relay_wt_url("arm")
        if not url:
            return
        backoff_s = 0.5
        while True:
            try:
                try:
                    from shared.webtransport import connect_webtransport_datagrams
                except ModuleNotFoundError:
                    _allow_system_dist_packages_for_aioquic()
                    from shared.webtransport import connect_webtransport_datagrams

                async with connect_webtransport_datagrams(url) as wt:
                    backoff_s = 0.5
                    print("[browser_phone] session relay webtransport connected")
                    rec = self._session_recorder
                    if rec is not None:
                        rec.record_event("relay_connected", {
                            "session": SESSION_RELAY_SESSION,
                            "transport": "webtransport",
                            "url": url,
                        })

                    async def send_echo(echo: dict) -> None:
                        payload = json.dumps(echo, separators=(",", ":")).encode("utf-8")
                        wt.send(payload)

                    # Receive timeout: if no datagram (incl. server keepalive)
                    # for WT_RECV_TIMEOUT_S, treat the connection as stuck
                    # (one-way QUIC zombie) and force a reconnect.
                    WT_RECV_TIMEOUT_S = 10.0
                    while True:
                        try:
                            payload = await asyncio.wait_for(wt.recv(), timeout=WT_RECV_TIMEOUT_S)
                        except asyncio.TimeoutError:
                            raise RuntimeError(
                                f"no webtransport datagram for {WT_RECV_TIMEOUT_S:.0f}s -- forcing reconnect"
                            )
                        if payload is None:
                            break
                        try:
                            data = json.loads(payload.decode("utf-8"))
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            continue
                        if isinstance(data, dict):
                            if data.get("type") == "relay_keepalive":
                                continue
                            await self._handle_relay_data(data, send_echo)
            except asyncio.CancelledError:
                return
            except Exception as e:  # noqa: BLE001
                print(f"[browser_phone] session relay webtransport disconnected: {e}")
                rec = self._session_recorder
                if rec is not None:
                    rec.record_event("relay_disconnected", {
                        "transport": "webtransport",
                        "error": repr(e),
                    })
            await asyncio.sleep(backoff_s)
            backoff_s = min(backoff_s * 2.0, 10.0)

    async def _handle_relay_data(self, data: dict, send_echo) -> None:
        msg_type = data.get("type")
        if msg_type == "relay_welcome":
            print(
                "[browser_phone] session relay welcome "
                f"session={data.get('session')} role={data.get('role')} "
                f"transport={data.get('transport', 'ws')}"
            )
            return
        if "seq" not in data:
            return

        epoch = data.get("_relay_epoch")
        try:
            epoch_int = int(epoch)
        except (TypeError, ValueError):
            epoch_int = None
        if epoch_int is not None:
            if self._relay_epoch_seen is not None and epoch_int != self._relay_epoch_seen:
                self._require_control_reacquire("relay_reconnect")
            self._relay_epoch_seen = epoch_int

        # Relay-pose sessions may begin before a video/control offer opens the
        # recorder. Use the relay session name rather than leaving the run
        # unrecorded. If an authenticated video offer already opened a recorder,
        # this is a no-op.
        self._ensure_session_recorder_name(f"relay_{SESSION_RELAY_SESSION}")
        echo = self._ingest_pose_msg(data, is_active=True)
        if echo is not None:
            if epoch_int is not None:
                echo["_relay_epoch"] = epoch_int
            try:
                await send_echo(echo)
            except Exception:
                pass

    def _snapshot_control_arrival_stats(self) -> dict | None:
        """Read all window counters into a dict AND reset them, atomically
        with respect to other loop callbacks (both this and _handle_relay_data
        run on the same asyncio thread). Returns the dict payload ready for
        record_event, or None if no recorder is attached.

        Split from record_event to isolate the fast counter I/O (this method)
        from the potentially slow queue.put on the recorder proxy (which
        occasionally spikes ~250 ms from mp.Queue semaphore contention or a
        GC pause). Caller runs record_event in an executor -- see task #41."""
        rec = self._session_recorder
        if rec is None:
            # Still have to reset counters so the next window starts fresh
            self._reset_dc_window()
            return None

        def avg(total: float, count: int) -> float:
            return round(total / count, 1) if count else 0.0

        gate = self._operator_gate_status()
        payload = {
            "count_in_window": self._dc_window_count,
            "burst_subms_count": self._dc_window_bursts,
            "max_gap_ms": round(self._dc_window_max_gap_ms, 1),
            "out_of_order_drops": self._dc_window_out_of_order_drops,
            "max_source_age_ms": round(self._dc_window_max_source_age_ms, 1),
            "client_skipped": self._dc_window_client_skipped,
            "client_buffered_max": self._dc_window_client_buffered_max,
            "seq_first": self._dc_window_first_seq,
            "seq_last": self._dc_window_last_seq,
            "seq_gap_count": self._dc_window_seq_gap_count,
            "seq_missing": self._dc_window_seq_missing,
            "seq_max_gap": self._dc_window_seq_max_gap,
            "seq_backwards": self._dc_window_seq_backwards,
            "seq_duplicates": self._dc_window_seq_duplicates,
            "phone_to_relay_avg_ms": avg(
                self._dc_window_phone_to_relay_sum_ms,
                self._dc_window_phone_to_relay_count,
            ),
            "phone_to_relay_max_ms": round(self._dc_window_phone_to_relay_max_ms, 1),
            "phone_to_relay_count": self._dc_window_phone_to_relay_count,
            "browser_queue_avg_ms": avg(
                self._dc_window_browser_queue_sum_ms,
                self._dc_window_browser_queue_count,
            ),
            "browser_queue_max_ms": round(self._dc_window_browser_queue_max_ms, 1),
            "browser_queue_count": self._dc_window_browser_queue_count,
            "browser_write_to_edge_avg_ms": avg(
                self._dc_window_browser_write_to_edge_sum_ms,
                self._dc_window_browser_write_to_edge_count,
            ),
            "browser_write_to_edge_max_ms": round(
                self._dc_window_browser_write_to_edge_max_ms, 1
            ),
            "browser_write_to_edge_count": self._dc_window_browser_write_to_edge_count,
            "browser_write_to_relay_avg_ms": avg(
                self._dc_window_browser_write_to_relay_sum_ms,
                self._dc_window_browser_write_to_relay_count,
            ),
            "browser_write_to_relay_max_ms": round(
                self._dc_window_browser_write_to_relay_max_ms, 1
            ),
            "browser_write_to_relay_count": self._dc_window_browser_write_to_relay_count,
            "edge_queue_avg_ms": avg(
                self._dc_window_edge_queue_sum_ms,
                self._dc_window_edge_queue_count,
            ),
            "edge_queue_max_ms": round(self._dc_window_edge_queue_max_ms, 1),
            "edge_queue_count": self._dc_window_edge_queue_count,
            "edge_to_relay_avg_ms": avg(
                self._dc_window_edge_to_relay_sum_ms,
                self._dc_window_edge_to_relay_count,
            ),
            "edge_to_relay_max_ms": round(self._dc_window_edge_to_relay_max_ms, 1),
            "edge_to_relay_count": self._dc_window_edge_to_relay_count,
            "relay_queue_avg_ms": avg(
                self._dc_window_relay_queue_sum_ms,
                self._dc_window_relay_queue_count,
            ),
            "relay_queue_max_ms": round(self._dc_window_relay_queue_max_ms, 1),
            "relay_queue_count": self._dc_window_relay_queue_count,
            "relay_to_pi_avg_ms": avg(
                self._dc_window_relay_to_pi_sum_ms,
                self._dc_window_relay_to_pi_count,
            ),
            "relay_to_pi_max_ms": round(self._dc_window_relay_to_pi_max_ms, 1),
            "relay_to_pi_count": self._dc_window_relay_to_pi_count,
            "client_ctrl_state": self._dc_last_client_ctrl.get("ctrl_state"),
            "client_ctrl_ready": self._dc_last_client_ctrl.get("ctrl_ready"),
            "client_ctrl_conn_seq": self._dc_last_client_ctrl.get("ctrl_conn_seq"),
            "client_ctrl_reconnect_count": self._dc_last_client_ctrl.get(
                "ctrl_reconnect_count"
            ),
            "client_ctrl_last_reconnect_reason": self._dc_last_client_ctrl.get(
                "ctrl_last_reconnect_reason"
            ),
            "control_reacquire_required": int(self._control_reacquire_required),
            "control_reacquire_reason": self._control_reacquire_reason,
            "operator_gate_state": gate["state"],
            "operator_gate_reason": gate["reason"],
            "out_of_order_drops_total": self._ws_out_of_order_drops,
        }
        self._reset_dc_window()
        return payload

    def _reset_dc_window(self) -> None:
        self._dc_window_count = 0
        self._dc_window_bursts = 0
        self._dc_window_max_gap_ms = 0.0
        self._dc_window_out_of_order_drops = 0
        self._dc_window_max_source_age_ms = 0.0
        self._dc_window_client_skipped = 0
        self._dc_window_client_buffered_max = 0
        self._dc_window_first_seq = None
        self._dc_window_last_seq = None
        self._dc_window_seq_gap_count = 0
        self._dc_window_seq_missing = 0
        self._dc_window_seq_max_gap = 0
        self._dc_window_seq_backwards = 0
        self._dc_window_seq_duplicates = 0
        self._dc_window_phone_to_relay_sum_ms = 0.0
        self._dc_window_phone_to_relay_max_ms = 0.0
        self._dc_window_phone_to_relay_count = 0
        self._dc_window_browser_queue_sum_ms = 0.0
        self._dc_window_browser_queue_max_ms = 0.0
        self._dc_window_browser_queue_count = 0
        self._dc_window_browser_write_to_edge_sum_ms = 0.0
        self._dc_window_browser_write_to_edge_max_ms = 0.0
        self._dc_window_browser_write_to_edge_count = 0
        self._dc_window_browser_write_to_relay_sum_ms = 0.0
        self._dc_window_browser_write_to_relay_max_ms = 0.0
        self._dc_window_browser_write_to_relay_count = 0
        self._dc_window_edge_queue_sum_ms = 0.0
        self._dc_window_edge_queue_max_ms = 0.0
        self._dc_window_edge_queue_count = 0
        self._dc_window_edge_to_relay_sum_ms = 0.0
        self._dc_window_edge_to_relay_max_ms = 0.0
        self._dc_window_edge_to_relay_count = 0
        self._dc_window_relay_queue_sum_ms = 0.0
        self._dc_window_relay_queue_max_ms = 0.0
        self._dc_window_relay_queue_count = 0
        self._dc_window_relay_to_pi_sum_ms = 0.0
        self._dc_window_relay_to_pi_max_ms = 0.0
        self._dc_window_relay_to_pi_count = 0

    async def _relay_stats_loop(self) -> None:
        loop = asyncio.get_event_loop()
        while True:
            try:
                await asyncio.sleep(2.0)
            except asyncio.CancelledError:
                return
            # Snapshot + reset run synchronously in the async loop so the
            # counters can't race with _handle_relay_data. Only the
            # potentially-slow record_event (mp.Queue.put_nowait via the
            # recorder proxy) is offloaded to an executor thread. Task #41:
            # this removes the ~250 ms loop_lag we caught at 09:19 UTC on
            # 2026-07-09.
            payload = self._snapshot_control_arrival_stats()
            rec = self._session_recorder
            if payload is None or rec is None:
                continue
            try:
                await loop.run_in_executor(
                    None, rec.record_event, "relay_pose_stats", payload,
                )
            except Exception as exc:  # noqa: BLE001
                # record_event should not raise; log if it does so we notice.
                print(f"[relay_stats_loop] record_event failed: {exc!r}")

    def _maybe_log_webrtc_stats(self, msg: dict, now_t: float) -> None:
        """Rate-limited (1Hz) WebRTC transport stat log. Grep '[webrtc]' to SEE
        the lag each operator faced."""
        if (
            "wrtc_rtt" not in msg
            and "wrtc_rtt_ms" not in msg
            or now_t - self._last_wrtc_print_t <= 1.0
        ):
            return
        self._last_wrtc_print_t = now_t
        try:
            rtt_ms = float(msg.get("wrtc_rtt", msg.get("wrtc_rtt_ms", 0.0)))
            jbuf_ms = float(msg.get("wrtc_jbuf", msg.get("wrtc_jbuf_ms", 0.0)))
            decode_ms = float(msg.get("wrtc_decode", msg.get("wrtc_decode_ms", 0.0)))
            jitter_ms = float(msg.get("wrtc_jitter", msg.get("wrtc_jitter_ms", 0.0)))
            vfps = float(msg.get("wrtc_fps", 0.0))
            freezes = int(msg.get("wrtc_freezes", 0))
        except (TypeError, ValueError):
            rtt_ms = jbuf_ms = decode_ms = jitter_ms = vfps = 0.0
            freezes = 0
        # Glass-to-glass proxy: half the network RTT + jitter buffer + decode.
        transport_ms = 0.5 * rtt_ms + jbuf_ms + decode_ms
        print(
            f"[webrtc] transport~{transport_ms:.0f}ms "
            f"rtt={rtt_ms:.0f}ms jbuf={jbuf_ms:.0f}ms decode={decode_ms:.0f}ms "
            f"jitter={jitter_ms:.0f}ms fps={vfps:.0f} freezes={freezes} "
            f"via={msg.get('wrtc_via', '?') or '?'}"
        )

    def _maybe_log_phone_debug(self, now_t: float, raw_pos: np.ndarray,
                                pos_cal: np.ndarray, *, enable: bool, b1_raw: bool,
                                pose_valid: bool, pose_age_ms: float,
                                yaw_delta_deg: float, pitch_deg: float,
                                roll_deg: float) -> None:
        if now_t - self._last_debug_print_t <= DEBUG_PRINT_PERIOD_S:
            return
        self._last_debug_print_t = now_t
        raw_delta_mm = (raw_pos - self._calib_pos) * 1000.0
        pos_cal_mm = pos_cal * 1000.0
        print(
            f"[phone] enable={int(enable)} "
            f"b1={int(b1_raw)} pose_valid={int(pose_valid)} "
            f"yaw_d={yaw_delta_deg:+.0f} pitch={pitch_deg:+.0f} roll={roll_deg:+.0f} "
            f"age={pose_age_ms:.0f}ms | "
            f"raw=({raw_pos[0]:+.3f},{raw_pos[1]:+.3f},{raw_pos[2]:+.3f}) "
            f"delta_mm=({raw_delta_mm[0]:+.0f},{raw_delta_mm[1]:+.0f},{raw_delta_mm[2]:+.0f}) "
            f"pos_cal_mm=({pos_cal_mm[0]:+.0f},{pos_cal_mm[1]:+.0f},{pos_cal_mm[2]:+.0f})"
        )

    def _process_pose_state(self, data: dict, t_recv_ms: float) -> dict:
        """Run the gate + recovery + orientation pipeline for ONE incoming
        pose msg, in the order msgs arrived. Returns a dict the IK loop will
        surface via get_action(). Called from _ingest_pose_msg, NOT from the
        IK tick -- the gate processes every msg exactly once.

        Two implications of being on the ingest timeline:
          - dt and step in the discontinuity check are always between two
            CONSECUTIVE arrivals -- no skipping of intermediate msgs because
            the IK loop ticked at a different cadence;
          - is_stale at ingest is always False (this msg just landed). Stale
            detection that the IK loop cares about (no fresh msg for >250ms)
            happens in get_action() against the cached t_recv_ms.
        """
        pose_valid = bool(data.get("pose_valid", True))
        pos_field = data.get("pos")
        if not pose_valid or pos_field is None:
            raw_pos = self._last_raw_pos
        else:
            try:
                raw_pos = np.asarray(pos_field, dtype=float)
                if raw_pos.shape != (3,) or not np.all(np.isfinite(raw_pos)):
                    raise ValueError("bad pos")
                self._last_raw_pos = raw_pos
            except (ValueError, TypeError):
                raw_pos = self._last_raw_pos
        rot_field = data.get("rot_xyzw")
        if not pose_valid or rot_field is None:
            raw_rot = self._last_raw_rot
        else:
            try:
                raw_rot = Rotation.from_quat(rot_field)
                self._last_raw_rot = raw_rot
            except (ValueError, TypeError):
                raw_rot = self._last_raw_rot
        raw_inputs = dict(data.get("raw_inputs", {}))

        if self._last_pose_valid is None:
            self._last_pose_valid = pose_valid
        elif pose_valid != self._last_pose_valid:
            state = "valid" if pose_valid else "INVALID"
            print(f"[browser_phone] pose_valid -> {state}")
            self._last_pose_valid = pose_valid

        # Fresh-on-arrival: is_stale is always False here. Carried through to
        # keep the existing gate signatures unchanged.
        is_stale = False
        b1_raw = bool(data.get("enabled", False))
        control_hold = False
        if self._control_reacquire_required:
            if b1_raw:
                control_hold = True
                self._b1_reengage_required = False
                self._b1_reengage_reason = None
            else:
                reason = self._control_reacquire_reason or "unknown"
                self._control_reacquire_required = False
                self._control_reacquire_reason = None
                self._fault.reset()
                print(f"[browser_phone] control reacquire ready after B1 release "
                      f"(reason={reason})")
                rec = self._session_recorder
                if rec is not None:
                    rec.record_event("control_reacquire_ready", {"reason": reason})
        if control_hold:
            # Do not let the first post-reconnect phone pose act like a live
            # B1-held sample. It may reflect seconds of off-screen hand motion.
            self._fault.reset()
        else:
            self._handle_b1_edges(
                b1_raw, raw_pos, raw_rot, pose_valid, is_stale, raw_inputs)

        # Position uses the YAW-ONLY clutch frame so phone tilt doesn't leak
        # vertical hand motion into EE-reach. Orientation is unused (wrist on
        # buttons). See _remap_phone_to_robot_frame for the axis derivation.
        pos_cal = self._remap_phone_to_robot_frame(
            self._calib_yaw_inv.apply(raw_pos - self._calib_pos))
        # For dt in the fault detectors, prefer the browser send-time (msg
        # "t") over the Pi receive-time. WT delivery bursts occasionally
        # bunch two datagrams within a few ms at the Pi even though the
        # browser sent them 30-40 ms apart -- receive-time dt would then
        # false-trigger a speed fault. Fall back to receive-time if the
        # message lacks "t" (defensive; every real client sends it).
        _t_client_ms = self._finite_float(data.get("t"))
        t_motion_ms = _t_client_ms if _t_client_ms is not None else t_recv_ms
        pose_jump_m, pose_speed_mps = self._check_pose_discontinuity(
            b1_raw and not control_hold, pos_cal, pose_valid, is_stale, t_motion_ms)

        _cur_fwd = raw_rot.apply([0.0, 0.0, -1.0])
        # Yaw + roll become numerically ill-defined as the phone forward axis
        # approaches vertical (|pitch| -> 90°). Both extractions divide by the
        # horizontal projection of fwd; near zero, tiny IMU noise flips them by
        # 100°+ in one frame, which drove a sudden shoulder_pan + wrist_roll
        # slam in a real session. Gate: if the forward axis is more than ~60°
        # off horizontal (cos(60°) = 0.5), HOLD the previous yaw_delta and roll.
        # Pitch (-> wrist_flex) and position (-> reach/height) keep updating.
        _horiz = float(np.hypot(_cur_fwd[0], _cur_fwd[2]))
        _orient_frozen = _horiz < ORIENT_FREEZE_COS_THRESHOLD
        if _orient_frozen:
            yaw_delta_deg = self._last_yaw_delta_deg
            # self._phone_roll_deg also held at its prior value -- don't update.
        else:
            _cur_yaw = float(np.arctan2(_cur_fwd[0], -_cur_fwd[2]))
            # Guard active->released edge: re-anchor the clutch yaw and the
            # phone-roll offset so the reported yaw_delta and phone_roll come
            # out equal to the values we held during the freeze. Without this,
            # any hand rotation the operator did while the guard was active
            # surfaces in ONE frame, jumping the IK target by ~1m and slamming
            # the arm into a runaway slew (root cause of the 2026-06-05 crash).
            if self._orient_was_frozen:
                self._clutch_yaw = _cur_yaw - float(np.radians(self._last_yaw_delta_deg))
                _raw_roll = self._compute_phone_roll(raw_rot, _cur_fwd)
                self._phone_roll_offset = self._phone_roll_deg - _raw_roll
                print(f"[browser_phone] gimbal guard released -> re-anchored "
                      f"(held yaw_d={self._last_yaw_delta_deg:+.0f}, "
                      f"roll_offset={self._phone_roll_offset:+.0f})")
            _yd = _cur_yaw - self._clutch_yaw
            yaw_delta_deg = float(np.degrees(np.arctan2(np.sin(_yd), np.cos(_yd))))
            self._last_yaw_delta_deg = yaw_delta_deg
            self._phone_roll_deg = (
                self._compute_phone_roll(raw_rot, _cur_fwd) + self._phone_roll_offset
            )
        self._orient_was_frozen = _orient_frozen
        raw_inputs["_yaw_delta_deg"] = yaw_delta_deg
        raw_inputs["_phone_roll_deg"] = self._phone_roll_deg
        # Phone PITCH: elevation of the forward axis. 0 = phone forward is horizontal,
        # +90 = phone forward points straight up. Reading-tilt sits at a small positive
        # value; the consumer absorbs that via a deadband around the clutch baseline.
        self._phone_pitch_deg = float(np.degrees(np.arcsin(np.clip(_cur_fwd[1], -1.0, 1.0))))
        raw_inputs["_phone_pitch_deg"] = self._phone_pitch_deg
        rot_jump_deg, rot_speed_dps, yaw_jump_deg, yaw_speed_dps = (
            self._check_orientation_discontinuity(
                b1_raw and not control_hold, raw_rot, yaw_delta_deg, pose_valid, is_stale,
                t_motion_ms, _orient_frozen)
        )

        # Final "enabled" gates on pose_valid (ARCore tracking). Do NOT gate on
        # is_stale: during a brief delivery gap the _latest_msg is unchanged so the
        # phone delta stays constant on its own and the arm holds in place.
        # Toggling enable=False during stale used to re-latch the arm reference
        # on recovery, producing 200-300mm jumps every gap (operator's
        # accumulated displacement got applied twice).
        enable = (
            b1_raw
            and pose_valid
            and not self._fault.latched
            and not self._b1_reengage_required
            and not control_hold
        )

        # Diagnostics. _pose_age_ms / _pose_stale are filled in get_action()
        # so they reflect the IK tick's notion of "now", not ingest time.
        try:
            raw_inputs["_seq"] = int(data.get("seq"))
        except (TypeError, ValueError):
            raw_inputs["_seq"] = data.get("seq")
        # Browser send-time of the frame that produced this pose. Kept as a
        # jitter-resilient source of truth for downstream round-trip calc:
        # the tracking-feedback echo carries it back so the browser can
        # compute latency against its OWN clock instead of relying on the
        # Pi's wall-clock (which drifts across sessions and doesn't survive
        # NTP resyncs).
        raw_inputs["_t_browser_ms"] = self._finite_float(data.get("t"))
        raw_inputs["_pose_valid"] = int(pose_valid)
        raw_inputs["_pose_jump_m"] = float(pose_jump_m)
        raw_inputs["_pose_speed_mps"] = float(pose_speed_mps)
        raw_inputs["_orient_jump_deg"] = float(rot_jump_deg)
        raw_inputs["_orient_speed_dps"] = float(rot_speed_dps)
        raw_inputs["_yaw_jump_deg"] = float(yaw_jump_deg)
        raw_inputs["_yaw_speed_dps"] = float(yaw_speed_dps)
        raw_inputs["_pose_fault"] = int(self._fault.latched)
        raw_inputs["_b1_reengage_required"] = int(self._b1_reengage_required)
        raw_inputs["_b1_reengage_reason"] = self._b1_reengage_reason
        raw_inputs["_control_reacquire_required"] = int(self._control_reacquire_required)
        raw_inputs["_control_hold"] = int(control_hold)
        raw_inputs["_ws_msg_count"] = int(self._n_msgs + 1)
        source_age_ms = data.get("_source_age_ms")
        if source_age_ms is not None:
            try:
                raw_inputs["_source_age_ms"] = float(source_age_ms)
            except (TypeError, ValueError):
                pass
        for src, dst in (
            ("t_browser_write_ms", "_t_browser_write_ms"),
            ("_browser_queue_ms", "_browser_queue_ms"),
            ("_browser_write_to_edge_ms", "_browser_write_to_edge_ms"),
            ("_browser_write_to_relay_ms", "_browser_write_to_relay_ms"),
            ("_edge_recv_ms", "_edge_recv_ms"),
            ("_edge_send_ms", "_edge_send_ms"),
            ("_edge_q_ms", "_edge_q_ms"),
            ("_edge_to_relay_ms", "_edge_to_relay_ms"),
            ("_phone_to_relay_ms", "_phone_to_relay_ms"),
            ("_relay_queue_age_ms", "_relay_queue_age_ms"),
            ("_relay_to_pi_ms", "_relay_to_pi_ms"),
            ("_relay_total_age_ms", "_relay_total_age_ms"),
            ("ws_buffered", "_client_buffered"),
            ("ws_skipped", "_client_skipped"),
            ("ctrl_state", "_client_ctrl_state"),
            ("ctrl_ready", "_client_ctrl_ready"),
            ("ctrl_conn_seq", "_client_ctrl_conn_seq"),
            ("ctrl_reconnect_count", "_client_ctrl_reconnect_count"),
            ("ctrl_last_reconnect_reason", "_client_ctrl_last_reconnect_reason"),
        ):
            if src in data:
                raw_inputs[dst] = data.get(src)
        raw_inputs["_ws_out_of_order_drops"] = int(self._ws_out_of_order_drops)

        now_t = time.time()
        self._maybe_log_webrtc_stats(data, now_t)
        self._maybe_log_phone_debug(
            now_t, raw_pos, pos_cal,
            enable=enable, b1_raw=b1_raw, pose_valid=pose_valid,
            pose_age_ms=0.0,  # at-ingest age is zero
            yaw_delta_deg=raw_inputs.get("_yaw_delta_deg", 0.0),
            pitch_deg=self._phone_pitch_deg,
            roll_deg=self._phone_roll_deg,
        )

        return {
            "phone.pos": pos_cal,
            "phone.raw_inputs": raw_inputs,
            "phone.enabled": enable,
            "_t_recv_ms": t_recv_ms,
            "_pose_valid": pose_valid,
        }

    def _process_leader_state(self, data: dict, t_recv_ms: float) -> dict:
        """Validate a network leader frame and normalize it for main-process IPC."""
        positions = parse_leader_positions(data)
        valid = positions is not None
        raw_inputs = {
            "_seq": data.get("seq"),
            "_control_source": "leader_arm",
            "_source_age_ms": data.get("_source_age_ms"),
            "_relay_epoch": data.get("_relay_epoch"),
        }
        if not valid:
            print("[leader] rejected malformed joint frame")
        return {
            # Keep the existing recorder/IPC envelope backward compatible.
            "phone.pos": np.zeros(3),
            "phone.raw_inputs": raw_inputs,
            "phone.enabled": False,
            "control.source": "leader_arm",
            "leader.positions": positions,
            "leader.enabled": bool(data.get("enabled", False)) and valid,
            "leader.source_id": str(data.get("source_id") or data.get("page_id") or "leader"),
            "_t_recv_ms": t_recv_ms,
            "_pose_valid": valid,
        }

    def get_action(self) -> dict:
        self._drain_action_ipc()
        with self._state_lock:
            cached = self._latest_action_state
        if cached is None:
            return {
                "phone.pos": np.zeros(3),
                "phone.raw_inputs": {},
                "phone.enabled": False,
                "control.source": "none",
            }
        # Recompute age against IK-tick "now" so the IK loop sees current
        # staleness, not at-ingest staleness. We do NOT override enable here:
        # disabling on the stale tick is a clutch edge downstream. Instead,
        # keep repeating the cached phone.pos so the arm holds the last target.
        # Relay reconnects set _control_reacquire_required on the ingest path.
        now_ms = time.time() * 1000.0
        pose_age_ms = now_ms - float(cached["_t_recv_ms"])
        is_stale = pose_age_ms > STALE_POSE_TIMEOUT_MS
        if is_stale and not self._stale_logged:
            print(f"[browser_phone] pose STALE ({pose_age_ms:.0f}ms since last msg) "
                  f"-> arm hold")
            self._stale_started_ms = now_ms
            details = self._pose_link_event_details(cached, pose_age_ms)
            _record_session_event("pose_stale", details)
            self._record_incident("incident_pose_stale", details)
            self._stale_logged = True
        elif not is_stale and self._stale_logged:
            print("[browser_phone] pose fresh again")
            details = self._pose_link_event_details(cached, pose_age_ms)
            if self._stale_started_ms is not None:
                details["stale_duration_ms"] = round(now_ms - self._stale_started_ms, 1)
            _record_session_event("pose_fresh", details)
            self._stale_started_ms = None
            self._stale_logged = False
        raw_inputs = dict(cached["phone.raw_inputs"])
        raw_inputs["_pose_age_ms"] = float(pose_age_ms)
        raw_inputs["_pose_stale"] = int(is_stale)
        result = {
            "phone.pos": cached["phone.pos"],
            "phone.raw_inputs": raw_inputs,
            "phone.enabled": cached["phone.enabled"],
            "control.source": cached.get("control.source", "phone"),
        }
        if result["control.source"] == "leader_arm":
            result["leader.positions"] = cached.get("leader.positions")
            # A stale leader stream is a hard hold. The mapper resets on the
            # stale marker, so recovery re-anchors instead of catching up.
            result["leader.enabled"] = bool(cached.get("leader.enabled", False)) and not is_stale
            result["leader.source_id"] = cached.get("leader.source_id")
        return result


    def send_feedback(self, feedback: dict[str, float]) -> None:
        q = self._command_ipc_queue
        if q is None:
            return
        try:
            payload = dict(feedback)
        except (TypeError, ValueError):
            return
        try:
            q.put_nowait(("robot_feedback", payload))
        except (_queue.Full, BrokenPipeError, EOFError, OSError):
            pass

    def disconnect(self) -> None:
        global _ACTIVE_RECORDER
        active_recorder = _ACTIVE_RECORDER
        _ACTIVE_RECORDER = None
        close_active = getattr(active_recorder, "close", None)
        if callable(close_active):
            try:
                close_active()
            except Exception as e:  # noqa: BLE001
                print(f"[browser_phone] recorder proxy close failed: {e}")
        if self._command_ipc_queue is not None:
            try:
                self._command_ipc_queue.put_nowait(("shutdown", None))
            except (_queue.Full, BrokenPipeError, EOFError, OSError):
                pass
        if self._server_process is not None:
            self._server_process.join(timeout=5.0)
            if self._server_process.is_alive():
                print("[browser_phone] server process did not stop; terminating")
                self._server_process.terminate()
                self._server_process.join(timeout=3.0)
            self._server_process = None
        self._stop_trajectory_writer_process()

    # --- Internal: aiohttp server ---------------------------------------
    def _run_server(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        # Diagnostic: identify what blocks the asyncio loop on Pi (task #41).
        # loop_lag events tell us WHEN we stall but not WHY. Debug mode + a
        # low slow_callback_duration makes asyncio print a WARNING to the
        # 'asyncio' logger for every callback that runs longer than the
        # threshold, including its repr (which contains the coroutine/handle
        # name and source location). Zero overhead when the loop is healthy.
        if _os.environ.get("PHONE_ARM_ASYNCIO_DEBUG", "1") != "0":
            loop.set_debug(True)
            try:
                loop.slow_callback_duration = float(
                    _os.environ.get("PHONE_ARM_SLOW_CALLBACK_MS", "20")) / 1000.0
            except (TypeError, ValueError):
                loop.slow_callback_duration = 0.020
            # Route asyncio warnings so we can see them in teleop.log. Root
            # logger might not be configured for asyncio's default WARNING.
            import logging
            aio_logger = logging.getLogger("asyncio")
            if not aio_logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter(
                    "[asyncio-slow] %(asctime)s %(message)s",
                    datefmt="%H:%M:%S"))
                aio_logger.addHandler(handler)
                aio_logger.setLevel(logging.WARNING)
        try:
            loop.run_until_complete(self._start_server())
            loop.run_forever()
        except Exception as e:  # noqa: BLE001
            print(f"[browser_phone] server crashed: {e}")
            self._notify_process_status("error", repr(e))
            self._ready_event.set()
        finally:
            pending = [t for t in asyncio.all_tasks(loop) if not t.done()]
            for task in pending:
                task.cancel()
            if pending:
                loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
            loop.close()

    def _notify_process_status(self, kind: str, payload=None) -> None:
        q = self._status_ipc_queue
        if q is None:
            return
        try:
            q.put_nowait((kind, payload))
        except (_queue.Full, BrokenPipeError, EOFError, OSError):
            pass

    def _notify_trajectory_session(self, session_dir: str | Path) -> None:
        q = self._trajectory_ipc_queue
        if q is None:
            return
        payload = str(session_dir)
        try:
            q.put_nowait(("open", payload))
        except _queue.Full:
            try:
                q.put(("open", payload), timeout=5.0)
            except Exception as e:  # noqa: BLE001
                print(f"[trajectory-recorder] session open signal failed: {e}")
        except (BrokenPipeError, EOFError, OSError) as e:
            print(f"[trajectory-recorder] session open signal failed: {e}")

    def _start_ipc_command_thread(self) -> None:
        if self._command_ipc_queue is None or self._ipc_command_thread is not None:
            return
        self._ipc_command_thread = threading.Thread(
            target=self._ipc_command_loop,
            name="BrowserPhoneIPCCommands",
            daemon=True,
        )
        self._ipc_command_thread.start()

    def _ipc_command_loop(self) -> None:
        q = self._command_ipc_queue
        if q is None:
            return
        while True:
            try:
                item = q.get()
            except (EOFError, OSError):
                return
            if not item:
                continue
            kind = item[0]
            payload = item[1] if len(item) > 1 else None
            if kind == "trajectory":
                tq = self._trajectory_ipc_queue
                if tq is not None:
                    try:
                        tq.put_nowait(("trajectory", payload))
                    except _queue.Full:
                        tq.put(("trajectory", payload))
                    except (BrokenPipeError, EOFError, OSError):
                        pass
                else:
                    rec = self._session_recorder
                    if rec is not None:
                        rec.record_trajectory(payload)
            elif kind == "event":
                rec = self._session_recorder
                if rec is not None:
                    event_type, details = payload
                    rec.record_event(event_type, details)
            elif kind == "robot_feedback":
                self._store_robot_feedback(payload)
            elif kind == "reload_tokens":
                if self._loop is not None and self._loop.is_running():
                    self._loop.call_soon_threadsafe(self._load_tokens)
            elif kind == "shutdown":
                if self._loop is not None and self._loop.is_running():
                    asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)
                return

    # --- Token auth -----------------------------------------------------
    def _load_tokens(self) -> None:
        """Read TOKENS_FILE and refresh in-memory token list. Called at startup
        and on SIGHUP. If the file is absent the server runs unauthenticated."""
        if not TOKENS_FILE.exists():
            self._tokens = []
            self._auth_enabled = False
            print(f"[browser_phone] auth DISABLED (no {TOKENS_FILE}) -- any "
                  "connection is accepted; use follower/mint_token.py to lock it down")
            return
        try:
            with open(TOKENS_FILE) as f:
                data = json.load(f)
            if not isinstance(data, list):
                raise ValueError("tokens file is not a JSON array")
        except Exception as e:  # noqa: BLE001
            print(f"[browser_phone] failed to load {TOKENS_FILE}: {e} -- "
                  "auth stays in current state")
            return
        self._tokens = data
        self._auth_enabled = True
        now = time.time()
        active = sum(1 for t in data
                     if t.get("expires_at") is None or t.get("expires_at") > now)
        print(f"[browser_phone] auth ENABLED: {active}/{len(data)} tokens active")

    def _token_name_for(self, token: str | None) -> str | None:
        """Return the named identity for `token`, or None if invalid/expired."""
        if not token:
            return None
        now = time.time()
        for entry in self._tokens:
            if entry.get("value") != token:
                continue
            exp = entry.get("expires_at")
            if exp is not None and exp <= now:
                return None
            return entry.get("name") or "(unnamed)"
        return None

    # Paths that bypass token auth. The landing page and static assets must be
    # reachable without a token so the client-side JS can run and read ?t=
    # from the URL on first load. The interesting endpoints (/ws, /webrtc/*,
    # /control/*, /stats) are still gated -- those are what actually grant
    # control.
    _AUTH_PUBLIC_PREFIXES = ("/healthz", "/static/", "/test_event")
    _AUTH_PUBLIC_EXACT = {"/"}

    @web.middleware
    async def _auth_middleware(self, request: web.Request, handler):
        if request.path in self._AUTH_PUBLIC_EXACT:
            return await handler(request)
        if any(request.path.startswith(p) for p in self._AUTH_PUBLIC_PREFIXES):
            return await handler(request)
        if not self._auth_enabled:
            return await handler(request)
        token = request.query.get("t")
        name = self._token_name_for(token)
        if name is None:
            return web.Response(status=401, text="invalid or missing token\n")
        return await handler(request)

    async def _start_server(self) -> None:
        cert, key = _ensure_cert()
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(cert, key)

        # Auth: load tokens at startup. The SIGHUP handler is registered from
        # the MAIN thread (in connect()) because signal.signal / asyncio's
        # add_signal_handler only work there; this method runs in the daemon
        # server thread and can't register one of its own.
        self._load_tokens()

        app = web.Application(middlewares=[self._auth_middleware])
        app.router.add_get("/", self._serve_index)
        if not TURN_URL and not ALLOW_NO_TURN_FOR_LOCAL_REPRO:
            raise RuntimeError(
                "WHEP video is relay-only but no TURN server is configured. "
                "follower/run.sh supplies PHONE_ARM_TURN_URL/USER/PW from ~/.turn_secret."
            )
        if ((not MEDIAMTX_WHEP_URL or not MEDIAMTX_PLAY_TOKEN)
                and not ALLOW_NO_TURN_FOR_LOCAL_REPRO):
            raise RuntimeError(
                "WHEP video is enabled in the browser but MediaMTX config is missing. "
                "follower/run.sh supplies PHONE_ARM_MEDIAMTX_WHEP_URL/PLAY_TOKEN."
            )
        app.router.add_get("/webrtc/config", self._webrtc_config)
        app.router.add_get("/leader/config", self._leader_config)
        app.router.add_post("/control/release", self._control_release)
        print(f"[browser_phone] robot_video=WHEP(MediaMTX relay-only) "
              f"device={VIDEO_DEVICE} "
              f"cameras={[k for k, _l, _d in _present_cameras()]} "
              f"turn={TURN_URL} control=relay-webtransport")
        app.router.add_get("/healthz", lambda _r: web.Response(text="ok"))
        app.router.add_get("/stats", self._serve_stats)
        app.router.add_get("/static/app.js", self._serve_app_js)
        app.router.add_static("/static", STATIC_DIR, show_index=False)
        app.router.add_post("/test_event", self._test_event_handler)

        self._runner = web.AppRunner(app, access_log=None)
        await self._runner.setup()
        # Loopback ONLY: the phone reaches us exclusively through the VPS bore
        # tunnel (bore connects to localhost:8443 on this Pi). Nothing is served
        # directly on the LAN -- all phone traffic goes through the VPS.
        self._site = web.TCPSite(self._runner, host="127.0.0.1", port=self.port,
                                 ssl_context=ssl_ctx)
        await self._site.start()
        # One global asyncio-loop-lag probe; logs incidents only.
        asyncio.ensure_future(self._loop_lag_probe())
        # Arm-side WebTransport client -- the only control transport.
        print(
            "[browser_phone] session relay (WT) enabled "
            f"session={SESSION_RELAY_SESSION} url={SESSION_RELAY_WT_URL}"
        )
        self._relay_task = asyncio.ensure_future(self._relay_wt_arm_loop())
        self._relay_stats_task = asyncio.ensure_future(self._relay_stats_loop())
        self._start_ipc_command_thread()
        self._ready_event.set()
        self._notify_process_status("ready", None)

    async def _serve_stats(self, _request: web.Request) -> web.Response:
        with self._state_lock:
            last = dict(self._latest_msg) if self._latest_msg else None
            n = self._n_msgs
            owner = self._control_owner_page_id
            owner_name = self._control_owner_name
            owner_seen_age_ms = (
                time.time() * 1000.0 - self._control_owner_seen_ms
                if owner is not None and self._control_owner_seen_ms > 0
                else None
            )
        out = {
            "control_msgs_total": n,
            "control_last_msg": last,
            "robot_feedback": self._robot_feedback_snapshot(),
            "control_owner": {
                "page_id": owner,
                "token_name": owner_name,
                "seen_age_ms": (
                    round(owner_seen_age_ms, 1)
                    if owner_seen_age_ms is not None else None
                ),
                "timeout_s": CONTROL_OWNER_TIMEOUT_S,
            },
        }
        return web.json_response(out)

    async def _test_event_handler(self, request: web.Request) -> web.Response:
        try:
            payload = await request.json()
        except Exception:
            return web.json_response({"error": "bad json"}, status=400)
        self._ensure_session_recorder(request.query.get("t"))
        payload.setdefault("server_recv_ms", round(time.time() * 1000.0, 3))
        payload.setdefault("http_user_agent", request.headers.get("User-Agent", ""))
        payload.setdefault("http_remote", request.remote)
        payload.setdefault("http_x_forwarded_for", request.headers.get("X-Forwarded-For", ""))
        kind = str(payload.get("kind", "browser_event"))
        _record_session_event(f"browser_{kind}", payload)
        if kind in {"pose_source_stats", "control_debug"}:
            self._record_browser_debug_payload(payload)
        return web.json_response({"ok": True})

    # --- Robot camera video ---------------------------------------------
    def _ice_servers_json(self) -> list[dict]:
        # Relay-only: TURN is the SOLE ICE server. No STUN -- a srflx candidate
        # would only enable a direct (non-VPS) path, which we deliberately
        # forbid (iceTransportPolicy=relay on the client drops it anyway).
        # Prefer failing fast over silently falling back to TURN-over-TCP,
        # which can add head-of-line blocking and high latency.
        if not TURN_URLS_PHONE:
            return []
        return [{"urls": TURN_URLS_PHONE, "username": TURN_USER, "credential": TURN_PW}]

    @staticmethod
    def _truthy_query(value: str | None) -> bool:
        return str(value or "").strip().lower() in {"1", "true", "yes", "on", "controller"}

    @staticmethod
    def _request_page_id(request: web.Request) -> str | None:
        raw = (
            request.query.get("page_id")
            or request.query.get("app_page_id")
            or request.headers.get("X-Phone-Arm-Page-Id")
            or ""
        )
        page_id = str(raw).strip()
        if not page_id:
            return None
        return page_id[:80]

    def _control_owner_expired_locked(self, now_ms: float) -> bool:
        return (
            self._control_owner_page_id is not None
            and now_ms - self._control_owner_seen_ms
            > CONTROL_OWNER_TIMEOUT_S * 1000.0
        )

    def _note_control_owner_seen(self, page_id: str | None) -> None:
        if not page_id:
            return
        now_ms = time.time() * 1000.0
        with self._state_lock:
            if self._control_owner_page_id == page_id:
                self._control_owner_seen_ms = now_ms

    def _claim_control_owner(self, request: web.Request) -> tuple[bool, dict]:
        now_ms = time.time() * 1000.0
        page_id = self._request_page_id(request)
        token_name = (
            self._token_name_for(request.query.get("t"))
            if self._auth_enabled
            else "no_auth"
        )
        if not page_id:
            return False, {
                "controlRole": "viewer",
                "controlDeniedReason": "missing_page_id",
            }
        with self._state_lock:
            if self._control_owner_expired_locked(now_ms):
                expired_page = self._control_owner_page_id
                self._control_owner_page_id = None
                self._control_owner_name = None
                self._control_owner_claim_ms = 0.0
                self._control_owner_seen_ms = 0.0
                _record_session_event("control_owner_expired", {
                    "page_id": expired_page,
                    "timeout_s": CONTROL_OWNER_TIMEOUT_S,
                })
            if (
                self._control_owner_page_id is None
                or self._control_owner_page_id == page_id
            ):
                fresh_claim = self._control_owner_page_id is None
                self._control_owner_page_id = page_id
                self._control_owner_name = token_name
                if fresh_claim:
                    self._control_owner_claim_ms = now_ms
                self._control_owner_seen_ms = now_ms
                owner_age_ms = now_ms - self._control_owner_claim_ms
                if fresh_claim:
                    _record_session_event("control_owner_claimed", {
                        "page_id": page_id,
                        "token_name": token_name,
                    })
                return True, {
                    "controlRole": "controller",
                    "controlOwnerPageId": page_id,
                    "controlOwnerAgeMs": round(owner_age_ms, 1),
                    "controlOwnerTimeoutS": CONTROL_OWNER_TIMEOUT_S,
                }
            owner_age_ms = now_ms - self._control_owner_claim_ms
            owner_seen_age_ms = now_ms - self._control_owner_seen_ms
            details = {
                "controlRole": "viewer",
                "controlDeniedReason": "controller_active",
                "controlOwnerPageId": self._control_owner_page_id,
                "controlOwnerAgeMs": round(owner_age_ms, 1),
                "controlOwnerSeenAgeMs": round(owner_seen_age_ms, 1),
                "controlOwnerTimeoutS": CONTROL_OWNER_TIMEOUT_S,
            }
            _record_session_event("control_owner_denied", {
                "page_id": page_id,
                "token_name": token_name,
                **details,
            })
            return False, details

    async def _webrtc_config(self, request: web.Request) -> web.Response:
        wants_control = self._truthy_query(request.query.get("want_control"))
        cfg = {
            "iceServers": self._ice_servers_json(),
            "iceTransportPolicy": "relay",
            "controlRole": "viewer",
            "controlTransport": "viewer",
        }
        if wants_control:
            granted, claim = self._claim_control_owner(request)
            cfg.update(claim)
            if granted:
                cfg["controlTransport"] = "relay-webtransport"
                cfg["sessionRelayWtUrl"] = _session_relay_wt_url("phone")
                cfg["sessionRelaySession"] = SESSION_RELAY_SESSION
        if MEDIAMTX_WHEP_URL and MEDIAMTX_PLAY_TOKEN:
            cfg["mediamtxWhepUrl"] = MEDIAMTX_WHEP_URL
            cfg["mediamtxPlayToken"] = MEDIAMTX_PLAY_TOKEN
        return web.json_response(cfg, headers={"Cache-Control": "no-store"})

    async def _leader_config(self, _request: web.Request) -> web.Response:
        """Return the one-to-one source command for the current relay session.

        This endpoint is token protected by the normal middleware. The command
        consequently contains the phone-role relay credential and must not be
        logged or exposed from a public, unauthenticated page.
        """
        # A physical leader beside the robot should use the canonical relay
        # directly, rather than the geographically remote phone forwarder.
        relay_url = _session_relay_wt_url(
            "phone", base_override=SESSION_RELAY_WT_URL
        )
        command = (
            "cd ~/dev/phone_arm && ./controllers/leader_arm/run.sh --url "
            + shlex.quote(relay_url)
            + " --exclude-serial "
            + shlex.quote(ROBOT_ADAPTER_SERIAL)
        )
        return web.json_response(
            {
                "mode": "one_to_one",
                "session": SESSION_RELAY_SESSION,
                "command": command,
            },
            headers={"Cache-Control": "no-store"},
        )

    async def _control_release(self, request: web.Request) -> web.Response:
        page_id = self._request_page_id(request)
        if page_id is None:
            try:
                payload = await request.json()
            except Exception:
                payload = {}
            page_id = str(payload.get("page_id") or payload.get("app_page_id") or "").strip()[:80]
        released = False
        with self._state_lock:
            if page_id and self._control_owner_page_id == page_id:
                released = True
                self._control_owner_page_id = None
                self._control_owner_name = None
                self._control_owner_claim_ms = 0.0
                self._control_owner_seen_ms = 0.0
        if released:
            _record_session_event("control_owner_released", {"page_id": page_id})
        return web.json_response({"ok": True, "released": released})


    def _ensure_session_recorder(self, token: str | None) -> None:
        """Lazy-open the per-session recorder on first authenticated activity.
        Once-per-process: subsequent calls (e.g. browser reconnects from the same
        operator) are no-ops, so a flaky link doesn't fragment the recording.
        To start a new session, restart teleop."""
        name = self._token_name_for(token) if self._auth_enabled else "no_auth"
        if name is None:
            name = "unknown"
        self._ensure_session_recorder_name(name)

    def _ensure_session_recorder_name(self, name: str) -> None:
        global _ACTIVE_RECORDER
        if self._session_recorder is not None:
            return
        try:
            rec = SessionRecorder(name)
        except Exception as e:  # noqa: BLE001
            print(f"[recorder] open failed: {e} (continuing without recording)")
            return
        self._session_recorder = rec
        self._notify_trajectory_session(rec.session_dir)
        _ACTIVE_RECORDER = rec

    async def _shutdown(self) -> None:
        global _ACTIVE_RECORDER
        for task in (self._relay_task, self._relay_stats_task):
            if task is not None:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._relay_task = None
        self._relay_stats_task = None
        if self._session_recorder is not None:
            _ACTIVE_RECORDER = None
            try:
                self._session_recorder.close()
            except Exception as e:  # noqa: BLE001
                print(f"[recorder] close failed: {e}")
            self._session_recorder = None
        if self._site is not None:
            await self._site.stop()
        if self._runner is not None:
            await self._runner.cleanup()
        self._loop.stop()

    async def _serve_index(self, _request: web.Request) -> web.Response:
        return web.FileResponse(STATIC_DIR / "index.html", headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        })

    async def _serve_app_js(self, _request: web.Request) -> web.Response:
        return web.FileResponse(STATIC_DIR / "app.js", headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        })

    def _record_browser_debug_payload(self, data: dict) -> None:
        """Record one best-effort browser debug payload.

        Debug payloads arrive over the non-control HTTP /test_event path, so
        they are excluded from pose sequence/order checks and from the motion
        gate. They are joined post-run by page_id + last_pose_seq.
        """
        t_recv_ms = time.time() * 1000.0
        self._note_control_owner_seen(
            data.get("page_id") or data.get("app_page_id")
        )
        for src, dst in (
            ("ctrl_state", "ctrl_state"),
            ("ctrl_ready", "ctrl_ready"),
            ("ctrl_conn_seq", "ctrl_conn_seq"),
            ("ctrl_connect_seq", "ctrl_conn_seq"),
            ("ctrl_reconnect_count", "ctrl_reconnect_count"),
            ("ctrl_last_reconnect_reason", "ctrl_last_reconnect_reason"),
        ):
            if src in data:
                self._dc_last_client_ctrl[dst] = data.get(src)

        t_browser_debug_ms = data.get("t", data.get("t_browser_ms"))
        source_age_data = dict(data)
        if "t" not in source_age_data and t_browser_debug_ms is not None:
            source_age_data["t"] = t_browser_debug_ms
        source_age_ms = self._source_age_ms(source_age_data, t_recv_ms)
        relay_recv_ms = self._finite_float(data.get("_relay_recv_ms"))
        relay_send_ms = self._finite_float(data.get("_relay_send_ms"))
        relay_queue_ms = self._finite_float(data.get("_relay_queue_age_ms"))
        if relay_queue_ms is None and relay_recv_ms is not None and relay_send_ms is not None:
            relay_queue_ms = max(0.0, relay_send_ms - relay_recv_ms)
        relay_to_pi_ms = (
            t_recv_ms - relay_send_ms
            if relay_send_ms is not None
            else None
        )
        edge_send_ms = self._finite_float(data.get("_edge_send_ms"))
        edge_to_relay_ms = (
            relay_recv_ms - edge_send_ms
            if relay_recv_ms is not None and edge_send_ms is not None
            else self._finite_float(data.get("_edge_to_relay_ms"))
        )
        t_client_ms = self._finite_float(t_browser_debug_ms)
        t_browser_write_ms = self._finite_float(data.get("t_browser_write_ms"))
        browser_queue_ms = (
            t_browser_write_ms - t_client_ms
            if t_client_ms is not None and t_browser_write_ms is not None
            else self._finite_float(data.get("_browser_queue_ms"))
        )
        phone_to_relay_ms = None
        if bool(data.get("off_valid", False)):
            clock_offset_ms = self._finite_float(data.get("off"))
            if (
                t_client_ms is not None
                and clock_offset_ms is not None
                and relay_recv_ms is not None
            ):
                phone_to_relay_ms = relay_recv_ms - (t_client_ms + clock_offset_ms)

        row = {
            "type": data.get("type") or data.get("kind"),
            "app_v": data.get("app_v") or data.get("app_schema_id"),
            "page_id": data.get("page_id") or data.get("app_page_id"),
            "last_pose_seq": data.get("last_pose_seq", data.get("lastPoseSeqSent")),
            "last_pose_t_browser_ms": data.get(
                "last_pose_t_browser_ms",
                data.get("lastPoseTBrowserMs"),
            ),
            "last_pose_sent_age_ms": data.get("last_pose_sent_age_ms"),
            "t_browser_debug_ms": t_browser_debug_ms,
            "t_browser_write_ms": data.get("t_browser_write_ms"),
            "t_pi_recv_ms": round(t_recv_ms, 3),
            "clock_offset_ms": data.get("off"),
            "clock_offset_valid": int(bool(data.get("off_valid", False))),
            "source_age_ms": round(source_age_ms, 3) if source_age_ms is not None else None,
            "ctrl_ack_age_ms": data.get("ctrl_ack_age_ms"),
            "ctrl_edge_ack_age_ms": data.get("ctrl_edge_ack_age_ms"),
            "ctrl_robot_ack_age_ms": data.get("ctrl_robot_ack_age_ms"),
            "ctrl_edge_rtt_ms": data.get("ctrl_edge_rtt_ms"),
            "ctrl_robot_rtt_ms": data.get("ctrl_robot_rtt_ms"),
            "ctrl_state": data.get("ctrl_state"),
            "ctrl_ready": data.get("ctrl_ready"),
            "ctrl_conn_seq": data.get("ctrl_conn_seq", data.get("ctrl_connect_seq")),
            "ctrl_reconnect_count": data.get("ctrl_reconnect_count"),
            "ctrl_last_reconnect_reason": data.get("ctrl_last_reconnect_reason"),
            "video_state": data.get("video_state"),
            "video_error": data.get("video_error"),
            "video_reconnect_count": data.get("video_reconnect_count"),
            "video_last_reconnect_reason": data.get("video_last_reconnect_reason"),
            "robot_tracking_error_m": data.get("robot_tracking_error_m"),
            "robot_tracking_command_error_m": data.get("robot_tracking_command_error_m"),
            "robot_tracking_goal_error_m": data.get("robot_tracking_goal_error_m"),
            "robot_tracking_age_ms": data.get("robot_tracking_age_ms"),
            "robot_tracking_seq": data.get("robot_tracking_seq"),
            "robot_tracking_phone_enabled": data.get("robot_tracking_phone_enabled"),
            "wrtc_rtt_ms": data.get("wrtc_rtt_ms", data.get("wrtc_rtt")),
            "wrtc_jbuf_ms": data.get("wrtc_jbuf_ms", data.get("wrtc_jbuf")),
            "wrtc_decode_ms": data.get("wrtc_decode_ms", data.get("wrtc_decode")),
            "wrtc_jitter_ms": data.get("wrtc_jitter_ms", data.get("wrtc_jitter")),
            "wrtc_fps": data.get("wrtc_fps"),
            "wrtc_freezes": data.get("wrtc_freezes"),
            "wrtc_via": data.get("wrtc_via"),
            "wrtc_decoder_impl": data.get("wrtc_decoder_impl"),
            "wrtc_power_efficient": data.get("wrtc_power_efficient"),
            "wrtc_packets_lost": data.get("wrtc_packets_lost"),
            "wrtc_packets_received": data.get("wrtc_packets_received"),
            "wt_write_attempts": data.get("wt_write_attempts", data.get("wtWriteAttempts")),
            "wt_writer_blocked_count": data.get(
                "wt_writer_blocked_count",
                data.get("wtWriterBlockedCount"),
            ),
            "wt_writer_blocked_ms": data.get(
                "wt_writer_blocked_ms",
                data.get("wtWriterBlockedMs"),
            ),
            "debug_post": data.get("debugPost"),
            "debug_post_fail": data.get("debugPostFail"),
            "relay_epoch": data.get("_relay_epoch"),
            "browser_queue_ms": browser_queue_ms,
            "browser_write_to_edge_ms": data.get("_browser_write_to_edge_ms"),
            "browser_write_to_relay_ms": data.get("_browser_write_to_relay_ms"),
            "edge_recv_ms": data.get("_edge_recv_ms"),
            "edge_send_ms": data.get("_edge_send_ms"),
            "edge_queue_ms": data.get("_edge_q_ms"),
            "edge_to_relay_ms": edge_to_relay_ms,
            "relay_recv_ms": data.get("_relay_recv_ms"),
            "relay_send_ms": data.get("_relay_send_ms"),
            "relay_queue_age_ms": relay_queue_ms,
            "phone_to_relay_ms": phone_to_relay_ms,
            "relay_to_pi_ms": relay_to_pi_ms,
        }
        rec = self._session_recorder
        if rec is not None:
            rec.record_debug(row)
        self._maybe_log_webrtc_stats(data, time.time())

    def _ingest_pose_msg(self, data: dict, *, is_active: bool = True) -> dict | None:
        """Process one WebTransport pose datagram.

        Stamps _t_recv_local_ms, applies the out-of-order drop logic, and
        updates self._latest_msg under the state lock.

        Returns an echo dict if the caller should send one back for RTT
        measurement (None if the inbound msg didn't carry a 't' field).
        """
        if not is_active:
            return None
        t_recv_ms = time.time() * 1000.0
        self._note_control_owner_seen(
            data.get("page_id") or data.get("app_page_id")
        )
        # WT arrival pattern: track inter-arrival gaps so the periodic stats
        # snapshot can report bursts (sub-ms = kernel buffer flush after the
        # asyncio loop was busy) vs steady delivery.
        if self._dc_last_arrival_ms is not None:
            gap_ms = t_recv_ms - self._dc_last_arrival_ms
            if gap_ms < 1.0:
                self._dc_window_bursts += 1
            if gap_ms > self._dc_window_max_gap_ms:
                self._dc_window_max_gap_ms = gap_ms
        self._dc_last_arrival_ms = t_recv_ms
        self._dc_window_count += 1
        out_of_order, order_drop_details = self._update_dc_seq_stats(data)
        try:
            self._dc_window_client_skipped += int(data.get("ws_skipped", 0) or 0)
        except (TypeError, ValueError):
            pass
        try:
            client_buffered = int(float(data.get("ws_buffered", 0) or 0))
            if client_buffered > self._dc_window_client_buffered_max:
                self._dc_window_client_buffered_max = client_buffered
        except (TypeError, ValueError):
            pass
        for key in (
            "ctrl_state",
            "ctrl_ready",
            "ctrl_conn_seq",
            "ctrl_reconnect_count",
            "ctrl_last_reconnect_reason",
        ):
            if key in data:
                self._dc_last_client_ctrl[key] = data.get(key)
        data["_t_recv_local_ms"] = t_recv_ms
        source_age_ms = self._source_age_ms(data, t_recv_ms)
        if source_age_ms is not None:
            data["_source_age_ms"] = float(source_age_ms)
            if source_age_ms > self._dc_window_max_source_age_ms:
                self._dc_window_max_source_age_ms = float(source_age_ms)
        if "_relay_recv_ms" in data or "_relay_send_ms" in data:
            self._update_relay_timing_stats(data, t_recv_ms)

        if out_of_order:
            self._ws_out_of_order_drops += 1
            self._dc_window_out_of_order_drops += 1
            if order_drop_details is not None:
                self._maybe_log_out_of_order_drop(data, order_drop_details)

        # Latest-only plus monotonic sequence order is the safety boundary here:
        # steady high latency remains operator-adaptable, but duplicate/backwards
        # datagrams are older than a command already seen by this process.
        usable_pose = not out_of_order
        if usable_pose:
            # Gate FIRST (still on the ingest thread, in arrival order), then
            # publish msg + cached action together under the state lock so
            # get_action() never sees them out of sync.
            t_recv_ms_for_gate = float(data.get("_t_recv_local_ms", t_recv_ms))
            if data.get("type") == LEADER_MESSAGE_TYPE:
                action_state = self._process_leader_state(data, t_recv_ms_for_gate)
            else:
                action_state = self._process_pose_state(data, t_recv_ms_for_gate)
            with self._state_lock:
                self._latest_msg = data
                self._latest_action_state = action_state
                self._n_msgs += 1
            self._publish_action_state(action_state)
            # Stream a per-message pose row to the session recorder. Filter
            # to B1-held only -- that's when the operator is intentionally
            # commanding, which is what a learner wants. Keep the displayed RTP
            # timestamp so video/display timing can be correlated post-run.
            rec = self._session_recorder
            if rec is not None and bool(data.get("enabled", False)):
                pos = data.get("pos") or [None, None, None]
                quat = data.get("rot_xyzw") or [None, None, None, None]
                processed_pos = action_state.get("phone.pos")
                processed_inputs = dict(action_state.get("phone.raw_inputs", {}))
                row = {
                    "seq": data.get("seq"),
                    "app_v": data.get("app_v"),
                    "page_id": data.get("page_id"),
                    "t_browser_send_ms": data.get("t"),
                    "t_browser_write_ms": data.get("t_browser_write_ms"),
                    "t_pi_recv_ms": round(t_recv_ms, 3),
                    "displayed_rtp_ts": data.get("displayed_rtp_ts"),
                    "t_op_displayed_ms": data.get("t_op_displayed_ms"),
                    "clock_offset_ms": data.get("off"),
                    "clock_offset_valid": int(bool(data.get("off_valid", False))),
                    "client_skipped": data.get("ws_skipped"),
                    "client_buffered": data.get("ws_buffered"),
                    "relay_epoch": data.get("_relay_epoch"),
                    "browser_queue_ms": data.get("_browser_queue_ms"),
                    "browser_write_to_edge_ms": data.get("_browser_write_to_edge_ms"),
                    "browser_write_to_relay_ms": data.get("_browser_write_to_relay_ms"),
                    "edge_recv_ms": data.get("_edge_recv_ms"),
                    "edge_send_ms": data.get("_edge_send_ms"),
                    "edge_queue_ms": data.get("_edge_q_ms"),
                    "edge_to_relay_ms": data.get("_edge_to_relay_ms"),
                    "relay_recv_ms": data.get("_relay_recv_ms"),
                    "relay_send_ms": data.get("_relay_send_ms"),
                    "relay_queue_age_ms": data.get("_relay_queue_age_ms"),
                    "phone_to_relay_ms": data.get("_phone_to_relay_ms"),
                    "relay_to_pi_ms": data.get("_relay_to_pi_ms"),
                    "relay_total_age_ms": data.get("_relay_total_age_ms"),
                    "pose_valid": int(bool(data.get("pose_valid", True))),
                    "b1_held": 1,
                    "raw_pos_x": pos[0] if len(pos) > 0 else None,
                    "raw_pos_y": pos[1] if len(pos) > 1 else None,
                    "raw_pos_z": pos[2] if len(pos) > 2 else None,
                    "raw_rot_x": quat[0] if len(quat) > 0 else None,
                    "raw_rot_y": quat[1] if len(quat) > 1 else None,
                    "raw_rot_z": quat[2] if len(quat) > 2 else None,
                    "raw_rot_w": quat[3] if len(quat) > 3 else None,
                }
                if processed_pos is not None:
                    row["phone_pos_x"] = processed_pos[0]
                    row["phone_pos_y"] = processed_pos[1]
                    row["phone_pos_z"] = processed_pos[2]
                row["phone_enabled"] = int(bool(action_state.get("phone.enabled", False)))
                for src, dst in (
                    ("_yaw_delta_deg", "yaw_delta_deg"),
                    ("_phone_roll_deg", "phone_roll_deg"),
                    ("_phone_pitch_deg", "phone_pitch_deg"),
                    ("_pose_jump_m", "pose_jump_m"),
                    ("_pose_speed_mps", "pose_speed_mps"),
                    ("_orient_jump_deg", "orient_jump_deg"),
                    ("_orient_speed_dps", "orient_speed_dps"),
                    ("_yaw_jump_deg", "yaw_jump_deg"),
                    ("_yaw_speed_dps", "yaw_speed_dps"),
                    ("_pose_fault", "pose_fault"),
                    ("_control_reacquire_required", "control_reacquire_required"),
                    ("_control_hold", "control_hold"),
                    ("_source_age_ms", "source_age_ms"),
                    ("_edge_recv_ms", "edge_recv_ms"),
                    ("_edge_send_ms", "edge_send_ms"),
                    ("_edge_q_ms", "edge_queue_ms"),
                    ("_edge_to_relay_ms", "edge_to_relay_ms"),
                    ("_ws_out_of_order_drops", "ws_out_of_order_drops"),
                    ("_b1_edge_seq", "b1_edge_seq"),
                    ("_b1_edge_reason", "b1_edge_reason"),
                    ("_b1_edge_t_ms", "b1_edge_t_ms"),
                ):
                    row[dst] = processed_inputs.get(src)
                rec.record_pose(row)

        if "t" in data:
            gate = self._operator_gate_status(data)
            echo = {
                "ack_t": data["t"],
                "t_server_recv_ms": t_recv_ms,
                "t_server_send_ms": time.time() * 1000.0,
                "server_gate_state": gate["state"],
                "server_gate_severity": gate["severity"],
                "server_gate_message": gate["message"],
                "server_gate_requires_reengage": gate["requires_reengage"],
                "server_gate_reason": gate["reason"],
            }
            echo.update(self._robot_feedback_snapshot())
            return echo
        return None

def _browser_phone_process_main(
    config: PhoneConfig,
    port: int,
    action_queue,
    command_queue,
    status_queue,
    trajectory_queue=None,
) -> None:
    """Child-process entry point for the browser/control server."""
    server = BrowserPhone(config, port=port)
    server._action_ipc_queue = action_queue
    server._command_ipc_queue = command_queue
    server._status_ipc_queue = status_queue
    server._trajectory_ipc_queue = trajectory_queue

    def _on_sighup(_signum, _frame):
        if server._loop is not None and server._loop.is_running():
            server._loop.call_soon_threadsafe(server._load_tokens)

    try:
        signal.signal(signal.SIGHUP, _on_sighup)
    except (ValueError, OSError):
        pass

    try:
        server._run_server()
    except BaseException as e:  # noqa: BLE001
        try:
            status_queue.put_nowait(("error", repr(e)))
        except Exception:
            pass
        raise
