#!/usr/bin/env python
"""Phone teleop for SO-101 via a browser/WebXR phone client.

The timing-sensitive teleop loop and action processors are local. This file adds:

  * `SafeStartupSO101Follower` subclass that seeds Goal=Present before
    torque-enable so the arm doesn't lurch toward raw tick 0 at connect.
  * Closed-form turret IK with a per-joint command rate cap and clean workspace
    edge clamps.
  * `CylindricalPhoneToEE` turret mapping (phone yaw -> shoulder pan; reach/up
    place the EE) plus wrist trim controls.
  * Bus-resilience retries for transient Feetech packet errors.

Tuning is inlined as constants (see project history for derivation). All
patches are process-local (no on-disk lerobot mutation).
"""
from __future__ import annotations

import faulthandler
import os
import signal
import subprocess
import sys
import time
from pprint import pformat
from typing import Any

# Enabled BEFORE any C-extension imports. Vendor bus, numeric, and crypto
# libraries run code in C; a segfault in any of them kills the process with no Python
# traceback by default. faulthandler installs a SIGSEGV/SIGABRT/SIGFPE/SIGBUS
# handler that dumps a Python-level stack trace to stderr -- which our run
# script pipes through `tee` to the persistent teleop log, so the cause
# survives the crash. SIGUSR1 lets us probe a hung process from another shell without
# attaching gdb (`kill -USR1 $(pidof python)`).
faulthandler.enable(file=sys.stderr, all_threads=True)
if hasattr(signal, "SIGUSR1"):
    faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True, chain=False)

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

import follower.gateway as _browser_phone_mod  # for access to _ACTIVE_RECORDER
from follower.hardware import select_follower_arm
from follower.gateway import BrowserPhone
from follower.leader_mapping import RelativeLeaderMapper


RobotAction = dict[str, Any]
RobotObservation = dict[str, Any]


class TransitionKey:
    ACTION = "action"
    OBSERVATION = "observation"


def robot_action_observation_to_transition(
    data: tuple[RobotAction, RobotObservation],
) -> dict[str, Any]:
    action, observation = data
    return {
        TransitionKey.ACTION: dict(action),
        TransitionKey.OBSERVATION: observation,
    }


def transition_to_robot_action(transition: dict[str, Any]) -> RobotAction:
    return transition[TransitionKey.ACTION]


class RobotActionProcessorStep:
    """Local subset of the lerobot processor-step contract used by this file."""

    _current_transition: dict[str, Any] | None = None

    @property
    def transition(self) -> dict[str, Any]:
        if self._current_transition is None:
            raise RuntimeError("Processor transition is not set")
        return self._current_transition

    def __call__(self, transition: dict[str, Any]) -> dict[str, Any]:
        self._current_transition = transition
        transition[TransitionKey.ACTION] = self.action(transition[TransitionKey.ACTION])
        return transition

    def action(self, action: RobotAction) -> RobotAction:
        return action

class RobotProcessorPipeline:
    """Small local action pipeline; kept API-compatible to minimize this phase."""

    def __init__(self, steps=(), to_transition=None, to_output=None):
        self.steps = list(steps)
        self.to_transition = to_transition or (lambda data: data)
        self.to_output = to_output or (lambda data: data)

    @classmethod
    def __class_getitem__(cls, _item):
        return cls

    def __call__(self, data):
        transition = self.to_transition(data)
        for step in self.steps:
            transition = step(transition)
        return self.to_output(transition)


# --- Constants -------------------------------------------------------------
URDF = os.path.join(
    os.path.dirname(__file__), "robot_models", "so101", "so101_new_calib.urdf"
)
TARGET_FRAME = "gripper_frame_link"
FPS = 60
CONTROL_WATCHDOG_HZ = 20.0
CONTROL_MAX_RATE_DT_S = 0.05
_KEY_CONTROL_DT_S = "control.dt_s"

# Tuned values, inlined as constants (each selected empirically; see project
# history for derivation).
MAX_CMD_DEG_PER_SEC = 200.0  # per-joint command rate cap; feeds the closed-form rate-limiter below.


# Joint limits in degrees. Populated at startup by _derive_joint_limits() from
# the loaded servo calibration (range_min/range_max in ticks → degrees) so the
# limits track the arm's actual calibration. Hardcoding would silently drift
# from reality after a recalibration.
_JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {}

# wrist_roll is a full-turn motor in calibration (range [0, 4095] → ±180°).
# Cap it narrower in software to avoid winding cables past safe rotation.
_WRIST_ROLL_LIMIT_DEG = (-157.0, 163.0)


def _derive_joint_limits(robot) -> None:
    """Populate _JOINT_LIMITS_DEG from the loaded servo calibration.

    Half the calibration range maps to ±N°, with each tick worth 360°/4096."""
    global _JOINT_LIMITS_DEG
    ticks_per_rev = 4096
    limits: dict[str, tuple[float, float]] = {}
    for motor, cal in robot.bus.calibration.items():
        if motor == "wrist_roll":
            limits[motor] = _WRIST_ROLL_LIMIT_DEG
        elif motor == "gripper":
            continue  # gripper position isn't a joint-angle clamp target
        else:
            half_deg = (cal.range_max - cal.range_min) / 2 * 360.0 / ticks_per_rev
            limits[motor] = (-half_deg, half_deg)
    _JOINT_LIMITS_DEG = limits

# Record phone/desired/actual/IK-promised EE per frame to
# PHONE_ARM_TRAJECTORY_CSV for offline analysis.

# --- Closed-form turret IK -------------------------------------------------
# The turret reduces to a determined 3-DOF problem: pan = -azimuth about the pan
# axis; lift+elbow = a 2-link planar reach to (r, h). No iteration, no QP -> no
# infeasible-QP crash, no local-minima dead-zones, no singular whipping. Edges
# become clean clamps (pan limit, reach annulus, joint limits). Geometry was
# extracted from the URDF FK and uses the fixed elbow>=0 branch.
_CF_AX = 38.8353            # pan-axis x in base frame (mm); pan axis at (AX, 0, _)
_CF_SX, _CF_SZ = 69.235, 116.600   # shoulder(lift) axis in the de-panned x-z plane (mm)
_CF_L1 = 116.000           # lift-axis -> elbow-axis (mm)
_CF_LFA = 135.000          # elbow -> wrist (mm)
_CF_LWG = 159.423          # wrist -> gripper tip (mm)
_CF_P1, _CF_P2, _CF_P3 = 76.032, 2.207, -2.841   # zero-config link angles (deg)
# Radial clamps widened to the arm's TRUE lift+elbow envelope (measured from the
# geometry below): the tip reaches ~479mm fully extended (was clipped at 440 --
# lost ~39mm), and the elbow-fold inner drops to ~87mm at high reach-up (was
# clipped at 120). Beyond these the joint-limit bisect / annulus clamp is the real
# wall, so the workspace is now bounded by the lift/elbow motors themselves, not an
# arbitrary number. Pan stays capped by its joint limit (the azimuth wall).
# NOTE: the very outer edge (~479mm) is the dead-straight, near-singular full-stretch
# -- elbow gets sensitive there; the per-frame rate cap keeps it bounded.
_CF_R_MIN_MM = 80.0
_CF_R_MAX_MM = 480.0
_CF_MAX_STEP_DEG = MAX_CMD_DEG_PER_SEC / FPS   # per-frame joint rate cap
_CF_LIMIT_MARGIN_DEG = 3.0     # keep joint goals this far inside the hard limits so
                               # the motor never stalls holding against its stop


def _cf_u(a_deg: float) -> np.ndarray:
    a = np.radians(a_deg)
    return np.array([np.cos(a), np.sin(a)])


def _cf_solve_raw(az_rad: float, r_mm: float, h_mm: float, wrist_flex_deg: float):
    """Raw analytic solve (pan, lift, elbow) in degrees. Clamps the 2-link reach
    distance to the annulus (so the math is always valid) but does NOT clamp the
    joint angles -- the caller decides reachability. Tip-control: wrist_flex is
    folded into the effective forearm so the gripper TIP lands on (az, r, h)."""
    x = r_mm * np.cos(az_rad); y = r_mm * np.sin(az_rad)        # base-frame mm
    pan = -np.degrees(np.arctan2(y, x - _CF_AX))
    rho = np.hypot(x - _CF_AX, y)                               # radius from pan axis
    X = _CF_AX + rho; Z = h_mm                                  # de-panned x-z target
    W = _CF_LFA * _cf_u(_CF_P2) + _CF_LWG * _cf_u(_CF_P3 - wrist_flex_deg)
    L2 = float(np.hypot(*W)); phi2 = float(np.degrees(np.arctan2(W[1], W[0])))
    d = np.array([X - _CF_SX, Z - _CF_SZ]); D = float(np.hypot(*d))
    Dc = float(np.clip(D, abs(_CF_L1 - L2) + 1e-6, _CF_L1 + L2 - 1e-6))
    d = d * (Dc / D) if D > 1e-6 else np.array([Dc, 0.0])
    cb = (Dc*Dc + _CF_L1*_CF_L1 - L2*L2) / (2*_CF_L1*Dc)
    b = np.degrees(np.arccos(np.clip(cb, -1.0, 1.0)))
    th1 = np.degrees(np.arctan2(d[1], d[0])) + b                # fixed branch +1 (elbow>=0)
    lift = _CF_P1 - th1
    rem = d - _CF_L1 * _cf_u(th1)
    elbow = (phi2 - np.degrees(np.arctan2(rem[1], rem[0]))) - lift
    return pan, lift, elbow


def _closed_form_arm_ik(az_rad: float, r_m: float, h_m: float, wrist_flex_deg: float):
    """Turret command (az, r, h) [base-origin cylindrical] + locked wrist_flex ->
    (pan, lift, elbow) degrees, projected onto the reachable workspace.

    az comes straight from phone yaw (never recovered via atan2 of a vanishing
    vector) so there's no azimuth singularity / negative-r flip. The reach wall
    is a clean RADIAL clamp: if the commanded radius puts lift/elbow past a limit,
    we slide r in/out along the SAME azimuth+height to the nearest reachable
    radius -- so the tip stays exactly where you're pointing, only the reach is
    capped. pan is clamped to the turret limit (the azimuth wall)."""
    h_mm = h_m * 1000.0
    r0 = float(np.clip(r_m * 1000.0, _CF_R_MIN_MM, _CF_R_MAX_MM))
    # Keep every wall a few degrees INSIDE the hard joint limit so the servo never
    # holds against its mechanical stop -- pushing the stop (commanded == limit,
    # measured ~1deg short) stalls/overloads the motor (shoulder_pan stuck at the
    # azimuth wall, 2026-05-26). _CF_LIMIT_MARGIN_DEG of headroom.
    M = _CF_LIMIT_MARGIN_DEG
    lo = _JOINT_LIMITS_DEG["shoulder_lift"][0] + M; hi = _JOINT_LIMITS_DEG["shoulder_lift"][1] - M
    elo = _JOINT_LIMITS_DEG["elbow_flex"][0] + M; ehi = _JOINT_LIMITS_DEG["elbow_flex"][1] - M
    plo = _JOINT_LIMITS_DEG["shoulder_pan"][0] + M; phi = _JOINT_LIMITS_DEG["shoulder_pan"][1] - M
    pan, lift, elbow = _cf_solve_raw(az_rad, r0, h_mm, wrist_flex_deg)

    def _ok(r):
        _p, _li, _el = _cf_solve_raw(az_rad, r, h_mm, wrist_flex_deg)
        return lo <= _li <= hi and elo <= _el <= ehi

    if not (lo <= lift <= hi and elo <= elbow <= ehi):
        # Radial wall: slide r along the SAME azimuth+height to the nearest
        # reachable radius, then BISECT to the exact reachable/unreachable boundary
        # so the wall is CONTINUOUS in r0. (A coarse grid scan snapped r to grid
        # points -> multi-degree goal jumps near the elbow-fold singularity, which
        # the rate cap turned into a frame-by-frame "tap". Bisecting to the true
        # boundary makes the held wall position smooth.)
        r_in = None
        for d in np.arange(4.0, _CF_R_MAX_MM - _CF_R_MIN_MM + 4.0, 4.0):
            for cand in (r0 + d, r0 - d):   # +d first: elbow-fold is relieved by extending
                if _CF_R_MIN_MM <= cand <= _CF_R_MAX_MM and _ok(cand):
                    r_in = cand
                    break
            if r_in is not None:
                break
        if r_in is not None:
            a, b = r_in, r0   # a reachable, b not -> bisect toward the boundary nearest r0
            for _ in range(16):
                m = 0.5 * (a + b)
                if _ok(m):
                    a = m
                else:
                    b = m
            pan, lift, elbow = _cf_solve_raw(az_rad, a, h_mm, wrist_flex_deg)
    pan = float(np.clip(pan, plo, phi))
    lift = float(np.clip(lift, lo, hi)); elbow = float(np.clip(elbow, elo, ehi))
    return pan, lift, elbow


CYL_YAW_GAIN = 1.0  # cylindrical/"turret" gain: phone yaw -> shoulder pan
# The cylindrical control stream keeps azimuth continuous across a clutch hold,
# so radius must stay in the nonnegative workspace before wrist compensation and
# IK see it. A signed negative radius would imply an azimuth flip by pi, which is
# the jump reproduced in the 2026-06-16 run.
CYL_MIN_RADIUS_M = _CF_R_MIN_MM / 1000.0
# Phone ROLL (twist about the viewing axis) drives wrist_roll. Incremental from the
# absolute gravity-referenced roll, so it's drift/jitter-immune (telescoping sum)
# and continuous across clutch.
PHONE_ROLL_GAIN = -0.5  # phone twist 90deg -> wrist_roll ~45deg (negative: match hand direction)
# Phone PITCH -> wrist_flex, positional (mirrors phone_roll). Incremental from the
# absolute gravity-referenced pitch so it's drift/jitter-immune and continuous
# across clutch. Reading-tilt couples in (no deadband) -- accept and tune by gain.
PHONE_PITCH_GAIN = -0.5  # phone pitch 30deg -> wrist_flex 15deg (match hand direction)
_EE_DEBUG_PRINT_PERIOD_S = 0.5  # cadence of the [placement_trim] live debug line

_LATEST_EE_TARGET_POS: np.ndarray | None = None
_PLACEMENT_WRIST_TARGET_DEG: dict[str, float] | None = None
_LATEST_ARM_GOAL_EE_POS: np.ndarray | None = None
_LATEST_CYL_DEBUG: dict[str, object] = {}
_LATEST_WRIST_DEBUG: dict[str, object] = {}
_LATEST_IK_DEBUG: dict[str, object] = {}


def _measured_qT(self):
    """(measured joints in motor_names order, FK transform) or (None, None)."""
    observation = self.transition.get(TransitionKey.OBSERVATION)
    if observation is None:
        return None, None
    try:
        q = np.array([float(observation[f"{m}.pos"]) for m in self.motor_names],
                     dtype=float)
    except (KeyError, TypeError, ValueError):
        return None, None
    if "wrist_flex" in self.motor_names:
        # The IK controls the STRAIGHT-wrist tip (direct flex), so the measured EE
        # (clutch re-cal, hold, tracking) must be the straight-wrist tip too -- FK
        # with wrist_flex zeroed. Otherwise re-cal reads the actual (flexed) tip and
        # the arm lurches the gripper-bend offset (~150mm at 60deg) on clutch engage.
        q = q.copy()
        q[self.motor_names.index("wrist_flex")] = 0.0
    return q, self.kinematics.forward_kinematics(q)


# --- scservo_sdk mutex-safety patch --------------------------------------
# The Feetech SDK's PortHandler carries an is_using flag as a mutex around
# each txRxPacket:
#     if port.is_using: return COMM_PORT_BUSY
#     port.is_using = True
#     ... port.readPort / port.writePort ...    # <-- CAN RAISE
#     port.is_using = False                     # <-- SKIPPED ON EXCEPTION
# When pyserial's read/write raises a Python exception (e.g. SerialException
# from a transient USB hiccup, "device reports readiness to read but returned
# no data") the flag stays True. Every subsequent bus operation then fails
# with "Port is in use", INCLUDING the teardown-time disable_torque. In the
# 2026-07-07 wedge session that produced a 30-second retry storm on a shared
# USB host controller; the storm starved the camera's USB bandwidth, ffmpeg
# emitted RTP with jitter, and chrome's video receiver wedged. See git log
# and events.jsonl 20260706_123206 for the full chain.
# Fix: wrap readPort/writePort at the SDK class level so an exception clears
# is_using before re-raising. Preserves the mutex invariant (True only while
# the port is actually mid-operation) without changing success behaviour.
from scservo_sdk import port_handler as _scs_port_handler

_orig_scs_readPort = _scs_port_handler.PortHandler.readPort
_orig_scs_writePort = _scs_port_handler.PortHandler.writePort
_SCS_PORT_RECOVERY_COUNT = 0

def _safe_scs_readPort(self, length):
    try:
        return _orig_scs_readPort(self, length)
    except BaseException:
        global _SCS_PORT_RECOVERY_COUNT
        _SCS_PORT_RECOVERY_COUNT += 1
        # Clear the SDK's stuck mutex. Without this every subsequent call
        # short-circuits with COMM_PORT_BUSY until the port is closed.
        try:
            self.is_using = False
        except Exception:
            pass
        raise

def _safe_scs_writePort(self, packet):
    try:
        return _orig_scs_writePort(self, packet)
    except BaseException:
        global _SCS_PORT_RECOVERY_COUNT
        _SCS_PORT_RECOVERY_COUNT += 1
        try:
            self.is_using = False
        except Exception:
            pass
        raise

_scs_port_handler.PortHandler.readPort = _safe_scs_readPort
_scs_port_handler.PortHandler.writePort = _safe_scs_writePort


# --- Bus resilience: retry sync_read on transient packet errors ----------
# A single dropped/corrupt packet on the Feetech bus would otherwise crash the
# entire control loop (lerobot's default num_retry is 0, so one bad packet ->
# ConnectionError -> uncaught -> exit). In live teleop this happens every few
# minutes from EMI / cable jiggle / motor 5 protection trips. Retry 3x with
# tiny backoff before giving up.
from lerobot.motors.motors_bus import MotorsBus as _MotorsBus  # noqa: N813
_orig_motorsbus_sync_read = _MotorsBus.sync_read
_orig_motorsbus_sync_write = _MotorsBus.sync_write
_orig_motorsbus_write = _MotorsBus.write

_BUS_RETRY_STATS = {"calls": 0, "retries": 0, "exhausted": 0}
_BUS_RETRY_PRINT_INTERVAL_S = 30.0
_bus_retry_last_print_t = 0.0
try:
    _BUS_CALL_SLOW_MS = float(os.environ.get("PHONE_ARM_BUS_CALL_SLOW_MS", "50"))
except (TypeError, ValueError):
    _BUS_CALL_SLOW_MS = 50.0


def _record_bus_event(event_type: str, details: dict) -> None:
    rec = _browser_phone_mod._ACTIVE_RECORDER
    record_event = getattr(rec, "record_event", None)
    if callable(record_event):
        record_event(event_type, details)


# Fast-path backoff (ms) for EMI / single-motor protection trip: 4 attempts in
# ~12ms covers nearly every normal-day blip without an operator-visible stall.
# Slow path (ms) for USB/PSU hiccups: keeps trying for ~2s before raising.
# Holding torque on the last commanded goal during the stall is safer than
# letting the loop die with torque latched on.
_BUS_RETRY_FAST_MS = (3, 3, 3, 3)
_BUS_RETRY_SLOW_MS = (10, 30, 100, 300, 800, 1500)
_BUS_STARTUP_MOTOR_CHECK_PASSES = 5
_BUS_STARTUP_MOTOR_CHECK_DELAY_S = 0.05
_BUS_STARTUP_FIRMWARE_CHECK_PASSES = 3
_BUS_STARTUP_CALIBRATION_CHECK_PASSES = 3


def _bus_retry(orig_fn, self, *args, normalize, num_retry, attempts):
    """Retry orig_fn on transient ConnectionError. Inner call passes num_retry=0
    so the retry policy lives here, not in upstream.

    Counts every call + each retry attempt + every exhausted (raised) call into
    _BUS_RETRY_STATS, and dumps a periodic [bus_retry] line every 30s. Periodic
    output survives broken-pipe / SIGKILL / hard exits where the shutdown
    finally-block's print would be lost.

    `attempts` keeps the legacy fast-path budget (caller picks 2 or 3). Once the
    fast path is exhausted we keep going through the slow-path schedule rather
    than raising -- a 12 ms-only policy used to crash the whole teleop loop on
    any USB hiccup, leaving torque latched on the arm."""
    global _bus_retry_last_print_t
    _BUS_RETRY_STATS["calls"] += 1
    now = time.time()
    if now - _bus_retry_last_print_t > _BUS_RETRY_PRINT_INTERVAL_S:
        _bus_retry_last_print_t = now
        s = _BUS_RETRY_STATS
        rate = 100 * s["retries"] / s["calls"] if s["calls"] else 0.0
        print(f"[bus_retry] calls={s['calls']} retries={s['retries']} "
              f"({rate:.3f}%) exhausted={s['exhausted']} "
              f"port_recoveries={_SCS_PORT_RECOVERY_COUNT}")
    fast_budget = max(num_retry, attempts)
    schedule_ms = list(_BUS_RETRY_FAST_MS[:fast_budget]) + list(_BUS_RETRY_SLOW_MS)
    last_exc: BaseException | None = None
    slow_announced = False
    call_retries = 0
    t_call_start = time.perf_counter()
    for i, backoff_ms in enumerate([0] + schedule_ms):
        if backoff_ms:
            time.sleep(backoff_ms / 1000.0)
        try:
            result = orig_fn(self, *args, normalize=normalize, num_retry=0)
            elapsed_ms = (time.perf_counter() - t_call_start) * 1000.0
            if call_retries or elapsed_ms > _BUS_CALL_SLOW_MS:
                _record_bus_event("bus_call_slow", {
                    "fn": orig_fn.__name__,
                    "data_name": args[0] if args else None,
                    "elapsed_ms": round(elapsed_ms, 1),
                    "retries": call_retries,
                    "slow_threshold_ms": _BUS_CALL_SLOW_MS,
                    "slow_path": int(slow_announced),
                })
            if slow_announced:
                print(f"[bus_retry] RECOVERED {orig_fn.__name__}"
                      f"({args[0] if args else '?'}) after {i} retries")
            return result
        except ConnectionError as exc:
            last_exc = exc
            _BUS_RETRY_STATS["retries"] += 1
            call_retries += 1
            if i == fast_budget and not slow_announced:
                # Fast path exhausted; entering slow path. Log once so the
                # operator sees we're stalling but not dying.
                slow_announced = True
                print(f"[bus_retry] STALL {orig_fn.__name__}"
                      f"({args[0] if args else '?'}): fast budget exhausted, "
                      f"entering slow retry (last err: {exc})")
    _BUS_RETRY_STATS["exhausted"] += 1
    # Slow path also exhausted (~2s of silence). Loud so it shows up in
    # /tmp/teleop.log next to the symptom.
    print(f"[bus_retry] EXHAUSTED after {len(schedule_ms) + 1} attempts "
          f"(~{sum(schedule_ms)} ms): {orig_fn.__name__}"
          f"({args[0] if args else '?'}) -> {last_exc.__class__.__name__}: {last_exc}")
    _record_bus_event("bus_call_exhausted", {
        "fn": orig_fn.__name__,
        "data_name": args[0] if args else None,
        "elapsed_ms": round((time.perf_counter() - t_call_start) * 1000.0, 1),
        "retries": call_retries,
        "scheduled_backoff_ms": sum(schedule_ms),
        "error": None if last_exc is None else repr(last_exc),
    })
    raise last_exc


def _resilient_sync_read(self, data_name, motors=None, *, normalize=True, num_retry=0):
    return _bus_retry(_orig_motorsbus_sync_read, self, data_name, motors,
                      normalize=normalize, num_retry=num_retry, attempts=3)


def _resilient_sync_write(self, data_name, values, *, normalize=True, num_retry=0):
    return _bus_retry(_orig_motorsbus_sync_write, self, data_name, values,
                      normalize=normalize, num_retry=num_retry, attempts=2)


def _resilient_write(self, data_name, motor, value, *, normalize=True, num_retry=0):
    # Connect/disconnect-time singular writes (Torque_Enable, P/D coeffs, ...)
    # aren't covered by the sync wrappers, so a single transient "no status
    # packet" glitch there is fatal at startup/shutdown. Retry the same way.
    return _bus_retry(_orig_motorsbus_write, self, data_name, motor, value,
                      normalize=normalize, num_retry=num_retry, attempts=3)


_MotorsBus.sync_read = _resilient_sync_read
_MotorsBus.sync_write = _resilient_sync_write
_MotorsBus.write = _resilient_write


def _bus_clear_stale_state(bus) -> None:
    port_handler = getattr(bus, "port_handler", None)
    if port_handler is None:
        return
    try:
        port_handler.clearPort()
    except Exception:
        pass
    try:
        port_handler.is_using = False
    except Exception:
        pass


def _close_bus_without_torque(bus) -> None:
    """Close a failed startup bus without trying torque writes.

    During motor discovery failures we do not know whether all expected motors
    can answer. Calling disable_torque here can turn a clean startup failure
    into a long retry storm, so cleanup is limited to clearing SDK state and
    closing the serial fd.
    """
    _bus_clear_stale_state(bus)
    try:
        if getattr(bus, "is_connected", False):
            bus.disconnect(disable_torque=False)
            return
    except Exception:
        pass
    try:
        bus.port_handler.closePort()
    except Exception:
        pass


def _expected_motor_models(bus) -> dict[int, int]:
    return {
        motor.id: bus.model_number_table[motor.model]
        for motor in bus.motors.values()
    }


def _format_wrong_models(expected: dict[int, int], found: dict[int, int]) -> list[str]:
    wrong = []
    for id_, model in sorted(found.items()):
        expected_model = expected.get(id_)
        if expected_model is not None and model != expected_model:
            wrong.append(f"{id_}: expected {expected_model}, found {model}")
    return wrong


def _assert_motors_exist_reliably(bus) -> None:
    """Ping expected motors over several short passes.

    A single missed status packet should not make startup rerun the whole robot
    connection sequence, but a motor that is still absent after these passes is
    a real startup fault and should fail loudly.
    """
    expected = _expected_motor_models(bus)
    found: dict[int, int] = {}
    ping_errors: dict[int, str] = {}

    for pass_idx in range(1, _BUS_STARTUP_MOTOR_CHECK_PASSES + 1):
        for id_ in sorted(expected):
            if found.get(id_) == expected[id_]:
                continue
            try:
                model_nb = bus.ping(id_, num_retry=1, raise_on_error=False)
            except Exception as exc:  # noqa: BLE001
                ping_errors[id_] = f"{exc.__class__.__name__}: {exc}"
                continue
            if model_nb is not None:
                found[id_] = model_nb

        missing = [id_ for id_ in sorted(expected) if id_ not in found]
        wrong = _format_wrong_models(expected, found)
        if not missing and not wrong:
            if pass_idx > 1:
                print(f"[connect] motor check recovered on pass {pass_idx}")
            return

        if pass_idx < _BUS_STARTUP_MOTOR_CHECK_PASSES:
            print(
                f"[connect] motor check pass {pass_idx}/"
                f"{_BUS_STARTUP_MOTOR_CHECK_PASSES}: "
                f"missing={missing or '-'} wrong={wrong or '-'}"
            )
            _bus_clear_stale_state(bus)
            time.sleep(_BUS_STARTUP_MOTOR_CHECK_DELAY_S)

    missing = [id_ for id_ in sorted(expected) if id_ not in found]
    wrong = _format_wrong_models(expected, found)
    error_lines = [
        f"{bus.__class__.__name__} motor check failed on port '{bus.port}' "
        f"after {_BUS_STARTUP_MOTOR_CHECK_PASSES} passes:"
    ]
    if missing:
        error_lines.append("\nMissing motor IDs:")
        error_lines.extend(
            f"  - {id_} (expected model: {expected[id_]})" for id_ in missing
        )
    if wrong:
        error_lines.append("\nMotors with incorrect model numbers:")
        error_lines.extend(f"  - {line}" for line in wrong)
    if ping_errors:
        error_lines.append("\nLast ping exceptions:")
        error_lines.extend(
            f"  - {id_}: {ping_errors[id_]}"
            for id_ in sorted(ping_errors)
        )
    error_lines.append("\nFull expected motor list (id: model_number):")
    error_lines.append(pformat(expected, indent=4, sort_dicts=False))
    error_lines.append("\nFull found motor list (id: model_number):")
    error_lines.append(pformat(found, indent=4, sort_dicts=False))
    raise RuntimeError("\n".join(error_lines))


def _run_startup_check_with_retries(label: str, fn, *, passes: int, bus=None):
    last_exc: Exception | None = None
    for pass_idx in range(1, passes + 1):
        try:
            result = fn()
            if pass_idx > 1:
                print(f"[connect] {label} recovered on pass {pass_idx}")
            return result
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if pass_idx < passes:
                print(
                    f"[connect] {label} pass {pass_idx}/{passes} failed: "
                    f"{exc.__class__.__name__}: {str(exc).splitlines()[0]}"
                )
                # Startup reads use the same tx/rx path as normal bus calls;
                # clear stale bytes and the SDK mutex before the next pass.
                clear_bus = bus if bus is not None else getattr(fn, "__self__", None)
                if clear_bus is not None:
                    _bus_clear_stale_state(clear_bus)
                time.sleep(_BUS_STARTUP_MOTOR_CHECK_DELAY_S)
    assert last_exc is not None
    raise last_exc


def _connect_bus_with_reliable_handshake(bus) -> None:
    try:
        bus.connect(handshake=False)
        _bus_clear_stale_state(bus)
        _assert_motors_exist_reliably(bus)
        _run_startup_check_with_retries(
            "firmware check",
            bus._assert_same_firmware,
            passes=_BUS_STARTUP_FIRMWARE_CHECK_PASSES,
            bus=bus,
        )
    except Exception:
        _close_bus_without_torque(bus)
        raise


def _disconnect_startup_cameras(robot) -> None:
    for cam in getattr(robot, "cameras", {}).values():
        if not getattr(cam, "is_connected", False):
            continue
        try:
            cam.disconnect()
        except Exception:
            pass


def _connect_robot(robot, *, calibrate: bool = True) -> None:
    """Connect SO101 with targeted bus startup recovery.

    This intentionally avoids SO101Follower.connect(), whose bus.connect()
    performs a one-shot motor handshake before packet timeout/retry policy is
    under our control, and can leave the serial fd open after RuntimeError.
    """
    if robot.is_connected:
        raise RuntimeError(f"{robot.__class__.__name__} is already connected")
    try:
        _connect_bus_with_reliable_handshake(robot.bus)
        is_calibrated = _run_startup_check_with_retries(
            "calibration check",
            lambda: robot.is_calibrated,
            passes=_BUS_STARTUP_CALIBRATION_CHECK_PASSES,
            bus=robot.bus,
        )
        if not is_calibrated and calibrate:
            robot.calibrate()
        for cam in robot.cameras.values():
            cam.connect()
        robot.configure()
    except Exception:
        _disconnect_startup_cameras(robot)
        _close_bus_without_torque(robot.bus)
        raise
    print("[connect] robot connected")


# --- SO101Follower subclass: seed Goal=Present to prevent startup lurch ----
class SafeStartupSO101Follower(SO101Follower):
    def configure(self) -> None:
        # CRITICAL: seed Goal_Position = Present_Position before torque enables.
        # Upstream SO101Follower.configure() does all its writes inside
        # `with self.bus.torque_disabled():`, and that context ENABLES torque on
        # exit. At that moment Goal_Position is still 0 (servos power up / reset
        # to 0 after a power-cycle), so every motor lurches toward raw tick 0 --
        # for shoulder_lift that's ~-191deg, slamming down through the maximum
        # gravity-torque region. It can't make it, grinds at high current, and
        # trips the overload latch. Seeding Goal=Present here (torque is still
        # off at this point in connect()) makes the servos hold their actual
        # physical pose when torque comes on, instead of slamming to zero.
        try:
            present = self.bus.sync_read("Present_Position")
            self.bus.sync_write("Goal_Position", present)
            print(f"[seed] Goal=Present before torque enable: "
                  f"{ {k: round(v, 1) for k, v in present.items()} }")
        except Exception as exc:  # noqa: BLE001
            print(f"[seed] failed to seed Goal=Present: {exc}")
        super().configure()


# --- Placement + trim control mode ---------------------------------------
# Action-dict keys passed from CylindricalPhoneToEE -> ClutchAndWristControl.
_KEY_TRIM_PHONE_ENABLED = "placement_trim.phone_enabled"
_KEY_TRIM_PHONE_ROLL_DEG = "placement_trim.phone_roll_deg"
_KEY_TRIM_PHONE_PITCH_DEG = "placement_trim.phone_pitch_deg"
_KEY_RESET_ARM_GOAL_TO_MEASURED = "placement.reset_arm_goal_to_measured"

class CylindricalPhoneToEE(RobotActionProcessorStep):
    """Map phone yaw/reach/up to cylindrical EE placement."""

    def __init__(self, kinematics, motor_names: list[str]):
        super().__init__()
        self.kinematics = kinematics
        self.motor_names = motor_names
        self._prev_enabled = False
        self._az0: float | None = None
        self._r0 = 0.0
        self._h0 = 0.0
        self._ee_hold: np.ndarray | None = None
        self._has_moved_since_clutch = False
        self._baseline_source_name = ""
        # Last raw EE target requested by this step. This is only a fallback:
        # the preferred clutch baseline is _LATEST_ARM_GOAL_EE_POS, which is FK
        # of the joint goal ClosedFormArmIK is actually holding. Raw desired EE
        # can be ahead of the rate-limited/clamped IK goal, and using it as the
        # next baseline makes the first cyl frame jump back toward stale desire.
        self._last_cmd_ee: np.ndarray | None = None
        self._radius_limited_prev = False

    def _baseline_source(self, meas: np.ndarray | None) -> tuple[np.ndarray | None, str]:
        if _LATEST_ARM_GOAL_EE_POS is not None:
            return _LATEST_ARM_GOAL_EE_POS, "arm_goal"
        if self._last_cmd_ee is not None:
            return self._last_cmd_ee, "last_cmd"
        return meas, "measured" if meas is not None else "none"

    def _capture_cyl_baseline(self, base: np.ndarray | None) -> None:
        if base is None:
            self._az0 = None
            return
        self._az0 = float(np.arctan2(base[1], base[0]))
        self._r0 = float(np.hypot(base[0], base[1]))
        self._h0 = float(base[2])

    def action(self, action: RobotAction) -> RobotAction:
        global _LATEST_CYL_DEBUG, _LATEST_ARM_GOAL_EE_POS

        enabled = bool(action.pop("phone.enabled"))
        pos = action.pop("phone.pos")
        action.pop("phone.rot", None)
        inputs = action.pop("phone.raw_inputs")
        if pos is None:
            raise ValueError("pos must be present in action")

        # Gripper: ABSOLUTE position from the holding slider. a3 in [0,1]
        # (0=closed, 1=open) maps to the gripper joint range [0,100]. Until the
        # operator first touches the slider the phone omits a3 -> hold the gripper
        # at its measured position so it never jolts on connect.
        if "a3" in inputs:
            grip_pos = float(np.clip(float(inputs["a3"]), 0.0, 1.0)) * 100.0
        else:
            _obs = self.transition.get(TransitionKey.OBSERVATION)
            grip_pos = (float(_obs["gripper.pos"])
                        if _obs is not None and "gripper.pos" in _obs else 50.0)
        phone_roll_deg = float(inputs.get("_phone_roll_deg", 0.0))
        phone_pitch_deg = float(inputs.get("_phone_pitch_deg", 0.0))
        yaw_delta_deg = float(inputs.get("_yaw_delta_deg", 0.0))
        clutch_rising = enabled and not self._prev_enabled
        motion_unlocked = False
        cyl_active = False
        reach = up = yaw_delta_rad = az = raw_r = r = h = None
        radius_limited = False
        radial_overpull_m = 0.0
        arm_goal_gap_m = None
        reset_arm_goal_to_measured = False

        _, T = _measured_qT(self)
        meas = T[:3, 3] if T is not None else None
        if meas is not None and _LATEST_ARM_GOAL_EE_POS is not None:
            arm_goal_gap_m = float(np.linalg.norm(_LATEST_ARM_GOAL_EE_POS - meas))
        if self._ee_hold is None and meas is not None:
            self._ee_hold = meas.copy()
        if not enabled and meas is not None:
            # Clutch off / on release: anchor the hold target to where the arm
            # ACTUALLY is, not the last commanded target (which leads the arm by
            # the motor lag). Otherwise releasing mid-move leaves a lag-sized gap
            # and the tracking bar wrongly reads "stuck".
            # NB: this also causes a small qgoal jump on release (IK(meas)~=qmeas
            # collapses the lag in one frame) -- investigated 2026-06-06; trying
            # to avoid it by keeping _ee_hold at last commanded EE caused the
            # arm to follow-through past the release point and each clutch cycle
            # locked-in the overshoot, drifting cumulatively. The snap stays.
            self._ee_hold = meas.copy()

        if clutch_rising:
            # B1 rising: capture the placement baseline immediately, but don't
            # emit a new cyl target until the phone actually moves.
            if meas is not None:
                # Re-engage means "the physical arm is here now". A stale
                # arm_goal can be hundreds of mm away if the operator released
                # while the arm was still catching up. Using that as the next
                # cylindrical baseline makes the first tiny phone motion command
                # a large move. Reset baseline fallbacks and IK _prev to measured.
                base = meas.copy()
                _LATEST_ARM_GOAL_EE_POS = meas.copy()
                self._last_cmd_ee = meas.copy()
                self._ee_hold = meas.copy()
                self._baseline_source_name = "measured_reengage"
                reset_arm_goal_to_measured = True
                _browser_phone_mod._record_session_event("arm_goal_reset", {
                    "trigger": "b1_rising",
                    "previous_goal_gap_m": (
                        None if arm_goal_gap_m is None else round(arm_goal_gap_m, 4)
                    ),
                })
            else:
                base, self._baseline_source_name = self._baseline_source(meas)
            self._capture_cyl_baseline(base)
            self._has_moved_since_clutch = False

        # First-motion detector. The baseline is already captured on B1 rising;
        # this only gates when we start emitting cyl targets from phone deltas.
        # Only command-producing axes count: lateral phone motion is ignored by
        # the cylindrical mapping, so letting lateral tracker noise unlock this
        # gate caused small re-clutch taps.
        _MOTION_EPS_M = 0.002       # 2mm of phone displacement
        _MOTION_EPS_YAW_DEG = 0.5   # half a degree of yaw delta
        if enabled and not self._has_moved_since_clutch:
            moved = (
                abs(float(pos[1])) > _MOTION_EPS_M
                or abs(float(pos[2])) > _MOTION_EPS_M
                or abs(yaw_delta_deg) > _MOTION_EPS_YAW_DEG
            )
            if moved:
                self._has_moved_since_clutch = True
                motion_unlocked = True

        if enabled and self._has_moved_since_clutch and self._az0 is None:
            # If no measurement existed on the rising edge, recover as soon as
            # one is available rather than leaving the clutch inert forever.
            base, self._baseline_source_name = self._baseline_source(meas)
            self._capture_cyl_baseline(base)

        if enabled and self._has_moved_since_clutch and self._az0 is not None:
            reach = -float(pos[1])
            up = float(pos[2])
            # Negative sign so the turret rotates the SAME way the operator turns
            # the phone (yaw right -> arm swings right). Without it the azimuth
            # ran opposite to the hand, which read as "backwards" to the operator.
            yaw_delta_rad = float(np.radians(CYL_YAW_GAIN * yaw_delta_deg))
            az = self._az0 - yaw_delta_rad
            raw_r = self._r0 + reach
            r = max(raw_r, CYL_MIN_RADIUS_M)
            radius_limited = r > raw_r
            radial_overpull_m = max(0.0, CYL_MIN_RADIUS_M - raw_r)
            h = self._h0 + up
            ee = np.array([r * np.cos(az), r * np.sin(az), h], dtype=float)
            self._ee_hold = ee.copy()
            self._last_cmd_ee = ee.copy()  # anchor for the NEXT clutch cycle
            # Hand the continuous turret command to the closed-form IK so it
            # never has to recover azimuth from the Cartesian point near r=0.
            action["cyl.az"] = float(az)
            action["cyl.r"] = float(r)
            action["cyl.h"] = float(h)
            cyl_active = True
            if radius_limited != self._radius_limited_prev:
                _browser_phone_mod._record_session_event("cyl_radius_limited", {
                    "limited": int(radius_limited),
                    "raw_r_m": round(raw_r, 4),
                    "r_m": round(r, 4),
                    "min_r_m": round(CYL_MIN_RADIUS_M, 4),
                    "radial_overpull_m": round(radial_overpull_m, 4),
                    "r0_m": round(self._r0, 4),
                    "reach_m": round(reach, 4),
                })
            self._radius_limited_prev = radius_limited
        elif self._ee_hold is not None:
            ee = self._ee_hold
            self._radius_limited_prev = False
        else:
            ee = np.zeros(3, dtype=float)
            self._radius_limited_prev = False

        self._prev_enabled = enabled

        action["enabled"] = enabled
        action["ee.x"] = float(ee[0])
        action["ee.y"] = float(ee[1])
        action["ee.z"] = float(ee[2])
        action["ee.wx"] = 0.0
        action["ee.wy"] = 0.0
        action["ee.wz"] = 0.0
        action["ee.gripper_pos"] = grip_pos
        action[_KEY_TRIM_PHONE_ENABLED] = enabled
        action[_KEY_TRIM_PHONE_ROLL_DEG] = phone_roll_deg
        action[_KEY_TRIM_PHONE_PITCH_DEG] = phone_pitch_deg
        if reset_arm_goal_to_measured:
            action[_KEY_RESET_ARM_GOAL_TO_MEASURED] = True
        _LATEST_CYL_DEBUG = {
            "cyl_enabled": int(enabled),
            "cyl_clutch_rising": int(clutch_rising),
            "cyl_reset_arm_goal_to_measured": int(reset_arm_goal_to_measured),
            "cyl_arm_goal_gap_m": arm_goal_gap_m,
            "cyl_motion_unlocked": int(motion_unlocked),
            "cyl_has_moved": int(self._has_moved_since_clutch),
            "cyl_active": int(cyl_active),
            "cyl_baseline_source": self._baseline_source_name,
            "cyl_az0_rad": self._az0,
            "cyl_r0_m": self._r0 if self._az0 is not None else None,
            "cyl_h0_m": self._h0 if self._az0 is not None else None,
            "cyl_reach_m": reach,
            "cyl_up_m": up,
            "cyl_yaw_delta_deg": yaw_delta_deg,
            "cyl_yaw_delta_rad": yaw_delta_rad,
            "cyl_az_rad": az,
            "cyl_raw_r_m": raw_r,
            "cyl_r_m": r,
            "cyl_min_r_m": CYL_MIN_RADIUS_M,
            "cyl_radius_limited": int(radius_limited),
            "cyl_radial_overpull_m": radial_overpull_m,
            "cyl_h_m": h,
            "cyl_ee_x": float(ee[0]),
            "cyl_ee_y": float(ee[1]),
            "cyl_ee_z": float(ee[2]),
        }
        return action

class ClutchAndWristControl(RobotActionProcessorStep):
    """Three jobs that run together in one pipeline step:

    1. Cartesian EE hold on clutch release -- caches the last commanded EE and
       overwrites ee.x/y/z while phone_enabled is False so the arm doesn't
       drift when the operator lets go of B1.
    2. Phone roll -> wrist_roll  delta integration (positional mapping, gain
       PHONE_ROLL_GAIN).
    3. Phone pitch -> wrist_flex delta integration (positional, PHONE_PITCH_GAIN).

    Also publishes the shared globals _LATEST_EE_TARGET_POS, _PLACEMENT_PHONE_ENABLED,
    and _PLACEMENT_WRIST_TARGET_DEG so ClosedFormArmIK and the trajectory logger
    can read them without threading state through the action dict.
    """

    def __init__(self, motor_names: list[str]):
        super().__init__()
        self.motor_names = motor_names
        self._prev_phone_enabled = False
        self._held_pos: np.ndarray | None = None
        self._wrist_target: dict[str, float] = {}
        self._phone_roll_prev: float | None = None  # for incremental phone-roll -> wrist_roll
        self._phone_pitch_prev: float | None = None  # for incremental phone-pitch -> wrist_flex
        self._last_print_t = 0.0

    def _init_wrist_from_observation(self, observation: RobotObservation) -> None:
        for joint in ("wrist_flex", "wrist_roll"):
            key = f"{joint}.pos"
            if key in observation:
                self._wrist_target[joint] = float(observation[key])

    def action(self, action: RobotAction) -> RobotAction:
        global _LATEST_EE_TARGET_POS, _PLACEMENT_WRIST_TARGET_DEG, _LATEST_WRIST_DEBUG

        phone_enabled = bool(action.pop(_KEY_TRIM_PHONE_ENABLED, False))
        phone_roll_deg = float(action.pop(_KEY_TRIM_PHONE_ROLL_DEG, 0.0))
        phone_pitch_deg = float(action.pop(_KEY_TRIM_PHONE_PITCH_DEG, 0.0))
        reset_arm_goal_to_measured = bool(action.pop(_KEY_RESET_ARM_GOAL_TO_MEASURED, False))
        roll_delta_deg = None
        pitch_delta_deg = None

        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            self._prev_phone_enabled = False
            self._held_pos = None
            _PLACEMENT_WRIST_TARGET_DEG = None
            _LATEST_WRIST_DEBUG = {}
            return action

        if not self._wrist_target:
            self._init_wrist_from_observation(observation)

        try:
            action_pos = np.array([
                float(action["ee.x"]),
                float(action["ee.y"]),
                float(action["ee.z"]),
            ], dtype=float)
        except (KeyError, TypeError, ValueError):
            action_pos = None

        if phone_enabled:
            self._held_pos = None
        elif action_pos is not None:
            if self._prev_phone_enabled or self._held_pos is None:
                self._held_pos = action_pos.copy()
            action["ee.x"] = float(self._held_pos[0])
            action["ee.y"] = float(self._held_pos[1])
            action["ee.z"] = float(self._held_pos[2])

        # Phone ROLL -> wrist_roll. Incremental from the absolute gravity-referenced
        # roll (telescoping -> drift/jitter-immune), only while enabled. On the
        # clutch edge, seed the previous phone orientation without integrating so
        # off-clutch hand repositioning can't create a wrist tap on re-grab.
        if "wrist_roll" in self._wrist_target:
            if phone_enabled and self._prev_phone_enabled and self._phone_roll_prev is not None:
                d = (phone_roll_deg - self._phone_roll_prev + 180.0) % 360.0 - 180.0
                roll_delta_deg = d
                self._wrist_target["wrist_roll"] += PHONE_ROLL_GAIN * d
                lo, hi = _JOINT_LIMITS_DEG["wrist_roll"]
                self._wrist_target["wrist_roll"] = float(
                    np.clip(self._wrist_target["wrist_roll"], lo, hi))
            self._phone_roll_prev = phone_roll_deg

        # Phone PITCH -> wrist_flex. Mirrors the wrist_roll integration: delta-from-
        # absolute, so the mapping is positional (tilt 20deg -> wrist 10deg, return
        # tilt -> wrist returns) while the clutch is held. The rising-edge sample is
        # only used as the new reference, matching wrist_roll above.
        if "wrist_flex" in self._wrist_target:
            if phone_enabled and self._prev_phone_enabled and self._phone_pitch_prev is not None:
                d = phone_pitch_deg - self._phone_pitch_prev
                pitch_delta_deg = d
                self._wrist_target["wrist_flex"] += PHONE_PITCH_GAIN * d
                lo, hi = _JOINT_LIMITS_DEG["wrist_flex"]
                self._wrist_target["wrist_flex"] = float(
                    np.clip(self._wrist_target["wrist_flex"], lo, hi))
            self._phone_pitch_prev = phone_pitch_deg

        _LATEST_EE_TARGET_POS = np.array([
            float(action["ee.x"]),
            float(action["ee.y"]),
            float(action["ee.z"]),
        ], dtype=float)
        if reset_arm_goal_to_measured:
            action[_KEY_RESET_ARM_GOAL_TO_MEASURED] = True
        _PLACEMENT_WRIST_TARGET_DEG = dict(self._wrist_target)
        _LATEST_WRIST_DEBUG = {
            "wrist_phone_enabled": int(phone_enabled),
            "wrist_reset_arm_goal_to_measured": int(reset_arm_goal_to_measured),
            "wrist_phone_roll_deg": phone_roll_deg,
            "wrist_phone_pitch_deg": phone_pitch_deg,
            "wrist_roll_delta_deg": roll_delta_deg,
            "wrist_pitch_delta_deg": pitch_delta_deg,
            "wrist_roll_target_deg": self._wrist_target.get("wrist_roll"),
            "wrist_flex_target_deg": self._wrist_target.get("wrist_flex"),
        }

        now_t = time.time()
        if now_t - self._last_print_t > _EE_DEBUG_PRINT_PERIOD_S:
            self._last_print_t = now_t
            hold = "phone" if phone_enabled else "held"
            print(
                "[placement_trim] "
                f"pos={hold} "
                f"wrist={ {k: round(v, 1) for k, v in self._wrist_target.items()} }"
            )

        self._prev_phone_enabled = phone_enabled
        return action

class ClosedFormArmIK(RobotActionProcessorStep):
    """Analytic turret IK.

    Reads cyl.az/r/h (stashed by CylindricalPhoneToEE) plus the locked wrist
    target, and writes pan/lift/elbow/wrist joint goals. Holds the last arm goal
    when disabled; the per-frame joint rate cap bounds commanded motion.
    """

    def __init__(self, kinematics, motor_names: list[str]):
        super().__init__()
        self.kinematics = kinematics
        self.motor_names = motor_names
        self._prev = None  # last commanded [pan, lift, elbow]
        # Joint indices into the FK joint vector. Cached for the wrist-roll
        # compensation FK calls, which run twice per IK tick.
        self._idx_pan = motor_names.index("shoulder_pan")
        self._idx_lift = motor_names.index("shoulder_lift")
        self._idx_elbow = motor_names.index("elbow_flex")
        self._idx_wflex = motor_names.index("wrist_flex") if "wrist_flex" in motor_names else None
        self._idx_wroll = motor_names.index("wrist_roll") if "wrist_roll" in motor_names else None

    def _compensate_for_wrist_roll(self, az: float, r: float, h: float,
                                    wr_deg: float, wf_deg: float) -> tuple[float, float, float]:
        """Adjust the cyl target so the EE TIP -- not the wrist mount -- ends
        up where the operator commanded. The closed-form IK ignores the wrist
        when solving pan/lift/elbow (passes wf=0), so the tip swings in a
        circle of radius ~8 mm around the wrist_roll axis as the operator
        twists. Measured 2026-06-15: swing radius is a rigid 7.9 mm across
        the workspace. Compensation: first-pass IK to pick a pose, FK twice
        (once at wr=0, once at wr=wr_deg) to get the world-frame swing
        vector, then subtract it from the cartesian target and convert back
        to cyl. One iteration converges to sub-mm because the swing is tiny
        relative to arm reach.
        """
        if self._idx_wroll is None:
            return az, r, h
        try:
            pan0, lift0, elbow0 = _closed_form_arm_ik(az, r, h, 0.0)
        except Exception:
            return az, r, h
        # FK requires a full joint vector. Zeros for everything else is fine
        # for the wrist_roll-induced tip offset since gripper doesn't move
        # the tip and pan/lift/elbow/wflex are the actual planned values.
        q_base = np.zeros(len(self.motor_names))
        q_base[self._idx_pan] = pan0
        q_base[self._idx_lift] = lift0
        q_base[self._idx_elbow] = elbow0
        if self._idx_wflex is not None:
            q_base[self._idx_wflex] = wf_deg
        q_zero = q_base.copy()
        q_roll = q_base.copy()
        q_roll[self._idx_wroll] = wr_deg
        try:
            tip_zero = self.kinematics.forward_kinematics(q_zero)[:3, 3]
            tip_roll = self.kinematics.forward_kinematics(q_roll)[:3, 3]
        except Exception:
            return az, r, h
        swing = tip_roll - tip_zero
        ee_cart = np.array([r * np.cos(az), r * np.sin(az), h])
        ee_comp = ee_cart - swing
        az_c = float(np.arctan2(ee_comp[1], ee_comp[0]))
        r_c = float(np.hypot(ee_comp[0], ee_comp[1]))
        h_c = float(ee_comp[2])
        return az_c, r_c, h_c

    def _publish_goal_ee(self, observation: RobotObservation, cmd: np.ndarray, wr: float) -> None:
        global _LATEST_ARM_GOAL_EE_POS
        try:
            q = np.array([float(observation[f"{m}.pos"]) for m in self.motor_names],
                         dtype=float)
            for i, motor in enumerate(self.motor_names):
                if motor == "shoulder_pan":
                    q[i] = float(cmd[0])
                elif motor == "shoulder_lift":
                    q[i] = float(cmd[1])
                elif motor == "elbow_flex":
                    q[i] = float(cmd[2])
                elif motor == "wrist_flex":
                    q[i] = 0.0
                elif motor == "wrist_roll":
                    q[i] = wr
            T = self.kinematics.forward_kinematics(q)
            _LATEST_ARM_GOAL_EE_POS = T[:3, 3].copy()
        except (KeyError, TypeError, ValueError):
            pass

    def action(self, action: RobotAction) -> RobotAction:
        global _LATEST_IK_DEBUG

        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            _LATEST_IK_DEBUG = {}
            return action
        try:
            control_dt_s = float(action.pop(_KEY_CONTROL_DT_S, 1.0 / FPS))
        except (TypeError, ValueError):
            control_dt_s = 1.0 / FPS
        control_dt_s = float(np.clip(control_dt_s, 1.0 / FPS, CONTROL_MAX_RATE_DT_S))
        max_step_deg = MAX_CMD_DEG_PER_SEC * control_dt_s
        reset_to_measured = bool(action.pop(_KEY_RESET_ARM_GOAL_TO_MEASURED, False))
        enabled = bool(action.get("enabled", False))
        wt = _PLACEMENT_WRIST_TARGET_DEG or {}
        wf = float(wt.get("wrist_flex", float(observation.get("wrist_flex.pos", 0.0))))
        wr = float(wt.get("wrist_roll", float(observation.get("wrist_roll.pos", 0.0))))
        ik_mode = "unknown"
        az_in = r_in = h_in = az_c = r_c = h_c = None
        cmd_pre_rate = None
        rate_limited = 0
        max_delta_deg = None
        if reset_to_measured:
            cmd = np.array([float(observation["shoulder_pan.pos"]),
                            float(observation["shoulder_lift.pos"]),
                            float(observation["elbow_flex.pos"])])
            cmd_pre_rate = cmd.copy()
            max_delta_deg = 0.0
            rate_limited = 0
            ik_mode = "measured_reengage"
        elif enabled and "cyl.r" in action:
            # Wrist flex is DIRECT: solve lift/elbow as if the wrist were straight
            # (pass 0) so flexing only angles the gripper -- it never reconfigures the
            # arm. (Tip-control folded wf into the solve and swung lift+elbow ~98deg
            # per 66deg flex, leaving the arm stuck/sluggish.) The actual flex still
            # goes to the wrist joint below.
            # Wrist ROLL is different: the rigid 7.9 mm tool offset from the
            # roll axis means an operator twist swings the tip in an arc even
            # though their phone hasn't moved. Compensate by shifting the
            # cyl target so the tip (not the wrist mount) follows phone_pos.
            az_in = float(action["cyl.az"])
            r_in  = float(action["cyl.r"])
            h_in  = float(action["cyl.h"])
            az_c, r_c, h_c = self._compensate_for_wrist_roll(az_in, r_in, h_in, wr, wf)
            pan, lift, elbow = _closed_form_arm_ik(az_c, r_c, h_c, 0.0)
            cmd_pre_rate = np.array([pan, lift, elbow])
            cmd = cmd_pre_rate.copy()
            if self._prev is not None:
                delta = cmd_pre_rate - self._prev
                max_delta_deg = float(np.max(np.abs(delta)))
                clipped = np.clip(delta, -max_step_deg, max_step_deg)
                rate_limited = int(not np.allclose(clipped, delta))
                cmd = self._prev + clipped
            else:
                max_delta_deg = 0.0
            ik_mode = "solve"
        elif self._prev is not None:
            # Clutch off: HOLD the last commanded goal. The arm finishes its
            # in-flight motion and glides to rest there (the goal led the arm by
            # the motor lag, so it's right where the arm was heading). Snapping to
            # the *measured* pose instead makes the moving arm overshoot it and get
            # yanked back -- the "tap" on release.
            cmd = self._prev.copy()
            cmd_pre_rate = cmd.copy()
            max_delta_deg = 0.0
            ik_mode = "hold"
        else:
            cmd = np.array([float(observation["shoulder_pan.pos"]),
                            float(observation["shoulder_lift.pos"]),
                            float(observation["elbow_flex.pos"])])
            cmd_pre_rate = cmd.copy()
            max_delta_deg = 0.0
            ik_mode = "seed"
        self._prev = cmd.copy()
        self._publish_goal_ee(observation, cmd, wr)
        _LATEST_IK_DEBUG = {
            "ik_mode": ik_mode,
            "ik_enabled": int(enabled),
            "ik_reset_to_measured": int(reset_to_measured),
            "ik_az_in_rad": az_in,
            "ik_r_in_m": r_in,
            "ik_h_in_m": h_in,
            "ik_az_comp_rad": az_c,
            "ik_r_comp_m": r_c,
            "ik_h_comp_m": h_c,
            "ik_pan_pre_deg": float(cmd_pre_rate[0]),
            "ik_lift_pre_deg": float(cmd_pre_rate[1]),
            "ik_elbow_pre_deg": float(cmd_pre_rate[2]),
            "ik_rate_limited": rate_limited,
            "ik_max_delta_deg": max_delta_deg,
            "ik_max_step_deg": max_step_deg,
            "ik_control_dt_s": control_dt_s,
            "ik_wrist_flex_deg": wf,
            "ik_wrist_roll_deg": wr,
        }
        action["shoulder_pan.pos"] = float(cmd[0])
        action["shoulder_lift.pos"] = float(cmd[1])
        action["elbow_flex.pos"] = float(cmd[2])
        action["wrist_flex.pos"] = wf
        action["wrist_roll.pos"] = wr
        # Gripper: CylindricalPhoneToEE supplies an ABSOLUTE position in
        # ee.gripper_pos; convert it to the joint goal.
        if "ee.gripper_pos" in action:
            action["gripper.pos"] = float(action.pop("ee.gripper_pos"))
        return action

# --- Trajectory logger -----------------------------------------------------
_TRAJECTORY = {"phone": [], "ee": [], "desired_ee": [], "ee_from_goal": [],
               "q_meas": [], "q_goal": []}
_KEEP_IN_MEMORY_TRAJECTORY = os.environ.get(
    "PHONE_ARM_IN_MEMORY_TRAJECTORY", "0"
) == "1"
_STEP_SIZE = 1.0  # fallback trajectory scaling if no desired EE target is available
_TRACKING_FEEDBACK_PERIOD_S = 0.1
_RECORDING_STOP_HOLDOFF_S = 0.75
_RECORDING_GRIPPER_COMMAND_DELTA_DEG = 0.5
_RECORDING_GRIPPER_COMMAND_SPEED_DPS = 2.0
_RECORDING_ARM_SETTLE_ERROR_M = 0.005
_RECORDING_ARM_MEASURED_SPEED_DPS = 2.0
_RECORDING_GRIPPER_MEASURED_SPEED_DPS = 2.0


class _TrajectoryLogger:
    """Wraps the teleop pipeline to record per-frame phone/desired/actual/IK
    EE positions for offline tracking-error analysis."""

    def __init__(self, inner, kin, motor_names, feedback_sink=None):
        self.inner = inner
        self.kin = kin
        self.motor_names = motor_names
        self.feedback_sink = feedback_sink
        self._latched_ref = None
        self._prev_enabled = False
        self._last_feedback_t = 0.0
        self._prev_q_goal = None
        self._prev_q_obs = None
        self._prev_motion_t = None
        self._recording_active = False
        self._recording_quiet_since_t = None
        self._recording_seq = 0
        self._gripper_idx = (
            self.motor_names.index("gripper")
            if "gripper" in self.motor_names else None
        )
        self._arm_indices = [
            idx for idx, name in enumerate(self.motor_names)
            if name != "gripper"
        ]

    def _publish_tracking_feedback(
        self,
        *,
        display_error_m: float | None,
        command_error_m: float | None,
        goal_error_m: float | None,
        raw_inputs: dict,
        phone_enabled: bool,
        error_source: str,
    ) -> None:
        if self.feedback_sink is None:
            return
        now = time.perf_counter()
        if now - self._last_feedback_t < _TRACKING_FEEDBACK_PERIOD_S:
            return
        self._last_feedback_t = now
        display_error_m = (
            float(display_error_m)
            if display_error_m is not None and np.isfinite(display_error_m)
            else None
        )
        command_error_m = (
            float(command_error_m)
            if command_error_m is not None and np.isfinite(command_error_m)
            else None
        )
        goal_error_m = (
            float(goal_error_m)
            if goal_error_m is not None and np.isfinite(goal_error_m)
            else None
        )
        # Browser send-time of the pose that triggered this tracking error,
        # if the ingest path preserved it. Echoed straight back to the client
        # so it can compute latency against its OWN clock -- resilient to Pi
        # wall-clock skew / NTP resyncs and to per-hop network jitter.
        t_browser_ms = raw_inputs.get("_t_browser_ms")
        try:
            self.feedback_sink.send_feedback({
                "robot_tracking_error_m": display_error_m,
                "robot_tracking_command_error_m": command_error_m,
                "robot_tracking_goal_error_m": goal_error_m,
                "robot_tracking_phone_enabled": int(phone_enabled),
                "robot_tracking_error_source": error_source,
                "robot_tracking_enabled": 1,
                "robot_tracking_seq": raw_inputs.get("_seq"),
                "robot_tracking_t_pi_ms": time.time() * 1000.0,
                "robot_tracking_t_browser_ms": t_browser_ms,
            })
        except Exception:
            pass

    @staticmethod
    def _finite_float(value) -> float | None:
        if value is None:
            return None
        try:
            out = float(value)
        except (TypeError, ValueError):
            return None
        return out if np.isfinite(out) else None

    def _recording_event_details(
        self,
        *,
        reason: str,
        phone_enabled: bool,
        raw_inputs: dict,
        metrics: dict,
    ) -> dict:
        details = {
            "reason": reason,
            "phone_enabled": int(phone_enabled),
            "pose_seq": raw_inputs.get("_seq"),
            "b1_edge_seq": raw_inputs.get("_b1_edge_seq"),
            "t_pi_ms": round(time.time() * 1000.0, 3),
        }
        for key, value in metrics.items():
            details[key] = self._finite_float(value)
        return details

    def _update_recording_gate(
        self,
        *,
        phone_enabled: bool,
        q_obs: np.ndarray | None,
        q_goal: np.ndarray | None,
        goal_error_m: float | None,
        raw_inputs: dict,
    ) -> tuple[bool, str, dict]:
        """Drive dataset/video recording from robot motion, not only B1."""
        now_t = time.perf_counter()
        dt_s = (
            now_t - self._prev_motion_t
            if self._prev_motion_t is not None else None
        )
        metrics = {
            "recording_goal_error_m": goal_error_m,
            "recording_gripper_command_delta_deg": None,
            "recording_gripper_command_speed_dps": None,
            "recording_arm_measured_speed_dps": None,
            "recording_gripper_measured_speed_dps": None,
        }
        trigger_reasons: list[str] = []
        settle_reasons: list[str] = []
        if phone_enabled:
            trigger_reasons.append("b1")

        if q_goal is not None and self._gripper_idx is not None:
            idx = self._gripper_idx
            if self._prev_q_goal is not None:
                delta = abs(float(q_goal[idx] - self._prev_q_goal[idx]))
                metrics["recording_gripper_command_delta_deg"] = delta
                command_speed_dps = (
                    delta / dt_s
                    if dt_s is not None and dt_s > 0.0 else None
                )
                metrics["recording_gripper_command_speed_dps"] = command_speed_dps
                if (
                    delta > _RECORDING_GRIPPER_COMMAND_DELTA_DEG
                    or (
                        command_speed_dps is not None
                        and command_speed_dps > _RECORDING_GRIPPER_COMMAND_SPEED_DPS
                    )
                ):
                    trigger_reasons.append("gripper_command")
            elif "a3" in raw_inputs and q_obs is not None:
                delta = abs(float(q_goal[idx] - q_obs[idx]))
                metrics["recording_gripper_command_delta_deg"] = delta
                if delta > _RECORDING_GRIPPER_COMMAND_DELTA_DEG:
                    trigger_reasons.append("gripper_command")

        if (
            self._recording_active
            and not phone_enabled
            and q_obs is not None
            and self._prev_q_obs is not None
            and dt_s is not None
            and dt_s > 0.0
        ):
            if self._arm_indices:
                arm_delta_deg = float(np.max(np.abs(
                    q_obs[self._arm_indices] - self._prev_q_obs[self._arm_indices]
                )))
                arm_speed_dps = arm_delta_deg / dt_s
                metrics["recording_arm_measured_speed_dps"] = arm_speed_dps
                if (
                    arm_speed_dps > _RECORDING_ARM_MEASURED_SPEED_DPS
                    and (
                        goal_error_m is None
                        or goal_error_m > _RECORDING_ARM_SETTLE_ERROR_M
                    )
                ):
                    settle_reasons.append("arm_settling")
            if self._gripper_idx is not None:
                idx = self._gripper_idx
                gripper_delta_deg = abs(float(q_obs[idx] - self._prev_q_obs[idx]))
                gripper_speed_dps = gripper_delta_deg / dt_s
                metrics["recording_gripper_measured_speed_dps"] = gripper_speed_dps
                if gripper_speed_dps > _RECORDING_GRIPPER_MEASURED_SPEED_DPS:
                    settle_reasons.append("gripper_settling")

        reasons = trigger_reasons + settle_reasons
        active_now = bool(reasons)
        reason_text = "+".join(reasons)
        if active_now:
            self._recording_quiet_since_t = None
            if not self._recording_active:
                self._recording_active = True
                self._recording_seq += 1
                details = self._recording_event_details(
                    reason=reason_text,
                    phone_enabled=phone_enabled,
                    raw_inputs=raw_inputs,
                    metrics=metrics,
                )
                details["recording_seq"] = self._recording_seq
                _browser_phone_mod._record_session_event("recording_start", details)
        elif self._recording_active:
            if self._recording_quiet_since_t is None:
                self._recording_quiet_since_t = now_t
            quiet_s = now_t - self._recording_quiet_since_t
            metrics["recording_quiet_s"] = quiet_s
            if quiet_s >= _RECORDING_STOP_HOLDOFF_S:
                details = self._recording_event_details(
                    reason="quiet",
                    phone_enabled=phone_enabled,
                    raw_inputs=raw_inputs,
                    metrics=metrics,
                )
                details["recording_seq"] = self._recording_seq
                _browser_phone_mod._record_session_event("recording_stop", details)
                self._recording_active = False
                self._recording_quiet_since_t = None
            else:
                reason_text = "quiet_holdoff"

        if q_goal is not None:
            self._prev_q_goal = q_goal.copy()
        if q_obs is not None:
            self._prev_q_obs = q_obs.copy()
        self._prev_motion_t = now_t
        return self._recording_active, reason_text, metrics

    def __call__(self, *args, **kwargs):
        sampled = False
        phone_enabled = False
        phone_pos = None
        ee_meas = None
        desired_ee = None
        q_obs = None
        raw_inputs = {}
        if args and isinstance(args[0], tuple) and len(args[0]) == 2:
            raw_action, obs = args[0]
            try:
                phone_enabled = bool(raw_action.get("phone.enabled"))
                raw_inputs = dict(raw_action.get("phone.raw_inputs", {}))
                phone_pos = np.asarray(raw_action["phone.pos"], dtype=float)
                q_obs = np.array([obs[f"{m}.pos"] for m in self.motor_names])
                ee_T = self.kin.forward_kinematics(q_obs)
                ee_meas = ee_T[:3, 3].copy()
                if not self._prev_enabled or self._latched_ref is None:
                    self._latched_ref = ee_meas.copy()
                target = np.array([-phone_pos[1], phone_pos[0], phone_pos[2]])
                desired_ee = (
                    np.asarray(_LATEST_EE_TARGET_POS, dtype=float).copy()
                    if _LATEST_EE_TARGET_POS is not None
                    else self._latched_ref + target * _STEP_SIZE
                )
                sampled = True
                self._prev_enabled = phone_enabled
            except (KeyError, TypeError, ValueError):
                pass

        out = self.inner(*args, **kwargs)

        if sampled:
            try:
                commanded_ee = (
                    np.asarray(_LATEST_EE_TARGET_POS, dtype=float).copy()
                    if _LATEST_EE_TARGET_POS is not None
                    else desired_ee
                )
                if commanded_ee is not None and np.all(np.isfinite(commanded_ee)):
                    desired_ee = commanded_ee
                if isinstance(out, dict):
                    q_goal = np.array([float(out[f"{m}.pos"]) for m in self.motor_names])
                    ee_goal = self.kin.forward_kinematics(q_goal)
                else:
                    ee_goal = None
                    q_goal = None
            except (KeyError, TypeError, ValueError):
                ee_goal = None
                q_goal = None

            command_error_m = None
            if ee_meas is not None and desired_ee is not None:
                command_error_m = float(np.linalg.norm(desired_ee - ee_meas))
            goal_error_m = None
            if ee_meas is not None and ee_goal is not None:
                goal_error_m = float(np.linalg.norm(ee_goal[:3, 3] - ee_meas))
            recording_active = False
            recording_reason = ""
            recording_metrics = {}
            if q_obs is not None:
                recording_active, recording_reason, recording_metrics = (
                    self._update_recording_gate(
                        phone_enabled=phone_enabled,
                        q_obs=q_obs,
                        q_goal=q_goal,
                        goal_error_m=goal_error_m,
                        raw_inputs=raw_inputs,
                    )
                )
            if phone_enabled:
                display_error_m = command_error_m if command_error_m is not None else goal_error_m
                error_source = "command"
            else:
                display_error_m = goal_error_m if goal_error_m is not None else command_error_m
                error_source = "goal"
            self._publish_tracking_feedback(
                display_error_m=display_error_m,
                command_error_m=command_error_m,
                goal_error_m=goal_error_m,
                raw_inputs=raw_inputs,
                phone_enabled=phone_enabled,
                error_source=error_source,
            )

            # Stream the same row to the session recorder. Join key for pose.csv
            # is t_pi_ms (nearest match; IK ticks ~60 Hz, pose msgs ~30 Hz, so
            # multiple trajectory rows can share a pose). Rows are written while
            # robot motion is being commanded or settling, so gripper-only motion
            # after B1 release still has aligned telemetry and video.
            rec = _browser_phone_mod._ACTIVE_RECORDER
            if rec is not None and recording_active:
                if (ee_meas is None or phone_pos is None
                        or desired_ee is None or q_obs is None):
                    return out
                if _KEEP_IN_MEMORY_TRAJECTORY:
                    _TRAJECTORY["phone"].append(phone_pos.copy())
                    _TRAJECTORY["ee"].append(ee_meas.copy())
                    _TRAJECTORY["desired_ee"].append(desired_ee.copy())
                    _TRAJECTORY["q_meas"].append(q_obs.copy())
                    _TRAJECTORY["ee_from_goal"].append(
                        ee_goal[:3, 3].copy()
                        if ee_goal is not None else np.full(3, np.nan)
                    )
                    _TRAJECTORY["q_goal"].append(
                        q_goal.copy()
                        if q_goal is not None else np.full(len(self.motor_names), np.nan)
                    )
                row = {
                    "t_pi_ms": round(time.time() * 1000.0, 3),
                    "recording_active": int(recording_active),
                    "recording_reason": recording_reason,
                    "phone_enabled": int(phone_enabled),
                    "phone_x": float(phone_pos[0]),
                    "phone_y": float(phone_pos[1]),
                    "phone_z": float(phone_pos[2]),
                    "desired_ee_x": float(desired_ee[0]),
                    "desired_ee_y": float(desired_ee[1]),
                    "desired_ee_z": float(desired_ee[2]),
                    "ee_meas_x": float(ee_meas[0]),
                    "ee_meas_y": float(ee_meas[1]),
                    "ee_meas_z": float(ee_meas[2]),
                    "tracking_error_m": command_error_m,
                    "goal_tracking_error_m": goal_error_m,
                    "recording_goal_error_m": recording_metrics.get(
                        "recording_goal_error_m"
                    ),
                    "recording_gripper_command_delta_deg": recording_metrics.get(
                        "recording_gripper_command_delta_deg"
                    ),
                    "recording_gripper_command_speed_dps": recording_metrics.get(
                        "recording_gripper_command_speed_dps"
                    ),
                    "recording_arm_measured_speed_dps": recording_metrics.get(
                        "recording_arm_measured_speed_dps"
                    ),
                    "recording_gripper_measured_speed_dps": recording_metrics.get(
                        "recording_gripper_measured_speed_dps"
                    ),
                    "recording_quiet_s": recording_metrics.get("recording_quiet_s"),
                    "phone_a3": raw_inputs.get("a3"),
                    "pose_seq": raw_inputs.get("_seq"),
                    "b1_edge_seq": raw_inputs.get("_b1_edge_seq"),
                    "b1_edge_reason": raw_inputs.get("_b1_edge_reason"),
                    "b1_edge_t_ms": raw_inputs.get("_b1_edge_t_ms"),
                    "t_browser_write_ms": raw_inputs.get("_t_browser_write_ms"),
                    "command_age_at_ik_ms": raw_inputs.get("_pose_age_ms"),
                    "pose_stale": raw_inputs.get("_pose_stale"),
                    "source_age_ms": raw_inputs.get("_source_age_ms"),
                    "browser_queue_ms": raw_inputs.get("_browser_queue_ms"),
                    "browser_write_to_edge_ms": raw_inputs.get("_browser_write_to_edge_ms"),
                    "browser_write_to_relay_ms": raw_inputs.get("_browser_write_to_relay_ms"),
                    "edge_queue_ms": raw_inputs.get("_edge_q_ms"),
                    "edge_to_relay_ms": raw_inputs.get("_edge_to_relay_ms"),
                    "phone_to_relay_ms": raw_inputs.get("_phone_to_relay_ms"),
                    "relay_queue_age_ms": raw_inputs.get("_relay_queue_age_ms"),
                    "relay_to_pi_ms": raw_inputs.get("_relay_to_pi_ms"),
                    "relay_total_age_ms": raw_inputs.get("_relay_total_age_ms"),
                    "client_buffered": raw_inputs.get("_client_buffered"),
                    "client_skipped": raw_inputs.get("_client_skipped"),
                    "control_reacquire_required": raw_inputs.get(
                        "_control_reacquire_required"
                    ),
                    "control_hold": raw_inputs.get("_control_hold"),
                }
                for i, mname in enumerate(self.motor_names):
                    row[f"q_meas_{mname}"] = float(q_obs[i])
                if q_goal is not None and ee_goal is not None:
                    row["ee_from_goal_x"] = float(ee_goal[0, 3])
                    row["ee_from_goal_y"] = float(ee_goal[1, 3])
                    row["ee_from_goal_z"] = float(ee_goal[2, 3])
                    for i, mname in enumerate(self.motor_names):
                        row[f"q_goal_{mname}"] = float(q_goal[i])
                else:
                    row["ee_from_goal_x"] = None
                    row["ee_from_goal_y"] = None
                    row["ee_from_goal_z"] = None
                    for mname in self.motor_names:
                        row[f"q_goal_{mname}"] = None
                row.update(dict(_LATEST_CYL_DEBUG))
                row.update(dict(_LATEST_WRIST_DEBUG))
                row.update(dict(_LATEST_IK_DEBUG))
                rec.record_trajectory(row)
        return out

    def __getattr__(self, name):
        return getattr(self.inner, name)


def _force_disable_torque(robot, *, budget_s: float = 30.0) -> None:
    """Best-effort hard shutdown: keep retrying disable_torque() until every
    motor confirms torque-off OR until budget_s elapses. The per-call patient
    retry in _bus_retry already covers normal-day blips; this exists for the
    "bus is genuinely down for several seconds" case (USB renumeration, PSU
    glitch) where 2s isn't enough and the arm would otherwise hold its last
    commanded current at a stretched pose with no operator. Re-prints progress
    so the operator sees we are not stuck."""
    bus = getattr(robot, "bus", None)
    if bus is None:
        print("[teardown] no robot.bus; skipping force-disable")
        return
    deadline = time.time() + budget_s
    attempt = 0
    last_err: BaseException | None = None
    while time.time() < deadline:
        attempt += 1
        try:
            bus.disable_torque()
            print(f"[teardown] disable_torque OK (attempt {attempt})")
            return
        except Exception as e:  # noqa: BLE001 -- want every failure mode
            last_err = e
            remaining = deadline - time.time()
            print(f"[teardown] disable_torque attempt {attempt} failed "
                  f"({e.__class__.__name__}: {e}); {remaining:.1f}s left")
            time.sleep(0.5)
    print(f"[teardown] GIVING UP on disable_torque after {budget_s:.0f}s, "
          f"{attempt} attempts; last err: {last_err}. "
          f"ARM TORQUE MAY STILL BE LATCHED -- power-cycle the motors.")


def _save_trajectory_csv(out_path: str | None = None) -> None:
    if len(_TRAJECTORY["phone"]) < 2:
        return
    import csv
    if out_path is None:
        out_path = os.environ.get(
            "PHONE_ARM_TRAJECTORY_CSV",
            os.path.expanduser("~/phone_arm_logs/teleop_trajectory.csv"),
        )
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    phone = np.array(_TRAJECTORY["phone"])
    ee = np.array(_TRAJECTORY["ee"])
    desired = np.array(_TRAJECTORY["desired_ee"])
    ee_from_goal = (np.array(_TRAJECTORY["ee_from_goal"])
                    if _TRAJECTORY["ee_from_goal"] else np.full_like(ee, np.nan))
    qm = np.array(_TRAJECTORY["q_meas"]) if _TRAJECTORY["q_meas"] else None
    qg = np.array(_TRAJECTORY["q_goal"]) if _TRAJECTORY["q_goal"] else None
    phone_robot = np.column_stack([-phone[:, 1], phone[:, 0], phone[:, 2]])
    n = min(len(phone), len(ee_from_goal))
    jnames = ["pan", "lift", "elbow", "wflex", "wroll", "grip"]
    njoints = qm.shape[1] if qm is not None else 0
    jn = jnames[:njoints]
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "i",
            "phone_robot_x_m", "phone_robot_y_m", "phone_robot_z_m",
            "desired_x_m", "desired_y_m", "desired_z_m",
            "ee_x_m", "ee_y_m", "ee_z_m",
            "ee_from_goal_x_m", "ee_from_goal_y_m", "ee_from_goal_z_m",
            *[f"qmeas_{j}_deg" for j in jn],
            *[f"qgoal_{j}_deg" for j in jn],
        ])
        for i in range(n):
            row = [i, *phone_robot[i].tolist(), *desired[i].tolist(),
                   *ee[i].tolist(), *ee_from_goal[i].tolist()]
            if qm is not None and i < len(qm):
                row += qm[i].tolist()
            if qg is not None and i < len(qg):
                row += qg[i].tolist()
            w.writerow(row)
    print(f"[trajectory] saved {out_path} ({n} samples, {njoints} joints logged)")


# --- Rehome ---------------------------------------------------------------
_KIN_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

# Low-gravity rest pose (read off the physical arm 2026-05-20 via read_pose.py).
# The old all-zeros home put the arm fully horizontal -> shoulder_lift held the
# entire arm at its maximum gravitational moment, ran near-saturated current
# continuously, and tripped the overload latch after a couple hours of idle
# holding. This folded pose (shoulder_lift raised back, elbow folded in) brings
# the forearm's mass close to the shoulder axis so the holding torque -- and
# thus the steady-state current -- is much lower. NOTE: the servo's
# Min/Max_Position_Limit clips Goal to the calibration range (lift to ~-97,
# elbow to ~+96), so the old -110.9/101.4 targets were silently clipped (and
# showed up as the spurious "+13.5deg shoulder_lift home error"). Set the rest
# pose just inside the servo clip = the most-folded (lowest-gravity) reachable.
_REST_POSE_DEG = {
    "shoulder_pan": 8.7,
    "shoulder_lift": -96.5,
    "elbow_flex": 95.0,
    "wrist_flex": 0.9,
    "wrist_roll": -0.6,
}




def _report_home(robot, target_deg: dict, prefix: str, obs: dict) -> None:
    # Per-joint error so we can see WHICH joint can't reach target, plus
    # holding load/temperature so we can confirm the rest pose isn't cooking
    # shoulder_lift. Load/temp reads are best-effort.
    per_joint = "  ".join(
        f"{n}:{obs[f'{n}.pos'] - target_deg[n]:+.1f}" for n in target_deg
    )
    try:
        load = robot.bus.sync_read("Present_Load", normalize=False)
        temp = robot.bus.sync_read("Present_Temperature", normalize=False)
        lt = "  ".join(f"{n}:L{load[n]}/T{temp[n]}C" for n in target_deg)
    except Exception as exc:  # noqa: BLE001
        lt = f"(load/temp read failed: {exc})"
    print(f"[home] {prefix}  err_deg[{per_joint}]")
    print(f"[home] holding  {lt}")


def _drive_to(robot, motor_names, stage_target: dict,
              tolerance_deg: float, timeout_s: float) -> dict:
    """Drive the listed joints in stage_target to those angles; joints not in
    stage_target are commanded to their current measured position (hold).
    Returns the final observation."""
    obs = robot.get_observation()
    action = {}
    for name in motor_names:
        if name in stage_target:
            action[f"{name}.pos"] = float(stage_target[name])
        else:
            action[f"{name}.pos"] = float(obs[f"{name}.pos"])
    deadline = time.perf_counter() + timeout_s
    while True:
        robot.send_action(action)
        obs = robot.get_observation()
        err = max(abs(obs[f"{n}.pos"] - stage_target[n]) for n in stage_target)
        if err < tolerance_deg or time.perf_counter() > deadline:
            return obs
        time.sleep(1.0 / 60)


def _rehome(robot, motor_names, target_deg: dict | None = None,
            tolerance_deg: float = 2.0, timeout_s: float = 15.0) -> None:
    """Two-stage home into a low-gravity rest pose.

    Stage 1 folds elbow_flex + wrist toward target while holding the shoulder
    where it is. Folding the elbow first brings the forearm mass close to the
    shoulder axis, so when stage 2 lifts shoulder_lift it works at a small
    moment arm and can actually reach the target -- even recovering from a
    full droop (torque-off / power loss). Doing it in one shot fails: the arm
    sweeps through the horizontal max-torque region extended and stalls."""
    if target_deg is None:
        target_deg = dict(_REST_POSE_DEG)
    print(f"[home] two-stage home to {target_deg}...")

    # Stage 1: fold elbow + wrist, hold shoulder.
    stage1 = {n: target_deg[n] for n in ("elbow_flex", "wrist_flex", "wrist_roll")
              if n in target_deg}
    if stage1:
        print(f"[home] stage1 fold: {stage1}")
        _drive_to(robot, motor_names, stage1, tolerance_deg, timeout_s * 0.5)

    # Stage 2: full target (shoulder now lifts a folded, low-moment arm).
    print(f"[home] stage2 full: {target_deg}")
    deadline = time.perf_counter() + timeout_s
    while True:
        obs = robot.get_observation()
        action = {}
        for name in motor_names:
            if name in target_deg:
                action[f"{name}.pos"] = float(target_deg[name])
            else:
                action[f"{name}.pos"] = float(obs[f"{name}.pos"])
        robot.send_action(action)
        obs = robot.get_observation()
        err = max(abs(obs[f"{n}.pos"] - target_deg[n]) for n in target_deg)
        if err < tolerance_deg:
            print(f"[home] reached, max err {err:.2f} deg")
            _report_home(robot, target_deg, "reached", obs)
            return
        if time.perf_counter() > deadline:
            print(f"[home] timed out at {timeout_s:.0f}s, max err {err:.2f} deg "
                  "(continuing anyway)")
            _report_home(robot, target_deg, "timeout", obs)
            return
        time.sleep(1.0 / 60)


def _leader_robot_action(
    raw_action: dict,
    obs: dict,
    leader_mapper: RelativeLeaderMapper,
    elapsed_s: float,
) -> dict[str, float]:
    """Produce a bounded follower action from one cached leader frame."""
    leader_positions = raw_action.get("leader.positions")
    leader_enabled = bool(raw_action.get("leader.enabled", False))
    stale = bool(raw_action.get("phone.raw_inputs", {}).get("_pose_stale", 0))
    if not leader_enabled or stale or not isinstance(leader_positions, dict):
        leader_mapper.reset()
        return {
            f"{name}.pos": float(obs[f"{name}.pos"])
            for name in leader_mapper.joint_names
        }

    follower_now = {
        name: float(obs[f"{name}.pos"])
        for name in leader_mapper.joint_names
    }
    desired = leader_mapper.targets(
        leader_positions,
        follower_now,
        source_id=str(raw_action.get("leader.source_id") or "leader"),
    )
    max_step = MAX_CMD_DEG_PER_SEC * min(
        max(float(elapsed_s), 0.0), CONTROL_MAX_RATE_DT_S
    )
    robot_action: dict[str, float] = {}
    for name in leader_mapper.joint_names:
        target = float(desired[name])
        if name == "gripper":
            target = float(np.clip(target, 0.0, 100.0))
        elif name in _JOINT_LIMITS_DEG:
            target = float(np.clip(target, *_JOINT_LIMITS_DEG[name]))
        current = follower_now[name]
        robot_action[f"{name}.pos"] = float(
            np.clip(target, current - max_step, current + max_step)
        )
    return robot_action


def _control_loop(
    *,
    teleop,
    robot,
    fps: int,
    teleop_action_processor,
    robot_action_processor,
    leader_mapper: RelativeLeaderMapper,
) -> None:
    period_s = 1.0 / fps
    watchdog_s = 1.0 / CONTROL_WATCHDOG_HZ
    wait_for_action_update = getattr(teleop, "wait_for_action_update", None)
    last_send_t = time.perf_counter() - period_s
    last_report_t = time.perf_counter()
    count = 0
    update_ticks = 0
    watchdog_ticks = 0
    loop_sum_ms = 0.0
    loop_max_ms = 0.0
    overruns = 0

    while True:
        updated = False
        if callable(wait_for_action_update):
            updated = bool(wait_for_action_update(watchdog_s))
        else:
            time.sleep(watchdog_s)

        now = time.perf_counter()
        elapsed_s = now - last_send_t
        if updated and elapsed_s < period_s:
            # Coalesce bursts while preserving latest-only semantics.
            updated = bool(wait_for_action_update(period_s - elapsed_s)) or updated
            now = time.perf_counter()
            elapsed_s = now - last_send_t

        loop_start = time.perf_counter()
        obs = robot.get_observation()
        raw_action = teleop.get_action()
        raw_action[_KEY_CONTROL_DT_S] = elapsed_s
        if raw_action.get("control.source") == "leader_arm":
            robot_action = _leader_robot_action(
                raw_action, obs, leader_mapper, elapsed_s
            )
        else:
            leader_mapper.reset()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action = robot_action_processor((teleop_action, obs))
        robot.send_action(robot_action)
        last_send_t = time.perf_counter()

        loop_ms = (time.perf_counter() - loop_start) * 1000.0
        count += 1
        if updated:
            update_ticks += 1
        else:
            watchdog_ticks += 1
        loop_sum_ms += loop_ms
        loop_max_ms = max(loop_max_ms, loop_ms)
        if loop_ms > period_s * 1000.0:
            overruns += 1

        now = time.perf_counter()
        if now - last_report_t >= 2.0:
            avg_ms = loop_sum_ms / max(count, 1)
            print(
                "[control_loop] "
                f"hz={count / (now - last_report_t):.1f} "
                f"avg_ms={avg_ms:.2f} max_ms={loop_max_ms:.2f} "
                f"overruns={overruns} updates={update_ticks} "
                f"watchdog={watchdog_ticks}"
            )
            last_report_t = now
            count = 0
            update_ticks = 0
            watchdog_ticks = 0
            loop_sum_ms = 0.0
            loop_max_ms = 0.0
            overruns = 0


# --- Main -----------------------------------------------------------------
def main() -> None:
    port, robot_id = select_follower_arm()
    print(f"[config] port={port} calibration={robot_id} max_cmd_dps={MAX_CMD_DEG_PER_SEC:g}")
    robot = SafeStartupSO101Follower(SO101FollowerConfig(port=port, id=robot_id, use_degrees=True))
    teleop = BrowserPhone()

    # Connect robot BEFORE teleop so startup owns the serial bus exclusively.
    # The helper retries only the fragile motor-discovery reads and guarantees
    # the serial fd closes if startup fails; it does not rerun the whole robot
    # connect/configure sequence as a catch-all.
    _connect_robot(robot)
    teleop.connect()

    motor_names = list(robot.bus.motors.keys())
    _derive_joint_limits(robot)
    print(f"[config] joint_limits_deg={ {m: (round(lo, 1), round(hi, 1)) for m, (lo, hi) in _JOINT_LIMITS_DEG.items()} }")

    _rehome(robot, motor_names)

    kin = RobotKinematics(urdf_path=URDF, target_frame_name=TARGET_FRAME, joint_names=motor_names)

    print(f"[config] cylindrical turret mapping (yaw->pan, gain={CYL_YAW_GAIN:g})")
    teleop_steps = [
        CylindricalPhoneToEE(kinematics=kin, motor_names=motor_names),
        ClutchAndWristControl(motor_names=motor_names),
        ClosedFormArmIK(kinematics=kin, motor_names=motor_names),
    ]

    teleop_action_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=teleop_steps,
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )
    identity_action = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](steps=[], to_transition=robot_action_observation_to_transition,
       to_output=transition_to_robot_action)

    pipeline = _TrajectoryLogger(
        teleop_action_processor,
        kin=kin,
        motor_names=motor_names,
        feedback_sink=teleop,
    )
    leader_mapper = RelativeLeaderMapper(motor_names)

    # SIGTERM (systemd, kill <pid>, etc.) by default kills the process with no
    # Python cleanup, so the recorder leaves an unfinalized webm and the arm
    # may not get torque-disabled. Translate it to KeyboardInterrupt so the
    # existing graceful-shutdown path below runs. SIGINT is already handled
    # by Python's default (raises KeyboardInterrupt). Reset prior handlers
    # on success so re-running in the same interpreter is safe.
    def _on_sigterm(_signum, _frame):
        raise KeyboardInterrupt("SIGTERM")
    signal.signal(signal.SIGTERM, _on_sigterm)
    try:
        _control_loop(
            teleop=teleop,
            robot=robot,
            fps=FPS,
            teleop_action_processor=pipeline,
            robot_action_processor=identity_action,
            leader_mapper=leader_mapper,
        )
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # Log + print the crash so the persistent teleop log keeps a forensic trail.
        import traceback
        print(f"\n[FATAL] control loop raised {exc.__class__.__name__}: {exc}")
        traceback.print_exc()
    finally:
        _save_trajectory_csv()
        s = _BUS_RETRY_STATS
        rate = 100 * s["retries"] / s["calls"] if s["calls"] else 0.0
        print(f"[bus_retry] calls={s['calls']} retries={s['retries']} "
              f"({rate:.2f}%) exhausted={s['exhausted']}")
        # Safety-critical: arm MUST go limp before we exit. If the bus is dead,
        # keep trying for ~30s -- a transient USB hiccup recovers in under a
        # second, and the cost of a 30s shutdown is dwarfed by the cost of the
        # arm holding torque indefinitely at a stretched pose with no operator.
        _force_disable_torque(robot, budget_s=30.0)
        try:
            teleop.disconnect()
        except Exception as e:
            print(f"[teardown] teleop.disconnect failed: {e}")
        try:
            robot.disconnect()
        except Exception as e:
            print(f"[teardown] robot.disconnect failed: {e}")


if __name__ == "__main__":
    main()
