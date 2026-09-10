#!/usr/bin/env python3
"""Regional WebTransport relay carrying control between controller and arm.

Devices terminate QUIC at their nearest edge. Same-edge peers pair locally;
controller traffic for a remote arm crosses a latest-only UDP path inside
WireGuard to the arm's relay.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import re
import socket
import struct
import time
import urllib.parse
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from .capabilities import verify as verify_capability
except ImportError:  # Direct deployment alongside capabilities.py.
    from capabilities import verify as verify_capability

try:
    import aioquic
    from aioquic.asyncio import QuicConnectionProtocol, serve
    from aioquic.h3.connection import H3_ALPN, H3Connection
    from aioquic.h3.events import (
        DatagramReceived,
        DataReceived,
        H3Event,
        HeadersReceived,
    )
    from aioquic.quic.configuration import QuicConfiguration
    from aioquic.quic.events import ProtocolNegotiated, QuicEvent
    from aioquic.tls import SessionTicket
except ImportError as e:  # pragma: no cover - exercised on hosts without aioquic.
    raise SystemExit(
        "aioquic is required for the WebTransport relay. "
        "Install python3-aioquic or pip install aioquic."
    ) from e

def _now_ms() -> float:
    return time.time() * 1000.0


def _json_dumps(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), allow_nan=False)


def _float_or_none(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if not (out == out and abs(out) != float("inf")):
        return None
    return out


TIMING_TRIM_FIELDS = (
    "ctrl_last_reconnect_reason",
    "wrtc_freezes",
    "wrtc_decode",
    "wrtc_jbuf",
    "wrtc_rtt",
    "wrtc_jitter",
    "wrtc_fps",
    "wrtc_via",
    "displayed_rtp_ts",
    "t_op_displayed_ms",
    "ctrl_ack_age_ms",
    "rtt",
    "_relay_arm_connected",
    "_relay_session",
)
EDGE_NAME_RE = re.compile(r"^[a-z0-9-]{1,32}$")


def _trim_for_datagram_limit(msg: dict[str, Any], max_bytes: int) -> bool:
    if len(_json_dumps(msg).encode("utf-8")) <= max_bytes:
        return True
    for field_name in TIMING_TRIM_FIELDS:
        msg.pop(field_name, None)
        if len(_json_dumps(msg).encode("utf-8")) <= max_bytes:
            return True
    return False


def _ensure_edge_timing(msg: dict[str, Any], relay_recv_ms: float) -> None:
    """Guarantee edge attribution fields exist for phone pose datagrams.

    Older regional forwarders stamped these fields before sending upstream.
    Full regional relays synthesize a zero-distance edge measurement so
    downstream logs can distinguish local relay ingress from missing telemetry.
    """
    edge_recv_ms = _float_or_none(msg.get("_edge_recv_ms"))
    edge_send_ms = _float_or_none(msg.get("_edge_send_ms"))
    if edge_recv_ms is None:
        edge_recv_ms = relay_recv_ms
        msg["_edge_recv_ms"] = round(edge_recv_ms, 3)
    if edge_send_ms is None:
        edge_send_ms = edge_recv_ms
        msg["_edge_send_ms"] = round(edge_send_ms, 3)
    if _float_or_none(msg.get("_edge_q_ms")) is None:
        msg["_edge_q_ms"] = round(max(0.0, edge_send_ms - edge_recv_ms), 3)


def _read_secret_arg(value: str) -> str:
    if not value:
        return ""
    if value.startswith("@"):
        try:
            return Path(value[1:]).read_text().strip()
        except OSError:
            return ""
    return value


@dataclass
class ProbeStats:
    accepted: int = 0
    closed: int = 0
    datagrams_rx: int = 0
    datagrams_tx: int = 0
    bytes_rx: int = 0
    bytes_tx: int = 0
    last_rx_ms: float | None = None
    sessions: dict[str, "WebTransportHandler"] = field(default_factory=dict)

    def snapshot(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "closed": self.closed,
            "active": len(self.sessions),
            "datagrams_rx": self.datagrams_rx,
            "datagrams_tx": self.datagrams_tx,
            "bytes_rx": self.bytes_rx,
            "bytes_tx": self.bytes_tx,
            "last_rx_age_ms": None if self.last_rx_ms is None else round(_now_ms() - self.last_rx_ms, 1),
        }


STATS = ProbeStats()
EVENT_LOG_PATH = ""
EDGE_NAME = "europe"
BACKBONE: "BackboneProtocol | None" = None
PEER_BACKBONES: dict[str, tuple[str, int]] = {}


class LatestSlot:
    def __init__(self) -> None:
        self._event = asyncio.Event()
        self._latest: dict | None = None
        self.put_count = 0
        self.drop_count = 0
        self.sent_count = 0
        self.send_drop_count = 0

    def put(self, msg: dict) -> None:
        if self._latest is not None:
            self.drop_count += 1
        self._latest = msg
        self.put_count += 1
        self._event.set()

    def clear(self) -> None:
        self._latest = None
        self._event.clear()

    async def get(self) -> dict:
        while True:
            await self._event.wait()
            msg = self._latest
            self._latest = None
            self._event.clear()
            if msg is not None:
                return msg

    def snapshot(self) -> dict[str, int]:
        return {
            "put": self.put_count,
            "drop": self.drop_count,
            "sent": self.sent_count,
            "send_drop": self.send_drop_count,
            "pending": int(self._latest is not None),
        }


@dataclass
class ControlPeer:
    role: str
    peer_id: str
    stream_id: int
    addr: tuple[str, int] | None
    send_datagram: Any
    connected_ms: float = field(default_factory=_now_ms)
    epoch: int = 0
    rx_count: int = 0
    tx_count: int = 0
    last_rx_ms: float | None = None
    last_tx_ms: float | None = None
    sender_task: asyncio.Task | None = None
    keepalive_task: asyncio.Task | None = None
    closed: bool = False

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.peer_id,
            "role": self.role,
            "transport": "webtransport",
            "addr": self.addr,
            "epoch": self.epoch,
            "connected_s": round((_now_ms() - self.connected_ms) / 1000.0, 1),
            "rx": self.rx_count,
            "tx": self.tx_count,
            "last_rx_age_ms": (
                None if self.last_rx_ms is None else round(_now_ms() - self.last_rx_ms, 1)
            ),
            "last_tx_age_ms": (
                None if self.last_tx_ms is None else round(_now_ms() - self.last_tx_ms, 1)
            ),
        }


@dataclass
class ControlSession:
    name: str
    expires_at_s: float
    phone: ControlPeer | None = None
    arm: ControlPeer | None = None
    phone_epoch: int = 0
    to_arm: LatestSlot = field(default_factory=LatestSlot)
    to_phone: LatestSlot = field(default_factory=LatestSlot)
    created_ms: float = field(default_factory=_now_ms)

    def summary(self) -> dict[str, Any]:
        return {
            "session": self.name,
            "age_s": round((_now_ms() - self.created_ms) / 1000.0, 1),
            "expires_at_s": self.expires_at_s,
            "phone_epoch": self.phone_epoch,
            "phone": None if self.phone is None else self.phone.summary(),
            "arm": None if self.arm is None else self.arm.summary(),
            "to_arm": self.to_arm.snapshot(),
            "to_phone": self.to_phone.snapshot(),
        }


class ControlRelay:
    def __init__(
        self,
        *,
        capability_secret: str,
        max_datagram_bytes: int = 1024,
    ) -> None:
        self.capability_secret = capability_secret
        self.max_datagram_bytes = max_datagram_bytes
        self.sessions: dict[str, ControlSession] = {}

    def authorize(
        self,
        role: str,
        token: str,
        session: str,
        *,
        home_edge: str | None = None,
    ) -> float | None:
        capability_role = f"phone@{home_edge}" if role == "phone" and home_edge else role
        if not verify_capability(
            self.capability_secret,
            token,
            role=capability_role,
            session=session,
        ):
            return None
        return float(token.split(".", 2)[1])

    def prune_expired(self, now_s: float | None = None) -> None:
        current = time.time() if now_s is None else now_s
        expired = [
            session
            for session in self.sessions.values()
            if session.expires_at_s <= current
        ]
        for session in expired:
            if session.phone is not None:
                self._close_peer(session.phone, "session expired")
            if session.arm is not None:
                self._close_peer(session.arm, "session expired")
            self.sessions.pop(session.name, None)
            record_event("session_expired", {"session": session.name})

    def get_session(self, name: str, expires_at_s: float) -> ControlSession:
        self.prune_expired()
        session = self.sessions.get(name)
        if session is None:
            session = ControlSession(name=name, expires_at_s=expires_at_s)
            self.sessions[name] = session
        else:
            # Different role capabilities can have different lifetimes. Keep
            # the latest observed expiry and never shorten a resumable session.
            session.expires_at_s = max(session.expires_at_s, expires_at_s)
        return session

    def stats(self) -> dict[str, Any]:
        self.prune_expired()
        return {
            "sessions": [session.summary() for session in self.sessions.values()],
            "time_ms": round(_now_ms(), 3),
            "transport": "webtransport",
        }

    def _slot_for_peer(self, session: ControlSession, peer: ControlPeer) -> LatestSlot:
        return session.to_arm if peer.role == "arm" else session.to_phone

    def _close_peer(self, peer: ControlPeer, reason: str) -> None:
        if peer.closed:
            return
        peer.closed = True
        if peer.sender_task is not None and not peer.sender_task.done():
            peer.sender_task.cancel()
        if peer.keepalive_task is not None and not peer.keepalive_task.done():
            peer.keepalive_task.cancel()
        record_event("peer_closed", {
            "role": peer.role,
            "peer_id": peer.peer_id,
            "epoch": peer.epoch,
            "rx": peer.rx_count,
            "tx": peer.tx_count,
            "reason": reason,
        })

    def attach_peer(self, session: ControlSession, peer: ControlPeer) -> None:
        old: ControlPeer | None
        if peer.role == "phone":
            session.phone_epoch += 1
            peer.epoch = session.phone_epoch
            old = session.phone
            session.phone = peer
            session.to_phone.clear()
        else:
            old = session.arm
            session.arm = peer
            session.to_arm.clear()
        if old is not None and old is not peer:
            self._close_peer(old, "replaced by newer peer")
        peer.sender_task = asyncio.create_task(
            self._sender_loop(session, peer, self._slot_for_peer(session, peer))
        )
        # Per-peer keepalive task: send a small heartbeat datagram every
        # KEEPALIVE_S so receivers can detect stuck QUIC sessions even when no
        # application traffic flows (e.g., browser hasn't sent any pose yet, or
        # the arm is mid-IK). Without this, an idle WebTransport datagram path
        # can quietly stop delivering events while both sides think they're
        # still connected.
        peer.keepalive_task = asyncio.create_task(self._keepalive_loop(session, peer))
        record_event("peer_connected", {
            "session": session.name,
            "role": peer.role,
            "transport": "webtransport",
            "peer_id": peer.peer_id,
            "epoch": peer.epoch,
            "addr": peer.addr,
        })
        self.send_json(peer, {
            "type": "relay_welcome",
            "role": peer.role,
            "session": session.name,
            "epoch": peer.epoch,
            "transport": "webtransport",
            "t_relay_ms": round(_now_ms(), 3),
        })

    KEEPALIVE_S = 3.0

    async def _keepalive_loop(self, session: ControlSession, peer: ControlPeer) -> None:
        try:
            while not peer.closed:
                await asyncio.sleep(self.KEEPALIVE_S)
                if peer.closed:
                    return
                active = (session.arm is peer) if peer.role == "arm" else (session.phone is peer)
                if not active:
                    return
                self.send_json(peer, {
                    "type": "relay_keepalive",
                    "t_relay_ms": round(_now_ms(), 3),
                })
        except asyncio.CancelledError:
            return

    def detach_peer(self, session: ControlSession, peer: ControlPeer, reason: str) -> None:
        if peer.role == "phone" and session.phone is peer:
            session.phone = None
            session.to_phone.clear()
        elif peer.role == "arm" and session.arm is peer:
            session.arm = None
            session.to_arm.clear()
        self._close_peer(peer, reason)

    def send_json(self, peer: ControlPeer, msg: dict) -> bool:
        if peer.closed:
            return False
        payload = _json_dumps(msg).encode("utf-8")
        if len(payload) > self.max_datagram_bytes:
            slot = None
            for session in self.sessions.values():
                if session.phone is peer or session.arm is peer:
                    slot = self._slot_for_peer(session, peer)
                    break
            if slot is not None:
                slot.send_drop_count += 1
            record_event("datagram_oversize_drop", {
                "role": peer.role,
                "peer_id": peer.peer_id,
                "bytes": len(payload),
                "limit": self.max_datagram_bytes,
                "seq": msg.get("seq"),
                "type": msg.get("type"),
            })
            return False
        peer.send_datagram(payload)
        peer.tx_count += 1
        peer.last_tx_ms = _now_ms()
        return True

    async def _sender_loop(
        self,
        session: ControlSession,
        peer: ControlPeer,
        slot: LatestSlot,
    ) -> None:
        try:
            while not peer.closed:
                msg = await slot.get()
                active = session.arm is peer if peer.role == "arm" else session.phone is peer
                if not active:
                    return
                send_start_ms = _now_ms()
                recv_ms = _float_or_none(msg.get("_relay_recv_ms"))
                if recv_ms is not None:
                    msg["_relay_queue_age_ms"] = round(max(0.0, send_start_ms - recv_ms), 3)
                msg["_relay_send_ms"] = round(send_start_ms, 3)
                _trim_for_datagram_limit(msg, self.max_datagram_bytes)
                if self.send_json(peer, msg):
                    slot.sent_count += 1
                else:
                    slot.send_drop_count += 1
        except asyncio.CancelledError:
            return

    def handle_datagram(self, session: ControlSession, peer: ControlPeer, data: bytes) -> None:
        if peer.closed:
            return
        active = session.phone is peer if peer.role == "phone" else session.arm is peer
        if not active:
            return
        try:
            msg = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if not isinstance(msg, dict):
            return
        if msg.get("type") in {"relay_ping", "ping"}:
            self.send_json(peer, {
                "type": "relay_pong",
                "t_relay_ms": round(_now_ms(), 3),
                "echo": msg.get("t"),
            })
            return
        peer.rx_count += 1
        peer.last_rx_ms = _now_ms()
        if peer.role == "phone":
            self._from_phone(session, peer, msg)
        else:
            self._from_arm(session, peer, msg)

    def _from_phone(self, session: ControlSession, peer: ControlPeer, msg: dict) -> None:
        if session.phone is not peer or session.arm is None:
            return
        relay_recv_ms = _now_ms()
        msg["_relay_session"] = session.name
        msg["_relay_epoch"] = peer.epoch
        msg["_relay_recv_ms"] = round(relay_recv_ms, 3)
        _ensure_edge_timing(msg, relay_recv_ms)
        msg["_relay_arm_connected"] = 1
        _trim_for_datagram_limit(msg, self.max_datagram_bytes)
        session.to_arm.put(msg)

    def _from_arm(self, session: ControlSession, peer: ControlPeer, msg: dict) -> None:
        if session.arm is not peer or session.phone is None:
            return
        msg["_relay_session"] = session.name
        msg["_relay_epoch"] = session.phone.epoch
        msg["_relay_recv_ms"] = round(_now_ms(), 3)
        session.to_phone.put(msg)


CONTROL_RELAY: ControlRelay | None = None


def record_event(event: str, details: dict[str, Any]) -> None:
    rec = {
        "t_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "t_ms": round(_now_ms(), 3),
        "event": event,
        "details": details,
    }
    logging.info("%s %s", event, details)
    if not EVENT_LOG_PATH:
        return
    try:
        path = Path(EVENT_LOG_PATH)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", buffering=1) as f:
            f.write(_json_dumps(rec) + "\n")
    except Exception as e:  # noqa: BLE001
        logging.warning("event log write failed: %s", e)


BACKBONE_MAGIC = b"PAU1"
BACKBONE_CONTROL = 1
BACKBONE_FEEDBACK = 2
BACKBONE_CLOSE = 3
BACKBONE_HEADER = struct.Struct("!4sBQQQBBH")
BACKBONE_MAX_PACKET_BYTES = 1400
BACKBONE_MAX_FUTURE_MS = 100
BACKBONE_SOCKET_BUFFER_BYTES = 16 * 1024


@dataclass
class BackbonePacket:
    kind: int
    epoch: int
    sequence: int
    sent_ms: int
    source_edge: str
    session: str
    token: str
    payload: bytes

    def encode(self) -> bytes:
        try:
            edge = self.source_edge.encode("ascii")
            session = self.session.encode("ascii")
            token = self.token.encode("ascii")
        except UnicodeEncodeError as exc:
            raise ValueError("backbone routing fields must be ASCII") from exc
        if len(edge) > 255 or len(session) > 255 or len(token) > 65535:
            raise ValueError("backbone routing field is too long")
        return b"".join((
            BACKBONE_HEADER.pack(
                BACKBONE_MAGIC,
                self.kind,
                self.epoch,
                self.sequence,
                self.sent_ms,
                len(edge),
                len(session),
                len(token),
            ),
            edge,
            session,
            token,
            self.payload,
        ))

    @classmethod
    def decode(cls, data: bytes) -> "BackbonePacket | None":
        if len(data) < BACKBONE_HEADER.size:
            return None
        magic, kind, epoch, sequence, sent_ms, edge_len, session_len, token_len = (
            BACKBONE_HEADER.unpack_from(data)
        )
        body_at = BACKBONE_HEADER.size
        fields_end = body_at + edge_len + session_len + token_len
        if magic != BACKBONE_MAGIC or fields_end > len(data):
            return None
        try:
            source_edge = data[body_at:body_at + edge_len].decode("ascii")
            session_at = body_at + edge_len
            session = data[session_at:session_at + session_len].decode("ascii")
            token_at = session_at + session_len
            token = data[token_at:token_at + token_len].decode("ascii")
        except UnicodeDecodeError:
            return None
        return cls(
            kind=kind,
            epoch=epoch,
            sequence=sequence,
            sent_ms=sent_ms,
            source_edge=source_edge,
            session=session,
            token=token,
            payload=data[fields_end:],
        )


@dataclass
class RemotePhone:
    peer: ControlPeer
    epoch: int
    token: str
    expires_at_s: float
    last_sequence: int = -1
    feedback_sequence: int = 0
    last_seen_ms: float = field(default_factory=_now_ms)


class BackboneProtocol(asyncio.DatagramProtocol):
    """Latest-only UDP session router intended to run inside WireGuard."""

    REMOTE_PEER_TIMEOUT_MS = 15_000

    def __init__(self, *, max_age_ms: float) -> None:
        self.max_age_ms = max_age_ms
        self.transport: asyncio.DatagramTransport | None = None
        self.ingress_handlers: dict[tuple[str, str], "WebTransportHandler"] = {}
        self.remote_phones: dict[tuple[str, str], RemotePhone] = {}
        self.closed_epochs: dict[tuple[str, str], tuple[int, float]] = {}
        self.pending: dict[
            tuple[int, str, str], tuple[BackbonePacket, tuple[str, int]]
        ] = {}
        self._flush_scheduled = False
        self.drop_counts: dict[str, int] = {}

    def connection_made(self, transport) -> None:
        self.transport = transport
        sock = transport.get_extra_info("socket")
        if sock is not None:
            # A small kernel queue is intentional: under overload, discard
            # commands instead of accumulating latency behind obsolete state.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, BACKBONE_SOCKET_BUFFER_BYTES)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BACKBONE_SOCKET_BUFFER_BYTES)

    def _drop(self, reason: str) -> None:
        self.drop_counts[reason] = self.drop_counts.get(reason, 0) + 1

    def _trusted_source(self, packet: BackbonePacket, addr: tuple[str, int]) -> bool:
        expected = PEER_BACKBONES.get(packet.source_edge)
        return expected is not None and addr[0] == expected[0] and addr[1] == expected[1]

    def register_ingress(
        self, home_edge: str, session: str, handler: "WebTransportHandler"
    ) -> None:
        self.ingress_handlers[(home_edge, session)] = handler

    def unregister_ingress(
        self, home_edge: str, session: str, handler: "WebTransportHandler"
    ) -> None:
        key = (home_edge, session)
        if self.ingress_handlers.get(key) is handler:
            self.ingress_handlers.pop(key, None)

    def send_control(self, home_edge: str, packet: BackbonePacket) -> None:
        self._queue_latest(home_edge, packet)

    def _queue_latest(self, destination_edge: str, packet: BackbonePacket) -> None:
        destination = PEER_BACKBONES.get(destination_edge)
        if destination is None:
            self._drop("unknown_destination")
            return
        self.pending[(packet.kind, destination_edge, packet.session)] = (
            packet,
            destination,
        )
        if not self._flush_scheduled:
            self._flush_scheduled = True
            asyncio.get_running_loop().call_soon(self._flush_latest)

    def _flush_latest(self) -> None:
        self._flush_scheduled = False
        pending, self.pending = self.pending, {}
        if self.transport is None:
            self._drop("transport_unavailable")
            return
        for packet, destination in pending.values():
            try:
                encoded = packet.encode()
            except ValueError:
                self._drop("encode")
                continue
            if len(encoded) > BACKBONE_MAX_PACKET_BYTES:
                self._drop("oversize")
                continue
            self.transport.sendto(encoded, destination)

    def send_close(self, home_edge: str, packet: BackbonePacket) -> None:
        pending_key = (BACKBONE_CONTROL, home_edge, packet.session)
        pending = self.pending.get(pending_key)
        if pending is not None and pending[0].epoch == packet.epoch:
            self.pending.pop(pending_key, None)
        destination = PEER_BACKBONES.get(home_edge)
        if self.transport is not None and destination is not None:
            encoded = packet.encode()
            if len(encoded) <= BACKBONE_MAX_PACKET_BYTES:
                self.transport.sendto(encoded, destination)

    def datagram_received(self, data: bytes, addr) -> None:
        if len(data) > BACKBONE_MAX_PACKET_BYTES:
            self._drop("oversize")
            return
        packet = BackbonePacket.decode(data)
        if packet is None or not self._trusted_source(packet, addr):
            self._drop("untrusted_or_malformed")
            return
        age_ms = _now_ms() - packet.sent_ms
        if age_ms > self.max_age_ms:
            self._drop("expired")
            return
        if age_ms < -BACKBONE_MAX_FUTURE_MS:
            self._drop("clock_skew")
            return
        if packet.kind == BACKBONE_CONTROL:
            self._receive_control(packet)
        elif packet.kind == BACKBONE_FEEDBACK:
            self._receive_feedback(packet)
        elif packet.kind == BACKBONE_CLOSE:
            self._receive_close(packet)
        else:
            self._drop("unknown_kind")

    def _receive_control(self, packet: BackbonePacket) -> None:
        if CONTROL_RELAY is None or packet.session == "":
            return
        key = (packet.source_edge, packet.session)
        closed = self.closed_epochs.get(key)
        if closed is not None and packet.epoch <= closed[0]:
            self._drop("closed_epoch")
            return
        remote = self.remote_phones.get(key)
        if remote is not None and packet.epoch < remote.epoch:
            self._drop("old_epoch")
            return
        if remote is None or packet.epoch > remote.epoch:
            expires_at_s = CONTROL_RELAY.authorize(
                "phone",
                packet.token,
                packet.session,
                home_edge=EDGE_NAME,
            )
            if expires_at_s is None:
                self._drop("unauthorized")
                return
            if remote is not None:
                session = CONTROL_RELAY.sessions.get(packet.session)
                if session is not None:
                    CONTROL_RELAY.detach_peer(session, remote.peer, "new backbone epoch")
            peer = ControlPeer(
                role="phone",
                peer_id=f"backbone:{packet.source_edge}:{packet.epoch}",
                stream_id=-1,
                addr=PEER_BACKBONES.get(packet.source_edge),
                send_datagram=lambda payload, k=key: self._send_feedback(k, payload),
            )
            remote = RemotePhone(peer, packet.epoch, packet.token, expires_at_s)
            self.remote_phones[key] = remote
            session = CONTROL_RELAY.get_session(packet.session, expires_at_s)
            # Register before attach_peer sends its welcome through the
            # virtual peer's feedback callback.
            CONTROL_RELAY.attach_peer(session, peer)
        if packet.sequence <= remote.last_sequence:
            self._drop("old_sequence")
            return
        remote.last_sequence = packet.sequence
        remote.last_seen_ms = _now_ms()
        session = CONTROL_RELAY.sessions.get(packet.session)
        if session is not None:
            CONTROL_RELAY.handle_datagram(session, remote.peer, packet.payload)

    def _send_feedback(self, key: tuple[str, str], payload: bytes) -> None:
        remote = self.remote_phones.get(key)
        if remote is None or key[0] not in PEER_BACKBONES:
            return
        remote.feedback_sequence += 1
        packet = BackbonePacket(
            BACKBONE_FEEDBACK,
            remote.epoch,
            remote.feedback_sequence,
            int(_now_ms()),
            EDGE_NAME,
            key[1],
            "",
            payload,
        )
        self._queue_latest(key[0], packet)

    def _receive_feedback(self, packet: BackbonePacket) -> None:
        handler = self.ingress_handlers.get((packet.source_edge, packet.session))
        if handler is None or handler.backbone_epoch != packet.epoch:
            self._drop("unknown_ingress")
            return
        if packet.sequence <= handler.backbone_feedback_sequence:
            self._drop("old_feedback_sequence")
            return
        handler.backbone_feedback_sequence = packet.sequence
        handler.send_datagram(packet.payload)

    def _receive_close(self, packet: BackbonePacket) -> None:
        key = (packet.source_edge, packet.session)
        if not CONTROL_RELAY or not CONTROL_RELAY.authorize(
            "phone", packet.token, packet.session, home_edge=EDGE_NAME
        ):
            return
        prior = self.closed_epochs.get(key)
        if prior is None or packet.epoch > prior[0]:
            self.closed_epochs[key] = (packet.epoch, _now_ms())
        remote = self.remote_phones.get(key)
        if remote is not None and remote.epoch == packet.epoch:
            self._detach_remote(key, remote, "backbone ingress closed")

    def _detach_remote(self, key: tuple[str, str], remote: RemotePhone, reason: str) -> None:
        session = CONTROL_RELAY.sessions.get(key[1]) if CONTROL_RELAY is not None else None
        if session is not None:
            CONTROL_RELAY.detach_peer(session, remote.peer, reason)
        self.remote_phones.pop(key, None)

    def prune_remote_peers(self, now_ms: float | None = None) -> None:
        current = _now_ms() if now_ms is None else now_ms
        for key, remote in list(self.remote_phones.items()):
            if (
                current - remote.last_seen_ms > self.REMOTE_PEER_TIMEOUT_MS
                or remote.expires_at_s * 1000 <= current
            ):
                self._detach_remote(key, remote, "backbone peer timeout")
        self.closed_epochs = {
            key: value
            for key, value in self.closed_epochs.items()
            if current - value[1] <= self.REMOTE_PEER_TIMEOUT_MS
        }


class WebTransportHandler:
    def __init__(
        self,
        *,
        connection: H3Connection,
        stream_id: int,
        path: str,
        query: dict[str, list[str]],
        client: tuple[str, int] | None,
        transmit,
    ) -> None:
        self.connection = connection
        self.stream_id = stream_id
        self.path = path
        self.query = query
        self.client = client
        self.transmit = transmit
        self.max_datagram_bytes = (
            CONTROL_RELAY.max_datagram_bytes if CONTROL_RELAY is not None else 1024
        )
        self.accepted = False
        self.closed = False
        self.session: ControlSession | None = None
        self.peer: ControlPeer | None = None
        self.backbone_home: str | None = None
        self.backbone_token = ""
        self.backbone_epoch = time.time_ns()
        self.backbone_sequence = 0
        self.backbone_feedback_sequence = -1
        self.backbone_keepalive_task: asyncio.Task | None = None
        self.stats_key = f"{id(connection)}:{stream_id}"
        self.rx = 0
        self.tx = 0

    def accept(
        self,
        *,
        expires_at_s: float,
        backbone_home: str | None = None,
        backbone_token: str = "",
    ) -> None:
        self.accepted = True
        STATS.accepted += 1
        STATS.sessions[self.stats_key] = self
        self.connection.send_headers(
            stream_id=self.stream_id,
            headers=[
                (b":status", b"200"),
                (b"server", f"phone-arm-wt-relay/aioquic-{aioquic.__version__}".encode()),
                (b"sec-webtransport-http3-draft", b"draft02"),
            ],
        )
        self.transmit()
        record_event("wt_accept", {"stream_id": self.stream_id, "path": self.path, "client": self.client})
        if backbone_home is None:
            self._activate_control(expires_at_s)
        else:
            self.backbone_home = backbone_home
            self.backbone_token = backbone_token
            assert BACKBONE is not None
            BACKBONE.register_ingress(backbone_home, self._session_name(), self)
            self.backbone_keepalive_task = asyncio.create_task(
                self._backbone_keepalive_loop()
            )

    def reject(self, status: int = 404) -> None:
        self.closed = True
        self.connection.send_headers(
            stream_id=self.stream_id,
            headers=[(b":status", str(status).encode())],
            end_stream=True,
        )
        self.transmit()

    def close(self, reason: str) -> None:
        if self.closed:
            return
        self.closed = True
        if self.backbone_keepalive_task is not None:
            self.backbone_keepalive_task.cancel()
        STATS.closed += 1
        STATS.sessions.pop(self.stats_key, None)
        if self.backbone_home is not None and BACKBONE is not None:
            session_name = self._session_name()
            BACKBONE.unregister_ingress(self.backbone_home, session_name, self)
            BACKBONE.send_close(
                self.backbone_home,
                BackbonePacket(
                    BACKBONE_CLOSE,
                    self.backbone_epoch,
                    self.backbone_sequence + 1,
                    int(_now_ms()),
                    EDGE_NAME,
                    session_name,
                    self.backbone_token,
                    b"",
                ),
            )
        elif CONTROL_RELAY is not None and self.session is not None and self.peer is not None:
            CONTROL_RELAY.detach_peer(self.session, self.peer, reason)
        record_event("wt_close", {
            "stream_id": self.stream_id,
            "reason": reason,
            "rx": self.rx,
            "tx": self.tx,
        })

    def _activate_control(self, expires_at_s: float) -> None:
        if CONTROL_RELAY is None:
            self.reject(503)
            return
        role = self.path.rsplit("/", 1)[-1]
        session_name = (self.query.get("session") or ["default"])[0] or "default"
        peer = ControlPeer(
            role=role,
            peer_id=str(uuid.uuid4()),
            stream_id=self.stream_id,
            addr=self.client,
            send_datagram=self._send_datagram,
        )
        self.session = CONTROL_RELAY.get_session(session_name, expires_at_s)
        self.peer = peer
        CONTROL_RELAY.attach_peer(self.session, peer)

    def _session_name(self) -> str:
        return (self.query.get("session") or ["default"])[0] or "default"

    def _send_datagram(self, data: bytes) -> None:
        self.connection.send_datagram(stream_id=self.stream_id, data=data)
        self.tx += 1
        STATS.datagrams_tx += 1
        STATS.bytes_tx += len(data)
        self.transmit()

    def send_datagram(self, data: bytes) -> None:
        if not self.closed:
            self._send_datagram(data)

    def send_json_datagram(self, message: dict[str, Any]) -> None:
        self.send_datagram(_json_dumps(message).encode("utf-8"))

    async def _backbone_keepalive_loop(self) -> None:
        try:
            while not self.closed:
                await asyncio.sleep(ControlRelay.KEEPALIVE_S)
                if self.closed:
                    return
                # This proves only the short controller-to-ingress connection.
                # End-to-end arm health is tracked separately by robot ACKs.
                self.send_json_datagram({
                    "type": "relay_keepalive",
                    "edge": EDGE_NAME,
                    "t_relay_ms": round(_now_ms(), 3),
                })
        except asyncio.CancelledError:
            return

    def handle_datagram(self, data: bytes) -> None:
        self.rx += 1
        STATS.datagrams_rx += 1
        STATS.bytes_rx += len(data)
        STATS.last_rx_ms = _now_ms()
        if self.backbone_home is not None and BACKBONE is not None:
            received_ms = _now_ms()
            try:
                message = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                message = None
            if isinstance(message, dict) and message.get("t") is not None:
                self.send_json_datagram({
                    "type": "edge_ack",
                    "ack_t": message.get("t"),
                    "ack_seq": message.get("seq"),
                    "t_edge_recv_ms": round(received_ms, 3),
                    "t_edge_send_ms": round(_now_ms(), 3),
                    "edge": EDGE_NAME,
                })
                stamped = dict(message)
                stamped["_edge_recv_ms"] = round(received_ms, 3)
                stamped["_edge_send_ms"] = round(_now_ms(), 3)
                stamped["_edge_q_ms"] = max(
                    0.0, stamped["_edge_send_ms"] - stamped["_edge_recv_ms"]
                )
                try:
                    candidate = _json_dumps(stamped).encode("utf-8")
                except (TypeError, ValueError):
                    candidate = b""
                if candidate and len(candidate) <= self.max_datagram_bytes:
                    data = candidate
            self.backbone_sequence += 1
            BACKBONE.send_control(
                self.backbone_home,
                BackbonePacket(
                    BACKBONE_CONTROL,
                    self.backbone_epoch,
                    self.backbone_sequence,
                    int(received_ms),
                    EDGE_NAME,
                    self._session_name(),
                    self.backbone_token,
                    data,
                ),
            )
        elif CONTROL_RELAY is not None and self.session is not None and self.peer is not None:
            CONTROL_RELAY.handle_datagram(self.session, self.peer, data)


class HttpServerProtocol(QuicConnectionProtocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._http: H3Connection | None = None
        self._handlers: dict[int, WebTransportHandler] = {}

    def _client(self) -> tuple[str, int] | None:
        try:
            return self._quic._network_paths[0].addr
        except Exception:  # noqa: BLE001
            return None

    def _send_text_response(self, stream_id: int, status: int, body: str) -> None:
        assert self._http is not None
        data = body.encode("utf-8")
        self._http.send_headers(
            stream_id=stream_id,
            headers=[
                (b":status", str(status).encode()),
                (b"content-type", b"text/plain; charset=utf-8"),
                (b"content-length", str(len(data)).encode()),
            ],
        )
        self._http.send_data(stream_id=stream_id, data=data, end_stream=True)
        self.transmit()

    def http_event_received(self, event: H3Event) -> None:
        assert self._http is not None
        if isinstance(event, HeadersReceived):
            headers = dict(event.headers)
            method = headers.get(b":method", b"").decode()
            protocol = headers.get(b":protocol", b"").decode()
            raw_path = headers.get(b":path", b"/")
            parsed = urllib.parse.urlsplit(raw_path.decode(errors="replace"))
            path = parsed.path
            query = urllib.parse.parse_qs(parsed.query)
            if method == "CONNECT" and protocol == "webtransport":
                handler = WebTransportHandler(
                    connection=self._http,
                    stream_id=event.stream_id,
                    path=path,
                    query=query,
                    client=self._client(),
                    transmit=self.transmit,
                )
                self._handlers[event.stream_id] = handler
                if path in {"/wt/phone", "/wt/arm"}:
                    if CONTROL_RELAY is None:
                        handler.reject(503)
                        return
                    role = path.rsplit("/", 1)[-1]
                    token = (query.get("token") or [""])[0]
                    session = (query.get("session") or ["default"])[0]
                    home_edge = (query.get("home") or [EDGE_NAME])[0] or EDGE_NAME
                    expires_at_s = CONTROL_RELAY.authorize(
                        role,
                        token,
                        session,
                        home_edge=home_edge if role == "phone" else None,
                    )
                    if expires_at_s is None:
                        record_event("wt_unauthorized", {
                            "role": role,
                            "session": (query.get("session") or ["default"])[0],
                            "client": self._client(),
                        })
                        handler.reject(401)
                        return
                    if home_edge == EDGE_NAME:
                        handler.accept(expires_at_s=expires_at_s)
                        return
                    # Arms connect to their own selected edge. Only controller
                    # traffic crosses the backbone, toward the arm's edge.
                    has_backbone = (
                        role == "phone"
                        and BACKBONE is not None
                        and home_edge in PEER_BACKBONES
                    )
                    if not has_backbone:
                        record_event("edge_route_rejected", {
                            "edge": EDGE_NAME,
                            "home": home_edge,
                            "role": role,
                            "session": session,
                        })
                        handler.reject(503)
                        return
                    handler.accept(
                        expires_at_s=expires_at_s,
                        backbone_home=home_edge,
                        backbone_token=token,
                    )
                else:
                    handler.reject(404)
                return
            if method == "GET" and path == "/healthz":
                self._send_text_response(event.stream_id, 200, "ok\n")
                return
            if method == "GET" and path == "/stats":
                out = STATS.snapshot()
                if CONTROL_RELAY is not None:
                    out["control"] = CONTROL_RELAY.stats()
                self._send_text_response(event.stream_id, 200, _json_dumps(out) + "\n")
                return
            self._send_text_response(
                event.stream_id,
                200,
                (
                    f"phone-arm WebTransport control relay ({EDGE_NAME})\n"
                    "control: /wt/phone?session=default&token=... and /wt/arm?session=default&token=...\n"
                ),
            )
        elif isinstance(event, DatagramReceived):
            handler = self._handlers.get(event.stream_id)
            if handler is not None and handler.accepted:
                handler.handle_datagram(event.data)
        elif isinstance(event, DataReceived) and event.stream_ended:
            handler = self._handlers.get(event.stream_id)
            if handler is not None:
                handler.close("stream ended")

    def quic_event_received(self, event: QuicEvent) -> None:
        if isinstance(event, ProtocolNegotiated):
            if event.alpn_protocol in H3_ALPN:
                self._http = H3Connection(self._quic, enable_webtransport=True)
                record_event("protocol", {"alpn": event.alpn_protocol, "client": self._client()})
            else:
                record_event("protocol_rejected", {"alpn": event.alpn_protocol})
        if self._http is not None:
            for http_event in self._http.handle_event(event):
                self.http_event_received(http_event)

    def connection_lost(self, exc: Exception | None) -> None:
        for handler in list(self._handlers.values()):
            handler.close("connection lost")
        super().connection_lost(exc)


class SessionTicketStore:
    def __init__(self) -> None:
        self.tickets: dict[bytes, SessionTicket] = {}

    def add(self, ticket: SessionTicket) -> None:
        self.tickets[ticket.ticket] = ticket

    def pop(self, label: bytes) -> SessionTicket | None:
        return self.tickets.pop(label, None)


async def _stats_loop(interval_s: float) -> None:
    while True:
        await asyncio.sleep(interval_s)
        details = STATS.snapshot()
        if CONTROL_RELAY is not None:
            CONTROL_RELAY.prune_expired()
            details["control"] = CONTROL_RELAY.stats()
        if BACKBONE is not None:
            details["backbone"] = {
                "transport": "udp",
                "pending": len(BACKBONE.pending),
                "remote_phones": len(BACKBONE.remote_phones),
                "drops": dict(BACKBONE.drop_counts),
            }
        record_event("stats", details)


async def _expiry_loop() -> None:
    while True:
        await asyncio.sleep(60)
        if CONTROL_RELAY is not None:
            CONTROL_RELAY.prune_expired()
        if BACKBONE is not None:
            BACKBONE.prune_remote_peers()


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=4433)
    parser.add_argument("--certificate", required=True)
    parser.add_argument("--private-key", required=True)
    parser.add_argument("--event-log", default="")
    parser.add_argument("--stats-interval-s", type=float, default=5.0)
    parser.add_argument(
        "--edge-name",
        default=os.environ.get("PHONE_ARM_EDGE_NAME", "europe"),
        help="short name for this regional relay",
    )
    parser.add_argument("--backbone-host", default="127.0.0.1")
    parser.add_argument("--backbone-port", type=int, default=7443)
    parser.add_argument("--backbone-max-age-ms", type=float, default=150.0)
    parser.add_argument(
        "--peer-backbone",
        action="append",
        default=[],
        metavar="NAME=HOST:PORT",
        help="WireGuard peer UDP address; repeat for each reachable region",
    )
    parser.add_argument(
        "--capability-secret",
        default=os.environ.get("PHONE_ARM_CAPABILITY_SECRET", ""),
        required=not bool(os.environ.get("PHONE_ARM_CAPABILITY_SECRET", "")),
    )
    parser.add_argument("--max-datagram-bytes", type=int, default=int(os.environ.get("PHONE_ARM_WT_MAX_DATAGRAM_BYTES", "1024")))
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    global BACKBONE, CONTROL_RELAY, EDGE_NAME, EVENT_LOG_PATH, PEER_BACKBONES
    EVENT_LOG_PATH = args.event_log
    EDGE_NAME = args.edge_name.strip().lower()
    if not EDGE_NAME_RE.fullmatch(EDGE_NAME):
        parser.error("--edge-name must contain only lowercase letters, digits, and hyphens")
    peers: dict[str, tuple[str, int]] = {}
    for value in args.peer_backbone:
        try:
            name, address = value.split("=", 1)
            host, raw_port = address.rsplit(":", 1)
            port = int(raw_port)
        except ValueError:
            parser.error("--peer-backbone must use NAME=HOST:PORT")
        name = name.strip().lower()
        if (
            not EDGE_NAME_RE.fullmatch(name)
            or name == EDGE_NAME
            or not host
            or not 1 <= port <= 65535
        ):
            parser.error(f"invalid --peer-backbone value: {value!r}")
        try:
            peer_ip = socket.gethostbyname(host.strip())
        except OSError as exc:
            parser.error(f"cannot resolve --peer-backbone host {host!r}: {exc}")
        peers[name] = (peer_ip, port)
    PEER_BACKBONES = peers
    capability_secret = _read_secret_arg(args.capability_secret)
    if not capability_secret:
        parser.error("--capability-secret must not be empty")
    CONTROL_RELAY = ControlRelay(
        capability_secret=capability_secret,
        max_datagram_bytes=max(1, args.max_datagram_bytes),
    )
    backbone_transport = None
    if PEER_BACKBONES:
        loop = asyncio.get_running_loop()
        backbone_transport, protocol = await loop.create_datagram_endpoint(
            lambda: BackboneProtocol(max_age_ms=max(1.0, args.backbone_max_age_ms)),
            local_addr=(args.backbone_host, args.backbone_port),
        )
        BACKBONE = protocol

    logging.basicConfig(
        format="%(asctime)s %(levelname)s %(message)s",
        level=logging.DEBUG if args.verbose else logging.INFO,
    )
    configuration = QuicConfiguration(
        alpn_protocols=H3_ALPN,
        is_client=False,
        max_datagram_frame_size=65536,
    )
    configuration.load_cert_chain(args.certificate, args.private_key)
    store = SessionTicketStore()
    record_event("started", {
        "host": args.host,
        "port": args.port,
        "certificate": args.certificate,
        "aioquic": aioquic.__version__,
        "control_auth": {
            "phone": True,
            "arm": True,
        },
        "edge": EDGE_NAME,
        "backbone": {
            "host": args.backbone_host,
            "port": args.backbone_port,
            "max_age_ms": args.backbone_max_age_ms,
            "peers": sorted(PEER_BACKBONES),
            "transport": "wireguard-udp",
        },
        "max_datagram_bytes": CONTROL_RELAY.max_datagram_bytes,
    })
    if args.stats_interval_s > 0:
        asyncio.create_task(_stats_loop(args.stats_interval_s))
    asyncio.create_task(_expiry_loop())
    try:
        await serve(
            args.host,
            args.port,
            configuration=configuration,
            create_protocol=HttpServerProtocol,
            session_ticket_fetcher=store.pop,
            session_ticket_handler=store.add,
        )
        await asyncio.Future()
    finally:
        if backbone_transport is not None:
            backbone_transport.close()


if __name__ == "__main__":
    asyncio.run(main())
