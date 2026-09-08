#!/usr/bin/env python3
"""WebTransport relay carrying control datagrams between controller and arm."""
from __future__ import annotations

import argparse
import asyncio
import hmac
import json
import logging
import os
import time
import urllib.parse
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import aioquic
    from aioquic.asyncio import QuicConnectionProtocol, serve
    from aioquic.h3.connection import H3_ALPN, H3Connection
    from aioquic.h3.events import (
        DatagramReceived,
        DataReceived,
        H3Event,
        HeadersReceived,
        WebTransportStreamDataReceived,
    )
    from aioquic.quic.configuration import QuicConfiguration
    from aioquic.quic.events import ProtocolNegotiated, QuicEvent
    from aioquic.tls import SessionTicket
except ImportError as e:  # pragma: no cover - exercised on hosts without aioquic.
    raise SystemExit(
        "aioquic is required for the WebTransport probe. "
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

    A Singapore forwarder stamps these fields before sending upstream. If the
    phone connects directly to this relay, synthesize a zero-distance edge at
    the relay so downstream logs can distinguish direct mode from missing
    telemetry.
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
        phone_secret: str = "",
        arm_secret: str = "",
        max_datagram_bytes: int = 1024,
    ) -> None:
        self.role_secrets = {"phone": phone_secret, "arm": arm_secret}
        self.max_datagram_bytes = max_datagram_bytes
        self.sessions: dict[str, ControlSession] = {}

    def authorized(self, role: str, token: str) -> bool:
        secret = self.role_secrets.get(role, "")
        if not secret:
            return True
        return hmac.compare_digest(token or "", secret)

    def get_session(self, name: str) -> ControlSession:
        session = self.sessions.get(name)
        if session is None:
            session = ControlSession(name=name)
            self.sessions[name] = session
        return session

    def stats(self) -> dict[str, Any]:
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
        self.accepted = False
        self.closed = False
        self.mode = "echo"
        self.session: ControlSession | None = None
        self.peer: ControlPeer | None = None
        self.stats_key = f"{id(connection)}:{stream_id}"
        self.rx = 0
        self.tx = 0

    def accept(self, *, mode: str = "echo") -> None:
        self.mode = mode
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
        if mode == "control":
            self._activate_control()

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
        STATS.closed += 1
        STATS.sessions.pop(self.stats_key, None)
        if self.mode == "control" and CONTROL_RELAY is not None and self.session is not None and self.peer is not None:
            CONTROL_RELAY.detach_peer(self.session, self.peer, reason)
        record_event("wt_close", {
            "stream_id": self.stream_id,
            "reason": reason,
            "rx": self.rx,
            "tx": self.tx,
        })

    def _activate_control(self) -> None:
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
        self.session = CONTROL_RELAY.get_session(session_name)
        self.peer = peer
        CONTROL_RELAY.attach_peer(self.session, peer)

    def _send_datagram(self, data: bytes) -> None:
        self.connection.send_datagram(stream_id=self.stream_id, data=data)
        self.tx += 1
        STATS.datagrams_tx += 1
        STATS.bytes_tx += len(data)
        self.transmit()

    def handle_datagram(self, data: bytes) -> None:
        self.rx += 1
        STATS.datagrams_rx += 1
        STATS.bytes_rx += len(data)
        STATS.last_rx_ms = _now_ms()
        if self.mode == "control":
            if CONTROL_RELAY is not None and self.session is not None and self.peer is not None:
                CONTROL_RELAY.handle_datagram(self.session, self.peer, data)
            return
        self._send_datagram(data)

    def handle_stream_data(self, event: WebTransportStreamDataReceived) -> None:
        # Echo reliable stream data too, but pose-control experiments should use
        # datagrams. This just helps detect browser API shape changes.
        self.connection._quic.send_stream_data(stream_id=event.stream_id, data=event.data)
        self.transmit()


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
                if path == "/wt":
                    handler.accept()
                elif path in {"/wt/phone", "/wt/arm"}:
                    if CONTROL_RELAY is None:
                        handler.reject(503)
                        return
                    role = path.rsplit("/", 1)[-1]
                    token = (query.get("token") or [""])[0]
                    if not CONTROL_RELAY.authorized(role, token):
                        record_event("wt_unauthorized", {
                            "role": role,
                            "session": (query.get("session") or ["default"])[0],
                            "client": self._client(),
                        })
                        handler.reject(401)
                        return
                    handler.accept(mode="control")
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
                    "phone-arm WebTransport probe/control relay\n"
                    "echo: /wt\n"
                    "control: /wt/phone?session=default&token=... and /wt/arm?session=default&token=...\n"
                ),
            )
        elif isinstance(event, DatagramReceived):
            handler = self._handlers.get(event.stream_id)
            if handler is not None and handler.accepted:
                handler.handle_datagram(event.data)
        elif isinstance(event, WebTransportStreamDataReceived):
            handler = self._handlers.get(event.session_id)
            if handler is not None and handler.accepted:
                handler.handle_stream_data(event)
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
        record_event("stats", STATS.snapshot())


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=4433)
    parser.add_argument("--certificate", required=True)
    parser.add_argument("--private-key", required=True)
    parser.add_argument("--event-log", default="")
    parser.add_argument("--stats-interval-s", type=float, default=5.0)
    parser.add_argument("--phone-secret", default=os.environ.get("PHONE_ARM_WT_PHONE_SECRET", ""))
    parser.add_argument("--arm-secret", default=os.environ.get("PHONE_ARM_WT_ARM_SECRET", ""))
    parser.add_argument("--max-datagram-bytes", type=int, default=int(os.environ.get("PHONE_ARM_WT_MAX_DATAGRAM_BYTES", "1024")))
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    global CONTROL_RELAY, EVENT_LOG_PATH
    EVENT_LOG_PATH = args.event_log
    phone_secret = _read_secret_arg(args.phone_secret)
    arm_secret = _read_secret_arg(args.arm_secret)
    if not phone_secret or not arm_secret:
        parser.error(
            "both --phone-secret and --arm-secret must resolve to non-empty values"
        )
    CONTROL_RELAY = ControlRelay(
        phone_secret=phone_secret,
        arm_secret=arm_secret,
        max_datagram_bytes=max(1, args.max_datagram_bytes),
    )

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
        "max_datagram_bytes": CONTROL_RELAY.max_datagram_bytes,
    })
    if args.stats_interval_s > 0:
        asyncio.create_task(_stats_loop(args.stats_interval_s))
    await serve(
        args.host,
        args.port,
        configuration=configuration,
        create_protocol=HttpServerProtocol,
        session_ticket_fetcher=store.pop,
        session_ticket_handler=store.add,
    )
    await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
