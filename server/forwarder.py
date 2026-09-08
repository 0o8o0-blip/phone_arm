#!/usr/bin/env python3
"""Regional WebTransport forwarder for an upstream Phone Arm relay.

Architecture: this process listens for WT CONNECT requests at /wt/phone or
/wt/arm and,
for each accepted session, opens its own WT client connection upstream to a
real relay (e.g. the London phone-arm-wt server). All datagrams flow:

    operator phone --(short leg)--> forwarder ==(persistent QUIC)==> upstream
                                                                       |
                                                                       v
                                                                Pi (arm peer,
                                                                  connected
                                                                  directly to
                                                                   upstream)

Why this is faster than putting the whole relay near the operator:

    operator -> SGP relay -> Pi      = operator_to_SGP + SGP_to_Pi
                                       (SGP_to_Pi over public internet,
                                        typically ~82 ms one-way and
                                        relatively jittery)

    operator -> SGP fwd  -> LON relay -> Pi
                                       = operator_to_SGP + SGP_to_LON
                                                         + LON_to_Pi
                                       (LON_to_Pi is LAN-fast, ~2 ms;
                                        SGP_to_LON is a dedicated long-running
                                        QUIC connection between two
                                        DigitalOcean data centers with good
                                        peering)

We get ~10 ms RTT improvement on a Manila->London tele-op path plus a
meaningful jitter reduction on the long leg.

Both roles are accepted. This makes the edge symmetric: an Asia controller can
reach a London robot through it, and an Asia robot can reach a London
controller through the same service.

USAGE
    python3 -m server.forwarder \\
        --host 0.0.0.0 --port 4434 \\
        --certificate /var/lib/caddy/.../cert.crt \\
        --private-key /var/lib/caddy/.../cert.key \\
        --upstream https://relay.example.com:4433/wt \\
        --event-log /var/log/phone-arm-wt-forwarder-events.jsonl
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
import urllib.parse
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

import aioquic
from aioquic.asyncio import serve as quic_serve
from aioquic.asyncio.protocol import QuicConnectionProtocol
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

# We reuse the project's small WT client wrapper.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shared.webtransport import connect_webtransport_datagrams


# --- Config ---------------------------------------------------------------
UPSTREAM_ORIGIN = ""           # set from --upstream (scheme://host:port, no path)
EVENT_LOG: str | None = None   # set from --event-log
MAX_DATAGRAM_BYTES = 1024      # set from --max-datagram-bytes

EDGE_TIMING_TRIM_FIELDS = (
    # Browser already trims these first when building a datagram. If the
    # forwarder needs a few more bytes for timing stamps, keep the timing and
    # drop lower-value diagnostics instead of silently losing attribution.
    "ctrl_last_reconnect_reason",
    "wrtc_freezes",
    "wrtc_decode",
    "wrtc_jbuf",
    "wrtc_rtt",
    "wrtc_jitter",
    "wrtc_fps",
    "wrtc_via",
)


def _now_ms() -> float:
    return time.time() * 1000.0


def _emit(event: str, details: dict) -> None:
    rec = {
        "t_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "t_ms": round(_now_ms(), 3),
        "event": event,
        "details": details,
    }
    line = json.dumps(rec, ensure_ascii=False)
    print(line, flush=True)
    if EVENT_LOG:
        try:
            with open(EVENT_LOG, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass


# --- Per-session forwarder ------------------------------------------------
@dataclass
class PendingDatagram:
    data: bytes
    msg: dict[str, Any] | None
    recv_ms: float


@dataclass
class ForwarderSession:
    """One forwarded WT session: incoming server stream <-> upstream client."""
    handler: "WebTransportHandler"
    upstream_url: str
    upstream_ctx: Any = None
    upstream_wt: Any = None
    up_to_down_task: asyncio.Task | None = None
    rx_from_client: int = 0
    rx_from_upstream: int = 0
    tx_to_upstream: int = 0
    overwritten_to_upstream: int = 0
    debug_dropped_for_pose: int = 0
    debug_overwritten_to_upstream: int = 0
    edge_ack_sent: int = 0
    edge_timing_stamped: int = 0
    edge_timing_trimmed: int = 0
    edge_timing_omitted: int = 0
    closed: bool = False
    session_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    started_ms: float = field(default_factory=_now_ms)
    latest_to_upstream: PendingDatagram | None = None
    latest_event: asyncio.Event = field(default_factory=asyncio.Event)
    client_to_up_task: asyncio.Task | None = None

    async def start(self) -> None:
        try:
            ctx = connect_webtransport_datagrams(self.upstream_url)
            self.upstream_ctx = ctx
            self.upstream_wt = await ctx.__aenter__()
            _emit("upstream_connected", {
                "session_id": self.session_id,
                "url_host": urllib.parse.urlsplit(self.upstream_url).hostname,
                "client": self.handler.client,
            })
        except Exception as e:  # noqa: BLE001
            _emit("upstream_connect_failed", {
                "session_id": self.session_id,
                "error": repr(e),
                "client": self.handler.client,
            })
            self.handler.close("upstream connect failed")
            return
        self.client_to_up_task = asyncio.create_task(self._pump_client_latest_to_upstream())
        self.up_to_down_task = asyncio.create_task(self._pump_upstream_to_client())

    def _send_edge_ack(self, msg: dict[str, Any] | None, recv_ms: float) -> None:
        if not isinstance(msg, dict) or msg.get("t") is None:
            return
        if msg.get("type") == "debug":
            return
        ack = {
            "type": "edge_ack",
            "ack_t": msg.get("t"),
            "ack_seq": msg.get("seq"),
            "t_edge_recv_ms": round(recv_ms, 3),
            "t_edge_send_ms": round(_now_ms(), 3),
            "edge": "wt_forwarder",
        }
        self.handler.send_json_datagram(ack)
        self.edge_ack_sent += 1

    @staticmethod
    def _decode_json_dict(data: bytes) -> dict[str, Any] | None:
        try:
            msg = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        return msg if isinstance(msg, dict) else None

    @staticmethod
    def _encode_json(msg: dict[str, Any]) -> bytes:
        return json.dumps(msg, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

    def _with_edge_timing(self, pending: PendingDatagram, send_ms: float) -> bytes:
        if pending.msg is None:
            return pending.data
        msg = dict(pending.msg)
        msg["_edge_recv_ms"] = round(pending.recv_ms, 3)
        msg["_edge_send_ms"] = round(send_ms, 3)
        msg["_edge_q_ms"] = round(max(0.0, send_ms - pending.recv_ms), 3)
        stamped = self._encode_json(msg)
        if len(stamped) <= MAX_DATAGRAM_BYTES:
            self.edge_timing_stamped += 1
            return stamped

        trimmed = False
        for field_name in EDGE_TIMING_TRIM_FIELDS:
            if field_name in msg:
                msg.pop(field_name, None)
                trimmed = True
        if trimmed:
            stamped = self._encode_json(msg)
            if len(stamped) <= MAX_DATAGRAM_BYTES:
                self.edge_timing_stamped += 1
                self.edge_timing_trimmed += 1
                return stamped

        # Do not turn a valid phone datagram into an oversize upstream datagram.
        # Emit only a few examples; otherwise one oversized stream can flood logs.
        self.edge_timing_omitted += 1
        if self.edge_timing_omitted <= 3:
            _emit("edge_timing_omitted", {
                "session_id": self.session_id,
                "seq": pending.msg.get("seq"),
                "original_bytes": len(pending.data),
                "stamped_bytes": len(stamped),
                "limit": MAX_DATAGRAM_BYTES,
            })
        return pending.data

    async def _pump_client_latest_to_upstream(self) -> None:
        try:
            while not self.closed:
                await self.latest_event.wait()
                self.latest_event.clear()
                pending = self.latest_to_upstream
                self.latest_to_upstream = None
                if pending is None or self.upstream_wt is None:
                    continue
                try:
                    payload = self._with_edge_timing(pending, _now_ms())
                    pending_type = (
                        pending.msg.get("type")
                        if isinstance(pending.msg, dict)
                        else None
                    )
                    send_latest = getattr(self.upstream_wt, "send_latest", None)
                    if pending_type != "debug" and send_latest is not None:
                        send_latest(payload)
                    else:
                        self.upstream_wt.send(payload)
                    self.tx_to_upstream += 1
                except Exception as e:  # noqa: BLE001
                    _emit("upstream_send_error", {
                        "session_id": self.session_id,
                        "error": repr(e),
                    })
        except asyncio.CancelledError:
            raise

    async def _pump_upstream_to_client(self) -> None:
        try:
            while not self.closed:
                data = await self.upstream_wt.recv()
                if data is None:
                    _emit("upstream_eof", {"session_id": self.session_id})
                    break
                self.rx_from_upstream += 1
                # The forwarder owns this datagram path; send to client via H3.
                self.handler.send_datagram(data)
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001
            _emit("upstream_recv_error", {
                "session_id": self.session_id,
                "error": repr(e),
            })
        finally:
            if not self.closed:
                self.handler.close("upstream closed")

    def forward_client_datagram(self, data: bytes) -> None:
        self.rx_from_client += 1
        recv_ms = _now_ms()
        msg = self._decode_json_dict(data)
        self._send_edge_ack(msg, recv_ms)
        msg_type = msg.get("type") if isinstance(msg, dict) else None
        pending_type = (
            self.latest_to_upstream.msg.get("type")
            if self.latest_to_upstream is not None
            and isinstance(self.latest_to_upstream.msg, dict)
            else None
        )
        if msg_type == "debug" and self.latest_to_upstream is not None and pending_type != "debug":
            self.debug_dropped_for_pose += 1
            return
        if self.latest_to_upstream is not None:
            self.overwritten_to_upstream += 1
            if pending_type == "debug":
                self.debug_overwritten_to_upstream += 1
        self.latest_to_upstream = PendingDatagram(data=data, msg=msg, recv_ms=recv_ms)
        self.latest_event.set()

    async def close(self, reason: str) -> None:
        if self.closed:
            return
        self.closed = True
        if self.up_to_down_task is not None and not self.up_to_down_task.done():
            self.up_to_down_task.cancel()
            try:
                await self.up_to_down_task
            except (asyncio.CancelledError, Exception):
                pass
        if self.client_to_up_task is not None and not self.client_to_up_task.done():
            self.client_to_up_task.cancel()
            try:
                await self.client_to_up_task
            except (asyncio.CancelledError, Exception):
                pass
        if self.upstream_ctx is not None:
            try:
                await self.upstream_ctx.__aexit__(None, None, None)
            except Exception:  # noqa: BLE001
                pass
        _emit("forwarder_session_closed", {
            "session_id": self.session_id,
            "reason": reason,
            "duration_s": round((_now_ms() - self.started_ms) / 1000.0, 3),
            "rx_from_client": self.rx_from_client,
            "rx_from_upstream": self.rx_from_upstream,
            "tx_to_upstream": self.tx_to_upstream,
            "overwritten_to_upstream": self.overwritten_to_upstream,
            "debug_dropped_for_pose": self.debug_dropped_for_pose,
            "debug_overwritten_to_upstream": self.debug_overwritten_to_upstream,
            "edge_ack_sent": self.edge_ack_sent,
            "edge_timing_stamped": self.edge_timing_stamped,
            "edge_timing_trimmed": self.edge_timing_trimmed,
            "edge_timing_omitted": self.edge_timing_omitted,
        })


# --- WT-stream handler (server side) --------------------------------------
class WebTransportHandler:
    """One WT session as the operator phone sees it."""

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
        self.forwarder: ForwarderSession | None = None

    def accept_and_forward(self) -> None:
        self.accepted = True
        self.connection.send_headers(
            stream_id=self.stream_id,
            headers=[
                (b":status", b"200"),
                (b"server", f"phone-arm-wt-forwarder/aioquic-{aioquic.__version__}".encode()),
                (b"sec-webtransport-http3-draft", b"draft02"),
            ],
        )
        self.transmit()
        # Preserve the original path and query string so the upstream relay sees
        # exactly what the operator sent us. --upstream is the origin only
        # (https://host:port), no path; we append the request path verbatim.
        query_string = urllib.parse.urlencode({k: v[0] for k, v in self.query.items()})
        upstream_url = UPSTREAM_ORIGIN + self.path + (f"?{query_string}" if query_string else "")
        self.forwarder = ForwarderSession(handler=self, upstream_url=upstream_url)
        asyncio.create_task(self.forwarder.start())
        _emit("wt_accept", {
            "stream_id": self.stream_id,
            "path": self.path,
            "session": (self.query.get("session") or ["default"])[0],
            "client": self.client,
        })

    def reject(self, status: int = 404) -> None:
        self.closed = True
        self.connection.send_headers(
            stream_id=self.stream_id,
            headers=[(b":status", str(status).encode())],
            end_stream=True,
        )
        self.transmit()

    def send_datagram(self, data: bytes) -> None:
        if self.closed:
            return
        self.connection.send_datagram(stream_id=self.stream_id, data=data)
        self.transmit()

    def send_json_datagram(self, msg: dict[str, Any]) -> None:
        payload = json.dumps(msg, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_datagram(payload)

    def handle_datagram(self, data: bytes) -> None:
        if self.forwarder is not None:
            self.forwarder.forward_client_datagram(data)

    def close(self, reason: str) -> None:
        if self.closed:
            return
        self.closed = True
        if self.forwarder is not None:
            asyncio.create_task(self.forwarder.close(reason))


# --- HTTP/3 + QUIC server protocol ----------------------------------------
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

    def _text(self, stream_id: int, status: int, body: str) -> None:
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
                    handler.accept_and_forward()
                else:
                    handler.reject(404)
                return
            if method == "GET" and path == "/healthz":
                self._text(event.stream_id, 200, "ok\n")
                return
            self._text(event.stream_id, 200,
                       "phone-arm WebTransport edge\nforwarding /wt/phone and /wt/arm -> upstream\n")
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
                _emit("protocol", {"alpn": event.alpn_protocol, "client": self._client()})
            else:
                _emit("protocol_rejected", {"alpn": event.alpn_protocol})
        if self._http is not None:
            for ev in self._http.handle_event(event):
                self.http_event_received(ev)

    def connection_lost(self, exc: Exception | None) -> None:
        for h in list(self._handlers.values()):
            h.close("connection lost")
        super().connection_lost(exc)


class SessionTicketStore:
    def __init__(self) -> None:
        self.tickets: dict[bytes, SessionTicket] = {}

    def add(self, ticket: SessionTicket) -> None:
        self.tickets[ticket.ticket] = ticket

    def pop(self, label: bytes) -> SessionTicket | None:
        return self.tickets.pop(label, None)


# --- main -----------------------------------------------------------------
async def amain() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=4434)
    p.add_argument("--certificate", required=True)
    p.add_argument("--private-key", required=True)
    p.add_argument("--upstream", required=True,
                   help="Upstream origin (scheme://host:port). Request path is appended verbatim.")
    p.add_argument("--event-log", default=None)
    p.add_argument("--max-datagram-bytes", type=int, default=1024)
    args = p.parse_args()

    global UPSTREAM_ORIGIN, EVENT_LOG, MAX_DATAGRAM_BYTES
    # Strip any trailing path so callers can pass the WT URL as-is.
    parsed = urllib.parse.urlsplit(args.upstream)
    UPSTREAM_ORIGIN = f"{parsed.scheme}://{parsed.netloc}"
    EVENT_LOG = args.event_log
    MAX_DATAGRAM_BYTES = args.max_datagram_bytes

    cfg = QuicConfiguration(is_client=False, alpn_protocols=H3_ALPN)
    cfg.load_cert_chain(args.certificate, args.private_key)
    cfg.max_datagram_frame_size = max(args.max_datagram_bytes + 64, 1280)

    store = SessionTicketStore()
    _emit("forwarder_starting", {
        "host": args.host,
        "port": args.port,
        "upstream_origin": UPSTREAM_ORIGIN,
    })
    server = await quic_serve(
        host=args.host,
        port=args.port,
        configuration=cfg,
        create_protocol=HttpServerProtocol,
        session_ticket_fetcher=store.pop,
        session_ticket_handler=store.add,
    )
    try:
        await asyncio.Event().wait()
    finally:
        server.close()


def main() -> None:
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
