"""Small aioquic WebTransport datagram client.

Only the datagram path needed by phone-arm control is implemented. Reliable
streams are intentionally left out so the control path cannot accidentally grow
ordered/reliable backlog semantics.
"""
from __future__ import annotations

import asyncio
import ssl
import urllib.parse
from dataclasses import dataclass

from aioquic.asyncio import QuicConnectionProtocol, connect
from aioquic.h3.connection import H3_ALPN, H3Connection
from aioquic.h3.events import (
    DatagramReceived,
    DataReceived,
    H3Event,
    HeadersReceived,
)
from aioquic.quic.configuration import QuicConfiguration
from aioquic.quic.events import ProtocolNegotiated, QuicEvent


class WebTransportError(RuntimeError):
    pass


class WebTransportClientProtocol(QuicConnectionProtocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._http: H3Connection | None = None
        self._http_ready = asyncio.Event()
        self._session_id: int | None = None
        self._ready = asyncio.Event()
        self._closed = asyncio.Event()
        self._error: Exception | None = None
        self.datagrams: asyncio.Queue[bytes | None] = asyncio.Queue()

    def quic_event_received(self, event: QuicEvent) -> None:
        if isinstance(event, ProtocolNegotiated):
            if event.alpn_protocol not in H3_ALPN:
                self._error = WebTransportError(f"HTTP/3 not negotiated: {event.alpn_protocol!r}")
                self._closed.set()
            else:
                self._http = H3Connection(self._quic, enable_webtransport=True)
                self._http_ready.set()
        if self._http is None:
            return
        for http_event in self._http.handle_event(event):
            self._handle_h3_event(http_event)

    def _handle_h3_event(self, event: H3Event) -> None:
        if isinstance(event, HeadersReceived) and event.stream_id == self._session_id:
            headers = dict(event.headers)
            status = int(headers.get(b":status", b"0") or b"0")
            if 200 <= status < 300:
                self._ready.set()
            else:
                self._error = WebTransportError(f"WebTransport CONNECT failed: {status}")
                self._closed.set()
        elif isinstance(event, DatagramReceived) and event.stream_id == self._session_id:
            self.datagrams.put_nowait(event.data)
        elif isinstance(event, DataReceived) and event.stream_id == self._session_id and event.stream_ended:
            self._closed.set()
            self.datagrams.put_nowait(None)

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None and self._error is None:
            self._error = exc
        self._closed.set()
        self.datagrams.put_nowait(None)
        super().connection_lost(exc)

    async def open_webtransport(self, authority: str, path: str) -> None:
        await asyncio.wait_for(self._http_ready.wait(), timeout=10.0)
        if self._http is None:
            raise WebTransportError("HTTP/3 connection is not ready")
        self._session_id = self._quic.get_next_available_stream_id()
        self._http.send_headers(
            stream_id=self._session_id,
            headers=[
                (b":method", b"CONNECT"),
                (b":scheme", b"https"),
                (b":authority", authority.encode("ascii")),
                (b":path", path.encode("ascii")),
                (b":protocol", b"webtransport"),
                (b"sec-webtransport-http3-draft", b"draft02"),
            ],
        )
        self.transmit()
        ready = asyncio.create_task(self._ready.wait())
        closed = asyncio.create_task(self._closed.wait())
        done, pending = await asyncio.wait({ready, closed}, return_when=asyncio.FIRST_COMPLETED, timeout=10.0)
        for task in pending:
            task.cancel()
        if not done:
            raise TimeoutError("WebTransport CONNECT timed out")
        if self._error is not None:
            raise self._error
        if not self._ready.is_set():
            raise WebTransportError("WebTransport session closed before ready")

    def send_datagram(self, data: bytes) -> None:
        if self._http is None or self._session_id is None or self._closed.is_set():
            raise WebTransportError("WebTransport session is not open")
        self._http.send_datagram(stream_id=self._session_id, data=data)
        self.transmit()

    def send_latest_datagram(self, data: bytes) -> None:
        """Send one latest-state datagram, replacing unsent datagrams.

        aioquic exposes HTTP/3 datagram sending as an append-only pending list.
        This helper is intentionally limited to this single-session WT client:
        before queuing the newest pose, drop any older DATAGRAM frames that have
        not yet been packetized by QUIC. Already-sent packets remain governed by
        QUIC, but local backlog cannot build up.
        """
        if self._http is None or self._session_id is None or self._closed.is_set():
            raise WebTransportError("WebTransport session is not open")
        pending = getattr(self._quic, "_datagrams_pending", None)
        if pending is not None:
            pending.clear()
        self._http.send_datagram(stream_id=self._session_id, data=data)
        self.transmit()

    async def recv_datagram(self) -> bytes | None:
        data = await self.datagrams.get()
        if data is None and self._error is not None:
            raise self._error
        return data

    async def wait_closed(self) -> None:
        await self._closed.wait()
        if self._error is not None:
            raise self._error


@dataclass
class WebTransportDatagramSession:
    protocol: WebTransportClientProtocol

    def send(self, data: bytes) -> None:
        self.protocol.send_datagram(data)

    def send_latest(self, data: bytes) -> None:
        self.protocol.send_latest_datagram(data)

    async def recv(self) -> bytes | None:
        return await self.protocol.recv_datagram()


class connect_webtransport_datagrams:
    def __init__(self, url: str, *, verify_tls: bool = True) -> None:
        parsed = urllib.parse.urlsplit(url)
        if parsed.scheme != "https":
            raise ValueError("WebTransport URL must use https://")
        if not parsed.hostname:
            raise ValueError("WebTransport URL must include a host")
        self._host = parsed.hostname
        self._port = parsed.port or 443
        self._authority = parsed.netloc
        self._path = urllib.parse.urlunsplit(("", "", parsed.path or "/", parsed.query, ""))
        self._verify_tls = verify_tls
        self._connect_cm = None
        self._protocol: WebTransportClientProtocol | None = None

    async def __aenter__(self) -> WebTransportDatagramSession:
        configuration = QuicConfiguration(
            alpn_protocols=H3_ALPN,
            is_client=True,
            max_datagram_frame_size=65536,
        )
        if not self._verify_tls:
            configuration.verify_mode = ssl.CERT_NONE
        self._connect_cm = connect(
            self._host,
            self._port,
            configuration=configuration,
            create_protocol=WebTransportClientProtocol,
        )
        protocol = await self._connect_cm.__aenter__()
        assert isinstance(protocol, WebTransportClientProtocol)
        await protocol.open_webtransport(self._authority, self._path)
        self._protocol = protocol
        return WebTransportDatagramSession(protocol=protocol)

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._connect_cm is not None:
            await self._connect_cm.__aexit__(exc_type, exc, tb)
