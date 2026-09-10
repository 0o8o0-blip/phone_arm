#!/usr/bin/env python3
"""Hosted session authority for anonymously created Phone Arm robots."""
from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import hmac
import ipaddress
import json
import os
import re
import secrets
import shlex
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlencode

from aiohttp import web

try:
    from .capabilities import issue as issue_capability
    from .capabilities import verify as verify_capability
except ImportError:  # Direct deployment alongside capabilities.py.
    from capabilities import issue as issue_capability
    from capabilities import verify as verify_capability


SESSION_RE = re.compile(r"^r_[A-Za-z0-9_-]{12,40}$")
OWNER_TIMEOUT_S = 8.0
CREATE_LIMIT_PER_HOUR = 12
MAX_ACTIVE_FOLLOWERS = 100


def read_secret_arg(value: str) -> str:
    if value.startswith("@"):
        return Path(value[1:]).read_text().strip()
    return value.strip()


def _bearer(request: web.Request) -> str:
    value = request.headers.get("Authorization", "")
    return value[7:].strip() if value.startswith("Bearer ") else ""


def _token_hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class SessionApi:
    def __init__(
        self,
        *,
        capability_secret: str,
        turn_secret: str,
        public_url: str,
        relay_urls: dict[str, str],
        turn_urls: dict[str, str],
        state_file: Path,
        event_log: Path | None,
        follower_timeout_s: float,
        session_lifetime_s: float,
    ) -> None:
        self.capability_secret = capability_secret
        self.turn_secret = turn_secret
        self.public_url = public_url.rstrip("/")
        self.relay_urls = {key: value.rstrip("/") for key, value in relay_urls.items()}
        self.turn_urls = turn_urls
        self.state_file = state_file
        self.event_log = event_log
        self.follower_timeout_s = follower_timeout_s
        self.session_lifetime_s = session_lifetime_s
        self.tokens = self._load_tokens()
        self.followers: dict[str, dict[str, Any]] = {}
        self.owners: dict[str, dict[str, Any]] = {}
        self.creates_by_address: dict[str, list[float]] = {}

    def _load_tokens(self) -> list[dict[str, Any]]:
        try:
            data = json.loads(self.state_file.read_text())
        except FileNotFoundError:
            return []
        if not isinstance(data, list):
            raise RuntimeError("API token state must be a JSON list")
        return [entry for entry in data if isinstance(entry, dict)]

    def _save_tokens(self) -> None:
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_file.with_suffix(self.state_file.suffix + ".tmp")
        temporary.write_text(json.dumps(self.tokens, indent=2) + "\n")
        os.chmod(temporary, 0o600)
        temporary.replace(self.state_file)

    def _event(self, event: str, details: dict[str, Any]) -> None:
        if self.event_log is None:
            return
        self.event_log.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "t_utc_ms": round(time.time() * 1000.0, 3),
            "event": event,
            "details": details,
        }
        with self.event_log.open("a") as output:
            output.write(json.dumps(record, separators=(",", ":")) + "\n")

    def _active(self, follower: dict[str, Any], now: float | None = None) -> bool:
        current = time.time() if now is None else now
        return (
            current - float(follower["seen_at"]) <= self.follower_timeout_s
            and current < float(follower["expires_at"])
        )

    def _prune_expired(self, now: float | None = None) -> None:
        """Drop session state only after explicit expiry, not disconnection."""
        current = time.time() if now is None else now
        expired_sessions = {
            session
            for session, follower in self.followers.items()
            if float(follower.get("expires_at") or 0) <= current
        }
        for session in expired_sessions:
            self.followers.pop(session, None)
            self.owners.pop(session, None)

        old_token_count = len(self.tokens)
        self.tokens = [
            entry
            for entry in self.tokens
            if float(entry.get("expires_at") or 0) > current
        ]
        if len(self.tokens) != old_token_count:
            self._save_tokens()

        cutoff = current - 3600
        self.creates_by_address = {
            address: recent
            for address, stamps in self.creates_by_address.items()
            if (recent := [stamp for stamp in stamps if stamp > cutoff])
        }

    def _remote_address(self, request: web.Request) -> str:
        remote = request.remote or "unknown"
        try:
            trusted_proxy = ipaddress.ip_address(remote).is_loopback
        except ValueError:
            trusted_proxy = False
        if trusted_proxy:
            forwarded = request.headers.get("X-Forwarded-For", "").split(",", 1)[0].strip()
            if forwarded:
                return forwarded
        return remote

    def _check_create_rate(self, request: web.Request) -> None:
        now = time.time()
        self._prune_expired(now)
        address = self._remote_address(request)
        recent = self.creates_by_address.get(address, [])
        if len(recent) >= CREATE_LIMIT_PER_HOUR:
            raise web.HTTPTooManyRequests(text="robot creation rate limit reached\n")
        if sum(self._active(item, now) for item in self.followers.values()) >= MAX_ACTIVE_FOLLOWERS:
            raise web.HTTPServiceUnavailable(text="robot capacity reached\n")
        recent.append(now)
        self.creates_by_address[address] = recent

    def _capability(self, role: str, session: str, expires_at: float) -> str:
        return issue_capability(
            self.capability_secret,
            role=role,
            session=session,
            expires_at=expires_at,
        )

    @staticmethod
    def _edge(value: Any) -> str:
        return "asia" if str(value or "").lower() == "asia" else "europe"

    @staticmethod
    def _relay_endpoint(
        base: str,
        role: str,
        session: str,
        token: str,
        *,
        home_edge: str | None = None,
    ) -> str:
        if base.startswith("wss://"):
            base = "https://" + base[len("wss://") :]
        base = base.rstrip("/")
        if base.endswith(("/phone", "/arm")):
            base = base.rsplit("/", 1)[0]
        elif not base.endswith("/wt"):
            base += "/wt"
        query = {"session": session, "token": token}
        if home_edge:
            query["home"] = home_edge
        return f"{base}/{role}?{urlencode(query)}"

    def _mint_access(self, *, session: str, expires_at: float) -> str:
        now = time.time()
        self._prune_expired(now)
        token = secrets.token_urlsafe(32)
        self.tokens.append(
            {
                "name": "invite",
                "session": session,
                "hash": _token_hash(token),
                "expires_at": expires_at,
            }
        )
        self._save_tokens()
        return token

    def _access_token(self, value: str | None) -> dict[str, Any] | None:
        if not value:
            return None
        wanted_hash = _token_hash(value)
        now = time.time()
        self._prune_expired(now)
        for entry in self.tokens:
            stored_hash = str(entry.get("hash") or "")
            if stored_hash and hmac.compare_digest(stored_hash, wanted_hash):
                return entry
        return None

    def _browser_context(self, request: web.Request) -> tuple[dict[str, Any], dict[str, Any]]:
        token = self._access_token(_bearer(request) or request.query.get("t"))
        if token is None:
            raise web.HTTPUnauthorized(text="invalid or missing invitation token\n")
        session = str(token.get("session") or "")
        follower = self.followers.get(session)
        if follower is None or not self._active(follower):
            raise web.HTTPServiceUnavailable(text="robot is offline\n")
        return token, follower

    @staticmethod
    def _page_id(request: web.Request) -> str | None:
        value = (
            request.query.get("page_id")
            or request.query.get("app_page_id")
            or request.headers.get("X-Phone-Arm-Page-Id")
            or ""
        ).strip()
        return value[:80] or None

    async def create_follower(self, request: web.Request) -> web.Response:
        self._check_create_rate(request)
        try:
            body = await request.json()
        except Exception:
            body = {}
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text="request must be an object\n")
        now = time.time()
        session = "r_" + secrets.token_urlsafe(12)
        expires_at = now + self.session_lifetime_s
        access_expires_at = min(expires_at, now + 7200)
        follower_id = str(body.get("follower_id") or "robot")[:120]
        display_name = str(body.get("name") or follower_id)[:80]
        listed = bool(body.get("listed", False))
        video_available = bool(body.get("video_available", True))
        edge = self._edge(body.get("edge"))
        registration_token = self._capability("register", session, expires_at)
        arm_token = self._capability("arm", session, expires_at)
        publish_token = (
            self._capability("media-publish", session, expires_at)
            if video_available
            else ""
        )
        access_token = self._mint_access(session=session, expires_at=access_expires_at)
        self.followers[session] = {
            "session": session,
            "follower_id": follower_id,
            "name": display_name,
            "listed": listed,
            "video_available": video_available,
            "edge": edge,
            "seen_at": now,
            "expires_at": expires_at,
        }
        self._event(
            "follower_created",
            {
                "session": session,
                "follower_id": follower_id,
                "listed": listed,
                "video_available": video_available,
            },
        )
        return web.json_response(
            {
                "session": session,
                "registration_token": registration_token,
                "session_expires_at": expires_at,
                "edge": edge,
                "relay_url": self.relay_urls[edge],
                "arm_relay_token": arm_token,
                "video_available": video_available,
                "mediamtx_whip_url": (
                    f"{self.public_url}/media/{session}/whip"
                    if video_available
                    else ""
                ),
                "mediamtx_publish_token": publish_token,
                "share_url": f"{self.public_url}/robot/{session}#access={access_token}",
                "access_expires_at": access_expires_at,
            },
            headers={"Cache-Control": "no-store"},
        )

    async def register(self, request: web.Request) -> web.Response:
        self._prune_expired()
        try:
            body = await request.json()
        except Exception as exc:
            raise web.HTTPBadRequest(text="invalid JSON\n") from exc
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text="request must be an object\n")
        session = str(body.get("session") or "")
        if not SESSION_RE.fullmatch(session):
            raise web.HTTPBadRequest(text="invalid session\n")
        registration_token = _bearer(request)
        if not verify_capability(
            self.capability_secret,
            registration_token,
            role="register",
            session=session,
        ):
            raise web.HTTPUnauthorized(text="invalid robot capability\n")
        now = time.time()
        follower = self.followers.get(session)
        if follower is None:
            expiry = int(registration_token.split(".", 2)[1])
            follower = {
                "session": session,
                "follower_id": str(body.get("follower_id") or "robot")[:120],
                "name": str(body.get("name") or body.get("follower_id") or "robot")[:80],
                "listed": bool(body.get("listed", False)),
                "video_available": bool(body.get("video_available", True)),
                "edge": self._edge(body.get("edge")),
                "expires_at": expiry,
            }
            self.followers[session] = follower
            self._event("follower_restored", {"session": session})
        follower["seen_at"] = now
        follower["video_available"] = bool(
            body.get("video_available", follower.get("video_available", True))
        )
        # Preserve the arm's rendezvous region. Controllers still use their
        # own nearest ingress relay; that relay routes UDP here when needed.
        follower["edge"] = self._edge(body.get("edge", follower.get("edge")))
        return web.json_response(
            {"ok": True, "session": session, "lease_s": self.follower_timeout_s},
            headers={"Cache-Control": "no-store"},
        )

    def _turn_credentials(
        self, session: str, edge: str, expires_at: float
    ) -> dict[str, Any]:
        username = f"{int(expires_at)}:{session}"
        digest = hmac.new(
            self.turn_secret.encode(), username.encode(), hashlib.sha1
        ).digest()
        return {
            "urls": [
                self.turn_urls[edge],
                self.turn_urls["europe" if edge == "asia" else "asia"],
            ],
            "username": username,
            "credential": base64.b64encode(digest).decode(),
        }

    async def webrtc_config(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        session = follower["session"]
        controller_edge = self._edge(request.query.get("edge"))
        arm_edge = self._edge(follower.get("edge"))
        capability_expiry = min(float(token["expires_at"]), time.time() + 3600)
        video_available = bool(follower.get("video_available", True))
        wants_video = str(request.query.get("want_video") or "1").lower() not in {
            "0", "false", "no", "off"
        }
        response: dict[str, Any] = {
            "videoAvailable": video_available,
            "iceServers": (
                [self._turn_credentials(session, controller_edge, capability_expiry)]
                if video_available and wants_video
                else []
            ),
            "iceTransportPolicy": "relay",
            "controlRole": "viewer",
            "controlTransport": "viewer",
            "mediamtxWhepUrl": (
                f"{self.public_url}/media/{session}/whep"
                if video_available and wants_video
                else ""
            ),
            "mediamtxPlayToken": (
                self._capability("media-view", session, capability_expiry)
                if video_available and wants_video
                else ""
            ),
        }
        wants_control = str(request.query.get("want_control") or "").lower() in {
            "1", "true", "yes", "on", "controller"
        }
        if wants_control:
            page_id = self._page_id(request)
            now = time.time()
            owner = self.owners.get(session)
            if owner and now - float(owner["seen_at"]) > OWNER_TIMEOUT_S:
                self.owners.pop(session, None)
                owner = None
            if page_id and (owner is None or owner["page_id"] == page_id):
                claimed_at = now if owner is None else float(owner["claimed_at"])
                self.owners[session] = {
                    "page_id": page_id,
                    "token_name": token.get("name"),
                    "claimed_at": claimed_at,
                    "seen_at": now,
                }
                relay_token = self._capability(
                    f"phone@{arm_edge}", session, capability_expiry
                )
                response.update(
                    {
                        "controlRole": "controller",
                        "controlTransport": "relay-webtransport",
                        "controlOwnerPageId": page_id,
                        "controlOwnerAgeMs": round((now - claimed_at) * 1000.0, 1),
                        "controlOwnerTimeoutS": OWNER_TIMEOUT_S,
                        "controlEdge": controller_edge,
                        "armEdge": arm_edge,
                        "sessionRelayWtUrl": self._relay_endpoint(
                            self.relay_urls[controller_edge],
                            "phone",
                            session,
                            relay_token,
                            home_edge=arm_edge,
                        ),
                        "sessionRelaySession": session,
                    }
                )
            else:
                response["controlDeniedReason"] = (
                    "missing_page_id" if not page_id else "controller_active"
                )
        return web.json_response(response, headers={"Cache-Control": "no-store"})

    async def leader_config(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        session = follower["session"]
        controller_edge = self._edge(request.query.get("edge"))
        arm_edge = self._edge(follower.get("edge"))
        relay_token = self._capability(
            f"phone@{arm_edge}",
            session,
            min(float(token["expires_at"]), time.time() + 3600),
        )
        relay_url = self._relay_endpoint(
            self.relay_urls[controller_edge],
            "phone",
            session,
            relay_token,
            home_edge=arm_edge,
        )
        command = (
            "cd ~/dev/phone_arm && ./controllers/leader_arm/run.sh --url "
            + shlex.quote(relay_url)
        )
        return web.json_response(
            {
                "mode": "one_to_one",
                "session": session,
                "control_edge": controller_edge,
                "arm_edge": arm_edge,
                "command": command,
            },
            headers={"Cache-Control": "no-store"},
        )

    async def media_authorize(self, request: web.Request) -> web.Response:
        session = request.headers.get("X-Media-Session", "")
        action = request.headers.get("X-Media-Action", "")
        role = {"whip": "media-publish", "whep": "media-view"}.get(action)
        if (
            role is None
            or not SESSION_RE.fullmatch(session)
            or not verify_capability(
                self.capability_secret,
                _bearer(request),
                role=role,
                session=session,
            )
        ):
            raise web.HTTPUnauthorized(text="invalid media capability\n")
        return web.Response(status=204)

    async def release(self, request: web.Request) -> web.Response:
        _, follower = self._browser_context(request)
        page_id = self._page_id(request)
        if page_id is None:
            try:
                body = await request.json()
            except Exception:
                body = {}
            page_id = str(body.get("page_id") or body.get("app_page_id") or "")[:80] or None
        owner = self.owners.get(follower["session"])
        released = bool(owner and page_id and owner["page_id"] == page_id)
        if released:
            self.owners.pop(follower["session"], None)
        return web.json_response({"ok": True, "released": released})

    async def stats(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        return web.json_response(
            {
                "session": follower["session"],
                "follower_id": follower["follower_id"],
                "follower_seen_age_ms": round((time.time() - follower["seen_at"]) * 1000.0, 1),
                "token_name": token.get("name"),
                "control_owner": self.owners.get(follower["session"]),
            },
            headers={"Cache-Control": "no-store"},
        )

    async def browser_event(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        try:
            body = await request.json()
        except Exception as exc:
            raise web.HTTPBadRequest(text="invalid JSON\n") from exc
        if not isinstance(body, dict):
            raise web.HTTPBadRequest(text="event must be an object\n")
        body = dict(list(body.items())[:160])
        body["token_name"] = token.get("name")
        body["session"] = follower["session"]
        self._event("browser_event", body)
        return web.json_response({"ok": True})

    async def robots(self, _request: web.Request) -> web.Response:
        now = time.time()
        self._prune_expired(now)
        robots = [
            {
                "session": item["session"],
                "name": item["name"],
                "videoAvailable": bool(item.get("video_available", True)),
            }
            for item in self.followers.values()
            if item.get("listed") and self._active(item, now)
        ]
        return web.json_response({"robots": robots})

    async def health(self, _request: web.Request) -> web.Response:
        now = time.time()
        self._prune_expired(now)
        active = sum(self._active(item, now) for item in self.followers.values())
        return web.json_response({"ok": True, "active_followers": active})

    def app(self) -> web.Application:
        app = web.Application(client_max_size=128 * 1024)
        app.cleanup_ctx.append(self._cleanup_context)
        app.router.add_post("/api/follower/create", self.create_follower)
        app.router.add_post("/api/follower/register", self.register)
        app.router.add_get("/api/media/authorize", self.media_authorize)
        app.router.add_get("/api/robots", self.robots)
        app.router.add_get("/webrtc/config", self.webrtc_config)
        app.router.add_get("/leader/config", self.leader_config)
        app.router.add_post("/control/release", self.release)
        app.router.add_get("/stats", self.stats)
        app.router.add_post("/test_event", self.browser_event)
        app.router.add_get("/healthz", self.health)
        return app

    async def _cleanup_context(self, _app: web.Application):
        async def cleanup_loop() -> None:
            while True:
                await asyncio.sleep(60)
                self._prune_expired()

        task = asyncio.create_task(cleanup_loop())
        yield
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--capability-secret", required=True)
    parser.add_argument("--turn-secret", required=True)
    parser.add_argument("--public-url", required=True)
    parser.add_argument("--relay-url-europe", required=True)
    parser.add_argument("--relay-url-asia", required=True)
    parser.add_argument("--turn-url-europe", required=True)
    parser.add_argument("--turn-url-asia", required=True)
    parser.add_argument("--state-file", type=Path, required=True)
    parser.add_argument("--event-log", type=Path)
    parser.add_argument("--follower-timeout-s", type=float, default=20.0)
    parser.add_argument("--session-lifetime-s", type=float, default=86400.0)
    args = parser.parse_args()
    capability_secret = read_secret_arg(args.capability_secret)
    turn_secret = read_secret_arg(args.turn_secret)
    if not capability_secret or not turn_secret:
        parser.error("capability and TURN secrets must not be empty")
    api = SessionApi(
        capability_secret=capability_secret,
        turn_secret=turn_secret,
        public_url=args.public_url,
        relay_urls={"europe": args.relay_url_europe, "asia": args.relay_url_asia},
        turn_urls={"europe": args.turn_url_europe, "asia": args.turn_url_asia},
        state_file=args.state_file,
        event_log=args.event_log,
        follower_timeout_s=max(5.0, args.follower_timeout_s),
        session_lifetime_s=max(300.0, args.session_lifetime_s),
    )
    web.run_app(api.app(), host=args.host, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
