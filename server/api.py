#!/usr/bin/env python3
"""VPS web API for follower registration and browser access."""
from __future__ import annotations

import argparse
import hmac
import json
import os
import re
import secrets
import shlex
import time
from pathlib import Path
from typing import Any

from aiohttp import web


SESSION_RE = re.compile(r"^[A-Za-z0-9._-]{1,80}$")
OWNER_TIMEOUT_S = 8.0
CONFIG_KEYS = {
    "iceServers",
    "iceTransportPolicy",
    "mediamtxWhepUrl",
    "mediamtxPlayToken",
    "phoneRelayWtUrl",
    "leaderRelayWtUrl",
}


def read_secret_arg(value: str) -> str:
    if value.startswith("@"):
        return Path(value[1:]).read_text().strip()
    return value.strip()


class SessionApi:
    def __init__(
        self,
        *,
        arm_secret: str,
        public_url: str,
        state_file: Path,
        event_log: Path | None,
        follower_timeout_s: float,
    ) -> None:
        self.arm_secret = arm_secret
        self.public_url = public_url.rstrip("/")
        self.state_file = state_file
        self.event_log = event_log
        self.follower_timeout_s = follower_timeout_s
        self.tokens = self._load_tokens()
        self.followers: dict[str, dict[str, Any]] = {}
        self.owners: dict[str, dict[str, Any]] = {}

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

    def _follower_authorized(self, request: web.Request) -> bool:
        authorization = request.headers.get("Authorization", "")
        supplied = authorization[7:] if authorization.startswith("Bearer ") else ""
        return hmac.compare_digest(supplied, self.arm_secret)

    def _token(self, value: str | None) -> dict[str, Any] | None:
        if not value:
            return None
        now = time.time()
        for entry in self.tokens:
            if not hmac.compare_digest(str(entry.get("value", "")), value):
                continue
            expires_at = entry.get("expires_at")
            if expires_at is not None and float(expires_at) <= now:
                return None
            return entry
        return None

    def _browser_context(
        self, request: web.Request
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        token = self._token(request.query.get("t"))
        if token is None:
            raise web.HTTPUnauthorized(text="invalid or missing token\n")
        session = str(token.get("session") or "default")
        follower = self.followers.get(session)
        if follower is None:
            raise web.HTTPServiceUnavailable(text="follower is not registered\n")
        if time.time() - float(follower["seen_at"]) > self.follower_timeout_s:
            raise web.HTTPServiceUnavailable(text="follower registration is stale\n")
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

    def _mint(
        self, *, name: str, session: str, expires_s: float
    ) -> dict[str, Any]:
        now = time.time()
        self.tokens = [
            token
            for token in self.tokens
            if token.get("expires_at") is None or float(token["expires_at"]) > now
        ]
        if any(token.get("name") == name for token in self.tokens):
            raise web.HTTPConflict(text=f"token name already exists: {name}\n")
        entry = {
            "name": name,
            "session": session,
            "value": secrets.token_urlsafe(32),
            "expires_at": now + expires_s,
        }
        self.tokens.append(entry)
        self._save_tokens()
        self._event(
            "token_minted",
            {"name": name, "session": session, "expires_s": expires_s},
        )
        return entry

    async def register(self, request: web.Request) -> web.Response:
        if not self._follower_authorized(request):
            raise web.HTTPUnauthorized(text="invalid follower credential\n")
        try:
            body = await request.json()
        except Exception as exc:
            raise web.HTTPBadRequest(text="invalid JSON\n") from exc
        session = str(body.get("session") or "default")
        if not SESSION_RE.fullmatch(session):
            raise web.HTTPBadRequest(text="invalid session\n")
        raw_config = body.get("config")
        if not isinstance(raw_config, dict):
            raise web.HTTPBadRequest(text="config must be an object\n")
        config = {
            key: raw_config[key] for key in CONFIG_KEYS if key in raw_config
        }
        required = {
            "iceServers",
            "mediamtxWhepUrl",
            "mediamtxPlayToken",
            "phoneRelayWtUrl",
            "leaderRelayWtUrl",
        }
        missing = sorted(required - config.keys())
        if missing:
            raise web.HTTPBadRequest(
                text=f"missing config: {', '.join(missing)}\n"
            )
        follower_id = str(body.get("follower_id") or "follower")[:120]
        first_registration = session not in self.followers
        self.followers[session] = {
            "session": session,
            "follower_id": follower_id,
            "config": config,
            "seen_at": time.time(),
        }
        if first_registration:
            self._event(
                "follower_registered",
                {"session": session, "follower_id": follower_id},
            )
        response: dict[str, Any] = {
            "ok": True,
            "session": session,
            "lease_s": self.follower_timeout_s,
        }
        mint = body.get("mint")
        if isinstance(mint, dict):
            name = str(mint.get("name") or "").strip()[:120]
            expires_s = float(mint.get("expires_s") or 7200)
            if not name or not 60 <= expires_s <= 86400 * 30:
                raise web.HTTPBadRequest(text="invalid token request\n")
            entry = self._mint(
                name=name, session=session, expires_s=expires_s
            )
            response.update(
                {
                    "token_name": name,
                    "expires_at": entry["expires_at"],
                    "share_url": f"{self.public_url}/?t={entry['value']}",
                }
            )
        return web.json_response(
            response, headers={"Cache-Control": "no-store"}
        )

    async def webrtc_config(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        config = follower["config"]
        response: dict[str, Any] = {
            "iceServers": config["iceServers"],
            "iceTransportPolicy": config.get(
                "iceTransportPolicy", "relay"
            ),
            "controlRole": "viewer",
            "controlTransport": "viewer",
            "mediamtxWhepUrl": config["mediamtxWhepUrl"],
            "mediamtxPlayToken": config["mediamtxPlayToken"],
        }
        wants_control = str(
            request.query.get("want_control") or ""
        ).lower() in {"1", "true", "yes", "on", "controller"}
        if wants_control:
            session = follower["session"]
            page_id = self._page_id(request)
            now = time.time()
            owner = self.owners.get(session)
            if owner and now - float(owner["seen_at"]) > OWNER_TIMEOUT_S:
                self.owners.pop(session, None)
                owner = None
            if page_id and (
                owner is None or owner["page_id"] == page_id
            ):
                claimed_at = (
                    now if owner is None else float(owner["claimed_at"])
                )
                self.owners[session] = {
                    "page_id": page_id,
                    "token_name": token.get("name"),
                    "claimed_at": claimed_at,
                    "seen_at": now,
                }
                response.update(
                    {
                        "controlRole": "controller",
                        "controlTransport": "relay-webtransport",
                        "controlOwnerPageId": page_id,
                        "controlOwnerAgeMs": round(
                            (now - claimed_at) * 1000.0, 1
                        ),
                        "controlOwnerTimeoutS": OWNER_TIMEOUT_S,
                        "sessionRelayWtUrl": config["phoneRelayWtUrl"],
                        "sessionRelaySession": session,
                    }
                )
            else:
                response["controlDeniedReason"] = (
                    "missing_page_id" if not page_id else "controller_active"
                )
                if owner:
                    response["controlOwnerPageId"] = owner["page_id"]
        return web.json_response(
            response, headers={"Cache-Control": "no-store"}
        )

    async def leader_config(self, request: web.Request) -> web.Response:
        _, follower = self._browser_context(request)
        relay_url = str(follower["config"]["leaderRelayWtUrl"])
        command = (
            "cd ~/dev/phone_arm && "
            "./controllers/leader_arm/run.sh --url "
            + shlex.quote(relay_url)
        )
        return web.json_response(
            {
                "mode": "one_to_one",
                "session": follower["session"],
                "command": command,
            },
            headers={"Cache-Control": "no-store"},
        )

    async def release(self, request: web.Request) -> web.Response:
        _, follower = self._browser_context(request)
        page_id = self._page_id(request)
        if page_id is None:
            try:
                body = await request.json()
            except Exception:
                body = {}
            page_id = str(
                body.get("page_id") or body.get("app_page_id") or ""
            )[:80] or None
        owner = self.owners.get(follower["session"])
        released = bool(
            owner and page_id and owner["page_id"] == page_id
        )
        if released:
            self.owners.pop(follower["session"], None)
        return web.json_response({"ok": True, "released": released})

    async def stats(self, request: web.Request) -> web.Response:
        token, follower = self._browser_context(request)
        return web.json_response(
            {
                "session": follower["session"],
                "follower_id": follower["follower_id"],
                "follower_seen_age_ms": round(
                    (time.time() - follower["seen_at"]) * 1000.0, 1
                ),
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

    async def health(self, _request: web.Request) -> web.Response:
        now = time.time()
        active = sum(
            1
            for follower in self.followers.values()
            if now - follower["seen_at"] <= self.follower_timeout_s
        )
        return web.json_response(
            {"ok": True, "active_followers": active}
        )

    def app(self) -> web.Application:
        app = web.Application(client_max_size=128 * 1024)
        app.router.add_post("/api/follower/register", self.register)
        app.router.add_get("/webrtc/config", self.webrtc_config)
        app.router.add_get("/leader/config", self.leader_config)
        app.router.add_post("/control/release", self.release)
        app.router.add_get("/stats", self.stats)
        app.router.add_post("/test_event", self.browser_event)
        app.router.add_get("/healthz", self.health)
        return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--arm-secret", required=True)
    parser.add_argument("--public-url", required=True)
    parser.add_argument("--state-file", type=Path, required=True)
    parser.add_argument("--event-log", type=Path)
    parser.add_argument("--follower-timeout-s", type=float, default=20.0)
    args = parser.parse_args()
    arm_secret = read_secret_arg(args.arm_secret)
    if not arm_secret:
        parser.error("arm secret must not be empty")
    api = SessionApi(
        arm_secret=arm_secret,
        public_url=args.public_url,
        state_file=args.state_file,
        event_log=args.event_log,
        follower_timeout_s=max(5.0, args.follower_timeout_s),
    )
    web.run_app(
        api.app(), host=args.host, port=args.port, access_log=None
    )


if __name__ == "__main__":
    main()
