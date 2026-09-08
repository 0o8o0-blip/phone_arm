#!/usr/bin/env python3
"""Register a follower with the hosted browser API."""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


DEFAULT_API_URL = "https://188-166-154-201.sslip.io"


def _required(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError(f"required environment variable is missing: {name}")
    return value


def _relay_url(role: str) -> str:
    canonical = _required("PHONE_ARM_SESSION_RELAY_WT_URL")
    base = (
        os.environ.get("PHONE_ARM_SESSION_RELAY_WT_URL_PHONE", "").strip()
        if role == "phone"
        else canonical
    ) or canonical
    if base.startswith("wss://"):
        base = "https://" + base[len("wss://") :]
    base = base.rstrip("/")
    if base.endswith(("/phone", "/arm")):
        base = base.rsplit("/", 1)[0]
    elif not base.endswith("/wt"):
        base += "/wt"
    token_name = (
        "PHONE_ARM_SESSION_RELAY_ARM_TOKEN"
        if role == "arm"
        else "PHONE_ARM_SESSION_RELAY_PHONE_TOKEN"
    )
    query = urlencode(
        {
            "session": os.environ.get(
                "PHONE_ARM_SESSION_RELAY_SESSION", "default"
            ).strip()
            or "default",
            "token": _required(token_name),
        }
    )
    return f"{base}/{role}?{query}"


def _payload(mint_name: str | None, expires_s: float) -> dict[str, object]:
    turn_url = (
        os.environ.get("PHONE_ARM_TURN_URL_PHONE", "").strip()
        or _required("PHONE_ARM_TURN_URL")
    )
    session = (
        os.environ.get("PHONE_ARM_SESSION_RELAY_SESSION", "default").strip()
        or "default"
    )
    payload: dict[str, object] = {
        "follower_id": os.environ.get("PHONE_ARM_FOLLOWER_ID", socket.gethostname()),
        "session": session,
        "config": {
            "iceServers": [
                {
                    "urls": [turn_url],
                    "username": _required("PHONE_ARM_TURN_USER"),
                    "credential": _required("PHONE_ARM_TURN_PW"),
                }
            ],
            "iceTransportPolicy": "relay",
            "mediamtxWhepUrl": _required("PHONE_ARM_MEDIAMTX_WHEP_URL"),
            "mediamtxPlayToken": _required("PHONE_ARM_MEDIAMTX_PLAY_TOKEN"),
            "phoneRelayWtUrl": _relay_url("phone"),
            "leaderRelayWtUrl": _relay_url("arm"),
        },
    }
    if mint_name:
        payload["mint"] = {"name": mint_name, "expires_s": expires_s}
    return payload


def register(mint_name: str | None = None, expires_s: float = 7200) -> dict:
    api_url = os.environ.get("PHONE_ARM_HOSTED_API_URL", DEFAULT_API_URL).rstrip("/")
    body = json.dumps(_payload(mint_name, expires_s)).encode()
    request = Request(
        f"{api_url}/api/follower/register",
        data=body,
        headers={
            "Authorization": "Bearer "
            + _required("PHONE_ARM_SESSION_RELAY_ARM_TOKEN"),
            "Content-Type": "application/json",
            "User-Agent": "phone-arm-follower/1",
        },
        method="POST",
    )
    try:
        with urlopen(request, timeout=10) as response:
            return json.load(response)
    except HTTPError as exc:
        detail = exc.read(2048).decode(errors="replace").strip()
        raise RuntimeError(f"hosted API returned HTTP {exc.code}: {detail}") from exc
    except URLError as exc:
        raise RuntimeError(f"could not reach hosted API: {exc.reason}") from exc


def heartbeat(interval_s: float) -> None:
    failures = 0
    while True:
        try:
            register()
            if failures:
                print("[access] hosted API connection restored", flush=True)
            failures = 0
        except Exception as exc:  # noqa: BLE001 - heartbeat must keep retrying
            failures += 1
            if failures == 1 or failures % 12 == 0:
                print(f"[access] hosted API heartbeat failed: {exc}", file=sys.stderr, flush=True)
        time.sleep(interval_s)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    register_parser = subparsers.add_parser("register")
    register_parser.add_argument("--mint-name")
    register_parser.add_argument("--expires-s", type=float, default=7200)
    heartbeat_parser = subparsers.add_parser("heartbeat")
    heartbeat_parser.add_argument("--interval", type=float, default=5.0)
    args = parser.parse_args()

    if args.command == "register":
        result = register(args.mint_name, args.expires_s)
        if result.get("share_url"):
            print("[access] two-hour operator link:")
            print(result["share_url"])
        else:
            print("[access] follower registered with hosted API")
        return
    heartbeat(max(1.0, args.interval))


if __name__ == "__main__":
    main()
