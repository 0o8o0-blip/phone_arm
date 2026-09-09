#!/usr/bin/env python3
"""Create and maintain an anonymous hosted robot session."""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import shlex
import socket
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_API_URL = "https://188-166-154-201.sslip.io"
EDGE_PROBES = {
    "europe": "https://188-166-154-201.sslip.io/healthz",
    "asia": "https://146-190-104-81.sslip.io/",
}


def _probe_edge(item: tuple[str, str]) -> tuple[str, float]:
    name, url = item
    started = time.monotonic()
    try:
        with urlopen(Request(url, headers={"User-Agent": "phone-arm-edge-probe/1"}), timeout=3):
            return name, time.monotonic() - started
    except Exception:
        return name, float("inf")


def choose_edge() -> str:
    requested = os.environ.get("PHONE_ARM_EDGE", "").strip().lower()
    if requested in EDGE_PROBES:
        return requested
    api_url = os.environ.get("PHONE_ARM_HOSTED_API_URL", DEFAULT_API_URL)
    if api_url.startswith(("http://127.0.0.1", "http://localhost")):
        return "europe"
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = dict(pool.map(_probe_edge, EDGE_PROBES.items()))
    return min(results, key=results.get) if any(value < float("inf") for value in results.values()) else "europe"


def _post(path: str, body: dict, token: str = "") -> dict:
    api_url = os.environ.get("PHONE_ARM_HOSTED_API_URL", DEFAULT_API_URL).rstrip("/")
    headers = {
        "Content-Type": "application/json",
        "User-Agent": "phone-arm-follower/2",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    encoded_body = json.dumps(body).encode()
    last_error: Exception | None = None
    for attempt in range(1, 4):
        request = Request(
            api_url + path,
            data=encoded_body,
            headers=headers,
            method="POST",
        )
        try:
            with urlopen(request, timeout=15) as response:
                return json.load(response)
        except HTTPError as exc:
            detail = exc.read(2048).decode(errors="replace").strip()
            # Authentication, rate-limit and validation failures need operator
            # action. Only retry server-side failures that may be transient.
            if exc.code < 500 or attempt == 3:
                raise RuntimeError(
                    f"hosted API returned HTTP {exc.code}: {detail}"
                ) from exc
            last_error = exc
        except (URLError, TimeoutError) as exc:
            last_error = exc
            if attempt == 3:
                break
        delay_s = 2 ** (attempt - 1)
        reason = getattr(last_error, "reason", last_error)
        print(
            f"[access] hosted API attempt {attempt}/3 failed: {reason}; "
            f"retrying in {delay_s}s",
            file=sys.stderr,
            flush=True,
        )
        time.sleep(delay_s)
    reason = getattr(last_error, "reason", last_error)
    raise RuntimeError(f"could not reach hosted API after 3 attempts: {reason}")


def _atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def _write_env(path: Path, state: dict) -> None:
    values = {
        "PHONE_ARM_HOSTED_API_URL": os.environ.get(
            "PHONE_ARM_HOSTED_API_URL", DEFAULT_API_URL
        ),
        "PHONE_ARM_SESSION_RELAY_SESSION": state["session"],
        "PHONE_ARM_SESSION_RELAY_WT_URL": state["relay_url"],
        "PHONE_ARM_SESSION_RELAY_ARM_TOKEN": state["arm_relay_token"],
        "PHONE_ARM_MEDIAMTX_WHIP_URL": state.get("mediamtx_whip_url", ""),
        "PHONE_ARM_MEDIAMTX_PUBLISH_TOKEN": state.get("mediamtx_publish_token", ""),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        "".join(f"export {key}={shlex.quote(str(value))}\n" for key, value in values.items())
    )
    os.chmod(temporary, 0o600)
    temporary.replace(path)


def create(state_path: Path, env_path: Path, *, listed: bool = False) -> dict:
    follower_id = os.environ.get("PHONE_ARM_FOLLOWER_ID", socket.gethostname())
    name = os.environ.get("PHONE_ARM_ROBOT_NAME", follower_id)
    video_available = os.environ.get(
        "PHONE_ARM_VIDEO_AVAILABLE", "1"
    ).strip().lower() not in {"0", "false", "no", "off"}
    edge = choose_edge()
    result = _post(
        "/api/follower/create",
        {
            "follower_id": follower_id,
            "name": name,
            "listed": listed,
            "video_available": video_available,
            "edge": edge,
        },
    )
    result.update(
        {
            "follower_id": follower_id,
            "name": name,
            "listed": listed,
            "video_available": video_available,
            "edge": edge,
        }
    )
    _atomic_json(state_path, result)
    _write_env(env_path, result)
    return result


def register(state: dict) -> dict:
    return _post(
        "/api/follower/register",
        {
            "session": state["session"],
            "follower_id": state["follower_id"],
            "name": state["name"],
            "listed": state["listed"],
            "video_available": state.get("video_available", True),
        },
        str(state["registration_token"]),
    )


def heartbeat(state_path: Path, interval_s: float) -> None:
    state = json.loads(state_path.read_text())
    failures = 0
    while True:
        try:
            register(state)
            if failures:
                print("[access] hosted API connection restored", flush=True)
            failures = 0
        except Exception as exc:  # noqa: BLE001 - heartbeat must keep retrying
            failures += 1
            if failures == 1 or failures % 12 == 0:
                print(
                    f"[access] hosted API heartbeat failed: {exc}",
                    file=sys.stderr,
                    flush=True,
                )
        time.sleep(interval_s)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    create_parser = subparsers.add_parser("create")
    create_parser.add_argument("--state", type=Path, required=True)
    create_parser.add_argument("--env-file", type=Path, required=True)
    create_parser.add_argument("--listed", action="store_true")
    heartbeat_parser = subparsers.add_parser("heartbeat")
    heartbeat_parser.add_argument("--state", type=Path, required=True)
    heartbeat_parser.add_argument("--interval", type=float, default=5.0)
    args = parser.parse_args()

    if args.command == "create":
        result = create(args.state, args.env_file, listed=args.listed)
        print("[access] two-hour control link:")
        print(result["share_url"])
        print(f"[access] nearest relay edge: {result['edge']}")
        return
    heartbeat(args.state, max(1.0, args.interval))


if __name__ == "__main__":
    main()
