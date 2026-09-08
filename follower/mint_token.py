#!/usr/bin/env python3
"""Mint, list, and revoke phone-arm session tokens.

Tokens live in ~/.phone_arm_tokens.json as a JSON array of
  {"value": str, "name": str, "expires_at": float | null}
where expires_at is a unix timestamp (null = never expires).

Examples:
    ./follower/mint_token.py mint --name global
    ./follower/mint_token.py mint --name alice --expires 1h
    ./follower/mint_token.py show global
    ./follower/mint_token.py revoke alice

After any mutation the running teleop process is sent SIGHUP so the new tokens
take effect immediately -- no teleop restart needed.

The public URL is hardcoded to match VPS_PUBLIC_URL in follower/gateway.py; change
both together if the relay hostname ever moves.
"""
import argparse
import json
import os
import re
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

TOKENS_FILE = Path.home() / ".phone_arm_tokens.json"
PUBLIC_URL = "https://188-166-154-201.sslip.io"


def parse_duration(s: str) -> float:
    """Parse '1h', '30m', '7d', '120s' into seconds (float)."""
    m = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*([smhd])\s*", s)
    if not m:
        raise argparse.ArgumentTypeError(
            f"bad duration {s!r} -- use formats like 30s, 5m, 2h, 7d"
        )
    val = float(m.group(1))
    unit = m.group(2)
    return val * {"s": 1, "m": 60, "h": 3600, "d": 86400}[unit]


def load_tokens() -> list[dict]:
    if not TOKENS_FILE.exists():
        return []
    with open(TOKENS_FILE) as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise SystemExit(f"{TOKENS_FILE} is not a JSON array")
    return data


def save_tokens(tokens: list[dict]) -> None:
    TOKENS_FILE.parent.mkdir(parents=True, exist_ok=True)
    tmp = TOKENS_FILE.with_suffix(".json.tmp")
    with open(tmp, "w") as f:
        json.dump(tokens, f, indent=2)
    os.replace(tmp, TOKENS_FILE)
    # File contains share-able URLs; only the owner should read it.
    os.chmod(TOKENS_FILE, 0o600)


def notify_teleop() -> None:
    """SIGHUP every running follower.main so it reloads tokens in-process."""
    try:
        out = subprocess.run(
            ["pgrep", "-f", "follower.main"],
            capture_output=True, text=True, check=False,
        ).stdout
    except FileNotFoundError:
        print("(pgrep not found; teleop will pick up changes on next start)")
        return
    pids = [int(p) for p in out.split() if p.strip().isdigit()]
    # pgrep -f also matches this script if its argv contains 'follower.main'.
    # Filter our own pid out for safety.
    pids = [p for p in pids if p != os.getpid()]
    if not pids:
        print("(no running teleop; changes apply on next start)")
        return
    for pid in pids:
        try:
            os.kill(pid, signal.SIGHUP)
            print(f"sent SIGHUP to pid {pid}")
        except ProcessLookupError:
            pass
        except PermissionError as e:
            print(f"  could not SIGHUP {pid}: {e}")


def fmt_expiry(epoch: float | None) -> str:
    if epoch is None:
        return "never"
    remaining = epoch - time.time()
    if remaining <= 0:
        return f"EXPIRED {int(-remaining)}s ago"
    if remaining < 60:
        return f"in {int(remaining)}s"
    if remaining < 3600:
        return f"in {int(remaining / 60)}m"
    if remaining < 86400:
        return f"in {remaining / 3600:.1f}h"
    return f"in {remaining / 86400:.1f}d"


def cmd_mint(args: argparse.Namespace) -> None:
    tokens = load_tokens()
    if any(t.get("name") == args.name for t in tokens):
        raise SystemExit(
            f"name {args.name!r} is already in use; revoke first or pick another"
        )
    value = secrets.token_urlsafe(32)
    expires_at = None
    if args.expires is not None:
        expires_at = time.time() + parse_duration(args.expires)
    tokens.append({"value": value, "name": args.name, "expires_at": expires_at})
    save_tokens(tokens)
    notify_teleop()
    print(f"\nminted token {args.name!r}  (expires {fmt_expiry(expires_at)})")
    print(f"share URL:\n    {PUBLIC_URL}/?t={value}")


def cmd_list(_args: argparse.Namespace) -> None:
    tokens = load_tokens()
    if not tokens:
        print(f"no tokens in {TOKENS_FILE}")
        return
    print(f"{'name':<24} {'expires':<28} value")
    print("-" * 80)
    for t in tokens:
        name = t.get("name") or "(unnamed)"
        exp = fmt_expiry(t.get("expires_at"))
        v = t.get("value", "")
        v_short = v[:12] + "..." if len(v) > 16 else v
        print(f"{name:<24} {exp:<28} {v_short}")


def cmd_show(args: argparse.Namespace) -> None:
    """Print an existing token's share URL without changing token state."""
    for token in load_tokens():
        if token.get("name") != args.name:
            continue
        expires_at = token.get("expires_at")
        if expires_at is not None and expires_at <= time.time():
            raise SystemExit(
                f"token {args.name!r} is expired; extend it or mint a new token"
            )
        value = token.get("value")
        if not value:
            raise SystemExit(f"token {args.name!r} has no value")
        print(f"share URL:\n    {PUBLIC_URL}/?t={value}")
        return
    raise SystemExit(f"no token named {args.name!r}")


def cmd_extend(args: argparse.Namespace) -> None:
    tokens = load_tokens()
    expires_at = None if args.expires is None else time.time() + parse_duration(args.expires)
    for t in tokens:
        if t.get("name") == args.name:
            t["expires_at"] = expires_at
            save_tokens(tokens)
            notify_teleop()
            print(f"extended {args.name!r}  (expires {fmt_expiry(expires_at)})")
            print(f"share URL:\n    {PUBLIC_URL}/?t={t.get('value')}")
            return
    raise SystemExit(f"no token named {args.name!r}")


def cmd_revoke(args: argparse.Namespace) -> None:
    tokens = load_tokens()
    before = len(tokens)
    tokens = [t for t in tokens if t.get("name") != args.name]
    if len(tokens) == before:
        raise SystemExit(f"no token named {args.name!r}")
    save_tokens(tokens)
    notify_teleop()
    print(f"revoked {args.name!r}")


def cmd_revoke_all(_args: argparse.Namespace) -> None:
    save_tokens([])
    notify_teleop()
    print("revoked all tokens; auth still ENABLED (file exists, empty list)")
    print(f"to disable auth entirely, delete {TOKENS_FILE}")


def cmd_purge_expired(_args: argparse.Namespace) -> None:
    tokens = load_tokens()
    now = time.time()
    keep, drop = [], []
    for t in tokens:
        exp = t.get("expires_at")
        if exp is not None and exp <= now:
            drop.append(t)
        else:
            keep.append(t)
    if not drop:
        print("nothing to purge")
        return
    save_tokens(keep)
    notify_teleop()
    print(f"purged {len(drop)} expired token(s): "
          + ", ".join(t.get("name", "?") for t in drop))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=False)

    mint = sub.add_parser("mint", help="create a new token (default if no subcommand)")
    mint.add_argument("--name", required=True, help="label for the token (must be unique)")
    mint.add_argument("--expires", default=None,
                      help="duration like 30s, 5m, 2h, 7d (omit for never-expiring)")
    mint.set_defaults(func=cmd_mint)

    lst = sub.add_parser("list", help="show all tokens with their expiry status")
    lst.set_defaults(func=cmd_list)

    show = sub.add_parser("show", help="print an existing token's share URL")
    show.add_argument("name", help="name of the token to show")
    show.set_defaults(func=cmd_show)

    ext = sub.add_parser("extend", help="change a token expiry without changing its value")
    ext.add_argument("name", help="name of the token to extend")
    ext.add_argument("--expires", default=None,
                     help="duration like 30s, 5m, 2h, 7d (omit for never-expiring)")
    ext.set_defaults(func=cmd_extend)

    rev = sub.add_parser("revoke", help="revoke a token by name")
    rev.add_argument("name", help="name of the token to revoke")
    rev.set_defaults(func=cmd_revoke)

    rev_all = sub.add_parser("revoke-all", help="revoke every token (auth stays enabled)")
    rev_all.set_defaults(func=cmd_revoke_all)

    purge = sub.add_parser("purge-expired",
                           help="remove tokens whose expires_at is in the past")
    purge.set_defaults(func=cmd_purge_expired)

    # Shortcut flags so `./follower/mint_token.py --list` works too.
    p.add_argument("--list", action="store_const", const=cmd_list, dest="shortcut")
    p.add_argument("--revoke-all", action="store_const",
                   const=cmd_revoke_all, dest="shortcut")
    return p


def main() -> None:
    p = build_parser()
    args, extras = p.parse_known_args()
    if args.shortcut is not None:
        args.shortcut(args)
        return
    if args.cmd is None:
        p.print_help(sys.stderr)
        raise SystemExit(2)
    args.func(args)


if __name__ == "__main__":
    main()
