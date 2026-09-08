"""Short-lived, session-scoped capability tokens for hosted services."""
from __future__ import annotations

import base64
import hashlib
import hmac
import time


def _signature(secret: str, payload: str) -> str:
    digest = hmac.new(secret.encode(), payload.encode(), hashlib.sha256).digest()
    return base64.urlsafe_b64encode(digest).decode().rstrip("=")


def issue(secret: str, *, role: str, session: str, expires_at: float) -> str:
    payload = f"v1.{int(expires_at)}.{role}.{session}"
    return f"{payload}.{_signature(secret, payload)}"


def verify(
    secret: str,
    token: str,
    *,
    role: str,
    session: str,
    now: float | None = None,
) -> bool:
    try:
        version, raw_expiry, actual_role, actual_session, signature = token.split(".")
        expiry = int(raw_expiry)
    except (TypeError, ValueError):
        return False
    if version != "v1" or actual_role != role or actual_session != session:
        return False
    if expiry <= int(time.time() if now is None else now):
        return False
    payload = f"{version}.{raw_expiry}.{actual_role}.{actual_session}"
    return hmac.compare_digest(signature, _signature(secret, payload))
