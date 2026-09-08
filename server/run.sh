#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

CERT="${PHONE_ARM_WT_CERT:-server/certs/server.crt}"
KEY="${PHONE_ARM_WT_KEY:-server/certs/server.key}"
PHONE_SECRET="${PHONE_ARM_WT_PHONE_SECRET:-@server/secrets/phone.token}"
ARM_SECRET="${PHONE_ARM_WT_ARM_SECRET:-@server/secrets/arm.token}"

exec python3 -m server.relay \
  --host "${PHONE_ARM_WT_HOST:-0.0.0.0}" \
  --port "${PHONE_ARM_WT_PORT:-4433}" \
  --certificate "$CERT" \
  --private-key "$KEY" \
  --event-log "${PHONE_ARM_WT_EVENT_LOG:-/tmp/phone_arm_webtransport_relay.jsonl}" \
  --phone-secret "$PHONE_SECRET" \
  --arm-secret "$ARM_SECRET"
