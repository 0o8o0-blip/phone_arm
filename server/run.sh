#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

CERT="${PHONE_ARM_WT_CERT:-server/certs/server.crt}"
KEY="${PHONE_ARM_WT_KEY:-server/certs/server.key}"
CAPABILITY_SECRET="${PHONE_ARM_CAPABILITY_SECRET:-@server/secrets/capability.token}"

ARGS=(
  --host "${PHONE_ARM_WT_HOST:-0.0.0.0}"
  --port "${PHONE_ARM_WT_PORT:-4433}"
  --certificate "$CERT"
  --private-key "$KEY"
  --event-log "${PHONE_ARM_WT_EVENT_LOG:-/tmp/phone_arm_webtransport_relay.jsonl}"
  --capability-secret "$CAPABILITY_SECRET"
  --edge-name "${PHONE_ARM_EDGE_NAME:-europe}"
  --backbone-host "${PHONE_ARM_BACKBONE_HOST:-127.0.0.1}"
  --backbone-port "${PHONE_ARM_BACKBONE_PORT:-7443}"
  --backbone-max-age-ms "${PHONE_ARM_BACKBONE_MAX_AGE_MS:-150}"
)
if [ -n "${PHONE_ARM_PEER_BACKBONE:-}" ]; then
  ARGS+=(--peer-backbone "$PHONE_ARM_PEER_BACKBONE")
fi
exec python3 -m server.relay "${ARGS[@]}"
