#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../.."
PYTHON_BIN="${PHONE_ARM_PYTHON:-$HOME/.venv/lerobot/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  echo "Leader Python environment not found: $PYTHON_BIN" >&2
  echo "Set PHONE_ARM_PYTHON to the Python executable containing lerobot." >&2
  exit 1
fi
exec "$PYTHON_BIN" -u -m controllers.leader_arm.controller "$@"
