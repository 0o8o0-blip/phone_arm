#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../.."
exec /home/user/.venv/lerobot/bin/python -u -m controllers.leader_arm.controller "$@"
