#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

HOST="${1:-10.1.42.44}"
PORT="${2:-8080}"
PYTHON="${PYTHON:-${REPO_ROOT}/.venv-light-tp/bin/python}"

cd "${REPO_ROOT}"

exec "${PYTHON}" test/recv_vr_udp.py \
  --host "${HOST}" \
  --port "${PORT}"
