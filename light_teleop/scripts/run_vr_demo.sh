#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

HOST="${1:-10.1.42.44}"
if [[ $# -gt 0 ]]; then
  shift
fi
PORT="${1:-8080}"
if [[ $# -gt 0 ]]; then
  shift
fi
PYTHON="${PYTHON:-${REPO_ROOT}/.venv-light-tp/bin/python}"

cd "${REPO_ROOT}/light_teleop"

export PYTHONDONTWRITEBYTECODE="${PYTHONDONTWRITEBYTECODE:-1}"

exec "${PYTHON}" -m light_teleop.vr_visual_demo \
  --host "${HOST}" \
  --port "${PORT}" \
  "$@"
