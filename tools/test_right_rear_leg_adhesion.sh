#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRELOAD_OFFSET_M="${PRELOAD_OFFSET_M:-0.005}" exec "$SCRIPT_DIR/test_single_leg_adhesion.sh" --leg rr "$@"