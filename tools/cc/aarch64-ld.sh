#!/usr/bin/env bash
set -euo pipefail
exec /usr/bin/aarch64-linux-gnu-ld --sysroot=/usr/aarch64-linux-gnu "$@"
