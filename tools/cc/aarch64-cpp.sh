#!/usr/bin/env bash
set -euo pipefail
exec /usr/bin/aarch64-linux-gnu-cpp --sysroot=/usr/aarch64-linux-gnu "$@"
