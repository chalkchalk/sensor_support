#!/usr/bin/env bash
set -euo pipefail

# Portable one-command installer for Livox-SDK2 on Debian/Ubuntu.
# Optional overrides:
#   LIVOX_INSTALL_PREFIX=/usr/local  Installation prefix
#   LIVOX_BUILD_JOBS=8               Parallel build jobs
#   LIVOX_SKIP_APT=1                 Skip apt dependency installation

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_PREFIX="${LIVOX_INSTALL_PREFIX:-/usr/local}"
BUILD_JOBS="${LIVOX_BUILD_JOBS:-$(nproc)}"
BUILD_DIR="${SCRIPT_DIR}/build"

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "Error: root privileges are required for: $*" >&2
    exit 1
  fi
}

if [[ "${LIVOX_SKIP_APT:-0}" != "1" ]]; then
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "Error: apt-get is unavailable; set LIVOX_SKIP_APT=1 after installing dependencies." >&2
    exit 1
  fi
  run_as_root apt-get update
  run_as_root apt-get install -y build-essential cmake
fi

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS}"
run_as_root cmake --install "${BUILD_DIR}"

if command -v ldconfig >/dev/null 2>&1; then
  run_as_root ldconfig
fi

echo "Livox-SDK2 installed successfully in ${INSTALL_PREFIX}"
