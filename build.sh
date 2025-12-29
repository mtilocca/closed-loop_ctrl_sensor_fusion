#!/usr/bin/env bash
set -euo pipefail

APP_NAME="closed_loop_sender"
OUT_DIR="bin"
PKG="./closed_loop"

mkdir -p "${OUT_DIR}"

echo "Building ${APP_NAME}..."

build() {
  local os="$1"
  local arch="$2"
  local ext="$3"

  local out="${OUT_DIR}/${APP_NAME}_${os}_${arch}${ext}"
  echo "  → ${os}/${arch}"
  GOOS="${os}" GOARCH="${arch}" go build -o "${out}" "${PKG}"
}

# Linux
build linux amd64 ""


# Linux
build linux arm64 ""


# macOS (Apple Silicon)
build darwin arm64 ""

# Windows
build windows amd64 ".exe"


echo "Build complete."
