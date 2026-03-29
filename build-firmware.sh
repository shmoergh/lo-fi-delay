#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_NAME="lo-fi-delay"

build_target() {
	local board="$1"
	local platform="$2"
	local build_dir="$3"
	local output_file="$4"

	echo "Configuring ${board} (${platform})..."
	cmake -S "$ROOT_DIR" -B "$ROOT_DIR/$build_dir" -DPICO_BOARD="$board" -DPICO_PLATFORM="$platform"

	echo "Building ${board} (${platform})..."
	cmake --build "$ROOT_DIR/$build_dir"

	local uf2_path="$ROOT_DIR/$build_dir/${TARGET_NAME}.uf2"
	if [[ ! -f "$uf2_path" ]]; then
		echo "Expected UF2 file not found: $uf2_path" >&2
		exit 1
	fi

	cp "$uf2_path" "$ROOT_DIR/$output_file"
	echo "Saved $output_file"
}

# RP2040 (pico) build intentionally disabled for current release.
# build_target "pico" "rp2040" "build-pico" "lo-fi-delay-pico.uf2"
build_target "pico2" "rp2350-arm-s" "build-pico-2" "lo-fi-delay-pico-2.uf2"

echo "Done."
