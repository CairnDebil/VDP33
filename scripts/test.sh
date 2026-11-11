#!/usr/bin/env bash
# Native offline test: renders demo WAVs and checks pitch, levels, vocoder
# tracking, zones, hold, ensemble, state round-trip and CPU load.
set -e
cd "$(dirname "$0")/../tests"
mkdir -p demo
BIN="$(mktemp -d)/render_test"          # build outside the repo (works on noexec mounts too)
gcc -O2 -std=gnu11 -Wall -Wextra -I../src/dsp render_test.c -o "$BIN" -lm
"$BIN" demo
