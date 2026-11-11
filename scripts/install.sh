#!/usr/bin/env bash
# Install the VDP33 module on a Move that runs Schwung.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

if [ ! -d "dist/vdp33" ]; then
    echo "Error: dist/vdp33 not found. Run ./scripts/build.sh first."
    exit 1
fi

DEST=/data/UserData/schwung/modules/sound_generators/vdp33

echo "=== Installing VDP33 ==="
ssh ableton@move.local "mkdir -p $DEST"
scp -r dist/vdp33/* ableton@move.local:$DEST/
ssh ableton@move.local "chmod -R a+rw $DEST"

echo ""
echo "Installed to $DEST"
echo "Restart Schwung to load the module."
