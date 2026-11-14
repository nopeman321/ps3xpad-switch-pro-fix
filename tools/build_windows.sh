#!/bin/sh
# Build both ps3xpad plugins with the official Sony Cell SDK on Windows.
#
# The stock Makefiles call `scetool` as a bare name and expect it on PATH, but
# scetool resolves its `data/` key directory relative to the *current working
# directory*. When make invokes it from src/ or game/, the keys are not found,
# the SELF is written without encryption, and the plugin will not load on a PS3.
# This script therefore runs the signing step itself, from the scetool
# directory, after `make` has produced the .prx files.
#
# Usage:
#   tools/build_windows.sh          # build + sign both plugins
#   tools/build_windows.sh clean    # remove all build products

set -e

# --- locate the workspace (script lives in tools/) -------------------------
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# --- toolchain locations ---------------------------------------------------
CELL_SDK=${CELL_SDK:-'C:\usr\local\cell'}
SCETOOL_DIR=${SCETOOL_DIR:-/c/Nueva_carpeta_2/tools/scetool}
MAKE_BIN=$(command -v make || true)

if [ -z "$MAKE_BIN" ]; then
    for c in "/c/Users/Juanjo/AppData/Local/Microsoft/WinGet/Packages/ezwinports.make_Microsoft.Winget.Source_8wekyb3d8bbwe/bin/make.exe" \
             "/c/Program Files/GnuWin32/bin/make.exe"; do
        [ -x "$c" ] && MAKE_BIN="$c" && break
    done
fi
if [ -z "$MAKE_BIN" ]; then
    echo "ERROR: GNU make not found. Install it (winget install ezwinports.make)." >&2
    exit 1
fi

export CELL_SDK
export PATH="/c/usr/local/cell/host-win32/ppu/bin:/c/usr/local/cell/host-win32/bin:$SCETOOL_DIR:$PATH"

echo "CELL_SDK = $CELL_SDK"
echo "make     = $MAKE_BIN"
echo

# --- clean -----------------------------------------------------------------
if [ "${1:-}" = "clean" ]; then
    for d in src game; do
        ( cd "$ROOT/$d" && rm -rf objs *.sym *.prx *.sprx )
    done
    echo "cleaned"
    exit 0
fi

# --- build (compile + link + strip) ----------------------------------------
# scetool will fail inside make; that failure is expected and handled below.
sign_and_report() {
    dir=$1; base=$2
    echo
    echo "--- signing $base.sprx ---"
    ( cd "$SCETOOL_DIR" && ./scetool.exe \
        -0 SELF -1 TRUE -s FALSE -2 0A -3 1070000052000001 \
        -4 01000002 -5 APP -6 0003004000000000 -A 0001000000000000 \
        -e "$dir/$base.prx" "$dir/$base.sprx" ) | sed 's/^/    /'

    if [ -f "$dir/$base.sprx" ]; then
        echo "    OK: $base.sprx ($(wc -c < "$dir/$base.sprx") bytes)"
    else
        echo "    FAILED to produce $base.sprx" >&2
        return 1
    fi
}

for d in src game; do
    echo "=== building $d ==="
    ( cd "$ROOT/$d" && "$MAKE_BIN" all 2>&1 | grep -vE "Could not load|keyset for SELF|Data not encrypted" ) || true
done

echo
echo "=== signing ==="
sign_and_report "$ROOT/src"  xpad
sign_and_report "$ROOT/game" xpad_game

echo
echo "=== done ==="
ls -la "$ROOT/src/xpad.sprx" "$ROOT/game/xpad_game.sprx"