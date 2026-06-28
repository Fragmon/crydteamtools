#!/bin/bash
# Crydteam Tools installer for Klipper
# Plugin by Steven (Fragmon) — Crydteam
# YouTube: https://www.youtube.com/@crydteamprinting

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
KLIPPER_EXTRAS="${HOME}/klipper/klippy/extras"

# Plugin files to symlink
PLUGINS=(
    "speed_test.py"
)

echo ""
echo "=========================================="
echo "  Crydteam Tools — Installer"
echo "=========================================="
echo ""

# Check klipper exists
if [ ! -d "${KLIPPER_EXTRAS}" ]; then
    echo "ERROR: Klipper extras directory not found at:"
    echo "  ${KLIPPER_EXTRAS}"
    echo ""
    echo "Make sure Klipper is installed at ~/klipper before running this."
    exit 1
fi

# Symlink each plugin
for plugin in "${PLUGINS[@]}"; do
    src="${REPO_DIR}/${plugin}"
    dst="${KLIPPER_EXTRAS}/${plugin}"

    if [ ! -f "${src}" ]; then
        echo "  ✗ ${plugin}: source missing at ${src}"
        continue
    fi

    if [ -L "${dst}" ] || [ -f "${dst}" ]; then
        echo "  • ${plugin}: replacing existing entry"
        rm -f "${dst}"
    fi

    ln -s "${src}" "${dst}"
    echo "  ✓ ${plugin} → ${dst}"
done

echo ""
echo "------------------------------------------"
echo "  Installation complete."
echo "------------------------------------------"
echo ""
echo "Next steps:"
echo "  1. Add [endstop_phase] and [speed_test] sections to your printer.cfg"
echo "  2. Run FIRMWARE_RESTART"
echo "  3. Run SPEED_TEST_STATUS to verify"
echo ""
echo "See README.md for full configuration."
echo ""
