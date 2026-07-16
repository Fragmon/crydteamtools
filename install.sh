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

# UI macros for Mainsail/Fluidd (optional include)
CONFIG_DIR="${HOME}/printer_data/config"
if [ -d "${CONFIG_DIR}" ]; then
    macro_src="${REPO_DIR}/speed_test_macros.cfg"
    macro_dst="${CONFIG_DIR}/speed_test_macros.cfg"
    if [ -f "${macro_src}" ]; then
        if [ -L "${macro_dst}" ] || [ -f "${macro_dst}" ]; then
            rm -f "${macro_dst}"
        fi
        ln -s "${macro_src}" "${macro_dst}"
        echo "  ✓ speed_test_macros.cfg → ${macro_dst}"
    fi
fi

echo ""
echo "------------------------------------------"
echo "  Installation complete."
echo "------------------------------------------"
echo ""
echo "Next steps:"
echo "  1. Add a [speed_test] section to your printer.cfg"
echo "     (do NOT add [endstop_phase] - remove it if present!)"
echo "  2. Optional UI macros: add  [include speed_test_macros.cfg]"
echo "  3. Run FIRMWARE_RESTART"
echo "  4. Run SPEED_TEST_STATUS (or the ST_STATUS macro) to verify"
echo ""
echo "See README.md and docs/ for full configuration."
echo ""
