#!/bin/bash
# Crydteam Tools installer for Klipper
# by Steven (Fragmon) — Crydteam
# YouTube: https://www.youtube.com/@crydteamprinting
#
# Usage:
#   ./install.sh                 interactive plugin selection
#   ./install.sh all             install every plugin
#   ./install.sh speed_test …    install the named plugin(s)

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
KLIPPER_EXTRAS="${HOME}/klipper/klippy/extras"
CONFIG_DIR="${HOME}/printer_data/config"
MOONRAKER_CONF="${CONFIG_DIR}/moonraker.conf"
UPDATER_NAME="crydteamtools"
GIT_ORIGIN="https://github.com/Fragmon/crydteamtools.git"

# Re-set the executable bit in case a transfer stripped it.
chmod +x "$0" 2>/dev/null || true

# ─── Plugin registry ──────────────────────────────────────────────
# id | description | python files (relative) | macro file (optional)
PLUGIN_IDS=(speed_test max_flow_test)

plugin_desc() {
    case "$1" in
        speed_test)    echo "Speed Test — motor velocity/accel/current limit finder" ;;
        max_flow_test) echo "TMC Flow Test — extruder max flow rate via StallGuard" ;;
    esac
}
plugin_files() {
    case "$1" in
        speed_test)    echo "speed_test/speed_test.py" ;;
        max_flow_test) echo "max_flow_test/tmc_flow_test.py" ;;
    esac
}
plugin_macros() {
    case "$1" in
        speed_test)    echo "speed_test/speed_test_macros.cfg" ;;
        max_flow_test) echo "max_flow_test/tmc_flow_test_macros.cfg" ;;
    esac
}

echo ""
echo "=========================================="
echo "  Crydteam Tools — Installer"
echo "=========================================="
echo ""

if [ ! -d "${KLIPPER_EXTRAS}" ]; then
    echo "ERROR: Klipper extras directory not found at:"
    echo "  ${KLIPPER_EXTRAS}"
    echo "Make sure Klipper is installed at ~/klipper before running this."
    exit 1
fi

# ─── Select plugins ───────────────────────────────────────────────
SELECTED=()
if [ "$#" -gt 0 ]; then
    if [ "$1" = "all" ]; then
        SELECTED=("${PLUGIN_IDS[@]}")
    else
        for arg in "$@"; do
            ok=0
            for id in "${PLUGIN_IDS[@]}"; do
                [ "$arg" = "$id" ] && ok=1
            done
            if [ "$ok" = 1 ]; then SELECTED+=("$arg")
            else echo "Unknown plugin: $arg  (available: ${PLUGIN_IDS[*]})"; exit 1
            fi
        done
    fi
else
    echo "Available plugins:"
    i=1
    for id in "${PLUGIN_IDS[@]}"; do
        echo "  $i) $id — $(plugin_desc "$id")"
        i=$((i+1))
    done
    echo "  a) all"
    echo ""
    read -r -p "Install which plugins? (numbers separated by spaces, or 'a'): " answer
    if [ "$answer" = "a" ] || [ "$answer" = "A" ]; then
        SELECTED=("${PLUGIN_IDS[@]}")
    else
        for n in $answer; do
            idx=$((n-1))
            if [ "$idx" -ge 0 ] 2>/dev/null && [ "$idx" -lt "${#PLUGIN_IDS[@]}" ]; then
                SELECTED+=("${PLUGIN_IDS[$idx]}")
            else
                echo "Invalid selection: $n"; exit 1
            fi
        done
    fi
fi

if [ "${#SELECTED[@]}" -eq 0 ]; then
    echo "Nothing selected — aborting."; exit 1
fi

# ─── Install ──────────────────────────────────────────────────────
link() {   # link <src> <dst>
    if [ -L "$2" ] || [ -f "$2" ]; then rm -f "$2"; fi
    ln -s "$1" "$2"
}

echo ""
for id in "${SELECTED[@]}"; do
    echo "── $id ──"
    for rel in $(plugin_files "$id"); do
        src="${REPO_DIR}/${rel}"
        dst="${KLIPPER_EXTRAS}/$(basename "$rel")"
        if [ ! -f "$src" ]; then
            echo "  ✗ missing: $src"; continue
        fi
        link "$src" "$dst"
        echo "  ✓ $(basename "$rel") → ${dst}"
    done
    macro_rel="$(plugin_macros "$id")"
    if [ -n "$macro_rel" ] && [ -f "${REPO_DIR}/${macro_rel}" ] && [ -d "${CONFIG_DIR}" ]; then
        dst="${CONFIG_DIR}/$(basename "$macro_rel")"
        link "${REPO_DIR}/${macro_rel}" "$dst"
        echo "  ✓ $(basename "$macro_rel") → ${dst}"
    fi
done

# ─── Moonraker update manager ─────────────────────────────────────
# Registers the repo so updates show up in Mainsail/Fluidd's update
# manager (with the release version from the git tag).
if [ -f "${MOONRAKER_CONF}" ]; then
    if grep -q "^\[update_manager ${UPDATER_NAME}\]" "${MOONRAKER_CONF}"; then
        echo ""
        echo "  • update manager entry already present in moonraker.conf"
    else
        cat <<EOF >> "${MOONRAKER_CONF}"

## Crydteam Tools automatic update management
[update_manager ${UPDATER_NAME}]
type: git_repo
path: ${REPO_DIR}
origin: ${GIT_ORIGIN}
primary_branch: main
managed_services: klipper
EOF
        echo ""
        echo "  ✓ update manager entry added to moonraker.conf"
        echo "    → restart Moonraker once:  sudo systemctl restart moonraker"
    fi
else
    echo ""
    echo "  • moonraker.conf not found — skipping update-manager registration"
fi

echo ""
echo "------------------------------------------"
echo "  Installation complete."
echo "------------------------------------------"
echo ""
echo "Next steps:"
for id in "${SELECTED[@]}"; do
    case "$id" in
        speed_test)
            echo "  speed_test:    add a [speed_test] section to printer.cfg"
            echo "                 optional macros: [include speed_test_macros.cfg]" ;;
        max_flow_test)
            echo "  max_flow_test: add a [tmc_flow_test] section to printer.cfg"
            echo "                 optional macros: [include tmc_flow_test_macros.cfg]" ;;
    esac
done
echo "  then: FIRMWARE_RESTART"
echo ""
echo "Docs: see the README.md inside each plugin folder."
echo ""
