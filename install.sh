#!/bin/bash
# Crydteam Tools installer for Klipper
# by Steven (Fragmon) — Crydteam
# YouTube: https://www.youtube.com/@crydteamprinting
#
# Usage:
#   ./install.sh                 interactive plugin selection
#   ./install.sh all             install every plugin
#   ./install.sh speed_test …    install the named plugin(s)
#   ./install.sh uninstall [all|<plugin> …]
#                                remove plugins: deletes the symlinks,
#                                removes the [include] lines and (when
#                                nothing is left) the Moonraker entry;
#                                settings files are archived to
#                                ~/printer_data/config/archive/

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
KLIPPER_EXTRAS="${HOME}/klipper/klippy/extras"
CONFIG_DIR="${HOME}/printer_data/config"
ARCHIVE_DIR="${CONFIG_DIR}/archive"
PRINTER_CFG="${CONFIG_DIR}/printer.cfg"
MOONRAKER_CONF="${CONFIG_DIR}/moonraker.conf"
UPDATER_NAME="crydteamtools"
GIT_ORIGIN="https://github.com/Fragmon/crydteamtools.git"

# Re-set the executable bit in case a transfer stripped it.
chmod +x "$0" 2>/dev/null || true

# ─── Plugin registry ──────────────────────────────────────────────
# id | description | python files (relative) | macro file (optional)
PLUGIN_IDS=(speed_test max_flow_test motor_sync pa_test)

plugin_desc() {
    case "$1" in
        speed_test)    echo "Speed Test — motor velocity/accel/current limit finder" ;;
        max_flow_test) echo "TMC Flow Test — extruder max flow rate via StallGuard" ;;
        motor_sync)    echo "Motor Sync — dual-motor axis sync via StallGuard (no ADXL)" ;;
        pa_test)       echo "PA Test — pressure-advance calibration via StallGuard (prototype)" ;;
    esac
}
plugin_files() {
    case "$1" in
        speed_test)    echo "speed_test/speed_test.py" ;;
        max_flow_test) echo "max_flow_test/tmc_flow_test.py" ;;
        motor_sync)    echo "motor_sync/motor_sync.py" ;;
        pa_test)       echo "pa_test/pa_test.py" ;;
    esac
}
plugin_macros() {
    case "$1" in
        speed_test)    echo "speed_test/speed_test_macros.cfg" ;;
        max_flow_test) echo "max_flow_test/tmc_flow_test_macros.cfg" ;;
        motor_sync)    echo "motor_sync/motor_sync_macros.cfg" ;;
        pa_test)       echo "pa_test/pa_test_macros.cfg" ;;
    esac
}
# Commented config-section template, COPIED (not linked) into the
# config dir so the user's edits survive plugin updates.
plugin_settings() {
    case "$1" in
        speed_test)    echo "speed_test/speed_test_settings.cfg" ;;
        max_flow_test) echo "max_flow_test/tmc_flow_test_settings.cfg" ;;
        motor_sync)    echo "motor_sync/motor_sync_settings.cfg" ;;
        pa_test)       echo "pa_test/pa_test_settings.cfg" ;;
    esac
}

clear 2>/dev/null || true
cat <<'BANNER'

   ██████╗██████╗ ██╗   ██╗██████╗ ████████╗███████╗ █████╗ ███╗   ███╗
  ██╔════╝██╔══██╗╚██╗ ██╔╝██╔══██╗╚══██╔══╝██╔════╝██╔══██╗████╗ ████║
  ██║     ██████╔╝ ╚████╔╝ ██║  ██║   ██║   █████╗  ███████║██╔████╔██║
  ██║     ██╔══██╗  ╚██╔╝  ██║  ██║   ██║   ██╔══╝  ██╔══██║██║╚██╔╝██║
  ╚██████╗██║  ██║   ██║   ██████╔╝   ██║   ███████╗██║  ██║██║ ╚═╝ ██║
   ╚═════╝╚═╝  ╚═╝   ╚═╝   ╚═════╝    ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝     ╚═╝
             T O O L S   —   Klipper tuning plugin installer
                youtube.com/@crydteamprinting  ·  by Fragmon

BANNER

if [ ! -d "${KLIPPER_EXTRAS}" ]; then
    echo "ERROR: Klipper extras directory not found at:"
    echo "  ${KLIPPER_EXTRAS}"
    echo "Make sure Klipper is installed at ~/klipper before running this."
    exit 1
fi

# ─── Mode: install (default) or uninstall ─────────────────────────
MODE="install"
if [ "${1:-}" = "uninstall" ] || [ "${1:-}" = "remove" ]; then
    MODE="uninstall"
    shift
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
        state=""
        if [ "$MODE" = "install" ] \
           && [ -e "${KLIPPER_EXTRAS}/$(basename "$(plugin_files "$id")")" ]; then
            state="  [installed]"
        fi
        echo "  $i) $id — $(plugin_desc "$id")${state}"
        i=$((i+1))
    done
    echo "  a) all"
    echo ""
    if [ "$MODE" = "uninstall" ]; then
        read -r -p "UNINSTALL which plugins? (numbers separated by spaces, or 'a'): " answer
    else
        read -r -p "Install which plugins? (numbers separated by spaces, or 'a'): " answer
    fi
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

# True if "[include <file>]" already exists ANYWHERE in the config
# dir — printer.cfg or any sub-config (users often keep their includes
# in a separate file). Tolerates extra whitespace, ignores commented
# lines. A duplicate include would break Klipper at restart, so this
# check errs on the side of finding existing entries.
include_present() {   # include_present <cfg-file-name>
    esc=$(printf '%s' "$1" | sed 's/[][\.*^$]/\\&/g')
    grep -rEqs --include='*.cfg' \
        "^[[:space:]]*\[include[[:space:]]+${esc}[[:space:]]*\]" \
        "${CONFIG_DIR}" 2>/dev/null
}

# Insert "[include <file>]" at the VERY TOP of printer.cfg — only if
# it is not already included somewhere.
add_include_top() {   # add_include_top <cfg-file-name>
    if [ ! -f "${PRINTER_CFG}" ]; then
        echo "  • printer.cfg not found — add [include $1] manually"
        return
    fi
    if include_present "$1"; then
        echo "  • [include $1] already present — skipped"
    else
        tmp="$(mktemp)"
        { echo "[include $1]"; cat "${PRINTER_CFG}"; } > "$tmp"
        mv "$tmp" "${PRINTER_CFG}"
        echo "  ✓ [include $1] added to top of printer.cfg"
    fi
}

# ─── Uninstall helpers ────────────────────────────────────────────
# Remove every "[include <file>]" line from all .cfg files in the
# config dir (tolerates whitespace, keeps commented lines untouched).
remove_include() {   # remove_include <cfg-file-name>
    local name="$1" found=0
    local esc
    esc=$(printf '%s' "$name" | sed 's/[][\.*^$/]/\\&/g')
    while IFS= read -r cfg; do
        if grep -Eq "^[[:space:]]*\[include[[:space:]]+${esc}[[:space:]]*\]" "$cfg"; then
            sed -i -E "/^[[:space:]]*\[include[[:space:]]+${esc}[[:space:]]*\]/d" "$cfg"
            echo "  ✓ [include ${name}] removed from $(basename "$cfg")"
            found=1
        fi
    done < <(find "${CONFIG_DIR}" -maxdepth 2 -name '*.cfg' -type f 2>/dev/null)
    [ "$found" = 0 ] && echo "  • no [include ${name}] found"
    return 0
}

# Move a config file into the archive folder instead of deleting it.
archive_file() {   # archive_file <path>
    local src="$1"
    [ -e "$src" ] || return 0
    mkdir -p "${ARCHIVE_DIR}"
    local base dst
    base="$(basename "$src")"
    dst="${ARCHIVE_DIR}/${base}"
    if [ -e "$dst" ]; then
        dst="${ARCHIVE_DIR}/${base%.cfg}_$(date +%Y%m%d-%H%M%S).cfg"
    fi
    mv "$src" "$dst"
    echo "  ✓ ${base} archived → ${dst}"
}

if [ "$MODE" = "uninstall" ]; then
    echo ""
    for id in "${SELECTED[@]}"; do
        echo "── removing $id ──"
        for rel in $(plugin_files "$id"); do
            dst="${KLIPPER_EXTRAS}/$(basename "$rel")"
            if [ -L "$dst" ] || [ -f "$dst" ]; then
                rm -f "$dst"
                echo "  ✓ $(basename "$rel") unlinked from extras"
            else
                echo "  • $(basename "$rel") was not installed"
            fi
        done
        for rel in "$(plugin_macros "$id")" "$(plugin_settings "$id")"; do
            [ -n "$rel" ] || continue
            name="$(basename "$rel")"
            remove_include "$name"
            target="${CONFIG_DIR}/${name}"
            repo_src="${REPO_DIR}/${rel}"
            if [ -L "$target" ]; then
                rm -f "$target"       # macros are symlinks — just drop
                echo "  ✓ ${name} link removed"
            elif [ -f "$target" ] && cmp -s "$target" "$repo_src"; then
                rm -f "$target"       # unmodified copy — nothing to keep
                echo "  ✓ ${name} removed (unchanged copy)"
            else
                archive_file "$target"  # user-edited file — keep it
            fi
        done
    done

    # Drop the Moonraker entry only when no plugin is left installed.
    still_installed=0
    for id in "${PLUGIN_IDS[@]}"; do
        for rel in $(plugin_files "$id"); do
            [ -e "${KLIPPER_EXTRAS}/$(basename "$rel")" ] && still_installed=1
        done
    done
    echo ""
    if [ "$still_installed" = 1 ]; then
        echo "  • other plugins still installed — Moonraker update entry kept"
    else
        removed=0
        while IFS= read -r conf; do
            if grep -Eq "^[[:space:]]*\[update_manager[[:space:]]+${UPDATER_NAME}[[:space:]]*\]" "$conf"; then
                # Delete the section header, its comment line and all
                # following key: value lines up to the next section.
                sed -i -E "/^##[[:space:]]*Crydteam Tools automatic update management/d" "$conf"
                sed -i -E "/^[[:space:]]*\[update_manager[[:space:]]+${UPDATER_NAME}[[:space:]]*\]/,/^[[:space:]]*(\[|$)/{/^[[:space:]]*\[update_manager[[:space:]]+${UPDATER_NAME}[[:space:]]*\]/d; /^[[:space:]]*[a-z_]+:.*/d}" "$conf"
                echo "  ✓ [update_manager ${UPDATER_NAME}] removed from $(basename "$conf")"
                removed=1
            fi
        done < <(find "${CONFIG_DIR}" -maxdepth 2 -name '*.conf' -type f 2>/dev/null)
        [ "$removed" = 0 ] && echo "  • no update-manager entry found"
    fi

    echo ""
    echo "------------------------------------------"
    echo "  Uninstall complete."
    echo "------------------------------------------"
    echo ""
    echo "Settings files were archived to: ${ARCHIVE_DIR}"
    echo "Remember to remove the plugin's config section (e.g."
    echo "[speed_test]) from printer.cfg if you added it there."
    echo "Then: FIRMWARE_RESTART   (and restart Moonraker if its"
    echo "entry was removed:  sudo systemctl restart moonraker)"
    echo ""
    echo "The repo folder itself is still at: ${REPO_DIR}"
    echo ""
    exit 0
fi

# ─── Already installed? Ask before touching it ────────────────────
ALREADY=()
for id in "${SELECTED[@]}"; do
    for rel in $(plugin_files "$id"); do
        if [ -L "${KLIPPER_EXTRAS}/$(basename "$rel")" ] \
           || [ -f "${KLIPPER_EXTRAS}/$(basename "$rel")" ]; then
            ALREADY+=("$id")
            break
        fi
    done
done

if [ "${#ALREADY[@]}" -gt 0 ]; then
    echo ""
    echo "Already installed: ${ALREADY[*]}"
    echo "  An update re-links the plugin files (picking up the code"
    echo "  you just pulled) and adds any missing [include] lines."
    echo "  Your existing *_settings.cfg files are NEVER overwritten."
    echo ""
    read -r -p "Update them? [Y/n/s=skip these, install only new]: " upd
    case "$upd" in
        [Nn]*)
            echo "Aborted — nothing changed."; exit 0 ;;
        [Ss]*)
            REMAINING=()
            for id in "${SELECTED[@]}"; do
                skip=0
                for a in "${ALREADY[@]}"; do [ "$id" = "$a" ] && skip=1; done
                [ "$skip" = 0 ] && REMAINING+=("$id")
            done
            SELECTED=("${REMAINING[@]}")
            if [ "${#SELECTED[@]}" -eq 0 ]; then
                echo "Nothing left to install — done."; exit 0
            fi
            echo "Installing only: ${SELECTED[*]}" ;;
        *)
            echo "Updating: ${ALREADY[*]}" ;;
    esac
fi

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
        add_include_top "$(basename "$macro_rel")"
    fi
    # Config-section template: copy once (user edits must survive
    # updates), then include it at the top of printer.cfg. The
    # section inside ships fully commented out, so including it
    # changes nothing until the user uncomments [<plugin section>].
    settings_rel="$(plugin_settings "$id")"
    if [ -n "$settings_rel" ] && [ -f "${REPO_DIR}/${settings_rel}" ] && [ -d "${CONFIG_DIR}" ]; then
        dst="${CONFIG_DIR}/$(basename "$settings_rel")"
        if [ -f "$dst" ]; then
            echo "  • $(basename "$settings_rel") already exists — left untouched"
        else
            cp "${REPO_DIR}/${settings_rel}" "$dst"
            echo "  ✓ $(basename "$settings_rel") copied to ${dst}"
        fi
        add_include_top "$(basename "$settings_rel")"
    fi
done

# ─── Moonraker update manager ─────────────────────────────────────
# Registers the repo so updates show up in Mainsail/Fluidd's update
# manager (with the release version from the git tag).
if [ -f "${MOONRAKER_CONF}" ]; then
    # Check moonraker.conf AND any included .conf for an existing
    # entry — only add it if it is missing everywhere.
    if grep -rEqs --include='*.conf' \
        "^[[:space:]]*\[update_manager[[:space:]]+${UPDATER_NAME}[[:space:]]*\]" \
        "${CONFIG_DIR}" 2>/dev/null; then
        echo ""
        echo "  • update manager entry already present — skipped"
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
            echo "  speed_test:    uncomment [speed_test] in speed_test_settings.cfg" ;;
        max_flow_test)
            echo "  max_flow_test: uncomment [tmc_flow_test] in tmc_flow_test_settings.cfg" ;;
        motor_sync)
            echo "  motor_sync:    uncomment [motor_sync] in motor_sync_settings.cfg" ;;
        pa_test)
            echo "  pa_test:       uncomment [pa_test] in pa_test_settings.cfg" ;;
    esac
done
echo "  (includes, settings templates and Moonraker update entry were added automatically)"
echo "  then: FIRMWARE_RESTART"
echo ""
echo "Docs: see the README.md inside each plugin folder."
echo ""
