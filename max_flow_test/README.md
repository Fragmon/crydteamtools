# TMC Flow Test

*Part of [Crydteam Tools](../README.md) — Klipper plugins by Steven (Fragmon).*

**Find your printer's real max flow rate in 30 minutes — without test prints.**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Klipper](https://img.shields.io/badge/Klipper-compatible-green.svg)](https://www.klipper3d.org/)
[![Kalico](https://img.shields.io/badge/Kalico-compatible-brightgreen.svg)](https://github.com/KalicoCrew/kalico)

Plugin by **Steven (Fragmon) — Crydteam**
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

<p align="center">
  <img src="images/results.png?v=1.0" alt="TMC Flow Test HTML report" width="700">
</p>

---

## What it does

The plugin runs your extruder at progressively higher flow rates and watches the TMC driver's stall signal to find the exact point where the motor starts losing grip. Output: an interactive dashboard showing your max safe flow, the slip onset curve, and recommended slicer values (80% conservative / 90% aggressive).

**Result:** A printer-specific flow curve in 30 minutes, with the same level of detail as a CNC Kitchen video — for free.

## Supported drivers

| Driver | Status | Notes |
|---|---|---|
| **TMC5160** / TMC2130 | ✅ Production-ready | full feature set |
| **TMC2240** | ✅ Production-ready | full feature set |
| **TMC2209** | ⚠️ Experimental | works on some hardware, see [docs/ADVANCED.md](docs/ADVANCED.md#tmc2209-experimental) |

---

## Install

```bash
cd ~
git clone https://github.com/Fragmon/crydteamtools.git
cd crydteamtools
./install.sh max_flow_test
```

The installer symlinks `tmc_flow_test.py` into Klipper's extras folder and the
optional UI macros into your config directory (run it without arguments to
pick plugins interactively).

**Updates**: `cd ~/crydteamtools && git pull`, then restart Klipper.

## Configure

Add this to your `printer.cfg`:

```ini
[tmc_flow_test]
extruder_stepper: extruder
filament_diameter: 1.75
melt_zone_length: 42         # see hotend table below
```

Your existing `[tmcXXXX extruder]` section also needs `driver_SFILT: 1` (TMC5160/2240 only). See [docs/ADVANCED.md](docs/ADVANCED.md#configuration) for full driver-specific settings.

**Melt zone length**:
- V6 / Revo Six: ~13 mm
- Volcano: ~21 mm
- Mosquito: ~18 mm
- Chube Air: ~42 mm
- Goliath / CHC-XL: ~50 mm

## Run a test

Move the toolhead **above the bed with at least 50 mm of clearance** — the test extrudes 2-5 meters of filament. Then in your printer console:

```
TMC_FLOW_FIND_MAX MAX=120 START=25 COARSE_STEP=5
```

Test takes ~10–15 minutes. CSV and HTML report land in `~/printer_data/config/Flowtest/`. Open the HTML in your browser.

### Picking the right START / MAX

Set MAX to roughly 1.5× what you think your hotend can flow:

| Hotend type | Command |
|---|---|
| V6 stock 0.4 (~15 mm³/s) | `TMC_FLOW_FIND_MAX MAX=25 START=5 COARSE_STEP=5` |
| Volcano / CHT 0.4 (~25 mm³/s) | `TMC_FLOW_FIND_MAX MAX=40 START=10 COARSE_STEP=5` |
| Rapido HF 0.4 (~30 mm³/s) | `TMC_FLOW_FIND_MAX MAX=60 START=10 COARSE_STEP=5` |
| Rapido HF 0.6 (~50 mm³/s) | `TMC_FLOW_FIND_MAX MAX=80 START=15 COARSE_STEP=5` |
| Goliath / Chc XL 0.8 (~80 mm³/s) | `TMC_FLOW_FIND_MAX MAX=120 START=25 COARSE_STEP=5` |
| Unknown | `TMC_FLOW_FIND_MAX MAX=150 START=10 COARSE_STEP=5` |

If the test reaches MAX without detecting slip, **double MAX** and re-run.

## Reading the report

The HTML report shows:

- **Hero panel** — your max safe flow rate, plus 80%/90% slicer-ready values
- **Insight cards** — quick-glance status: result quality, where slip first triggered, thermal headroom, driver config
- **Three charts** (click through tabs):
  - **StallGuard signal** — the load curve. Sudden flat spots = slip onset. Click any line in the legend to hide/show it.
  - **Thermal profile** — heater PWM, temperature drop, residence time. Tells you if heater could keep up.
  - **Run-to-run variance** — bar chart showing measurement consistency. Coloured bars: green = clean, yellow = borderline, red = slip likely.
- **Decision timeline** — phase-by-phase explanation of how the result was chosen
- **Test details** — full data table, configuration snapshot, decision trail

The chart background is tinted in three zones: green (safe), yellow (cold-extrusion suspected), red (slip detected).

## Use the result

Copy the **80% value** from the hero panel into your slicer's max volumetric speed setting. That gives you ~20% safety margin for filament variation, temperature drift, and longer prints. The 90% value works after you've validated it with a few real-world prints.

---

## Troubleshooting

**Test reaches MAX without trigger** → your hotend can flow more than MAX. Double MAX and re-run.

**Trigger fires too early (e.g. flow=50 on a high-flow setup)** → check `driver_SFILT: 1` is in your TMC config, restart Klipper, re-test.

**`Section 'tmc_flow_test' is not a valid config section`** → run `sudo systemctl restart klipper` (NOT `FIRMWARE_RESTART`) and verify the symlink exists in `~/klipper/klippy/extras/tmc_flow_test.py`.

**TMC2240 results below ~70 mm³/s on a fast extruder** → you're in StealthChop mode. Remove `stealthchop_threshold` from your `[tmc2240]` section.

**More issues** → see [docs/ADVANCED.md#troubleshooting](docs/ADVANCED.md#troubleshooting).

---

## Need more?

- **Driver-specific tuning, chopper mode internals, TMC2209 setup** → [docs/ADVANCED.md](docs/ADVANCED.md)
- **Command reference** → [docs/COMMANDS.md](docs/COMMANDS.md)
- **How slip detection works** → [docs/INTERNALS.md](docs/INTERNALS.md)

---

## Credits & License

Released under **GPL-3.0**.

Inspired by Klipper's StallGuard implementation and [klipper_tmc_autotune](https://github.com/andrewmcgr/klipper_tmc_autotune).

Plugin author: **Steven (Fragmon) — Crydteam** · [YouTube: @crydteamprinting](https://www.youtube.com/@crydteamprinting)

Contributions and feedback welcome — open an issue or PR.
