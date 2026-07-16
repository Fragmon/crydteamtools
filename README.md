# Crydteam Tools for Klipper

A collection of Klipper diagnostic and tuning plugins by **Steven (Fragmon) — Crydteam**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Klipper](https://img.shields.io/badge/Klipper-compatible-green.svg)](https://www.klipper3d.org/)
[![Kalico](https://img.shields.io/badge/Kalico-compatible-brightgreen.svg)](https://github.com/KalicoCrew/kalico)
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## Plugins

| Plugin | What it does | Docs |
| ------ | ------------ | ---- |
| **[Speed Test](speed_test/README.md)** | Finds a motor's real limits: max velocity, max acceleration per speed (limit map), square-corner velocity and the lowest run_current that still holds them — validated with a simulated print. CSV + interactive HTML reports, beginner-friendly control-panel GUI. | [speed_test/](speed_test/README.md) |
| **[TMC Flow Test](max_flow_test/README.md)** | Finds your printer's real max flow rate in ~30 minutes without test prints: runs the extruder at rising flow rates and watches the TMC StallGuard signal for slip onset. Interactive dashboard with recommended slicer values. | [max_flow_test/](max_flow_test/README.md) |

More plugins will join this collection — each lives in its own folder with its
own README and docs.

---

## Installation

```bash
cd ~
git clone https://github.com/Fragmon/crydteamtools.git
cd crydteamtools
./install.sh
```

The installer lets you **pick which plugins to install** (or pass them
directly: `./install.sh all`, `./install.sh speed_test`,
`./install.sh max_flow_test`). It symlinks the plugin files into Klipper's
`extras` folder and the optional UI macros into your config directory.

Then configure the plugin(s) in `printer.cfg` (see each plugin's README) and
run `FIRMWARE_RESTART`.

## Updating

The installer registers the repo with **Moonraker's update manager**, so new
releases appear directly in Mainsail/Fluidd (Machine → Update Manager) —
one-click update, Klipper is restarted automatically.

Manual alternative:

```bash
cd ~/crydteamtools && git pull
```

Then `FIRMWARE_RESTART`. Re-run `./install.sh` only if new plugins were added.

---

## Credits & License

Plugins by Steven (Fragmon) — Crydteam · YouTube: [@crydteamprinting](https://www.youtube.com/@crydteamprinting)

Released under the GNU General Public License v3.0. Per-plugin credits are
listed in each plugin's README.

## Contributing

Issues and pull requests welcome. If you've tested on hardware not listed in
the docs, let me know — I'd love to add confirmed-working markers.
