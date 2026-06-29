# Crydteam Tools for Klipper

A collection of Klipper diagnostic and tuning plugins by **Steven (Fragmon) — Crydteam**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Klipper](https://img.shields.io/badge/Klipper-compatible-green.svg)](https://www.klipper3d.org/)
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## `speed_test.py`

**Adaptive max velocity / acceleration / square-corner-velocity finder for stepper motors**, plus a combined **velocity–acceleration envelope** that also trims motor current.

It pushes a motor until it loses steps, then narrows in on the safe limit with an adaptive bracket search (≈ 8–12 measurements instead of dozens). Skipped steps are detected by reading the stepper's MCU position directly after a re-home — **no `[endstop_phase]` needed** (and it must not be loaded). Every run saves a CSV + interactive HTML report.

> Looking for the extruder StallGuard flow test? That's in a separate repo: [klipper_max_flow_test](https://github.com/Fragmon/klipper_max_flow_test).

---

## Requirements

- Klipper or Kalico
- An endstop that homes repeatably (a physical switch is best)
- **`[endstop_phase]` must NOT be loaded** — it aborts homing the moment the motor loses a step. [Why →](docs/configuration.md#do-not-enable-endstop_phase)
- Optional: TMC drivers for StallGuard monitoring and current trimming

## Installation

```bash
cd ~
git clone https://github.com/Fragmon/crydteamtools.git
ln -sf ~/crydteamtools/speed_test.py ~/klipper/klippy/extras/speed_test.py
```

Add the config below, then `FIRMWARE_RESTART`. (Or run `./install.sh`.)

## Minimal config

```ini
[speed_test]
structure: corexy        # cartesian | corexy
default_axis: X
max_current: 1.5         # safety cap (A) for current tests; 0 = no cap
```

Then run `SPEED_TEST_STATUS` to confirm it loaded. Full options, testbench mode,
and the `[endstop_phase]` note: **[Configuration →](docs/configuration.md)**

---

## Commands

| Command | What it does |
| ------- | ------------ |
| [`SPEED_TEST_FIND_ENVELOPE`](docs/envelope.md) | **The flagship.** Sweeps velocities, finds max safe accel at each, trims `run_current`, and validates with a simulated print → a full V/A/current operating map |
| [`SPEED_TEST_FIND_MAX_VELOCITY`](docs/commands.md#speed_test_find_max_velocity) | Max safe velocity for an axis |
| [`SPEED_TEST_FIND_MAX_ACCEL`](docs/commands.md#speed_test_find_max_accel) | Max safe acceleration at a fixed velocity |
| [`SPEED_TEST_FIND_MAX_SCV`](docs/commands.md#speed_test_find_max_scv) | Max safe square-corner velocity (XY) |
| [`SPEED_TEST_FIND_OPTIMAL_CURRENT`](docs/commands.md#speed_test_find_optimal_current) | Lowest `run_current` that still hits a speed/accel target |
| [`SPEED_TEST_BENCHMARK`](docs/commands.md#speed_test_benchmark) | Repeatable pass/fail stress test |
| [`SPEED_TEST_STATUS`](docs/commands.md#speed_test_status) | Diagnostic — config, axes, TMC, warnings |

### Quick start

```
SPEED_TEST_STATUS                              # check config first
SPEED_TEST_FIND_ENVELOPE AXIS=X                # the combined sweep
SPEED_TEST_FIND_MAX_ACCEL AXIS=X SPEED=200     # a single accel test
```

---

## Documentation

- **[Configuration](docs/configuration.md)** — all options, testbench mode, the `[endstop_phase]` rule
- **[V/A envelope](docs/envelope.md)** — the combined sweep, four-stage search, parameters
- **[Command reference](docs/commands.md)** — full parameter tables for the single-axis / utility commands
- **[How it works](docs/how-it-works.md)** — skip detection, adaptive search, TMC monitoring, cruise-aware sizing
- **[Output & reports](docs/output.md)** — CSV / HTML files
- **[Troubleshooting](docs/troubleshooting.md)**

---

## Credits & License

Plugin by Steven (Fragmon) — Crydteam · YouTube: [@crydteamprinting](https://www.youtube.com/@crydteamprinting)

Released under the GNU General Public License v3.0.

Algorithm derived from [Ellis's Print Tuning Guide](https://ellis3dp.com/Print-Tuning-Guide/articles/determining_max_speeds_accels.html), reimplemented in Python with adaptive bisection and HTML reporting. The stall-safe short *jab* move in the envelope search is adapted from Anonoei's [klipper_auto_speed](https://github.com/Anonoei/klipper_auto_speed) (MIT).

## Contributing

Issues and pull requests welcome. If you've tested on hardware not listed in the docs, let me know — I'd love to add confirmed-working markers.
