# Crydteam Tools for Klipper

A collection of Klipper diagnostic and tuning plugins by **Steven (Fragmon) — Crydteam**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Klipper](https://img.shields.io/badge/Klipper-compatible-green.svg)](https://www.klipper3d.org/)

[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## Plugins in this repo

| Plugin | Purpose |
| ------ | ------- |
| `speed_test.py` | Adaptive max-velocity / max-acceleration / max-SCV detection for steppers. Skipped-step detection via `endstop_phase`. CSV + HTML report. |

> Looking for the extruder StallGuard flow test? That's in a separate repo: [klipper_max_flow_test](https://github.com/Fragmon/klipper_max_flow_test).

---

# `speed_test.py`

**Adaptive max velocity / acceleration / square-corner-velocity (SCV) finder for stepper motors.**

Instead of stepping linearly from low to high and stopping at the first skip, this plugin uses a **three-phase bracket-bisection** to converge on the safe limit in roughly 8–12 measurements:

1. **Coarse sweep** — increase the test value in big steps until skipped steps are detected
2. **Bisection** — halve the bracket repeatedly until the limit is known to within `MIN_STEP`
3. **Verification** — confirm the value with extra repetitions

Each test re-homes the relevant axes and compares the MCU position with the previous home — any difference larger than `microsteps` is treated as a skip.

If your XY motors are TMC drivers, the plugin can additionally poll `StallGuard` at 20 Hz during each move and include the min/median SG values in the HTML report — useful to see *how close* you were to the limit on tests that still passed.

---

## Requirements

- Klipper or Kalico
- `[endstop_phase]` module configured (Klipper-stock — no extra install)
- Endstops that home reliably to the same MCU phase each time
- Optional: TMC drivers on XY for StallGuard monitoring (TMC2240, TMC2209, TMC5160, TMC2130, TMC2660)

---

## Installation

```bash
cd ~
git clone https://github.com/Fragmon/crydteamtools.git
ln -sf ~/crydteamtools/speed_test.py ~/klipper/klippy/extras/speed_test.py
```

Add the [configuration](#configuration) to your `printer.cfg`, then:

```
FIRMWARE_RESTART
```

Or use the install script:

```bash
cd ~/crydteamtools
./install.sh
```

---

## Configuration

### 1. Enable `endstop_phase` (Klipper-stock)

Add to your `printer.cfg` — required for skipped-step detection:

```ini
[endstop_phase]
```

### 2. Plugin section

```ini
[speed_test]
structure: corexy             # cartesian | corexy
default_axis: X               # X or Y — used when AXIS is not given
margin: 20                    # mm to keep from each axis end
z_pos: 20                     # Z height during XY tests
monitor_tmc: True             # poll TMC StallGuard during moves
#output_dir: ~/printer_data/config/Speedtest
```

After `FIRMWARE_RESTART`, run `SPEED_TEST_STATUS` to verify the plugin loaded and your axes are recognised.

---

## Commands

### `SPEED_TEST_FIND_MAX_VELOCITY`

Finds the maximum safe **velocity** for an axis using adaptive bisection.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `AXIS`             | default_axis | `X` or `Y` — the axis to test           |
| `MIN`              | 50      | Lower bound (mm/s)                           |
| `MAX`              | 500     | Upper bound (mm/s)                           |
| `COARSE_STEP`      | 25      | Phase-1 increment (mm/s)                     |
| `MIN_STEP`         | 5       | Bisection precision (mm/s)                   |
| `ACCEL`            | 5000    | Acceleration during the test (mm/s²)         |
| `REPEAT`           | 5       | Movements per coarse/bisect step             |
| `VERIFY_REPEATS`   | 20      | Movements during verification                |
| `MAX_BISECT_STEPS` | 6       | Cap on bisection iterations                  |
| `DISTANCE`         | full    | `full` (axis end-to-end) or `short` (just enough to hit target velocity) |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

### `SPEED_TEST_FIND_MAX_ACCEL`

Finds the maximum safe **acceleration** at a fixed velocity.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `AXIS`             | default_axis | `X` or `Y`                              |
| `MIN`              | 500     | Lower bound (mm/s²)                          |
| `MAX`              | 50000   | Upper bound (mm/s²)                          |
| `COARSE_STEP`      | 2500    | Phase-1 increment (mm/s²)                    |
| `MIN_STEP`         | 250     | Bisection precision (mm/s²)                  |
| `SPEED`            | 200     | Test velocity (mm/s)                         |
| `REPEAT`           | 30      | Movements per coarse/bisect step             |
| `VERIFY_REPEATS`   | 50      | Movements during verification                |
| `MAX_BISECT_STEPS` | 6       | Cap on bisection iterations                  |
| `MIN_DISTANCE`     | 50      | Minimum movement distance (mm)               |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

### `SPEED_TEST_FIND_MAX_SCV`

Finds the maximum safe **square-corner velocity** for XY.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `MIN`              | 1       | Lower bound (mm/s)                           |
| `MAX`              | 20      | Upper bound (mm/s)                           |
| `COARSE_STEP`      | 2       | Phase-1 increment (mm/s)                     |
| `MIN_STEP`         | 0.5     | Bisection precision (mm/s)                   |
| `SPEED`            | 200     | Pattern velocity (mm/s)                      |
| `ACCEL`            | 5000    | Acceleration (mm/s²)                         |
| `CORNER_SIZE`      | 50      | Pattern side length (mm)                     |
| `REPEAT`           | 3       | Pattern reps per SCV step                    |
| `VERIFY_REPEATS`   | 5       | Pattern reps during verification             |
| `MAX_BISECT_STEPS` | 6       | Cap on bisection iterations                  |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

### `SPEED_TEST_BENCHMARK`

Runs a repeatable stress test at a fixed `SPEED` / `ACCEL` / `SCV` and reports pass/fail. Useful for regression-testing after changes.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `SPEED`            | 300     | Test velocity (mm/s)                         |
| `ACCEL`            | 10000   | Acceleration (mm/s²)                         |
| `ITERATIONS`       | 3       | Pattern repetitions                          |
| `SCV`              | 5       | Square-corner velocity                       |
| `BOUND`            | 40      | Border margin from axis ends (mm)            |
| `SMALLPATTERNSIZE` | 20      | Small-pattern box size (mm)                  |
| `SEED`             | 12345   | Seed for reproducible offsets                |
| `ZPOS`             | z_pos   | Z height during test                         |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

### `SPEED_TEST_STATUS`

Diagnostic. Prints structure, default axis, axis bounds, TMC presence per axis, and whether `endstop_phase` is loaded.

---

## Output

Files are saved to `~/printer_data/config/Speedtest/` (configurable via `output_dir`):

```
speed_<kind>_YYYY-MM-DD_HH-MM-SS.csv     ← raw data
speed_<kind>_YYYY-MM-DD_HH-MM-SS.html    ← interactive report
```

The HTML report renders in any browser and includes:

- **Lost-steps chart** — bar per measurement; height = how many microsteps were lost
- **TMC StallGuard chart** (when monitoring is enabled and XY drivers are TMC)
- **Data table** with phase / value / pass-fail / lost-steps / SG min+median per axis
- **Phase markers** — dashed vertical lines at Coarse → Bisect → Verify transitions
- **Stop reason** — which trigger fired and at what value

---

## Examples

```
# Find max velocity on X with adaptive bisection (default range 50–500 mm/s)
SPEED_TEST_FIND_MAX_VELOCITY AXIS=X

# Higher range with shorter test distance
SPEED_TEST_FIND_MAX_VELOCITY AXIS=Y MIN=100 MAX=800 DISTANCE=short

# Find max acceleration at 200 mm/s (default range 500–50000 mm/s²)
SPEED_TEST_FIND_MAX_ACCEL AXIS=X SPEED=200

# Tighter precision on accel
SPEED_TEST_FIND_MAX_ACCEL AXIS=X MIN_STEP=100 VERIFY_REPEATS=80

# Find max square-corner velocity for printing motion
SPEED_TEST_FIND_MAX_SCV SPEED=300 ACCEL=10000

# Regression test — fixed speed/accel, pass/fail report
SPEED_TEST_BENCHMARK SPEED=500 ACCEL=20000 ITERATIONS=5

# Diagnostic — confirm config is right before testing
SPEED_TEST_STATUS
```

---

## How it works

### Skipped-step detection

Klipper's `endstop_phase` module records the **MCU step phase** at each home. If the motor lost steps during a move, the next home will land on a different phase by the corresponding number of microsteps. The plugin compares phases before and after each test step — a difference larger than `microsteps` (one full step) is reported as a skip.

This is the same mechanism the user macro version uses, but the plugin reads it directly from the module's API rather than via Jinja templates.

### Why adaptive bisection?

A linear sweep from 100 → 800 mm/s with a 10 mm/s step takes **70 measurements**. Bisection with the same precision takes **~12 measurements** (6 coarse + 3 bisect + 1 verify, give or take). For longer per-move durations or noisy environments, that difference is 20+ minutes.

### Optional TMC monitoring

If you have TMC drivers on X/Y with StallGuard support, the plugin polls SG values at 20 Hz during each move. Lower SG = higher load. Even on tests that pass, seeing SG drop a lot tells you the motor was *close* to slipping — useful margin information for the slicer.

Supported drivers (auto-detected): TMC2240, TMC2209, TMC5160, TMC2130, TMC2660, TMC2226, TMC2208.

---

## Troubleshooting

### "speed_test: [endstop_phase] module not configured"

Add `[endstop_phase]` to your `printer.cfg` and `FIRMWARE_RESTART`. The module ships with Klipper — no extra install.

### Tests always pass even at obviously-too-high values

- Make sure `[endstop_phase]` is actually loaded — `SPEED_TEST_STATUS` reports this
- Check that your endstops are physical switches that home to a repeatable phase (sensorless homing can give false negatives here)
- Try a smaller `MIN_STEP` to bisect more aggressively past stable plateaus

### Test triggers immediately at MIN

`MIN` is already past your motor's limit. Lower it and reduce `COARSE_STEP`:

```
SPEED_TEST_FIND_MAX_VELOCITY AXIS=X MIN=20 COARSE_STEP=10
```

### "Need X mm of axis range for SPEED=Y at MIN=Z"

The accel test needs enough distance to actually reach `SPEED` while accelerating at `MIN`. Either raise `MIN` or lower `SPEED`.

### Results vary between runs

- Increase `REPEAT` (e.g. 50) and `VERIFY_REPEATS` (e.g. 100) for accel tests — more cycles = less luck-dependent
- Check belt tension and pulley setscrews; loose mechanics give random skips
- If TMC monitoring shows SG dropping to near 0 even on passes, the motor is right at its torque limit — consider increasing `run_current`

---

## Credits & License

Plugin by Steven (Fragmon) — Crydteam.

YouTube: [@crydteamprinting](https://www.youtube.com/@crydteamprinting)

Released under the GNU General Public License v3.0.

Algorithm derived from [Ellis's Print Tuning Guide](https://ellis3dp.com/Print-Tuning-Guide/articles/determining_max_speeds_accels.html), reimplemented in Python with adaptive bisection and HTML reporting.

---

## Contributing

Issues and pull requests welcome. If you've tested on hardware not listed above, let me know — I'd love to add confirmed-working markers.
