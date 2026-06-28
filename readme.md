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
testbench: False              # see "Testbench mode" below
max_current: 1.5              # hard safety cap (A) for OPTIMAL_CURRENT.
                              # 0 (default) = no cap.
#output_dir: ~/printer_data/config/Speedtest
```

### Testbench mode

If you're testing a **single stepper on a bench** (only X wired, no Y, no Z,
no full printer), set:

```ini
[speed_test]
testbench: True
default_axis: X
```

In testbench mode the plugin:
- Only homes X (no `G28 Y`, no `G28 Z`, no full `G28`)
- Doesn't lift Z before moves
- Only checks X for skipped steps
- Only samples X-stepper TMC (if present)
- Refuses `SPEED_TEST_FIND_MAX_SCV` and `SPEED_TEST_BENCHMARK` (those need XY)

You can also enable it per command without changing the config:

```
SPEED_TEST_FIND_MAX_VELOCITY AXIS=X TESTBENCH=1
SPEED_TEST_FIND_MAX_ACCEL    AXIS=X TESTBENCH=1 SPEED=200
```

`TESTBENCH=0` forces it off even when the config has `testbench: True`.

Make sure your `[endstop_phase]` either covers only X, or that you've added a
real X endstop. A minimal testbench `[endstop_phase]` looks like:

```ini
[endstop_phase stepper_x]
```

That tracks X only and won't complain about a missing Y endstop.

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
| `ACCEL`            | auto    | Acceleration (mm/s²). `0` or omitted = auto-compute so MAX hits the cruise target |
| `CRUISE_RATIO`     | 0.5     | Minimum fraction of each move spent at the target velocity. 0.5 = at least half the distance at full speed |
| `REPEAT`           | 5       | Movements per coarse/bisect step             |
| `VERIFY_REPEATS`   | 20      | Movements during verification                |
| `MAX_BISECT_STEPS` | 6       | Cap on bisection iterations                  |
| `DISTANCE`         | full    | `full` (axis end-to-end) or `short` (just enough to hit target velocity) |
| `TESTBENCH`        | config  | `1` = single-stepper bench mode (X only, no Y/Z) |
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
| `MAX_DIST_FACTOR`  | 4       | Upper bound for random moves: `MAX_DIST_FACTOR × V²/A`. 4 ≈ 75 % cruise at the long end, capped at axis range |
| `SHORT_BIAS`       | 2       | Distribution skew: 1 = uniform, 2 = quadratic (default, ~75 % moves in the short half), 3 = cubic (even shorter) |
| `SEED`             | 12345   | Random seed — same seed reproduces the same move sequence |
| `TESTBENCH`        | config  | `1` = single-stepper bench mode (X only, no Y/Z) |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

The accel test focuses on **direction-reversal stress**. Each move's distance is randomly chosen between:

- **Min** = `V²/A` (the triangle distance — just barely touches `SPEED` at the peak, then immediately reverses)
- **Max** = `MAX_DIST_FACTOR × V²/A` (capped at axis range)

The distribution is biased toward the short end (`SHORT_BIAS=2` → ~75 % of moves are in the lower half of the range). The reasoning: motors that lose steps usually do so on the reversal, not during steady cruise. Short moves that just touch the target velocity and immediately decelerate stress the driver and motor harder than long sweeps.

Set `SHORT_BIAS=1` for uniform distribution if you'd rather have a flat mix of short and long moves.

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

### `SPEED_TEST_FIND_OPTIMAL_CURRENT`

Finds the **lowest** TMC `run_current` that still passes a `SPEED` / `ACCEL` performance target.

Workflow:

1. Run `SPEED_TEST_FIND_MAX_ACCEL` (or set a target manually) to know what your motor *can* do at full current
2. Pick a comfortable performance target below that maximum (e.g. 80–90 %)
3. Run `SPEED_TEST_FIND_OPTIMAL_CURRENT SPEED=… ACCEL=…` — the plugin starts at `MAX_CURRENT` and bisects downward until the test fails
4. The recommended value = lowest passing current + `SAFETY_MARGIN`

**Safety cap.** The `[speed_test] max_current` config option is a **hard ceiling** — the search will never raise current above this value, even if `MAX_CURRENT=` is set higher on the command. Always set this for testbench / new motor evaluation so you can't accidentally cook a small motor.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `AXIS`             | default_axis | `X` or `Y`                              |
| `SPEED`            | 200     | Target velocity (mm/s) — what the motor must achieve |
| `ACCEL`            | 5000    | Target acceleration (mm/s²)                  |
| `MAX_CURRENT`      | config / current | Upper bound of the search (A). Falls back to `[speed_test] max_current`, then to the current stepper's `run_current × 1.2` |
| `MIN_CURRENT`      | 0.3     | Lower bound of the search (A)                |
| `COARSE_STEP`      | 0.1     | Phase-1 decrement per step (A)               |
| `MIN_STEP`         | 0.05    | Bisection precision (A)                      |
| `SAFETY_MARGIN`    | 0.10    | Final result = `lowest_passing × (1 + SAFETY_MARGIN)` |
| `REPEAT`           | 10      | Movements per current step                   |
| `VERIFY_REPEATS`   | 30      | Movements during verification                |
| `MAX_BISECT_STEPS` | 6       | Cap on bisection iterations                  |
| `MAX_DIST_FACTOR`  | 4       | Upper bound for the per-move random distance |
| `SHORT_BIAS`       | 2       | Short-move bias (same as accel test)         |
| `SEED`             | 12345   | Same seed across all current steps so move sequences are identical → fair comparison |
| `TESTBENCH`        | config  | `1` = single-stepper bench mode              |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

When the test completes, the plugin **restores the original `run_current`** so you don't accidentally leave the motor running at a reduced setting. Apply the recommendation by updating `printer.cfg` and `FIRMWARE_RESTART`.

**Example:**

```
# 1) First find what the motor can do at default current
SPEED_TEST_FIND_MAX_ACCEL AXIS=X SPEED=200

# Suppose this reports max safe accel = 25 000 mm/s². Pick 80 %:
# target ACCEL = 20 000.

# 2) Now find the lowest current that still does 20 000 at 200 mm/s
SPEED_TEST_FIND_OPTIMAL_CURRENT AXIS=X SPEED=200 ACCEL=20000

# Output: "Recommended (with margin): 0.85 A"

# 3) Update printer.cfg → run_current: 0.85, FIRMWARE_RESTART.
```

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

## Cruise-aware sizing

A short axis combined with a high target velocity can give misleading results:
the motor briefly touches the target speed at the peak of a triangle profile
and immediately decelerates again, never actually *cruising* there.

The plugin avoids that by sizing motion so that **at least half of every
move** is spent at the target velocity (`CRUISE_RATIO` default 0.5).

Two modes:

**Auto-acceleration (`ACCEL` omitted or `0`)** — recommended.
The plugin computes:

```
ACCEL ≥ MAX_V² / (axis_range × (1 − CRUISE_RATIO))
```

and rounds up to the nearest 500 mm/s². You'll see a line like:

```
Auto-set ACCEL = 26000 mm/s² so MAX=2000 mm/s has ≥50% cruise on 310 mm of usable X travel.
```

**Fixed acceleration (`ACCEL=…` given)** — useful for chasing motor limits.
The plugin clips `MAX` down to the velocity where the cruise ratio still
holds:

```
MAX = √(axis_range × ACCEL × (1 − CRUISE_RATIO))
```

You'll see:

```
MAX=2000 mm/s exceeds the velocity that keeps ≥50% cruise at ACCEL=5000 …
Clipped MAX to 880 mm/s.
To test higher, increase ACCEL or omit it for auto-sizing.
```

The achieved cruise fraction is reported per measurement (e.g. `cruise=58%`)
and saved in the CSV/HTML report so you can verify each step was a fair test.

`CRUISE_RATIO=0` reverts to the old behaviour (triangle profile, no cruise
required). `CRUISE_RATIO=0.75` is stricter — useful when comparing motors
under more realistic load.

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
