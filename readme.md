# Crydteam Tools for Klipper

A collection of Klipper diagnostic and tuning plugins by **Steven (Fragmon) — Crydteam**.

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Klipper](https://img.shields.io/badge/Klipper-compatible-green.svg)](https://www.klipper3d.org/)

[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## Plugins in this repo

| Plugin | Purpose |
| ------ | ------- |
| `speed_test.py` | Adaptive max-velocity / max-acceleration / max-SCV detection for steppers, plus a combined velocity–acceleration envelope sweep. Skipped-step detection by reading the stepper MCU position directly (no `endstop_phase`). CSV + HTML report. |

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
- Endstops that home repeatably (a physical switch is best; sensorless homing has more jitter — raise `max_missed` if needed)
- **`[endstop_phase]` must NOT be loaded** — see the note in [Configuration](#configuration). Skipped-step detection no longer needs it, and if it is configured it aborts homing the moment the motor loses a step.
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

### 1. Do NOT enable `endstop_phase`

Skipped-step detection reads the stepper's MCU step position directly after
each re-home — **no `[endstop_phase]` is needed**. If `[endstop_phase]` (or
any `[endstop_phase stepper_*]`) is configured, **remove it** and
`FIRMWARE_RESTART`: its TMC phase cross-check raises `Endstop … incorrect
phase` and aborts homing exactly when the motor loses a step — which is the
event the test is built to detect. `SPEED_TEST_STATUS` warns if it's loaded.

### 2. Plugin section

```ini
[speed_test]
structure: corexy             # cartesian | corexy
default_axis: X               # X or Y — used when AXIS is not given
margin: 20                    # mm to keep from each axis end
z_pos: 20                     # Z height during XY tests
monitor_tmc: True             # poll TMC StallGuard during moves
testbench: False              # see "Testbench mode" below
max_missed: 1.5               # skip tolerance, in FULL motor steps. A move
                              # counts as a skip when the stepper drifts more
                              # than this across a re-home. ~1 step of homing
                              # jitter is normal; real stalls lose far more.
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

All you need wired is a working X endstop so `G28 X` homes repeatably — the
plugin reads the X stepper position directly, so no `[endstop_phase]` setup
is required (and it should not be present).

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

### `SPEED_TEST_FIND_ENVELOPE`

Maps the **velocity–acceleration envelope** of an axis — the combined test.

`FIND_MAX_ACCEL` finds the max accel at *one* fixed `SPEED`. But velocity and acceleration are physically coupled: a stepper's usable torque drops as speed rises (back-EMF eats into the current the driver can push through the windings), so the max safe acceleration is **lower at high velocity and higher at low velocity**. Pick the wrong `SPEED` for an accel test and the result is either unsafe at higher speeds or needlessly conservative at lower ones.

This test sweeps several velocities and finds the max safe accel at each, producing the whole curve plus a balanced `max_velocity` / `max_accel` recommendation taken from the **knee** of the envelope (the point past which buying more speed costs the most acceleration). To save time, each velocity warm-starts its accel search from the previous (lower-velocity) result — since the curve only falls as velocity rises.

**Three-stage search per velocity.** A value is accepted only after it passes *all three* stages; if stage 2 or 3 fails it drops the ceiling and goes back to stage 1 (up to `MAX_REDO` times). If nothing passes every stage, that velocity is honestly **excluded** from the curve rather than reported as safe.

1. **Stage 1 — bracket.** A relative-accuracy **binary search** (no fixed step; stops when the guess is within `ACCEL_ACCU` of a bracket bound) using short, stall-safe *jab* moves. Each jab is sized to the motion profile (`V²/A`) and anchored **near home, 10 % into the travel**, so the search is fast, a stall barely grinds, and the re-home stays short. The search ends on a freshly **confirmed** passing value before handing off. (Jab idea adapted from Anonoei's [klipper_auto_speed](https://github.com/Anonoei/klipper_auto_speed).)
2. **Stage 2 — validate.** The candidate is re-tested with the thorough reversal-stress pattern (random distances across the axis — where motors actually lose steps). Must pass, or back to stage 1 lower.
3. **Stage 3 — simulated print.** A print-like run at `BENCH_DERATE` × the candidate: bursts of short infill zigzag + perimeter passes + travels, realistic lengths. For safety it stays in the **centre of the axis** and runs in **chunks** (`BENCH_CHUNK` moves) that re-home between them and **abort on the first lost-step chunk**, so a stall can't grind the whole run into the limit. Must pass, or back to stage 1 lower.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `AXIS`             | default_axis | `X` or `Y`                              |
| `V_MIN`            | 100     | Lowest velocity in the sweep (mm/s)          |
| `V_MAX`            | 500     | Highest velocity in the sweep (mm/s)         |
| `V_POINTS`         | 5       | Number of velocities sampled between MIN and MAX |
| `A_MIN`            | 1000    | Lower bound of the accel search (mm/s²)      |
| `A_MAX`            | 50000   | Upper bound of the accel search (mm/s²)      |
| `ACCEL_ACCU`       | 0.05    | Stage-1 stop tolerance, *relative* (0.05 = ±5 %). Resolution scales with the value instead of a fixed step |
| `REPEAT`           | 15      | Jab moves per stage-1 search step            |
| `VERIFY_REPEATS`   | 30      | Reversal-stress moves in stage-2 validation  |
| `MAX_ITERS`        | 12      | Cap on stage-1 binary-search iterations      |
| `BENCH_SHORT`      | 400     | Stage-3 short **infill** segments            |
| `BENCH_LONG`       | 60      | Stage-3 long **travel** moves                |
| `BENCH_CHUNK`      | 80      | Stage-3 moves per chunk before re-home + skip-check (abort on fail) |
| `BENCH_DERATE`     | 0.9     | Stage-3 tests/accepts at this fraction of the found value — safer, fewer crashes |
| `MAX_REDO`         | 4       | Re-determination attempts before a velocity is excluded |
| `MAX_DIST_FACTOR`  | 4       | Upper bound for stage-2 random moves         |
| `SHORT_BIAS`       | 2       | Stage-2 short-move bias                       |
| `SEED`             | 12345   | Random seed for reproducible move sequences  |
| `TESTBENCH`        | config  | `1` = single-stepper bench mode (X only). Stage 3 also runs on the single axis |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

The console prints the full table plus three ready-to-paste operating points — **balanced** (the knee), **speed-priority** (highest velocity with its accel ceiling), and **accel-priority** (lowest velocity with the highest accel) — each already including a 10 % safety margin. The HTML report draws the envelope curve with the knee highlighted, and lists your **current `printer.cfg` values** and the **TMC driver / run_current** side by side, plus a free-text **toolhead-weight** field saved with the report.

A velocity point is skipped if reaching it within the axis travel would need an accel above the search ceiling (very short axes can't reach high speeds in a triangle move) — the skip is reported so you know the curve has a gap.

> **Runtime:** stage 3 is thorough — hundreds of moves per accepted value, plus a fresh run on every re-determination. For a quick first pass, lower the load, e.g. `V_POINTS=3 BENCH_SHORT=120 BENCH_LONG=20 BENCH_CHUNK=40`.

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

Diagnostic. Prints structure, default axis, axis bounds, TMC presence + run_current per axis, the `max_missed` skip tolerance, and whether the stepper position is readable. It also **warns if `[endstop_phase]` is loaded** (which must be removed — see [Configuration](#configuration)).

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

# Map the combined velocity/acceleration envelope (5 speeds, 100–500 mm/s)
SPEED_TEST_FIND_ENVELOPE AXIS=X

# Finer envelope: 8 speeds up to 800 mm/s
SPEED_TEST_FIND_ENVELOPE AXIS=Y V_MAX=800 V_POINTS=8

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

The plugin reads the stepper's **MCU step position** directly from the kinematics (`get_mcu_position`) right after each re-home, and compares it with the position before the test move. A shift larger than `max_missed` full steps means the motor lost steps. Reading the position directly means **no `[endstop_phase]` module is needed** — and crucially, it can't be: `endstop_phase` cross-checks the TMC microstep phase and aborts homing with `incorrect phase` the instant a step is lost, which is exactly the event this test is built to measure.

About `max_missed`: without `endstop_phase`'s phase-snapping, a home can land up to ~1 full step off purely from mechanical jitter, so the default tolerance of 1.5 full steps leaves headroom. A real stall loses tens to thousands of microsteps — far above the threshold.

### Why adaptive bisection?

A linear sweep from 100 → 800 mm/s with a 10 mm/s step takes **70 measurements**. The envelope's relative-accuracy binary search converges in **~6–10 measurements** per velocity. For longer per-move durations or noisy environments, that difference is 20+ minutes.

### Optional TMC monitoring

If you have TMC drivers on X/Y with StallGuard support, the plugin polls SG values at 20 Hz during each move. Lower SG = higher load. Even on tests that pass, seeing SG drop a lot tells you the motor was *close* to slipping — useful margin information for the slicer.

Supported drivers (auto-detected): TMC2240, TMC2209, TMC5160, TMC2130, TMC2660, TMC2226, TMC2208.

---

## Troubleshooting

### "Endstop stepper_x incorrect phase (got … vs …)"

`[endstop_phase]` is loaded. It aborts homing the moment the motor loses a step — which the test does on purpose. **Remove `[endstop_phase]` and any `[endstop_phase stepper_*]` from `printer.cfg` and `FIRMWARE_RESTART`.** The plugin detects skips without it. `SPEED_TEST_STATUS` warns when it's still present.

### Tests always pass even at obviously-too-high values

- Check that your endstop is a physical switch that homes repeatably (sensorless homing gives more jitter — raise `max_missed` only if you see false *positives*, not to mask false negatives)
- If homing isn't repeatable, the position reference drifts; verify `G28` lands consistently
- Lower `ACCEL_ACCU` (e.g. 0.02) so the envelope search resolves the limit more finely

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

The stall-safe short *jab* move used in stage 1 of the envelope search is adapted from Anonoei's [klipper_auto_speed](https://github.com/Anonoei/klipper_auto_speed) (MIT).

---

## Contributing

Issues and pull requests welcome. If you've tested on hardware not listed above, let me know — I'd love to add confirmed-working markers.
