# Command reference

[← back to README](../README.md)

The combined V/A/current envelope has its own page:
[V/A envelope](envelope.md). The single-axis / utility commands are below.

Every test saves a CSV + HTML report — see [Output & reports](output.md).

---

## `SPEED_TEST_FIND_MAX_VELOCITY`

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

See [How it works → cruise-aware sizing](how-it-works.md#cruise-aware-sizing)
for `ACCEL`/`CRUISE_RATIO`.

---

## `SPEED_TEST_FIND_MAX_ACCEL`

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

The accel test focuses on **direction-reversal stress**. Each move's distance
is randomly chosen between:

- **Min** = `V²/A` (the triangle distance — just barely touches `SPEED` at the
  peak, then immediately reverses)
- **Max** = `MAX_DIST_FACTOR × V²/A` (capped at axis range)

The distribution is biased toward the short end (`SHORT_BIAS=2` → ~75 % of moves
are in the lower half of the range). The reasoning: motors that lose steps
usually do so on the reversal, not during steady cruise. Short moves that just
touch the target velocity and immediately decelerate stress the driver and
motor harder than long sweeps. Set `SHORT_BIAS=1` for a uniform mix.

---

## `SPEED_TEST_FIND_MAX_SCV`

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

---

## `SPEED_TEST_FIND_OPTIMAL_CURRENT`

Finds the **lowest** TMC `run_current` that still passes a `SPEED` / `ACCEL`
performance target.

**Workflow:**

1. Run `SPEED_TEST_FIND_MAX_ACCEL` (or set a target manually) to know what your
   motor *can* do at full current
2. Pick a comfortable performance target below that maximum (e.g. 80–90 %)
3. Run `SPEED_TEST_FIND_OPTIMAL_CURRENT SPEED=… ACCEL=…` — the plugin starts at
   `MAX_CURRENT` and bisects downward until the test fails
4. The recommended value = lowest passing current + `SAFETY_MARGIN`

**Safety cap.** The `[speed_test] max_current` config option is a **hard
ceiling** — the search will never raise current above this value, even if
`MAX_CURRENT=` is set higher on the command. Always set this for testbench /
new-motor evaluation so you can't accidentally cook a small motor.

| Parameter          | Default | Description                                  |
| ------------------ | ------- | -------------------------------------------- |
| `AXIS`             | default_axis | `X` or `Y`                              |
| `SPEED`            | 200     | Target velocity (mm/s) — what the motor must achieve |
| `ACCEL`            | 5000    | Target acceleration (mm/s²)                  |
| `MAX_CURRENT`      | config / current | Upper bound of the search (A). Falls back to `[speed_test] max_current`, then to the stepper's `run_current × 1.2` |
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

When the test completes, the plugin **restores the original `run_current`** so
you don't accidentally leave the motor running at a reduced setting. Apply the
recommendation by updating `printer.cfg` and `FIRMWARE_RESTART`.

```
# 1) First find what the motor can do at default current
SPEED_TEST_FIND_MAX_ACCEL AXIS=X SPEED=200
# Suppose this reports max safe accel = 25 000 mm/s². Pick 80 % → 20 000.

# 2) Now find the lowest current that still does 20 000 at 200 mm/s
SPEED_TEST_FIND_OPTIMAL_CURRENT AXIS=X SPEED=200 ACCEL=20000
# Output: "Recommended (with margin): 0.85 A"

# 3) Update printer.cfg → run_current: 0.85, FIRMWARE_RESTART.
```

> Tip: the [V/A envelope](envelope.md) does this current trim automatically per
> velocity (stage 3), so you usually don't need this command separately.

---

## `SPEED_TEST_BENCHMARK`

Runs a repeatable stress test at a fixed `SPEED` / `ACCEL` / `SCV` and reports
pass/fail. Useful for regression-testing after changes.

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

---

## `SPEED_TEST_STATUS`

Diagnostic. Prints structure, default axis, axis bounds, TMC presence +
run_current per axis, the `max_missed` skip tolerance, and whether the stepper
position is readable. It also **warns if `[endstop_phase]` is loaded** (which
must be removed — see [Configuration](configuration.md)).
