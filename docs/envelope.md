# `SPEED_TEST_FIND_ENVELOPE` — the V/A/current envelope

[← back to README](../README.md)

Maps the **velocity–acceleration envelope** of an axis — the combined test.

`FIND_MAX_ACCEL` finds the max accel at *one* fixed `SPEED`. But velocity and
acceleration are physically coupled: a stepper's usable torque drops as speed
rises (back-EMF eats into the current the driver can push through the
windings), so the max safe acceleration is **lower at high velocity and higher
at low velocity**. Pick the wrong `SPEED` for an accel test and the result is
either unsafe at higher speeds or needlessly conservative at lower ones.

This test sweeps several velocities and finds the max safe accel at each,
producing the whole curve plus a balanced `max_velocity` / `max_accel`
recommendation taken from the **knee** of the envelope (the point past which
buying more speed costs the most acceleration). To save time, each velocity
warm-starts its accel search from the previous (lower-velocity) result — since
the curve only falls as velocity rises.

## Four-stage search per velocity

A value is accepted only once it survives every stage. Any failure drops the
accel ceiling and goes back to stage 1 (up to `MAX_REDO` times); if nothing
passes, that velocity is honestly **excluded** rather than reported as safe.

1. **Stage 1 — bracket.** A relative-accuracy **binary search** (no fixed
   step; stops when the guess is within `ACCEL_ACCU` of a bracket bound) using
   short, stall-safe *jab* moves. Each jab is sized to the motion profile
   (`V²/A`) and anchored **near home, 10 % into the travel**, so the search is
   fast, a stall barely grinds, and the re-home stays short. The search ends on
   a freshly **confirmed** passing value before handing off. (Jab idea adapted
   from Anonoei's [klipper_auto_speed](https://github.com/Anonoei/klipper_auto_speed).)
2. **Stage 2 — validate.** The candidate is re-tested with the thorough
   reversal-stress pattern (random distances across the axis — where motors
   actually lose steps). Must pass, or back to stage 1 lower. The accepted
   accel is `BENCH_DERATE` × this value.
3. **Stage 3 — trim current** (needs a TMC driver; auto-skipped otherwise).
   Searches the **lowest `run_current` that still holds `(velocity, accel)`**,
   from the **ceiling** downward — a lower current means a **cooler, quieter
   motor**. The ceiling is the `[speed_test] max_current` cap when set,
   otherwise the stepper's configured `run_current`; the search and the stage-4
   bump **never exceed it**. It uses the same short, near-home jab moves, so a
   stall at low current barely grinds. The result is the lowest passing current
   plus `CURRENT_MARGIN`. Your configured `run_current` is restored at the end.
4. **Stage 4 — final benchmark.** A print-like run **at the trimmed current** —
   the real operating point: bursts of short infill zigzag + perimeter passes +
   travels, realistic lengths. For safety it stays in the **centre of the axis**
   and runs in **adaptive sections** that re-home between them and **abort on
   the first lost-step section**, so a stall can't grind the whole run into the
   limit. Sections start short (`BENCH_CHUNK` moves) and each clean one **grows
   the next by `BENCH_CHUNK_GROW`** (up to 8×), so checks loosen as the motor
   proves itself. If it fails, the **current was too low** for a sustained print
   → bump it up toward the configured value and retry; if it fails even at full
   current, the **accel is too high** → back to stage 1. The configured current
   is always restored afterwards; the accepted current is reported per velocity
   and saved in the CSV/HTML.

## Parameters

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
| `BENCH_SHORT`      | 400     | Stage-4 benchmark short **infill** segments  |
| `BENCH_LONG`       | 60      | Stage-4 benchmark long **travel** moves      |
| `BENCH_CHUNK`      | 40      | Stage-4 **initial** section length (moves) before re-home + skip-check |
| `BENCH_CHUNK_GROW` | 1.5     | Each clean section grows the next by this factor (capped at 8× the initial) — tight early, looser as the motor holds |
| `BENCH_DERATE`     | 0.9     | Accepted accel = this fraction of the stage-2 value — safer, fewer crashes |
| `MAX_REDO`         | 4       | Re-determination / current-bump attempts before a velocity is excluded |
| `FIND_CURRENT`     | 1       | Stage 3 on/off. `1` = trim current per point (needs TMC); `0` = skip |
| `MIN_CURRENT`      | 0.3     | Lower bound of the stage-3 current search (A) |
| `CURRENT_MARGIN`   | 0.1     | Trimmed current = lowest passing × (1 + this) |
| `CURRENT_ACCU`     | 0.05    | Stage-3 current search tolerance, *relative*  |
| `CURRENT_REPEAT`   | 10      | Jab moves per stage-3 current step           |
| `MAX_DIST_FACTOR`  | 4       | Upper bound for stage-2 random moves         |
| `SHORT_BIAS`       | 2       | Stage-2 short-move bias                       |
| `SEED`             | 12345   | Random seed for reproducible move sequences  |
| `TESTBENCH`        | config  | `1` = single-stepper bench mode (X only). Stage 4 also runs on the single axis |
| `NO_HTML`          | 0       | Set to 1 for CSV-only output                 |

## Output

The console prints the full table (velocity → max accel, and the stage-3 min
current) plus three ready-to-paste operating points — **balanced** (the knee),
**speed-priority** (highest velocity with its accel ceiling), and
**accel-priority** (lowest velocity with the highest accel) — each with a 10 %
margin on accel/velocity, and the knee's recommended `run_current`. The HTML
report draws the envelope curve with the knee highlighted, lists your **current
`printer.cfg` values** and the **TMC driver / run_current** side by side, adds a
**min-current** column, plus a free-text **toolhead-weight** field saved with
the report.

A velocity point is skipped if reaching it within the axis travel would need an
accel above the search ceiling (very short axes can't reach high speeds in a
triangle move) — the skip is reported so you know the curve has a gap.

> **Runtime:** stages 3 and 4 are thorough — hundreds of moves per accepted
> value, plus a fresh run on every re-determination, plus the per-point current
> search. For a quick first pass, lower the load, e.g.
> `V_POINTS=3 BENCH_SHORT=120 BENCH_LONG=20`, and add `FIND_CURRENT=0` to skip
> current trimming. Raising `BENCH_CHUNK_GROW` (e.g. 2.0) cuts re-homes further
> once the motor is proven.

## Examples

```
# Map the combined velocity/acceleration envelope (5 speeds, 100–500 mm/s)
SPEED_TEST_FIND_ENVELOPE AXIS=X

# Finer envelope: 8 speeds up to 800 mm/s
SPEED_TEST_FIND_ENVELOPE AXIS=Y V_MAX=800 V_POINTS=8

# Quick first pass, no current trim
SPEED_TEST_FIND_ENVELOPE AXIS=X V_POINTS=3 BENCH_SHORT=120 BENCH_LONG=20 FIND_CURRENT=0
```
