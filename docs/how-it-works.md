# How it works

[← back to README](../README.md)

## Skipped-step detection

The plugin reads the stepper's **MCU step position** directly from the
kinematics (`get_mcu_position`) right after each re-home, and compares it with
the position before the test move. A shift larger than `max_missed` full steps
means the motor lost steps.

Reading the position directly means **no `[endstop_phase]` module is needed** —
and crucially, it can't be present: `endstop_phase` cross-checks the TMC
microstep phase and aborts homing with `incorrect phase` the instant a step is
lost, which is exactly the event this test is built to measure.

About `max_missed`: without `endstop_phase`'s phase-snapping, a home can land up
to ~1 full step off purely from mechanical jitter, so the default tolerance of
1.5 full steps leaves headroom. A real stall loses tens to thousands of
microsteps — far above the threshold.

## Why adaptive bisection?

A linear sweep from 100 → 800 mm/s with a 10 mm/s step takes **70
measurements**. The limit map's relative-accuracy binary search converges in
**~6–10 measurements** per velocity. For longer per-move durations or noisy
environments, that difference is 20+ minutes.

## Optional TMC monitoring

If you have TMC drivers on X/Y with StallGuard support, the plugin polls SG
values at 20 Hz during each move. Lower SG = higher load. Even on tests that
pass, seeing SG drop a lot tells you the motor was *close* to slipping — useful
margin information for the slicer.

Supported drivers (auto-detected): TMC2240, TMC2209, TMC5160, TMC2130, TMC2660,
TMC2226, TMC2208.

## Cruise-aware sizing

(applies to `SPEED_TEST_FIND_MAX_VELOCITY`)

A short axis combined with a high target velocity can give misleading results:
the motor briefly touches the target speed at the peak of a triangle profile and
immediately decelerates again, never actually *cruising* there.

The plugin avoids that by sizing motion so that **at least half of every move**
is spent at the target velocity (`CRUISE_RATIO` default 0.5). Two modes:

**Auto-acceleration (`ACCEL` omitted or `0`)** — recommended. The plugin
computes:

```
ACCEL ≥ MAX_V² / (axis_range × (1 − CRUISE_RATIO))
```

and rounds up to the nearest 500 mm/s². You'll see a line like:

```
Auto-set ACCEL = 26000 mm/s² so MAX=2000 mm/s has ≥50% cruise on 310 mm of usable X travel.
```

**Fixed acceleration (`ACCEL=…` given)** — useful for chasing motor limits. The
plugin clips `MAX` down to the velocity where the cruise ratio still holds:

```
MAX = √(axis_range × ACCEL × (1 − CRUISE_RATIO))
```

```
MAX=2000 mm/s exceeds the velocity that keeps ≥50% cruise at ACCEL=5000 …
Clipped MAX to 880 mm/s.
To test higher, increase ACCEL or omit it for auto-sizing.
```

The achieved cruise fraction is reported per measurement (e.g. `cruise=58%`) and
saved in the CSV/HTML report so you can verify each step was a fair test.

`CRUISE_RATIO=0` reverts to the old behaviour (triangle profile, no cruise
required). `CRUISE_RATIO=0.75` is stricter — useful when comparing motors under
more realistic load.

## Does every test reach the target velocity?

Yes for the limit-**finding** moves: stage-1 jab and stage-2 reversal moves are
sized to at least `V²/A`, so they reach the target velocity. The stage-4
simulated print deliberately uses **realistic segment lengths** (short infill
that never reaches top speed) — exactly like a real print — so most of its moves
do *not* reach the target velocity. That's by design: it validates the operating
point under realistic load, not a synthetic best case.
