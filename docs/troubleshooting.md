# Troubleshooting

[← back to README](../README.md)

## "Endstop stepper_x incorrect phase (got … vs …)"

`[endstop_phase]` is loaded. It aborts homing the moment the motor loses a step
— which the test does on purpose. **Remove `[endstop_phase]` and any
`[endstop_phase stepper_*]` from `printer.cfg` and `FIRMWARE_RESTART`.** The
plugin detects skips without it. `SPEED_TEST_STATUS` warns when it's still
present.

## Tests always pass even at obviously-too-high values

- Check that your endstop is a physical switch that homes repeatably (sensorless
  homing gives more jitter — raise `max_missed` only if you see false
  *positives*, not to mask false negatives)
- If homing isn't repeatable, the position reference drifts; verify `G28` lands
  consistently
- Lower `ACCEL_ACCU` (e.g. 0.02) so the envelope search resolves the limit more
  finely

## Test triggers immediately at MIN

`MIN` is already past your motor's limit. Lower it and reduce `COARSE_STEP`:

```
SPEED_TEST_FIND_MAX_VELOCITY AXIS=X MIN=20 COARSE_STEP=10
```

## "Need X mm of axis range for SPEED=Y at MIN=Z"

The accel test needs enough distance to actually reach `SPEED` while
accelerating at `MIN`. Either raise `MIN` or lower `SPEED`.

## Stage 3/4 of the envelope keeps failing / takes too long

- The simulated print is the binding test. If a velocity is excluded, the motor
  genuinely can't sustain that accel under a realistic print at the allowed
  current.
- To speed up a first pass: `V_POINTS=3 BENCH_SHORT=120 BENCH_LONG=20
  FIND_CURRENT=0`.
- If stalls are violent, lower `BENCH_DERATE` (e.g. 0.8) and `BENCH_CHUNK`
  (e.g. 20).

## Results vary between runs

- Increase `REPEAT` (e.g. 50) and `VERIFY_REPEATS` (e.g. 100) for accel tests —
  more cycles = less luck-dependent
- Check belt tension and pulley setscrews; loose mechanics give random skips
- If TMC monitoring shows SG dropping to near 0 even on passes, the motor is
  right at its torque limit — consider increasing `run_current`
