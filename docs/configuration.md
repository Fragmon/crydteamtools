# Configuration

[← back to README](../README.md)

## Do NOT enable `endstop_phase`

Skipped-step detection reads the stepper's MCU step position directly after
each re-home — **no `[endstop_phase]` is needed**. If `[endstop_phase]` (or
any `[endstop_phase stepper_*]`) is configured, **remove it** and
`FIRMWARE_RESTART`: its TMC phase cross-check raises `Endstop … incorrect
phase` and aborts homing exactly when the motor loses a step — which is the
event the test is built to detect. `SPEED_TEST_STATUS` warns if it's loaded.

## `[speed_test]` section

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
max_current: 1.5              # hard safety cap (A). Upper bound for
                              # OPTIMAL_CURRENT and the envelope's stage-3
                              # current trim. 0 (default) = no cap (use the
                              # stepper's configured run_current instead).
#output_dir: ~/printer_data/config/Speedtest
```

After `FIRMWARE_RESTART`, run `SPEED_TEST_STATUS` to verify the plugin loaded
and your axes are recognised.

## Testbench mode

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
