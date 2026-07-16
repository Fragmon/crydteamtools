# Configuration

[← back to README](../README.md)

## `[speed_test]` section

```ini
[speed_test]
structure: corexy             # cartesian | corexy
default_axis: X               # X or Y — used when AXIS is not given
margin: 20                    # mm to keep from each axis end
z_pos: 20                     # Z height during XY tests
monitor_tmc: True             # poll TMC StallGuard during moves
testbench: False              # see "Testbench mode" below
start_offset: 0               # mm from the endstop where the search probes
                              # start. 0 = automatic (20% of usable travel).
travel_speed: 100             # mm/s for positioning moves to a test's start
travel_accel: 3000            # mm/s^2 for those positioning moves
max_missed: 1.5               # skip tolerance, in FULL motor steps. A move
                              # counts as a skip when the stepper drifts more
                              # than this across a re-home. ~1 step of homing
                              # jitter is normal; real stalls lose far more.
max_current: 1.5              # hard safety cap (A). Upper bound for
                              # OPTIMAL_CURRENT and the limit map's stage-3
                              # current trim. 0 (default) = no cap (use the
                              # stepper's configured run_current instead).
#output_dir: ~/printer_data/config/Speedtest
```

After `FIRMWARE_RESTART`, run `SPEED_TEST_STATUS` to verify the plugin loaded
and your axes are recognised.

## UI macros (Mainsail / Fluidd)

The repo ships `speed_test_macros.cfg` with one `[gcode_macro]` per test
(`ST_FIND_LIMITS`, `ST_FIND_MAX_VELOCITY`, `ST_FIND_MAX_ACCEL`,
`ST_FIND_MAX_SCV`, `ST_FIND_OPTIMAL_CURRENT`, `ST_BENCHMARK`, `ST_STATUS`,
`ST_GUI`).
They show up in the web UI's macro panel **with input fields for every
parameter**; empty fields are not passed on, so the plugin defaults and your
`[speed_test]` config stay in charge. `install.sh` links the file into
`~/printer_data/config/`; enable it with:

```ini
[include speed_test_macros.cfg]
```

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
plugin reads the X stepper position directly.
