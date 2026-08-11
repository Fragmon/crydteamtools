# Chopper Test

**Tune the TMC chopper registers for more torque and less heat — without an
accelerometer.**

Part of **[Crydteam Tools](../README.md)** · by Steven (Fragmon) — Crydteam ·
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## The idea

The chopper registers (`TOFF`, `TBL`, `HSTRT`, `HEND`, `TPFD`) decide how the
driver regulates coil current. Badly matched values waste energy as heat,
make the motor sing and cost usable torque.

The excellent
**[chopper-resonance-tuner](https://github.com/MRX8024/chopper-resonance-tuner)**
by Maksim Bolgov (MRX8024) sweeps exactly these registers and scores each
combination by the **vibration an ADXL accelerometer measures**.

This plugin scores the **outcome** instead, so it needs no extra hardware:

- **Torque** — can the motor still hold a hard reversal-stress move without
  losing steps? Measured by comparing the stepper's MCU step counter across a
  re-home, the same method the [Speed Test](../speed_test/README.md) uses.
- **Temperature** — how warm does the motor get during a fixed run?

### Why not StallGuard?

It looks like the obvious sensor, and it is a trap: `TBL` and `TOFF` define
the very chopper window StallGuard samples in. Comparing SG readings across
different `TBL`/`TOFF` values measures the measurement, not the motor. (This
is also why SGT has to be re-tuned after chopper changes.)

## Credits

Register set and the staged sweep order (TOFF → TBL → hysteresis → TPFD) are
adapted from **[chopper-resonance-tuner](https://github.com/MRX8024/chopper-resonance-tuner)**
by **Maksim Bolgov (MRX8024)**, GPLv3. The measurement method is different
and originates here.

## Requirements

- A TMC driver with runtime register access: TMC2130 / 2209 / 2240 / 5160.
- **SpreadCycle at the test speed.** The chopper registers do nothing in
  StealthChop, so the plugin checks the real driver state and refuses rather
  than produce meaningless numbers. Note that `stealthchop_threshold: 0` in
  Klipper does *not* mean "no StealthChop": it enables StealthChop but pins
  its velocity limit to the minimum, so the motor uses StealthChop at
  standstill and SpreadCycle for every real move — that setup is tunable, and
  `CHOPPER_TEST_STATUS` tells you the exact threshold speed. Only
  `stealthchop_threshold: 999999` (StealthChop at every speed) is refused.
- For the thermal stage: a temperature sensor on the motor
  (`motor_sensor:` in `[chopper_test]`) or a TMC2240, which reports its own
  die temperature. Without either, the stage is skipped.

## Install

```bash
cd ~/crydteamtools && ./install.sh chopper_test
```

Then uncomment `[chopper_test]` in `chopper_test_settings.cfg` and
`FIRMWARE_RESTART`.

## Usage

```
CHOPPER_TEST_STATUS [AXIS=X]      # check first: driver, mode, registers
CHOPPER_TUNE [AXIS=X] [VELOCITY=200] [ACCEL=0] [REPEAT=8]
             [TPFD=0] [THERMAL=1] [THERMAL_SECONDS=90]
```

(UI macros `CT_STATUS` / `CT_TUNE`)

| Parameter | Default | Meaning |
|---|---|---|
| `AXIS` | config `default_axis` | Which axis/motor to tune (X or Y) |
| `VELOCITY` | 200 | Speed of the stress move |
| `ACCEL` | 0 = auto | Starting acceleration for the screening. It rises automatically until the settings separate |
| `REPEAT` | 8 | Out-and-back moves per candidate |
| `TPFD` | 0 | `1` also sweeps TPFD (TMC5160/2240 only) |
| `THERMAL` | 1 | `0` skips the temperature comparison |

### How a run proceeds

1. **TOFF** (chopper off time) is swept first — it dominates the chopper
   frequency.
2. **TBL** (comparator blank time) at the winning TOFF.
3. **HSTRT / HEND** as a pair, because only their *sum* is constrained and
   they shape the same regulation window.
4. **TPFD** (optional, TMC5160/2240 only).
5. **Thermal comparison**: the original settings and the tuned ones each run
   the same movement for the same time, with a cool-down between, and the
   temperature rise is reported.

Whenever more than one candidate survives a pass, the acceleration is raised
by 15 % and only the survivors are re-measured — so the test auto-calibrates
to the point where your motor actually discriminates, instead of guessing a
threshold. A single failure is always re-tested once before it counts.

Roughly 30–40 measurements, about 7–12 minutes plus the thermal stage.

## Safety

- Every register is restored to its original value when the run ends —
  including on errors and aborts. **Nothing is written to your config**; the
  console prints the `driver_*` lines to add if you want to keep the result.
- The toolhead is lifted to `z_pos` before any test motion, and the live
  motion limits (including `minimum_cruise_ratio`) are snapshotted and put
  back afterwards.
- When a pass cannot separate its candidates even at the highest
  acceleration, it is reported as **inconclusive** and your existing value is
  kept — the tool never invents a winner from sweep order.
- `TOFF=0` (which disables all bridges) and `TOFF=1` (only legal with
  `TBL≥2`) are excluded from the sweep.
- The datasheet rule **`HSTRT + HEND ≤ 18`** (register units; effective
  `HEND+HSTRT ≤ 16`) is enforced — and writes are ordered *decreases first*,
  so no intermediate state can violate it either.
- `vsense`, `chm`, `vhighchm`, `vhighfs`, `fd3` and `disfdcc` are never
  touched: `vsense` would change the real motor current by ~2× behind
  Klipper's back, `chm` remaps the meaning of HSTRT/HEND, and the rest are
  inert or dcStep-only.

## Limitations

- This measures torque and heat, **not** acoustics. If your goal is a
  *quieter* machine, an accelerometer-based tuner measures that directly.
- The result is specific to this motor, driver, voltage and current —
  manufacturing tolerances are ±20 %, so do not copy values between machines.
- Single-motor axes only in this version.

---

Released under the GNU General Public License v3.0.
