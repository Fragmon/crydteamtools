# Motor Sync

**Synchronize the two motors of a dual-motor axis — without an accelerometer.**

Part of **[Crydteam Tools](../README.md)** · by Steven (Fragmon) — Crydteam ·
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## The problem

On axes driven by two motors (AWD CoreXY, dual-Y gantries, `[stepper_x]` +
`[stepper_x1]` setups), the motors can end up **out of phase** after every
power-on: each rotor snaps to its nearest detent, and the electrical phase
relationship between the two drivers is random within ±2 full steps. The
motors then permanently fight each other — extra heat, lost torque, worse
print quality.

The excellent [motors-sync](https://github.com/MRX8024/motors-sync) project
by Maksim Bolgov (MRX8024) solves this by measuring the enable-snap impact
with an **accelerometer** (ADXL) or an encoder.

**This plugin needs neither.** Two motors that fight each other show the
extra load in their TMC drivers' **StallGuard** signal. Motor Sync measures
SG of both drivers during a slow constant move and shifts one motor in
microsteps until the combined SG is maximal — which is exactly the point of
minimal fight, i.e. sync.

## Credits

- Sync approach (single-motor microstep correction, hill-descent with
  direction probing and step-size refinement) adapted from
  **[motors-sync](https://github.com/MRX8024/motors-sync/blob/main/motors_sync.py)**
  by **Maksim Bolgov (MRX8024)**, GPLv3.
- StallGuard measurement methodology (50 Hz register sampling, transient
  trim, median statistics) shared with the other Crydteam Tools plugins.

## Requirements

- A dual-motor axis: `[stepper_x]` + `[stepper_x1]` (or Y/Z equivalents).
- Both motors on TMC drivers **with StallGuard readback**:
  TMC2130 / TMC2209 / TMC2240 / TMC5160 / TMC2660.
  **TMC2208/2225 have no StallGuard and cannot work.**
- SG2 drivers (2130/2240/5160/2660) need **SpreadCycle**
  (no `stealthchop_threshold`, or `stealthchop_threshold: 0`); the
  TMC2209 needs **StealthChop** (`stealthchop_threshold: 999999`).
- Enough rotation speed: StallGuard reads 0 below roughly
  **1.5 motor revolutions/s**. With a typical `rotation_distance` of
  40 mm the default `buzz_speed` of 100 mm/s is fine; raise it if
  `MOTOR_SYNC_STATUS` or the error message says the move is too slow.

## Install

```bash
cd ~
git clone https://github.com/Fragmon/crydteamtools.git
cd crydteamtools
./install.sh motor_sync
```

The installer drops a fully commented `motor_sync_settings.cfg` into your
config directory and includes it at the top of `printer.cfg` — uncomment
the `[motor_sync]` line in that file, then `FIRMWARE_RESTART`.

## Usage

```
MOTOR_SYNC [AXIS=X] [BUZZ_SPEED=100] [BUZZ_DIST=40] [REPEATS=2]
           [COARSE=4] [MAX_OFFSET=2.0] [MIN_GAIN=4]
MOTOR_SYNC_STATUS
```

(or the UI macros `MS_SYNC` / `MS_STATUS`)

The axis is homed if needed, then the plugin measures a StallGuard
baseline, probes the correction direction and walks one motor in
microsteps (coarse → fine) until the SG score stops improving:

```
──── MOTOR_SYNC X ────
  motors: stepper_x (tmc5160) + stepper_x1 (tmc5160) | microsteps=16
  baseline: SG score = 529.9 (stepper_x=265.0, stepper_x1=265.0) — higher = less fight
  +4 msteps (total +4): score 617.3 → improved
  +4 msteps (total +8): score 713.4 → improved
  ...
  applied offset: +13/16 msteps on stepper_x1 (0.1625 mm)
  SG score: 529.9 → 839.6
```

**The correction is lost whenever the motors are disabled** (M84 /
power-off) — same as with any phase-sync tool. Run `MOTOR_SYNC` after
every power-on, e.g. at the start of `PRINT_START`.

## How accurate is it?

StallGuard resolves the fight load clearly down to ~1–2 microsteps of
residual offset (1 full step = 90° electrical; even 4/16 microsteps of
mismatch cost ~40 % of the holding torque). An accelerometer-based sync
can go a bit finer — but SG-based sync needs zero extra hardware and no
toolhead board.

A note on the ±2-full-step cap (`max_offset_fullsteps`): the electrical
period of a stepper is 4 full steps, so a desync can never legitimately
exceed half a period. The cap keeps the search inside that window.

## Limitations

- Exactly **2 motors per axis** in this version (no 3/4-motor Z).
- No offset persistence across restarts (run it after power-on instead).
- TMC2208/2225 unsupported (no StallGuard).

---

Released under the GNU General Public License v3.0.
