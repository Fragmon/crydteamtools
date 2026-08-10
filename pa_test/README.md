# PA Test (prototype)

**Pressure-advance calibration via StallGuard — no camera, no calipers.**

Part of **[Crydteam Tools](../README.md)** · by Steven (Fragmon) — Crydteam ·
[![YouTube](https://img.shields.io/badge/YouTube-@crydteamprinting-red?logo=youtube)](https://www.youtube.com/@crydteamprinting)

---

## The idea

Pressure advance compensates the melt-pressure build-up in the nozzle.
That pressure loads the extruder motor — and motor load is exactly what
StallGuard measures (proven by the [TMC Flow Test](../max_flow_test/README.md)).

PA errors are classic step-response errors:

- **PA too low** — pressure lags after a speed jump (undershoot → thin
  extrusion after corners)
- **PA too high** — pressure overshoots and relaxes (blobs/bulge after
  accelerations)
- **PA right** — pressure snaps to the new steady level

The end goal: measure the pressure step response per candidate PA value,
bisect to the optimum — at **multiple flow rates** — and emit a ready-made
**adaptive pressure-advance table for OrcaSlicer** (PA vs. flow), which is
what sub-3-minute Benchys at up to 120 mm³/s actually need.

## Control panel (GUI)

Run **`PA_TEST_GUI`** (or the `PT_GUI` macro) — it writes `pa_test_gui.html`
into the PAtest folder: a live checklist (driver, hotend temperature, output
path), presets for the flow gap, an estimate of runtime *and filament
consumption*, plus an explanation of how to read the verdict. Open it from
Mainsail/Fluidd's file browser in any browser.

## Current state: feasibility probe

This prototype ships stage 0 — `PA_TEST_PROBE` steps the extrusion rate
between two flows (pure E-moves, PA not involved yet) and records the SG
step response at 50 Hz:

```
PA_TEST_PROBE [FLOW_LOW=10] [FLOW_HIGH=60] [DWELL=1.0] [CYCLES=6]
```

(UI macro: `PT_PROBE`)

Heat the hotend to printing temperature first and let it extrude into
free air (over the purge bucket / bed edge). Console output reports the
SG step size, noise floor, SNR, median rise time and overshoot; a CSV
with the full time series lands in `output_dir` for offline analysis.

**Verdict line:** SNR ≥ 5 means the signal is good enough to build the
full PA search on. If it's lower, widen the flow spread
(e.g. `FLOW_LOW=10 FLOW_HIGH=100`).

## Requirements

Same as the TMC Flow Test: the extruder's TMC driver needs working
StallGuard readback (TMC2130/2209/2240/5160/2660 — see the
[flow-test docs](../max_flow_test/README.md) for chopper mode, SGT and
`coolstep_threshold`).

## Install

```bash
cd ~/crydteamtools && ./install.sh pa_test
```

Then uncomment `[pa_test]` in `pa_test_settings.cfg` and
`FIRMWARE_RESTART`.

## Roadmap

1. ✅ Feasibility probe (`PA_TEST_PROBE`) + control panel (`PA_TEST_GUI`)
2. Step-response scoring per PA candidate (real XY moves with speed
   jumps over a purge line, so PA is actually active)
3. PA bisection per flow rate + smooth-time tuning
4. Adaptive-PA table export for OrcaSlicer + HTML report

---

Released under the GNU General Public License v3.0.
