# Command reference

Three commands are exposed by the plugin.

## `TMC_FLOW_FIND_MAX`

Run the StallGuard-based flow test (Auto-SGT → Coarse → Bisection → Verification).

| Parameter | Default | Description |
|---|---|---|
| `START` | 10 | Starting flow (mm³/s) |
| `MAX` | 80 | Upper search bound |
| `COARSE_STEP` | 10 | Coarse sweep step size (use 5 for low-flow setups) |
| `REF_FLOW` | 15 (config `reference_flow`) | Fixed low reference flow. When `START` is well above it, two slip-free reference steps (`REF_FLOW` and `REF_FLOW`+5) run before the sweep and anchor the trigger baselines and Auto-SGT — so a high `START` can't blind the detection. `0` disables |
| `MIN_STEP` | 1 | Bisection precision |
| `DURATION` | 5 | Seconds per measurement |
| `REPEAT` | 5 | Repetitions per measurement |
| `VERIFY_REPEATS` | 5 | Repetitions in the verify phase |
| `COOLDOWN` | 15 | Pause between phases (seconds) |
| `PURGE` | 0 | Purge length (mm) before test |
| `MAX_BISECT_STEPS` | 6 | Max bisection iterations |
| `AUTO_SGT` | 1 | `1` = run Auto-SGT calibration before test (SG2 drivers only). `0` = skip |
| `SAVE_SGT` | 0 | `1` = keep the auto-tuned SGT after the test AND stage it as `driver_SGT` for `SAVE_CONFIG` (persisted in printer.cfg). Overrides `KEEP_SGT` |
| `KEEP_SGT` | 0 | `1` = leave the tuned SGT active until next FIRMWARE_RESTART. `0` = restore original after test |
| `FAN_SPEED` | config `test_fan_speed` (0) | Part-cooling fan speed in % (0–100), locked for the whole test and restored afterwards — fan speed materially changes the achievable max flow |
| `COLD_EXTRUSION_HINT` | 0 | Flow (mm³/s) you observed as under-extruding; drawn as a marker line in the HTML report. Report-only, the detection ignores it |
| `NO_HTML` | 0 | Set 1 to skip HTML report |
| `SKIP_TMC_CHECK` | 0 | Set 1 to bypass config validation |

### Examples

```
# Default (medium-flow setup)
TMC_FLOW_FIND_MAX

# High-flow setup (Goliath, Bondtech CHT 0.8)
TMC_FLOW_FIND_MAX MAX=150 START=25 COARSE_STEP=5

# Low-flow setup (V6 stock 0.4)
TMC_FLOW_FIND_MAX MAX=30 START=5 COARSE_STEP=5

# Keep the tuned SGT after test ends
TMC_FLOW_FIND_MAX MAX=150 START=10 COARSE_STEP=5 KEEP_SGT=1

# Skip Auto-SGT and use your configured SGT directly
TMC_FLOW_FIND_MAX MAX=150 START=10 AUTO_SGT=0

# Quicker, less accurate
TMC_FLOW_FIND_MAX REPEAT=3 VERIFY_REPEATS=3 COOLDOWN=10 COARSE_STEP=10

# More accurate (longer)
TMC_FLOW_FIND_MAX REPEAT=10 DURATION=8 VERIFY_REPEATS=10 COARSE_STEP=5
```

---

## `TMC_FLOW_STATUS`

Diagnostic check: reads current SG value, verifies driver, chopper mode, and StallGuard threshold. **Run this before the first test.**

| Parameter | Default | Description |
|---|---|---|
| `ACTIVATE` | 1 | Briefly run motor (1 mm extrusion) so SG can be read |

```
TMC_FLOW_STATUS
```

---

## `TMC_FLOW_GUI`

Writes the beginner-friendly **control panel** (`tmc_flow_gui.html`) into the
output folder, with your live driver/config values baked in. No parameters.
See **[GUI.md](GUI.md)**.

```
TMC_FLOW_GUI
```

(UI macro: `CRYD_FLOW_GUI`)

---

## `TMC_FLOW_SGT_CALIBRATE`

Standalone SGT tuning **with persistence**: runs the same probe-based
auto-tune the main test uses (anchored at a low, slip-free flow), keeps the
tuned value active and stages it as `driver_SGT` for Klipper's
`SAVE_CONFIG`. After running it, execute `SAVE_CONFIG` — the value is
written into printer.cfg's autosave block and survives every restart.
SG2 drivers only (TMC5160/2130/2240). Hotend must be at printing
temperature (it extrudes).

| Parameter | Default | Description |
|---|---|---|
| `FLOW` | config `reference_flow` (15) | Probe flow (mm³/s) the SGT is calibrated at |
| `SAVE` | 1 | `1` = stage for `SAVE_CONFIG`; `0` = session-only (until `FIRMWARE_RESTART`) |

```
TMC_FLOW_SGT_CALIBRATE
SAVE_CONFIG
```

(UI macro: `CRYD_FLOW_SGT_CALIBRATE`)

---

## `TMC_FLOW_TEST_SG_VARIANTS` *(TMC2209 only)*

Empirical pre-flight check: probes SG_RESULT in both StealthChop and SpreadCycle to determine which mode produces a usable slip signal on YOUR hardware. **Mandatory before relying on max-flow results from a TMC2209.**

| Parameter | Default | Description |
|---|---|---|
| `LOW_FLOW` | 5 | Low-load probe flow (mm³/s) |
| `HIGH_FLOW` | 20 | High-load probe flow (mm³/s) — set this near or above your hotend's expected slip point |
| `DURATION` | 5 | Seconds per probe |
| `REPEAT` | 5 | Repetitions per probe |
| `SGTHRS` | 100 | Temporary SGTHRS used for the test (your config is restored after) |

```
TMC_FLOW_TEST_SG_VARIANTS LOW_FLOW=30 HIGH_FLOW=140 DURATION=8
```

See [ADVANCED.md → TMC2209 pre-flight check](ADVANCED.md#tmc2209-pre-flight-check) for what to do with the output.
