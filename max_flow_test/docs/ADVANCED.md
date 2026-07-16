# Advanced documentation

This document covers everything that's not in the [README](../README.md):
detailed driver setup, the TMC2209 experimental path, configuration
details, troubleshooting beyond the basics, and tuning notes.

## Table of contents

- [Configuration](#configuration)
  - [TMC5160 (and TMC2130 / TMC2660)](#tmc5160-and-tmc2130--tmc2660)
  - [TMC2240](#tmc2240)
  - [TMC2209 (experimental)](#tmc2209-experimental)
  - [Important driver settings](#important-driver-settings)
- [Chopper mode](#chopper-mode)
- [Auto-SGT calibration](#auto-sgt-calibration)
- [Physical setup before running](#physical-setup-before-running)
- [TMC2209 pre-flight check](#tmc2209-pre-flight-check)
- [Per-driver tuning](#per-driver-tuning)
- [Troubleshooting](#troubleshooting)
- [Compatibility](#compatibility)

---

## Configuration

The plugin needs a `[tmc_flow_test]` section in `printer.cfg` plus
specific settings on your existing `[tmcXXXX extruder]` section.
**Run the test with the same chopper-mode and current you use for
printing.**

> ⚠️ Lines marked `# ADAPT` depend on your hardware. The other lines
> are required by the plugin.

### TMC5160 (and TMC2130 / TMC2660)

```ini
[tmc5160 extruder]
cs_pin: PA15                     # ADAPT
spi_bus: spi4                    # ADAPT
run_current: 0.85                # ADAPT
hold_current: 0.6                # ADAPT
sense_resistor: 0.075            # ADAPT
interpolate: false
# DO NOT add stealthchop_threshold — see "Chopper mode" below
coolstep_threshold: 0.5          # required for StallGuard reads
driver_SGT: 15                   # SG2 sensitivity (Auto-SGT will tune this)
driver_SFILT: 1                  # SG2 filter — REQUIRED for clean signal
```

### TMC2240

The plugin runs TMC2240 in **SG2/SpreadCycle mode** — same path as TMC5160. This gives ~50 % more peak torque than the SG4/StealthChop path.

```ini
[tmc2240 extruder]
cs_pin: PA15                     # ADAPT
spi_bus: spi4                    # ADAPT
rref: 12300                      # ADAPT (12000-60000 depending on hardware)
run_current: 0.85                # ADAPT
hold_current: 0.6                # ADAPT
interpolate: false
# DO NOT add stealthchop_threshold — SG2 needs SpreadCycle
coolstep_threshold: 0.5          # required for StallGuard reads
driver_SGT: 15                   # SG2 sensitivity (Auto-SGT will tune this)
driver_SFILT: 1                  # SG2 filter — REQUIRED for clean signal
```

> **Switching from SG4 to SG2 on TMC2240?** If you previously ran the TMC2240 with `stealthchop_threshold: 999999` and a `SET_TMC_FIELD ... sg4_thrs ...` macro for sensorless homing, you'll need to disable that for the test. The plugin will detect StealthChop in `TMC_FLOW_STATUS` and tell you what to remove.

### TMC2209 (experimental)

> ⚠️ The TMC2209 only implements **StallGuard4 (SG4)**, which Trinamic
> explicitly designed for StealthChop mode. The plugin runs it in
> **SpreadCycle anyway** to access full motor torque (~100 % vs. ~50 %
> in StealthChop). This is **outside official Trinamic specification**
> and works empirically on some hardware combinations but not others.

From the [TMC2209 Datasheet Rev 1.09](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2209_datasheet_rev1.09.pdf):

> *"SG_RESULT becomes updated with each fullstep [...] **Intended for StealthChop mode, only.**"*

**What this means for you:**

- Results are **not** guaranteed reliable across all TMC2209 boards/motors
- Some TMC2209 boards (especially budget clones) produce unusable SG4 in SpreadCycle
- The plugin uses CV-spike detection (variance jump at slip) instead of SG-magnitude — works when SG4 is responsive, fails silently when it isn't
- You **must** run `TMC_FLOW_TEST_SG_VARIANTS` first to verify your hardware (see [TMC2209 pre-flight check](#tmc2209-pre-flight-check))
- Validate any max-flow result with **multiple long real-world prints** before trusting it in your slicer

If your TMC2209 doesn't pass the pre-flight check, swap to a pin-compatible TMC2240 (~€15) for guaranteed results, or use a traditional flow-tower print test.

```ini
[tmc2209 extruder]
uart_pin: PB12                   # ADAPT
run_current: 0.85                # ADAPT — see notes below
hold_current: 0.5                # keep modest to avoid overheating
sense_resistor: 0.110            # ADAPT (typical 0.110 on most boards)
interpolate: false
# stealthchop_threshold INTENTIONALLY OMITTED — see notes below
coolstep_threshold: 0.5          # required for StallGuard reads
driver_SGTHRS: 100               # SG4 threshold
driver_SEMIN: 0                  # CoolStep off — clean SG signal
```

**Why no `stealthchop_threshold`?** Omitting it leaves the TMC2209 in **SpreadCycle** mode (the chip's default after Klipper init). This is the experimental setup. **Always verify with `TMC_FLOW_TEST_SG_VARIANTS` first.** If your hardware fails the pre-flight check, fall back to:
```ini
stealthchop_threshold: 999999    # forces StealthChop — Trinamic-supported but ~50 % torque loss
```

**Why higher `run_current`?** TMC2209 typically runs lower currents (0.5-0.7 A) for silent operation. For max-flow testing you want full torque — set 0.85-1.0 A. Watch motor temperature; reduce `hold_current` if the motor heats up during dwell.

**Why `driver_SEMIN: 0`?** Disables CoolStep, which would otherwise modulate motor current during the test and noise up the SG signal.

> **Auto-SGT skipped for TMC2209**: The Auto-SGT calibration phase is only meaningful for SG2 drivers (TMC5160/2130/2240). On TMC2209, `driver_SGTHRS` is the DIAG-pin trigger threshold and doesn't affect SG_RESULT magnitude (per Trinamic spec). The plugin uses your configured value directly.

### Important driver settings

#### SFILT — keep it on

`driver_SFILT: 1` enables the StallGuard hardware filter (averages SG over 4 cycles). The plugin's slip detection thresholds are **calibrated against filtered SG signal** — running with `driver_SFILT: 0` produces noisy per-sample variance that triggers false positives in the coarse phase.

If you've previously run the test with `SFILT=0` and got early triggers (e.g. at flow=50), set `driver_SFILT: 1`, restart Klipper, and re-test.

#### CoolStep — what about it?

CoolStep is the TMC feature that dynamically reduces motor current under low load. **The plugin does not require CoolStep to be on or off** — slip detection uses StallGuard signal directly.

For the **most conservative max-flow result**, set `driver_SEMIN: 0` (CoolStep off). With CoolStep off, the motor runs at constant `run_current` — the test then reflects what the motor can sustain at that exact current, which matches what happens during high-load printing.

If you leave CoolStep on, the test still works, but CS-driven current changes during the sweep can dampen the StallGuard signal slightly and produce a marginally higher (less conservative) result.

---

## Chopper mode

Each TMC chip family supports StallGuard only in one specific chopper mode:

| Driver | StallGuard variant | Required mode |
|---|---|---|
| TMC5160 / TMC2130 / TMC2660 | SG2 | **SpreadCycle** (Klipper default — **don't add `stealthchop_threshold`**) |
| TMC2240 | SG2 *(used by this plugin)* | **SpreadCycle** (don't add `stealthchop_threshold`) |
| TMC2209 | SG4 | **SpreadCycle** (experimental) or StealthChop (lower torque) |

> **`stealthchop_threshold: 0` is NOT the same as "no line".** It enables StealthChop with threshold 0, which breaks SG2. For SG2 drivers, **remove the line entirely**.

If you're not sure what mode you're in, run `TMC_FLOW_STATUS` — the plugin checks the configuration and tells you what's wrong. Or `DUMP_TMC STEPPER=extruder` and check `en_pwm_mode` (1 = StealthChop) and `tpwmthrs` (1048575 = pure SpreadCycle).

> **TMC2209 flag note:** TMC2209 inverts the StealthChop bit semantics. Its `en_spreadcycle` GCONF bit reads as 1 when SpreadCycle is active — opposite of the SG2 chips' `en_pwm_mode`. The plugin handles this internally.

---

## Auto-SGT calibration

The biggest factor in StallGuard accuracy is the `driver_SGT` setting. Set it too high and SG saturates at 1023 (no useful range). Set it too low and SG hits the noise floor before the motor actually slips.

The plugin's **Auto-SGT** phase (on by default) handles this for you:

1. Reads your current `driver_SGT` value
2. Probes SG_RESULT at the test's `START` flow (5 reps × 5 s, like the main test)
3. If saturation is detected (any sample at 1023) → lowers SGT
4. If SG is too low (median < 600) → raises SGT
5. Iterates until SG sits in the healthy 600–1022 range
6. Runs the actual flow test with the tuned SGT
7. **Restores your original SGT after the test** (unless `KEEP_SGT=1` is passed)

The console output recommends the tuned value for permanent inclusion in `printer.cfg`:

```
Auto-SGT: tuned to SGT=13 (was 18).
→ For a permanent fix, add to your [tmc5160 extruder] section:
    driver_SGT: 13
```

To skip Auto-SGT entirely (use your config value as-is), pass `AUTO_SGT=0`.

---

## Physical setup before running

The test extrudes **a lot** of filament — at high flow rates the motor pushes around 30 mm/sec linear feed for 5 seconds at a time, with dozens of repetitions across coarse, bisection, and verification phases. Total extruded filament can easily be **2–5 meters** over the full test run.

**Before running the test, move the toolhead so molten extrusion has room to fall away cleanly.** Two good options:

**Option A — Above the bed:** Move the toolhead to a Z-height of at least 50–80 mm above the bed, ideally near the centre.

```
G28
G1 Z80 F600
G1 X100 Y100 F3000
M109 S230
```

**Option B — Off the bed entirely (recommended if printer geometry allows):** Park the toolhead over a purge bucket, drop chute, or just past the bed edge.

```
G28
G1 X<purge_x> Y<purge_y> Z50 F3000
M109 S230
```

### Pre-flight checklist

- [ ] Hotend at printing temperature
- [ ] Filament loaded and the path from spool to extruder is clear
- [ ] Spool turns freely
- [ ] At least 1–2 meters of filament available
- [ ] Toolhead position: high above bed, or parked over an open area
- [ ] Nothing fragile near the nozzle (cables, fans, BL-Touch probes)

### What to expect

You'll see filament extrude in chunks every few seconds. Some "spaghetti" buildup near the nozzle is normal. **Clacking or grinding from the extruder at high flow rates is expected and intentional** — it's how the plugin finds your real limit. Only stop the test (`M112`) if you hear something else (spool jamming, thermistor disconnect, nozzle scraping bed).

---

## TMC2209 pre-flight check

> ⚠️ **Mandatory step before TMC2209 max-flow tests.** TMC2209 SG4 in SpreadCycle is unsupported per Trinamic spec. Whether it works on YOUR hardware is empirical — this command tells you.

```
TMC_FLOW_TEST_SG_VARIANTS LOW_FLOW=30 HIGH_FLOW=140 DURATION=8
```

The check probes SG_RESULT in **both** StealthChop and SpreadCycle at low flow and high flow, then evaluates whether the variance signature (CV jump from low-load to slip) is detectable.

> ⚠️ At HIGH_FLOW values close to your hotend's limit, the motor will physically slip during the test. This is intentional. Be ready to stop with `M112` if you hear excessive clacking, and ensure your filament path is clear.

### What "USABLE" means

The plugin reports each mode as USABLE or NOT USABLE based on:

- **CV-spike detected** (CV ratio ≥ 3× between low and high flow, with high CV ≥ 10 %) → real slip will trigger detection
- **Large SG-magnitude change** (|delta| ≥ 50) → magnitude-based triggers can also work as a backup

### Three typical outcomes

**Both modes USABLE, SpreadCycle stronger** — best case. SpreadCycle gives full torque AND a clean slip signal. Omit `stealthchop_threshold` from your config.

**Only StealthChop USABLE** — your hardware doesn't produce reliable SG4 in SpreadCycle. Add `stealthchop_threshold: 999999` to your config and accept ~50 % torque loss.

**Neither mode USABLE** — your TMC2209 hardware can't produce a usable SG4 signal for this purpose. Try increasing `run_current`, increase HIGH_FLOW so the motor actually reaches stall, or accept that this hardware combo isn't suitable. As a last resort, swap to TMC2240 or use a flow-tower print.

---

## Per-driver tuning

Each driver family has its own `TriggerProfile` in the source code with all detection thresholds. To tune sensitivity for one driver without affecting the others, edit only its profile class.

- **TMC5160Profile** — validated production baseline (SGT=15, SFILT=1)
- **TMC2240Profile** — inherits TMC5160 base, with TMC2240-specific overrides:
  - `WARMUP_DRIFT_THRESHOLD = 0.04` — catches the systematic first-run drift
  - `PLATEAU_RATIO = 0.2` — the SG2 saturation curve is steeper on TMC2240
- **TMC2209Profile** *(experimental)* — fundamentally different detection strategy from SG2:
  - SG-magnitude triggers (snap-back, plateau, max-spike) **DISABLED** — SG4 doesn't follow the SG2 "smooth load curve" model
  - CV-spike triggers as primary slip indicator (`CV_HIGH_VARIANCE = 12.0` vs. 5.0 for SG2)
  - IQR triggers calibrated for the 0-510 SG4 scale (`IQR_ABSOLUTE_TRIGGER = 50` vs. 25 for SG2)
  - `WARMUP_DRIFT_THRESHOLD = 0.15` — SG4 has higher first-run drift

---

## Troubleshooting

**`Section 'tmc_flow_test' is not a valid config section`** —
The plugin file isn't being loaded. Common causes:
1. File at wrong path → must be `~/klipper/klippy/extras/tmc_flow_test.py` (or symlink to it)
2. Service not restarted → run `sudo systemctl restart klipper` (NOT `FIRMWARE_RESTART`)
3. Stale Python cache → `rm -f ~/klipper/klippy/extras/__pycache__/tmc_flow_test*.pyc` then restart
4. **Kalico**: the plugin works around Kalico's `extras/statistics.py` shadowing — make sure you have the latest version (no `import statistics` at top of file)

**Plugin throws `ImportError: attempted relative import with no known parent package`** —
You're on Kalico (or another Klipper fork) that ships its own `extras/statistics.py`, which shadows Python's stdlib `statistics`. The plugin works around this with a local `_pstdev` helper — make sure you have the latest version of `tmc_flow_test.py` and re-pull from the repo if needed.

**`TMC_FLOW_STATUS` reports "StallGuard2 needs SpreadCycle"** *(SG2 drivers — TMC5160, TMC2130, TMC2240)* —
Remove the `stealthchop_threshold:` line entirely from your TMC section, restart Klipper. For TMC2240: also remove any `[delayed_gcode]` block that sets `sg4_thrs` or `sg4_filt_en` — they're not needed in SG2 mode.

**Auto-SGT can't reach target range** —
Console says "could not reach target range" after 5 iterations. Usually means your SGT is at an extreme and still doesn't produce useful SG values. Check your `run_current` — if it's very low, even max-sensitive SGT may not see enough load. Try increasing `run_current` slightly or running with `AUTO_SGT=0` and a manually-chosen SGT.

**Trigger fires very early in coarse phase (e.g. at flow=50)** —
1. **SFILT is off** — check that `driver_SFILT: 1` is in your TMC section. SG noise without filter triggers false plateau detection.
2. **SGT was set too high** before Auto-SGT ran (or Auto-SGT is disabled). Re-run with default `AUTO_SGT=1`.
3. **CoolStep is masking signal** — try setting `driver_SEMIN: 0` for the most conservative result.

**Test reaches MAX without trigger** —
Either your hotend really can flow that fast (raise MAX), or SG sensitivity is still too low. Check the Auto-SGT output — if it tuned to a very high SGT (e.g. > 30), your motor torque headroom is bigger than the test's MAX value.

**Test ends without trigger and the chart shows a clear plateau** —
If `COARSE_STEP=10` skips over the slip onset, the single-step plateau trigger may not have enough resolution. Lower `COARSE_STEP` to 5 — the IQR cumulative growth trigger needs ≥5 coarse steps of history to fire, so 5 mm³/s steps give the algorithm enough data.

**TMC2240 results much lower than expected (<70 mm³/s on a fast extruder)** —
Check that you're running in **SpreadCycle/SG2** mode, not StealthChop/SG4. The SG4 path of the TMC2240 reduces peak torque by ~50 %. `TMC_FLOW_STATUS` will tell you which mode is active. Remove `stealthchop_threshold` from your `[tmc2240]` section if present.

**Result varies between runs by more than 5 mm³/s** —
Check filament consistency, hotend temperature stability, possible filament path obstructions. Increase `COOLDOWN` between phases (e.g. 30 s).

**"CoolStep is active" notice** —
The plugin works fine with CoolStep on, but for the most conservative result set `driver_SEMIN: 0` to disable CoolStep during testing. CoolStep can dampen StallGuard signal during high-load steps.

**Auto-SGT keeps tuning to the same value as my config** —
That's fine — it confirms your SGT is already optimal. The console will say "current SGT=N already optimal — no change needed".

**Thermal stress score (yellow/red zones) appears even though my heater seems fine** —
The score combines five signals; PWM peaks above 95 %, drops above 5 °C, or large intra-run SG drift each contribute. Check the chart's tooltip to see which component dominates. If only intra-run drift is high while PWM and temperature look stable, that often indicates the filament was getting harder to push during runs — possible cold extrusion or filament tangling.

### TMC2209-specific

**`TMC_FLOW_TEST_SG_VARIANTS` reports both modes "NOT USABLE"** —
1. HIGH_FLOW too low — motor never reaches actual stall. Increase HIGH_FLOW until you hear physical clacking during the test.
2. `run_current` too low — motor has too much torque headroom to slip. Try 0.85-1.0 A.
3. SG_RESULT stays at 0 or one fixed value across both modes → hardware-level SG4 problem (some TMC2209 clones have non-functional SG4). Try a different TMC2209 board or switch to TMC2240.

**TMC2209 SG values seem to "go the wrong way"** (rise with load instead of fall) —
That's normal on TMC2209 SG4. The Trinamic spec says higher SG = lower load, but in practice on many TMC2209 boards SG_RESULT magnitude is unreliable. The plugin's CV-spike triggers are direction-agnostic and detect slip regardless. As long as the pre-flight check reports "USABLE", the main test will work.

**TMC2209 main test ends very early or very late** —
The CV-based triggers tuned in `TMC2209Profile` are necessarily looser than for SG2 to handle SG4 noise. If borderline measurements seem off, try:
- Higher `REPEAT` (e.g. 10) for tighter run-to-run statistics
- Higher `VERIFY_REPEATS` (e.g. 10) for more confident verification
- Verify with multiple long real-world prints

**TMC2209 main test passes but real prints under-extrude at the recommended flow** —
Expected risk with the experimental TMC2209 path. Either:
1. Drop the slicer max-flow value 20 % below what the plugin reported
2. Re-test with `stealthchop_threshold: 999999` for the conservative documented mode
3. Switch to TMC2240 for a guaranteed-reliable result

---

## Compatibility

| Software | Status |
|---|---|
| **Klipper** (vanilla) | ✅ Tested |
| **Kalico** | ✅ Tested (uses local `_pstdev` to avoid Kalico's `statistics.py` shadowing stdlib) |
| **DangerKlipper** | ✅ Should work (same plugin path) |

The plugin uses only stdlib Python imports plus Klipper's `gcode`, `pins`, and `tmc` modules — no external dependencies.
