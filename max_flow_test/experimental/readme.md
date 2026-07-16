# CoolStep Archive — tmc_flow_test v3.0

## What is this?

This folder contains the last version of `tmc_flow_test.py` that included
**full CoolStep (CS) integration** — CS sampling, CS-based slip triggers,
CS_ACTUAL chart in the HTML report, and the `MODE=cs` / `MODE=sg` /
`MODE=auto` test modes.

The mainline plugin has since been simplified to **SG-only operation**.
This archive preserves the CoolStep functionality for reference, future
research, or potential reintegration.

## Why was CoolStep removed from mainline?

Empirical testing across multiple driver/extruder combinations showed
CoolStep introduced more problems than it solved for max-flow detection:

1. **Confounding variable**. CoolStep dynamically changes motor current
   during the test based on its own SG thresholds (`SEMIN`, `SEMAX`).
   This means each test step ran under different electrical conditions,
   making SG values across steps not directly comparable.

2. **False-positive trigger**. On TMC2240 (CSV `19-57-51`), CoolStep
   crashed `CS_ACTUAL` from 31 to 7 between flow=20 and flow=30. At
   flow=40 the within-step IQR widened to 43 (vs prior baseline 9), but
   CV was only 0.6 % — clearly not slip. The IQR widening was caused by
   the CS-driven current transition, not by the motor losing torque.

3. **Setup complexity**. Users had to configure five CoolStep registers
   (`SEMIN`, `SEMAX`, `SEUP`, `SEDN`, `SEIMIN`) with values that vary
   per motor. Wrong values (e.g. `SEMAX < SEMIN`) silently broke
   CoolStep without errors.

4. **Realistic worst-case**. Without CoolStep the motor runs at the
   user's configured `run_current` for the entire test. If it slips at
   that current, it will slip during real prints too — a more
   conservative and more useful result.

5. **Driver-agnostic**. SG-only operation works identically across
   TMC5160, TMC2240 (SG2 path), TMC2130, TMC2660, and TMC2209 — no
   per-driver CoolStep tuning needed.

## What's in this folder

| File | Description |
|---|---|
| `tmc_flow_test_v3.0_with_coolstep.py` | Last full-CS version. Has all CS triggers, CS sampling, CS_ACTUAL chart, MODE parameter. |
| `README_COOLSTEP_ARCHIVE.md` | This file. |

## What CoolStep features are in the archived version?

### CS sampling
- `_read_cs()` reads `CS_ACTUAL` (DRV_STATUS bits 16-20) at the same
  20 Hz rate as SG.
- Sample buffer `samples_cs` aligned with `samples_sg`.

### CS-specific slip triggers (in `_check_triggers_cs`)
- **CS pegged + SG drop** — CS at 31 (no current adaptation) but SG
  fell hard → motor ran out of torque headroom.
- **CS pegged + CV spike** — CS at 31 + run-to-run variance jumped →
  intermittent slip without CS reaction.
- **CS hard drop** — CS dropped sharply between consecutive steps →
  CoolStep saw stall load.

### CSV columns (CS-specific)
- `cs_median`, `cs_p25`, `cs_p75`, `cs_avg`
- `run_cs_avgs` (per-repeat CS averages, used for `_check_run_outlier`
  CS-confirmation logic)

### HTML report
- `CS_ACTUAL` time-series chart per step, alongside `SG_RESULT`.
- CS-related decision-trail entries.

### Config validation (in `_check_tmc_config`)
- Mode-specific checks: `mode='cs'` requires `SEMIN > 0`, `mode='sg'`
  requires `SEMIN == 0`.
- AN-002-based SEMIN recommendation at end of test
  (`SEMIN ≈ SG_max / 32 / 4..8`).

### Commands
- `TMC_FLOW_FIND_MAX_CS` — force CS mode
- `TMC_FLOW_FIND_MAX_SG` — force SG-only mode
- `TMC_FLOW_FIND_MAX` with `MODE=auto` — autodetect from `SEMIN`

## How to restore CoolStep functionality

If you want to re-enable CoolStep operation in a future version:

1. **Compare against current mainline**:
   ```bash
   diff -u tmc_flow_test.py archive/tmc_flow_test_v3.0_with_coolstep.py
   ```

2. **Cherry-pick the relevant sections** from the archived file:
   - `_read_cs()` method
   - CS sample buffer setup in `__init__`
   - CS sampling in `_sample_callback`
   - `_check_triggers_cs()` method
   - CS columns in `_measure_step` results dict
   - CSV header + row writers (look for `cs_median` etc.)
   - HTML report CS chart logic
   - CS-related fields in `_snapshot_tmc_settings`
   - `MODE` parameter handling and the `cs`/`sg`/`auto` branch logic

3. **Test thoroughly** with both SG2 (TMC5160/2240) and SG4 (TMC2209)
   drivers — CoolStep behaviour differs significantly between them.

4. **Be aware** that CoolStep can mask the very condition this test is
   trying to find (max sustained flow before slip). If you reintroduce
   CS, consider making it a research/debug mode rather than the
   default.

## What stays in mainline (post-archive)

The mainline plugin keeps:
- All SG-based triggers (Snap-back, Over-jump, CV-Spike, IQR-Spread,
  SG-Max-Spike, Run-Outlier in SG-only form)
- TriggerProfile architecture (per-driver tuning)
- Three-phase test (Coarse → Bisection → Verification)
- Borderline re-test logic
- HTML report (without CS chart)
- CSV output (without CS columns)

## Klipper config changes when removing CS

Users do **not** need to edit their Klipper config when upgrading from
the CS-version to the SG-only mainline. CoolStep registers in
`[tmc<NNNN> extruder]` (`SEMIN`, `SEMAX`, etc.) are simply ignored by
the SG-only test — they continue to work normally for actual printing.

The plugin will warn at test start if `SEMIN > 0`:
```
INFO: CoolStep is active (SEMIN=2). The test will run with your
configured SEMIN — this may dampen SG signal during high-load steps.
For most consistent results, set SEMIN: 0 temporarily.
```

(That advisory replaces the previous `MODE=cs` mode logic.)

## Version reference

- **Archive snapshot**: tmc_flow_test v3.0 with full CoolStep, refactored
  to use TriggerProfile architecture, TMC2240 already migrated to SG2 path.
- **Date archived**: 2026-04-30
- **Source line count**: 3358 lines

## Credits

Plugin by Steven (Fragmon) — Crydteam
YouTube: <https://www.youtube.com/@crydteamprinting>
License: GPLv3
