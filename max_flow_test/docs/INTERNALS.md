# How slip detection works

The plugin samples StallGuard at 20 Hz during each measurement step (5 repeats × 5 s by default) and tracks per-step **median**, **IQR (P25–P75)**, **run-to-run CV**, and **intra-run trend** (slope of SG over time within a single run).

Slip detection uses **multiple independent triggers** that look for different signatures:

- **SG signal patterns** — snap-back, over-jump, single-step plateau, 2-step cumulative plateau (with saturation-skip and median-baseline)
- **Run-to-run variance** — CV spike, CV jump, rising trend, vs coarse-baseline
- **Sample distribution** — IQR widening (single-step), IQR cumulative growth (vs early-test baseline), IQR vs coarse-baseline, IQR absolute floor
- **Per-run analysis** — single-run outlier detection (warmup-aware), SG max spike for decoupling

Each trigger fires under tighter conditions in **bisection / verify** than in coarse, so the coarse phase stays noise-resistant while the final result is accurate to ±1 mm³/s.

The HTML report's **decision-trail panel** lists every trigger event with the metrics that caused it, so you can see exactly why the plugin chose the value it did.

---

## Plateau and IQR-growth triggers

Two of the most important trigger types deserve more explanation:

### Single-step plateau

Fires when the most recent step's SG-delta is essentially flat compared to the recent baseline (e.g. typical −25 deltas then suddenly +0.5). Catches abrupt plateaus that a 2-step cumulative trigger would smooth over.

The threshold is half the per-driver `PLATEAU_RATIO` — the actual delta must be a much clearer single-step plateau to fire than a borderline 2-step accumulation.

### IQR cumulative growth

Fires when within-step spread (P75 − P25) has gradually doubled relative to the early-test baseline AND now exceeds an absolute floor of 12 raw units. Catches gradual stick-slip onset that single-step ratio triggers don't see.

Requirements:
- ≥5 coarse steps of history (so the baseline is meaningful)
- Last IQR ≥ 12 absolute units
- Last IQR ≥ 2.0 × early baseline (median of 2nd+3rd smallest of first 4 IQRs)

The early baseline uses the median of mid-ranked IQRs (not the smallest, which can be a noise floor). This makes the trigger robust against initial noise.

---

## Warmup-skip

The first repetition of every measurement step shows different behaviour than the others (motor transitions from cold-stop, filament path settles). The plugin detects this drift and excludes run 1 from the median/IQR/CV stats when it deviates significantly from the rest.

The threshold is per-driver:
- TMC5160: 10 % cutoff
- TMC2240: 4 % cutoff (more aggressive — TMC2240 has systematic 3–6 % first-run drift)
- TMC2209: 15 % cutoff (SG4 has higher first-run drift)

---

## Thermal monitoring

During every measurement the plugin captures heater PWM (avg/max), hotend temperature (target/actual/min/drop), and TMC driver thermal flags (otpw / ot). These appear in the CSV and HTML report. **They do NOT trigger anything** — they're context for interpreting the result.

A heuristic 0–100 **thermal stress score** is computed per step combining five signals:

1. **Heater PWM level** (0–30 points) — how hard the heater is working
2. **Temperature drop from target** (0–25 points) — how much actual temp falls below target
3. **PWM rising trend** (0–15 points) — heater taking on more work over the sweep
4. **PWM peak saturation hits** (0–10 points) — saturation events even briefly
5. **Intra-run SG drift** (0–20 points) — load growing within a single run = cold extrusion fingerprint

The chart backgrounds get tinted: **green (0–30 stable)** / **yellow (30–60 moderate)** / **red (60+ likely cold extrusion)**. The first flow where the score crosses 30 is marked as "cold extrusion onset" — purely visual, doesn't affect detection.

> **Why intra-run SG drift matters:** Within a single 5-second run, if SG drifts steadily toward higher load, the filament was getting harder to push as it heated. This is the physical signature of cold extrusion (vs. abrupt motor slip, which jumps suddenly without a gradient). Hover any thermal-chart point to see the per-step drift value.

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
