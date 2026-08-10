# Control panel (GUI)

[← back to README](../README.md)

A beginner-friendly web page that builds the flow-test commands for you — no
parameter memorizing. It shows a live driver checklist (and names any
StallGuard problem), lets you pick a hotend preset, explains every field in
plain language and gives you the finished command to copy or send.

## Generate it

In the Klipper console:

```
TMC_FLOW_GUI
```

(or click the `CRYD_FLOW_GUI` macro). The plugin writes `tmc_flow_gui.html`
into the Flowtest output folder — **with your live setup baked in**: detected
driver and SGT value, any TMC configuration problem, filament diameter, melt
zone, minimum hotend temperature and the current hotend temperature. The
console prints the exact path.

Re-run `TMC_FLOW_GUI` after changing the `[tmc_flow_test]` config or the
driver section so the page reflects the new values.

## Open it

In Mainsail/Fluidd: **Machine → file browser → `Flowtest` folder** →
download/open `tmc_flow_gui.html` in any browser. It is a plain,
self-contained HTML file and also works offline.

## The four steps on the page

1. **Before you start** — a checklist built from your real setup: which
   driver was found and its SGT, every StallGuard configuration problem the
   plugin detects (wrong chopper mode, `tcoolthrs=0`, …), filament/melt-zone
   values, whether the hotend is hot enough right now, and where the reports
   go.
2. **What do you want to run?** — one card per command. The **max flow test**
   is marked *recommended*; cards that don't apply to your driver are greyed
   out (the TMC2209 pre-flight on non-2209 drivers, SGT calibration on
   drivers without a signed `sgt` field).
3. **Settings** — pick a **hotend preset** (V6 / Volcano / Rapido / Rapido
   0.6 / Goliath / unknown-discover) which sets a sensible start, ceiling and
   step size. Adjust the basic fields; everything a beginner never needs is
   folded away under *Advanced settings*. A runtime estimate updates live.
4. **Run it** — the finished command is shown; **Copy** puts it on the
   clipboard (or offers a selectable text box when the browser blocks the
   clipboard on plain `http://`), **Send to printer** posts it straight to
   Moonraker.

## Sending directly to the printer

The **Send to printer** button posts to Moonraker's
`/printer/gcode/script` endpoint. For that to work, the page's origin must be
allowed in Moonraker:

```ini
[authorization]
cors_domains:
    *://*.local
    *://my-printer-ip
```

Without it the browser blocks the request — the page then tells you to use
**Copy** and paste into the console instead, which always works.

## Safety

The page only *builds* commands — it never runs anything on its own. The
test itself extrudes real filament, so heat the hotend to printing
temperature, load filament and extrude into free air or over the purge
bucket.
