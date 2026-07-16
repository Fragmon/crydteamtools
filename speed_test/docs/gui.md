# Control panel (GUI)

[← back to README](../README.md)

A beginner-friendly web page that builds the test commands for you — no
parameter memorizing. It walks you through a setup checklist, test selection,
presets and settings with plain-language help, then gives you the finished
command to copy or send.

## Generate it

In the Klipper console:

```
SPEED_TEST_GUI
```

(or click the `ST_GUI` macro). The plugin writes
`speed_test_gui.html` into the Speedtest output folder — **with your live
config baked in**: testbench mode, current safety cap, detected TMC driver
and run_current. The console prints the exact path.

Re-run `SPEED_TEST_GUI` whenever you change the `[speed_test]` config so the
page reflects the new values.

## Open it

In Mainsail/Fluidd: **Machine → file browser → `Speedtest` folder** →
download/open `speed_test_gui.html` in any browser. It is a plain,
self-contained HTML file — it also works offline.

## The four steps on the page

1. **Before you start** — a checklist built from your real config: testbench
   mode, `max_current` cap (warns if none is set), TMC driver + run_current,
   and where the reports go.
2. **What do you want to test?** — one card per test; the **limit map** is
   marked *recommended* and is the right choice for most people.
3. **Settings** — pick a preset (**Quick look ~15 min / Standard /
   Thorough**), adjust the basic fields (axis, speed range, …). Every field
   has a short explanation; advanced options stay collapsed and are safe to
   ignore.
4. **Run it** — the finished command is previewed live (only values that
   differ from the defaults are included).
   - **Copy command** → paste into the Klipper console.
   - **Send to printer** → posts the command straight to Moonraker. The URL
     is pre-filled with `http://<your-host>:7125`.

## "Send to printer" and CORS

Direct sending only works if your browser's origin is allowed by Moonraker.
If the button reports an error, either just use **Copy**, or add the origin
to `moonraker.conf`:

```ini
[authorization]
cors_domains:
    *://my.mainsail.xyz
    *://*.local
    *://localhost
```

(Adjust to where you open the page from, then restart Moonraker.)

## Help section

The bottom of the page answers the most common questions: what the four
stages of the limit map mean, why results can differ between runs
(motor temperature!), where the reports are saved, and how testbench mode
works.
