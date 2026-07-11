# Output & reports

[← back to README](../README.md)

Files are saved to `~/printer_data/config/Speedtest/` (configurable via
`output_dir`):

```
speed_<kind>_YYYY-MM-DD_HH-MM-SS.csv     ← raw data
speed_<kind>_YYYY-MM-DD_HH-MM-SS.html    ← interactive report
```

Pass `NO_HTML=1` to any command for CSV-only output.

## HTML report

The HTML report renders in any browser and includes:

- **Lost-steps chart** — bar per measurement; height = how many microsteps were
  lost
- **TMC StallGuard chart** (when monitoring is enabled and XY drivers are TMC)
- **Data table** with phase / value / pass-fail / lost-steps / SG min+median per
  axis
- **Phase markers** — dashed vertical lines at phase transitions
- **Stop reason** — which trigger fired and at what value

## Limit-map report

The [V/A limits test](limits.md) writes its own report:

- the limit map **curve** (max safe accel vs. velocity) with the **sweet spot**
  highlighted
- a **min-current** column per velocity (from stage 3)
- your **current `printer.cfg` values** and the **TMC driver / run_current**
  side by side, for comparison
- a free-text **toolhead-weight** field, saved with the report in your browser
