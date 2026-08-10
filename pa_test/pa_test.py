# PA Test — pressure-advance calibration via StallGuard (prototype)
# credits:
#   Steven (Fragmon) — Crydteam
#   YouTube: https://www.youtube.com/@crydteamprinting
#
# Concept: melt pressure loads the extruder motor, and that load is
# visible in the TMC driver's StallGuard signal (proven by the
# Crydteam TMC Flow Test). Pressure-advance errors are classic
# step-response errors — too little PA undershoots the pressure after
# a speed jump, too much overshoots. This module measures those
# pressure transients directly.
#
# This PROTOTYPE ships the feasibility stage:
#   PA_TEST_PROBE — step the extrusion rate between two flows
#   (E-only moves, no XY motion, no PA involved) and record the SG
#   step response at 50 Hz. The resulting CSV answers the question
#   the full PA search depends on: is the pressure time constant
#   resolvable above the SG noise floor on this hardware?
#
# License: GPLv3

import json
import logging
import math
import os
import time

MODULE_NAME = "PA Test"
MODULE_VERSION = "0.1.0"

TRINAMIC_DRIVERS = ("tmc2130", "tmc2209", "tmc2240", "tmc5160", "tmc2660")
SG2_DRIVERS = ("tmc2130", "tmc2240", "tmc5160", "tmc2660")

SAMPLE_INTERVAL = 0.02      # 50 Hz SG polling
MIN_HOTEND_TEMP = 180.0


class PATest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.stepper_name = config.get('extruder_stepper', 'extruder')
        self.filament_diameter = config.getfloat(
            'filament_diameter', 1.75, above=0.)
        self.min_hotend_temp = config.getfloat(
            'min_hotend_temp', MIN_HOTEND_TEMP, above=0.)

        config_dir = os.path.expanduser('~/printer_data/config')
        if not os.path.isdir(config_dir):
            config_dir = os.path.expanduser('~')
        self.output_dir = os.path.expanduser(config.get(
            'output_dir', os.path.join(config_dir, 'PAtest')))

        self.filament_area = math.pi * (self.filament_diameter / 2.) ** 2

        self.tmc = None
        self.driver_type = None
        self.is_2209 = False
        self.sg2_driver = False

        self.sampling_active = False
        self.sample_timer = None
        self.samples = []

        self.gcode.register_command(
            'PA_TEST_PROBE', self.cmd_PA_TEST_PROBE,
            desc='Record the extruder SG step response to flow jumps '
                 '(feasibility probe for SG-based pressure-advance '
                 'calibration)')
        self.gcode.register_command(
            'PA_TEST_GUI', self.cmd_PA_TEST_GUI,
            desc='Write the control panel (HTML) with live config values '
                 'into the output directory')

    # ─── TMC lookup / SG reading (flow-test pattern) ────────────────

    def _lookup_tmc(self):
        if self.tmc is not None:
            return
        for drv in TRINAMIC_DRIVERS:
            tmc = self.printer.lookup_object(
                "%s %s" % (drv, self.stepper_name), None)
            if tmc is not None:
                self.tmc = tmc
                self.driver_type = drv
                self.is_2209 = drv == 'tmc2209'
                self.sg2_driver = drv in SG2_DRIVERS
                return
        raise self.gcode.error(
            "pa_test: no TMC driver with StallGuard found for '%s'."
            % self.stepper_name)

    def _read_sg(self):
        try:
            if self.is_2209:
                return self.tmc.mcu_tmc.get_register('SG_RESULT') & 0x3FF
            return self.tmc.mcu_tmc.get_register('DRV_STATUS') & 0x3FF
        except Exception as e:
            logging.debug("pa_test: SG read failed: %s", e)
            return None

    # ─── Sampling ───────────────────────────────────────────────────

    def _start_sampling(self):
        self.samples = []
        self.sample_start = self.reactor.monotonic()
        self.sampling_active = True
        self.sample_timer = self.reactor.register_timer(
            self._sample_cb, self.reactor.NOW)

    def _stop_sampling(self):
        self.sampling_active = False
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

    def _sample_cb(self, eventtime):
        if not self.sampling_active:
            return self.reactor.NEVER
        sg = self._read_sg()
        if sg is not None:
            self.samples.append((eventtime - self.sample_start, sg))
        return eventtime + SAMPLE_INTERVAL

    # ─── Analysis (pure, unit-testable) ─────────────────────────────

    @staticmethod
    def _median(vals):
        if not vals:
            return None
        s = sorted(vals)
        n = len(s)
        return float(s[n // 2]) if n % 2 else (s[n // 2 - 1]
                                              + s[n // 2]) / 2.

    @staticmethod
    def analyze_transitions(samples, segments):
        """Step-response metrics from one probe recording.

        samples:  [(t, sg), ...] at ~50 Hz
        segments: [(t_start, t_end, flow), ...] commanded timeline

        Per transition (segment i -> i+1) computes:
          rise_time — first time the signal stays within 20 % of the
                      new steady band (relative to transition start)
          overshoot — max deviation beyond the new steady level in
                      the first 40 % of the segment, in SG units
        Steady level per segment = median of the last 40 % of it.
        Returns dict with per-transition list and aggregate medians,
        or None if there is not enough data.
        """
        if len(samples) < 20 or len(segments) < 2:
            return None
        med = PATest._median

        def window(t0, t1):
            return [sg for t, sg in samples if t0 <= t < t1]

        steadies = []
        for (t0, t1, flow) in segments:
            dur = t1 - t0
            steadies.append(med(window(t1 - dur * 0.4, t1)))

        transitions = []
        for i in range(len(segments) - 1):
            t0, t1, flow_from = segments[i]
            n0, n1, flow_to = segments[i + 1]
            s_from, s_to = steadies[i], steadies[i + 1]
            if s_from is None or s_to is None:
                continue
            step = s_to - s_from
            if abs(step) < 1e-9:
                continue
            band = abs(step) * 0.2
            seg = [(t, sg) for t, sg in samples if n0 <= t < n1]
            rise_time = None
            for t, sg in seg:
                if abs(sg - s_to) <= band:
                    rise_time = t - n0
                    break
            early = [sg for t, sg in seg if t < n0 + (n1 - n0) * 0.4]
            overshoot = 0.
            if early:
                if step > 0:
                    overshoot = max(0., max(early) - s_to)
                else:
                    overshoot = max(0., s_to - min(early))
            transitions.append({
                'flow_from': flow_from, 'flow_to': flow_to,
                'sg_from': s_from, 'sg_to': s_to, 'step': step,
                'rise_time': rise_time, 'overshoot': overshoot,
            })
        if not transitions:
            return None
        rts = [tr['rise_time'] for tr in transitions
               if tr['rise_time'] is not None]
        return {
            'transitions': transitions,
            'rise_time_median': med(rts) if rts else None,
            'overshoot_median': med(
                [tr['overshoot'] for tr in transitions]),
            'step_median': med(
                [abs(tr['step']) for tr in transitions]),
        }

    # ─── Command ────────────────────────────────────────────────────

    def _check_hotend(self):
        try:
            heater = self.printer.lookup_object('extruder').get_heater()
            status = heater.get_status(self.reactor.monotonic())
            temp = status['temperature']
        except Exception:
            return
        if temp < self.min_hotend_temp:
            raise self.gcode.error(
                "pa_test: hotend at %.0f °C — heat it to printing "
                "temperature first (min %.0f °C)."
                % (temp, self.min_hotend_temp))

    def cmd_PA_TEST_GUI(self, gcmd):
        """Write the control panel (pa_test_gui.html) into the output
        directory with the live config baked in. Must survive a cold
        printer and a missing driver — the page's job is to report that.
        """
        src = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                           'pa_test_gui.html')
        if not os.path.isfile(src):
            raise gcmd.error(
                "pa_test: GUI template not found at %s — pull the latest "
                "crydteamtools repo." % src)
        try:
            with open(src, encoding='utf-8') as f:
                html = f.read()
        except Exception as e:
            # A non-gcode.error would put Klipper into shutdown.
            raise gcmd.error(
                "pa_test: cannot read the GUI template %s: %s" % (src, e))
        if '/*CFG*/null' not in html:
            raise gcmd.error(
                "pa_test: GUI template at %s has no /*CFG*/null "
                "placeholder — the file was modified or reformatted." % src)

        try:
            self._lookup_tmc()
        except Exception:
            pass

        hotend_temp = None
        try:
            extruder = self.printer.lookup_object('extruder', None)
            if extruder is not None:
                hotend_temp = round(
                    extruder.get_heater().get_temp(
                        self.reactor.monotonic())[0], 1)
        except Exception:
            pass

        cfg = {
            'version': MODULE_VERSION,
            'driver': self.driver_type,
            'stepper': self.stepper_name,
            'filament_diameter': self.filament_diameter,
            'filament_area': round(self.filament_area, 4),
            'min_hotend_temp': self.min_hotend_temp,
            'hotend_temp': hotend_temp,
            'output_dir': self.output_dir,
        }
        # json.dumps escapes for a JS string, not for HTML: a value with
        # "</script>" would end the script block.
        html = html.replace('/*CFG*/null',
                            json.dumps(cfg).replace('</', '<\\/'))

        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as e:
            raise gcmd.error(
                "pa_test: cannot create output dir %s: %s"
                % (self.output_dir, e))
        dst = os.path.join(self.output_dir, 'pa_test_gui.html')
        tmp = dst + '.tmp'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                f.write(html)
            os.replace(tmp, dst)
        except Exception as e:
            try:
                os.remove(tmp)
            except Exception:
                pass
            raise gcmd.error(
                "pa_test: cannot write the control panel to %s: %s"
                % (dst, e))
        gcmd.respond_info(
            "Control panel written to:\n  %s\n"
            "Open it via your web UI's file browser (PAtest folder) or any "
            "browser. Re-run PA_TEST_GUI after config changes." % dst)

    def cmd_PA_TEST_PROBE(self, gcmd):
        flow_low = gcmd.get_float('FLOW_LOW', 10., above=0.)
        flow_high = gcmd.get_float('FLOW_HIGH', 60., above=flow_low)
        dwell = gcmd.get_float('DWELL', 1.0, minval=0.3)
        cycles = gcmd.get_int('CYCLES', 6, minval=1)

        self._lookup_tmc()
        self._check_hotend()

        def feed(flow):     # mm/s filament
            return flow / self.filament_area

        gcmd.respond_info(
            "──── PA_TEST_PROBE ────\n"
            "  extruder: %s (%s)\n"
            "  flow %.0f ↔ %.0f mm³/s (%.1f ↔ %.1f mm/s filament), "
            "%d cycles × %.1f s\n"
            "  E-only moves — measures the raw pressure step "
            "response, PA is not involved yet"
            % (self.stepper_name, self.driver_type,
               flow_low, flow_high, feed(flow_low), feed(flow_high),
               cycles, dwell))

        # Build one continuous script; segment timeline from the
        # commanded durations.
        script = ["M83"]
        segments = []
        t = 0.
        # settle lead-in at low flow so segment 0 has a clean steady
        plan = [flow_low]
        for _ in range(cycles):
            plan += [flow_high, flow_low]
        for flow in plan:
            f = feed(flow)
            script.append("G1 E%.4f F%.1f" % (f * dwell, f * 60.))
            segments.append((t, t + dwell, flow))
            t += dwell
        script.append("M400")

        self._start_sampling()
        try:
            self.gcode.run_script_from_command("\n".join(script))
        finally:
            self._stop_sampling()
        samples = list(self.samples)

        if len(samples) < 20:
            raise self.gcode.error(
                "pa_test: only %d SG samples — check the TMC wiring."
                % len(samples))
        sg_vals = [sg for _, sg in samples]
        if max(sg_vals) <= 0:
            raise self.gcode.error(
                "pa_test: StallGuard reads 0 throughout. Check "
                "chopper mode / tcoolthrs / SGT of [%s %s] (see the "
                "TMC Flow Test docs — same requirements)."
                % (self.driver_type, self.stepper_name))

        result = self.analyze_transitions(samples, segments)

        # CSV dump for offline analysis
        ts = time.strftime('%Y-%m-%d_%H-%M-%S')
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            path = os.path.join(self.output_dir,
                                'pa_probe_%s.csv' % ts)
            with open(path, 'w') as f:
                f.write("# PA Test v%s probe\n" % MODULE_VERSION)
                f.write("# Plugin by Steven (Fragmon) — Crydteam\n")
                f.write("# driver: %s on %s\n"
                        % (self.driver_type, self.stepper_name))
                f.write("# flow_low: %.1f  flow_high: %.1f  "
                        "dwell: %.2f  cycles: %d\n"
                        % (flow_low, flow_high, dwell, cycles))
                f.write("# segments: %s\n"
                        % "; ".join("%.2f-%.2f@%.0f" % s
                                    for s in segments))
                f.write("t_s,sg\n")
                for t_s, sg in samples:
                    f.write("%.3f,%d\n" % (t_s, sg))
            gcmd.respond_info("CSV saved: %s" % path)
        except Exception as e:
            gcmd.respond_info("CSV write failed: %s" % e)

        if result is None:
            gcmd.respond_info(
                "Not enough clean transitions to analyze — try "
                "DWELL=1.5 or more CYCLES.")
            return

        noise = self._median(
            [abs(sg_vals[i] - sg_vals[i - 1])
             for i in range(1, len(sg_vals))])
        snr = (result['step_median'] / noise) if noise else 0.
        gcmd.respond_info(
            "──── step response ────\n"
            "  SG step (low↔high flow): %.0f units | sample-to-sample "
            "noise: %.1f | SNR ≈ %.1f\n"
            "  rise time (median): %s\n"
            "  overshoot (median): %.1f SG units\n"
            "  verdict: %s"
            % (result['step_median'], noise, snr,
               ("%.0f ms" % (result['rise_time_median'] * 1000.)
                if result['rise_time_median'] is not None else "n/a"),
               result['overshoot_median'],
               ("signal is usable for PA calibration — the full "
                "PA search can be built on this hardware" if snr >= 5
                else "SNR too low at these flows — retry with a "
                     "larger FLOW_HIGH-FLOW_LOW spread")))


def load_config(config):
    return PATest(config)
