# Speed Test — Adaptive max velocity / accel / SCV detection for steppers
# Detects skipped steps via the endstop_phase module and narrows the safe
# limit with bracket bisection (Coarse → Bisect → Verify).
#
# Plugin by Steven (Fragmon) — Crydteam
# YouTube: https://www.youtube.com/@crydteamprinting
#
# License: GPLv3

import logging
import math
import os
import statistics
import time
import json

MODULE_NAME = "Speed Test"
MODULE_VERSION = "1.0"
SAMPLE_INTERVAL = 0.05          # 20 Hz TMC polling during moves
TMC_DRIVERS = ['tmc2240', 'tmc5160', 'tmc2209', 'tmc2226',
               'tmc2130', 'tmc2208', 'tmc2660']
SG2_DRIVERS = ('tmc5160', 'tmc2130', 'tmc2660')


class SpeedTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.structure = config.get('structure', 'cartesian').lower()
        if self.structure not in ('cartesian', 'corexy'):
            raise config.error(
                "speed_test: structure must be 'cartesian' or 'corexy'")
        self.default_axis = config.get('default_axis', 'X').upper()
        if self.default_axis not in ('X', 'Y'):
            raise config.error(
                "speed_test: default_axis must be X or Y")
        self.margin = config.getfloat('margin', 20.0, above=0.)
        self.z_pos = config.getfloat('z_pos', 20.0, minval=0.)
        self.monitor_tmc = config.getboolean('monitor_tmc', True)
        # Testbench mode: only X-stepper connected, no Y, no Z. Skips all
        # Y/Z homing and ignores Y in skip checks and TMC monitoring.
        # Per-command TESTBENCH=1/0 override is honored.
        self.testbench_default = config.getboolean('testbench', False)

        config_dir = os.path.expanduser('~/printer_data/config')
        if not os.path.isdir(config_dir):
            config_dir = os.path.expanduser('~')
        default_dir = os.path.join(config_dir, 'Speedtest')
        self.output_dir = config.get('output_dir', default_dir)

        # State
        self._last_mcu_pos = {}
        self._tmc_cache = {}            # axis -> tmc obj (or None)
        self._sample_buf = {}           # axis -> list[int]
        self._sampling_active = False
        self._sample_timer = None
        self._sample_start = 0.0
        self._sample_axes = []

        self.gcode.register_command(
            'SPEED_TEST_FIND_MAX_VELOCITY',
            self.cmd_FIND_MAX_VELOCITY,
            desc='Find max safe velocity for an axis (adaptive bisection)')
        self.gcode.register_command(
            'SPEED_TEST_FIND_MAX_ACCEL',
            self.cmd_FIND_MAX_ACCEL,
            desc='Find max safe acceleration for an axis (adaptive bisection)')
        self.gcode.register_command(
            'SPEED_TEST_FIND_MAX_SCV',
            self.cmd_FIND_MAX_SCV,
            desc='Find max safe square-corner velocity (adaptive bisection)')
        self.gcode.register_command(
            'SPEED_TEST_BENCHMARK',
            self.cmd_BENCHMARK,
            desc='Repeatable random-pattern stress test at fixed speed/accel')
        self.gcode.register_command(
            'SPEED_TEST_STATUS',
            self.cmd_STATUS,
            desc='Show speed_test configuration and axis state')

    # ─── Settings helpers ─────────────────────────────────────────────

    def _get_axis_bounds(self, axis):
        """Return (min, max, mid, range) for the axis with margin applied."""
        stepper = 'stepper_' + axis.lower()
        try:
            cfg = self.printer.lookup_object('configfile')
            settings = cfg.get_status(self.reactor.monotonic())['settings']
            axis_cfg = settings[stepper]
            ax_min = float(axis_cfg['position_min'])
            ax_max = float(axis_cfg['position_max'])
        except Exception as e:
            raise self.gcode.error(
                "speed_test: failed to read %s bounds: %s" % (stepper, e))
        margin = min(self.margin, 0.1 * (ax_max - ax_min))
        ax_min += margin
        ax_max -= margin
        return ax_min, ax_max, (ax_min + ax_max) / 2.0, ax_max - ax_min

    def _get_microsteps(self, axis):
        stepper = 'stepper_' + axis.lower()
        try:
            cfg = self.printer.lookup_object('configfile')
            settings = cfg.get_status(self.reactor.monotonic())['settings']
            return int(settings[stepper].get('microsteps', 16))
        except Exception:
            return 16

    def _get_printer_limits(self):
        try:
            cfg = self.printer.lookup_object('configfile')
            settings = cfg.get_status(self.reactor.monotonic())['settings']
            p = settings['printer']
            return (float(p['max_velocity']), float(p['max_accel']),
                    float(p.get('square_corner_velocity', 5.0)))
        except Exception:
            return (300., 5000., 5.0)

    def _set_limits(self, velocity=None, accel=None, scv=None):
        parts = []
        if velocity is not None:
            parts.append("VELOCITY=%.2f" % velocity)
        if accel is not None:
            parts.append("ACCEL=%.2f" % accel)
        if scv is not None:
            parts.append("SQUARE_CORNER_VELOCITY=%.3f" % scv)
        if parts:
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT " + " ".join(parts))

    def _restore_limits(self):
        v, a, scv = self._get_printer_limits()
        self._set_limits(velocity=v, accel=a, scv=scv)

    # ─── Homing & skip detection ──────────────────────────────────────

    def _ensure_homed(self, axes, testbench=False):
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(
            self.reactor.monotonic()).get('homed_axes', '')

        if testbench:
            # Only home X — no Y, no Z. Useful for bench setups with a
            # single stepper hooked up to X.
            if 'x' not in homed:
                self.gcode.run_script_from_command("G28 X")
            else:
                self.gcode.run_script_from_command("G28 X")
            return

        if self.structure == 'corexy':
            # CoreXY: X and Y motors are mechanically coupled — must home both.
            if 'x' not in homed or 'y' not in homed or 'z' not in homed:
                self.gcode.run_script_from_command("G28")
                return
            self.gcode.run_script_from_command("G28 X Y")
            return

        # Cartesian: home only what's actually needed.
        needed = {a.lower() for a in axes}
        needed.add('z')  # Z is required so we can lift to z_pos safely
        missing = [a for a in needed if a not in homed]
        if missing:
            # Home only the missing axes — never trigger Y unless we need Y.
            self.gcode.run_script_from_command(
                "G28 " + " ".join(a.upper() for a in missing))
            return
        self.gcode.run_script_from_command(
            "G28 " + " ".join(axes))

    def _read_mcu_pos(self, axis):
        ep = self.printer.lookup_object('endstop_phase', None)
        if ep is None:
            return None
        try:
            status = ep.get_status(self.reactor.monotonic())
        except Exception:
            return None
        last_home = status.get('last_home', {}) if isinstance(status, dict) \
            else getattr(ep, 'last_home', {})
        stepper_name = 'stepper_' + axis.lower()
        data = last_home.get(stepper_name) if isinstance(last_home, dict) \
            else getattr(last_home, stepper_name, None)
        if data is None:
            return None
        if isinstance(data, dict):
            return data.get('mcu_position')
        return getattr(data, 'mcu_position', None)

    def _store_mcu_pos(self, axes):
        for axis in axes:
            pos = self._read_mcu_pos(axis)
            if pos is not None:
                self._last_mcu_pos[axis] = pos

    def _check_skip(self, axes):
        """Returns list of (axis, abs_diff) for axes that lost steps."""
        skips = []
        for axis in axes:
            new_pos = self._read_mcu_pos(axis)
            old_pos = self._last_mcu_pos.get(axis)
            if new_pos is None or old_pos is None:
                continue
            diff = abs(new_pos - old_pos)
            threshold = self._get_microsteps(axis)
            if diff > threshold:
                skips.append((axis, diff))
            self._last_mcu_pos[axis] = new_pos
        return skips

    def _check_endstop_phase(self):
        ep = self.printer.lookup_object('endstop_phase', None)
        if ep is None:
            raise self.gcode.error(
                "speed_test: [endstop_phase] module not configured.\n"
                "Add this to your printer.cfg:\n"
                "  [endstop_phase]\n"
                "After FIRMWARE_RESTART, run the test again.")

    # ─── TMC monitoring (optional) ────────────────────────────────────

    def _lookup_tmc_for_axis(self, axis):
        if axis in self._tmc_cache:
            return self._tmc_cache[axis]
        stepper_name = 'stepper_' + axis.lower()
        for drv in TMC_DRIVERS:
            tmc = self.printer.lookup_object(
                '%s %s' % (drv, stepper_name), None)
            if tmc is not None:
                self._tmc_cache[axis] = (drv, tmc)
                return self._tmc_cache[axis]
        self._tmc_cache[axis] = None
        return None

    def _read_tmc_sg(self, axis):
        info = self._lookup_tmc_for_axis(axis)
        if info is None:
            return None
        drv, tmc = info
        try:
            if drv == 'tmc2240':
                return tmc.mcu_tmc.get_register('SG4_RESULT') & 0x3FF
            if drv == 'tmc2209':
                return tmc.mcu_tmc.get_register('SG_RESULT') & 0x3FF
            if drv in SG2_DRIVERS:
                return tmc.mcu_tmc.get_register('DRV_STATUS') & 0x3FF
        except Exception:
            return None
        return None

    def _start_tmc_sampling(self, axes):
        if not self.monitor_tmc:
            return
        self._sample_axes = list(axes)
        self._sample_buf = {ax: [] for ax in axes}
        self._sample_start = self.reactor.monotonic()
        self._sampling_active = True
        self._sample_timer = self.reactor.register_timer(
            self._tmc_sample_cb, self.reactor.NOW)

    def _stop_tmc_sampling(self):
        self._sampling_active = False
        if self._sample_timer is not None:
            self.reactor.unregister_timer(self._sample_timer)
            self._sample_timer = None

    def _tmc_sample_cb(self, eventtime):
        if not self._sampling_active:
            return self.reactor.NEVER
        for axis in self._sample_axes:
            sg = self._read_tmc_sg(axis)
            if sg is not None and sg > 0:
                self._sample_buf[axis].append(sg)
        return eventtime + SAMPLE_INTERVAL

    def _tmc_stats(self, axis):
        buf = self._sample_buf.get(axis, [])
        if not buf:
            return None
        s = sorted(buf)
        n = len(s)
        return {
            'min': s[0], 'max': s[-1],
            'median': s[n // 2],
            'avg': sum(s) / n,
            'n': n,
        }

    # ─── Movement primitives ──────────────────────────────────────────

    def _move_to_axis(self, axis, pos, feed_mm_s):
        feed = max(60., feed_mm_s * 60.)
        if axis == 'X':
            self.gcode.run_script_from_command(
                "G1 X%.3f F%.1f" % (pos, feed))
        else:
            self.gcode.run_script_from_command(
                "G1 Y%.3f F%.1f" % (pos, feed))

    def _do_velocity_pattern(self, axis, velocity, distance, repeat,
                             testbench=False):
        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        dist = min(distance, ax_range)
        feed = velocity * 60.
        low = ax_mid - dist / 2.0
        high = ax_mid + dist / 2.0
        # Park at middle first — skip Z in testbench mode (no Z motor)
        if testbench:
            self._move_to_axis(axis, ax_mid, velocity)
        elif axis == 'X':
            self.gcode.run_script_from_command(
                "G1 X%.3f Z%.3f F%.1f"
                % (ax_mid, self.z_pos, feed))
        else:
            self.gcode.run_script_from_command(
                "G1 Y%.3f Z%.3f F%.1f"
                % (ax_mid, self.z_pos, feed))
        for _ in range(repeat):
            self._move_to_axis(axis, low, velocity)
            self._move_to_axis(axis, high, velocity)
            self._move_to_axis(axis, ax_mid, velocity)
        self.gcode.run_script_from_command("M400")

    def _do_scv_pattern(self, speed, corner_size, repeat):
        x_min, x_max, x_mid, _ = self._get_axis_bounds('X')
        y_min, y_max, y_mid, _ = self._get_axis_bounds('Y')
        half = min(corner_size / 2.0, (x_max - x_min) / 2.5,
                   (y_max - y_min) / 2.5)
        xl, xr = x_mid - half, x_mid + half
        yf, yb = y_mid - half, y_mid + half
        feed = speed * 60.
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f Z%.3f F%.1f"
            % (x_mid, y_mid, self.z_pos, feed))
        for _ in range(repeat):
            # Square corners
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xl, yf, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xl, yb, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xr, yb, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xr, yf, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xl, yf, feed))
            # Zigzag (rapid direction changes — hardest on SCV)
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xr, yb, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xl, yb, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xr, yf, feed))
        self.gcode.run_script_from_command("M400")

    def _do_benchmark_pattern(self, speed, iterations, bound,
                              small_size, zpos, seed):
        x_min, x_max, _, _ = self._get_axis_bounds('X')
        y_min, y_max, _, _ = self._get_axis_bounds('Y')
        xmin = x_min + max(0., bound - self.margin)
        xmax = x_max - max(0., bound - self.margin)
        ymin = y_min + max(0., bound - self.margin)
        ymax = y_max - max(0., bound - self.margin)
        xc = (xmin + xmax) / 2.0
        yc = (ymin + ymax) / 2.0
        xcl, xcr = xc - small_size / 2.0, xc + small_size / 2.0
        ycf, ycb = yc - small_size / 2.0, yc + small_size / 2.0
        feed = speed * 60.

        # Deterministic small offsets seeded by the SEED parameter
        # (avoids real RNG so result is reproducible)
        seq = []
        s = max(1, int(seed))
        for _ in range(2 * iterations):
            s = (1103515245 * s + 12345) & 0x7FFFFFFF
            seq.append((s % 1000) / 200.0)
        seq_iter = iter(seq)

        self.gcode.run_script_from_command(
            "G1 Y%.3f Z%.3f F%.1f" % (ymin, zpos, feed))
        for _ in range(iterations):
            for (x, y) in (
                    (xmin, ymin), (xmax, ymax), (xmin, ymin),
                    (xmax, ymin), (xmin, ymax), (xmax, ymin),
                    (xmin, ymin), (xmin, ymax), (xmax, ymax),
                    (xmax, ymin),
                    (xcl, ycf), (xcr, ycb), (xcl, ycf),
                    (xcr, ycf), (xcl, ycb), (xcr, ycf),
                    (xcl, ycf), (xcl, ycb), (xcr, ycb), (xcr, ycf),
                    ):
                self.gcode.run_script_from_command(
                    "G1 X%.3f Y%.3f F%.1f" % (x, y, feed))
            try:
                r1 = next(seq_iter)
                r2 = next(seq_iter)
            except StopIteration:
                r1 = r2 = 0.0
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xcl + r1, ycf + r1, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xcr - r1, ycb - r1, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xcl + r2, ycf + r2, feed))
            self.gcode.run_script_from_command(
                "G1 X%.3f Y%.3f F%.1f" % (xcr - r2, ycb - r2, feed))
        self.gcode.run_script_from_command("M400")

    # ─── Measurement step ─────────────────────────────────────────────

    def _measure_step(self, gcmd, axes, label, value, do_pattern,
                      testbench=False):
        """Run a movement pattern, re-home, return skip info + TMC stats."""
        if testbench:
            sample_axes = ('X',)
        elif self.structure == 'corexy':
            sample_axes = ('X', 'Y')
        else:
            sample_axes = tuple(axes)

        self._store_mcu_pos(sample_axes)
        if self.monitor_tmc:
            self._start_tmc_sampling(sample_axes)
        try:
            do_pattern()
        finally:
            self._stop_tmc_sampling()

        # Re-home and compare
        self._ensure_homed(list(axes), testbench=testbench)
        skips = self._check_skip(sample_axes)
        tmc_stats = {ax: self._tmc_stats(ax) for ax in sample_axes}

        failed = bool(skips)
        max_diff = max((d for _, d in skips), default=0)
        skip_axes = ",".join(a for a, _ in skips)

        gcmd.respond_info(
            "  %s = %.1f  →  %s%s"
            % (label, value,
               "FAILED (%d steps on %s)" % (max_diff, skip_axes)
               if failed else "OK",
               self._format_tmc(tmc_stats)))

        return {
            'value': value,
            'failed': failed,
            'max_diff': max_diff,
            'skip_axes': skip_axes,
            'tmc': tmc_stats,
        }

    def _format_tmc(self, tmc_stats):
        parts = []
        for ax, s in tmc_stats.items():
            if s is None:
                continue
            parts.append("%s SG min=%d med=%d" % (ax, s['min'], s['median']))
        return " | " + " | ".join(parts) if parts else ""

    # ─── Generic adaptive bisection ───────────────────────────────────

    def _adaptive_find_max(self, gcmd, results, low_bound, high_bound,
                           coarse_step, min_step, verify_repeats,
                           max_bisect, label, measure_at):
        """results is appended in place. measure_at(value, phase) is called."""
        # Phase 1: Coarse
        gcmd.respond_info(
            "\n>>> Phase 1: Coarse Sweep <<<\n"
            "  Step %s from %.1f to %.1f in increments of %.1f."
            % (label, low_bound, high_bound, coarse_step))
        value = low_bound
        low = low_bound
        high = None
        first_fail_reason = None
        while value <= high_bound + 0.001:
            r = measure_at(value, 'coarse')
            results.append(r)
            if r['failed']:
                high = value
                low = max(low_bound, value - coarse_step)
                first_fail_reason = "skipped %d on %s at %s=%.1f" % (
                    r['max_diff'], r['skip_axes'], label, value)
                gcmd.respond_info(
                    "  >>> FAIL at %s=%.1f → bracket [%.1f, %.1f]"
                    % (label, value, low, high))
                break
            low = value
            value += coarse_step

        if high is None:
            gcmd.respond_info(
                "Reached MAX %.1f without failure. Increase MAX." % high_bound)
            return low, None, results
        if low <= low_bound and high == low_bound + coarse_step:
            gcmd.respond_info(
                "First coarse step (%.1f) already failed — lower MIN."
                % high_bound)
            return low, first_fail_reason, results

        # Phase 2: Bisect
        gcmd.respond_info(
            "\n>>> Phase 2: Bisection <<<\n"
            "  Narrowing [%.1f, %.1f] until interval ≤ %.1f."
            % (low, high, min_step))
        last_fail_reason = first_fail_reason
        iter_count = 0
        while (high - low) > min_step + 0.0001 and iter_count < max_bisect:
            iter_count += 1
            raw_mid = (low + high) / 2.0
            mid = round(raw_mid / min_step) * min_step
            if mid <= low + 0.0001 or mid >= high - 0.0001:
                break
            r = measure_at(mid, 'bisect')
            results.append(r)
            if r['failed']:
                high = mid
                last_fail_reason = "skipped %d on %s at %s=%.1f" % (
                    r['max_diff'], r['skip_axes'], label, mid)
                gcmd.respond_info(
                    "  >>> %s=%.1f FAIL → [%.1f, %.1f] (%d/%d)"
                    % (label, mid, low, high, iter_count, max_bisect))
            else:
                low = mid
                gcmd.respond_info(
                    "  >>> %s=%.1f OK → [%.1f, %.1f] (%d/%d)"
                    % (label, mid, low, high, iter_count, max_bisect))

        # Phase 3: Verify
        gcmd.respond_info(
            "\n>>> Phase 3: Verification <<<\n"
            "  Confirming %s = %.1f with %d repeats."
            % (label, low, verify_repeats))
        # Caller's measure_at decides repeat count via phase='verify'
        v = measure_at(low, 'verify')
        v['phase'] = 'verify'
        results.append(v)
        if v['failed']:
            gcmd.respond_info(
                "  ⚠ Verify FAILED — value not stable. Re-run with more "
                "REPEAT.")

        return low, last_fail_reason, results

    # ─── Commands ─────────────────────────────────────────────────────

    def cmd_FIND_MAX_VELOCITY(self, gcmd):
        self._check_endstop_phase()
        testbench = bool(gcmd.get_int(
            'TESTBENCH', 1 if self.testbench_default else 0,
            minval=0, maxval=1))
        axis = gcmd.get('AXIS', self.default_axis).upper()
        if testbench and axis != 'X':
            raise gcmd.error(
                "Testbench mode supports AXIS=X only "
                "(single stepper wired to X).")
        if axis not in ('X', 'Y'):
            raise gcmd.error("AXIS must be X or Y")
        min_v = gcmd.get_float('MIN', 50.0, above=0.)
        max_v = gcmd.get_float('MAX', 500.0, above=min_v)
        coarse = gcmd.get_float('COARSE_STEP', 25.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 5.0, above=0.)
        accel = gcmd.get_float('ACCEL', 5000.0, above=0.)
        repeat = gcmd.get_int('REPEAT', 5, minval=1, maxval=50)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 20, minval=1, maxval=100)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        distance = gcmd.get('DISTANCE', 'full').lower()

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        if (max_v ** 2) / accel > ax_range:
            new_max = math.sqrt(ax_range * accel)
            gcmd.respond_info(
                "MAX=%.1f exceeds axis range. Clipped to %.1f mm/s "
                "(required distance > %.0f mm)."
                % (max_v, new_max, ax_range))
            max_v = new_max

        meta = self._build_meta('VELOCITY', axis,
                                {'MIN': min_v, 'MAX': max_v,
                                 'COARSE_STEP': coarse, 'MIN_STEP': min_step,
                                 'ACCEL': accel, 'REPEAT': repeat,
                                 'DISTANCE': distance})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._banner(gcmd, 'VELOCITY', axis, min_v, max_v, coarse, min_step,
                     repeat, accel=accel)

        self._set_limits(velocity=max_v * 1.5, accel=accel)
        self._ensure_homed([axis], testbench=testbench)
        results = []

        def measure_at(velocity, phase):
            self._set_limits(velocity=velocity, accel=accel)
            reps = verify_repeats if phase == 'verify' else repeat
            if distance == 'full':
                dist = ax_range
            else:
                dist = max(50., 4 * (velocity ** 2) / accel)
                dist = min(dist, ax_range)
            r = self._measure_step(
                gcmd, [axis], 'V', velocity,
                lambda: self._do_velocity_pattern(
                    axis, velocity, dist, reps, testbench=testbench),
                testbench=testbench)
            r['phase'] = phase
            r['accel'] = accel
            return r

        try:
            safe, reason, _ = self._adaptive_find_max(
                gcmd, results, min_v, max_v, coarse, min_step,
                verify_repeats, max_bisect, 'V', measure_at)
        finally:
            self._restore_limits()

        self._final_summary(gcmd, 'VELOCITY', axis, safe, results)
        self._save_report(results, meta, timestamp, reason,
                          no_html, 'velocity', gcmd)

    def cmd_FIND_MAX_ACCEL(self, gcmd):
        self._check_endstop_phase()
        testbench = bool(gcmd.get_int(
            'TESTBENCH', 1 if self.testbench_default else 0,
            minval=0, maxval=1))
        axis = gcmd.get('AXIS', self.default_axis).upper()
        if testbench and axis != 'X':
            raise gcmd.error(
                "Testbench mode supports AXIS=X only "
                "(single stepper wired to X).")
        if axis not in ('X', 'Y'):
            raise gcmd.error("AXIS must be X or Y")
        min_a = gcmd.get_float('MIN', 500.0, above=0.)
        max_a = gcmd.get_float('MAX', 50000.0, above=min_a)
        coarse = gcmd.get_float('COARSE_STEP', 2500.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 250.0, above=0.)
        speed = gcmd.get_float('SPEED', 200.0, above=0.)
        repeat = gcmd.get_int('REPEAT', 30, minval=1, maxval=200)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 50, minval=1, maxval=300)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        min_distance = gcmd.get_float('MIN_DISTANCE', 50.0, above=0.)

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        required = (speed ** 2) / min_a
        if required > ax_range:
            raise gcmd.error(
                "Need %.0f mm of axis range for SPEED=%.0f at MIN=%.0f, "
                "only %.0f mm available. Increase MIN or decrease SPEED."
                % (required, speed, min_a, ax_range))

        meta = self._build_meta('ACCEL', axis,
                                {'MIN': min_a, 'MAX': max_a,
                                 'COARSE_STEP': coarse, 'MIN_STEP': min_step,
                                 'SPEED': speed, 'REPEAT': repeat})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._banner(gcmd, 'ACCEL', axis, min_a, max_a, coarse, min_step,
                     repeat, speed=speed)

        self._set_limits(velocity=speed, accel=max_a * 1.5)
        self._ensure_homed([axis], testbench=testbench)
        results = []

        def measure_at(accel, phase):
            self._set_limits(velocity=speed, accel=accel)
            reps = verify_repeats if phase == 'verify' else repeat
            cruise = (speed ** 2) / accel
            dist = min(max(min_distance, 4 * cruise), ax_range)
            r = self._measure_step(
                gcmd, [axis], 'A', accel,
                lambda: self._do_velocity_pattern(
                    axis, speed, dist, reps, testbench=testbench),
                testbench=testbench)
            r['phase'] = phase
            r['speed'] = speed
            return r

        try:
            safe, reason, _ = self._adaptive_find_max(
                gcmd, results, min_a, max_a, coarse, min_step,
                verify_repeats, max_bisect, 'A', measure_at)
        finally:
            self._restore_limits()

        self._final_summary(gcmd, 'ACCEL', axis, safe, results)
        self._save_report(results, meta, timestamp, reason,
                          no_html, 'accel', gcmd)

    def cmd_FIND_MAX_SCV(self, gcmd):
        self._check_endstop_phase()
        if self.testbench_default or gcmd.get_int('TESTBENCH', 0,
                                                  minval=0, maxval=1):
            raise gcmd.error(
                "SCV test needs both X and Y motors — not available in "
                "testbench mode.")
        min_s = gcmd.get_float('MIN', 1.0, above=0.)
        max_s = gcmd.get_float('MAX', 20.0, above=min_s)
        coarse = gcmd.get_float('COARSE_STEP', 2.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 0.5, above=0.)
        speed = gcmd.get_float('SPEED', 200.0, above=0.)
        accel = gcmd.get_float('ACCEL', 5000.0, above=0.)
        repeat = gcmd.get_int('REPEAT', 3, minval=1, maxval=20)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 5, minval=1, maxval=20)
        corner_size = gcmd.get_float('CORNER_SIZE', 50.0, above=0.)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        meta = self._build_meta('SCV', 'XY',
                                {'MIN': min_s, 'MAX': max_s,
                                 'COARSE_STEP': coarse, 'MIN_STEP': min_step,
                                 'SPEED': speed, 'ACCEL': accel,
                                 'CORNER_SIZE': corner_size, 'REPEAT': repeat})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._banner(gcmd, 'SCV', 'XY', min_s, max_s, coarse, min_step,
                     repeat, speed=speed, accel=accel)

        self._set_limits(velocity=speed, accel=accel, scv=min_s)
        self._ensure_homed(['X', 'Y'])
        results = []

        def measure_at(scv, phase):
            self._set_limits(velocity=speed, accel=accel, scv=scv)
            reps = verify_repeats if phase == 'verify' else repeat
            r = self._measure_step(
                gcmd, ['X', 'Y'], 'SCV', scv,
                lambda: self._do_scv_pattern(speed, corner_size, reps))
            r['phase'] = phase
            r['speed'] = speed
            r['accel'] = accel
            return r

        try:
            safe, reason, _ = self._adaptive_find_max(
                gcmd, results, min_s, max_s, coarse, min_step,
                verify_repeats, max_bisect, 'SCV', measure_at)
        finally:
            self._restore_limits()

        self._final_summary(gcmd, 'SCV', 'XY', safe, results)
        self._save_report(results, meta, timestamp, reason,
                          no_html, 'scv', gcmd)

    def cmd_BENCHMARK(self, gcmd):
        self._check_endstop_phase()
        if self.testbench_default or gcmd.get_int('TESTBENCH', 0,
                                                  minval=0, maxval=1):
            raise gcmd.error(
                "Benchmark needs both X and Y motors — not available in "
                "testbench mode.")
        speed = gcmd.get_float('SPEED', 300.0, above=0.)
        accel = gcmd.get_float('ACCEL', 10000.0, above=0.)
        iterations = gcmd.get_int('ITERATIONS', 3, minval=1, maxval=50)
        bound = gcmd.get_float('BOUND', 40.0, above=0.)
        small = gcmd.get_float('SMALLPATTERNSIZE', 20.0, above=0.)
        scv = gcmd.get_float('SCV', 5.0, above=0.)
        zpos = gcmd.get_float('ZPOS', self.z_pos, minval=0.)
        seed = gcmd.get_int('SEED', 12345)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)

        meta = self._build_meta('BENCHMARK', 'XY',
                                {'SPEED': speed, 'ACCEL': accel,
                                 'ITERATIONS': iterations,
                                 'BOUND': bound, 'SCV': scv, 'SEED': seed})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        gcmd.respond_info(
            "===== Speed Test v%s — BENCHMARK =====\n"
            "Plugin by Steven (Fragmon) — Crydteam\n"
            "Speed=%.0f mm/s | Accel=%.0f mm/s² | SCV=%.1f | ITER=%d | "
            "SEED=%d\n"
            "----------------------------------------"
            % (MODULE_VERSION, speed, accel, scv, iterations, seed))

        self._set_limits(velocity=speed, accel=accel, scv=scv)
        self._ensure_homed(['X', 'Y'])
        results = []

        def do_bench():
            self._do_benchmark_pattern(
                speed, iterations, bound, small, zpos, seed)
        try:
            r = self._measure_step(
                gcmd, ['X', 'Y'], 'ITER', iterations, do_bench)
            r['phase'] = 'benchmark'
            r['speed'] = speed
            r['accel'] = accel
            r['scv'] = scv
            results.append(r)
        finally:
            self._restore_limits()

        verdict = "FAILED — skipped steps detected" if r['failed'] \
            else "PASSED — no skipped steps"
        gcmd.respond_info(
            "========== BENCHMARK RESULT ==========\n"
            "%s\n"
            "%d iterations at %.0f mm/s / %.0f mm/s² / SCV=%.1f\n"
            "======================================"
            % (verdict, iterations, speed, accel, scv))
        self._save_report(results, meta, timestamp,
                          "skipped steps" if r['failed'] else None,
                          no_html, 'benchmark', gcmd)

    def cmd_STATUS(self, gcmd):
        gcmd.respond_info(
            "===== Speed Test v%s — STATUS =====\n"
            "Structure: %s\n"
            "Default axis: %s\n"
            "Testbench mode (default): %s\n"
            "Z position for tests: %.1f mm\n"
            "Margin from axis ends: %.1f mm\n"
            "TMC SG monitoring: %s\n"
            "Output dir: %s"
            % (MODULE_VERSION, self.structure, self.default_axis,
               "on (only X used, no Y/Z homing)"
               if self.testbench_default else "off",
               self.z_pos, self.margin,
               "on" if self.monitor_tmc else "off",
               self.output_dir))
        ep = self.printer.lookup_object('endstop_phase', None)
        gcmd.respond_info(
            "endstop_phase module: %s"
            % ("present ✓" if ep is not None else "MISSING ✗"))
        for axis in ('X', 'Y'):
            try:
                lo, hi, mid, rng = self._get_axis_bounds(axis)
                info = self._lookup_tmc_for_axis(axis)
                tmc_str = ("TMC: %s" % info[0]) if info else "TMC: none"
                gcmd.respond_info(
                    "  %s: usable range [%.1f, %.1f] mm (width %.1f), %s"
                    % (axis, lo, hi, rng, tmc_str))
            except Exception as e:
                gcmd.respond_info("  %s: bounds error: %s" % (axis, e))

    # ─── Reporting ────────────────────────────────────────────────────

    def _build_meta(self, kind, axis, params):
        v, a, scv = self._get_printer_limits()
        meta = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'plugin': '%s v%s' % (MODULE_NAME, MODULE_VERSION),
            'test_kind': kind,
            'axis': axis,
            'structure': self.structure,
            'printer_max_velocity': '%.1f mm/s' % v,
            'printer_max_accel': '%.1f mm/s²' % a,
            'printer_scv': '%.2f mm/s' % scv,
        }
        for k, val in params.items():
            meta[k] = "%.3f" % val if isinstance(val, float) else str(val)
        return meta

    def _banner(self, gcmd, kind, axis, lo, hi, coarse, min_step, repeat,
                speed=None, accel=None):
        extra = []
        if speed is not None:
            extra.append("Speed: %.1f mm/s" % speed)
        if accel is not None:
            extra.append("Accel: %.0f mm/s²" % accel)
        gcmd.respond_info(
            "===== Speed Test v%s — %s on %s =====\n"
            "Plugin by Steven (Fragmon) — Crydteam\n"
            "Range: %.1f → %.1f | Coarse: %.1f | Min: %.1f | Repeat: %d\n"
            "%s\n"
            "Algorithm: ADAPTIVE BISECTION (Coarse → Bisect → Verify)\n"
            "================================================"
            % (MODULE_VERSION, kind, axis, lo, hi, coarse, min_step, repeat,
               " | ".join(extra) if extra else ""))

    def _final_summary(self, gcmd, kind, axis, safe, results):
        verify = [r for r in results if r.get('phase') == 'verify']
        verify_ok = verify and not verify[-1].get('failed')
        if verify_ok:
            quality = "VERIFIED OK"
        elif verify:
            quality = "VERIFY FAILED — value may be unstable"
        else:
            quality = "no verification phase"
        gcmd.respond_info(
            "\n========== %s RESULT ==========\n"
            "Axis: %s | Test: %s\n"
            "Maximum safe %s: %.1f\n"
            "Quality: %s\n"
            "----------------------------------\n"
            "Slicer / printer.cfg recommendation:\n"
            "  Conservative (80%%): %.1f\n"
            "  Aggressive   (90%%): %.1f\n"
            "================================="
            % (kind, axis, kind, kind, safe, quality,
               safe * 0.8, safe * 0.9))

    def _write_csv(self, path, results, meta, kind):
        with open(path, 'w') as f:
            f.write("# Speed Test v%s results — %s\n" % (MODULE_VERSION, kind))
            f.write("# Plugin by Steven (Fragmon) — Crydteam\n")
            f.write("# YouTube: https://www.youtube.com/@crydteamprinting\n")
            for k, v in meta.items():
                f.write("# %s: %s\n" % (k, v))
            f.write("phase,value,failed,max_diff,skip_axes,"
                    "x_sg_min,x_sg_median,y_sg_min,y_sg_median\n")
            for r in results:
                tmc = r.get('tmc') or {}
                x = tmc.get('X') or {}
                y = tmc.get('Y') or {}
                f.write("%s,%.3f,%d,%d,%s,%s,%s,%s,%s\n" % (
                    r.get('phase', 'coarse'),
                    r['value'],
                    1 if r['failed'] else 0,
                    r['max_diff'],
                    r['skip_axes'],
                    x.get('min', ''), x.get('median', ''),
                    y.get('min', ''), y.get('median', ''),
                ))

    def _write_html(self, path, results, meta, limit_reason, kind):
        values = [r['value'] for r in results]
        phases = [r.get('phase', 'coarse') for r in results]
        diffs = [r['max_diff'] for r in results]
        pass_y = [r['value'] if not r['failed'] else None for r in results]
        fail_y = [r['value'] if r['failed'] else None for r in results]

        def sg_series(axis, key):
            out = []
            for r in results:
                tmc = (r.get('tmc') or {}).get(axis)
                out.append(tmc.get(key) if tmc else None)
            return out
        x_sg_med = sg_series('X', 'median')
        x_sg_min = sg_series('X', 'min')
        y_sg_med = sg_series('Y', 'median')
        y_sg_min = sg_series('Y', 'min')
        has_tmc = any(v is not None for v in
                      x_sg_med + x_sg_min + y_sg_med + y_sg_min)

        if limit_reason:
            summary_html = (
                '<div class="summary"><h2>Result</h2>'
                '<p>Stop reason: <strong>%s</strong></p></div>'
                % limit_reason)
        else:
            summary_html = (
                '<div class="summary"><h2>Result</h2>'
                '<p>Test completed without trigger.</p></div>')

        meta_html = ''.join(
            '<div><strong>%s:</strong> %s</div>' % (k, v)
            for k, v in meta.items())

        rows = []
        for r in results:
            tmc = r.get('tmc') or {}
            x = tmc.get('X') or {}
            y = tmc.get('Y') or {}
            cls = ' style="background:#ffcdd2"' if r['failed'] else ''
            rows.append(
                "<tr%s><td>%s</td><td>%.2f</td><td>%s</td><td>%d</td>"
                "<td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
                % (cls, r.get('phase', '-'), r['value'],
                   'FAIL' if r['failed'] else 'OK',
                   r['max_diff'], r['skip_axes'] or '-',
                   _fmt(x.get('min')), _fmt(x.get('median')),
                   _fmt(y.get('min')), _fmt(y.get('median'))))
        table = (
            "<table><thead><tr><th>Phase</th><th>Value</th><th>Result</th>"
            "<th>Lost steps</th><th>Axis</th>"
            "<th>X SG min</th><th>X SG med</th>"
            "<th>Y SG min</th><th>Y SG med</th>"
            "</tr></thead><tbody>" + "".join(rows) + "</tbody></table>")

        tmc_block = ""
        tmc_script = ""
        if has_tmc:
            tmc_block = """
<div class="chart-container">
  <h2>TMC StallGuard during moves (lower = more load)</h2>
  <canvas id="tmcChart"></canvas>
</div>
"""
            tmc_script = """
new Chart(document.getElementById('tmcChart'), {
    type: 'line',
    data: { labels: values, datasets: [
        {label:'X SG min', data: %(xsm)s, borderColor:'#1976d2',
         borderDash:[4,3], fill:false, pointRadius:3},
        {label:'X SG median', data: %(xmd)s, borderColor:'#0d47a1',
         fill:false, borderWidth:2, pointRadius:4},
        {label:'Y SG min', data: %(ysm)s, borderColor:'#e53935',
         borderDash:[4,3], fill:false, pointRadius:3},
        {label:'Y SG median', data: %(ymd)s, borderColor:'#b71c1c',
         fill:false, borderWidth:2, pointRadius:4},
    ]},
    options: { ...commonOptions, scales: { ...commonOptions.scales,
        y: { title: { display:true, text:'SG value (0-510)' } } } },
});
""" % {
                'xsm': json.dumps(x_sg_min),
                'xmd': json.dumps(x_sg_med),
                'ysm': json.dumps(y_sg_min),
                'ymd': json.dumps(y_sg_med),
            }

        html_tpl = """<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<title>Speed Test (%(kind)s) — %(ts)s</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-annotation@3.0.1"></script>
<style>
body { font-family: system-ui, sans-serif; max-width: 1200px;
       margin: 20px auto; padding: 0 20px; color: #333; }
h1 { color: #1565c0; }
.meta { background:#f5f5f5; padding:15px; border-radius:8px;
        margin-bottom:20px; display:grid;
        grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
        gap:8px; font-size:14px; }
.summary { background:#e3f2fd; padding:15px; border-radius:8px;
           margin-bottom:20px; border-left:4px solid #1976d2; }
.summary h2 { margin:0 0 10px 0; color:#1976d2; }
.chart-container { background:white; padding:20px; border-radius:8px;
                   margin-bottom:20px;
                   box-shadow:0 2px 4px rgba(0,0,0,0.1); }
.footer { text-align:center; color:#666; padding:20px; font-size:13px; }
.footer a { color:#1976d2; }
table { width:100%%; border-collapse:collapse; font-size:13px; }
th, td { padding:8px 12px; border:1px solid #ddd; text-align:right; }
th { background:#f5f5f5; }
</style></head><body>
<h1>Speed Test — %(kind)s</h1>
<p style="color:#666;margin-top:-10px;">
  Plugin by Steven (Fragmon) — Crydteam ·
  <a href="https://www.youtube.com/@crydteamprinting" target="_blank">YouTube: @crydteamprinting</a></p>

<div class="meta">%(meta_html)s</div>
%(summary_html)s

<div class="chart-container">
  <h2>Lost steps vs. test value</h2>
  <p>OK points sit on the x-axis. FAIL points show how many microsteps
     were lost — higher is worse.</p>
  <canvas id="diffChart"></canvas>
</div>

%(tmc_block)s

<div class="chart-container">
  <h2>Data Table</h2>
  %(table)s
</div>

<div class="footer">
  <p>Generated by <strong>%(plugin)s</strong> at %(ts)s</p>
</div>

<script>
const values = %(values)s;
const phases = %(phases)s;
const diffs = %(diffs)s;

const phaseAnn = (() => {
    const labels = { coarse:'Coarse', bisect:'Bisection', verify:'Verify',
                     benchmark:'Benchmark' };
    const colors = { coarse:'#90a4ae', bisect:'#fb8c00', verify:'#43a047',
                     benchmark:'#5e35b1' };
    const ann = {};
    for (let i = 1; i < phases.length; i++) {
        if (phases[i] !== phases[i-1]) {
            const x = i - 0.5;
            ann['p_'+i] = { type:'line', xMin:x, xMax:x,
                borderColor: colors[phases[i]]||'#999',
                borderWidth:2, borderDash:[6,4],
                label: { display:true, content: labels[phases[i]]||phases[i],
                         position:'start',
                         backgroundColor: colors[phases[i]]||'#999',
                         color:'#fff', font:{size:11, weight:'bold'},
                         padding:{top:2,bottom:2,left:6,right:6}, yAdjust:-2 } };
        }
    }
    if (phases.length > 0) {
        ann['p_s'] = { type:'line', xMin:-0.5, xMax:-0.5,
            borderColor:'rgba(0,0,0,0)', borderWidth:0,
            label:{display:true, content: labels[phases[0]]||phases[0],
                   position:'start',
                   backgroundColor: colors[phases[0]]||'#999',
                   color:'#fff', font:{size:11, weight:'bold'},
                   padding:{top:2,bottom:2,left:6,right:6},
                   yAdjust:-2, xAdjust:30 } };
    }
    return ann;
})();

const commonOptions = {
    responsive:true,
    interaction:{mode:'index', intersect:false},
    scales:{ x:{ title:{display:true, text:'Test value'} } },
    plugins:{
        legend:{position:'top'},
        annotation:{annotations: phaseAnn},
    },
};

new Chart(document.getElementById('diffChart'), {
    type:'bar',
    data: { labels: values, datasets:[
        { label:'Lost steps', data: diffs, backgroundColor:'#e57373',
          borderColor:'#c62828', borderWidth:1 },
    ]},
    options: { ...commonOptions, scales:{ ...commonOptions.scales,
        y:{ title:{display:true, text:'Lost microsteps'}, beginAtZero:true } } },
});
%(tmc_script)s
</script>
</body></html>"""
        rendered = html_tpl % {
            'kind': kind.upper(),
            'ts': meta.get('timestamp', '-'),
            'plugin': '%s v%s' % (MODULE_NAME, MODULE_VERSION),
            'meta_html': meta_html,
            'summary_html': summary_html,
            'table': table,
            'tmc_block': tmc_block,
            'values': json.dumps(values),
            'phases': json.dumps(phases),
            'diffs': json.dumps(diffs),
            'tmc_script': tmc_script,
        }
        with open(path, 'w') as f:
            f.write(rendered)

    def _save_report(self, results, meta, timestamp, limit_reason,
                     no_html, kind, gcmd):
        if not results:
            return
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            csv_path = os.path.join(
                self.output_dir, 'speed_%s_%s.csv' % (kind, timestamp))
            self._write_csv(csv_path, results, meta, kind)
            if gcmd is not None:
                gcmd.respond_info("CSV saved: %s" % csv_path)
            if not no_html:
                html_path = os.path.join(
                    self.output_dir,
                    'speed_%s_%s.html' % (kind, timestamp))
                self._write_html(html_path, results, meta, limit_reason, kind)
                if gcmd is not None:
                    gcmd.respond_info("HTML saved: %s" % html_path)
        except Exception as e:
            if gcmd is not None:
                gcmd.respond_info(
                    "Warning: report write failed: %s" % e)
            logging.exception("speed_test: report write failed")


def _fmt(v, fs="%.0f"):
    if v is None or v == '':
        return '-'
    try:
        return fs % v
    except Exception:
        return str(v)


def load_config(config):
    return SpeedTest(config)
