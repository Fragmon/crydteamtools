# Speed Test — Adaptive max velocity / accel / SCV detection for steppers
# Detects skipped steps by reading the stepper MCU position directly
# (no endstop_phase module needed) and narrows the safe limit with
# adaptive bracket search.
#
# Plugin by Steven (Fragmon) — Crydteam
# YouTube: https://www.youtube.com/@crydteamprinting
#
# License: GPLv3

import logging
import math
import os
import random
import time
import json

MODULE_NAME = "Speed Test"
MODULE_VERSION = "1.6"
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
        # Skipped-step tolerance, in FULL motor steps. A move is a "skip"
        # when the stepper's MCU position shifts by more than this across a
        # re-home. ~1 step of homing jitter is normal without endstop_phase,
        # so the default leaves headroom; real stalls lose far more.
        self.max_missed = config.getfloat('max_missed', 1.5, above=0.)
        # Testbench mode: only X-stepper connected, no Y, no Z. Skips all
        # Y/Z homing and ignores Y in skip checks and TMC monitoring.
        # Per-command TESTBENCH=1/0 override is honored.
        self.testbench_default = config.getboolean('testbench', False)
        # Hard safety cap for the OPTIMAL_CURRENT search. 0 = no cap
        # (use TMC defaults / per-command MAX_CURRENT). When set, the
        # plugin will never raise current above this value, even if a
        # command parameter asks for more.
        self.max_current = config.getfloat('max_current', 0.0, minval=0.)

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
            'SPEED_TEST_FIND_ENVELOPE',
            self.cmd_FIND_ENVELOPE,
            desc='Map the velocity-acceleration envelope: find max safe '
                 'accel at several velocities (they are physically coupled)')
        self.gcode.register_command(
            'SPEED_TEST_FIND_OPTIMAL_CURRENT',
            self.cmd_FIND_OPTIMAL_CURRENT,
            desc='Find the lowest TMC run_current that still passes '
                 'a SPEED/ACCEL target. Starts at MAX_CURRENT and '
                 'searches downward.')
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
        """Read the stepper's raw MCU step position straight from the
        kinematics — no endstop_phase needed. Each unit is one microstep,
        so a full motor step = `microsteps` units."""
        try:
            toolhead = self.printer.lookup_object('toolhead')
            kin = toolhead.get_kinematics()
            stepper_name = 'stepper_' + axis.lower()
            for s in kin.get_steppers():
                if s.get_name() == stepper_name:
                    return s.get_mcu_position()
        except Exception:
            return None
        return None

    def _store_mcu_pos(self, axes):
        for axis in axes:
            pos = self._read_mcu_pos(axis)
            if pos is not None:
                self._last_mcu_pos[axis] = pos

    def _check_skip(self, axes):
        """Returns list of (axis, abs_diff) for axes that lost steps.

        Compares the stepper MCU position before/after a re-home. A shift
        larger than max_missed full steps (converted to microsteps) is a
        skip. Reading the position directly avoids the endstop_phase
        module, which aborts homing with 'incorrect phase' the moment a
        step is lost — exactly what we want to measure, not crash on.
        """
        skips = []
        for axis in axes:
            new_pos = self._read_mcu_pos(axis)
            old_pos = self._last_mcu_pos.get(axis)
            if new_pos is None or old_pos is None:
                continue
            diff = abs(new_pos - old_pos)
            threshold = self.max_missed * self._get_microsteps(axis)
            if diff > threshold:
                skips.append((axis, diff))
            self._last_mcu_pos[axis] = new_pos
        return skips

    def _check_ready(self):
        # Skip detection reads the stepper MCU position directly, so no
        # endstop_phase module is required. If it IS loaded, warn: it
        # aborts homing with "incorrect phase" as soon as the motor loses
        # a step (its TMC phase cross-check), which kills the test.
        ep = self.printer.lookup_object('endstop_phase', None)
        if ep is not None:
            self.gcode.respond_info(
                "speed_test: WARNING — [endstop_phase] is configured. It "
                "aborts homing with 'incorrect phase' the moment the motor "
                "loses a step, which will abort this test. Remove "
                "[endstop_phase] (and any [endstop_phase stepper_*]) from "
                "printer.cfg and FIRMWARE_RESTART.")
        if self._read_mcu_pos(self.default_axis) is None:
            raise self.gcode.error(
                "speed_test: cannot read the %s stepper MCU position. "
                "Check that stepper_%s exists and the printer is configured."
                % (self.default_axis, self.default_axis.lower()))

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
        self.gcode.run_script_from_command(
            "G1 %s%.3f F%.1f" % (axis, pos, feed))

    def _park_with_zlift(self, axis, pos, velocity, testbench):
        """Park the toolhead at `pos` on `axis`. In testbench mode this is
        a plain axis move (no Z motor); otherwise it lifts Z to z_pos while
        moving. The Z-lift arm uses raw velocity*60 (no 60 mm/min floor),
        matching the original inline behaviour."""
        if testbench:
            self._move_to_axis(axis, pos, velocity)
        else:
            self.gcode.run_script_from_command(
                "G1 %s%.3f Z%.3f F%.1f"
                % (axis, pos, self.z_pos, velocity * 60.))

    def _do_velocity_pattern(self, axis, velocity, distance, repeat,
                             testbench=False):
        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        dist = min(distance, ax_range)
        low = ax_mid - dist / 2.0
        high = ax_mid + dist / 2.0
        # Park at middle first — skip Z in testbench mode (no Z motor)
        self._park_with_zlift(axis, ax_mid, velocity, testbench)
        for _ in range(repeat):
            self._move_to_axis(axis, low, velocity)
            self._move_to_axis(axis, high, velocity)
            self._move_to_axis(axis, ax_mid, velocity)
        self.gcode.run_script_from_command("M400")

    def _do_accel_pattern(self, axis, velocity, min_dist, max_dist,
                          repeat, seed=12345, testbench=False,
                          short_bias=2.0):
        """Movement pattern for the accel test.

        The main stress on a motor during the accel test comes from
        rapid direction reversals — short back-and-forth moves where
        the motor has to brake hard and accelerate again immediately.
        So the distance distribution is biased toward the small end of
        [min_dist, max_dist] via a power-law skew (short_bias > 1 →
        more short moves, fewer long ones).

        Each move also starts at a random offset along the axis so the
        test covers varied positions, not a tight loop on the same
        coordinate.
        """
        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)

        rng = random.Random(seed)

        # Park at middle first — skip Z in testbench mode
        self._park_with_zlift(axis, ax_mid, velocity, testbench)

        # Clamp distance bounds to axis range and sanity-check
        max_dist = min(max_dist, ax_range)
        min_dist = max(0.5, min(min_dist, max_dist))

        for _ in range(repeat):
            # Power-law biased random: r^short_bias maps uniform [0,1] to
            # a distribution that concentrates near 0 (short distances).
            # short_bias=1 → uniform; 2 → quadratic skew; 3 → cubic, etc.
            r = rng.random()
            biased = r ** short_bias
            dist = min_dist + (max_dist - min_dist) * biased
            slack = max(0.0, ax_range - dist)
            offset = rng.uniform(0.0, slack)
            low = ax_min + offset
            high = low + dist
            # Three-leg motion: low → high → mid (the high→mid leg is
            # the half-distance reversal stress)
            self._move_to_axis(axis, low, velocity)
            self._move_to_axis(axis, high, velocity)
            self._move_to_axis(axis, ax_mid, velocity)
        self.gcode.run_script_from_command("M400")

    def _do_jab_pattern(self, axis, velocity, accel, repeat,
                        testbench=False):
        """Stage-1 fast, stall-safe 'jab' move (idea from Anonoei's
        klipper_auto_speed).

        The move length is sized to the motion profile — V²/A, the
        triangle distance that just reaches the target velocity at its
        peak — and anchored near the home end (10 % into the usable
        travel). Three wins over a full-axis sweep:
          • each test is short, so the bracketing search is fast,
          • a stalled motor only grinds that short distance before the
            commanded move ends, instead of being driven across the whole
            axis into the limit, and
          • sitting near home keeps the approach and the re-home after
            each test short, instead of driving out to the far axis end
            and back every time.

        It's a quick out-and-back (reversal at both turns for some stress)
        running from the 10 % point toward the far end and back — used
        only to bracket the limit. The thorough reversal-stress pattern
        validates the final value in stage 2.
        """
        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        # Triangle distance that just touches V at the peak: V²/A.
        dist = (velocity ** 2) / accel
        dist = max(5.0, min(dist, ax_range))
        # Anchor near home: start 10 % into the usable travel and jab
        # toward the far end (away from the home limit), then back. Short
        # approach + short re-home. For a large jab that wouldn't fit from
        # the 10 % mark, shift it down just enough to stay on the axis.
        low = ax_min + 0.10 * ax_range
        high = low + dist
        if high > ax_max:
            high = ax_max
            low = max(ax_min, high - dist)
        # Park at the near (home-side) point first — skip Z in testbench.
        self._park_with_zlift(axis, low, velocity, testbench)
        for _ in range(repeat):
            self._move_to_axis(axis, high, velocity)
            self._move_to_axis(axis, low, velocity)
        self.gcode.run_script_from_command("M400")

    def _build_print_sim_moves(self, axis, n_infill, n_travel, seed):
        """Build the stage-3 'simulated print' as a list of absolute target
        coordinates — a print-like sequence of bursts of short infill
        zigzag (rapid reversals — the hardest stress), interleaved with
        medium perimeter passes and travels.

        Two safety choices keep a stall from crashing the head:
          • the whole run is confined to the CENTRE of the axis, leaving a
            generous pad at each end, so lost steps drift the head but
            don't drive it into the axis limit, and
          • travels are capped at that central region's width — no
            full-axis sweeps.
        Returned targets are executed in chunks by the caller, which
        re-homes between chunks and aborts on the first lost-step chunk,
        so grinding is bounded to one chunk.
        """
        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        pad = 0.15 * ax_range
        rmin, rmax = ax_min + pad, ax_max - pad
        region = max(10.0, rmax - rmin)
        rng = random.Random(seed)
        short_hi = max(2.0, min(8.0, region * 0.12))
        med_lo = short_hi
        med_hi = max(med_lo + 1.0, min(40.0, region * 0.45))
        travel_lo = min(40.0, region * 0.45)
        lengths = []
        infill_left = int(n_infill)
        travel_left = int(n_travel)
        guard = 0
        while (infill_left > 0 or travel_left > 0) and guard < 100000:
            guard += 1
            for _ in range(rng.randint(2, 4)):
                lengths.append(rng.uniform(med_lo, med_hi))
            if infill_left > 0:
                burst = min(infill_left, rng.randint(15, 40))
                for _ in range(burst):
                    lengths.append(rng.uniform(1.0, short_hi))
                infill_left -= burst
            if travel_left > 0 and (infill_left <= 0 or rng.random() < 0.7):
                lengths.append(rng.uniform(travel_lo, region))
                travel_left -= 1
        # Bounce the lengths into absolute targets within [rmin, rmax].
        targets = []
        pos = (rmin + rmax) / 2.0
        direction = 1
        for d in lengths:
            d = min(d, region)
            target = pos + direction * d
            if target > rmax or target < rmin:
                direction = -direction
                target = pos + direction * d
            target = min(max(target, rmin), rmax)
            targets.append(target)
            pos = target
            direction = -direction
        return targets

    def _run_axis_moves(self, axis, targets, velocity, testbench=False):
        """Execute a list of absolute target coordinates on one axis. The
        first move lifts Z (non-testbench) so the run clears the bed."""
        first = True
        for t in targets:
            if first:
                self._park_with_zlift(axis, t, velocity, testbench)
            else:
                self._move_to_axis(axis, t, velocity)
            first = False
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
                      testbench=False, cruise_fraction=None):
        """Run a movement pattern, re-home, return skip info + TMC stats."""
        self._last_cruise_fraction = cruise_fraction
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

        cruise_str = ""
        cf = getattr(self, '_last_cruise_fraction', None)
        if cf is not None:
            cruise_str = " | cruise=%.0f%%" % (cf * 100)
            self._last_cruise_fraction = None
        v_cur, a_cur, scv_cur = self._current_limits()
        gcmd.respond_info(
            "  %s = %.1f  →  %s%s | accel=%.0f velo=%.0f scv=%.1f"
            % (label, value,
               "FAILED (%d steps on %s)" % (max_diff, skip_axes)
               if failed else "OK",
               cruise_str,
               a_cur, v_cur, scv_cur))

        return {
            'value': value,
            'failed': failed,
            'max_diff': max_diff,
            'skip_axes': skip_axes,
            'tmc': tmc_stats,
        }

    def _current_limits(self):
        """Live motion limits from the toolhead — reflects the
        SET_VELOCITY_LIMIT values active during the current test."""
        try:
            th = self.printer.lookup_object('toolhead')
            return (th.max_velocity, th.max_accel,
                    th.square_corner_velocity)
        except Exception:
            return self._get_printer_limits()

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

    def _binary_search_max(self, gcmd, results, low_bound, high_bound,
                           accuracy, max_iter, label, measure_at):
        """Relative-accuracy binary search (adapted from Anonoei's
        klipper_auto_speed) — no fixed step size.

        Probes the first guess at 1/3 of the bracket (biased low for
        safety), then bisects: a passing value raises the floor, a failing
        one lowers the ceiling. Stops when the last guess is within
        `accuracy` (a fraction, e.g. 0.05 = 5%) of a bracket bound, so the
        resolution scales with the magnitude instead of an absolute step.

        measure_at(value, phase) is called with phase='search' (the caller
        uses that to pick the fast stall-safe pattern). After bracketing,
        the candidate is re-tested so the returned value's most recent
        result is always a PASS. Returns that freshly-confirmed value — or
        None if nothing in the bracket can be confirmed (so the caller
        never treats an untested or just-failed value as safe).
        """
        low = low_bound
        high = high_bound
        guess = low + (high - low) / 3.0
        last_pass = None
        measured = None
        it = 0
        while it < max_iter:
            it += 1
            r = measure_at(guess, 'search')
            results.append(r)
            valid = not r['failed']
            if valid:
                last_pass = guess if last_pass is None else max(last_pass,
                                                                guess)
            # Converged when the guess sits within `accuracy` of the
            # bracket — checked against the bounds BEFORE this result
            # narrows them (skipped on the very first probe).
            converged = (measured is not None
                         and (guess * (1.0 + accuracy) > high
                              or guess * (1.0 - accuracy) < low))
            measured = guess
            if valid:
                low = guess
            else:
                high = guess
            gcmd.respond_info(
                "  >>> %s=%.1f %s → bracket [%.0f, %.0f] (%d/%d)"
                % (label, guess, "OK" if valid else "FAIL",
                   low, high, it, max_iter))
            if converged:
                break
            guess = (low + high) / 2.0
        # The bracketing may have ended on a FAILING probe, leaving the
        # best passing value untested for a couple of steps. Confirm the
        # candidate with a fresh test so the value handed to stage 2 was
        # JUST validated — never one whose most recent result was a fail.
        # If it can't be confirmed, step down until one passes (or give up
        # → None, so the caller never treats an unconfirmed value as safe).
        cand = last_pass if last_pass is not None else low_bound
        tries = 0
        while tries < max_iter:
            tries += 1
            r = measure_at(cand, 'search')
            results.append(r)
            gcmd.respond_info(
                "  >>> %s=%.1f %s (confirm)"
                % (label, cand, "OK" if not r['failed'] else "FAIL"))
            if not r['failed']:
                return cand
            cand = cand * (1.0 - max(accuracy, 0.05))
            if cand < low_bound:
                break
        return None

    # ─── Commands ─────────────────────────────────────────────────────

    def _parse_testbench_axis(self, gcmd,
                              tb_note=" (single stepper wired to X)."):
        """Shared prologue for the per-axis commands: resolve TESTBENCH and
        AXIS, and validate (testbench ⇒ X only; axis ∈ {X, Y}). `tb_note`
        tailors the testbench error message per command."""
        testbench = bool(gcmd.get_int(
            'TESTBENCH', 1 if self.testbench_default else 0,
            minval=0, maxval=1))
        axis = gcmd.get('AXIS', self.default_axis).upper()
        if testbench and axis != 'X':
            raise gcmd.error("Testbench mode supports AXIS=X only" + tb_note)
        if axis not in ('X', 'Y'):
            raise gcmd.error("AXIS must be X or Y")
        return testbench, axis

    def cmd_FIND_MAX_VELOCITY(self, gcmd):
        self._check_ready()
        testbench, axis = self._parse_testbench_axis(gcmd)
        min_v = gcmd.get_float('MIN', 50.0, above=0.)
        max_v = gcmd.get_float('MAX', 500.0, above=min_v)
        coarse = gcmd.get_float('COARSE_STEP', 25.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 5.0, above=0.)
        # ACCEL=0 (default) → auto-compute from MAX_V and axis range so
        # that at least CRUISE_RATIO of the move is spent at MAX velocity.
        accel_param = gcmd.get_float('ACCEL', 0.0, minval=0.)
        cruise_ratio = gcmd.get_float('CRUISE_RATIO', 0.5,
                                       minval=0.0, maxval=0.95)
        repeat = gcmd.get_int('REPEAT', 5, minval=1, maxval=50)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 20, minval=1, maxval=100)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        distance = gcmd.get('DISTANCE', 'full').lower()

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)

        # ─── Acceleration ⇄ velocity sizing ───
        # Triangle (no cruise) needs V²/A. To keep CRUISE_RATIO at V, need
        # total distance D ≥ V² / (A · (1 − CRUISE_RATIO)).
        # → A ≥ V² / (D · (1 − CRUISE_RATIO))
        # → V ≤ √(D · A · (1 − CRUISE_RATIO))
        non_cruise = max(1 - cruise_ratio, 0.05)
        if accel_param <= 0.0:
            # Auto-compute acceleration so MAX velocity hits cruise target
            need_accel = (max_v ** 2) / (ax_range * non_cruise)
            accel = math.ceil(need_accel / 500.0) * 500.0
            gcmd.respond_info(
                "Auto-set ACCEL = %.0f mm/s² so MAX=%.0f mm/s has ≥%.0f%% "
                "cruise on %.0f mm of usable %s travel."
                % (accel, max_v, cruise_ratio * 100, ax_range, axis))
        else:
            accel = accel_param
            v_limit = math.sqrt(ax_range * accel * non_cruise)
            if max_v > v_limit:
                gcmd.respond_info(
                    "MAX=%.0f mm/s exceeds the velocity that keeps ≥%.0f%% "
                    "cruise at ACCEL=%.0f mm/s² on %.0f mm of %s travel. "
                    "Clipped MAX to %.0f mm/s.\n"
                    "To test higher, increase ACCEL or omit it for "
                    "auto-sizing."
                    % (max_v, cruise_ratio * 100, accel, ax_range, axis,
                       v_limit))
                max_v = v_limit
            if min_v >= max_v:
                raise gcmd.error(
                    "After clipping, MIN (%.0f) >= MAX (%.0f). Increase "
                    "ACCEL or lower MIN." % (min_v, max_v))

        meta = self._build_meta('VELOCITY', axis,
                                {'MIN': min_v, 'MAX': max_v,
                                 'COARSE_STEP': coarse, 'MIN_STEP': min_step,
                                 'ACCEL': accel, 'REPEAT': repeat,
                                 'DISTANCE': distance})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._banner(gcmd, 'VELOCITY', axis, min_v, max_v, repeat,
                     accel=accel)

        self._set_limits(velocity=max_v * 1.5, accel=accel)
        self._ensure_homed([axis], testbench=testbench)
        results = []

        def measure_at(velocity, phase):
            self._set_limits(velocity=velocity, accel=accel)
            reps = verify_repeats if phase == 'verify' else repeat
            # Triangle distance V²/A. Total move distance to reach the
            # configured cruise ratio: V²/A / (1 − cruise_ratio).
            triangle = (velocity ** 2) / accel
            min_for_cruise = triangle / non_cruise
            if distance == 'full':
                dist = ax_range
            else:
                # Use enough distance to honor cruise ratio; cap at axis.
                dist = min(max(50., min_for_cruise), ax_range)
            actual_cruise = max(0.0, 1.0 - triangle / dist)
            r = self._measure_step(
                gcmd, [axis], 'V', velocity,
                lambda: self._do_velocity_pattern(
                    axis, velocity, dist, reps, testbench=testbench),
                testbench=testbench,
                cruise_fraction=actual_cruise)
            r['phase'] = phase
            r['accel'] = accel
            r['cruise_fraction'] = actual_cruise
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
        self._check_ready()
        testbench, axis = self._parse_testbench_axis(gcmd)
        min_a = gcmd.get_float('MIN', 500.0, above=0.)
        max_a = gcmd.get_float('MAX', 50000.0, above=min_a)
        coarse = gcmd.get_float('COARSE_STEP', 2500.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 250.0, above=0.)
        speed = gcmd.get_float('SPEED', 200.0, above=0.)
        repeat = gcmd.get_int('REPEAT', 30, minval=1, maxval=200)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 50, minval=1, maxval=300)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        # Random-distance pattern for the accel test:
        #   min distance per move = V²/A (just barely reaches SPEED →
        #     immediate reversal; this is the reversal-stress test)
        #   max distance per move = MAX_DIST_FACTOR × V²/A, capped at axis
        # Distribution biased toward short distances via SHORT_BIAS.
        seed = gcmd.get_int('SEED', 12345)
        max_dist_factor = gcmd.get_float(
            'MAX_DIST_FACTOR', 4.0, minval=1.0, maxval=50.0)
        short_bias = gcmd.get_float(
            'SHORT_BIAS', 2.0, minval=1.0, maxval=10.0)

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        # At the lowest tested accel the triangle distance is largest
        # (= SPEED² / MIN_A). It must fit in the axis range.
        required = (speed ** 2) / min_a
        if required > ax_range:
            min_a_fit = math.ceil(
                ((speed ** 2) / ax_range) / 500.0) * 500.0
            raise gcmd.error(
                "Need %.0f mm to reach SPEED=%.0f at MIN=%.0f, but only "
                "%.0f mm available.\n"
                "Either raise MIN to ≥ %.0f mm/s² or lower SPEED."
                % (required, speed, min_a, ax_range, min_a_fit))

        meta = self._build_meta('ACCEL', axis,
                                {'MIN': min_a, 'MAX': max_a,
                                 'COARSE_STEP': coarse, 'MIN_STEP': min_step,
                                 'SPEED': speed, 'REPEAT': repeat})
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._banner(gcmd, 'ACCEL', axis, min_a, max_a, repeat, speed=speed)
        gcmd.respond_info(
            "Random moves per step — biased toward SHORT distances for "
            "direction-reversal stress:\n"
            "  min  = V²/A (just touches SPEED, immediate reversal)\n"
            "  max  = %.1f × V²/A (capped at axis %.0f mm)\n"
            "  bias = r^%.1f → ~%d%% of moves are in the lower half of "
            "the range\n"
            "  seed = %d"
            % (max_dist_factor, ax_range, short_bias,
               int(100 * (1 - 0.5 ** short_bias)), seed))

        self._set_limits(velocity=speed, accel=max_a * 1.5)
        self._ensure_homed([axis], testbench=testbench)
        results = []

        def measure_at(accel, phase):
            self._set_limits(velocity=speed, accel=accel)
            reps = verify_repeats if phase == 'verify' else repeat
            triangle = (speed ** 2) / accel
            # Min = triangle (just touches V, immediate reversal —
            # the reversal-stress test). Max = factor × triangle,
            # capped at axis range.
            min_dist = triangle
            max_dist = min(max_dist_factor * triangle, ax_range)
            if max_dist < min_dist:
                max_dist = min_dist
            # Expected-mean distance under power-law bias r^short_bias is
            # min + (max−min)/(short_bias+1). Use that for cruise reporting.
            mean_dist = min_dist + (max_dist - min_dist) / (short_bias + 1.0)
            mean_cruise = max(0.0, 1.0 - triangle / mean_dist)
            step_seed = seed + int(accel)
            r = self._measure_step(
                gcmd, [axis], 'A', accel,
                lambda: self._do_accel_pattern(
                    axis, speed, min_dist, max_dist, reps,
                    seed=step_seed, testbench=testbench,
                    short_bias=short_bias),
                testbench=testbench,
                cruise_fraction=mean_cruise)
            r['phase'] = phase
            r['speed'] = speed
            r['cruise_fraction'] = mean_cruise
            r['move_range'] = (min_dist, max_dist)
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

    def cmd_FIND_ENVELOPE(self, gcmd):
        """Map the velocity-acceleration envelope of an axis.

        Velocity and acceleration are physically coupled: a stepper's
        usable torque falls as speed rises (back-EMF eats into the
        current the driver can push through the windings), so the max
        safe acceleration is lower at high velocity and higher at low
        velocity. A single FIND_MAX_ACCEL at one fixed SPEED only samples
        one slice of that curve — pick the wrong SPEED and the resulting
        accel is either unsafe at higher speeds or needlessly low at
        lower ones.

        This test sweeps several velocities and finds the max safe accel
        at each, producing the whole envelope plus a balanced
        max_velocity / max_accel recommendation (the knee of the curve).

        Three-stage per velocity:
          • Stage 1 brackets the limit with a relative-accuracy binary
            search (no fixed step) using short, stall-safe 'jab' moves
            (V²/A long, anchored near home) — fast, and a stall barely
            grinds.
          • Stage 2 validates the found value with the thorough
            reversal-stress pattern across the axis.
          • Stage 3 runs a simulated print (BENCH_SHORT infill + BENCH_LONG
            travels, realistic lengths) at BENCH_DERATE × the found value.
            It stays in the centre of the axis and runs in chunks that
            re-home between them and abort on the first lost-step chunk, so
            a stall can't grind the whole sequence into the limit.

        A value is only accepted once it passes ALL THREE stages. If
        stage 2 or stage 3 fails it goes back to stage 1 with a lowered
        ceiling (up to MAX_REDO attempts) — it never backs off and accepts
        a failing value. If no value clears every stage, the velocity is
        excluded from the envelope rather than reported as safe.
        """
        self._check_ready()
        testbench, axis = self._parse_testbench_axis(gcmd)
        v_min = gcmd.get_float('V_MIN', 100.0, above=0.)
        v_max = gcmd.get_float('V_MAX', 500.0, above=v_min)
        v_points = gcmd.get_int('V_POINTS', 5, minval=2, maxval=12)
        a_min = gcmd.get_float('A_MIN', 1000.0, above=0.)
        a_max = gcmd.get_float('A_MAX', 50000.0, above=a_min)
        # Relative-accuracy binary search (no fixed step). Stop when the
        # guess is within ACCEL_ACCU of a bracket bound — resolution scales
        # with the value instead of an absolute MIN_STEP.
        accel_accu = gcmd.get_float('ACCEL_ACCU', 0.05, above=0., below=1.)
        repeat = gcmd.get_int('REPEAT', 15, minval=1, maxval=100)
        verify_repeats = gcmd.get_int('VERIFY_REPEATS', 30,
                                      minval=1, maxval=200)
        max_bisect = gcmd.get_int('MAX_ITERS', 12, minval=3, maxval=25)
        max_dist_factor = gcmd.get_float('MAX_DIST_FACTOR', 4.0, minval=1.)
        short_bias = gcmd.get_float('SHORT_BIAS', 2.0, minval=1., maxval=5.)
        seed = gcmd.get_int('SEED', 12345)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        # Stage 3: simulated-print benchmark per found value — bursts of
        # short infill zigzag + perimeters + travels at realistic lengths.
        # If it fails, the value is re-determined at a lower ceiling, up to
        # MAX_REDO times. BENCH_SHORT = infill segments, BENCH_LONG =
        # travel moves.
        bench_short = gcmd.get_int('BENCH_SHORT', 400, minval=0, maxval=5000)
        bench_long = gcmd.get_int('BENCH_LONG', 60, minval=0, maxval=2000)
        # Benchmark in chunks: re-home + skip-check every BENCH_CHUNK moves
        # and abort on the first failing chunk, so a stall grinds at most
        # one chunk instead of the whole run.
        bench_chunk = gcmd.get_int('BENCH_CHUNK', 80, minval=10, maxval=1000)
        # Validate (and accept) at BENCH_DERATE × the found value, not the
        # bleeding edge — fewer failures, far less likely to crash.
        bench_derate = gcmd.get_float('BENCH_DERATE', 0.9,
                                      minval=0.5, maxval=1.0)
        max_redo = gcmd.get_int('MAX_REDO', 4, minval=0, maxval=10)

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)

        # Velocity sweep points (ascending).
        step = (v_max - v_min) / (v_points - 1)
        v_list = [v_min + step * i for i in range(v_points)]

        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        meta = self._build_meta('ENVELOPE', axis,
                                {'V_MIN': v_min, 'V_MAX': v_max,
                                 'V_POINTS': v_points, 'A_MIN': a_min,
                                 'A_MAX': a_max, 'ACCEL_ACCU': accel_accu,
                                 'REPEAT': repeat,
                                 'BENCH': '%d+%d' % (bench_short, bench_long),
                                 'BENCH_DERATE': bench_derate,
                                 'BENCH_CHUNK': bench_chunk,
                                 'MAX_REDO': max_redo})
        # TMC driver + run_current for the tested axis, so the report can
        # show what current the result was found at (accel scales with it).
        tmc_info = self._lookup_tmc_for_axis(axis)
        if tmc_info is not None:
            cur = self._read_run_current(axis)
            meta['tmc_driver'] = tmc_info[0]
            meta['tmc_run_current'] = (
                '%.3f A' % cur if cur is not None else 'unknown')
        gcmd.respond_info(
            "===== Speed Test v%s — V/A ENVELOPE on %s =====\n"
            "Plugin by Steven (Fragmon) — Crydteam\n"
            "Sweeping %d velocities %.0f → %.0f mm/s; finding max safe "
            "accel at each.\n"
            "Why combined: motor torque drops as speed rises, so max accel "
            "depends on velocity.\n"
            "Stage 1: fast jab bracket. Stage 2: reversal-stress validate. "
            "Stage 3: simulated print (%d+%d moves, %.0f%% derate, chunked, "
            "centre-of-axis) — redo on fail.\n"
            "Usable %s travel: %.0f mm | %d moves/step\n"
            "================================================"
            % (MODULE_VERSION, axis, v_points, v_min, v_max,
               bench_short, bench_long, bench_derate * 100,
               axis, ax_range, repeat))

        self._set_limits(velocity=v_max * 1.2, accel=a_max)
        self._ensure_homed([axis], testbench=testbench)
        results = []
        envelope = []
        prev_amax = None

        try:
            for idx, v in enumerate(v_list):
                # Lowest accel that still lets a move actually reach v
                # within the axis: the triangle distance V²/A must fit,
                # so A ≥ V²/range. Below that the move never hits v.
                a_floor = (v * v) / ax_range
                low = max(a_min, a_floor * 1.05)
                # Warm-start the upper bound from the previous (lower-V)
                # result — the envelope only falls as V rises, so there's
                # no need to re-climb all the way to A_MAX every time.
                high = a_max if prev_amax is None \
                    else min(a_max, prev_amax * 1.3)
                if high <= low * (1.0 + accel_accu):
                    gcmd.respond_info(
                        "  ▶ V=%.0f mm/s needs accel ≥ %.0f mm/s² just to "
                        "reach it within %.0f mm of travel, but the ceiling "
                        "here is only %.0f (A_MAX, or the motor's accel at "
                        "lower speeds). This velocity isn't reachable on this "
                        "axis — skipped."
                        % (v, low, ax_range, high))
                    continue

                gcmd.respond_info(
                    "\n──────── V=%.0f mm/s (%d/%d) — max accel in "
                    "[%.0f, %.0f] mm/s² ────────"
                    % (v, idx + 1, v_points, low, high))

                def measure_at(accel, phase, _v=v):
                    self._set_limits(velocity=_v, accel=accel)
                    if phase == 'verify':
                        # Stage 2: validate the value the fast search found
                        # with the thorough reversal-stress pattern (random
                        # distances across the axis — the real accel killer).
                        triangle = (_v * _v) / accel
                        min_dist = triangle
                        max_dist = min(max_dist_factor * triangle, ax_range)
                        if max_dist < min_dist:
                            max_dist = min_dist
                        mean_dist = (min_dist + (max_dist - min_dist)
                                     / (short_bias + 1.0))
                        mean_cruise = max(0.0, 1.0 - triangle / mean_dist)
                        r = self._measure_step(
                            gcmd, [axis], 'A', accel,
                            lambda: self._do_accel_pattern(
                                axis, _v, min_dist, max_dist, verify_repeats,
                                seed=seed + int(accel), testbench=testbench,
                                short_bias=short_bias),
                            testbench=testbench, cruise_fraction=mean_cruise)
                    else:
                        # Stage 1: fast, stall-safe jab moves bracket the
                        # limit quickly with minimal grinding on a stall.
                        r = self._measure_step(
                            gcmd, [axis], 'A', accel,
                            lambda: self._do_jab_pattern(
                                axis, _v, accel, repeat, testbench=testbench),
                            testbench=testbench)
                    r['phase'] = phase
                    r['velocity'] = _v
                    r['accel'] = accel
                    return r

                # Rigorous staged search. A value is only accepted once it
                # passes ALL THREE stages. ANY stage failing sends it back
                # to stage 1 with a lowered ceiling — it never just backs
                # off and accepts a failing value. `best` stays None until
                # a value clears every stage; if none does, the point is
                # honestly excluded from the envelope.
                hi = high
                attempt = 0
                backoff = max(accel_accu, 0.1)
                best = None
                while attempt <= max_redo:
                    # Stage 1: search for the highest value that passes the
                    # quick jab test (None if even the floor fails).
                    cand = self._binary_search_max(
                        gcmd, results, low, hi, accel_accu, max_bisect,
                        'A', measure_at)
                    if cand is None:
                        gcmd.respond_info(
                            "  ✗ V=%.0f mm/s: no accel in [%.0f, %.0f] passes "
                            "even the quick search." % (v, low, hi))
                        break

                    # Stage 2: thorough reversal-stress validation must pass.
                    vres = measure_at(cand, 'verify')
                    vres['phase'] = 'verify'
                    results.append(vres)
                    if vres.get('failed'):
                        attempt += 1
                        hi = cand * (1.0 - backoff)
                        gcmd.respond_info(
                            "  ↻ Stage 2 FAILED at A=%.0f → back to stage 1 "
                            "below %.0f (attempt %d/%d)"
                            % (cand, hi, attempt, max_redo))
                        if hi <= low:
                            break
                        continue

                    # Stage 3: simulated-print benchmark must pass. Tested at
                    # a slightly derated value (safer, fewer crashes), run in
                    # chunks that re-home between them and abort on the first
                    # lost-step chunk — so a stall grinds at most one chunk.
                    bench_val = cand * bench_derate
                    targets = self._build_print_sim_moves(
                        axis, bench_short, bench_long, seed)
                    gcmd.respond_info(
                        "  ──── Stage 3: print simulation, %d moves at "
                        "V=%.0f A=%.0f (%.0f%% of %.0f), chunks of %d ────"
                        % (len(targets), v, bench_val, bench_derate * 100,
                           cand, bench_chunk))
                    self._set_limits(velocity=v, accel=bench_val)
                    bench_failed = False
                    for ci in range(0, len(targets), bench_chunk):
                        sub = targets[ci:ci + bench_chunk]
                        bres = self._measure_step(
                            gcmd, [axis], 'A', bench_val,
                            lambda _s=sub: self._run_axis_moves(
                                axis, _s, v, testbench=testbench),
                            testbench=testbench)
                        bres['phase'] = 'benchmark'
                        bres['velocity'] = v
                        results.append(bres)
                        if bres.get('failed'):
                            bench_failed = True
                            gcmd.respond_info(
                                "  ⚠ lost steps at move %d/%d — run aborted "
                                "(no further grinding)"
                                % (ci + len(sub), len(targets)))
                            break
                    if bench_failed:
                        attempt += 1
                        hi = cand * (1.0 - backoff)
                        gcmd.respond_info(
                            "  ↻ Stage 3 benchmark FAILED → back to stage 1 "
                            "below %.0f (attempt %d/%d)"
                            % (hi, attempt, max_redo))
                        if hi <= low:
                            break
                        continue

                    # Passed all three stages — accept the derated value.
                    best = bench_val
                    break

                if best is None:
                    gcmd.respond_info(
                        "  ✗ V=%.0f mm/s excluded — no value passed all three "
                        "stages (down to the %.0f floor, %d attempts)."
                        % (v, low, attempt))
                    continue

                envelope.append((v, best))
                prev_amax = best
                gcmd.respond_info(
                    "  ✓ V=%.0f mm/s  →  max safe accel %.0f mm/s² "
                    "(passed search + validation + benchmark)"
                    % (v, best))
        finally:
            self._restore_limits()

        if len(envelope) < 2:
            raise gcmd.error(
                "Envelope needs at least 2 testable velocity points. "
                "Lower V_MAX, raise A_MAX, or test on a longer axis.")

        self._envelope_summary(gcmd, axis, envelope)
        self._save_envelope_report(envelope, results, meta, timestamp,
                                   no_html, gcmd)

    def _find_knee(self, envelope):
        """Index of the curve's 'knee' — the velocity past which buying
        more speed costs the most acceleration. Kneedle-style: the point
        farthest from the chord between the first and last samples,
        measured in normalized V/A space."""
        n = len(envelope)
        if n <= 2:
            return n - 1
        vs = [p[0] for p in envelope]
        as_ = [p[1] for p in envelope]
        v0 = vs[0]
        dv = (vs[-1] - vs[0]) or 1.0
        amin, amax = min(as_), max(as_)
        da = (amax - amin) or 1.0
        na0 = (as_[0] - amin) / da
        na1 = (as_[-1] - amin) / da
        slope = na1 - na0
        denom = math.sqrt(1.0 + slope * slope)
        best_i, best_d = 0, -1.0
        for i in range(n):
            nv = (vs[i] - v0) / dv
            na = (as_[i] - amin) / da
            d = abs(nv * slope - (na - na0)) / denom
            if d > best_d:
                best_d, best_i = d, i
        return best_i

    def _envelope_summary(self, gcmd, axis, envelope):
        envelope = sorted(envelope, key=lambda p: p[0])
        table = "\n".join(
            "  V=%-4.0f mm/s   →   max accel %6.0f mm/s²" % (v, a)
            for v, a in envelope)
        knee_i = self._find_knee(envelope)
        vk, ak = envelope[knee_i]
        v_lo, a_lo = envelope[0]      # slowest velocity → highest accel
        v_hi, a_hi = envelope[-1]     # fastest velocity → lowest accel
        m = 0.9                       # 10 % safety margin on both axes
        gcmd.respond_info(
            "\n========== V/A ENVELOPE RESULT (%s) ==========\n"
            "%s\n"
            "----------------------------------\n"
            "The motor trades speed for acceleration. Pick the operating\n"
            "point that matches your prints, then set BOTH in printer.cfg.\n"
            "----------------------------------\n"
            "Balanced (knee) — recommended:\n"
            "  [printer]\n"
            "  max_velocity: %.0f\n"
            "  max_accel:    %.0f\n"
            "----------------------------------\n"
            "Speed-priority (long travels): max_velocity %.0f + "
            "max_accel %.0f\n"
            "Accel-priority (small parts):  max_velocity %.0f + "
            "max_accel %.0f\n"
            "(all recommendations include a 10%% safety margin)\n"
            "================================="
            % (axis, table,
               vk * m, ak * m,
               v_hi * m, a_hi * m,
               v_lo * m, a_lo * m))

    def _save_envelope_report(self, envelope, results, meta, timestamp,
                              no_html, gcmd):
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            csv_path = os.path.join(
                self.output_dir, 'speed_envelope_%s.csv' % timestamp)
            self._write_envelope_csv(csv_path, envelope, results, meta)
            if gcmd is not None:
                gcmd.respond_info("CSV saved: %s" % csv_path)
            if not no_html:
                html_path = os.path.join(
                    self.output_dir, 'speed_envelope_%s.html' % timestamp)
                self._write_envelope_html(html_path, envelope, meta)
                if gcmd is not None:
                    gcmd.respond_info("HTML saved: %s" % html_path)
        except Exception as e:
            if gcmd is not None:
                gcmd.respond_info("Warning: report write failed: %s" % e)
            logging.exception("speed_test: envelope report write failed")

    def _write_envelope_csv(self, path, envelope, results, meta):
        with open(path, 'w') as f:
            self._write_csv_preamble(
                f, "Speed Test v%s — V/A ENVELOPE" % MODULE_VERSION, meta)
            f.write("# --- envelope: max safe accel per velocity ---\n")
            f.write("velocity_mm_s,max_accel_mm_s2\n")
            for v, a in sorted(envelope, key=lambda p: p[0]):
                f.write("%.1f,%.1f\n" % (v, a))
            f.write("# --- all measurements ---\n")
            f.write("velocity,accel,phase,failed,lost_steps,skip_axes\n")
            for r in results:
                f.write("%.1f,%.1f,%s,%d,%d,%s\n" % (
                    r.get('velocity', 0.0),
                    r['value'],
                    r.get('phase', 'coarse'),
                    1 if r['failed'] else 0,
                    r['max_diff'], r['skip_axes'] or '-'))

    def _write_envelope_html(self, path, envelope, meta):
        envelope = sorted(envelope, key=lambda p: p[0])
        vs = [v for v, _ in envelope]
        as_ = [a for _, a in envelope]
        knee_i = self._find_knee(envelope)
        vk, ak = envelope[knee_i]
        knee_pts = [None] * len(envelope)
        knee_pts[knee_i] = ak
        meta_html = self._meta_to_html(meta)
        rows = "".join(
            "<tr%s><td>%.0f</td><td>%.0f</td></tr>"
            % (' style="background:#fff3cd"' if i == knee_i else '', v, a)
            for i, (v, a) in enumerate(envelope))
        # Current printer.cfg + TMC values, for side-by-side comparison.
        cfg_v = meta.get('printer_max_velocity', '—')
        cfg_a = meta.get('printer_max_accel', '—')
        cfg_scv = meta.get('printer_scv', '—')
        tmc_drv = meta.get('tmc_driver', 'none')
        tmc_cur = meta.get('tmc_run_current', '—')
        tpl = """<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<title>Speed Test (V/A Envelope) — %(ts)s</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0"></script>
<style>
body { font-family: system-ui, sans-serif; max-width: 1000px;
       margin: 20px auto; padding: 0 20px; color:#333; }
h1 { color:#1565c0; }
.meta { background:#f5f5f5; padding:15px; border-radius:8px;
        margin-bottom:20px; display:grid;
        grid-template-columns: repeat(auto-fit, minmax(220px,1fr));
        gap:8px; font-size:14px; }
.summary { background:#e3f2fd; padding:15px; border-radius:8px;
           margin-bottom:20px; border-left:4px solid #1976d2; }
.summary h2 { margin:0 0 10px 0; color:#1976d2; }
.chart-container { background:white; padding:20px; border-radius:8px;
                   margin-bottom:20px; box-shadow:0 2px 4px rgba(0,0,0,.1); }
table { width:100%%; border-collapse:collapse; font-size:14px; }
th,td { padding:8px 12px; border:1px solid #ddd; text-align:right; }
th { background:#f5f5f5; }
.footer { text-align:center; color:#666; padding:20px; font-size:13px; }
.footer a { color:#1976d2; }
.compare { background:white; padding:20px; border-radius:8px;
           margin-bottom:20px; box-shadow:0 2px 4px rgba(0,0,0,.1); }
.compare td:first-child, .compare th:first-child { text-align:left; }
.compare .tmc { margin-top:12px; color:#555; font-size:14px; }
.userfield { background:#fff8e1; border-left:4px solid #f9a825;
             padding:14px 18px; border-radius:8px; margin-bottom:20px; }
.userfield label { font-weight:bold; margin-right:8px; }
.userfield input { font-size:15px; padding:6px 10px; border:1px solid #ccc;
                   border-radius:6px; width:160px; }
.userfield .hint { display:block; margin-top:6px; color:#777;
                   font-size:13px; }
</style></head><body>
<h1>Speed Test — Velocity / Acceleration Envelope</h1>
<p style="color:#666;margin-top:-10px;">
  Plugin by Steven (Fragmon) — Crydteam ·
  <a href="https://www.youtube.com/@crydteamprinting" target="_blank">YouTube: @crydteamprinting</a></p>
<div class="meta">%(meta_html)s</div>
<div class="summary">
  <h2>Recommended balanced point (knee)</h2>
  <p>max_velocity <strong>%(vk).0f</strong> mm/s &nbsp;+&nbsp;
     max_accel <strong>%(ak).0f</strong> mm/s²
     <em>(raw knee value — apply your own safety margin)</em></p>
  <p>Everything on or below the curve is safe: pick any (velocity, accel)
     pair under the line and set <em>both</em> in printer.cfg.</p>
</div>
<div class="userfield">
  <label for="thw">Toolhead weight:</label>
  <input id="thw" type="text" placeholder="e.g. 450 g" />
  <span class="hint">Note your moving mass — a heavier toolhead lowers the
     safe acceleration. Saved with this report in your browser.</span>
</div>
<div class="compare">
  <h2>Your config vs. this run</h2>
  <table><thead><tr><th>Parameter</th><th>Current printer.cfg</th>
    <th>Found here (knee, raw)</th></tr></thead><tbody>
    <tr><td>max_velocity</td><td>%(cfg_v)s</td><td>%(vk).0f mm/s</td></tr>
    <tr><td>max_accel</td><td>%(cfg_a)s</td><td>%(ak).0f mm/s&sup2;</td></tr>
    <tr><td>square_corner_velocity</td><td>%(cfg_scv)s</td>
        <td>not tested</td></tr>
  </tbody></table>
  <p class="tmc">Motor driver: <strong>%(tmc_drv)s</strong> &middot;
     run_current: <strong>%(tmc_cur)s</strong> — acceleration scales with
     current, so note it when comparing runs.</p>
</div>
<div class="chart-container">
  <h2>Max safe acceleration vs. velocity</h2>
  <canvas id="envChart"></canvas>
</div>
<div class="chart-container">
  <h2>Data table</h2>
  <table><thead><tr><th>Velocity (mm/s)</th>
    <th>Max safe accel (mm/s²)</th></tr></thead>
    <tbody>%(rows)s</tbody></table>
</div>
<div class="footer"><p>Generated by <strong>%(plugin)s</strong> at %(ts)s</p></div>
<script>
const vs = %(vs)s, accels = %(as)s, knee = %(knee)s;
new Chart(document.getElementById('envChart'), {
  type:'line',
  data:{ labels: vs, datasets:[
    { label:'Max safe accel', data: accels, borderColor:'#1976d2',
      backgroundColor:'rgba(25,118,210,.12)', fill:true, tension:.2,
      pointRadius:4 },
    { label:'Knee (balanced)', data: knee, borderColor:'#fb8c00',
      backgroundColor:'#fb8c00', pointRadius:8, showLine:false },
  ]},
  options:{ responsive:true,
    scales:{ x:{ title:{display:true, text:'Velocity (mm/s)'} },
             y:{ title:{display:true, text:'Max safe acceleration (mm/s²)'},
                 beginAtZero:true } },
    plugins:{ legend:{position:'top'} } },
});
// Persist the toolhead-weight free-text field in the browser, keyed to
// this report so each run keeps its own value across reloads/printing.
(function () {
  var key = 'crydteam_thw_' + %(ts_key)s;
  var el = document.getElementById('thw');
  if (!el) return;
  try { el.value = localStorage.getItem(key) || ''; } catch (e) {}
  el.addEventListener('input', function () {
    try { localStorage.setItem(key, el.value); } catch (e) {}
  });
})();
</script>
</body></html>"""
        html = tpl % {
            'ts': meta.get('timestamp', '-'),
            'ts_key': json.dumps(meta.get('timestamp', '-')),
            'plugin': '%s v%s' % (MODULE_NAME, MODULE_VERSION),
            'meta_html': meta_html,
            'vk': vk, 'ak': ak,
            'cfg_v': cfg_v, 'cfg_a': cfg_a, 'cfg_scv': cfg_scv,
            'tmc_drv': tmc_drv, 'tmc_cur': tmc_cur,
            'rows': rows,
            'vs': json.dumps([round(v, 1) for v in vs]),
            'as': json.dumps([round(a, 1) for a in as_]),
            'knee': json.dumps([round(a, 1) if a is not None else None
                                for a in knee_pts]),
        }
        with open(path, 'w') as f:
            f.write(html)

    def cmd_FIND_MAX_SCV(self, gcmd):
        self._check_ready()
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
        self._banner(gcmd, 'SCV', 'XY', min_s, max_s, repeat,
                     speed=speed, accel=accel)

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
        self._check_ready()
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

    # ─── TMC current control ──────────────────────────────────────────

    def _read_run_current(self, axis):
        """Read the currently configured run_current from the TMC driver
        on `axis`. Returns None on failure or no driver."""
        info = self._lookup_tmc_for_axis(axis)
        if info is None:
            return None
        _drv, tmc = info
        try:
            status = tmc.get_status(self.reactor.monotonic())
            return status.get('run_current')
        except Exception:
            return None

    def _set_run_current(self, axis, value):
        """Set a stepper's run_current via SET_TMC_CURRENT."""
        stepper_name = 'stepper_' + axis.lower()
        self.gcode.run_script_from_command(
            "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f"
            % (stepper_name, value))

    def _final_current_summary(self, gcmd, axis, optimal, min_passing,
                                driver_name, verify_failed=False):
        quality = ("verified OK" if not verify_failed
                   else "VERIFY FAILED — bumped to fallback")
        gcmd.respond_info(
            "\n========== OPTIMAL CURRENT RESULT ==========\n"
            "Axis: %s | Driver: %s\n"
            "Minimum passing current:    %.3f A\n"
            "Recommended (with margin):  %.3f A\n"
            "Quality: %s\n"
            "----------------------------------\n"
            "To set permanently in printer.cfg:\n"
            "  [%s stepper_%s]\n"
            "  run_current:  %.3f\n"
            "  hold_current: %.3f   # ~65%% of run_current\n"
            "----------------------------------\n"
            "Session-only:\n"
            "  SET_TMC_CURRENT STEPPER=stepper_%s CURRENT=%.3f\n"
            "=================================="
            % (axis, driver_name, min_passing, optimal, quality,
               driver_name, axis.lower(),
               optimal, optimal * 0.65,
               axis.lower(), optimal))

    def cmd_FIND_OPTIMAL_CURRENT(self, gcmd):
        """Find the minimum run_current that still passes a SPEED/ACCEL
        target. Starts at MAX_CURRENT and searches downward via
        adaptive bisection on the current value.
        """
        self._check_ready()
        testbench, axis = self._parse_testbench_axis(gcmd, tb_note=".")

        # TMC driver is required to change current at runtime
        info = self._lookup_tmc_for_axis(axis)
        if info is None:
            raise gcmd.error(
                "No TMC driver found on stepper_%s. Current optimization "
                "requires a TMC stepper driver." % axis.lower())
        driver_name, _tmc = info

        # Read original current — restored at the end
        original_current = self._read_run_current(axis)
        if original_current is None:
            raise gcmd.error(
                "Could not read run_current from %s on stepper_%s. "
                "Cannot optimize without a known baseline."
                % (driver_name, axis.lower()))

        # Performance target
        speed = gcmd.get_float('SPEED', 200.0, above=0.)
        accel = gcmd.get_float('ACCEL', 5000.0, above=0.)

        # MAX_CURRENT precedence: command param → config max_current →
        # currently configured run_current (1.5× as upper search bound).
        max_current_param = gcmd.get_float('MAX_CURRENT', 0., minval=0.)
        if max_current_param > 0:
            max_current = max_current_param
        elif self.max_current > 0:
            max_current = self.max_current
        else:
            max_current = max(original_current * 1.2, original_current + 0.1)

        # Always apply the config safety cap on top
        if self.max_current > 0 and max_current > self.max_current:
            gcmd.respond_info(
                "MAX_CURRENT clipped to config cap: %.3f A → %.3f A"
                % (max_current, self.max_current))
            max_current = self.max_current

        min_current = gcmd.get_float('MIN_CURRENT', 0.3,
                                      above=0., below=max_current)
        coarse = gcmd.get_float('COARSE_STEP', 0.1, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 0.05, above=0.)
        safety_margin = gcmd.get_float(
            'SAFETY_MARGIN', 0.10, minval=0., maxval=1.)
        repeat = gcmd.get_int('REPEAT', 10, minval=1, maxval=100)
        verify_repeats = gcmd.get_int(
            'VERIFY_REPEATS', 30, minval=1, maxval=300)
        max_bisect = gcmd.get_int('MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        max_dist_factor = gcmd.get_float(
            'MAX_DIST_FACTOR', 4.0, minval=1.0, maxval=50.0)
        short_bias = gcmd.get_float(
            'SHORT_BIAS', 2.0, minval=1.0, maxval=10.0)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        seed = gcmd.get_int('SEED', 12345)

        if min_step >= coarse:
            raise gcmd.error("MIN_STEP must be smaller than COARSE_STEP")

        ax_min, ax_max, ax_mid, ax_range = self._get_axis_bounds(axis)
        triangle = (speed ** 2) / accel
        if triangle > ax_range:
            raise gcmd.error(
                "Need %.0f mm to reach SPEED=%.0f at ACCEL=%.0f, only "
                "%.0f mm available on %s." % (triangle, speed, accel,
                                              ax_range, axis))

        meta = self._build_meta('CURRENT', axis, {
            'driver': driver_name,
            'original_current_A': '%.3f' % original_current,
            'MIN_CURRENT': min_current, 'MAX_CURRENT': max_current,
            'COARSE_STEP': coarse, 'MIN_STEP': min_step,
            'SPEED': speed, 'ACCEL': accel, 'REPEAT': repeat,
            'SAFETY_MARGIN': safety_margin,
        })
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')

        gcmd.respond_info(
            "===== Speed Test v%s — OPTIMAL CURRENT on %s =====\n"
            "Plugin by Steven (Fragmon) — Crydteam\n"
            "Driver: %s | Currently configured: %.3f A\n"
            "Performance target: SPEED=%.0f mm/s | ACCEL=%.0f mm/s²\n"
            "Current search range: %.3f → %.3f A | %d moves/step "
            "(verify %d) | Safety margin: +%.0f%%\n"
            "Method: start at MAX, search DOWN for the minimum that passes.\n"
            "================================================"
            % (MODULE_VERSION, axis, driver_name, original_current,
               speed, accel, min_current, max_current,
               repeat, verify_repeats,
               safety_margin * 100))

        self._set_limits(velocity=speed * 1.2, accel=accel)
        self._ensure_homed([axis], testbench=testbench)

        results = []

        min_dist = triangle
        max_dist = min(max_dist_factor * triangle, ax_range)
        if max_dist < min_dist:
            max_dist = min_dist

        def measure_at_current(current_value, phase):
            self._set_run_current(axis, current_value)
            # Brief settle after current change
            self.gcode.run_script_from_command("G4 P200")
            # Read back the value the driver actually delivers (TMC
            # drivers have discrete current steps and a hardware ceiling
            # — asking for a value the driver can't hit silently rounds
            # or caps). All bracket math uses this actual value.
            actual = self._read_run_current(axis)
            if actual is None:
                actual = current_value
            if abs(actual - current_value) > 0.02:
                gcmd.respond_info(
                    "  • Requested %.3f A, driver delivers %.3f A "
                    "(discrete step / hardware cap)"
                    % (current_value, actual))
            reps = verify_repeats if phase == 'verify' else repeat
            # Same seed across all current steps → identical move
            # sequence, so any difference in outcome is caused by current.
            r = self._measure_step(
                gcmd, [axis], 'I', actual,
                lambda: self._do_accel_pattern(
                    axis, speed, min_dist, max_dist, reps,
                    seed=seed, testbench=testbench,
                    short_bias=short_bias),
                testbench=testbench)
            r['phase'] = phase
            r['speed'] = speed
            r['accel'] = accel
            r['requested_current'] = current_value
            r['actual_current'] = actual
            return r

        verify_failed = False
        optimal = max_current
        last_pass = max_current

        try:
            # ─── Phase 0: Sanity at MAX_CURRENT ───
            gcmd.respond_info(
                "\n>>> Phase 0: Sanity check at MAX = %.3f A <<<"
                % max_current)
            r0 = measure_at_current(max_current, 'sanity')
            results.append(r0)
            if r0['failed']:
                raise gcmd.error(
                    "Motor failed at MAX_CURRENT=%.3f A (actual %.3f A). "
                    "Even maximum current can't pass SPEED=%.0f/"
                    "ACCEL=%.0f. Reduce performance target or raise "
                    "MAX_CURRENT (within driver/motor limits)."
                    % (max_current, r0['actual_current'], speed, accel))
            # Use the actual achieved current as the effective MAX from
            # here on — that's what the driver can really deliver.
            effective_max = r0['actual_current']
            last_pass = effective_max
            gcmd.respond_info(
                "  ✓ Passes at MAX (actual %.3f A). Searching downward."
                % effective_max)

            # ─── Phase 1: Coarse downward sweep ───
            gcmd.respond_info(
                "\n>>> Phase 1: Coarse downward sweep "
                "(step -%.3f A) <<<" % coarse)
            first_fail = None
            cur = effective_max - coarse
            while cur >= min_current - 0.0001:
                r = measure_at_current(cur, 'coarse')
                results.append(r)
                act = r['actual_current']
                # Don't re-test currents the driver collapses to the
                # same actual value as the last one
                if act >= last_pass - 0.005:
                    gcmd.respond_info(
                        "  • Skipping: driver delivered %.3f A again "
                        "(same as last). Decrementing request more."
                        % act)
                    cur -= coarse
                    continue
                if r['failed']:
                    first_fail = act
                    gcmd.respond_info(
                        "  >>> FAIL at %.3f A → bracket (%.3f, %.3f]"
                        % (act, act, last_pass))
                    break
                last_pass = act
                cur -= coarse

            if first_fail is None:
                gcmd.respond_info(
                    "Motor passes even down to %.3f A (request floor "
                    "%.3f A). Lower MIN_CURRENT to search further."
                    % (last_pass, min_current))
                optimal = max(min_current,
                              last_pass * (1 + safety_margin))
                optimal = round(optimal / min_step) * min_step
                self._set_run_current(axis, optimal)
                actual_opt = self._read_run_current(axis) or optimal
                self._final_current_summary(
                    gcmd, axis, actual_opt, last_pass, driver_name)
                self._save_report(results, meta, timestamp,
                                  "passed down to MIN_CURRENT",
                                  no_html, 'current', gcmd)
                return

            # ─── Phase 2: Bisection ───
            # high = lowest known passing actual current
            # low  = highest known failing actual current
            high = last_pass
            low = first_fail
            gcmd.respond_info(
                "\n>>> Phase 2: Bisection in (%.3f, %.3f] A <<<"
                % (low, high))
            iter_count = 0
            same_count = 0
            while (high - low) > min_step + 0.0001 and iter_count < max_bisect:
                iter_count += 1
                raw_mid = (low + high) / 2.0
                mid = round(raw_mid / min_step) * min_step
                if mid <= low + 0.0001 or mid >= high - 0.0001:
                    break
                r = measure_at_current(mid, 'bisect')
                results.append(r)
                act = r['actual_current']
                if act >= high - 0.005 or act <= low + 0.005:
                    same_count += 1
                    gcmd.respond_info(
                        "  • Driver delivered %.3f A — bracket can't "
                        "narrow further at this resolution." % act)
                    if same_count >= 2:
                        break
                if r['failed']:
                    low = act
                    gcmd.respond_info(
                        "  >>> %.3f A FAIL → (%.3f, %.3f] (%d/%d)"
                        % (act, low, high, iter_count, max_bisect))
                else:
                    high = act
                    gcmd.respond_info(
                        "  >>> %.3f A OK → (%.3f, %.3f] (%d/%d)"
                        % (act, low, high, iter_count, max_bisect))

            # ─── Phase 3: Verify at high + safety margin ───
            optimal = high * (1 + safety_margin)
            if optimal > effective_max:
                optimal = effective_max
            if optimal < high:
                optimal = high

            gcmd.respond_info(
                "\n>>> Phase 3: Verify at OPTIMAL ≈ %.3f A "
                "(min passing %.3f A + %.0f%% margin) <<<"
                % (optimal, high, safety_margin * 100))
            rv = measure_at_current(optimal, 'verify')
            results.append(rv)
            actual_optimal = rv['actual_current']

            if rv['failed']:
                verify_failed = True
                fallback = min(effective_max, last_pass + coarse)
                self._set_run_current(axis, fallback)
                actual_optimal = self._read_run_current(axis) or fallback
                gcmd.respond_info(
                    "  ⚠ Verify FAILED at %.3f A — bumped to %.3f A "
                    "as conservative fallback."
                    % (rv['actual_current'], actual_optimal))

            self._final_current_summary(
                gcmd, axis, actual_optimal, high, driver_name,
                verify_failed=verify_failed)
        finally:
            self._set_run_current(axis, original_current)
            gcmd.respond_info(
                "Restored original current: %.3f A" % original_current)

        self._save_report(results, meta, timestamp,
                          "verify failed" if verify_failed else None,
                          no_html, 'current', gcmd)

    def cmd_STATUS(self, gcmd):
        max_curr_str = ("%.3f A" % self.max_current
                        if self.max_current > 0 else "unset (no cap)")
        gcmd.respond_info(
            "===== Speed Test v%s — STATUS =====\n"
            "Structure: %s\n"
            "Default axis: %s\n"
            "Testbench mode (default): %s\n"
            "Z position for tests: %.1f mm\n"
            "Margin from axis ends: %.1f mm\n"
            "TMC SG monitoring: %s\n"
            "Max current cap: %s\n"
            "Skip tolerance (max_missed): %.2f full steps\n"
            "Output dir: %s"
            % (MODULE_VERSION, self.structure, self.default_axis,
               "on (only X used, no Y/Z homing)"
               if self.testbench_default else "off",
               self.z_pos, self.margin,
               "on" if self.monitor_tmc else "off",
               max_curr_str, self.max_missed,
               self.output_dir))
        # Skip detection reads the stepper MCU position directly; the
        # endstop_phase module is not used and must NOT be loaded (it
        # aborts homing on the first lost step).
        ep = self.printer.lookup_object('endstop_phase', None)
        mcu_ok = self._read_mcu_pos(self.default_axis) is not None
        gcmd.respond_info(
            "Skip detection: direct stepper MCU position (%s)\n"
            "endstop_phase module: %s"
            % ("readable ✓" if mcu_ok else "NOT readable ✗",
               "PRESENT ✗ — remove it, it will abort the test"
               if ep is not None else "absent ✓ (correct)"))
        for axis in ('X', 'Y'):
            try:
                lo, hi, mid, rng = self._get_axis_bounds(axis)
                info = self._lookup_tmc_for_axis(axis)
                if info:
                    cur = self._read_run_current(axis)
                    tmc_str = "TMC: %s, run_current=%s" % (
                        info[0],
                        "%.3f A" % cur if cur is not None else "unknown")
                else:
                    tmc_str = "TMC: none"
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

    def _banner(self, gcmd, kind, axis, lo, hi, repeat,
                speed=None, accel=None):
        unit = {'VELOCITY': 'mm/s', 'ACCEL': 'mm/s²',
                'SCV': 'mm/s'}.get(kind, '')
        ctx = []
        if speed is not None:
            ctx.append("speed %.0f mm/s" % speed)
        if accel is not None:
            ctx.append("accel %.0f mm/s²" % accel)
        ctx_str = ("\nFixed: " + ", ".join(ctx)) if ctx else ""
        gcmd.respond_info(
            "===== Speed Test v%s — find max %s on %s =====\n"
            "Plugin by Steven (Fragmon) — Crydteam\n"
            "Search range: %.0f → %.0f %s | %d moves/step%s\n"
            "Method: adaptive bisection (coarse → bisect → verify)\n"
            "================================================"
            % (MODULE_VERSION, kind, axis, lo, hi, unit, repeat, ctx_str))

    def _final_summary(self, gcmd, kind, axis, safe, results):
        verify = [r for r in results if r.get('phase') == 'verify']
        verify_ok = verify and not verify[-1].get('failed')
        if verify_ok:
            quality = "VERIFIED OK"
        elif verify:
            quality = "VERIFY FAILED — value may be unstable"
        else:
            quality = "no verification phase"
        # Map each test to the printer.cfg key + unit it actually feeds,
        # so the result is a value the user can paste straight in.
        cfg = {
            'VELOCITY': ('max_velocity', 'mm/s'),
            'ACCEL':    ('max_accel', 'mm/s²'),
            'SCV':      ('square_corner_velocity', 'mm/s'),
        }
        cfg_key, unit = cfg.get(kind, (kind.lower(), ''))
        gcmd.respond_info(
            "\n========== %s RESULT (%s) ==========\n"
            "Max safe value: %.1f %s   (%s)\n"
            "----------------------------------\n"
            "Recommended for printer.cfg [printer]:\n"
            "  %s: %.1f %s   # safe limit −10%% margin\n"
            "  conservative −20%%: %.1f %s\n"
            "================================="
            % (kind, axis, safe, unit, quality,
               cfg_key, safe * 0.9, unit,
               safe * 0.8, unit))

    def _write_csv_preamble(self, f, title, meta):
        """Common CSV header: title + attribution lines + the meta block."""
        f.write("# %s\n" % title)
        f.write("# Plugin by Steven (Fragmon) — Crydteam\n")
        f.write("# YouTube: https://www.youtube.com/@crydteamprinting\n")
        for k, v in meta.items():
            f.write("# %s: %s\n" % (k, v))

    @staticmethod
    def _meta_to_html(meta):
        """Render the meta dict as the report's <div> grid entries."""
        return ''.join('<div><strong>%s:</strong> %s</div>' % (k, v)
                       for k, v in meta.items())

    def _write_csv(self, path, results, meta, kind):
        with open(path, 'w') as f:
            self._write_csv_preamble(
                f, "Speed Test v%s results — %s" % (MODULE_VERSION, kind),
                meta)
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

        meta_html = self._meta_to_html(meta)

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
