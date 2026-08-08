# Motor Sync — StallGuard-based dual-motor axis synchronization
# credits:
#   Steven (Fragmon) — Crydteam
#   YouTube: https://www.youtube.com/@crydteamprinting
#
#   The synchronization approach (correct one motor of a dual-motor
#   axis in microsteps, hill-descent on a measured mismatch magnitude
#   with direction probing and step-size refinement) is adapted from
#   "motors-sync" by Maksim Bolgov (MRX8024), GPLv3:
#   https://github.com/MRX8024/motors-sync/blob/main/motors_sync.py
#
#   The MEASUREMENT differs fundamentally: motors-sync measures the
#   enable-snap impact with an accelerometer (ADXL) or an encoder.
#   This plugin needs no extra sensor — two motors that are out of
#   phase fight each other during any move, and that extra load is
#   visible in the TMC drivers' StallGuard signal. We measure SG of
#   BOTH drivers during a slow constant-speed move and shift one
#   motor until the combined SG is maximal (= minimal fight).
#
# License: GPLv3

import logging
import math

MODULE_NAME = "Motor Sync"
MODULE_VERSION = "1.0.0"

TRINAMIC_DRIVERS = ("tmc2130", "tmc2209", "tmc2240", "tmc5160", "tmc2660")
SG2_DRIVERS = ("tmc2130", "tmc2240", "tmc5160", "tmc2660")

SAMPLE_INTERVAL = 0.02        # 50 Hz SG polling (same as tmc_flow_test)
TRIM_FRACTION = 0.20          # drop accel/decel transients at both ends
SHIFT_SPEED = 5.0             # mm/s for single-motor correction moves
SHIFT_ACCEL = 1000.0


class _TmcHandle:
    """One stepper's TMC driver with a direct SG read path."""

    def __init__(self, stepper_name, tmc, driver_type):
        self.stepper_name = stepper_name
        self.tmc = tmc
        self.driver_type = driver_type
        self.is_2209 = driver_type == "tmc2209"
        self.sg2 = driver_type in SG2_DRIVERS

    def read_sg(self):
        # Same register paths as tmc_flow_test: TMC2209 has a dedicated
        # SG_RESULT register (SG4); the SG2 family carries SG_RESULT in
        # DRV_STATUS bits 0-9. TMC2240 runs the SG2 path.
        try:
            if self.is_2209:
                return self.tmc.mcu_tmc.get_register('SG_RESULT') & 0x3FF
            return self.tmc.mcu_tmc.get_register('DRV_STATUS') & 0x3FF
        except Exception as e:
            logging.debug("motor_sync: SG read failed on %s: %s",
                          self.stepper_name, e)
            return None


class MotorSync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.default_axis = config.get('default_axis', 'X').upper()
        # Slow constant-speed measurement move. Must be fast enough
        # that TSTEP stays below the driver's coolstep_threshold so
        # StallGuard is active.
        self.buzz_speed = config.getfloat('buzz_speed', 30.0, above=0.)
        self.buzz_distance = config.getfloat('buzz_distance', 40.0,
                                             above=5.)
        self.repeats = config.getint('repeats', 2, minval=1)
        self.travel_speed = config.getfloat('travel_speed', 100.0,
                                            above=0.)
        # Search parameters
        self.coarse_msteps = config.getint('coarse_msteps', 4, minval=1)
        self.max_offset_fullsteps = config.getfloat(
            'max_offset_fullsteps', 2.0, above=0.)
        # Minimum SG-score improvement (raw SG units, summed over both
        # drivers) to accept a step as "better" — guards against noise.
        self.min_gain = config.getfloat('min_gain', 4.0, minval=0.)

        # Make sure the force_move module (single-stepper moves) exists
        self.printer.load_object(config, 'force_move')

        # Sampling state
        self.sampling_active = False
        self.sample_timer = None
        self.samples = ([], [])

        self.gcode.register_command(
            'MOTOR_SYNC', self.cmd_MOTOR_SYNC,
            desc='Synchronize the two motors of a dual-motor axis via '
                 'TMC StallGuard load measurement (no accelerometer '
                 'needed)')
        self.gcode.register_command(
            'MOTOR_SYNC_STATUS', self.cmd_MOTOR_SYNC_STATUS,
            desc='Show dual-motor axes, TMC drivers and motor_sync '
                 'config')

    # ─── Lookup helpers ─────────────────────────────────────────────

    def _get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def _find_axis_steppers(self, axis):
        """Return (primary_stepper, secondary_stepper) of a dual rail."""
        kin = self._get_toolhead().get_kinematics()
        want = 'stepper_' + axis.lower()
        for rail in getattr(kin, 'rails', []):
            steppers = rail.get_steppers()
            names = [s.get_name() for s in steppers]
            if want in names:
                primary = steppers[names.index(want)]
                extras = [s for s in steppers
                          if s.get_name().startswith(want)
                          and s.get_name() != want]
                if not extras:
                    raise self.gcode.error(
                        "motor_sync: axis %s has only one motor "
                        "(%s). A second stepper section like [%s1] "
                        "is required." % (axis, want, want))
                if len(extras) > 1:
                    raise self.gcode.error(
                        "motor_sync: axis %s has %d extra motors — "
                        "only dual-motor axes are supported in this "
                        "version." % (axis, len(extras)))
                return rail, primary, extras[0]
        raise self.gcode.error(
            "motor_sync: no rail with stepper '%s' found for axis %s."
            % (want, axis))

    def _find_tmc(self, stepper_name):
        for drv in TRINAMIC_DRIVERS:
            tmc = self.printer.lookup_object(
                "%s %s" % (drv, stepper_name), None)
            if tmc is not None:
                return _TmcHandle(stepper_name, tmc, drv)
        raise self.gcode.error(
            "motor_sync: no TMC driver with StallGuard found for "
            "'%s'. Supported: %s. (TMC2208/2225 have no StallGuard.)"
            % (stepper_name, ", ".join(TRINAMIC_DRIVERS)))

    @staticmethod
    def _microsteps(handle):
        """Microsteps per full step, read from the driver's mres field."""
        try:
            return 256 >> handle.tmc.fields.get_field('mres')
        except Exception:
            return 16

    # ─── Sampling ───────────────────────────────────────────────────

    def _start_sampling(self, handles):
        self.samples = ([], [])
        self._handles = handles
        self.sampling_active = True
        self.sample_timer = self.reactor.register_timer(
            self._sample_callback, self.reactor.NOW)

    def _stop_sampling(self):
        self.sampling_active = False
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None

    def _sample_callback(self, eventtime):
        if not self.sampling_active:
            return self.reactor.NEVER
        for i, handle in enumerate(self._handles):
            sg = handle.read_sg()
            if sg is not None:
                self.samples[i].append(sg)
        return eventtime + SAMPLE_INTERVAL

    @staticmethod
    def _median(vals):
        if not vals:
            return None
        s = sorted(vals)
        n = len(s)
        if n % 2:
            return float(s[n // 2])
        return (s[n // 2 - 1] + s[n // 2]) / 2.0

    def _trimmed_median(self, vals):
        n = len(vals)
        if n < 10:
            return self._median(vals)
        cut = max(1, int(n * TRIM_FRACTION))
        return self._median(vals[cut:n - cut])

    # ─── Measurement ────────────────────────────────────────────────

    def _measure_score(self, gcmd, ctx):
        """SG score of one measurement: sum of both drivers' trimmed
        medians, averaged over `repeats` passes. HIGHER = less fight
        between the motors."""
        toolhead = self._get_toolhead()
        axis = ctx['axis']
        start, end = ctx['seg_start'], ctx['seg_end']
        med_a = []
        med_b = []
        for rep in range(ctx['repeats']):
            self.gcode.run_script_from_command(
                "G90\nG1 %s%.3f F%.0f" % (axis, start,
                                          self.travel_speed * 60.0))
            toolhead.wait_moves()
            self._start_sampling(ctx['handles'])
            try:
                self.gcode.run_script_from_command(
                    "G1 %s%.3f F%.0f\nM400"
                    % (axis, end, ctx['buzz_speed'] * 60.0))
            finally:
                self._stop_sampling()
            a = self._trimmed_median(self.samples[0])
            b = self._trimmed_median(self.samples[1])
            if a is None or b is None:
                raise self.gcode.error(
                    "motor_sync: no SG samples received — check the "
                    "TMC wiring/UART/SPI of both drivers.")
            if a <= 0 and b <= 0:
                raise self.gcode.error(
                    "motor_sync: StallGuard reads 0 during the move. "
                    "SG is velocity-gated — make sure both driver "
                    "sections have 'coolstep_threshold' set below "
                    "%.0f mm/s (SG2 drivers also need SpreadCycle: "
                    "no stealthchop_threshold)." % ctx['buzz_speed'])
            gcmd.respond_info(
                "    pass %d/%d %s: %s=%.0f  %s=%.0f  (n=%d+%d)"
                % (rep + 1, ctx['repeats'],
                   '→' if rep % 2 == 0 else '←',
                   ctx['handles'][0].stepper_name, a,
                   ctx['handles'][1].stepper_name, b,
                   len(self.samples[0]), len(self.samples[1])))
            med_a.append(a)
            med_b.append(b)
            # Swap direction each pass so belt/friction asymmetry
            # averages out
            start, end = end, start
        n = float(len(med_a))
        score = (sum(med_a) + sum(med_b)) / n
        return score, sum(med_a) / n, sum(med_b) / n

    # ─── Single-motor correction moves ──────────────────────────────

    def _shift_secondary(self, ctx, msteps):
        """Move ONLY the secondary motor by `msteps` microsteps."""
        if not msteps:
            return
        toolhead = self._get_toolhead()
        toolhead.wait_moves()
        force_move = self.printer.lookup_object('force_move')
        dist = msteps * ctx['sec_step_dist']
        force_move.manual_move(ctx['sec_stepper'], dist,
                               SHIFT_SPEED, SHIFT_ACCEL)
        toolhead.wait_moves()

    # ─── Commands ───────────────────────────────────────────────────

    def cmd_MOTOR_SYNC(self, gcmd):
        axis = gcmd.get('AXIS', self.default_axis).upper()
        if axis not in ('X', 'Y', 'Z'):
            raise self.gcode.error("motor_sync: AXIS must be X, Y or Z")
        buzz_speed = gcmd.get_float('BUZZ_SPEED', self.buzz_speed,
                                    above=0.)
        buzz_dist = gcmd.get_float('BUZZ_DIST', self.buzz_distance,
                                   above=5.)
        repeats = gcmd.get_int('REPEATS', self.repeats, minval=1)
        coarse = gcmd.get_int('COARSE', self.coarse_msteps, minval=1)
        max_off_fs = gcmd.get_float('MAX_OFFSET',
                                    self.max_offset_fullsteps, above=0.)
        min_gain = gcmd.get_float('MIN_GAIN', self.min_gain, minval=0.)

        rail, primary, secondary = self._find_axis_steppers(axis)
        handle_a = self._find_tmc(primary.get_name())
        handle_b = self._find_tmc(secondary.get_name())
        microsteps = self._microsteps(handle_b)
        max_offset = int(round(max_off_fs * microsteps))

        # Home if needed
        kin = self._get_toolhead().get_kinematics()
        homed = kin.get_status(self.reactor.monotonic())['homed_axes']
        if axis.lower() not in homed:
            gcmd.respond_info("motor_sync: homing %s first..." % axis)
            self.gcode.run_script_from_command("G28 %s" % axis)

        # Measurement segment, centred in the axis range
        pos_min, pos_max = rail.get_range()
        mid = (pos_min + pos_max) / 2.0
        seg = min(buzz_dist, (pos_max - pos_min) * 0.8) / 2.0
        ctx = {
            'axis': axis,
            'handles': (handle_a, handle_b),
            'sec_stepper': secondary,
            'sec_step_dist': secondary.get_step_dist(),
            'seg_start': mid - seg,
            'seg_end': mid + seg,
            'buzz_speed': buzz_speed,
            'repeats': repeats,
        }

        gcmd.respond_info(
            "──── MOTOR_SYNC %s ────\n"
            "  motors: %s (%s) + %s (%s) | microsteps=%d\n"
            "  buzz_speed=%.0f mm/s buzz_distance=%.0f mm repeats=%d\n"
            "  coarse=%d msteps, max_offset=%d msteps (%.1f full "
            "steps), min_gain=%.1f"
            % (axis, handle_a.stepper_name, handle_a.driver_type,
               handle_b.stepper_name, handle_b.driver_type, microsteps,
               buzz_speed, seg * 2.0, repeats,
               coarse, max_offset, max_off_fs, min_gain))

        applied = 0
        try:
            gcmd.respond_info("  measuring baseline (%d passes)..."
                              % repeats)
            best, med_a, med_b = self._measure_score(gcmd, ctx)
            init_score = best
            gcmd.respond_info(
                "  baseline: SG score = %.1f (%s=%.1f, %s=%.1f) — "
                "higher = less fight"
                % (best, handle_a.stepper_name, med_a,
                   handle_b.stepper_name, med_b))

            # Hill-descent with step-size refinement (approach adapted
            # from MRX8024/motors-sync): probe both directions at the
            # current step size, keep going while the score improves,
            # then halve the step.
            step = coarse
            while step >= 1:
                for direction in (1, -1):
                    while True:
                        if abs(applied + direction * step) > max_offset:
                            break
                        self._shift_secondary(ctx, direction * step)
                        applied += direction * step
                        gcmd.respond_info(
                            "  testing %+d msteps (total %+d)..."
                            % (direction * step, applied))
                        score, med_a, med_b = self._measure_score(
                            gcmd, ctx)
                        if score > best + min_gain:
                            gcmd.respond_info(
                                "    → score %.1f (was %.1f) — "
                                "improved, keeping %+d msteps"
                                % (score, best, applied))
                            best = score
                        else:
                            self._shift_secondary(
                                ctx, -direction * step)
                            applied -= direction * step
                            gcmd.respond_info(
                                "    → score %.1f — no gain, reverted "
                                "to %+d msteps" % (score, applied))
                            break
                step //= 2

            gcmd.respond_info(
                "──── done ────\n"
                "  applied offset: %+d/%d msteps on %s (%.4f mm)\n"
                "  SG score: %.1f → %.1f\n"
                "  NOTE: the correction is lost when the motors are "
                "disabled (M84/power-off). Run MOTOR_SYNC after every "
                "power-on, e.g. from PRINT_START."
                % (applied, microsteps, handle_b.stepper_name,
                   applied * ctx['sec_step_dist'], init_score, best))
        except Exception:
            # Leave the axis as close to the starting state as we can
            if applied:
                try:
                    self._shift_secondary(ctx, -applied)
                except Exception:
                    logging.exception(
                        "motor_sync: revert after error failed")
            raise

    def cmd_MOTOR_SYNC_STATUS(self, gcmd):
        lines = ["%s v%s" % (MODULE_NAME, MODULE_VERSION)]
        kin = self._get_toolhead().get_kinematics()
        found = False
        for rail in getattr(kin, 'rails', []):
            steppers = rail.get_steppers()
            names = [s.get_name() for s in steppers]
            base = [n for n in names
                    if n in ('stepper_x', 'stepper_y', 'stepper_z')]
            if not base or len(names) < 2:
                continue
            axis = base[0][-1].upper()
            drvs = []
            for name in names:
                try:
                    h = self._find_tmc(name)
                    drvs.append("%s (%s)" % (name, h.driver_type))
                except Exception:
                    drvs.append("%s (no SG driver!)" % name)
            found = True
            lines.append("  axis %s: %s" % (axis, " + ".join(drvs)))
        if not found:
            lines.append("  no dual-motor axis found — motor_sync "
                         "needs e.g. [stepper_x] + [stepper_x1]")
        lines.append(
            "  config: buzz_speed=%.0f mm/s [buzz_speed], "
            "buzz_distance=%.0f mm [buzz_distance], repeats=%d "
            "[repeats], coarse=%d msteps [coarse_msteps], "
            "max_offset=%.1f full steps [max_offset_fullsteps], "
            "min_gain=%.1f [min_gain]"
            % (self.buzz_speed, self.buzz_distance, self.repeats,
               self.coarse_msteps, self.max_offset_fullsteps,
               self.min_gain))
        gcmd.respond_info("\n".join(lines))


def load_config(config):
    return MotorSync(config)
