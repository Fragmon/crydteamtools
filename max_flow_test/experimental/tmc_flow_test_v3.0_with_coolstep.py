# TMC Flow Test — Adaptive max-volumetric-flow detection for extruders
# credits:
#   Steven (Fragmon) — Crydteam
#   YouTube: https://www.youtube.com/@crydteamprinting
#
# License: GPLv3

import logging
import math
import os
import statistics
import time
import json

SAMPLE_INTERVAL = 0.05    # 20 Hz polling
MIN_HOTEND_TEMP = 180.0
MODULE_NAME = "TMC Flow Test"
MODULE_VERSION = "3.0"
SG_MIN_INFORMATIVE = 50   # below this SG value, readings are noise


# ─── Driver-specific trigger profiles ──────────────────────────────────
# Each TMC chip family has its own SG noise characteristics, scale, and
# CoolStep behaviour. Rather than embedding "if self.sg2_driver / else"
# branches throughout the trigger code, all tunable thresholds are
# centralized here in a profile per driver. To adjust sensitivity for
# one driver family without affecting the others, edit only its profile.
#
# The base profile contains the values that have been validated against
# real TMC5160 test runs (CSVs from 11-37, 12-40, 12-56, 13-20). Other
# drivers START as identical copies and can be tuned independently from
# real data.

class TriggerProfile:
    """Threshold values used by the slip-detection triggers.

    Inherit and override only the fields that need tuning for a
    specific driver family. Values here are the validated TMC5160
    defaults — DO NOT change them without re-running the TMC5160
    regression suite.
    """

    # ─── _check_cv_spike thresholds ────────────────────────────────
    # Pattern (a) "high-variance trip" — last CV above this absolute
    # value triggers immediately, regardless of baseline.
    CV_HIGH_VARIANCE = 10.0

    # Pattern (b) "ratio jump from immediate baseline"
    CV_JUMP_RATIO_COARSE = 2.5
    CV_JUMP_RATIO_BISECT = 2.0
    CV_JUMP_MIN_COARSE = 5.0
    CV_JUMP_MIN_BISECT = 4.0

    # Pattern (c) "rising trend" — three consecutive ≥1.3× steps
    CV_RISING_RATIO = 1.3
    CV_RISING_MIN_PRIOR = 2.5
    CV_RISING_MIN_LAST_COARSE = 5.0
    CV_RISING_MIN_LAST_BISECT = 4.0

    # Pattern (d) "vs coarse-phase baseline" (bisection only)
    CV_VS_COARSE_RATIO = 2.0
    CV_VS_COARSE_MIN = 5.0

    # Single-step low-baseline jump check
    CV_LOWBASE_RATIO = 1.5

    # ─── _check_iqr_spread thresholds ──────────────────────────────
    # Pattern (a) "ratio vs immediate prior steps"
    IQR_RATIO_COARSE = 3.0
    IQR_RATIO_BISECT = 1.7
    IQR_RATIO_MIN_ABS = 12   # don't fire on tiny absolute IQRs
    # Pattern (a) CV cross-check: when True, pattern (a) only fires
    # if CV ALSO confirms elevated noise relative to prior steps.
    # Helps drivers (e.g. SG4 with active CoolStep) where CS regulation
    # widens within-step IQR without affecting run-to-run reproducibility
    # (CV stays low). TMC5160 was validated with this OFF — leave it
    # off for SG2 unless real data shows false positives there too.
    IQR_RATIO_REQUIRE_CV = False
    IQR_RATIO_CV_FLOOR_COARSE = 3.0   # absolute CV floor, coarse phase
    IQR_RATIO_CV_FLOOR_BISECT = 2.5   # absolute CV floor, bisection

    # Pattern (b) "vs coarse-phase median"
    IQR_VS_COARSE_RATIO = 2.5
    IQR_VS_COARSE_MIN_ABS = 18
    IQR_VS_COARSE_REQUIRE_CV = True   # CV cross-check on by default
    IQR_VS_COARSE_CV_FLOOR = 3.0
    IQR_VS_COARSE_CV_RATIO = 1.5

    # Pattern (c) "absolute" — trigger if IQR ≥ this in bisection
    IQR_ABSOLUTE_TRIGGER = 25

    # ─── _check_sg_max_spike thresholds ────────────────────────────
    SG_MAX_RATIO_TO_MEDIAN = 3.0
    SG_MAX_RATIO_TO_COARSE = 1.3
    SG_MAX_ABS_GAP = 200            # absolute gap floor (sg2_driver)
    SG_MAX_ABS_GAP_SG4 = 150        # absolute gap floor (sg4 chips)
    SG_MAX_BIG_RATIO = 4.0          # alt path — extreme ratio
    SG_MAX_BIG_GAP = 300            # alt path — extreme gap
    # Inverse direction (rare case where SG rises with load)
    SG_MIN_RATIO_TO_MEDIAN = 3.0
    SG_MIN_ABS_GAP = 80

    # ─── _check_run_outlier thresholds ─────────────────────────────
    OUTLIER_MAD_RATIO = 4.0         # deviation ≥ this × MAD
    OUTLIER_MIN_REL = 0.08          # AND ≥ 8 % of median
    OUTLIER_CS_DROP = 1.5           # CS-min ≤ median - this
    OUTLIER_CS_RANGE = 2.0          # OR CS spread ≥ this

    # ─── _is_borderline thresholds ─────────────────────────────────
    BORDER_CV_LOW = 4.0
    BORDER_CV_HIGH = 7.0
    BORDER_CV_RATIO = 1.5
    BORDER_IQR_LOW = 15
    BORDER_IQR_HIGH = 25
    BORDER_IQR_RATIO = 1.7
    BORDER_IQR_CV_FLOOR = 2.0       # CV must be at least this for IQR-only borderline

    # ─── SG-level / step-jump basics ───────────────────────────────
    # Used by _sg_min_informative and _sg_jump_threshold.
    # Note: sg2_driver returns -1 (no gating) for SG_MIN_INFORMATIVE,
    # while SG4 returns the SG_MIN_INFORMATIVE module constant.
    SG_JUMP_THRESHOLD = 5           # SG2: small jumps significant


class TMC5160Profile(TriggerProfile):
    """TMC5160 / TMC2130 / TMC2660 — SG2 + SpreadCycle.

    These are the validated production values. Do not change without
    running the regression suite against the historical CSVs.
    """
    pass  # uses base defaults (= validated 5160 values)


class TMC2240Profile(TriggerProfile):
    """TMC2240 — SG2 + SpreadCycle (high-torque path).

    The plugin runs TMC2240 in the same SG2/SpreadCycle path as the
    TMC5160, because the SG4/StealthChop path delivers ~50 % less peak
    torque (confirmed empirically: same Sherpa Mini reaches 110 mm³/s
    on TMC5160 SG2 but only 60 mm³/s on TMC2240 SG4).

    Since the hardware path is now identical to TMC5160, this profile
    inherits all the validated TMC5160 thresholds. Override here only
    if real TMC2240 SG2 test data shows a different signature.
    """
    pass  # uses base defaults (= validated TMC5160 SG2 values)


class TMC2209Profile(TriggerProfile):
    """TMC2209 — SG4 + StealthChop only.

    Currently identical to the base profile — adjust based on real
    TMC2209 test data when available.
    """
    # SG4 has wider range (0-510) and higher jump magnitudes
    SG_JUMP_THRESHOLD = 15


def get_trigger_profile(driver_type):
    """Return the appropriate TriggerProfile subclass for a given driver."""
    if driver_type == 'tmc2240':
        return TMC2240Profile
    if driver_type == 'tmc2209':
        return TMC2209Profile
    # Default: TMC5160-style SG2 profile (covers tmc5160, tmc2130, tmc2660,
    # and any unknown driver — safest fallback).
    return TMC5160Profile


class TMCFlowTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.stepper_name = config.get('extruder_stepper', 'extruder')
        self.filament_diameter = config.getfloat(
            'filament_diameter', 1.75, above=0.)
        self.melt_zone_length = config.getfloat(
            'melt_zone_length', 42.0, above=0.)
        self.min_hotend_temp = config.getfloat(
            'min_hotend_temp', MIN_HOTEND_TEMP, above=0.)

        config_dir = os.path.expanduser('~/printer_data/config')
        if not os.path.isdir(config_dir):
            config_dir = os.path.expanduser('~')
        default_dir = os.path.join(config_dir, 'Flowtest')
        self.output_dir = config.get('output_dir', default_dir)

        self.filament_area = math.pi * (self.filament_diameter / 2.0) ** 2

        # Driver detection state
        self.tmc = None
        self.driver_type = None   # 'tmc2240', 'tmc2209', 'tmc5160', etc.
        self.is_2240 = False
        self.is_2209 = False
        self.is_5160 = False
        # SG2 family (TMC5160, TMC2130, TMC2660) — StallGuard2 in SpreadCycle,
        # SG_RESULT in DRV_STATUS bits 0-9, threshold field 'sgt' (signed).
        self.sg2_driver = False
        self.sg4_available = False  # SG4_RESULT register (TMC2240)
        # Trigger threshold profile — replaced in _lookup_tmc with the
        # driver-appropriate subclass. Default = TMC5160 (validated).
        self.profile = TMC5160Profile

        # Sample buffers
        self.samples_sg = []
        self.samples_cs = []      # only used in CS mode
        self.samples_time = []
        self.sampling_active = False
        self.sample_timer = None
        self.sample_start_time = 0.0

        # Track which mode we're in for trigger logic
        self._mode = 'sg'         # 'sg' or 'cs'

        # Main command — auto-detects mode from driver_SEMIN
        self.gcode.register_command(
            'TMC_FLOW_FIND_MAX', self.cmd_TMC_FLOW_FIND_MAX,
            desc='Find max volumetric flow rate. Auto-detects test mode '
                 'from driver_SEMIN (CoolStep enabled or disabled)')
        # Legacy / explicit mode commands (still work for existing macros)
        self.gcode.register_command(
            'TMC_FLOW_FIND_MAX_SG', self.cmd_TMC_FLOW_FIND_MAX_SG,
            desc='Force SG-only mode (for setups with CoolStep disabled)')
        self.gcode.register_command(
            'TMC_FLOW_FIND_MAX_CS', self.cmd_TMC_FLOW_FIND_MAX_CS,
            desc='Force CoolStep + StallGuard mode '
                 '(for setups with CoolStep enabled)')
        self.gcode.register_command(
            'TMC_FLOW_STATUS', self.cmd_TMC_FLOW_STATUS,
            desc='Show current TMC StallGuard / CoolStep diagnostic values')

    # ─── TMC driver lookup ──────────────────────────────────────────

    def _lookup_tmc(self):
        if self.tmc is not None:
            return
        candidates = ['tmc2240', 'tmc5160', 'tmc2209', 'tmc2226',
                      'tmc2130', 'tmc2208', 'tmc2660']
        for drv in candidates:
            obj_name = '%s %s' % (drv, self.stepper_name)
            tmc = self.printer.lookup_object(obj_name, None)
            if tmc is not None:
                self.tmc = tmc
                self.driver_type = drv
                self.is_2240 = (drv == 'tmc2240')
                self.is_2209 = (drv == 'tmc2209')
                self.is_5160 = (drv == 'tmc5160')
                # TMC2240 is treated as an SG2 driver (SpreadCycle path).
                # Its SG4/StealthChop path delivers significantly less
                # torque at high speeds — confirmed empirically: the same
                # Sherpa Mini extruder reaches ~110 mm³/s on TMC5160 SG2
                # but only ~60 mm³/s on TMC2240 SG4. Since this plugin
                # exists to find MAX flow, we always use the high-torque
                # SG2 path on TMC2240.
                self.sg2_driver = drv in ('tmc5160', 'tmc2130',
                                          'tmc2660', 'tmc2240')
                # Pick the trigger threshold profile for this driver
                # family. Each profile is independently tunable; the
                # TMC5160 profile is the validated baseline.
                self.profile = get_trigger_profile(drv)
                # Check for SG4_RESULT register (TMC2240 only)
                try:
                    self.sg4_available = (
                        self.is_2240
                        and 'SG4_RESULT' in self.tmc.mcu_tmc.name_to_reg)
                except AttributeError:
                    self.sg4_available = False
                logging.info(
                    "tmc_flow_test: using %s for stepper '%s' "
                    "(SG4=%s, is_2209=%s, sg2=%s, profile=%s)",
                    drv, self.stepper_name, self.sg4_available,
                    self.is_2209, self.sg2_driver,
                    self.profile.__name__)
                return
        raise self.gcode.error(
            "tmc_flow_test: no TMC driver found for stepper '%s'."
            % self.stepper_name)

    # ─── Driver-specific field accessors ────────────────────────────

    def _get_sg_threshold_field_name(self):
        """Return the correct SG-threshold field name for this driver.

        TMC5160 / TMC2130 / TMC2660 / TMC2240 (SG2 path) use 'sgt'.
        TMC2209 uses 'sgthrs'.
        """
        if self.sg2_driver:
            # TMC5160 / TMC2130 / TMC2660 / TMC2240 — StallGuard2 threshold
            # (signed -64..63)
            return 'sgt'
        # TMC2209, TMC2226, TMC2208 use 'sgthrs'
        return 'sgthrs'

    def _get_sg_label(self):
        """Human-readable label for the SG signal.

        TMC2240 in SG2 mode uses SG_RESULT in DRV_STATUS just like the
        TMC5160 — the SG4_RESULT register is unused on the SG2 path.
        """
        return 'SG_RESULT'

    # ─── TMC config validation ──────────────────────────────────────

    def _check_tmc_config(self, mode):
        """Verify TMC driver is configured correctly for the requested mode.

        mode: 'sg' (CoolStep should be OFF: SEMIN=0)
              'cs' (CoolStep should be ON: SEMIN > 0)

        Returns (problems, infos).
        """
        problems = []
        infos = []
        if self.tmc is None:
            return ([('tmc', None, 'No TMC driver found')], infos)

        def get(name):
            try:
                return self.tmc.fields.get_field(name)
            except (KeyError, AttributeError):
                return None

        tpwmthrs = get('tpwmthrs')
        tcoolthrs = get('tcoolthrs')
        semin = get('semin')
        en_pwm_mode = get('en_pwm_mode')
        en_spread_cycle = get('en_spreadCycle')
        sg_thrs_field = self._get_sg_threshold_field_name()
        sg_thrs_val = get(sg_thrs_field)

        # ─── Chopper-mode check (driver-specific) ───
        # SG4 family (TMC2209 only now): StallGuard4 needs StealthChop ON
        # SG2 family (TMC5160, TMC2130, TMC2660, TMC2240):
        #   StallGuard2 needs SpreadCycle
        if self.is_2209:
            stealthchop_active = True
            stealthchop_indicator = None
            if en_spread_cycle is not None:
                stealthchop_active = (en_spread_cycle == 0)
                stealthchop_indicator = (
                    'en_spreadCycle', en_spread_cycle, 'should be 0')

            if not stealthchop_active and stealthchop_indicator:
                problems.append(
                    (stealthchop_indicator[0], stealthchop_indicator[1],
                     'StealthChop is not active. StallGuard4 needs '
                     'StealthChop ON.\n'
                     'Add to your [%s extruder] section:\n'
                     '  stealthchop_threshold: 999999' % self.driver_type))
            elif (tpwmthrs is not None and tpwmthrs > 0
                  and tpwmthrs < 0x10000):
                # Mid-range tpwmthrs would switch to SpreadCycle at higher
                # speeds.
                problems.append(
                    ('tpwmthrs', tpwmthrs,
                     'StealthChop only active below a velocity threshold '
                     '(tpwmthrs=%d). At higher flows the driver switches '
                     'to SpreadCycle and breaks StallGuard4.\n'
                     'Set:\n'
                     '  stealthchop_threshold: 999999' % tpwmthrs))
        elif self.sg2_driver:
            # TMC5160 / TMC2130 / TMC2660 / TMC2240 — SG2 requires SpreadCycle.
            #
            # Klipper's tmc.py writes tpwmthrs = 0xFFFFF (= 1048575) as the
            # default when stealthchop_threshold is NOT set in the config —
            # together with en_pwm_mode=0 this is the "SpreadCycle at all
            # speeds" state we actually want for SG2. So:
            #
            #   • en_pwm_mode is the authoritative signal: 0 = SpreadCycle.
            #   • Only flag tpwmthrs when en_pwm_mode is 1 AND tpwmthrs is in
            #     the mid-range (0 < tpwmthrs < 0xFFFFF), which would make
            #     the driver switch chopper mode based on velocity.
            #   • tpwmthrs == 0xFFFFF means "never enter StealthChop" and is
            #     the Klipper default → not a problem.
            stealthchop_default_tpwmthrs = 0xFFFFF  # 1048575
            if en_pwm_mode is not None and en_pwm_mode == 1:
                # Build TMC2240-specific advice if applicable
                if self.is_2240:
                    extra = (
                        '\nNote: this plugin uses TMC2240 in '
                        'SG2/SpreadCycle mode for max torque (~50 %% '
                        'higher peak flow than SG4/StealthChop). If you '
                        'use sensorless homing, set the chopper mode '
                        'temporarily for the test only.')
                else:
                    extra = ''
                if (tpwmthrs is not None and 0 < tpwmthrs
                        < stealthchop_default_tpwmthrs):
                    problems.append(
                        ('tpwmthrs', tpwmthrs,
                         '%s StallGuard2 needs SpreadCycle at all speeds, '
                         'but tpwmthrs=%d enables StealthChop below that '
                         'velocity.\n'
                         'Remove stealthchop_threshold from your [%s '
                         'extruder] section (or set it to 0).%s'
                         % (self.driver_type.upper(), tpwmthrs,
                            self.driver_type, extra)))
                else:
                    problems.append(
                        ('en_pwm_mode', en_pwm_mode,
                         'StealthChop is active. %s StallGuard2 only works '
                         'in SpreadCycle mode.\n'
                         'Remove stealthchop_threshold from your [%s '
                         'extruder] section (do not set it to 0 — remove '
                         'the line).%s'
                         % (self.driver_type.upper(), self.driver_type,
                            extra)))
            # else: en_pwm_mode == 0 → SpreadCycle is active. tpwmthrs is
            # irrelevant in this case (the driver never enters StealthChop),
            # whether it's 0 or 0xFFFFF.

        # ─── tcoolthrs check (StallGuard gate) ───
        if tcoolthrs == 0:
            problems.append(
                ('tcoolthrs', tcoolthrs,
                 'StallGuard reading disabled (tcoolthrs=0).\n'
                 'Add to your [%s extruder] section:\n'
                 '  coolstep_threshold: 0.5' % self.driver_type))

        # ─── SG threshold check ───
        # Only enforced for TMC2209 (SG4 path). For SG2 drivers
        # (TMC5160/2130/2660/2240), sgt is signed (-64..63) and
        # SG_RESULT can be read regardless of sgt; we don't depend on
        # the hardware stop trigger.
        if self.is_2209 and sg_thrs_val == 0:
            problems.append(
                (sg_thrs_field, sg_thrs_val,
                 'SGTHRS is 0. StallGuard trigger inactive.\n'
                 'Add to your [tmc2209 extruder] section:\n'
                 '  driver_SGTHRS: 100'))

        # ─── Mode-specific CoolStep check ───
        if mode == 'sg':
            # SG-only mode requires CoolStep off for SG-only triggers to apply
            if semin is not None and semin != 0:
                problems.append(
                    ('semin', semin,
                     'SG-only mode is meant for setups with CoolStep '
                     'disabled, but driver_SEMIN = %d.\n'
                     'Easiest fix: just run TMC_FLOW_FIND_MAX (without _SG)\n'
                     '— it auto-selects the right mode for your config.\n'
                     'Or change your [%s extruder] section to driver_SEMIN: 0\n'
                     'if you want to test in SG-only mode.'
                     % (semin, self.driver_type)))
            else:
                infos.append(
                    "SG-only mode: CoolStep is disabled (driver_SEMIN=0). "
                    "Motor runs at constant IRUN. Test will use SG-based "
                    "triggers only.")
        elif mode == 'cs':
            # CS mode requires CoolStep on for CS-based triggers to fire
            if semin == 0:
                problems.append(
                    ('semin', semin,
                     'CoolStep mode is meant for setups with CoolStep '
                     'enabled, but driver_SEMIN = 0.\n'
                     'Easiest fix: just run TMC_FLOW_FIND_MAX (without _CS)\n'
                     '— it auto-selects the right mode for your config.\n'
                     'Or change your [%s extruder] section to enable CoolStep:\n'
                     '  driver_SEMIN: 5\n'
                     '  driver_SEMAX: 2\n'
                     '  driver_SEUP: 2\n'
                     '  driver_SEDN: 1\n'
                     '  driver_SEIMIN: 1\n'
                     'if you want to test in CoolStep mode.'
                     % self.driver_type))
            else:
                infos.append(
                    "CoolStep mode: CoolStep is active (driver_SEMIN=%d). "
                    "Motor current adapts to load. Test will use CS-based "
                    "triggers with SG fallback." % semin)

        # Driver info
        if self.is_2209:
            infos.append(
                "Driver: TMC2209 detected (uses SG_RESULT, sgthrs).")
        elif self.is_2240:
            infos.append(
                "Driver: TMC2240 detected — using SG2/SpreadCycle path "
                "(SG_RESULT via DRV_STATUS, sgt threshold) for max "
                "torque. SG4/StealthChop path is intentionally bypassed "
                "since it delivers ~50%% less peak flow.")
        elif self.sg2_driver:
            infos.append(
                "Driver: %s detected (StallGuard2 in SpreadCycle, "
                "SG_RESULT via DRV_STATUS, sgt threshold)."
                % self.driver_type.upper())

        return (problems, infos)

    # ─── SG sampling ─────────────────────────────────────────────────

    def _read_sg(self):
        """Read StallGuard value directly from the driver register.

        TMC2209: SG_RESULT register (dedicated SG4 register)
        TMC5160 / TMC2130 / TMC2660 / TMC2240 (SG2 path):
            DRV_STATUS bits 0-9 (sg_result field)
        Other drivers: fallback via get_status()

        Note: TMC2240 has a separate SG4_RESULT register which we
        deliberately ignore — we run TMC2240 in SpreadCycle/SG2 mode
        for max torque, and SG2 lives in DRV_STATUS just like TMC5160.
        """
        # TMC2209 has its own SG_RESULT register (SG4)
        if self.is_2209:
            try:
                reg_val = self.tmc.mcu_tmc.get_register('SG_RESULT')
                return reg_val & 0x3FF
            except Exception as e:
                logging.debug(
                    "tmc_flow_test: SG_RESULT read failed: %s", e)
                return None

        # SG2 family (TMC5160 / TMC2130 / TMC2660 / TMC2240):
        # SG_RESULT is in DRV_STATUS bits 0-9
        if self.sg2_driver:
            try:
                reg_val = self.tmc.mcu_tmc.get_register('DRV_STATUS')
                return reg_val & 0x3FF
            except Exception as e:
                logging.debug(
                    "tmc_flow_test: DRV_STATUS read failed: %s", e)
                return None

        # Fallback for any other unknown driver
        try:
            drv = self.tmc.get_status(self.reactor.monotonic())
            if 'drv_status' in drv and isinstance(drv['drv_status'], dict):
                return drv['drv_status'].get('sg_result')
            return drv.get('sg_result')
        except Exception:
            return None

    def _read_cs(self):
        """Read CS_ACTUAL directly from DRV_STATUS register.

        Bits 16-20 of DRV_STATUS = CS_ACTUAL (5 bits, range 0-31).
        Same layout on TMC2240, TMC2209, TMC5160 / TMC2130 / TMC2660.
        """
        if self.is_2209 or self.sg2_driver:
            try:
                reg_val = self.tmc.mcu_tmc.get_register('DRV_STATUS')
                return (reg_val >> 16) & 0x1F
            except Exception as e:
                logging.debug(
                    "tmc_flow_test: DRV_STATUS read failed: %s", e)
                return None
        # Fallback for other drivers
        try:
            drv = self.tmc.get_status(self.reactor.monotonic())
            if 'drv_status' in drv and isinstance(drv['drv_status'], dict):
                return drv['drv_status'].get('cs_actual')
            return drv.get('cs_actual')
        except Exception:
            return None

    def _start_sampling(self, sample_cs=False):
        self.samples_sg = []
        self.samples_cs = []
        self.samples_time = []
        self._sample_cs_flag = sample_cs
        self.sample_start_time = self.reactor.monotonic()
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
        sg = self._read_sg()
        rel_t = eventtime - self.sample_start_time
        # For drivers we read directly from registers, accept any non-None
        # value (including 0). For fallback drivers via get_status, we
        # filter 0/None as those usually mean "not yet polled".
        direct_read = self.is_2209 or self.sg2_driver
        if sg is not None:
            if direct_read or sg > 0:
                self.samples_sg.append(sg)
                self.samples_time.append(rel_t)
                if self._sample_cs_flag:
                    cs = self._read_cs()
                    if cs is not None:
                        self.samples_cs.append(cs)
        return eventtime + SAMPLE_INTERVAL

    # ─── Statistics ─────────────────────────────────────────────────

    @staticmethod
    def _stats(samples):
        """Median + IQR + basic stats. Returns None if empty."""
        if not samples:
            return None
        sorted_s = sorted(samples)
        n = len(sorted_s)

        def percentile(p):
            if n == 1:
                return sorted_s[0]
            k = (n - 1) * p / 100.0
            f = int(k)
            c = min(f + 1, n - 1)
            return sorted_s[f] + (k - f) * (sorted_s[c] - sorted_s[f])

        return {
            'min': sorted_s[0], 'max': sorted_s[-1],
            'avg': sum(sorted_s) / n,
            'median': percentile(50),
            'p25': percentile(25),
            'p75': percentile(75),
            'std': statistics.pstdev(sorted_s) if n > 1 else 0.0,
            'n': n,
        }

    # ─── CSV / HTML output ──────────────────────────────────────────

    def _write_csv(self, path, results, meta, mode):
        with open(path, 'w') as f:
            f.write("# TMC Flow Test v%s results (mode: %s)\n"
                    % (MODULE_VERSION, mode))
            f.write("# Plugin by Steven (Fragmon) — Crydteam\n")
            f.write("# YouTube: https://www.youtube.com/@crydteamprinting\n")
            tmc_settings = None
            for k, v in meta.items():
                if k == 'tmc_settings':
                    tmc_settings = v
                    continue
                f.write("# %s: %s\n" % (k, v))
            # TMC settings as a separate, readable block at the end
            # of the comment header — useful as a paper trail.
            if tmc_settings:
                f.write("#\n# TMC driver settings at test start:\n")
                for label, value, raw in tmc_settings:
                    f.write("#   %s = %s  [%s]\n" % (label, value, raw))

            # CSV header depends on mode
            if mode == 'cs':
                f.write("phase,flow_mm3s,sg_median,sg_p25,sg_p75,sg_avg,"
                        "sg_min,sg_max,sg_n,cs_median,cs_p25,cs_p75,cs_avg,"
                        "n_repeats,sg_run_cv_pct,run_sg_avgs,run_cs_avgs\n")
            else:
                f.write("phase,flow_mm3s,sg_median,sg_p25,sg_p75,sg_avg,"
                        "sg_min,sg_max,sg_n,n_repeats,sg_run_cv_pct,"
                        "run_sg_avgs\n")

            for r in results:
                sg = r.get('sg') or {}
                cs = r.get('cs') or {}
                rc = r.get('run_consistency') or {}
                run_sg = r.get('run_sg_avgs') or []
                run_cs = r.get('run_cs_avgs') or []
                phase = r.get('phase', 'coarse')

                def fmt(d, key):
                    v = d.get(key, '')
                    if isinstance(v, float):
                        return "%.1f" % v
                    return str(v)

                if mode == 'cs':
                    f.write("%s,%.2f,%s,%s,%s,%s,%s,%s,%s,"
                            "%s,%s,%s,%s,%d,%s,%s,%s\n" % (
                        phase,
                        r['flow'],
                        fmt(sg, 'median'), fmt(sg, 'p25'), fmt(sg, 'p75'),
                        fmt(sg, 'avg'), sg.get('min', ''), sg.get('max', ''),
                        sg.get('n', 0),
                        fmt(cs, 'median'), fmt(cs, 'p25'), fmt(cs, 'p75'),
                        fmt(cs, 'avg'),
                        len(run_sg),
                        "%.1f" % rc.get('sg_cv', 0) if rc else '',
                        '|'.join("%.1f" % v for v in run_sg),
                        '|'.join("%.1f" % v for v in run_cs),
                    ))
                else:
                    f.write("%s,%.2f,%s,%s,%s,%s,%s,%s,%s,"
                            "%d,%s,%s\n" % (
                        phase,
                        r['flow'],
                        fmt(sg, 'median'), fmt(sg, 'p25'), fmt(sg, 'p75'),
                        fmt(sg, 'avg'), sg.get('min', ''), sg.get('max', ''),
                        sg.get('n', 0),
                        len(run_sg),
                        "%.1f" % rc.get('sg_cv', 0) if rc else '',
                        '|'.join("%.1f" % v for v in run_sg),
                    ))

    def _write_html(self, path, results, meta, limit_reason, mode,
                    final_result=None):
        """Compact HTML report with chart.

        final_result (optional dict): when present, renders the prominent
        result panel with the optimal flow value. Expected keys:
            max_safe (float)         -- mm³/s, the optimal value
            verify_cv (float)        -- run-to-run CV in percent
            quality (str)            -- e.g. 'good (stable)'
            stop_reason (str|None)   -- last trigger reason (optional)
            trigger_events (list)    -- chronological trigger / borderline
                                         / verify-fail events for the
                                         decision-trail panel
            baseline_stats (dict)    -- "healthy" CV/IQR ranges from the
                                         coarse phase (used by the
                                         decision-trail and chart bands)
        """
        flows = [r['flow'] for r in results]
        phases = [r.get('phase', 'coarse') for r in results]
        sg_label = self._get_sg_label()
        sg_median = [r['sg']['median'] if r['sg'] else None for r in results]
        sg_p25 = [r['sg']['p25'] if r['sg'] else None for r in results]
        sg_p75 = [r['sg']['p75'] if r['sg'] else None for r in results]
        sg_avg = [r['sg']['avg'] if r['sg'] else None for r in results]

        cs_chart_html = ""
        cs_chart_script = ""
        if mode == 'cs':
            cs_median = [r['cs']['median'] if r.get('cs') else None
                         for r in results]
            cs_chart_html = """
<div class="chart-container">
  <h2>CS_ACTUAL vs. Flow Rate</h2>
  <p>CoolStep current scale (0-31). Higher = more current applied.
     Drops indicate motor needs less torque (or has lost load entirely).</p>
  <canvas id="csChart"></canvas>
</div>
"""
            cs_chart_script = """
const csMedian = %s;
new Chart(document.getElementById('csChart'), {
    type: 'line',
    data: { labels: flows,
        datasets: [{ label: 'CS_ACTUAL median', data: csMedian,
                     borderColor: '#d32f2f', fill: false, borderWidth: 2,
                     pointRadius: 4 }] },
    options: { ...commonOptions, scales: { ...commonOptions.scales,
        y: { title: { display: true, text: 'CS_ACTUAL (0-31)' },
             min: 0, max: 32 } } },
});
""" % json.dumps(cs_median)

        if final_result is not None:
            max_safe = final_result.get('max_safe')
            verify_cv = final_result.get('verify_cv', 0.0)
            quality = final_result.get('quality', '')
            stop_reason = final_result.get('stop_reason')
            trigger_events = final_result.get('trigger_events') or []
            baseline_stats = final_result.get('baseline_stats')

            stop_html = ''
            if stop_reason:
                stop_html = (
                    '<div class="stop-line">Stop trigger that ended the '
                    'search: <em>%s</em></div>' % stop_reason)

            summary_html = (
                '<div class="summary final">'
                '<h2>Maximum Safe Volumetric Flow</h2>'
                '<div class="big-number">%.1f<span class="unit">'
                'mm³/s</span></div>'
                '<div class="quality-line">Verification quality: '
                '<strong>%s</strong> (CV = %.1f%%)</div>'
                '<div class="recommendations">'
                '<div class="rec-table">'
                '<span class="rec-label">Conservative (80%%)</span>'
                '<span class="rec-value">%.1f mm³/s</span>'
                '<span class="rec-note">recommended for slicer</span>'
                '<span class="rec-label">Aggressive (90%%)</span>'
                '<span class="rec-value">%.1f mm³/s</span>'
                '<span class="rec-note">only with margin</span>'
                '</div>'
                '</div>'
                '%s'
                '</div>'
                % (max_safe, quality, verify_cv,
                   max_safe * 0.8, max_safe * 0.9, stop_html))
        elif limit_reason:
            summary_html = (
                '<div class="summary"><h2>Result</h2>'
                '<p>Stop reason: <strong>%s</strong></p></div>'
                % limit_reason)
            trigger_events = []
            baseline_stats = None
            max_safe = None
        else:
            summary_html = (
                '<div class="summary"><h2>Result</h2>'
                '<p>Test completed without trigger.</p></div>')
            trigger_events = []
            baseline_stats = None
            max_safe = None

        # ─── Decision-trail panel: WHY the result is what it is ───
        decision_trail_html = ''
        if trigger_events or baseline_stats:
            kind_labels = {
                'trigger': ('🚨', 'TRIGGER',
                            'Slip detected — phase ended at this flow'),
                'borderline-retest': ('🟡', 'BORDERLINE',
                                      'In gray zone — re-measured for '
                                      'confirmation'),
                'borderline-confirmed': ('🚨', 'BORDERLINE CONFIRMED',
                                         'Re-test also borderline — '
                                         'treated as trigger'),
                'verify-borderline': ('🟡', 'VERIFY BORDERLINE',
                                      'Verify in gray zone — re-measured'),
                'verify-fail': ('⚠️', 'VERIFY FAILED',
                                'Verify trigged at this flow — fell back '
                                'into bisection'),
            }
            event_rows = []
            for ev in trigger_events:
                icon, label, _desc = kind_labels.get(
                    ev['kind'], ('•', ev['kind'].upper(), ''))
                metric_bits = []
                if ev.get('cv') is not None:
                    metric_bits.append('CV %.1f%%' % ev['cv'])
                if ev.get('iqr') is not None:
                    metric_bits.append('IQR %.0f' % ev['iqr'])
                if ev.get('sg_median') is not None:
                    metric_bits.append('%s %.0f'
                                       % (sg_label, ev['sg_median']))
                metrics_str = ' · '.join(metric_bits) if metric_bits else '—'
                event_rows.append(
                    '<tr class="ev-%s">'
                    '<td class="ev-icon">%s</td>'
                    '<td class="ev-phase">%s</td>'
                    '<td class="ev-flow">%.1f mm³/s</td>'
                    '<td class="ev-label">%s</td>'
                    '<td class="ev-metrics">%s</td>'
                    '<td class="ev-reason">%s</td>'
                    '</tr>'
                    % (ev['kind'], icon,
                       ev['phase'], ev['flow'], label,
                       metrics_str, ev['reason']))

            # Baseline summary row
            baseline_block = ''
            if baseline_stats:
                baseline_block = (
                    '<div class="baseline-stats">'
                    '<h3>Healthy ranges (from %d coarse-phase steps)</h3>'
                    '<p class="baseline-explainer">'
                    'These are the typical CV and IQR values during the '
                    'slip-free portion of the test. Trigger thresholds '
                    'are calibrated relative to these baselines: '
                    '<strong>any value far outside this range was treated '
                    'as evidence of slip</strong>.'
                    '</p>'
                    '<table class="baseline-table">'
                    '<thead><tr><th>Metric</th><th>Min</th><th>Median</th>'
                    '<th>Mean</th><th>Max</th>'
                    '<th>Trigger floor (bisection)</th></tr></thead>'
                    '<tbody>'
                    '<tr><td>Run-to-run CV (%%)</td>'
                    '<td>%.1f</td><td>%.1f</td><td>%.1f</td><td>%.1f</td>'
                    '<td>≥ 4 %% AND ≥ 2.0× median, OR ≥ 7 %% absolute</td>'
                    '</tr>'
                    '<tr><td>IQR (P75 − P25, raw SG units)</td>'
                    '<td>%.0f</td><td>%.0f</td><td>%.0f</td><td>%.0f</td>'
                    '<td>≥ 18 AND ≥ 2.5× median, OR ≥ 25 absolute</td>'
                    '</tr>'
                    '</tbody></table>'
                    '</div>'
                    % (baseline_stats['n_steps'],
                       baseline_stats['cv_min'],
                       baseline_stats['cv_median'],
                       baseline_stats['cv_mean'],
                       baseline_stats['cv_max'],
                       baseline_stats['iqr_min'],
                       baseline_stats['iqr_median'],
                       baseline_stats['iqr_mean'],
                       baseline_stats['iqr_max']))

            events_block = ''
            if event_rows:
                events_block = (
                    '<h3>Decision events (chronological)</h3>'
                    '<p class="trail-explainer">Each event below is a '
                    'point where the plugin detected something unusual '
                    'and changed its decision. The bottom-most '
                    'event-row is what produced the final result.</p>'
                    '<div class="trail-table-wrap">'
                    '<table class="trail-table">'
                    '<thead><tr>'
                    '<th></th><th>Phase</th><th>Flow</th><th>Verdict</th>'
                    '<th>Metrics</th><th>Reason</th>'
                    '</tr></thead><tbody>%s</tbody></table>'
                    '</div>'
                    % ''.join(event_rows))

            decision_trail_html = (
                '<details class="decision-trail" open>'
                '<summary>Why this value? — Decision trail</summary>'
                '%s%s'
                '</details>' % (baseline_block, events_block))

        # Build meta info, but skip the tmc_settings list (rendered
        # separately below as its own collapsible section).
        meta_html = ''.join(
            '<div><strong>%s:</strong> %s</div>' % (k, v)
            for k, v in meta.items() if k != 'tmc_settings')

        # Build TMC settings block (collapsible <details> for compactness)
        tmc_settings = meta.get('tmc_settings') or []
        if tmc_settings:
            tmc_rows = ''.join(
                '<tr><td style="text-align:left">%s</td>'
                '<td style="text-align:right;font-family:monospace">%s</td>'
                '<td style="text-align:left;color:#888;font-family:monospace">'
                '%s</td></tr>'
                % (label, value, raw)
                for label, value, raw in tmc_settings)
            tmc_settings_html = (
                '<details class="tmc-settings"><summary>'
                'TMC driver settings at test start (%d fields) — '
                'click to expand</summary>'
                '<table class="tmc-settings-table"><thead><tr>'
                '<th style="text-align:left">Setting</th>'
                '<th style="text-align:right">Value</th>'
                '<th style="text-align:left">Field</th>'
                '</tr></thead><tbody>%s</tbody></table>'
                '</details>'
                % (len(tmc_settings), tmc_rows))
        else:
            tmc_settings_html = ''

        # Data table
        rows = []
        for r in results:
            sg = r.get('sg') or {}
            cs = r.get('cs') or {}
            rc = r.get('run_consistency') or {}
            cv_str = ''
            cv_class = ''
            if rc and 'sg_cv' in rc:
                cv = rc['sg_cv']
                cv_str = "%.1f%%" % cv
                if cv > 25:
                    cv_class = ' style="background:#ffcdd2"'
                elif cv > 10:
                    cv_class = ' style="background:#fff9c4"'

            def fmt(d, key, fs="%.1f"):
                v = d.get(key)
                if v is None or v == '':
                    return '-'
                return fs % v

            if mode == 'cs':
                rows.append(
                    "<tr><td>%.1f</td><td><b>%s</b></td>"
                    "<td>%s</td><td>%s</td><td><b>%s</b></td>"
                    "<td>%d</td><td%s>%s</td></tr>" % (
                        r['flow'], fmt(sg, 'median'),
                        fmt(sg, 'p25'), fmt(sg, 'p75'),
                        fmt(cs, 'median'), sg.get('n', 0),
                        cv_class, cv_str or '-'))
            else:
                rows.append(
                    "<tr><td>%.1f</td><td><b>%s</b></td>"
                    "<td>%s</td><td>%s</td><td>%s</td>"
                    "<td>%d</td><td%s>%s</td></tr>" % (
                        r['flow'], fmt(sg, 'median'),
                        fmt(sg, 'p25'), fmt(sg, 'p75'), fmt(sg, 'avg'),
                        sg.get('n', 0), cv_class, cv_str or '-'))

        if mode == 'cs':
            table_header = (
                "<th>Flow (mm³/s)</th><th>%s median</th>"
                "<th>%s P25</th><th>%s P75</th><th>CS median</th>"
                "<th>n</th><th>Inter-run CV</th>"
                % (sg_label, sg_label, sg_label))
        else:
            table_header = (
                "<th>Flow (mm³/s)</th><th>%s median</th>"
                "<th>%s P25</th><th>%s P75</th><th>%s avg</th>"
                "<th>n</th><th>Inter-run CV</th>"
                % (sg_label, sg_label, sg_label, sg_label))

        table = ("<table><thead><tr>" + table_header
                 + "</tr></thead><tbody>"
                 + "".join(rows) + "</tbody></table>")

        html = """<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<title>TMC Flow Test (%(mode)s) - %(timestamp)s</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-annotation@3.0.1"></script>
<style>
body { font-family: system-ui, sans-serif; max-width: 1200px;
       margin: 20px auto; padding: 0 20px; color: #333; }
h1 { color: #1565c0; }
.meta { background: #f5f5f5; padding: 15px; border-radius: 8px;
        margin-bottom: 20px; display: grid;
        grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
        gap: 8px; font-size: 14px; }
.summary { background: #e3f2fd; padding: 15px; border-radius: 8px;
           margin-bottom: 20px; border-left: 4px solid #1976d2; }
.summary h2 { margin: 0 0 10px 0; color: #1976d2; }
.summary.final { background: linear-gradient(135deg, #e8f5e9 0%%, #c8e6c9 100%%);
                 border-left: 6px solid #2e7d32; padding: 24px 28px; }
.summary.final h2 { color: #1b5e20; font-size: 22px;
                    margin: 0 0 6px 0; }
.big-number { font-size: 46px; font-weight: 700; color: #1b5e20;
              line-height: 1.1; margin: 8px 0 4px 0;
              font-variant-numeric: tabular-nums; }
.big-number .unit { font-size: 22px; font-weight: 500;
                    color: #2e7d32; margin-left: 6px; }
.quality-line { color: #2e7d32; font-size: 14px; margin-bottom: 14px; }
.recommendations { margin-top: 16px; padding-top: 14px;
                   border-top: 1px solid rgba(46,125,50,0.25); }
.rec-table { display: grid;
             grid-template-columns: auto auto 1fr;
             gap: 6px 14px; align-items: baseline; font-size: 14px; }
.rec-label { color: #1b5e20; font-weight: 600; }
.rec-value { font-variant-numeric: tabular-nums; font-weight: 600;
             color: #1b5e20; }
.rec-note { color: #2e7d32; font-size: 13px; }
.stop-line { margin-top: 12px; padding-top: 10px; font-size: 13px;
             color: #4e6e54;
             border-top: 1px dashed rgba(46,125,50,0.3); }
.chart-container { background: white; padding: 20px;
                   border-radius: 8px; margin-bottom: 20px;
                   box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
.tmc-settings { background: #fafafa; padding: 14px 18px;
                border-radius: 8px; margin-bottom: 20px;
                box-shadow: 0 1px 3px rgba(0,0,0,0.08);
                font-size: 13px; }
.tmc-settings summary { cursor: pointer; font-weight: 600;
                        color: #455a64; padding: 4px 0;
                        user-select: none; }
.tmc-settings summary:hover { color: #1976d2; }
.tmc-settings-table { width: 100%%; margin-top: 12px;
                      border-collapse: collapse; }
.tmc-settings-table th { background: #eceff1; padding: 6px 10px;
                         border: 1px solid #cfd8dc;
                         font-size: 12px; }
.tmc-settings-table td { padding: 4px 10px;
                         border: 1px solid #eceff1;
                         font-size: 12px; }
.decision-trail { background: white; padding: 18px 22px;
                  border-radius: 8px; margin-bottom: 20px;
                  box-shadow: 0 2px 4px rgba(0,0,0,0.08);
                  border-left: 4px solid #1976d2; }
.decision-trail summary { cursor: pointer; font-size: 16px;
                          font-weight: 600; color: #1565c0;
                          padding: 2px 0; user-select: none; }
.decision-trail summary:hover { color: #0d47a1; }
.decision-trail h3 { font-size: 14px; color: #37474f;
                     margin: 18px 0 6px 0; }
.decision-trail .baseline-explainer,
.decision-trail .trail-explainer {
                     font-size: 13px; color: #607d8b;
                     margin: 0 0 10px 0; line-height: 1.5; }
.baseline-table { width: 100%%; border-collapse: collapse;
                  font-size: 13px; margin-bottom: 6px; }
.baseline-table th { background: #eceff1; padding: 6px 10px;
                     text-align: right; border: 1px solid #cfd8dc; }
.baseline-table th:first-child { text-align: left; }
.baseline-table td { padding: 5px 10px;
                     border: 1px solid #eceff1;
                     text-align: right;
                     font-variant-numeric: tabular-nums; }
.baseline-table td:first-child { text-align: left;
                                 font-weight: 500; color: #455a64; }
.baseline-table td:last-child { text-align: left; color: #607d8b;
                                font-size: 12px; }
.trail-table-wrap { max-height: 420px; overflow-y: auto;
                    border-radius: 6px;
                    border: 1px solid #eceff1; }
.trail-table { width: 100%%; border-collapse: collapse;
               font-size: 13px; }
.trail-table thead { position: sticky; top: 0;
                     background: #eceff1; z-index: 1; }
.trail-table th { padding: 7px 10px; text-align: left;
                  border-bottom: 1px solid #cfd8dc;
                  font-size: 12px; color: #37474f; }
.trail-table td { padding: 7px 10px; vertical-align: top;
                  border-bottom: 1px solid #f5f5f5;
                  font-size: 12px; line-height: 1.4; }
.trail-table tr.ev-trigger { background: #ffebee; }
.trail-table tr.ev-borderline-confirmed { background: #ffebee; }
.trail-table tr.ev-borderline-retest { background: #fff8e1; }
.trail-table tr.ev-verify-borderline { background: #fff8e1; }
.trail-table tr.ev-verify-fail { background: #fff3e0; }
.trail-table .ev-icon { width: 24px; text-align: center;
                        font-size: 14px; }
.trail-table .ev-phase { text-transform: uppercase;
                         font-size: 11px; color: #607d8b;
                         font-weight: 600; }
.trail-table .ev-flow { font-variant-numeric: tabular-nums;
                        font-weight: 600; color: #263238;
                        white-space: nowrap; }
.trail-table .ev-label { font-weight: 600; color: #c62828;
                         white-space: nowrap; font-size: 11px; }
.trail-table tr.ev-borderline-retest .ev-label,
.trail-table tr.ev-verify-borderline .ev-label {
                         color: #ef6c00; }
.trail-table tr.ev-verify-fail .ev-label {
                         color: #d84315; }
.trail-table .ev-metrics { font-variant-numeric: tabular-nums;
                           color: #455a64;
                           font-family: monospace;
                           white-space: nowrap; }
.trail-table .ev-reason { color: #455a64; max-width: 420px; }
.footer { text-align: center; color: #666; padding: 20px;
          font-size: 13px; }
.footer a { color: #1976d2; }
table { width: 100%%; border-collapse: collapse; font-size: 13px; }
th, td { padding: 8px 12px; border: 1px solid #ddd; text-align: right; }
th { background: #f5f5f5; }
</style></head><body>
<h1>TMC Flow Test Results — %(mode_upper)s mode</h1>
<p style="color:#666;margin-top:-10px;">
  Plugin by Steven (Fragmon) — Crydteam ·
  <a href="https://www.youtube.com/@crydteamprinting"
     target="_blank">YouTube: @crydteamprinting</a></p>

<div class="meta">%(meta_html)s</div>
%(tmc_settings_html)s
%(summary_html)s
%(decision_trail_html)s

<div class="chart-container">
  <h2>%(sg_label)s vs. Flow Rate</h2>
  <p>Lower SG = higher mechanical load. Median is the robust statistic;
     IQR (P25-P75) shows sample spread.</p>
  <canvas id="sgChart"></canvas>
</div>

%(cs_chart_html)s

<div class="chart-container">
  <h2>Data Table</h2>
  %(data_table)s
</div>

<div class="footer">
  <p>Generated by <strong>TMC Flow Test</strong> v%(version)s
     at %(timestamp)s</p>
</div>

<script>
const flows = %(flows)s;
const phases = %(phases)s;
const sgMedian = %(sg_median)s;
const sgP25 = %(sg_p25)s;
const sgP75 = %(sg_p75)s;
const sgAvg = %(sg_avg)s;

// Build vertical-line annotations between phase transitions.
// xMin/xMax sit between two indices (e.g. 5.5) so the line falls in
// the gap between data points.
const phaseAnnotations = (() => {
    const labels = { coarse: 'Coarse', bisect: 'Bisection', verify: 'Verify' };
    const colors = { coarse: '#90a4ae', bisect: '#fb8c00', verify: '#43a047' };
    const ann = {};
    for (let i = 1; i < phases.length; i++) {
        if (phases[i] !== phases[i-1]) {
            const x = i - 0.5;
            ann['phase_' + i] = {
                type: 'line', xMin: x, xMax: x,
                borderColor: colors[phases[i]] || '#999',
                borderWidth: 2, borderDash: [6, 4],
                label: {
                    display: true, content: labels[phases[i]] || phases[i],
                    position: 'start', backgroundColor: colors[phases[i]] || '#999',
                    color: '#fff', font: { size: 11, weight: 'bold' },
                    padding: { top: 2, bottom: 2, left: 6, right: 6 },
                    yAdjust: -2,
                },
            };
        }
    }
    // Mark the very first phase too with a label at index 0
    if (phases.length > 0) {
        ann['phase_start'] = {
            type: 'line', xMin: -0.5, xMax: -0.5,
            borderColor: 'rgba(0,0,0,0)', borderWidth: 0,
            label: {
                display: true, content: labels[phases[0]] || phases[0],
                position: 'start',
                backgroundColor: colors[phases[0]] || '#999',
                color: '#fff', font: { size: 11, weight: 'bold' },
                padding: { top: 2, bottom: 2, left: 6, right: 6 },
                yAdjust: -2, xAdjust: 30,
            },
        };
    }
    return ann;
})();

// Trigger / borderline event markers — vertical lines at the data
// point where the event fired, plus a small label at the top of the
// chart describing the verdict.
const chartEvents = %(chart_events)s;
const eventAnnotations = (() => {
    const styles = {
        'trigger': {
            color: '#c62828', dash: [], label: 'TRIGGER', icon: '🚨',
        },
        'borderline-confirmed': {
            color: '#c62828', dash: [], label: 'BORDERLINE→TRIGGER',
            icon: '🚨',
        },
        'borderline-retest': {
            color: '#ef6c00', dash: [4, 4], label: 'BORDERLINE',
            icon: '🟡',
        },
        'verify-borderline': {
            color: '#ef6c00', dash: [4, 4], label: 'VERIFY BORDERLINE',
            icon: '🟡',
        },
        'verify-fail': {
            color: '#d84315', dash: [], label: 'VERIFY FAIL', icon: '⚠',
        },
    };
    const ann = {};
    chartEvents.forEach((ev, i) => {
        const s = styles[ev.kind] || { color: '#666', dash: [],
                                        label: ev.kind, icon: '•' };
        // Stagger label heights so multiple markers don't overlap.
        const labelOffset = (i %% 3) * 14;
        ann['ev_' + i] = {
            type: 'line',
            xMin: ev.idx, xMax: ev.idx,
            borderColor: s.color, borderWidth: 2,
            borderDash: s.dash,
            label: {
                display: true,
                content: s.icon + ' ' + s.label
                          + ' @ ' + ev.flow.toFixed(0) + ' mm³/s',
                position: 'start',
                backgroundColor: s.color,
                color: '#fff',
                font: { size: 10, weight: 'bold' },
                padding: { top: 2, bottom: 2, left: 5, right: 5 },
                yAdjust: 22 + labelOffset,
            },
        };
    });
    return ann;
})();

// Healthy IQR band — semi-transparent green region showing the range
// of P25-to-P75 spreads observed during the slip-free coarse phase.
// Helps users see at a glance when bisection / verify points exceed
// the normal envelope.
const healthyIqr = %(healthy_iqr)s;
const healthyBandAnnotations = (() => {
    if (!healthyIqr || sgMedian.length === 0) return {};
    // Find a safe Y-position centered on a typical median value.
    const validMeds = sgMedian.filter(v => v !== null);
    if (validMeds.length === 0) return {};
    // Render a faint horizontal "expected spread" band of width
    // healthyIqr.max around each median data point would be too
    // visually busy. Instead, draw a single info label in the corner.
    return {
        healthy_label: {
            type: 'label',
            xValue: 0, yValue: Math.max(...validMeds),
            position: { x: 'start', y: 'start' },
            xAdjust: 8, yAdjust: 8,
            content: ['Healthy ranges (coarse phase):',
                      '  CV typical: see decision-trail panel',
                      '  IQR typical: ' +
                      healthyIqr.min.toFixed(0) + ' – ' +
                      healthyIqr.max.toFixed(0) +
                      ' (median ' +
                      healthyIqr.median.toFixed(0) + ')'],
            backgroundColor: 'rgba(232, 245, 233, 0.85)',
            borderColor: '#43a047', borderWidth: 1,
            borderRadius: 4,
            color: '#1b5e20',
            font: { size: 10, family: 'monospace' },
            padding: { top: 5, bottom: 5, left: 8, right: 8 },
            textAlign: 'left',
        },
    };
})();

const allAnnotations = Object.assign({}, phaseAnnotations,
                                     eventAnnotations,
                                     healthyBandAnnotations);

const commonOptions = {
    responsive: true,
    interaction: { mode: 'index', intersect: false },
    scales: { x: { title: { display: true, text: 'Flow Rate (mm³/s)' } } },
    plugins: {
        legend: { position: 'top' },
        annotation: { annotations: allAnnotations },
    },
};

new Chart(document.getElementById('sgChart'), {
    type: 'line',
    data: { labels: flows, datasets: [
        { label: 'P75', data: sgP75,
          borderColor: 'rgba(150,200,255,0.5)',
          backgroundColor: 'rgba(150,200,255,0.15)', fill: '+1',
          borderDash: [3,3], pointRadius: 2 },
        { label: 'median', data: sgMedian, borderColor: '#1976d2',
          fill: false, borderWidth: 3, pointRadius: 5 },
        { label: 'P25', data: sgP25,
          borderColor: 'rgba(150,200,255,0.5)', fill: false,
          borderDash: [3,3], pointRadius: 2 },
        { label: 'avg', data: sgAvg, borderColor: '#90a4ae',
          borderDash: [6,3], fill: false, borderWidth: 1, pointRadius: 0 },
    ] },
    options: { ...commonOptions, scales: { ...commonOptions.scales,
        y: { title: { display: true,
             text: '%(sg_label)s (0-510)' } } } },
});
%(cs_chart_script)s
</script>
</body></html>"""
        # Build trigger-event annotations for the chart. We attach
        # each event to the LAST results-array index whose flow matches
        # (since re-tests replace earlier entries, this is well-defined
        # except for verify-fail where the failed verify still sits in
        # results — in that case we attach to the verify entry directly).
        chart_events = []
        if final_result is not None:
            for ev in (final_result.get('trigger_events') or []):
                # Find the matching results index (search in reverse so
                # we hit the last/most-recent entry for this flow+phase).
                idx = None
                for i in range(len(results) - 1, -1, -1):
                    rr = results[i]
                    if (rr.get('phase') == ev['phase']
                            and abs(rr.get('flow', 0) - ev['flow'])
                            < 0.01):
                        idx = i
                        break
                if idx is None:
                    continue
                chart_events.append({
                    'idx': idx,
                    'flow': ev['flow'],
                    'phase': ev['phase'],
                    'kind': ev['kind'],
                    'reason': ev['reason'],
                    'cv': ev.get('cv'),
                    'iqr': ev.get('iqr'),
                })

        # Healthy-range bands for the chart (drawn as horizontal
        # background regions if baseline_stats is available).
        healthy_iqr = None
        if final_result is not None:
            bs = final_result.get('baseline_stats')
            if bs and 'iqr_min' in bs and 'iqr_max' in bs:
                healthy_iqr = {
                    'min': bs['iqr_min'],
                    'max': max(bs['iqr_max'],
                               bs['iqr_median'] * 1.5),
                    'median': bs['iqr_median'],
                }

        rendered = html % {
            'timestamp': meta.get('timestamp', '-'),
            'version': MODULE_VERSION,
            'mode': mode,
            'mode_upper': mode.upper(),
            'meta_html': meta_html,
            'tmc_settings_html': tmc_settings_html,
            'decision_trail_html': decision_trail_html,
            'summary_html': summary_html,
            'sg_label': sg_label,
            'data_table': table,
            'cs_chart_html': cs_chart_html,
            'cs_chart_script': cs_chart_script,
            'flows': json.dumps(flows),
            'phases': json.dumps(phases),
            'sg_median': json.dumps(sg_median),
            'sg_p25': json.dumps(sg_p25),
            'sg_p75': json.dumps(sg_p75),
            'sg_avg': json.dumps(sg_avg),
            'chart_events': json.dumps(chart_events),
            'healthy_iqr': json.dumps(healthy_iqr),
        }
        with open(path, 'w') as f:
            f.write(rendered)

    def _save_report(self, results, meta, timestamp, limit_reason,
                     no_html, mode, gcmd=None, announce=True,
                     final_result=None):
        """Save CSV and HTML report. Safe to call repeatedly."""
        if not results:
            return
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            csv_path = os.path.join(
                self.output_dir, 'tmc_flow_%s_%s.csv' % (mode, timestamp))
            self._write_csv(csv_path, results, meta, mode)
            if announce and gcmd is not None:
                gcmd.respond_info("CSV saved: %s" % csv_path)
            if not no_html:
                html_path = os.path.join(
                    self.output_dir,
                    'tmc_flow_%s_%s.html' % (mode, timestamp))
                self._write_html(html_path, results, meta, limit_reason,
                                 mode, final_result=final_result)
                if announce and gcmd is not None:
                    gcmd.respond_info("HTML saved: %s" % html_path)
        except Exception as e:
            if gcmd is not None:
                gcmd.respond_info(
                    "Warning: report write failed: %s" % e)
            logging.exception("tmc_flow_test: report write failed")

    # ─── Single flow step measurement ───────────────────────────────

    def _measure_step(self, gcmd, target_flow, step_duration, repeat,
                      sample_cs=False, skip_warmup=True):
        """Run a single flow measurement (multiple repetitions, aggregate)."""
        mm_per_sec = target_flow / self.filament_area
        feed_rate = mm_per_sec * 60.0
        extrude_length = mm_per_sec * step_duration

        per_run_sg = []
        per_run_cs = []
        run_sg_avgs = []
        run_cs_avgs = []

        for rep in range(repeat):
            self._start_sampling(sample_cs=sample_cs)
            try:
                self.gcode.run_script_from_command(
                    "G1 E%.4f F%.1f\nM400" % (extrude_length, feed_rate))
            finally:
                self._stop_sampling()

            run_sg = list(self.samples_sg)
            per_run_sg.append(run_sg)
            if run_sg:
                run_sg_avgs.append(sum(run_sg) / len(run_sg))
            if sample_cs:
                run_cs = list(self.samples_cs)
                per_run_cs.append(run_cs)
                if run_cs:
                    run_cs_avgs.append(sum(run_cs) / len(run_cs))

            if rep < repeat - 1:
                self.gcode.run_script_from_command("G4 P300")

        # Warmup-skip: if first run deviates >10% from rest, exclude it
        warmup_dropped = False
        included_indices = list(range(len(run_sg_avgs)))
        if skip_warmup and len(run_sg_avgs) >= 3:
            run1 = run_sg_avgs[0]
            rest_mean = sum(run_sg_avgs[1:]) / len(run_sg_avgs[1:])
            if rest_mean > 0:
                deviation = abs(run1 - rest_mean) / rest_mean
                if deviation > 0.10:
                    warmup_dropped = True
                    included_indices = list(range(1, len(run_sg_avgs)))

        # Aggregate from included runs
        agg_sg = []
        agg_cs = []
        for idx in included_indices:
            agg_sg.extend(per_run_sg[idx])
            if sample_cs and idx < len(per_run_cs):
                agg_cs.extend(per_run_cs[idx])

        sg_stats = self._stats(agg_sg)
        cs_stats = self._stats(agg_cs) if sample_cs else None

        included_sg_avgs = [run_sg_avgs[i] for i in included_indices
                            if i < len(run_sg_avgs)]
        run_consistency = None
        if len(included_sg_avgs) > 1:
            sg_run_std = statistics.pstdev(included_sg_avgs)
            sg_run_mean = sum(included_sg_avgs) / len(included_sg_avgs)
            sg_cv = (sg_run_std / sg_run_mean * 100.0
                     if sg_run_mean > 0 else 0)
            run_consistency = {
                'sg_run_std': sg_run_std,
                'sg_cv': sg_cv,
                'warmup_dropped': warmup_dropped,
            }

        # Compact summary line
        sg_med_str = "%.0f" % sg_stats['median'] if sg_stats else 'n/a'
        cv_str = ("%.1f%%" % run_consistency['sg_cv']
                  if run_consistency else 'n/a')
        warmup_str = ' [run 1 excluded as warmup]' if warmup_dropped else ''
        if sample_cs and cs_stats is not None:
            cs_med_str = "%.0f" % cs_stats['median']
            gcmd.respond_info(
                "  %.1f mm³/s | SG median = %s | CS median = %s | "
                "run-to-run CV = %s%s"
                % (target_flow, sg_med_str, cs_med_str, cv_str, warmup_str))
        else:
            gcmd.respond_info(
                "  %.1f mm³/s | SG median = %s | "
                "run-to-run CV = %s%s"
                % (target_flow, sg_med_str, cv_str, warmup_str))

        return {
            'flow': target_flow,
            'sg': sg_stats,
            'cs': cs_stats,
            'run_consistency': run_consistency,
            'run_sg_avgs': run_sg_avgs,
            'run_cs_avgs': run_cs_avgs,
        }

    # ─── Trigger detection — SG-only mode ──────────────────────────

    def _sg_min_informative(self):
        """Lowest SG value still considered informative.

        For SG4 drivers (TMC2240/2209) low SG values mean 'almost no
        load' and are noisy; we ignore them. For SG2 drivers (TMC5160 et
        al.) low SG means HIGH load — exactly the regime we care about
        — so don't gate on it.
        """
        if self.sg2_driver:
            return -1  # never reject on this gate
        return SG_MIN_INFORMATIVE

    def _sg_jump_threshold(self):
        """Minimum raw |SG delta| considered a real (non-noise) jump.

        Uses the driver-specific profile value, which captures the
        relationship between SG scale and jump magnitudes for that
        chip family.
        """
        return self.profile.SG_JUMP_THRESHOLD

    def _check_triggers_sg(self, results):
        """SG-based slip detection.

        SG can either RISE or FALL with increasing motor load depending on
        driver chip, chopper mode, motor, and CoolStep state. Rather than
        hard-coding a direction per driver, we let the data decide:
        average sign of the recent step-to-step deltas tells us which way
        SG is trending. A slip then shows up as a sharp move in the
        OPPOSITE direction (motor decouples from load → SG snaps back to
        its 'no-load' value).

        Two triggers, both fire only when stepping UP:
          1. SG reload jump: SG moves against the established trend by
             more than the typical step magnitude (and >15 raw units)
          2. Plateau over 2 steps: the trend stalls — cumulative
             trend-direction movement is less than half the typical
        """
        if not results or len(results) < 5:
            return None
        r = results[-1]
        sg_stats = r['sg']
        if sg_stats is None or sg_stats['n'] == 0:
            return None
        sg_med = sg_stats['median']
        target_flow = r['flow']
        sg_label = self._get_sg_label()

        if sg_med <= self._sg_min_informative():
            return None

        prev_flow = results[-2].get('flow', 0)
        going_up = target_flow >= prev_flow - 0.001
        # SG trend triggers (snap-back, abnormal jump) are direction-
        # sensitive — they require a monotonic up-sweep to compare
        # against. Bisection probes can step DOWN, so we skip the
        # trend triggers in that case but still run the
        # direction-independent CV-spike and IQR-spread checks.

        # ─── Trend-direction triggers (snap-back / over-jump / plateau)
        # ─── only valid during monotonic up-sweep (coarse phase). In
        # ─── bisection, the trend baseline is meaningless because we
        # ─── jump around the slip point.
        if going_up:
            # Raw step-to-step SG deltas from recent history.
            sg_deltas = []
            for j in range(max(1, len(results) - 5), len(results) - 1):
                rj, rj_prev = results[j], results[j-1]
                if rj.get('sg') and rj_prev.get('sg'):
                    sg_deltas.append(
                        rj['sg']['median'] - rj_prev['sg']['median'])

            if len(sg_deltas) >= 3:
                expected_delta = sum(sg_deltas) / len(sg_deltas)
                actual_delta = sg_med - results[-2]['sg']['median']

                # Trend direction (+1 = SG rises with load, -1 = SG falls
                # with load, 0 = flat / inconclusive). Use 1 raw unit as
                # the noise-floor so a tiny non-zero average doesn't lock
                # us into a spurious sign.
                if expected_delta > 1.0:
                    trend_sign = +1
                elif expected_delta < -1.0:
                    trend_sign = -1
                else:
                    trend_sign = 0

                # In trend-direction terms, slip can manifest two ways:
                #   • "Over-jump"   — load_signal jumps in the same
                #                      direction as the trend, but >2×
                #                      the typical step. This is what
                #                      TMC2240/2209 (SG4) often show
                #                      empirically: SG keeps rising but
                #                      suddenly MUCH faster than before.
                #   • "Snap-back"   — load_signal moves OPPOSITE to the
                #                      trend (motor decouples, SG returns
                #                      toward its no-load value). This is
                #                      the textbook SG2 stall signature
                #                      seen on TMC5160 etc.
                # Both should fire; we test each independently.
                if trend_sign != 0:
                    expected_load = expected_delta * trend_sign     # > 0
                    actual_load = actual_delta * trend_sign         # any sign

                    # Snap-back (slip via decoupling)
                    if (actual_load < -expected_load
                            and abs(actual_delta)
                            > self._sg_jump_threshold()):
                        if trend_sign > 0:
                            return ("%s reload jump: %+.0f (expected to "
                                    "keep rising ~+%.0f) — slip detected"
                                    % (sg_label, actual_delta,
                                       expected_load))
                        return ("%s reload jump: %+.0f (expected to keep "
                                "falling ~-%.0f) — slip detected"
                                % (sg_label, actual_delta, expected_load))

                    # Over-jump (slip via abnormal acceleration)
                    if (actual_load > expected_load * 2.0
                            and abs(actual_delta)
                            > self._sg_jump_threshold()):
                        if trend_sign > 0:
                            return ("%s abnormal jump: %+.0f vs expected "
                                    "~+%.0f (%.1fx larger) — slip detected"
                                    % (sg_label, actual_delta,
                                       expected_load,
                                       actual_load / expected_load))
                        return ("%s abnormal drop: %+.0f vs expected "
                                "~-%.0f (%.1fx larger) — slip detected"
                                % (sg_label, actual_delta, expected_load,
                                   actual_load / expected_load))

                    # Trigger 2: plateau over 2 steps, only meaningful
                    # when the trend is sizeable. Threshold at 0.5 —
                    # anything tighter gives false positives on
                    # naturally asymptotic SG curves where each step
                    # falls by a smaller increment than the last.
                    if expected_load > 5:
                        prev_actual = (results[-2]['sg']['median']
                                       - results[-3]['sg']['median'])
                        cumulative_load = (actual_load
                                           + prev_actual * trend_sign)
                        expected_2step = expected_load * 2
                        if cumulative_load < expected_2step * 0.5:
                            direction = ("rising" if trend_sign > 0
                                         else "falling")
                            return ("%s plateau over 2 steps: %s trend "
                                    "stalled (only %.0f vs typical %.0f "
                                    "load-units) — flow no longer "
                                    "increasing motor load (slip starting)"
                                    % (sg_label, direction,
                                       cumulative_load, expected_2step))

        # ─── Direction-independent triggers (always evaluated) ───
        # CV-spike and IQR-spread fire on the per-step variance signal
        # itself, not on a trend baseline, so they make sense for both
        # coarse and bisection probes.

        # Trigger 3: run-to-run CV spike. Intermittent slip shows up as
        # a sudden burst of variance between repeats at the same flow,
        # even when the median doesn't move much. Fire when the latest
        # CV is both elevated in absolute terms AND much higher than the
        # baseline of recent steps.
        cv_reason = self._check_cv_spike(results, sg_label)
        if cv_reason:
            return cv_reason

        # Trigger 4: IQR/spread anomaly. Catches "quiet" stalls where
        # the median looks normal but the distribution widens (brief
        # intermittent stall absorbed by median over 5 repeats).
        iqr_reason = self._check_iqr_spread(results, sg_label)
        if iqr_reason:
            return iqr_reason

        # Trigger 5: SG max spike — bisection / verify only. Catches
        # decoupling events where one of the repeats briefly snaps SG
        # to its no-load value, but the median absorbs it.
        spike_reason = self._check_sg_max_spike(results, sg_label)
        if spike_reason:
            return spike_reason

        # Trigger 6: per-run outlier — bisection / verify only. Catches
        # the case where 1 of N repeats has a clearly different SG/CS
        # profile from the others (intermittent slip in just one run).
        outlier_reason = self._check_run_outlier(results, sg_label)
        if outlier_reason:
            return outlier_reason

        return None

    def _check_cv_spike(self, results, sg_label):
        """Detect a run-to-run CV spike — intermittent slip signature.

        A clean motor produces tight repeats (CV usually <3% on stable
        SG2 setups, may go up to 5% on noisier SG4). When the motor
        starts intermittently slipping, individual repeats diverge
        sharply, even if the median across all samples looks 'normal'.

        Triggers (any one fires):
          (a) "high-variance trip" — last CV >= 10% absolute (regardless
              of baseline). Catches sudden chaotic slip.
          (b) "low-baseline jump"  — last CV >= ratio * avg of prior
              N steps AND last CV >= absolute floor. Ratio and floor
              are tighter in coarse, looser in bisection (where we're
              already near slip).
          (c) "rising CV trend"   — CV grew across 2 consecutive
              steps (each >=1.3x prior).
          (d) "bisection absolute" — in bisection / verify, ANY CV
              >= 2x the *coarse-phase median CV* (or >= 5% absolute,
              whichever is higher). Catches the case where
              intermittent slip already started during coarse and
              "polluted" the immediate baseline, making patterns (b)
              and (c) under-react.

        Baseline strategy: when we're in bisection/verify, the most
        meaningful baseline is the *coarse phase* — it represents the
        clean, slip-free motor behaviour. Using only the immediate 3
        prior steps as baseline lets earlier elevated bisection CVs
        raise the threshold and mask later anomalies.
        """
        last = results[-1]
        last_rc = last.get('run_consistency') or {}
        if 'sg_cv' not in last_rc:
            return None
        last_cv = last_rc['sg_cv']

        last_phase = last.get('phase', 'coarse')
        in_bisection = last_phase in ('bisect', 'verify')

        # Build two baselines:
        #   - "immediate": last 3 steps regardless of phase (sensitive
        #     to recent trend)
        #   - "coarse": median of CVs from the COARSE phase only
        #     (slip-free reference, only available once we have data)
        prior_cvs = []
        for r in results[-4:-1]:
            rc = r.get('run_consistency') or {}
            if 'sg_cv' in rc:
                prior_cvs.append(rc['sg_cv'])

        coarse_cvs = []
        for r in results[:-1]:
            if r.get('phase', 'coarse') != 'coarse':
                continue
            rc = r.get('run_consistency') or {}
            if 'sg_cv' in rc:
                coarse_cvs.append(rc['sg_cv'])
        # Median of coarse CVs, excluding the LAST coarse step (which
        # may be the elevated step that triggered bisection).
        coarse_baseline_cv = None
        if len(coarse_cvs) >= 4:
            sorted_cv = sorted(coarse_cvs[:-1])
            n = len(sorted_cv)
            coarse_baseline_cv = (sorted_cv[n // 2] if n % 2
                                  else (sorted_cv[n // 2 - 1]
                                        + sorted_cv[n // 2]) / 2)

        # Pattern (a): high-variance trip — works without baseline
        if last_cv >= self.profile.CV_HIGH_VARIANCE:
            if not prior_cvs:
                return ("run-to-run %s CV spiked to %.1f%% — repeats "
                        "diverging, intermittent slip"
                        % (sg_label, last_cv))
            avg_prior_cv = sum(prior_cvs) / len(prior_cvs)
            if last_cv >= self.profile.CV_LOWBASE_RATIO * avg_prior_cv:
                return ("run-to-run %s CV spiked to %.1f%% (baseline "
                        "~%.1f%% over previous 3 steps) — repeats "
                        "diverging, intermittent slip"
                        % (sg_label, last_cv, avg_prior_cv))

        if len(prior_cvs) < 3:
            return None
        avg_prior_cv = sum(prior_cvs) / len(prior_cvs)

        # Pattern (b): low-baseline jump vs immediate prior
        b_ratio = (self.profile.CV_JUMP_RATIO_BISECT if in_bisection
                   else self.profile.CV_JUMP_RATIO_COARSE)
        b_min_cv = (self.profile.CV_JUMP_MIN_BISECT if in_bisection
                    else self.profile.CV_JUMP_MIN_COARSE)
        if last_cv >= b_min_cv and last_cv >= b_ratio * avg_prior_cv:
            return ("run-to-run %s CV jumped to %.1f%% (%.1fx baseline "
                    "of %.1f%% over previous 3 steps) — repeats "
                    "diverging, intermittent slip"
                    % (sg_label, last_cv,
                       last_cv / avg_prior_cv, avg_prior_cv))

        # Pattern (c): rising CV trend across 2 consecutive steps.
        c_min_cv = (self.profile.CV_RISING_MIN_LAST_BISECT if in_bisection
                    else self.profile.CV_RISING_MIN_LAST_COARSE)
        c_ratio = self.profile.CV_RISING_RATIO
        if (last_cv >= c_min_cv
                and len(prior_cvs) >= 2
                and prior_cvs[-1] >= c_ratio * prior_cvs[-2]
                and last_cv >= c_ratio * prior_cvs[-1]
                and prior_cvs[-1] >= self.profile.CV_RISING_MIN_PRIOR):
            return ("run-to-run %s CV rising across 3 steps "
                    "(%.1f%% → %.1f%% → %.1f%%) — gradual slip onset"
                    % (sg_label, prior_cvs[-2], prior_cvs[-1], last_cv))

        # Pattern (d): bisection absolute vs coarse baseline.
        # In bisection/verify, compare against the slip-free coarse
        # phase median. This catches cases where one elevated bisect
        # step has already pulled the immediate baseline up, masking
        # subsequent elevated values.
        if (in_bisection
                and coarse_baseline_cv is not None
                and last_cv >= self.profile.CV_VS_COARSE_MIN
                and last_cv >=
                    self.profile.CV_VS_COARSE_RATIO * coarse_baseline_cv):
            return ("run-to-run %s CV %.1f%% in bisection vs coarse-"
                    "phase baseline %.1f%% (%.1fx) — slip signature "
                    "above clean-extrusion noise floor"
                    % (sg_label, last_cv, coarse_baseline_cv,
                       last_cv / coarse_baseline_cv))

        return None

    def _check_iqr_spread(self, results, sg_label):
        """Detect an IQR/spread anomaly — quiet stall signature.

        Sometimes a motor stalls briefly (start or end of move) and
        recovers, leaving a median that looks normal but a wider
        distribution. The IQR (P75-P25) widens when this happens.

        Three triggers, in order:
          (a) "ratio vs immediate" — current IQR >= ratio * avg of
              prior 3 IQRs. Ratio is 3.0 in coarse, 1.7 in bisection.
          (b) "ratio vs coarse baseline" (bisection only) — current
              IQR >= 2.0 * the median IQR from the COARSE phase.
              Catches cases where earlier elevated bisect IQRs have
              raised the immediate baseline and masked subsequent
              anomalies.
          (c) "absolute" — current IQR >= 25 raw units in bisection /
              verify. A motor near slip should have median, P25 and
              P75 within a few units of each other; an IQR of 25+
              indicates intermittent stall regardless of context.
        """
        last = results[-1]
        sg_stats = last.get('sg') or {}
        if 'p25' not in sg_stats or 'p75' not in sg_stats:
            return None
        last_iqr = sg_stats['p75'] - sg_stats['p25']

        prior_iqrs = []
        for r in results[-4:-1]:
            sg_p = r.get('sg') or {}
            if 'p25' in sg_p and 'p75' in sg_p:
                prior_iqrs.append(sg_p['p75'] - sg_p['p25'])

        last_phase = last.get('phase', 'coarse')
        in_bisection = last_phase in ('bisect', 'verify')

        # (c) absolute — fires unconditionally on any genuinely large
        # IQR during bisection / verify. Cheap and reliable.
        if in_bisection and last_iqr >= self.profile.IQR_ABSOLUTE_TRIGGER:
            return ("%s spread widened: IQR %.0f raw units in %s — "
                    "samples no longer cluster, intermittent stall"
                    % (sg_label, last_iqr, last_phase))

        if len(prior_iqrs) < 3:
            return None
        avg_prior_iqr = sum(prior_iqrs) / len(prior_iqrs)
        if avg_prior_iqr < 1:
            return None

        # (a) ratio vs immediate prior steps.
        # If the profile requires CV cross-check (e.g. SG4 drivers with
        # active CoolStep), confirm with the run-to-run CV — without
        # elevated CV, a wide IQR is likely a CoolStep-current-transition
        # effect, not slip. Genuine slip raises both within-step spread
        # AND run-to-run variance.
        ratio_threshold = (self.profile.IQR_RATIO_BISECT if in_bisection
                           else self.profile.IQR_RATIO_COARSE)
        if (last_iqr >= self.profile.IQR_RATIO_MIN_ABS
                and last_iqr >= ratio_threshold * avg_prior_iqr):
            cv_confirms = True
            cv_note = ""
            if self.profile.IQR_RATIO_REQUIRE_CV:
                last_rc = last.get('run_consistency') or {}
                last_cv_a = last_rc.get('sg_cv')
                cv_floor = (self.profile.IQR_RATIO_CV_FLOOR_BISECT
                            if in_bisection
                            else self.profile.IQR_RATIO_CV_FLOOR_COARSE)
                # Build prior-CV baseline same way as prior-IQR baseline
                prior_cvs_a = []
                for r in results[-4:-1]:
                    rc = r.get('run_consistency') or {}
                    if 'sg_cv' in rc:
                        prior_cvs_a.append(rc['sg_cv'])
                avg_prior_cv = (sum(prior_cvs_a) / len(prior_cvs_a)
                                if prior_cvs_a else 0.0)
                # CV must be elevated above absolute floor AND above
                # baseline (or baseline must be tiny). 1.4× ratio is
                # less strict than the value-based jump triggers in
                # _check_cv_spike — IQR pattern (a) only needs CV to
                # *not contradict* the spread anomaly.
                cv_confirms = (last_cv_a is not None
                               and last_cv_a >= cv_floor
                               and (avg_prior_cv < 0.5
                                    or last_cv_a >= 1.4 * avg_prior_cv))
                if cv_confirms and last_cv_a is not None:
                    cv_note = (" (CV %.1f%% confirms, prior ~%.1f%%)"
                               % (last_cv_a, avg_prior_cv))
            if cv_confirms:
                return ("%s spread widened: IQR %.0f vs baseline ~%.0f "
                        "(%.1fx) — quiet/intermittent stall in samples"
                        "%s"
                        % (sg_label, last_iqr, avg_prior_iqr,
                           last_iqr / avg_prior_iqr, cv_note))

        # (b) ratio vs coarse baseline (bisection only).
        # Cross-check with CV: a pure spread anomaly with NORMAL CV is
        # often just statistical fluctuation (e.g. a single outlying
        # sample stretched the IQR but the run-to-run averages are
        # tight). Genuine slip widens BOTH spread AND CV. So we only
        # fire when CV also confirms elevated noise relative to coarse.
        if in_bisection:
            coarse_iqrs = []
            coarse_cvs = []
            for r in results[:-1]:
                if r.get('phase', 'coarse') != 'coarse':
                    continue
                sg_p = r.get('sg') or {}
                if 'p25' in sg_p and 'p75' in sg_p:
                    coarse_iqrs.append(sg_p['p75'] - sg_p['p25'])
                rc = r.get('run_consistency') or {}
                if 'sg_cv' in rc:
                    coarse_cvs.append(rc['sg_cv'])
            if len(coarse_iqrs) >= 4 and len(coarse_cvs) >= 4:
                # Median of all but the last coarse step (which may be
                # the elevated step that triggered bisection).
                sorted_iqr = sorted(coarse_iqrs[:-1])
                n = len(sorted_iqr)
                median_coarse_iqr = (sorted_iqr[n // 2] if n % 2
                                     else (sorted_iqr[n // 2 - 1]
                                           + sorted_iqr[n // 2]) / 2)
                sorted_cv = sorted(coarse_cvs[:-1])
                m = len(sorted_cv)
                median_coarse_cv = (sorted_cv[m // 2] if m % 2
                                    else (sorted_cv[m // 2 - 1]
                                          + sorted_cv[m // 2]) / 2)
                last_rc = last.get('run_consistency') or {}
                last_cv = last_rc.get('sg_cv', 0.0)
                # CV cross-check: must be elevated vs coarse AND above
                # an absolute floor. If CV is below baseline-ish, it's
                # not slip — it's a wide-distribution fluke.
                if self.profile.IQR_VS_COARSE_REQUIRE_CV:
                    cv_confirms = (
                        last_cv >= self.profile.IQR_VS_COARSE_CV_FLOOR
                        and median_coarse_cv >= 0.5
                        and last_cv >=
                            (self.profile.IQR_VS_COARSE_CV_RATIO
                             * median_coarse_cv))
                else:
                    cv_confirms = True
                if (median_coarse_iqr >= 1
                        and last_iqr >= self.profile.IQR_VS_COARSE_MIN_ABS
                        and last_iqr >=
                            (self.profile.IQR_VS_COARSE_RATIO
                             * median_coarse_iqr)
                        and cv_confirms):
                    return ("%s spread IQR %.0f in %s vs coarse-phase "
                            "median IQR ~%.0f (%.1fx); CV %.1f%% also "
                            "elevated vs coarse %.1f%% — slip widening"
                            % (sg_label, last_iqr, last_phase,
                               median_coarse_iqr,
                               last_iqr / median_coarse_iqr,
                               last_cv, median_coarse_cv))

        return None

    def _check_sg_max_spike(self, results, sg_label):
        """Detect a no-load decoupling spike inside a single step.

        When the motor briefly decouples from the load during one of
        the repeats, SG snaps toward its no-load value. The 5-second
        per-run average + median over 5 runs absorbs the brief spike,
        but sg_max / sg_min records it.

        Direction is data-driven, not driver-hardcoded: across all
        three driver families (TMC5160 SG2, TMC2240 SG4, TMC2209 SG4)
        SG normally FALLS as load increases. So decoupling = SG snaps
        UP toward the no-load value, which means sg_max is the right
        signal regardless of driver. (We still keep a sg_min check as
        fallback in case a particular setup runs the other direction.)

        Only fires in bisection / verify, since coarse has plenty of
        other triggers and natural variation is wider there.
        """
        last = results[-1]
        if last.get('phase') not in ('bisect', 'verify'):
            return None
        sg = last.get('sg') or {}
        if 'max' not in sg or 'min' not in sg or 'median' not in sg:
            return None
        last_max = sg['max']
        last_min = sg['min']
        last_med = sg['median']

        # Build baseline of "typical" sg_max and sg_min from the
        # coarse phase. We need both because the no-load direction
        # depends on the actual SG-vs-load slope of THIS hardware.
        coarse_maxes = []
        coarse_mins = []
        coarse_meds = []
        for r in results[:-1]:
            if r.get('phase', 'coarse') != 'coarse':
                continue
            sg_p = r.get('sg') or {}
            if 'max' in sg_p:
                coarse_maxes.append(sg_p['max'])
            if 'min' in sg_p:
                coarse_mins.append(sg_p['min'])
            if 'median' in sg_p:
                coarse_meds.append(sg_p['median'])
        if len(coarse_maxes) < 4 or len(coarse_meds) < 4:
            return None

        # Determine SG-vs-load trend direction: compare first vs last
        # coarse-phase median (excluding the last coarse step which
        # likely triggered). If SG dropped from start to end, normal
        # = falling-with-load → decoupling = max spikes UP. Otherwise
        # = rising-with-load → decoupling = min drops DOWN.
        first_med = coarse_meds[0]
        # Use median of last 2-3 (excluding very last, which triggered)
        if len(coarse_meds) >= 3:
            late_meds = coarse_meds[-3:-1] if len(coarse_meds) > 3 else coarse_meds[-2:-1]
            late_med = sum(late_meds) / len(late_meds)
        else:
            late_med = coarse_meds[-1]
        sg_falls_with_load = late_med < first_med

        def _median(xs):
            s = sorted(xs); n = len(s)
            return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2

        if sg_falls_with_load:
            # Normal case for all common driver setups. Decoupling →
            # SG snaps UP. Compare sg_max to typical coarse sg_max.
            median_coarse_max = _median(coarse_maxes[:-1])
            # Two paths to fire:
            # (a) Strong ratio + above coarse: max is dramatically
            #     above median AND clearly above what coarse saw.
            # (b) Big absolute jump: max is far above median AND the
            #     gap (max - median) is large. This covers narrow-range
            #     drivers (e.g. TMC2209) where coarse_max can already
            #     sit near the no-load value.
            absolute_floor = (self.profile.SG_MAX_ABS_GAP
                              if self.sg2_driver
                              else self.profile.SG_MAX_ABS_GAP_SG4)
            ratio_vs_coarse = (last_max
                               / max(median_coarse_max, 1))
            ratio_vs_med = last_max / max(last_med, 1)
            big_gap = (last_max - last_med) >= self.profile.SG_MAX_BIG_GAP
            fires_a = (ratio_vs_med >= self.profile.SG_MAX_RATIO_TO_MEDIAN
                       and ratio_vs_coarse >=
                            self.profile.SG_MAX_RATIO_TO_COARSE
                       and last_max - last_med >= absolute_floor)
            fires_b = (ratio_vs_med >= self.profile.SG_MAX_BIG_RATIO
                       and big_gap)
            if fires_a or fires_b:
                return ("%s max %.0f in this step is %.1fx the "
                        "median (%.0f) and %.1fx the typical coarse "
                        "max (%.0f) — at least one repeat showed "
                        "motor decoupling spike (no-load value)"
                        % (sg_label, last_max, ratio_vs_med, last_med,
                           ratio_vs_coarse, median_coarse_max))
        else:
            # Rare case: SG rises with load. Decoupling → SG snaps
            # DOWN. Compare sg_min to typical coarse sg_min.
            if not coarse_mins:
                return None
            median_coarse_min = _median(coarse_mins[:-1])
            if (last_min > 0
                    and last_min <=
                        last_med / self.profile.SG_MIN_RATIO_TO_MEDIAN
                    and last_med - last_min >=
                        self.profile.SG_MIN_ABS_GAP):
                return ("%s min %.0f is %.1fx below the median (%.0f)"
                        " — at least one repeat dropped to no-load "
                        "value (decoupling)"
                        % (sg_label, last_min,
                           last_med / max(last_min, 1), last_med))

        return None

    def _check_run_outlier(self, results, sg_label):
        """Detect when one of the per-run averages is an outlier.

        Each measurement step runs N repeats (default 5). We have the
        list of per-run SG averages and per-run CS averages. If the
        run-to-run variance is dominated by a SINGLE outlying run
        (one repeat much off, the others tight), the median absorbs it
        but it's clear evidence of intermittent slip.

        Detection:
          - At least 4 valid runs
          - Find the run whose SG_avg deviates most from the median of
            the other runs
          - If that deviation is > 2x the median absolute deviation
            (MAD) of the rest, it's an outlier
          - Plus: corresponding CS_avg should also deviate (load really
            changed). CS-variation > 1.5 between min and max OR min CS
            < 28 while median is >= 30 confirms.

        Only fires in verify (where we care about reproducibility) or
        in bisection at small bracket widths.
        """
        last = results[-1]
        if last.get('phase') not in ('bisect', 'verify'):
            return None
        run_sg = last.get('run_sg_avgs') or []
        run_cs = last.get('run_cs_avgs') or []
        if len(run_sg) < 4:
            return None

        # Find the run furthest from the median of the rest
        n = len(run_sg)
        outlier_idx = -1
        outlier_dev = 0.0
        median_rest = 0.0
        mad_rest = 0.0
        for i in range(n):
            others = [run_sg[j] for j in range(n) if j != i]
            sorted_o = sorted(others)
            m = len(sorted_o)
            med_o = (sorted_o[m // 2] if m % 2
                     else (sorted_o[m // 2 - 1] + sorted_o[m // 2]) / 2)
            devs = [abs(x - med_o) for x in others]
            sorted_d = sorted(devs)
            mad_o = (sorted_d[m // 2] if m % 2
                     else (sorted_d[m // 2 - 1] + sorted_d[m // 2]) / 2)
            dev_i = abs(run_sg[i] - med_o)
            if dev_i > outlier_dev:
                outlier_dev = dev_i
                outlier_idx = i
                median_rest = med_o
                mad_rest = mad_o

        if outlier_idx < 0 or median_rest <= 0:
            return None

        # Outlier criterion: deviation >= 4x MAD AND >= 8% of median
        # (avoids tripping on tiny absolute differences when MAD is
        # near zero).
        if mad_rest < 0.5:
            mad_rest = 0.5  # noise floor
        if not (outlier_dev >= self.profile.OUTLIER_MAD_RATIO * mad_rest
                and outlier_dev >=
                    self.profile.OUTLIER_MIN_REL * median_rest):
            return None

        # Confirm with CS variation (CoolStep should have noticed).
        cs_confirms = False
        cs_note = ""
        if len(run_cs) == n:
            cs_min = min(run_cs)
            cs_max = max(run_cs)
            cs_med = sorted(run_cs)[n // 2]
            # CS dropped on the outlier run? OR overall CS spread large?
            if (cs_min <= cs_med - self.profile.OUTLIER_CS_DROP
                    or (cs_max - cs_min) >= self.profile.OUTLIER_CS_RANGE):
                cs_confirms = True
                cs_note = (" (run %d CS=%.1f vs median %.1f)"
                           % (outlier_idx + 1,
                              run_cs[outlier_idx], cs_med))

        # In verify, also accept SG-only outliers (because verify is
        # the last line of defence and false-positives there cost
        # only one verify retry, not a wrong final result).
        last_phase = last.get('phase')
        if not cs_confirms and last_phase != 'verify':
            return None

        return ("run %d is an outlier (SG_avg %.1f vs %.1f for the "
                "other repeats, %.1fx MAD)%s — at least one of the "
                "repeats stalled while the others ran clean"
                % (outlier_idx + 1, run_sg[outlier_idx],
                   median_rest, outlier_dev / mad_rest, cs_note))

    def _is_borderline(self, results):
        """Detect if the latest bisection step sits in a 'gray zone'
        — not clearly safe but not clearly stalled either.

        Returns a string explaining why it's borderline, or None if
        the result is conclusive.

        Used by the bisection loop to decide whether to re-test the
        same flow before classifying it as safe or as a trigger.

        Heuristic: look at CV and IQR vs the coarse-phase baselines.
        Confidence ranges:
          - CV in coarse: typically 1-3% on a stable SG2 setup
          - CV >= 7% in bisection AND >= 3x coarse baseline: clearly slip
          - CV < 4% in bisection: clearly safe
          - CV in 4-6.9% range, OR 2-3x coarse baseline: BORDERLINE
          - IQR >= 25 in bisection: clearly slip (already trigger (c))
          - IQR < 15: clearly safe
          - IQR 15-24: BORDERLINE
        """
        if not results:
            return None
        last = results[-1]
        if last.get('phase') not in ('bisect', 'verify'):
            return None
        last_rc = last.get('run_consistency') or {}
        last_cv = last_rc.get('sg_cv')
        sg_stats = last.get('sg') or {}
        if 'p25' not in sg_stats or 'p75' not in sg_stats:
            return None
        last_iqr = sg_stats['p75'] - sg_stats['p25']

        # Build coarse-phase reference
        coarse_cvs = []
        coarse_iqrs = []
        for r in results[:-1]:
            if r.get('phase', 'coarse') != 'coarse':
                continue
            rc = r.get('run_consistency') or {}
            sg_p = r.get('sg') or {}
            if 'sg_cv' in rc:
                coarse_cvs.append(rc['sg_cv'])
            if 'p25' in sg_p and 'p75' in sg_p:
                coarse_iqrs.append(sg_p['p75'] - sg_p['p25'])
        if len(coarse_cvs) < 4:
            return None  # not enough baseline for comparison

        # Use median-of-coarse (excluding last coarse step which may be
        # the elevated one that triggered bisection)
        sorted_cv = sorted(coarse_cvs[:-1])
        n = len(sorted_cv)
        coarse_med_cv = (sorted_cv[n // 2] if n % 2
                         else (sorted_cv[n // 2 - 1]
                               + sorted_cv[n // 2]) / 2)
        sorted_iqr = sorted(coarse_iqrs[:-1])
        n = len(sorted_iqr)
        coarse_med_iqr = (sorted_iqr[n // 2] if n % 2
                          else (sorted_iqr[n // 2 - 1]
                                + sorted_iqr[n // 2]) / 2)

        # CV gray zone: elevated but not screaming
        cv_borderline = (last_cv is not None
                         and self.profile.BORDER_CV_LOW <= last_cv
                                                       < self.profile.BORDER_CV_HIGH
                         and (coarse_med_cv < 1
                              or last_cv >=
                                  self.profile.BORDER_CV_RATIO
                                  * coarse_med_cv))

        # IQR gray zone: wide but not extreme. Cross-check with CV — a
        # widened IQR with a very LOW CV is usually just one outlier
        # sample stretching the percentiles, not real intermittent
        # slip. Genuine slip widens both spread and run-to-run noise.
        # We require CV at least BORDER_IQR_CV_FLOOR (more than typical
        # clean noise) for IQR-only borderlines to count.
        iqr_borderline = (self.profile.BORDER_IQR_LOW <= last_iqr
                                                      < self.profile.BORDER_IQR_HIGH
                          and (coarse_med_iqr < 1
                               or last_iqr >=
                                   self.profile.BORDER_IQR_RATIO
                                   * coarse_med_iqr)
                          and last_cv is not None
                          and last_cv >= self.profile.BORDER_IQR_CV_FLOOR)

        if cv_borderline and iqr_borderline:
            return ("CV %.1f%% (vs coarse ~%.1f%%) AND IQR %.0f "
                    "(vs coarse ~%.0f) both in borderline range"
                    % (last_cv, coarse_med_cv,
                       last_iqr, coarse_med_iqr))
        if cv_borderline:
            return ("CV %.1f%% in borderline range (coarse baseline "
                    "~%.1f%%, clear trigger >=7%%)"
                    % (last_cv, coarse_med_cv))
        if iqr_borderline:
            return ("IQR %.0f in borderline range (coarse baseline "
                    "~%.0f, clear trigger >=25)"
                    % (last_iqr, coarse_med_iqr))
        return None

    def _coarse_baseline_stats(self, results):
        """Compute "healthy" CV and IQR ranges from the coarse phase.

        Used by the HTML report to show the user what was considered
        normal during the slip-free portion of the test, so trigger
        events can be interpreted in context.

        Returns dict with keys:
            cv_min, cv_median, cv_max, cv_mean
            iqr_min, iqr_median, iqr_max, iqr_mean
            n_steps
        Returns None if there's not enough coarse data.
        """
        coarse_cvs = []
        coarse_iqrs = []
        for r in results:
            if r.get('phase', 'coarse') != 'coarse':
                continue
            rc = r.get('run_consistency') or {}
            sg_p = r.get('sg') or {}
            if 'sg_cv' in rc:
                coarse_cvs.append(rc['sg_cv'])
            if 'p25' in sg_p and 'p75' in sg_p:
                coarse_iqrs.append(sg_p['p75'] - sg_p['p25'])
        # Drop the LAST coarse step (likely the one that triggered).
        # Only do so if we have enough data; otherwise keep them all.
        if len(coarse_cvs) >= 4:
            coarse_cvs = coarse_cvs[:-1]
        if len(coarse_iqrs) >= 4:
            coarse_iqrs = coarse_iqrs[:-1]

        if not coarse_cvs or not coarse_iqrs:
            return None

        def _median(xs):
            s = sorted(xs)
            n = len(s)
            return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2

        return {
            'cv_min': min(coarse_cvs),
            'cv_max': max(coarse_cvs),
            'cv_median': _median(coarse_cvs),
            'cv_mean': sum(coarse_cvs) / len(coarse_cvs),
            'iqr_min': min(coarse_iqrs),
            'iqr_max': max(coarse_iqrs),
            'iqr_median': _median(coarse_iqrs),
            'iqr_mean': sum(coarse_iqrs) / len(coarse_iqrs),
            'n_steps': len(coarse_cvs),
        }

    def _check_triggers_cs(self, results, baseline_cs):
        """CoolStep + SG triggers — robust for ACTIVE CoolStep regulation.

        AUTO-DETECTS whether CoolStep is actually changing CS_ACTUAL.
        If CS is essentially static (range < 1.0), falls back to SG-only.

        --- Background ---
        When CoolStep is actively regulating across the test flow range,
        CS rises gradually with load (e.g. 8 → 15 → 22 → 28 → 31). This
        is NORMAL behaviour, not slip — but the old "+5 step jump"
        heuristic interpreted every regulation step as slip.

        New trigger philosophy: a CS-jump alone is NOT a slip indicator
        when CoolStep is active. A real slip event is characterized by:
          (1) CS reaches FULL maximum (≥ 30) — not just "up a bit"
          (2) Previously CS was actually regulating below max (proves
              CoolStep wasn't already pegged from the start)
          (3) Confirmation from a second signal:
              - SG drops sharply (>=30% vs prior step), OR
              - run-to-run CV spikes, OR
              - SG snap-back / over-jump signature

        CS-based triggers (when CoolStep is active):
          A1. CS pegged at max + SG sharp drop — confirmed slip
          A2. CS pegged at max + CV spike — intermittent slip
          A3. CS hard drop after regulation — motor lost load contact
        SG triggers (always evaluated):
          B1. Snap-back / over-jump (data-driven trend detection)
          B2. CV spike (from _check_cv_spike helper)
        """
        if not results or len(results) < 3:
            return None
        r = results[-1]
        sg_stats = r['sg']
        cs_stats = r['cs']
        target_flow = r['flow']

        if cs_stats is None or cs_stats['n'] == 0:
            return None

        prev_cs = results[-2].get('cs')
        prev2_cs = results[-3].get('cs')
        if not prev_cs or not prev2_cs:
            return None

        # Detect mode: is CoolStep actually varying?
        cs_meds = [r_['cs']['median'] for r_ in results
                   if r_.get('cs') and r_['cs'].get('median') is not None]
        cs_range = max(cs_meds) - min(cs_meds) if len(cs_meds) >= 2 else 0
        coolstep_active = cs_range > 1.0

        prev_flow = results[-2].get('flow', 0)
        going_up_or_same = target_flow >= prev_flow - 0.001
        # Bisection / verify probe flows in arbitrary order, so the
        # going_up_or_same gate (which exists to suppress false-
        # positives during the monotonically-increasing coarse sweep)
        # must NOT block triggers in those phases. A stall is a stall
        # regardless of whether the previous probe was higher or lower.
        last_phase = results[-1].get('phase', 'coarse')
        in_bisection = last_phase in ('bisect', 'verify')
        evaluate_triggers = going_up_or_same or in_bisection

        cs_med = cs_stats['median']
        sg_med = sg_stats['median'] if sg_stats and sg_stats['n'] > 0 else None
        sg_label = self._get_sg_label()

        # ─── CoolStep-based triggers (only when CS actually regulates) ───
        # Use stricter thresholds when CoolStep is active to avoid
        # false-positives from normal regulation transitions.
        CS_FULL_MAX = 30.0      # CS_ACTUAL=31 is hardware max for TMC drivers
        CS_REGULATING_HIGH = 25.0  # below this = CoolStep was actively regulating

        if coolstep_active:
            cs_step_change = cs_med - prev_cs['median']

            # Check: was CS regulating below the high threshold within
            # the recent history? (Not just the immediately previous step
            # — gradual ramp-up like 22→28→31 should still count as
            # "transitioned to max from regulation".)
            recent_cs_was_regulating = any(
                r_.get('cs') and r_['cs']['median'] < CS_REGULATING_HIGH
                for r_ in results[-4:-1]  # last 3 steps before current
            )

            # A1: CS pegged at max + SG sharp drop (the canonical slip signature)
            #     - Current CS at maximum (>= 30)
            #     - CS was actually regulating recently (proves we
            #       weren't already pegged for the entire run)
            #     - SG dropped >= 30% vs previous step (load really increased)
            if (evaluate_triggers
                    and cs_med >= CS_FULL_MAX
                    and recent_cs_was_regulating
                    and sg_med is not None
                    and results[-2].get('sg')
                    and results[-2]['sg'].get('median') is not None
                    and results[-2]['sg']['median'] > 0):
                prev_sg = results[-2]['sg']['median']
                sg_drop_pct = (prev_sg - sg_med) / prev_sg * 100.0
                if sg_drop_pct >= 30.0:
                    return ("CS_ACTUAL pegged at max (%.0f → %.0f) "
                            "AND %s dropped %.0f%% (%.0f → %.0f) — "
                            "load suddenly increased to limit, slip "
                            "detected"
                            % (prev_cs['median'], cs_med,
                               sg_label, sg_drop_pct, prev_sg, sg_med))

            # A2: CS pegged at max + CV spike (intermittent slip)
            #     CoolStep regulated then suddenly maxed, AND repeats
            #     started diverging — clear sign of intermittent slip.
            if (evaluate_triggers
                    and cs_med >= CS_FULL_MAX
                    and recent_cs_was_regulating):
                last_rc = r.get('run_consistency') or {}
                last_cv = last_rc.get('sg_cv', 0)
                if last_cv >= 5.0:
                    cv_reason = self._check_cv_spike(results, sg_label)
                    if cv_reason:
                        return ("CS_ACTUAL pegged at max (%.0f → %.0f) "
                                "AND %s"
                                % (prev_cs['median'], cs_med, cv_reason))

            # A3: CS hard drop after regulation (hard stall — motor decoupled)
            #     CS dropped sharply after having been regulating.
            #     Indicates motor lost contact with the load entirely.
            if (evaluate_triggers
                    and prev_cs['median'] >= CS_REGULATING_HIGH
                    and cs_step_change < -8.0):
                had_regulation = any(
                    r_.get('cs') and r_['cs']['median'] < CS_REGULATING_HIGH
                    for r_ in results[:-1])
                if had_regulation:
                    return ("CS_ACTUAL dropped %.1f in one step "
                            "(median %.1f → %.1f) — hard stall: "
                            "motor lost load contact"
                            % (-cs_step_change, prev_cs['median'], cs_med))

        # ─── SG trend triggers (snap-back / over-jump) ───
        # These are direction-sensitive and meaningful only during the
        # monotonic up-sweep of coarse phase. In bisection we probe
        # arbitrary flows, so the trend baseline is meaningless — we
        # rely on CV-spike and IQR-spread (below) instead.
        if (going_up_or_same
                and not in_bisection
                and sg_med is not None
                and sg_med > self._sg_min_informative()
                and len(results) >= 4):
            sg_deltas = []
            for j in range(max(1, len(results) - 5), len(results) - 1):
                rj, rj_prev = results[j], results[j-1]
                if not (rj.get('sg') and rj_prev.get('sg')
                        and rj.get('cs') and rj_prev.get('cs')):
                    continue
                # When CoolStep is actively regulating, SG values can
                # be biased by the changing current — exclude steps
                # where CS was at max from the trend baseline.
                if coolstep_active:
                    if (rj['cs']['median'] >= CS_FULL_MAX
                            or rj_prev['cs']['median'] >= CS_FULL_MAX):
                        continue
                sg_deltas.append(
                    rj['sg']['median'] - rj_prev['sg']['median'])

            if len(sg_deltas) >= 3:
                expected_delta = sum(sg_deltas) / len(sg_deltas)
                actual_delta = (sg_med - results[-2]['sg']['median'])

                # Detect trend direction from the data, then look for
                # a sharp move opposite to the trend.
                if expected_delta > 1.0:
                    trend_sign = +1
                elif expected_delta < -1.0:
                    trend_sign = -1
                else:
                    trend_sign = 0

                if trend_sign != 0:
                    expected_load = expected_delta * trend_sign
                    actual_load = actual_delta * trend_sign

                    # Snap-back
                    if (actual_load < -expected_load
                            and abs(actual_delta) > self._sg_jump_threshold()):
                        if trend_sign > 0:
                            return ("%s reload jump: %+.0f (expected "
                                    "to keep rising ~+%.0f) — slip "
                                    "detected"
                                    % (sg_label, actual_delta,
                                       expected_load))
                        return ("%s reload jump: %+.0f (expected to "
                                "keep falling ~-%.0f) — slip detected"
                                % (sg_label, actual_delta,
                                   expected_load))

                    # Over-jump
                    if (actual_load > expected_load * 2.0
                            and abs(actual_delta) > self._sg_jump_threshold()):
                        if trend_sign > 0:
                            return ("%s abnormal jump: %+.0f vs "
                                    "expected ~+%.0f (%.1fx larger) — "
                                    "slip detected"
                                    % (sg_label, actual_delta,
                                       expected_load,
                                       actual_load / expected_load))
                        return ("%s abnormal drop: %+.0f vs expected "
                                "~-%.0f (%.1fx larger) — slip detected"
                                % (sg_label, actual_delta,
                                   expected_load,
                                   actual_load / expected_load))

        # CV spike fallback — also fires in CS mode (catches intermittent
        # slip even when CS-based triggers above don't fire because
        # CoolStep is regulating smoothly).
        if evaluate_triggers:
            cv_reason = self._check_cv_spike(results, sg_label)
            if cv_reason:
                return cv_reason

            # IQR/spread anomaly fallback — catches "quiet" stalls
            # where median is hidden but distribution widens.
            iqr_reason = self._check_iqr_spread(results, sg_label)
            if iqr_reason:
                return iqr_reason

            # SG max spike — decoupling event in at least one repeat.
            spike_reason = self._check_sg_max_spike(results, sg_label)
            if spike_reason:
                return spike_reason

            # Per-run outlier — one of the N repeats was clearly
            # different from the others (intermittent slip).
            outlier_reason = self._check_run_outlier(results, sg_label)
            if outlier_reason:
                return outlier_reason

        return None

    # ─── Helper: rotation_distance lookup ───────────────────────────

    def _get_rotation_distance(self, extruder):
        try:
            rd = extruder.extruder_stepper.stepper.get_rotation_distance()
            return rd[0] if isinstance(rd, tuple) else rd
        except AttributeError:
            pass
        try:
            rd = extruder.stepper.get_rotation_distance()
            return rd[0] if isinstance(rd, tuple) else rd
        except AttributeError:
            pass
        try:
            cfg = self.printer.lookup_object('configfile')
            settings = cfg.get_status(
                self.reactor.monotonic())['settings']
            return float(settings[self.stepper_name]['rotation_distance'])
        except Exception:
            return None

    # ─── TMC_FLOW_STATUS — diagnostic ───────────────────────────────

    def cmd_TMC_FLOW_STATUS(self, gcmd):
        """Show current TMC SG/CS values for the extruder."""
        self._lookup_tmc()
        activate = gcmd.get_int('ACTIVATE', 1, minval=0, maxval=1)

        if activate:
            gcmd.respond_info("Activating motor (1 mm extrusion)...")
            self.gcode.run_script_from_command(
                "M83\nG1 E1 F60\nM400\nG92 E0")

        gcmd.respond_info(
            "===== TMC %s Status (stepper '%s') ====="
            % (self.driver_type or 'unknown', self.stepper_name))

        sg = self._read_sg()
        cs = self._read_cs()
        sg_label = self._get_sg_label()
        gcmd.respond_info(
            "%s: %s (range 0-510, lower = more load)"
            % (sg_label, str(sg) if sg is not None else 'n/a'))
        gcmd.respond_info(
            "CS_ACTUAL: %s (CoolStep current scale, 0-31)"
            % (str(cs) if cs is not None else 'n/a'))

        # Detect which mode the user has configured for
        semin = self._read_cs_semin()
        if semin == 0:
            preferred_mode = 'sg'
            gcmd.respond_info(
                "→ Detected configuration: CoolStep DISABLED "
                "(driver_SEMIN=0)\n"
                "  Run: TMC_FLOW_FIND_MAX  "
                "(auto-selects SG-only mode for this config)")
        else:
            preferred_mode = 'cs'
            gcmd.respond_info(
                "→ Detected configuration: CoolStep ENABLED "
                "(driver_SEMIN=%d)\n"
                "  Run: TMC_FLOW_FIND_MAX  "
                "(auto-selects CoolStep mode for this config)" % semin)

        # Run check for the preferred mode and show results
        problems, infos = self._check_tmc_config(preferred_mode)
        if problems:
            gcmd.respond_info(
                "Configuration issues found for %s mode:"
                % preferred_mode.upper())
            for fname, val, _desc in problems:
                gcmd.respond_info("  ⚠ %s = %s" % (fname, val))
        else:
            gcmd.respond_info(
                "✓ Configuration looks good for %s mode."
                % preferred_mode.upper())
        for info in infos:
            gcmd.respond_info(info)

    def _read_cs_semin(self):
        """Helper: read semin to determine current mode. 0 if unknown."""
        try:
            val = self.tmc.fields.get_field('semin')
            return val if val is not None else 0
        except (KeyError, AttributeError):
            return 0

    def _snapshot_tmc_settings(self):
        """Read TMC register state most relevant to this test.

        Captures only the fields that affect StallGuard / CoolStep
        behaviour during the flow test, so the report has a short and
        focused paper trail of the configuration that produced the
        result. Missing fields are skipped silently.
        """
        snapshot = []
        if self.tmc is None:
            return snapshot

        def get(name):
            try:
                return self.tmc.fields.get_field(name)
            except (KeyError, AttributeError):
                return None

        def add(label, name):
            v = get(name)
            if v is None:
                return
            snapshot.append((label, str(v), name))

        # Identity
        snapshot.append(("Driver", str(self.driver_type or '?'),
                         'driver_type'))
        snapshot.append(("Stepper", str(self.stepper_name or '?'),
                         'stepper_name'))

        # Chopper mode (decides which StallGuard engine works)
        add("en_pwm_mode (1=StealthChop)", 'en_pwm_mode')
        add("en_spreadCycle (1=SpreadCycle, TMC2209)", 'en_spreadCycle')

        # Velocity gates that govern StallGuard / CoolStep activity
        add("TPWMTHRS (StealthChop velocity threshold)", 'tpwmthrs')
        add("TCOOLTHRS (CoolStep / StallGuard velocity threshold)",
            'tcoolthrs')
        add("THIGH (full-step velocity threshold)", 'thigh')

        # StallGuard sensitivity
        add("SGTHRS (TMC2209 StallGuard4 threshold)", 'sgthrs')
        add("SGT (StallGuard2 threshold, signed -64..+63)", 'sgt')
        add("sg4_thrs (StallGuard4 threshold, TMC2240)", 'sg4_thrs')
        add("sg4_filt_en (SG4 filter enabled)", 'sg4_filt_en')
        add("SFILT (SG2 filter enabled)", 'sfilt')

        # CoolStep configuration
        add("SEMIN (CoolStep lower threshold = SEMIN*32)", 'semin')
        add("SEMAX (CoolStep upper threshold offset)", 'semax')
        add("SEUP (current-up step size)", 'seup')
        add("SEDN (current-down step size)", 'sedn')
        add("SEIMIN (CoolStep min current; 0=1/2 IRUN, 1=1/4)",
            'seimin')

        # Run-time current state
        add("IRUN (run current scale 0..31)", 'irun')

        return snapshot

    # ─── Main commands ──────────────────────────────────────────────

    def cmd_TMC_FLOW_FIND_MAX(self, gcmd):
        """Auto-detect mode from driver_SEMIN, then run flow test.

        Optional MODE parameter to override:
          MODE=auto  (default) — detect from driver_SEMIN
          MODE=sg              — force SG-only mode
          MODE=cs              — force CoolStep + SG mode
        """
        self._lookup_tmc()

        mode_param = gcmd.get('MODE', 'auto').lower()

        if mode_param == 'auto':
            # Auto-detect from driver_SEMIN
            try:
                semin = self.tmc.fields.get_field('semin')
                if semin is None:
                    semin = 0
            except (KeyError, AttributeError):
                raise gcmd.error(
                    "Could not read driver_SEMIN to auto-detect mode. "
                    "Please specify MODE=sg or MODE=cs explicitly.")

            if semin == 0:
                mode = 'sg'
                gcmd.respond_info(
                    "Auto-detect: driver_SEMIN = 0 → CoolStep is disabled. "
                    "Using SG-only mode.")
            else:
                mode = 'cs'
                gcmd.respond_info(
                    "Auto-detect: driver_SEMIN = %d → CoolStep is enabled. "
                    "Using CoolStep + SG mode." % semin)
        elif mode_param == 'sg':
            mode = 'sg'
            gcmd.respond_info("Mode forced via MODE=sg parameter.")
        elif mode_param == 'cs':
            mode = 'cs'
            gcmd.respond_info("Mode forced via MODE=cs parameter.")
        else:
            raise gcmd.error(
                "Invalid MODE parameter: '%s'. Use 'auto', 'sg', or 'cs'."
                % mode_param)

        self._mode = mode
        self._run_find_max(gcmd, mode=mode)

    def cmd_TMC_FLOW_FIND_MAX_SG(self, gcmd):
        """Force SG-only mode — for setups with CoolStep disabled.

        Equivalent to: TMC_FLOW_FIND_MAX MODE=sg
        """
        self._mode = 'sg'
        self._run_find_max(gcmd, mode='sg')

    def cmd_TMC_FLOW_FIND_MAX_CS(self, gcmd):
        """Force CoolStep + SG mode — for setups with CoolStep enabled.

        Equivalent to: TMC_FLOW_FIND_MAX MODE=cs
        """
        self._mode = 'cs'
        self._run_find_max(gcmd, mode='cs')

    def _run_find_max(self, gcmd, mode):
        """Common bracket-bisection algorithm. Trigger logic switches
        based on mode."""
        self._lookup_tmc()

        start_flow = gcmd.get_float('START', 10.0, above=0.)
        max_flow = gcmd.get_float('MAX', 80.0, above=start_flow)
        coarse_step = gcmd.get_float('COARSE_STEP', 10.0, above=0.)
        min_step = gcmd.get_float('MIN_STEP', 1.0, above=0.)
        step_duration = gcmd.get_float('DURATION', 5.0, above=0.5)
        repeat = gcmd.get_int('REPEAT', 5, minval=1, maxval=10)
        verify_repeats = gcmd.get_int(
            'VERIFY_REPEATS', 5, minval=1, maxval=10)
        purge = gcmd.get_float('PURGE', 0.0, minval=0.)
        cooldown = gcmd.get_float('COOLDOWN', 15.0, minval=0., maxval=300.)
        max_bisect = gcmd.get_int(
            'MAX_BISECT_STEPS', 6, minval=2, maxval=15)
        no_html = gcmd.get_int('NO_HTML', 0, minval=0, maxval=1)
        skip_check = gcmd.get_int(
            'SKIP_TMC_CHECK', 0, minval=0, maxval=1)

        if min_step >= coarse_step:
            raise gcmd.error(
                "MIN_STEP (%.1f) must be smaller than COARSE_STEP (%.1f)"
                % (min_step, coarse_step))

        # ─── TMC config check ───
        if not skip_check:
            problems, infos = self._check_tmc_config(mode)
            if problems:
                msg = ("\n=== TMC Configuration Issue(s) for %s mode ===\n"
                       % mode.upper())
                for fname, val, desc in problems:
                    msg += "Problem: %s (current value: %s)\n%s\n\n" % (
                        fname, val, desc)
                msg += ("After updating printer.cfg, run:\n"
                        "  FIRMWARE_RESTART\n"
                        "  TMC_FLOW_FIND_MAX\n\n"
                        "To skip this check (advanced):\n"
                        "  TMC_FLOW_FIND_MAX SKIP_TMC_CHECK=1")
                raise gcmd.error(msg)
            gcmd.respond_info(
                "TMC configuration check passed for %s mode."
                % mode.upper())
            for info in infos:
                gcmd.respond_info(info)

        # ─── Hotend temp check ───
        extruder = self.printer.lookup_object('extruder')
        heater = extruder.get_heater()
        cur_temp, _ = heater.get_temp(self.reactor.monotonic())
        target_temp = heater.target_temp
        if cur_temp < self.min_hotend_temp:
            raise gcmd.error(
                "Hotend too cold: %.1f°C (min %.1f°C)."
                % (cur_temp, self.min_hotend_temp))
        if target_temp > 0 and cur_temp < target_temp - 5.0:
            raise gcmd.error(
                "Hotend not at target: %.1f°C (target %.1f°C). "
                "Wait for M109 or run M109 S%d before testing."
                % (cur_temp, target_temp, int(target_temp)))

        rotation_distance = self._get_rotation_distance(extruder)
        if rotation_distance is None:
            raise gcmd.error("Could not determine rotation_distance")

        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        sg_label = self._get_sg_label()
        meta = {
            'timestamp': timestamp,
            'mode': mode.upper(),
            'driver': '%s on %s (%s register, sample rate %.0f Hz)' % (
                self.driver_type, self.stepper_name, sg_label,
                1.0 / SAMPLE_INTERVAL),
            'algorithm': 'ADAPTIVE BISECTION (coarse=%.1f, min=%.1f, '
                         'repeats=%d)' % (coarse_step, min_step, repeat),
            'hotend_temp': '%.1f °C (target %.1f °C)' % (
                cur_temp, target_temp),
            'filament_diameter': '%.2f mm' % self.filament_diameter,
            'melt_zone_length': '%.1f mm' % self.melt_zone_length,
            'rotation_distance': '%.4f mm' % rotation_distance,
            'flow_range': '%.1f → %.1f mm³/s (adaptive)' % (
                start_flow, max_flow),
            'step_duration': '%.1f s' % step_duration,
            'tmc_settings': self._snapshot_tmc_settings(),
        }

        # ─── Banner ───
        mode_desc = ("SG-only mode (CoolStep disabled, "
                     "SG abnormal jump + plateau triggers)" if mode == 'sg'
                     else "CoolStep + SG mode (CoolStep enabled, "
                     "CS jump/leave/drop + SG backup triggers)")
        gcmd.respond_info(
            "===== TMC Flow Test v%s — %s mode =====\n"
            "%s\n"
            "Range: %.0f → %.0f mm³/s | Coarse: %.0f | Min: %.0f mm³/s\n"
            "Each measurement: %d reps × %.1f s (median over all samples)\n"
            "Cool-down between phases: %.0f s | Hotend: %.1f°C "
            "(target %.1f°C)\n"
            "Sample rate: %.0f Hz | Driver: %s | Register: %s\n"
            "------------------------------------------------\n"
            "Algorithm:\n"
            "  Phase 1 (Coarse): increase flow in big steps until trigger\n"
            "  Phase 2 (Bisection): narrow to ±%.0f mm³/s by halving\n"
            "  Phase 3 (Verification): confirm with %d reps\n"
            "================================================"
            % (MODULE_VERSION, mode.upper(), mode_desc,
               start_flow, max_flow, coarse_step, min_step,
               repeat, step_duration, cooldown,
               cur_temp, target_temp,
               1.0 / SAMPLE_INTERVAL, self.driver_type, sg_label,
               min_step, verify_repeats))

        if purge > 0:
            self.gcode.run_script_from_command(
                "M83\nG1 E%.2f F300\nG92 E0" % purge)
        self.gcode.run_script_from_command("M83\nG92 E0")

        results = []
        baseline_cs = None
        sample_cs = (mode == 'cs')

        def measure_and_save(flow, phase):
            r = self._measure_step(
                gcmd, flow, step_duration, repeat,
                sample_cs=sample_cs)
            r['phase'] = phase
            results.append(r)
            self._save_report(
                results, meta, timestamp, None, no_html, mode,
                gcmd=None, announce=False)
            return r

        def check(results):
            """Dispatch to right trigger function based on mode."""
            if mode == 'sg':
                return self._check_triggers_sg(results)
            return self._check_triggers_cs(results, baseline_cs)

        # ─── PHASE 1: Coarse ───
        gcmd.respond_info(
            "\n>>> Phase 1: Coarse Upward Sweep <<<\n"
            "  Stepping up by %.0f mm³/s. Each step: %d reps × %.1f s."
            % (coarse_step, repeat, step_duration))

        flow = start_flow
        low = None
        high = None
        first_trigger_reason = None
        # Track every trigger / borderline / verify-fail event for the
        # HTML "decision trail" panel. Each entry: dict with keys
        # 'phase', 'flow', 'kind' (one of: 'trigger', 'borderline-retest',
        # 'borderline-confirmed', 'verify-fail', 'verify-borderline'),
        # 'reason', 'cv', 'iqr', 'sg_median'.
        trigger_events = []

        def _record_event(phase, flow, kind, reason, last_result=None):
            ev = {'phase': phase, 'flow': flow,
                  'kind': kind, 'reason': reason,
                  'cv': None, 'iqr': None, 'sg_median': None}
            if last_result is not None:
                rc = last_result.get('run_consistency') or {}
                ev['cv'] = rc.get('sg_cv')
                sg = last_result.get('sg') or {}
                if 'p25' in sg and 'p75' in sg:
                    ev['iqr'] = sg['p75'] - sg['p25']
                if 'median' in sg:
                    ev['sg_median'] = sg['median']
            trigger_events.append(ev)

        while flow <= max_flow + 0.001:
            r = measure_and_save(flow, 'coarse')

            # Track baseline CS for CS-mode trigger thresholds
            if (mode == 'cs' and baseline_cs is None
                    and r.get('cs') and r['cs']['n'] > 0):
                cs_med = r['cs']['median']
                if cs_med < 30:  # not in saturation
                    baseline_cs = cs_med

            reason = check(results)
            if reason:
                high = flow
                low = flow - coarse_step
                first_trigger_reason = reason
                _record_event('coarse', flow, 'trigger', reason,
                              results[-1] if results else None)
                gcmd.respond_info(
                    "  >>> TRIGGER at %.1f mm³/s — %s\n"
                    "      → Safe range narrowed to [%.1f, %.1f] mm³/s"
                    % (flow, reason, low, high))
                break

            low = flow
            self.gcode.run_script_from_command("G4 P500")
            flow += coarse_step

        if high is None:
            gcmd.respond_info(
                "Reached MAX %.1f mm³/s without trigger. Try MAX higher."
                % max_flow)
            self._save_report(results, meta, timestamp, None,
                              no_html, mode, gcmd=gcmd)
            return

        if low < start_flow:
            gcmd.respond_info(
                "Trigger fired on first step (%.1f) — lower START." % high)
            self._save_report(results, meta, timestamp, first_trigger_reason,
                              no_html, mode, gcmd=gcmd)
            return

        # ─── PHASES 2 + 3: Bisection + Verify (with retry) ───
        # Phase 2 narrows the bracket; Phase 3 confirms the result. If
        # Phase 3 fails (verify itself trips a trigger or stays
        # borderline on re-test), we drop back into Phase 2 with a
        # tightened high bound and try again. Up to MAX_VERIFY_FAILURES
        # rounds before giving up — without this cap a hardware /
        # filament issue could loop forever.
        MAX_VERIFY_FAILURES = 3

        # Track which flows we've already re-tested in bisection (set
        # is reset on each bisection re-entry to allow fresh re-tests
        # at the new bracket).
        last_trigger_reason = first_trigger_reason
        verify_failures = 0
        verify_result = None
        verify_cv = 0.0

        while True:
            # ─── PHASE 2: Bisection ───
            if cooldown > 0:
                gcmd.respond_info(
                    "  ... Cool-down: %.0f s ..." % cooldown)
                self.gcode.run_script_from_command(
                    "G4 P%d" % int(cooldown * 1000))

            phase2_label = (("\n>>> Phase 2: Bisection (retry %d) <<<"
                             % verify_failures)
                            if verify_failures > 0
                            else "\n>>> Phase 2: Bisection <<<")
            gcmd.respond_info(
                "%s\n"
                "  Narrowing [%.0f, %.0f] by halving until interval ≤ %.0f. "
                "Up to %d steps.\n"
                "  Borderline measurements (CV 4-7%% or IQR 15-24) are "
                "re-tested once for confirmation."
                % (phase2_label, low, high, min_step, max_bisect))

            bisect_iter = 0
            retested_flows = set()
            while ((high - low) > min_step + 0.001
                   and bisect_iter < max_bisect):
                bisect_iter += 1
                raw_mid = (low + high) / 2.0
                mid = round(raw_mid / min_step) * min_step
                if mid <= low + 0.001 or mid >= high - 0.001:
                    break

                r = measure_and_save(mid, 'bisect')
                reason = check(results)

                # Borderline check: if no clear trigger fired but the
                # data sits in the gray zone, re-measure once before
                # classifying.
                if (reason is None
                        and mid not in retested_flows):
                    borderline_why = self._is_borderline(results)
                    if borderline_why:
                        retested_flows.add(mid)
                        _record_event('bisect', mid, 'borderline-retest',
                                      borderline_why,
                                      results[-1] if results else None)
                        gcmd.respond_info(
                            "  >>> %.1f mm³/s BORDERLINE — %s\n"
                            "      Re-measuring once for confirmation..."
                            % (mid, borderline_why))
                        if cooldown > 0:
                            self.gcode.run_script_from_command(
                                "G4 P%d" % int(cooldown * 1000))
                        # Drop the borderline measurement and replace
                        # with the fresh one.
                        results.pop()
                        r2 = measure_and_save(mid, 'bisect')
                        reason = check(results)
                        if reason is None:
                            re_borderline = self._is_borderline(results)
                            if re_borderline:
                                reason = ("borderline confirmed on re-"
                                          "test: " + re_borderline)
                                _record_event(
                                    'bisect', mid, 'borderline-confirmed',
                                    re_borderline,
                                    results[-1] if results else None)

                if reason:
                    high = mid
                    last_trigger_reason = reason
                    # Avoid double-recording if borderline-confirmed
                    # already recorded above.
                    if not (trigger_events
                            and trigger_events[-1].get('flow') == mid
                            and trigger_events[-1].get('kind')
                            == 'borderline-confirmed'):
                        _record_event('bisect', mid, 'trigger', reason,
                                      results[-1] if results else None)
                    gcmd.respond_info(
                        "  >>> TRIGGER at %.1f — %s\n"
                        "      → [%.1f, %.1f] (%d/%d)"
                        % (mid, reason, low, high,
                           bisect_iter, max_bisect))
                else:
                    low = mid
                    gcmd.respond_info(
                        "  >>> %.1f mm³/s SAFE → [%.1f, %.1f] (%d/%d)"
                        % (mid, low, high, bisect_iter, max_bisect))
                self.gcode.run_script_from_command("G4 P500")

            # ─── PHASE 3: Verify ───
            if cooldown > 0:
                gcmd.respond_info(
                    "  ... Cool-down before verification: %.0f s ..."
                    % cooldown)
                self.gcode.run_script_from_command(
                    "G4 P%d" % int(cooldown * 1000))

            gcmd.respond_info(
                "\n>>> Phase 3: Verification at %.1f mm³/s <<<\n"
                "  Confirming with %d repetitions."
                % (low, verify_repeats))
            verify_result = self._measure_step(
                gcmd, low, step_duration, verify_repeats,
                sample_cs=sample_cs)
            verify_result['phase'] = 'verify'
            results.append(verify_result)

            # Check verify result against triggers and borderline.
            verify_trigger = check(results)
            if verify_trigger is None:
                verify_borderline = self._is_borderline(results)
                if verify_borderline:
                    # Re-measure verify once at the same flow before
                    # accepting; if it stays borderline, treat as fail.
                    _record_event('verify', low, 'verify-borderline',
                                  verify_borderline,
                                  results[-1] if results else None)
                    gcmd.respond_info(
                        "  >>> Verify at %.1f BORDERLINE — %s\n"
                        "      Re-measuring once for confirmation..."
                        % (low, verify_borderline))
                    if cooldown > 0:
                        self.gcode.run_script_from_command(
                            "G4 P%d" % int(cooldown * 1000))
                    results.pop()
                    verify_result = self._measure_step(
                        gcmd, low, step_duration, verify_repeats,
                        sample_cs=sample_cs)
                    verify_result['phase'] = 'verify'
                    results.append(verify_result)
                    verify_trigger = check(results)
                    if verify_trigger is None:
                        re_borderline = self._is_borderline(results)
                        if re_borderline:
                            verify_trigger = ("borderline confirmed on "
                                              "re-test: "
                                              + re_borderline)

            if verify_trigger is None:
                # Verify clean — accept the result.
                break

            # Verify FAILED. Drop back into bisection with a tighter
            # high bound (the just-failed flow becomes the new high).
            verify_failures += 1
            _record_event('verify', low, 'verify-fail', verify_trigger,
                          results[-1] if results else None)
            verify_cv_now = (verify_result.get('run_consistency', {})
                             .get('sg_cv', 0.0))
            gcmd.respond_info(
                "\n  >>> ⚠ VERIFY FAILED at %.1f mm³/s "
                "(CV %.1f%%, attempt %d/%d)\n"
                "      Reason: %s"
                % (low, verify_cv_now, verify_failures,
                   MAX_VERIFY_FAILURES, verify_trigger))

            if verify_failures >= MAX_VERIFY_FAILURES:
                # Give up gracefully — report the next-lower safe step
                # we have evidence for. Search results history for the
                # last bisect/coarse step at flow < low that did NOT
                # trigger.
                fallback = None
                for r in reversed(results[:-1]):  # skip the failed verify
                    if r.get('phase') in ('coarse', 'bisect'):
                        if r['flow'] < low - 0.001:
                            fallback = r['flow']
                            break
                if fallback is None:
                    fallback = max(start_flow, low - min_step)
                gcmd.respond_info(
                    "  Maximum verify retries reached — falling back "
                    "to last known-good flow %.1f mm³/s." % fallback)
                low = fallback
                last_trigger_reason = verify_trigger
                # Do one final verify at the fallback to populate the
                # final verify_result with this lower value.
                if cooldown > 0:
                    self.gcode.run_script_from_command(
                        "G4 P%d" % int(cooldown * 1000))
                gcmd.respond_info(
                    "\n>>> Phase 3: Final verification at %.1f mm³/s "
                    "<<<\n  Confirming with %d repetitions."
                    % (low, verify_repeats))
                verify_result = self._measure_step(
                    gcmd, low, step_duration, verify_repeats,
                    sample_cs=sample_cs)
                verify_result['phase'] = 'verify'
                results.append(verify_result)
                break

            # Tighten bracket: failed flow becomes new high.
            high = low
            # Drop low to the previous safe step we have data for, or
            # one min_step below if no earlier safe step exists.
            new_low = None
            for r in reversed(results[:-1]):  # skip failed verify
                if r.get('phase') in ('coarse', 'bisect'):
                    if r['flow'] < high - 0.001:
                        # Was this flow a safe one?  Check it didn't
                        # carry a trigger.
                        # (Heuristic: if the flow appears as a
                        # 'safe' entry — i.e. CV < 4% AND not
                        # borderline by current standards — accept
                        # it as the new low.)
                        rc = r.get('run_consistency') or {}
                        cv = rc.get('sg_cv', 99.0)
                        sgp = r.get('sg') or {}
                        iqr = (sgp.get('p75', 0) - sgp.get('p25', 0)
                               if 'p25' in sgp and 'p75' in sgp else 99)
                        if cv < 4.0 and iqr < 15:
                            new_low = r['flow']
                            break
            if new_low is None:
                new_low = max(start_flow, high - 2 * min_step)
            low = new_low
            last_trigger_reason = verify_trigger
            gcmd.respond_info(
                "      → Re-entering bisection with bracket "
                "[%.1f, %.1f]" % (low, high))

            # Sanity: bracket too tight to bisect → just accept low.
            if (high - low) <= min_step + 0.001:
                gcmd.respond_info(
                    "      Bracket already tight, "
                    "accepting %.1f as final." % low)
                # Do one more verify at this low to capture the final
                # measurement metadata.
                if cooldown > 0:
                    self.gcode.run_script_from_command(
                        "G4 P%d" % int(cooldown * 1000))
                gcmd.respond_info(
                    "\n>>> Phase 3: Final verification at %.1f mm³/s "
                    "<<<\n  Confirming with %d repetitions."
                    % (low, verify_repeats))
                verify_result = self._measure_step(
                    gcmd, low, step_duration, verify_repeats,
                    sample_cs=sample_cs)
                verify_result['phase'] = 'verify'
                results.append(verify_result)
                break

        verify_cv = (verify_result.get('run_consistency', {}).get('sg_cv', 0)
                     if verify_result.get('run_consistency') else 0)

        max_safe = low
        if verify_cv < 5:
            quality = "excellent (very stable)"
        elif verify_cv < 10:
            quality = "good (stable)"
        elif verify_cv < 20:
            quality = "acceptable (some variation)"
        else:
            quality = "poor (high variation — re-run advised)"

        # ─── CoolStep diagnostic (only in CS mode) ───
        # If CS_ACTUAL stayed pinned at 31 throughout the test, CoolStep
        # never had a chance to regulate. Per Trinamic AN-002, that
        # means SG_RESULT was always below SEMIN*32, so SEMIN is set
        # too high for this motor / load. Recommend a better value.
        if mode == 'cs':
            cs_meds = [r['cs']['median'] for r in results
                       if r.get('cs') and r['cs'].get('n', 0) > 0]
            sg_meds = [r['sg']['median'] for r in results
                       if r.get('sg') and r['sg'].get('n', 0) > 0]
            if (cs_meds and sg_meds
                    and max(cs_meds) - min(cs_meds) < 1.0
                    and min(cs_meds) >= 30):
                sg_max_seen = max(sg_meds)
                cur_semin = self._read_cs_semin()
                # AN-002: SEMIN ≈ SG_MAX/4..SG_MAX/8
                # Lower threshold to ramp current UP is SEMIN*32, so we
                # want SEMIN*32 to be just above SG_MAX_seen so CoolStep
                # has room to regulate down when load drops.
                rec_semin = max(1, int(round(sg_max_seen / 32.0)) + 1)
                gcmd.respond_info(
                    "\n----- CoolStep diagnostic -----\n"
                    "CS_ACTUAL stayed pinned at %.0f for the whole "
                    "test (range %.1f).\n"
                    "Your SG_RESULT range was %.0f-%.0f; with "
                    "driver_SEMIN = %d the CoolStep ramp-up\n"
                    "threshold is SEMIN*32 = %d, which sits ABOVE "
                    "every SG value seen — so CoolStep is in\n"
                    "permanent 'ramp current up' mode and never "
                    "regulates. Slip detection in this run\n"
                    "relied on the SG fallback path, which still "
                    "works fine.\n"
                    "\n"
                    "If you want CoolStep to actually regulate (per "
                    "Trinamic AN-002, SEMIN ≈ SG_MAX/4..SG_MAX/8),\n"
                    "try in your [%s extruder] section:\n"
                    "  driver_SEMIN: %d\n"
                    "  driver_SEMAX: 4\n"
                    "  driver_SEUP:  3\n"
                    "  driver_SEDN:  1\n"
                    "Then FIRMWARE_RESTART and re-run."
                    % (max(cs_meds),
                       max(cs_meds) - min(cs_meds),
                       min(sg_meds), sg_max_seen,
                       cur_semin, cur_semin * 32,
                       self.driver_type, rec_semin))

        gcmd.respond_info(
            "\n========== FINAL RESULT ==========\n"
            "Test mode: %s\n"
            "Maximum safe volumetric flow: %.1f mm³/s\n"
            "Verification quality: %s (CV = %.1f%%)\n"
            "----------------------------------\n"
            "Slicer recommendation:\n"
            "  Conservative (80%%): %.1f mm³/s   ← recommended\n"
            "  Aggressive (90%%):   %.1f mm³/s   ← only with margin\n"
            "----------------------------------\n"
            "Detailed data: see CSV/HTML report\n"
            "=================================="
            % (mode.upper(), max_safe, quality, verify_cv,
               max_safe * 0.8, max_safe * 0.9))

        self._save_report(results, meta, timestamp, last_trigger_reason,
                          no_html, mode, gcmd=gcmd,
                          final_result={
                              'max_safe': max_safe,
                              'verify_cv': verify_cv,
                              'quality': quality,
                              'stop_reason': last_trigger_reason,
                              'trigger_events': trigger_events,
                              'baseline_stats':
                                  self._coarse_baseline_stats(results),
                          })
        self.gcode.run_script_from_command("G92 E0")


def load_config(config):
    return TMCFlowTest(config)
