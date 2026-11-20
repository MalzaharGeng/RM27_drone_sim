#!/usr/bin/env python3
"""[NP-01 FIX6-corrected] Takeoff height feedback contract.

Dependency-free (no ROS / no PX4 import) so the decision can be unit tested
with normal and fault inputs. Full rationale and ULog measurements:
newpkg_20261115/evidence/np01_height_feedback_contract.md

Contract summary:
  * source: DDS /fmu/out/vehicle_local_position_v1 (VehicleLocalPosition).
    MAVLink LOCAL_POSITION_NED carries NO validity/reset fields
    (mavlink/streams/LOCAL_POSITION_NED.hpp:64-75), so it cannot self-validate.
  * frame: PX4 local NED, z is DOWN positive. Climb = z0 - z (SIGNED).
    abs() is revoked: a descent must never count as a successful takeoff.
  * validity: z_valid AND v_z_valid are required. xy_valid / v_xy_valid are
    recorded independently and are NEITHER a veto NOR a proof for height.
  * freshness: host-monotonic RECEIVE time only, now - rx_mono <= max_age_s.
    The source sample time (header.stamp / timestamp_sample) is recorded but
    never mixed with the host clock and never advanced.
  * reset: any z/vz reset-counter change recaptures the reference and does NOT
    accept a takeoff on that tick. Exhausting recaptures is an explicit failure.
  * sanity: all numeric inputs must be finite; a diverged |z0| or |vz| is
    rejected. Validity flags alone are NOT sufficient (ULog 04_09_40 had
    z_valid=1 with z=19.25 m, vz=16.5 m/s).
  * NO ground truth is used. On invalid/stale/diverged input the caller holds
    the throttle centred and waits; it must never declare takeoff success.

These bounds are INPUT sanity limits, not control limits, Gate limits or
acceptance thresholds. No existing threshold is relaxed.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TakeoffConfig:
    max_age_s: float = 0.5
    ground_abs_z_m: float = 2.0
    max_abs_vz_m_s: float = 3.0
    max_abs_delta_z_m: float = 2.0
    max_recaptures: int = 3


@dataclass(frozen=True)
class HeightSample:
    z: float
    vz: float
    z_valid: bool
    v_z_valid: bool
    xy_valid: bool
    v_xy_valid: bool
    z_reset_counter: int
    vz_reset_counter: int
    delta_z: float
    sample_us: int          # source sample time (header.stamp), RECORD ONLY
    rx_mono: float          # host monotonic receive time, for FRESHNESS only


@dataclass(frozen=True)
class HeightReference:
    z: float
    z_reset_counter: int
    vz_reset_counter: int
    sample_us: int
    rx_mono: float


@dataclass(frozen=True)
class HeightDecision:
    status: str             # "capture" | "wait" | "reached"
    reason: str
    ref: object             # HeightReference or None
    recapture: bool = False


DEFAULT_CONFIG = TakeoffConfig()


def _finite(*values):
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return False
        if not math.isfinite(float(v)):
            return False
    return True


def sample_rejection(s, now_mono, cfg=DEFAULT_CONFIG):
    """Return "" when the sample is usable as height feedback, else a reason."""
    if s is None:
        return "missing_sample"
    if not _finite(s.z, s.vz, s.delta_z, s.sample_us, s.rx_mono, now_mono):
        return "non_finite"
    if not s.z_valid:
        return "z_invalid"
    if not s.v_z_valid:
        return "vz_invalid"
    age = now_mono - s.rx_mono
    if not math.isfinite(age) or age < 0.0 or age > cfg.max_age_s:
        return "stale_sample"
    if abs(s.vz) > cfg.max_abs_vz_m_s:
        return "vz_diverged"
    if abs(s.delta_z) > cfg.max_abs_delta_z_m:
        return "reset_delta_diverged"
    # xy_valid / v_xy_valid are deliberately NOT checked here.
    return ""


def capture_reference(s, cfg=DEFAULT_CONFIG):
    if abs(s.z) > cfg.ground_abs_z_m:
        return None, "reference_z_diverged"
    return (HeightReference(s.z, s.z_reset_counter, s.vz_reset_counter,
                            s.sample_us, s.rx_mono),
            "reference_captured")


def evaluate_takeoff(s, ref, alt, now_mono,
                     cfg=DEFAULT_CONFIG, recaptures=0):
    """One decision tick. Never returns "reached" on invalid input."""
    if not _finite(alt, now_mono) or alt <= 0.0:
        return HeightDecision("wait", "invalid_alt", ref)
    bad = sample_rejection(s, now_mono, cfg)
    if bad:
        return HeightDecision("wait", bad, ref)
    if ref is None:
        new_ref, reason = capture_reference(s, cfg)
        if new_ref is None:
            return HeightDecision("wait", reason, None)
        return HeightDecision("capture", reason, new_ref)
    if not _finite(ref.z, ref.rx_mono):
        return HeightDecision("wait", "non_finite_reference", None)
    if (s.z_reset_counter != ref.z_reset_counter
            or s.vz_reset_counter != ref.vz_reset_counter):
        if recaptures + 1 > cfg.max_recaptures:
            return HeightDecision("wait", "reset_recapture_exhausted", ref, True)
        new_ref, reason = capture_reference(s, cfg)
        if new_ref is None:
            return HeightDecision("wait", reason, None, True)
        return HeightDecision("capture", "height_reset_recaptured", new_ref, True)
    climb = ref.z - s.z            # SIGNED: positive means climbing in NED
    if not math.isfinite(climb):
        return HeightDecision("wait", "non_finite_climb", ref)
    if climb < 0.0:
        return HeightDecision("wait", "descending", ref)
    if climb >= alt and s.z < ref.z:
        return HeightDecision("reached", "takeoff_altitude_reached", ref)
    return HeightDecision("wait", "climbing", ref)


def diagnostics(s, now_mono, cfg=DEFAULT_CONFIG):
    """Bounded, read-only fields for the pilot CSV."""
    if s is None:
        return {"z_valid": 0, "v_z_valid": 0, "xy_valid": 0, "v_xy_valid": 0,
                "z_reset_counter": -1, "vz_reset_counter": -1,
                "delta_z": "", "h_rx_age_ms": "", "h_reject": "missing_sample"}
    age = now_mono - s.rx_mono if _finite(s.rx_mono, now_mono) else float("nan")
    return {"z_valid": int(bool(s.z_valid)), "v_z_valid": int(bool(s.v_z_valid)),
            "xy_valid": int(bool(s.xy_valid)), "v_xy_valid": int(bool(s.v_xy_valid)),
            "z_reset_counter": s.z_reset_counter,
            "vz_reset_counter": s.vz_reset_counter,
            "delta_z": round(float(s.delta_z), 4) if _finite(s.delta_z) else "",
            "h_rx_age_ms": round(age * 1000.0, 2) if math.isfinite(age) else "",
            "h_reject": sample_rejection(s, now_mono, cfg) or ""}


@dataclass(frozen=True)
class TakeoffStep:
    throttle: int           # 1700 climb / 1500 hold
    state: str              # TAKEOFF | TAKEOFF_SETTLE | FAULT
    ref: object
    recaptures: int
    status: str
    reason: str


# Reasons for which the takeoff state machine keeps applying the climb throttle.
# Everything else (missing / non-finite / invalid / stale / diverged / reset
# exhausted / non-finite reference) falls back to the neutral 1500.
CLIMB_THROTTLE_REASONS = ("climbing", "descending")


def advance_takeoff(sample, ref, alt, now_mono, elapsed,
                    cfg=DEFAULT_CONFIG, recaptures=0, timeout_s=12.0):
    """One state-machine tick. Shared by sim_pilot and the replay tests.

    The safety branch is explicit: an unconfirmed takeoff must NEVER fall
    through to EXCITE. On timeout or reset exhaustion it returns FAULT.
    """
    d = evaluate_takeoff(sample, ref, alt, now_mono, cfg, recaptures)
    if d.status == "reached":
        return TakeoffStep(1500, "TAKEOFF_SETTLE", d.ref, recaptures,
                           d.status, d.reason)
    if elapsed > timeout_s:
        return TakeoffStep(1500, "FAULT", d.ref, recaptures, "failed",
                           "takeoff_timeout:" + d.reason)
    if d.reason == "reset_recapture_exhausted":
        return TakeoffStep(1500, "FAULT", d.ref, recaptures, "failed", d.reason)
    if d.status == "capture":
        return TakeoffStep(1700, "TAKEOFF", d.ref,
                           recaptures + (1 if d.recapture else 0),
                           d.status, d.reason)
    if d.status == "wait" and d.reason in CLIMB_THROTTLE_REASONS:
        # [NP-01 FIX6b] The climb throttle is driven by FEEDBACK HEALTH, not by
        # the sign of the measured climb. Two flight attempts proved why:
        #   142648: 1700 only on the capture tick, then 1500 (neutral in
        #           Altitude) -> climbed 0.00 m, timeout:climbing.
        #   142907: 1700 only while reason==climbing, but z drifted 5 mm
        #           positive (down) right after arming -> reason==descending ->
        #           1500 -> could never start climbing, timeout:descending.
        # A vehicle on the ground needs throttle to start climbing, so a small
        # negative climb must not withhold it. The sign of the climb still gates
        # "reached" (see evaluate_takeoff); every rejection reason (missing /
        # stale / invalid / diverged / reset / timeout) still falls through to
        # the fail-closed 1500 below.
        return TakeoffStep(1700, "TAKEOFF", d.ref, recaptures,
                           d.status, d.reason)
    return TakeoffStep(1500, "TAKEOFF", d.ref, recaptures, d.status, d.reason)
