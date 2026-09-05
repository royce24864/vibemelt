#!/usr/bin/env python3
"""
Headless robustness test for the MeltyBrain simulator.

This script uses melty_sim.MeltyPhysics, so the tested control path matches the
viewer: the firmware spin limiter, continuous approximations of the leading-edge square pulses, max-axis normalization,
400 Hz noisy accelerometer-derived RPM, interpolated correction points, and LED
precession from phase-estimation error.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass

import melty_sim


DT = 1.0 / 240.0
CAL_TARGET_RPMS = (700, 1000, 1300, 1700, 2100, 2500, 3000, 3500)


class Var:
    def __init__(self, value: float) -> None:
        self.value = value

    def get(self) -> float:
        return self.value

    def set(self, value: float) -> None:
        self.value = value


@dataclass
class CalibrationResult:
    target_rpm: float
    saved_rpm: float
    offset: float
    precess_dps: float
    throttle_percent: float


@dataclass
class TranslationResult:
    label: str
    target_heading_deg: float
    distance_m: float
    avg_speed_mps: float
    direction_error_deg: float
    avg_rpm: float
    avg_sensor_rpm: float
    avg_precess_abs_dps: float
    wheel_lift_pct: float
    traction_limit_pct: float
    avg_contact: float


def make_params(overrides: dict[str, float] | None = None) -> dict[str, Var]:
    params = {
        name: Var(default)
        for name, (default, _low, _high, _step) in melty_sim.MeltySimApp.PARAMS.items()
    }
    for name, value in (overrides or {}).items():
        if name not in params:
            raise KeyError(f"unknown simulator parameter: {name}")
        params[name].set(value)
    return params


def make_physics(overrides: dict[str, float] | None = None) -> melty_sim.MeltyPhysics:
    return melty_sim.MeltyPhysics(make_params(overrides))


def verify_led_offset_is_decoupled() -> None:
    """Changing LED calibration must move the marker, never the motor target."""
    physics = make_physics({"accel_noise_g": 0.0, "rpm_sensor_bias_pct": 0.0})
    control = physics.control
    control.drive_heading_phase_deg = 23.0
    control.led_heading_offset_deg = 315.0
    control.field_forward = 1.0

    physics.step(DT)
    target_before = physics.outputs.target_phase_deg
    led_before = physics.outputs.led_actual_heading_deg

    adjustment_deg = 37.0
    control.led_heading_offset_deg = melty_sim.wrap360(
        control.led_heading_offset_deg + adjustment_deg
    )
    physics.step(DT)
    target_after = physics.outputs.target_phase_deg
    led_after = physics.outputs.led_actual_heading_deg

    target_change = melty_sim.angle_diff_deg(target_after, target_before)
    led_change = melty_sim.angle_diff_deg(led_after, led_before)
    if abs(target_change) > 1e-6:
        raise AssertionError(
            f"LED offset changed translation target by {target_change:.3f} deg"
        )
    if abs(led_change - adjustment_deg) > 1e-6:
        raise AssertionError(
            f"LED offset moved marker by {led_change:.3f} deg, expected {adjustment_deg:.3f}"
        )


def step_for(physics: melty_sim.MeltyPhysics, seconds: float) -> None:
    for _ in range(int(seconds / DT)):
        physics.step(DT)


def hold_target_rpm(physics: melty_sim.MeltyPhysics, target_rpm: float, seconds: float) -> None:
    integral = 0.0
    for _ in range(int(seconds / DT)):
        err = target_rpm - physics.outputs.rpm
        integral = melty_sim.clamp(integral + err * DT, -12000.0, 12000.0)
        throttle = (err * 0.018) + (integral * 0.002)
        physics.control.throttle_percent = melty_sim.clamp(
            physics.control.throttle_percent + throttle * DT,
            0.0,
            100.0,
        )
        physics.step(DT)


def mean(values: list[float]) -> float:
    return sum(values) / max(1, len(values))


def measure_precession_dps(physics: melty_sim.MeltyPhysics, seconds: float = 1.0) -> float:
    last_error: float | None = None
    unwrapped_delta = 0.0
    elapsed = 0.0
    for _ in range(int(seconds / DT)):
        physics.step(DT)
        if physics.outputs.sensor_rpm <= 400.0:
            continue
        current_error = physics.outputs.led_precession_error_deg
        if last_error is not None:
            unwrapped_delta += melty_sim.angle_diff_deg(current_error, last_error)
            elapsed += DT
        last_error = current_error
    if elapsed <= 0.0:
        return 0.0
    return unwrapped_delta / elapsed


def auto_trim_precession(physics: melty_sim.MeltyPhysics, target_rpm: float) -> float:
    physics.control.calibration_live = True
    physics.control.field_forward = 0.0
    physics.control.field_right = 0.0
    physics.state.estimated_theta = physics.state.theta

    # Same knob as config mode: offset is clamped 0.8..1.2 and saved as the
    # correction point. The sign follows phase-rate math:
    # positive precession rate means estimated phase is too slow, so lower offset.
    for _ in range(14):
        hold_target_rpm(physics, target_rpm, 0.65)
        rate = measure_precession_dps(physics, 0.75)
        physics.control.rpm_offset = melty_sim.clamp(
            physics.control.rpm_offset - rate * 0.00006,
            0.8,
            1.2,
        )

    hold_target_rpm(physics, target_rpm, 0.8)
    return measure_precession_dps(physics, 1.5)


def calibrate_8_points(physics: melty_sim.MeltyPhysics) -> list[CalibrationResult]:
    results: list[CalibrationResult] = []
    physics.control.calibration_live = True
    physics.control.rpm_offset = 1.0

    for target_rpm in CAL_TARGET_RPMS:
        precess = auto_trim_precession(physics, target_rpm)
        saved = physics.outputs.sensor_rpm
        offset = physics.control.rpm_offset
        if len(physics.correction_points) >= 8:
            nearest = min(range(8), key=lambda i: abs(physics.correction_points[i][0] - saved))
            physics.correction_points[nearest] = (saved, offset)
        else:
            physics.correction_points.append((saved, offset))
        physics.correction_points.sort(key=lambda point: point[0])
        results.append(CalibrationResult(target_rpm, saved, offset, precess, physics.control.throttle_percent))

    physics.control.calibration_live = False
    physics.control.field_forward = 0.0
    physics.control.field_right = 0.0
    return results


def spin_to_operating_rpm(physics: melty_sim.MeltyPhysics, target_rpm: float) -> None:
    physics.control.field_forward = 0.0
    physics.control.field_right = 0.0
    hold_target_rpm(physics, target_rpm, 5.0)
    hold_target_rpm(physics, target_rpm, 1.0)
    # Compare translation from a common calibrated LED/physical heading after
    # the RPM estimator has settled. Spin-up estimator lag is otherwise a
    # one-time heading offset, not translation direction error.
    physics.state.estimated_theta = physics.state.theta


def run_translation_test(
    physics: melty_sim.MeltyPhysics,
    label: str,
    field_forward: float,
    field_right: float,
    target_rpm: float,
    seconds: float = 2.0,
) -> TranslationResult:
    physics.reset_pose()
    spin_to_operating_rpm(physics, target_rpm)

    physics.control.field_forward = field_forward
    physics.control.field_right = field_right
    start_x = physics.state.x
    start_y = physics.state.y

    rpm_values: list[float] = []
    sensor_rpm_values: list[float] = []
    precession_values: list[float] = []
    contacts: list[float] = []
    lift_count = 0
    traction_count = 0
    samples = int(seconds / DT)

    for _ in range(samples):
        physics.step(DT)
        rpm_values.append(physics.outputs.rpm)
        sensor_rpm_values.append(physics.outputs.sensor_rpm)
        precession_values.append(abs(physics.outputs.led_precession_rate_dps))
        contacts.append(physics.outputs.contact_quality)
        lift_count += 1 if physics.outputs.wheel_lift else 0
        traction_count += 1 if physics.outputs.traction_limited else 0

    dx = physics.state.x - start_x
    dy = physics.state.y - start_y
    distance = math.hypot(dx, dy)
    actual_heading = math.degrees(math.atan2(dy, dx)) if distance > 0.001 else 0.0
    vector_angle = math.degrees(math.atan2(field_right, field_forward))
    target_heading = melty_sim.wrap360(physics.control.drive_heading_phase_deg + vector_angle)
    direction_error = melty_sim.angle_diff_deg(actual_heading, target_heading)

    physics.control.field_forward = 0.0
    physics.control.field_right = 0.0

    return TranslationResult(
        label=label,
        target_heading_deg=target_heading,
        distance_m=distance,
        avg_speed_mps=distance / seconds,
        direction_error_deg=direction_error,
        avg_rpm=mean(rpm_values),
        avg_sensor_rpm=mean(sensor_rpm_values),
        avg_precess_abs_dps=mean(precession_values),
        wheel_lift_pct=100.0 * lift_count / max(1, samples),
        traction_limit_pct=100.0 * traction_count / max(1, samples),
        avg_contact=mean(contacts),
    )


def no_load_spin_rpm(params: dict[str, Var], command: float = 1000.0) -> float:
    motor_kv = params["motor_kv"].get()
    battery_v = params["battery_v"].get()
    gear_ratio = params["gear_ratio"].get()
    wheel_radius = params["wheel_diameter_mm"].get() / 2000.0
    body_radius = params["body_radius_mm"].get() / 1000.0
    motor_rpm = motor_kv * battery_v * (command / 1000.0)
    wheel_rpm = motor_rpm / max(0.01, gear_ratio)
    wheel_surface_mps = wheel_rpm * (2.0 * math.pi * wheel_radius) / 60.0
    omega = wheel_surface_mps / max(0.01, body_radius)
    return omega * 60.0 / (2.0 * math.pi)


def run_case(name: str, overrides: dict[str, float], target_rpm: float) -> None:
    random.seed(7)
    physics = make_physics(overrides)
    params = physics.params
    print(f"\n=== {name} ===")
    print(
        "params: "
        f"floor_mu={params['floor_mu'].get():.2f}, "
        f"wheel_mu={params['wheel_mu'].get():.2f}, "
        f"noise_g={params['accel_noise_g'].get():.2f}, "
        f"rpm_bias_pct={params['rpm_sensor_bias_pct'].get():.2f}, "
        f"imbalance_mm={params['imbalance_offset_mm'].get():.2f}, "
        f"lift_sensitivity={params['lift_sensitivity'].get():.2f}"
    )
    print(f"no-load spin estimate at full command: {no_load_spin_rpm(params):.0f} rpm")

    cal = calibrate_8_points(physics)
    print("calibration points:")
    for result in cal:
        print(
            f"  target={result.target_rpm:4.0f} "
            f"saved={result.saved_rpm:7.0f} "
            f"offset={result.offset:.4f} "
            f"precess={result.precess_dps:8.1f} dps "
            f"thr={result.throttle_percent:5.1f}%"
        )

    tests = (
        ("forward", 1.0, 0.0),
        ("right", 0.0, 1.0),
        ("diag_fwd_right", 1.0, 1.0),
    )
    print("translation tests:")
    for label, forward, right in tests:
        result = run_translation_test(physics, label, forward, right, target_rpm)
        print(
            f"  {result.label:14s} "
            f"speed={result.avg_speed_mps:5.2f} m/s "
            f"dist={result.distance_m:5.2f} m "
            f"dir_err={result.direction_error_deg:7.1f} deg "
            f"rpm={result.avg_rpm:6.0f}/{result.avg_sensor_rpm:6.0f} "
            f"precess={result.avg_precess_abs_dps:7.1f} dps "
            f"lift={result.wheel_lift_pct:5.1f}% "
            f"traction={result.traction_limit_pct:5.1f}% "
            f"contact={result.avg_contact:.2f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run headless MeltyBrain simulator robustness checks.")
    parser.add_argument("--target-rpm", type=float, default=2200.0, help="RPM used for translation tests.")
    parser.add_argument("--quick", action="store_true", help="Run only the nominal case.")
    args = parser.parse_args()

    verify_led_offset_is_decoupled()
    print("LED-offset decoupling check: PASS")

    cases: list[tuple[str, dict[str, float]]] = [
        ("nominal", {}),
        ("slick_floor", {"floor_mu": 0.25, "wheel_mu": 0.75}),
        ("noisy_accel", {"accel_noise_g": 8.0}),
        ("rpm_bias_high", {"rpm_sensor_bias_pct": 3.0}),
        ("unbalanced", {"imbalance_offset_mm": 1.2, "lift_sensitivity": 0.28}),
        ("ugly_combined", {
            "floor_mu": 0.32,
            "wheel_mu": 0.70,
            "accel_noise_g": 5.0,
            "rpm_sensor_bias_pct": 2.0,
            "imbalance_offset_mm": 0.8,
            "lift_sensitivity": 0.22,
        }),
    ]
    if args.quick:
        cases = cases[:1]

    for name, overrides in cases:
        run_case(name, overrides, args.target_rpm)


if __name__ == "__main__":
    main()
