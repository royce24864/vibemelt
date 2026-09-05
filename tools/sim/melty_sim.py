#!/usr/bin/env python3
"""
2D melty-brain physics/control simulator for the firmware.

This is a deliberately lightweight simulator. It models the spin limiter,
field-oriented square translation pulses, traction limits, simple motor speed,
and a top-down arena view. The physics are useful for tuning intuition and
algorithm tests, not for predicting exact match performance.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    # The physics model and headless checks do not need GUI dependencies.
    tk = None
    ttk = None


G = 9.80665
FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap360(deg: float) -> float:
    while deg >= 360.0:
        deg -= 360.0
    while deg < 0.0:
        deg += 360.0
    return deg


def angle_diff_deg(a: float, b: float) -> float:
    return (a - b + 540.0) % 360.0 - 180.0


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    theta: float = 0.0
    estimated_theta: float = 0.0
    omega: float = 0.0


@dataclass
class ControlState:
    throttle_percent: float = 0.0
    field_forward: float = 0.0
    field_right: float = 0.0
    drive_heading_phase_deg: float = 0.0
    led_heading_offset_deg: float = 315.0
    rpm_offset: float = 1.0
    calibration_live: bool = False
    reversed_spin: bool = False


@dataclass
class SimOutputs:
    rpm: float = 0.0
    sensor_rpm: float = 0.0
    speed_mps: float = 0.0
    motor_l: float = 0.0
    motor_r: float = 0.0
    spinspeed: float = 0.0
    throttle_offset: float = 0.0
    pulse_active: bool = False
    target_phase_deg: float = 0.0
    phase_deg: float = 0.0
    led_actual_heading_deg: float = 0.0
    led_precession_error_deg: float = 0.0
    led_precession_rate_dps: float = 0.0
    led_visible: bool = False
    applied_rpm_offset: float = 1.0
    traction_limited: bool = False
    contact_quality: float = 1.0
    wheel_lift: bool = False


class MeltyPhysics:
    def __init__(self, params: dict[str, tk.DoubleVar]) -> None:
        self.params = params
        self.state = RobotState()
        self.control = ControlState()
        self.outputs = SimOutputs()
        self.correction_points: list[tuple[float, float]] = []
        self.last_led_precession_error_deg = 0.0
        self.has_led_precession_error = False
        self.sensor_sample_accumulator = 0.0
        self.accel_filter_samples: list[float] = []
        self.filtered_accel_g = 0.0
        self.accel_filter_initialized = False

    def reset_pose(self) -> None:
        self.state = RobotState()
        self.last_led_precession_error_deg = 0.0
        self.has_led_precession_error = False
        self.sensor_sample_accumulator = 0.0
        self.accel_filter_samples.clear()
        self.filtered_accel_g = 0.0
        self.accel_filter_initialized = False

    def _get(self, name: str) -> float:
        return float(self.params[name].get())

    def correction_offset_for_rpm(self, rpm: float, fallback: float) -> float:
        points = sorted(self.correction_points, key=lambda point: point[0])
        if not points:
            return clamp(fallback, 0.8, 1.2)
        if rpm <= points[0][0]:
            return points[0][1]
        if rpm >= points[-1][0]:
            return points[-1][1]

        for i in range(1, len(points)):
            low_rpm, low_offset = points[i - 1]
            high_rpm, high_offset = points[i]
            if rpm <= high_rpm:
                span = max(1.0, high_rpm - low_rpm)
                fraction = (rpm - low_rpm) / span
                return low_offset + ((high_offset - low_offset) * fraction)

        return clamp(fallback, 0.8, 1.2)

    def step(self, dt: float) -> None:
        dt = clamp(dt, 0.001, 0.040)

        mass = self._get("mass_g") / 1000.0
        body_radius = self._get("body_radius_mm") / 1000.0
        wheel_radius = self._get("wheel_diameter_mm") / 2000.0
        motor_kv = self._get("motor_kv")
        battery_v = self._get("battery_v")
        gear_ratio = self._get("gear_ratio")
        motor_stall_torque = self._get("motor_stall_torque_mnm") / 1000.0
        floor_mu = self._get("floor_mu")
        wheel_mu = self._get("wheel_mu")
        acc_rate = self._get("acc_rate")
        percentdecel = self._get("percentdecel")
        translation_scale_factor = clamp(self._get("translation_scale_factor"), 0.0, 1.0)
        spin_response = self._get("spin_response")
        linear_drag = self._get("linear_drag")
        angular_drag = self._get("angular_drag")
        accel_noise_g = self._get("accel_noise_g")
        rpm_sensor_bias_pct = self._get("rpm_sensor_bias_pct")
        accel_radius = max(0.001, self._get("accel_radius_mm") / 1000.0)
        imbalance_offset = self._get("imbalance_offset_mm") / 1000.0
        lift_sensitivity = self._get("lift_sensitivity")

        mu_eff = max(0.01, min(floor_mu, wheel_mu))
        body_radius = max(0.01, body_radius)
        wheel_radius = max(0.002, wheel_radius)
        mass = max(0.02, mass)
        inertia = 0.5 * mass * body_radius * body_radius

        rpm = abs(self.state.omega) * 60.0 / (2.0 * math.pi)
        accel_g = (self.state.omega * self.state.omega * accel_radius) / G
        # Match the firmware estimator: sample at 400 Hz, median-of-three, then
        # apply a 0.4084-alpha exponential low-pass filter. This preserves
        # approximately the old 200 Hz filter's time constant.
        self.sensor_sample_accumulator += dt
        while self.sensor_sample_accumulator >= 0.0025:
            self.sensor_sample_accumulator -= 0.0025
            noisy_accel_g = max(0.0, accel_g + random.gauss(0.0, accel_noise_g))
            self.accel_filter_samples.append(noisy_accel_g)
            self.accel_filter_samples = self.accel_filter_samples[-3:]
            filter_input = (
                sorted(self.accel_filter_samples)[1]
                if len(self.accel_filter_samples) == 3
                else noisy_accel_g
            )
            if self.accel_filter_initialized:
                self.filtered_accel_g = (
                    self.filtered_accel_g * (1.0 - 0.4084)
                ) + (filter_input * 0.4084)
            else:
                self.filtered_accel_g = filter_input
                self.accel_filter_initialized = True

        sensor_omega = math.sqrt(max(0.0, self.filtered_accel_g) * G / accel_radius)
        if self.state.omega < 0.0:
            sensor_omega = -sensor_omega
        sensor_omega *= 1.0 + (rpm_sensor_bias_pct * 0.01)
        sensor_rpm = abs(sensor_omega) * 60.0 / (2.0 * math.pi)

        imbalance_g = (self.state.omega * self.state.omega * imbalance_offset) / G
        contact_drop = imbalance_g * lift_sensitivity * (0.5 + 0.5 * math.cos(self.state.theta))
        contact_quality = clamp(1.0 - contact_drop, 0.0, 1.0)
        wheel_lift = contact_quality < 0.08

        requested = clamp(self.control.throttle_percent, 0.0, 100.0) * 10.0
        spin_limit = acc_rate * sensor_rpm + 200.0
        spinspeed = min(requested, spin_limit)
        if self.control.reversed_spin:
            spinspeed = -spinspeed

        motor_l = spinspeed
        motor_r = spinspeed
        throttle_offset = 0.0
        pulse_active = False
        drive_heading_phase = self.control.drive_heading_phase_deg
        led_heading_phase = wrap360(drive_heading_phase + self.control.led_heading_offset_deg)
        target_phase = drive_heading_phase

        if sensor_rpm > 400.0:
            field_forward = self.control.field_forward
            field_right = self.control.field_right
            magnitude = max(abs(field_forward), abs(field_right))
            if magnitude >= 0.01:
                scaled_magnitude = magnitude * translation_scale_factor
                vector_angle = math.degrees(math.atan2(field_right, field_forward))
                target_phase = wrap360(drive_heading_phase + vector_angle)
                phase = wrap360(math.degrees(self.state.estimated_theta))
                phase_delta = wrap360(target_phase - phase) if self.control.reversed_spin else wrap360(phase - target_phase)
                pulse_width_deg = clamp(percentdecel, 1.0, 50.0) * 3.6
                positive_pulse = phase_delta < pulse_width_deg
                negative_pulse = 180.0 <= phase_delta < 180.0 + pulse_width_deg
                if positive_pulse or negative_pulse:
                    def pulse_command(base: float, accelerate: bool) -> float:
                        if abs(base) < 0.001:
                            return 0.0
                        target = math.copysign(999.0, base) if accelerate else 0.0
                        result = base + ((target - base) * scaled_magnitude)
                        return clamp(result, 0.0, 999.0) if base > 0.0 else clamp(result, -999.0, 0.0)

                    accelerate_right = positive_pulse != self.control.reversed_spin
                    motor_r = pulse_command(spinspeed, accelerate_right)
                    motor_l = pulse_command(spinspeed, not accelerate_right)
                    throttle_offset = (motor_r - motor_l) * 0.5
                    pulse_active = max(abs(motor_r - spinspeed), abs(motor_l - spinspeed)) > 5.0

        avg_cmd = (motor_l + motor_r) * 0.5
        diff_cmd = (motor_r - motor_l) * 0.5
        avg_fraction = clamp(avg_cmd / 1000.0, -1.0, 1.0)
        diff_fraction = clamp(diff_cmd / 1000.0, -1.0, 1.0)

        no_load_motor_rpm = motor_kv * battery_v * abs(avg_fraction)
        no_load_wheel_rpm = no_load_motor_rpm / max(0.01, gear_ratio)
        no_load_surface_mps = no_load_wheel_rpm * (2.0 * math.pi * wheel_radius) / 60.0
        desired_omega = math.copysign(no_load_surface_mps / body_radius, avg_fraction) if abs(avg_fraction) > 0.001 else 0.0

        max_motor_force = (2.0 * motor_stall_torque * gear_ratio) / wheel_radius
        max_traction_force = mu_eff * mass * G * contact_quality
        spin_force_cap = min(max_motor_force, max_traction_force)
        max_alpha = (spin_force_cap * body_radius) / max(0.00001, inertia)
        alpha = clamp((desired_omega - self.state.omega) * spin_response, -max_alpha, max_alpha)
        alpha -= self.state.omega * angular_drag
        self.state.omega += alpha * dt
        live_rpm_offset = clamp(self.control.rpm_offset, 0.8, 1.2)
        applied_rpm_offset = live_rpm_offset if self.control.calibration_live else self.correction_offset_for_rpm(sensor_rpm, live_rpm_offset)
        self.state.estimated_theta = (self.state.estimated_theta + (sensor_omega / applied_rpm_offset) * dt) % (2.0 * math.pi)

        traction_limited = False
        force_mag = 0.0
        if abs(diff_fraction) > 0.001:
            speed_along_body = self.state.vx * math.cos(self.state.theta) + self.state.vy * math.sin(self.state.theta)
            motor_force_available = max_motor_force * abs(diff_fraction)
            speed_limit = motor_kv * battery_v * (2.0 * math.pi * wheel_radius) / (60.0 * max(0.01, gear_ratio))
            speed_factor = clamp(1.0 - abs(speed_along_body) / max(0.1, speed_limit), 0.15, 1.0)
            motor_force_available *= speed_factor
            available_force = min(motor_force_available, max_traction_force)
            traction_limited = motor_force_available > max_traction_force
            force_mag = math.copysign(available_force, diff_fraction)

        ax = (force_mag * math.cos(self.state.theta)) / mass
        ay = (force_mag * math.sin(self.state.theta)) / mass
        ax -= self.state.vx * linear_drag
        ay -= self.state.vy * linear_drag

        self.state.vx += ax * dt
        self.state.vy += ay * dt
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.theta = (self.state.theta + self.state.omega * dt) % (2.0 * math.pi)
        true_phase_deg = wrap360(math.degrees(self.state.theta))
        estimated_phase_deg = wrap360(math.degrees(self.state.estimated_theta))
        led_precession_error_deg = angle_diff_deg(true_phase_deg, estimated_phase_deg)
        if self.has_led_precession_error:
            led_precession_rate_dps = angle_diff_deg(led_precession_error_deg, self.last_led_precession_error_deg) / dt
        else:
            led_precession_rate_dps = 0.0
            self.has_led_precession_error = True
        self.last_led_precession_error_deg = led_precession_error_deg
        led_actual_heading_deg = wrap360(led_heading_phase + led_precession_error_deg)
        led_visible = sensor_rpm > 400.0 and abs(angle_diff_deg(estimated_phase_deg, led_heading_phase)) <= 20.0

        half = self._get("arena_size_ft") * FT_TO_M * 0.5
        if self.state.x > half:
            self.state.x = half
            self.state.vx *= -0.25
        elif self.state.x < -half:
            self.state.x = -half
            self.state.vx *= -0.25
        if self.state.y > half:
            self.state.y = half
            self.state.vy *= -0.25
        elif self.state.y < -half:
            self.state.y = -half
            self.state.vy *= -0.25

        self.outputs = SimOutputs(
            rpm=abs(self.state.omega) * 60.0 / (2.0 * math.pi),
            sensor_rpm=sensor_rpm,
            speed_mps=math.hypot(self.state.vx, self.state.vy),
            motor_l=motor_l,
            motor_r=motor_r,
            spinspeed=spinspeed,
            throttle_offset=throttle_offset,
            pulse_active=pulse_active,
            target_phase_deg=target_phase,
            phase_deg=estimated_phase_deg,
            led_actual_heading_deg=led_actual_heading_deg,
            led_precession_error_deg=led_precession_error_deg,
            led_precession_rate_dps=led_precession_rate_dps,
            led_visible=led_visible,
            applied_rpm_offset=applied_rpm_offset,
            traction_limited=traction_limited,
            contact_quality=contact_quality,
            wheel_lift=wheel_lift,
        )


class MeltySimApp:
    FLOOR_PRESETS = {
        "Rubber mat": 0.90,
        "Carpet": 0.70,
        "Painted wood": 0.45,
        "Dusty polycarb": 0.35,
        "HDPE/UHMW": 0.25,
    }

    PARAMS = {
        "arena_size_ft": (5.0, 4.0, 6.0, 0.1),
        "floor_mu": (0.45, 0.1, 1.4, 0.01),
        "wheel_mu": (1.0, 0.1, 1.6, 0.01),
        "battery_v": (12.0, 6.0, 25.2, 0.1),
        "motor_kv": (2700.0, 500.0, 12000.0, 50.0),
        "motor_stall_torque_mnm": (30.0, 2.0, 200.0, 1.0),
        "gear_ratio": (1.0, 1.0, 8.0, 0.1),
        "wheel_diameter_mm": (38.1, 8.0, 60.0, 0.5),
        "body_radius_mm": (76.2, 35.0, 160.0, 1.0),
        "mass_g": (453.6, 100.0, 1500.0, 10.0),
        "acc_rate": (0.4, 0.05, 1.2, 0.01),
        "percentdecel": (15.0, 1.0, 50.0, 1.0),
        "translation_scale_factor": (1.0, 0.0, 1.0, 0.05),
        "spin_response": (10.0, 1.0, 40.0, 0.5),
        "linear_drag": (0.8, 0.0, 8.0, 0.1),
        "angular_drag": (0.08, 0.0, 1.0, 0.01),
        "accel_noise_g": (0.5, 0.0, 20.0, 0.1),
        "rpm_sensor_bias_pct": (0.0, -10.0, 10.0, 0.1),
        "accel_radius_mm": (13.3862803, 5.0, 80.0, 1.0),
        "imbalance_offset_mm": (0.2, 0.0, 3.0, 0.05),
        "lift_sensitivity": (0.15, 0.0, 0.5, 0.01),
    }

    def __init__(self) -> None:
        if tk is None:
            raise SystemExit("The GUI simulator requires Python with Tkinter installed.")
        self.root = tk.Tk()
        self.root.title("MeltyBrain Physics Simulator")
        self.root.geometry("1220x780")
        self.keys: set[str] = set()
        self.paused = False
        self.trail: list[tuple[float, float]] = []
        self.last_time = time.perf_counter()
        self.status = tk.StringVar()
        self.floor_choice = tk.StringVar(value="Painted wood")
        self.config_mode = False
        self.config_feedback = ""
        self.config_feedback_until = 0.0

        self.params: dict[str, tk.DoubleVar] = {
            name: tk.DoubleVar(value=spec[0]) for name, spec in self.PARAMS.items()
        }
        self.physics = MeltyPhysics(self.params)
        self.correction_points = self.physics.correction_points

        self._build_ui()
        self._bind_keys()
        self._tick()

    def _build_ui(self) -> None:
        root = self.root
        main = ttk.Frame(root, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main, bg="#101820", width=840, height=720, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        panel = ttk.Frame(main, width=340)
        panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        panel.pack_propagate(False)

        ttk.Label(panel, text="Controls", font=("TkDefaultFont", 11, "bold")).pack(anchor=tk.W)
        control_text = (
            "Up/Down: throttle\n"
            "Left/Right: rotate LED heading\n"
            "W/S: forward/back translation\n"
            "A/D: left/right translation\n"
            "K: enter/exit config, V/6: save point\n"
            "Space: throttle zero\n"
            "R: reset robot, C: clear trail\n"
            "P: pause, X: reverse spin"
        )
        ttk.Label(panel, text=control_text, justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 8))

        button_row = ttk.Frame(panel)
        button_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(button_row, text="Reset", command=self._reset_robot).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        ttk.Button(button_row, text="Clear Trail", command=self.trail.clear).pack(side=tk.LEFT, expand=True, fill=tk.X)

        ttk.Label(panel, text="Floor preset", font=("TkDefaultFont", 10, "bold")).pack(anchor=tk.W)
        floor_combo = ttk.Combobox(panel, textvariable=self.floor_choice, values=list(self.FLOOR_PRESETS.keys()), state="readonly")
        floor_combo.pack(fill=tk.X, pady=(2, 8))
        floor_combo.bind("<<ComboboxSelected>>", self._apply_floor_preset)

        ttk.Label(panel, textvariable=self.status, justify=tk.LEFT, font=("TkFixedFont", 10)).pack(anchor=tk.W, fill=tk.X, pady=(0, 10))

        sliders = ttk.Frame(panel)
        sliders.pack(fill=tk.BOTH, expand=True)
        for name, (_, low, high, step) in self.PARAMS.items():
            self._add_slider(sliders, name, low, high, step)

    def _add_slider(self, parent: ttk.Frame, name: str, low: float, high: float, step: float) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        label = ttk.Label(row, text=name, width=23)
        label.pack(side=tk.LEFT)
        value_label = ttk.Label(row, width=8)
        value_label.pack(side=tk.RIGHT)

        def update_label(*_: object) -> None:
            value = self.params[name].get()
            value_label.configure(text=f"{value:.2f}" if step < 1.0 else f"{value:.0f}")

        scale = ttk.Scale(row, from_=low, to=high, variable=self.params[name], orient=tk.HORIZONTAL, command=lambda _v: update_label())
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.params[name].trace_add("write", update_label)
        update_label()

    def _bind_keys(self) -> None:
        self.root.bind("<KeyPress>", self._key_down)
        self.root.bind("<KeyRelease>", self._key_up)

    def _key_down(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.keys.add(key)
        if key == "space":
            self.physics.control.throttle_percent = 0.0
        elif key == "r":
            self._reset_robot()
        elif key == "c":
            self.trail.clear()
        elif key == "p":
            self.paused = not self.paused
        elif key == "x":
            self.physics.control.reversed_spin = not self.physics.control.reversed_spin
        elif key == "k":
            self._toggle_config_mode()
        elif key == "v" or key == "6":
            self._save_config_point()

    def _key_up(self, event: tk.Event) -> None:
        self.keys.discard(event.keysym.lower())

    def _apply_floor_preset(self, _event: object | None = None) -> None:
        self.params["floor_mu"].set(self.FLOOR_PRESETS[self.floor_choice.get()])

    def _reset_robot(self) -> None:
        self.physics.reset_pose()
        self.trail.clear()

    def _show_config_feedback(self, message: str, duration_s: float = 0.8) -> None:
        self.config_feedback = message
        self.config_feedback_until = time.perf_counter() + duration_s

    def _toggle_config_mode(self) -> None:
        if self.config_mode:
            self.config_mode = False
            self.physics.control.calibration_live = False
            self._show_config_feedback(f"FINAL SAVE: {len(self.correction_points)} points", 1.4)
            return

        if self.physics.control.throttle_percent > 5.0:
            self._show_config_feedback("CONFIG BLOCKED: throttle must be low", 1.2)
            return

        self.config_mode = True
        self.physics.control.calibration_live = True
        self.physics.control.rpm_offset = self.physics.correction_offset_for_rpm(
            self.physics.outputs.sensor_rpm,
            self.physics.control.rpm_offset,
        )
        self.physics.state.estimated_theta = self.physics.state.theta
        self._show_config_feedback("ACC ZEROING COMPLETE", 1.0)

    def _save_config_point(self) -> None:
        if not self.config_mode:
            return

        rpm = self.physics.outputs.sensor_rpm
        if rpm > 200.0:
            if len(self.correction_points) >= 8:
                nearest = min(range(8), key=lambda i: abs(self.correction_points[i][0] - rpm))
                self.correction_points[nearest] = (rpm, self.physics.control.rpm_offset)
            else:
                self.correction_points.append((rpm, self.physics.control.rpm_offset))
            self.correction_points.sort(key=lambda point: point[0])
            self._show_config_feedback(f"SAVED POINT {len(self.correction_points)}/8", 0.8)
        else:
            self._show_config_feedback("SAVE IGNORED: RPM TOO LOW", 0.8)

    def _update_controls(self, dt: float) -> None:
        ctrl = self.physics.control
        throttle_rate = 80.0
        if "up" in self.keys:
            ctrl.throttle_percent += throttle_rate * dt
        if "down" in self.keys:
            ctrl.throttle_percent -= throttle_rate * dt
        ctrl.throttle_percent = clamp(ctrl.throttle_percent, 0.0, 100.0)

        yaw_rate_dps = 180.0
        if "left" in self.keys and not self.config_mode:
            ctrl.drive_heading_phase_deg = wrap360(ctrl.drive_heading_phase_deg - yaw_rate_dps * dt)
        if "right" in self.keys and not self.config_mode:
            ctrl.drive_heading_phase_deg = wrap360(ctrl.drive_heading_phase_deg + yaw_rate_dps * dt)

        forward = 0.0
        right = 0.0
        if self.config_mode:
            adjust = 0.0
            if "d" in self.keys:
                adjust += 1.0
            if "a" in self.keys:
                adjust -= 1.0

            if "w" in self.keys:
                forward = 1.0
            elif "s" in self.keys:
                ctrl.led_heading_offset_deg = wrap360(ctrl.led_heading_offset_deg + adjust * 90.0 * dt)
            elif adjust != 0.0:
                ctrl.rpm_offset = clamp(ctrl.rpm_offset + adjust * 0.04 * dt, 0.8, 1.2)
        else:
            if "w" in self.keys:
                forward += 1.0
            if "s" in self.keys:
                forward -= 1.0
            if "d" in self.keys:
                right -= 1.0
            if "a" in self.keys:
                right += 1.0
        ctrl.field_forward = clamp(forward, -1.0, 1.0)
        ctrl.field_right = clamp(right, -1.0, 1.0)

    def _tick(self) -> None:
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now

        self._update_controls(dt)
        if not self.paused:
            substeps = max(1, int(math.ceil(dt / 0.008)))
            sub_dt = dt / substeps
            for _ in range(substeps):
                self.physics.step(sub_dt)
            self.trail.append((self.physics.state.x, self.physics.state.y))
            if len(self.trail) > 1200:
                self.trail = self.trail[-1200:]

        self._draw()
        self._update_status()
        self.root.after(16, self._tick)

    def _world_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        margin = 42
        arena_m = self.params["arena_size_ft"].get() * FT_TO_M
        scale = min((width - 2 * margin) / arena_m, (height - 2 * margin) / arena_m)
        cx = width * 0.5 + x * scale
        cy = height * 0.5 - y * scale
        return cx, cy

    def _draw_arrow(self, x: float, y: float, angle: float, length: float, color: str, width: int = 2) -> None:
        x2 = x + math.cos(angle) * length
        y2 = y - math.sin(angle) * length
        self.canvas.create_line(x, y, x2, y2, fill=color, width=width, arrow=tk.LAST, arrowshape=(12, 14, 5))

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        width = max(1, c.winfo_width())
        height = max(1, c.winfo_height())
        arena_m = self.params["arena_size_ft"].get() * FT_TO_M
        half = arena_m * 0.5
        x0, y0 = self._world_to_canvas(-half, half)
        x1, y1 = self._world_to_canvas(half, -half)
        c.create_rectangle(x0, y0, x1, y1, outline="#87d7ff", width=2)

        for i in range(1, int(self.params["arena_size_ft"].get())):
            pos = -half + i * FT_TO_M
            gx0, gy0 = self._world_to_canvas(pos, -half)
            gx1, gy1 = self._world_to_canvas(pos, half)
            c.create_line(gx0, gy0, gx1, gy1, fill="#243744")
            gx0, gy0 = self._world_to_canvas(-half, pos)
            gx1, gy1 = self._world_to_canvas(half, pos)
            c.create_line(gx0, gy0, gx1, gy1, fill="#243744")

        if len(self.trail) > 1:
            pts: list[float] = []
            for x, y in self.trail:
                cx, cy = self._world_to_canvas(x, y)
                pts.extend((cx, cy))
            c.create_line(*pts, fill="#ffd166", width=2, smooth=True)

        st = self.physics.state
        out = self.physics.outputs
        ctrl = self.physics.control
        rx, ry = self._world_to_canvas(st.x, st.y)
        world_r = self.params["body_radius_mm"].get() / 1000.0
        edge_x, _ = self._world_to_canvas(st.x + world_r, st.y)
        radius_px = max(12.0, abs(edge_x - rx))

        fill = "#22c55e" if out.pulse_active else "#7dd3fc"
        if out.wheel_lift:
            fill = "#ef4444"
        elif out.traction_limited:
            fill = "#f97316"
        c.create_oval(rx - radius_px, ry - radius_px, rx + radius_px, ry + radius_px, outline="#e5e7eb", fill=fill, width=2)
        self._draw_arrow(rx, ry, st.theta, radius_px * 1.2, "#111827", 3)

        led_heading_phase_deg = wrap360(ctrl.drive_heading_phase_deg + ctrl.led_heading_offset_deg)
        led_heading_rad = math.radians(led_heading_phase_deg)
        self._draw_arrow(rx, ry, led_heading_rad, radius_px * 1.75, "#ff4d6d", 3)
        c.create_text(rx + radius_px + 8, ry - radius_px - 8, text="desired LED", fill="#ffb3c1", anchor=tk.W)

        actual_led_rad = math.radians(out.led_actual_heading_deg)
        self._draw_arrow(rx, ry, actual_led_rad, radius_px * 1.45, "#facc15", 3)
        led_dot_x = rx + math.cos(actual_led_rad) * radius_px
        led_dot_y = ry - math.sin(actual_led_rad) * radius_px
        dot_r = 5 if out.led_visible else 3
        dot_fill = "#fde047" if out.led_visible else "#a16207"
        c.create_oval(led_dot_x - dot_r, led_dot_y - dot_r, led_dot_x + dot_r, led_dot_y + dot_r, outline="", fill=dot_fill)
        c.create_text(rx + radius_px + 8, ry + radius_px + 8, text="actual LED", fill="#fde047", anchor=tk.W)

        if abs(ctrl.field_forward) > 0.01 or abs(ctrl.field_right) > 0.01:
            vector_angle = math.atan2(ctrl.field_right, ctrl.field_forward) + math.radians(ctrl.drive_heading_phase_deg)
            self._draw_arrow(rx, ry, vector_angle, radius_px * 2.2, "#a7f3d0", 3)

        c.create_text(16, 16, text="MeltyBrain simulator", fill="#e5e7eb", anchor=tk.NW, font=("TkDefaultFont", 14, "bold"))
        c.create_text(16, 42, text="Red: calibrated LED marker. Yellow: actual precessing LED. Green: commanded translation.", fill="#cbd5e1", anchor=tk.NW)
        if self.config_mode:
            c.create_text(width * 0.5, 32, text="CONFIG MODE", fill="#67e8f9", font=("TkDefaultFont", 16, "bold"))
        if self.config_feedback and time.perf_counter() < self.config_feedback_until:
            c.create_text(width * 0.5, 62, text=self.config_feedback, fill="#fef08a", font=("TkDefaultFont", 14, "bold"))
        if self.paused:
            c.create_text(width * 0.5, 32, text="PAUSED", fill="#fca5a5", font=("TkDefaultFont", 16, "bold"))

    def _update_status(self) -> None:
        out = self.physics.outputs
        st = self.physics.state
        ctrl = self.physics.control
        points = ", ".join(f"{rpm:.0f}@{offset:.3f}" for rpm, offset in self.correction_points)
        if not points:
            points = "none"
        status = (
            f"RPM true/sens:  {out.rpm:7.0f} {out.sensor_rpm:7.0f}\n"
            f"Speed:          {out.speed_mps:7.2f} m/s  {out.speed_mps * M_TO_FT:5.2f} ft/s\n"
            f"Mode:           {'CONFIG' if self.config_mode else 'NORMAL':>7}\n"
            f"Throttle:       {ctrl.throttle_percent:7.1f}%\n"
            f"Motors L/R:     {out.motor_l:7.0f} {out.motor_r:7.0f}\n"
            f"Spin speed cmd: {out.spinspeed:7.0f}\n"
            f"Pulse diff:     {out.throttle_offset:7.0f}\n"
            f"Pulse active:   {str(out.pulse_active):>7}\n"
            f"Traction limit: {str(out.traction_limited):>7}\n"
            f"Contact:        {out.contact_quality:7.2f}\n"
            f"Wheel lift:     {str(out.wheel_lift):>7}\n"
            f"Phase/target:   {out.phase_deg:7.1f} {out.target_phase_deg:7.1f} deg\n"
            f"Drive heading:  {ctrl.drive_heading_phase_deg:7.1f} deg\n"
            f"LED offset:     {ctrl.led_heading_offset_deg:7.1f} deg\n"
            f"LED heading:    {wrap360(ctrl.drive_heading_phase_deg + ctrl.led_heading_offset_deg):7.1f} deg\n"
            f"LED actual:     {out.led_actual_heading_deg:7.1f} deg\n"
            f"LED precess err:{out.led_precession_error_deg:7.1f} deg\n"
            f"LED precess dps:{out.led_precession_rate_dps:7.1f}\n"
            f"LED visible:    {str(out.led_visible):>7}\n"
            f"RPM corr live:  {ctrl.rpm_offset:7.3f}\n"
            f"RPM corr used:  {out.applied_rpm_offset:7.3f}\n"
            f"Saved points:   {len(self.correction_points):7d}/8\n"
            f"Field F/R:      {ctrl.field_forward:7.1f} {ctrl.field_right:7.1f}\n"
            f"Position:       {st.x * M_TO_FT:7.2f} {st.y * M_TO_FT:7.2f} ft\n"
            f"Spin reversed:  {str(ctrl.reversed_spin):>7}\n"
            f"Points: {points}"
        )
        self.status.set(status)

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    MeltySimApp().run()
