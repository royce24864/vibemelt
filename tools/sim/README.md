# MeltyBrain Simulator

Requires Python 3.10 or newer. Run these commands from `tools/sim`:

```bash
python3 melty_sim.py
```

This is a 2D tuning simulator for the melty-brain control algorithm. The interactive window requires Python's Tkinter GUI toolkit. Headless checks do not require Tkinter or any third-party packages.

Headless robustness test:

```bash
python3 robustness_test.py
python3 robustness_test.py --quick
python3 robustness_test.py --target-rpm 2600
```

## Controls

- `Up` / `Down`: increase or decrease spin throttle.
- `Left` / `Right`: rotate the runtime drive-heading frame and its calibrated LED marker.
- `W` / `S`: field forward/back translation command.
- `A` / `D`: field left/right translation command.
- `K`: enter or exit simulated config/calibration mode.
- `V` or `6`: save the current simulated RPM correction point while in config mode.
- `Space`: set throttle to zero.
- `X`: toggle reverse spin.
- `R`: reset robot position and velocity.
- `C`: clear the trail.
- `P`: pause.

## What It Models

- Spinup limiter:

```text
spinspeed = min(throttle_request, acc_rate * rpm + 200)
```

- Field-oriented square translation:

```text
target_phase = drive_heading_phase + atan2(field_right, field_forward)
led_phase = drive_heading_phase + led_heading_offset
positive pulse starts at target_phase
negative pulse starts at target_phase + 180 deg
```

The simulator uses the requested angular width continuously. The controller firmware additionally rounds that width to a matched number of 1 kHz DShot frames for both opposing pulses; this sub-millisecond output quantization is not represented by the lightweight physics model.

- Max-axis translation normalization, matching the firmware:

```text
magnitude = max(abs(field_forward), abs(field_right))
```

- Approximate wheel/floor traction, motor KV, battery voltage, wheel diameter, robot mass, body radius, and motor torque.
- Accelerometer noise and RPM bias. The simulator reports both true RPM and sensor-derived RPM. Sensor acceleration is sampled at 400 Hz and passes through the same median-of-three plus 0.4084-alpha low-pass estimator used by the firmware before phase integration.
- LED precession. The red arrow is the desired LED marker after its independent phase offset. The yellow arrow is where the LED would appear if the estimated spin phase drifts away from the true spin phase.
- Rotating imbalance/contact loss. `imbalance_offset_mm` acts like a small center-of-mass offset; at higher RPM it periodically reduces effective wheel contact. The robot turns red when contact is nearly lost.

## Notes

The physics are intentionally simple. Use this to compare settings and reason about control behavior, not to predict exact match speeds. Real melty translation depends on wheel compliance, ESC behavior, impacts, floor dust, shell balance, and contact geometry.

Useful first things to try:

- Raise/lower `acc_rate` and watch spinup RPM.
- Change `percentdecel` and see how pulse width affects translation smoothness.
- Change `translation_scale_factor` to scale the motor difference from the base throttle without allowing reversal.
- Change translation-stick displacement and watch the motor interpolation versus traction limiting.
- Compare floor presets such as `Rubber mat`, `Dusty polycarb`, and `HDPE/UHMW`.
- Increase `accel_noise_g` to see phase drift/jitter from noisy RPM.
- Change `rpm_sensor_bias_pct` or `RPM corr` in config mode to make the yellow LED heading steadily precess around the red desired heading.
- Increase `imbalance_offset_mm` or `lift_sensitivity` to see periodic contact loss and weaker translation.

## What Causes LED Precession?

In the firmware, the heading LED is turned on using estimated rotation phase. If the estimate runs slightly faster or slower than the real robot, the LED turns on at a slightly different physical angle each revolution. That makes the heading LED appear to slowly rotate, or precess.

Common causes:

- RPM correction is too high or too low.
- Accelerometer radius or calibration is wrong.
- Accelerometer noise creates RPM jitter.
- The robot is bouncing or unbalanced, so acceleration is not clean centripetal acceleration.
- The spinup rate is changing quickly, so a stale RPM estimate lags the real period.

In the simulator:

- `rpm_sensor_bias_pct` creates steady precession.
- `accel_noise_g` creates jittery precession.
- `imbalance_offset_mm` and `lift_sensitivity` can make the RPM signal and contact patch less stable.

## Simulated Calibration Mode

This mirrors the workflow without changing the firmware files:

1. Set throttle to zero.
2. Press `K` to enter config mode. The simulator flashes an accelerometer-zeroing message and synchronizes estimated phase.
3. Spin up with `Up`.
4. With no `W`/`S` input, press `A` or `D` to adjust simulated RPM correction.
5. Press `V` or `6` to save the current RPM/correction point. Up to 8 points are kept and sorted by RPM.
6. Hold `W` to test forward advance.
7. Hold `S` and press `A`/`D` to rotate only the LED heading offset until forward advance agrees with the LED heading. The translation target remains unchanged during this adjustment.
8. Press `K` again to exit config mode and show the final-save feedback.

In config mode, `A`/`D` are interpreted as calibration adjustment unless `W` is held for the advance test.

Saved correction points are active in normal mode. The simulator sorts them by RPM and linearly interpolates between adjacent points, matching the intent of the correction curve. In config mode, the live `RPM corr live` value is used directly so you can tune until `LED precess dps` is near zero, then save that point.

## Robustness Test

`robustness_test.py` runs the same simulator core without the GUI. It:

1. Spins the robot to 8 RPMs across the operating range.
2. Auto-adjusts the same live RPM correction value you tune with `A`/`D`.
3. Saves 8 correction points out of order-safe sorted/interpolated table logic.
4. Runs forward, right, and diagonal translation tests.
5. Prints speed, distance, direction error, true/sensor RPM, precession rate, wheel lift percentage, traction-limit percentage, and average contact quality.

Built-in cases include nominal, slick floor, noisy accelerometer, RPM bias, imbalance/contact loss, and a combined ugly case.

Use this for algorithm robustness checks:

- If `LED precess dps` remains high after calibration, the RPM correction process is not robust enough for that noise/mismatch case.
- If direction error is large while precession is low, the issue is more likely translation pulse timing/LED offset/friction than RPM correction.
- If wheel lift or traction limit percentages are high, the physics assumptions are telling you the floor/contact model is dominating the algorithm.
- If achieved RPM is far below the no-load spin estimate, traction/contact/drag are limiting spinup in the simulated setup.
