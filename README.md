# Vibemelt

Firmware for the XIAO ESP32-C3 melty brain controller.

This document describes the current firmware code as written. It covers the complete wiring map, the exact receiver signals that select each state, and the estimator/phase algorithm used to turn those inputs into motor commands.

> **Safety:** Test with the weapon mechanically restrained or removed. Config mode can run the motors, and tank mode does not have a separate enable switch.

## Credits

This project builds on the work shared in these repositories. Thanks to their authors and contributors:

- [Beyblade Melty Brain Kit](https://github.com/AriM224/Beyblade-Melty-Brain-Kit) by **AriM224**.
- [PotatoMelt ESP32](https://github.com/skysdottir/potatomelt-esp32) by **skysdottir**.

## Repository Layout

```text
src/main/       Arduino sketch, firmware tabs, and DShot support
libraries/      Bundled Arduino dependencies and their license notices
tools/sim/      Python simulator and robustness checks
```

The sketch entry point is [main.ino](src/main/main.ino). See [Build](#build) for Arduino IDE and command-line setup, or the [simulator README](tools/sim/README.md) for desktop simulation.

## Hardware Connections

### XIAO ESP32-C3 Pin Map

The `D` labels below are printed on the XIAO. GPIO numbers are the ESP32-C3 internal numbers used by the code and some schematics.

| XIAO pin | GPIO | Connect to | Signal |
| --- | ---: | --- | --- |
| `D0 / A0` | 2 | Battery voltage-divider output | Analog battery measurement |
| `D2 / A2` | 4 | Bottom NeoPixel strip DIN | LED data |
| `D3 / A3` | 5 | Top NeoPixel strip DIN | LED data |
| `D4 / SDA` | 6 | H3LIS331DL SDA | I2C data |
| `D5 / SCL` | 7 | H3LIS331DL SCL | I2C clock |
| `D6 / TX` | 21 | Right ESC signal | DShot300 |
| `D7 / RX` | 20 | Receiver iBUS signal | 115200-baud serial input |
| `D8 / SCK` | 8 | Leave unconnected | Internal SPI clock for LED waveform generation |
| `D10 / MOSI` | 10 | Left ESC signal | DShot300 |
| `GND` | — | Receiver, sensor, ESC signal grounds, LEDs, divider | Common reference |
| USB-C | — | Computer | Upload and optional serial debug |

`D1` and `D9` are not assigned. Leave `D8` unconnected; it carries the otherwise-unused SPI clock while the firmware generates NeoPixel data on `D2` and `D3`. The receiver connection is serial iBUS, not SBUS, CRSF, or one-wire PWM. `Serial1` uses receive only, so no receiver transmit wire is required.

### Wiring Overview

```text
iBUS receiver signal ----------------------> D7 / RX
Receiver ground ---------------------------> GND

H3LIS331DL SDA ----------------------------> D4 / SDA
H3LIS331DL SCL ----------------------------> D5 / SCL
H3LIS331DL ground -------------------------> GND
H3LIS331DL power --------------------------> Compatible regulated supply

Right ESC signal <-------------------------- D6 / GPIO21
Right ESC signal ground -------------------> GND
Left ESC signal  <-------------------------- D10 / GPIO10
Left ESC signal ground --------------------> GND

Top NeoPixel DIN <-------------------------- D3 / GPIO5
Bottom NeoPixel DIN <----------------------- D2 / GPIO4
Both NeoPixel grounds ---------------------> GND

Battery + -- upper divider resistor --+----> D0 / A0
                                      |
                              lower divider resistor
                                      |
Battery - ----------------------------+----> GND
```

All signal grounds must be common. Do not power motors, ESC power stages, or the LED-strip load from a GPIO. ESC battery leads and motor phase wires follow the ESC/motor manufacturers' wiring and do not connect to XIAO I/O. Use regulated power compatible with each peripheral and verify its supply and logic-level limits.

The firmware includes the DShotRMT 0.9.5 library source and uses its unmodified ESP-IDF 5 RMT encoder, DShot300 timing, packet construction, command handling, and frame-spacing logic. The firmware only supplies PotatoMelt's AM32 3D throttle mapping: raw zero is stopped input, forward throttle uses raw values 49-1047, and reverse throttle uses 1050-2047. At startup, a short sequence of non-throttle ESC-info frames helps AM32 classify DShot when its input mode is set to AUTO; 1.5 seconds of raw zero then satisfies AM32's arming counter. The controller subsequently sends raw zero while idle and immediately replaces it with mapped throttle when tank, spin-up, melty, or config mode requests motor output. Configure and save 3D motor mode on both AM32 ESCs before use; this is separate from bidirectional DShot telemetry, which this transport does not enable. If one mechanical motor direction is wrong, correct its wiring or change `reverse_1_motor` near the top of `main.ino`; do not interchange the documented left/right signal pins.

Each LED output is configured for ten NeoPixels. If a strip has a different length, change `NUMPIXELST` and/or `NUMPIXELSB` near the top of `main.ino`.

### Accelerometer Location and Orientation

The estimator assumes the accelerometer axes point in the same directions as the robot's physical X/Y axes. Its die is configured at:

- X offset: `8.55 mm`
- Y offset: `12.30 mm`
- Radial distance: approximately `14.9797 mm`

The radial distance is calculated automatically as `sqrt(X * X + Y * Y)` from `ACCEL_X_OFFSET_MM` and `ACCEL_Y_OFFSET_MM`; only those two mounting offsets need to be edited. The code projects measured X/Y acceleration onto that radial direction. Moving or rotating the sensor without changing `ACCEL_X_OFFSET_MM`, `ACCEL_Y_OFFSET_MM`, and any needed axis signs will make the RPM estimate wrong.

The H3LIS331DL must answer at I2C address `0x18`. The firmware runs I2C at 400 kHz and configures the sensor for ±400 g and a 1 kHz output data rate. Startup makes up to five complete probe/configure/verify attempts. Runtime reads reject incomplete six-byte I2C responses, and the 500 ms health check verifies the control registers rather than only probing the address. If the sensor resets or recovers after a bus fault, the firmware reapplies its configuration and resets the RPM filter and phase before resuming phase-sensitive output.

### Battery Divider

Connect only the resistor-divider midpoint to `D0/A0`; never connect battery voltage directly to the ADC. The current conversion is empirical:

```cpp
measured_volts = (analogRead(A0) / 310.0f) * 1.29f;
```

Check the displayed voltage against a multimeter and change this formula to match the actual resistors. `BATTERY_CELL_COUNT` can be fixed to `3` or `4`. Its default value of `0` auto-detects at boot: above 13 V is treated as 4S, otherwise as 3S. Fixing the value is safer when the robot always uses one pack size.

## Channel Map

| Channel | Control | Function |
| --- | --- | --- |
| CH1 | Right stick X | Field left/right translation. In config mode, adjusts RPM correction or the LED-only phase offset according to the active calibration stage. |
| CH2 | Right stick Y | Field forward/back translation. Forward advances the bot in the current drive-heading direction. |
| CH3 | Left stick Y | Spin throttle. Low is off; the configured full-output endpoint becomes the new 100% request. |
| CH4 | Left stick X | Yaw/heading trim. Rotates the drive-heading frame and its calibrated LED marker left or right during normal driving. |
| CH5 | Switch | Spin direction switch during normal operation. |
| CH6 | Switch | WiFi/OTA mode. Must be held high for 1 second to enter. Motors are held off while active. |
| CH7 | Switch | Config/calibration mode. Must be held high for 1 second to enter; low exits and saves. |
| CH8 | Momentary/switch | Hold for tank-drive boost during normal driving. In config mode, retains the calibration capture and multi-press gestures. |

## Receiver Signal Conventions

The receiver must send valid FlySky iBUS frames. Each of all eight channels must remain between 900 and 2100 microseconds. Any channel outside that range invalidates the frame. No valid frame, an invalid frame, or more than 250 ms without a fresh valid frame selects `FAILSAFE`.

The firmware maps nominal 1000-2000 microseconds to an internal duty of 0-100:

| Meaning | Raw duty or normalized spin request | Approximate iBUS value |
| --- | ---: | ---: |
| Stick/switch low | `0` | 1000 µs |
| Center | `50` | 1500 µs |
| Stick/switch high | `100` | 2000 µs |
| Centered stick deadband | `43` through `57` | about 1430-1570 µs |
| Translation becomes active | `42` or lower, or `58` or higher | outside the centered range |
| Normal spin request | Normalized CH3 greater than `10%` | above about 1065 µs at the default endpoint |
| Safe config/WiFi entry throttle | Normalized CH3 `5%` or lower | about 1032 µs or lower at the default endpoint |

CH3 uses `motor_full_scale_percent` as its full-output endpoint. The default is `100 × 1100 / 1700 = 64.70588%`, or about 1647 µs. From that point upward, the normalized spin request is 100%. The firmware uses the retained CH3 pulse width for this calculation rather than rounding it to integer duty first. All CH3 thresholds below, including spin selection above 10%, direction changes below 20%, and low-throttle checks at 5%, refer to this normalized request. Other channel duties, raw CH3 diagnostics, and the transmitter positions saved with RPM calibration points retain their original receiver scale.

Switch “high” normally means duty greater than 50; low means 50 or less. In config mode, CH8 save uses an edge latch: move the switch above 50 to trigger once, then below 40 before it can trigger again.

The current CH2 sign convention calculates field-forward directly from the centered CH2 value. With the configured EdgeTX direction, CH2 `+100` maps to duty 100 and forward, while CH2 `-100` maps to duty 0 and backward.

## State Machine and Selection Priority

The firmware evaluates the state conditions in the following strict order. The first matching row wins:

| Priority | State | Required signal/condition | Motor result |
| ---: | --- | --- | --- |
| 1 | `WIFI` | WiFi mode has completed its entry hold | Explicit zero DShot |
| 2 | `LOW_BATTERY` | Critical battery shutdown has latched | Explicit zero DShot until reboot |
| 3 | `SENSOR_FAULT` | Accelerometer health gate has failed | Explicit zero DShot |
| 4 | `FAILSAFE` | iBUS data is invalid or more than 250 ms old | Explicit zero DShot |
| 5 | `CONFIG` | Config mode has completed its entry hold | Config controls may run motors |
| 6 | `MELTY` | Estimated RPM greater than 400 and CH3 greater than 10 | Spin, with translation if CH1/CH2 request it |
| 7 | `SPINUP` | CH3 greater than 10 but RPM is not above 400 | Open-loop spin-up |
| 8 | `TANK` | CH3 is 10 or lower and CH1 or CH2 is outside the deadband | Direct differential drive |
| 9 | `ARMED_IDLE` | Healthy RC with no movement requested | Armed, zero throttle |

This gives the following normal-operation truth table:

| Normalized CH3 (%) | Estimated RPM | CH1/CH2 | Result |
| --- | ---: | --- | --- |
| `<= 10` | Any | Both centered | `ARMED_IDLE` |
| `<= 10` | Any | Either displaced | `TANK` |
| `> 10` | `<= 400` | Any | `SPINUP` |
| `> 10` | `> 400` | Both centered | `MELTY`, pure spin |
| `> 10` | `> 400` | Either displaced | `MELTY`, field-oriented translation |

The 2 kHz output task independently checks the safety gates again. Nonzero output is permitted only in `TANK`, `SPINUP`, `MELTY`, or `CONFIG`, with fresh valid RC, a healthy accelerometer, no critical battery latch, and WiFi inactive.

### `ARMED_IDLE`

With all safety gates healthy, CH3 at 10 or below, and CH1/CH2 centered, the robot is armed and idle. Both ESCs continue receiving zero-throttle packets at 1 kHz, so tank or spin commands can begin immediately. Hard safety states keep sending stopped packets and block all nonzero motor demand.

### `TANK`

There is no tank-mode switch. Tank mode activates whenever:

- Config and WiFi modes are not active.
- All RC, sensor, and battery safety gates pass.
- CH3 is at 10 or below.
- CH1 or CH2 is outside its 8% center deadband.

The tank mixer uses these command ranges, with no percentage output scaling:

| CH8 in tank drive | CH2 forward/reverse range | CH1 steering range |
| --- | ---: | ---: |
| Released | ±100 | ±20 |
| Held high | ±999 | ±200 |

Outside calibration, CH8 is a momentary hold-to-boost control for tank drive. Its held state is sampled every 20 ms (50 Hz); releasing it restores the ordinary ranges on the next sample. Forward/reverse and steering contributions are combined and clamped to ±999. Ordinary tank output can reach ±120 when both contributions add; full boosted output can reach physical ±999. Both ordinary and boosted tank commands bypass `motor_full_scale_percent`, including at its default 64.71% setting. In config mode, CH8 keeps its existing correction capture, stage-advance, and clear-table gestures.

The state check does not require measured RPM to be zero. If the shell is still coasting and CH3 is low, moving CH1/CH2 can therefore select tank mode.

### `SPINUP`

A normalized spin request greater than 10% selects spin-up until estimated RPM exceeds 400. CH1/CH2 do not create melty pulses in this state. The base motor demand follows CH3 but is limited by:

```cpp
growth = pow(max(measured_rpm, spin_ramp_start_rpm), 1.2)
       - pow(spin_ramp_start_rpm, 1.2);
allowed_command = int(constrain(200 + acc_rate * growth, 200, 999));
```

The start-command ceiling is fixed at logical 200 and the exponent is fixed at 1.2. `spin_ramp_start_rpm` sets the knee where the ceiling begins rising above that level; its default is approximately `1003.9606 RPM`, adjustable from 0 to 5000 RPM. `acc_rate` independently sets the growth coefficient above the knee, from `0.01` to `1.00`, default `0.05`. Changing this coefficient changes the curve's steepness without moving the selected knee. Changing the knee shifts where the rise starts without changing the coefficient. The default pair reproduces the former `max(200, 0.05 × RPM^1.2)` curve before the 999 cap and integer truncation.

Requested throttle can always select a command below the ceiling, including below 200. The same base-command ceiling applies in both spin directions and while translating; translation pulses can still accelerate one motor toward full logical command. Logical commands are clamped to ±999 before the final physical output scaling. This is a command ceiling versus measured body RPM, not a prediction of acceleration over time; actual acceleration also depends on the motors, ESCs, battery, traction, and load.

`motor_full_scale_percent`, shown as **Full-output throttle (%)** in the WiFi configurator, has two linked effects: it sets the raw CH3 position that becomes a 100% logical spin request, and it sets the physical output ceiling for spin, translation, and config-mode commands. Its range is `1–100%`, with a default of `64.70588%`. Their command flow is:

```text
Raw CH3 position / configured endpoint -> logical spin request, 0..999
Logical request -> RPM acceleration ceiling -> spin/translation mixing
Final mixed command × configured percent / 100 -> rounded physical command -> DShot
```

All mixing retains the logical ±999 range. Final spin, translation, and config-mode commands are scaled exactly once; tank commands retain the ±999 clamp and bypass that scaling. At a setting of 50%, raw CH3 at 50% requests logical 999; when the RPM ceiling permits it, the final spin command is 500. Full translation interpolates to logical 999/0, which becomes physical 500/0. At the default endpoint, full logical spin or translation output becomes physical 646 after rounding. Reaching the configured CH3 endpoint does not bypass the acceleration ceiling: low-RPM spin-up can still command less than the physical maximum.

The default ratio uses 1100 KV / 1700 KV on the same 3S supply as an ideal voltage/speed scale. It does not promise equal measured motor RPM, body RPM, motor differential, or translation force. The setting leaves measured body-RPM filtering, phase timing, pulse width, and calibration values unchanged.

CH5 chooses spin direction only while the normalized spin request is below 20%. CH5 above 50 selects reverse and below 50 selects normal. At exactly 50, the prior direction remains latched. Once the normalized spin request reaches 20% or higher, changing CH5 has no immediate effect.

### `MELTY`

Melty state requires a normalized spin request greater than 10% and estimated RPM greater than 400.

- With CH1 and CH2 centered, both motors receive the normal spin command.
- With either stick outside the deadband, the firmware computes a field vector and applies two phase-timed motor-differential pulses per revolution.

The drive-heading frame can be rotated manually with CH4. Full CH4 deflection changes it at `heading_yaw_rate_dps`, currently 180 degrees per second. The displayed marker follows that frame plus the separately calibrated LED phase offset.

Reverse spin applies the drivetrain's motor-phase-only inverse coordinate transform: motor forward is calculated from requested right, and motor right from requested forward. This crossed-axis mapping corrects the empirically observed reverse-spin orientation while leaving the displayed translation vector based on the original stick request.

## Normal Driving

1. Start with CH3 low, CH1/CH2/CH4 centered, and CH6/CH7 low.
2. Select spin direction with CH5 while the normalized spin request is below 20%.
3. Raise CH3. The robot enters `SPINUP`, then automatically changes to `MELTY` above 400 estimated RPM.
4. Leave CH1/CH2 centered for pure spin.
5. Move CH2 forward/back or CH1 left/right for field-oriented translation.
6. Use CH4 to rotate the commanded field heading during normal driving.
7. Lower CH3 and recenter CH1/CH2 to return to `ARMED_IDLE`. Moving a translation stick with CH3 low selects `TANK`.

## Config Mode

Enter config mode by holding CH7 high for 1 second. Entry is allowed only when:

- RC is valid and fresh.
- CH3 is at 5 or lower, not merely below the normal spin threshold of 10.
- CH1, CH2, and CH4 are inside their centered deadbands.
- Both requested motor commands are zero and `motor_on` is false.

All conditions must remain true throughout the one-second CH7 hold. If one fails, the hold timer resets. Exiting config mode is CH7 at 50 or lower; exiting saves all current settings.

Config entry intentionally does not use estimated RPM or accelerometer stationary detection. Before the first successful accelerometer zero, sensor offset can make a stationary robot report a high RPM and otherwise prevent the calibration needed to correct it. The operator must keep the robot physically still while entering config mode and during the automatic zero operation.

Config always progresses in this order: accelerometer zero, RPM-table calibration, then heading-offset calibration. The complete control map is:

| Signal while CH7 remains high | Result |
| --- | --- |
| CH3 `<= 10` | Motors commanded off |
| CH3 `> 10` | Spin motors |
| CH1 during RPM calibration | Adjust the live RPM/phase correction factor from 0.8 to 1.2 |
| CH8 transition above 50 during RPM calibration | Capture a stable RPM correction point in RAM |
| Three or four distinct CH8 presses within three seconds | After the three-second sequence window closes, advance from RPM calibration to heading-offset calibration |
| Five distinct CH8 presses within three seconds | Clear the staged RPM correction table and reset the live RPM correction factor to 1.0; preserve the LED heading offset |
| CH1 during heading calibration | Rotate only the LED phase offset; the forward motor-pulse phase stays fixed for comparison |
| CH2 forward during heading calibration with CH3 `> 10` | Enable the explicit forward translation-direction test; keep CH1 centered during the test |
| CH8 below 40 | Rearm the CH8 save latch |
| CH7 `<= 50` | Commit all staged values to NVS and leave config mode |

CH8 must return below 40 between each press in either multi-press gesture. The three-press action waits until the sequence window closes so a fourth and fifth press can still select the clear action. CH8 does not write NVS in either stage. Accelerometer offsets, captured RPM points, RPM-table clears, and heading changes stay in RAM until CH7 is lowered; that exit performs the calibration session's only persistent save.

### Accelerometer Zero

When config mode starts, the firmware stops both requested motor commands, takes 100 accelerometer samples separated by 2 ms, and calculates the X/Y zero offsets before entering RPM calibration. The new offsets remain staged in RAM until config exit. It then resets the estimator and phase accumulator. Z is not zeroed, loaded, or saved; its raw gravity reading remains available for orientation and inversion detection. Keep the robot completely still during this roughly 200 ms operation.

LED feedback:

- Pulsing blue: accelerometer is zeroing.
- Solid teal/green: zero complete.
- Solid amber with the throttle low: RPM calibration stage is active.

### RPM Correction Table

The firmware stores up to 8 RPM correction points. Each point contains measured RPM, phase correction offset, and the CH3 radio position that produced it. CH3 duty `0` through `100` is stored on the EdgeTX-style integer scale `-100` through `100`; this value represents stick position, not motor direction or the RPM-limited motor command. Points can be saved in any order; all three values remain paired when the table is sorted.

To save a correction point:

1. Enter config mode with CH7.
2. Spin the bot to a stable RPM.
3. Use CH1 left/right to adjust the RPM correction until the heading LED does not precess. The RPM-stage response is rescaled after the deadband for fine control near center while retaining the full-stick adjustment rate.
4. Press CH8 to capture the current RPM, correction value, and signed throttle level in RAM.
5. Repeat at the required RPM values.
6. Press CH8 three times within three seconds to enter heading-offset calibration. The stage advances when the three-second sequence window closes. Pressing CH8 five times within that window clears the RPM table and resets the live correction factor instead; it does not clear the LED heading offset.

The RPM must be above 400 and stable for at least 750 ms before CH8 will accept a correction point. “Stable” means the RPM-change EWMA is no greater than the larger of 12 RPM or 1% of current RPM, with a sensor sample less than 50 ms old. Pulsing orange means a correction capture above 200 RPM was rejected because RPM was still changing. A new point within 75 RPM of an existing point updates that point instead of consuming another slot.

No RPM-stage CH8 press writes NVS. Lowering CH7 after heading calibration commits the complete staged session.

When all eight correction slots are full, another accepted capture replaces the nearest-RPM point. After re-sorting the table, the matching LED index on both strips shows white for the 600 ms save-feedback interval; the other seven LEDs remain green.

If all 8 slots are full, capturing another point replaces the closest existing RPM point.

LED feedback:

- Green pixel count on both strips: number of saved RPM correction points.
- White marker: next available slot.
- The heading strip is orange while RPM is stable.
- Outside the heading strip, red means RPM is still settling and solid green means RPM has passed the 750 ms stability check and is ready for CH8.

### LED Heading Offset

After completing RPM calibration:

1. Press CH8 three times within three seconds, then wait for the sequence window to close, to enter heading-offset calibration.
2. Use CH1 left/right to rotate the LED heading offset.
3. Push CH2 forward to test whether the bot advances in the LED direction.
4. Recenter CH2 and repeat the adjustment/test as needed.
5. Lower CH7 to commit accelerometer zero, RPM table, and heading offset together.

The translation test requires CH3 above 10 and CH2 above the forward deadband. While CH2 is forward, the LEDs show the same two dark-red phase windows used to display the translation vector during normal MELTY operation. Each displayed window is centered on its requested field phase, so the applicable top or upside-down heading marker bisects the forward window and retains color priority on that strip. CH1 always changes only the LED phase offset in this stage, while forward translation remains referenced to the unchanged drive phase. This separation is what lets the marker be aligned to actual movement. Keep CH1 centered during each forward test. The staged RPM table is used for phase correction during heading calibration. Heading-offset adjustment itself does not require stable RPM, although testing it at a representative stable speed is easier to interpret. With the throttle low, solid cyan identifies the heading stage.

LED feedback for final save:

- Yellow background on both strips.
- Green pixel count on both strips shows saved RPM correction points.
- White marker shows the next slot if fewer than 8 points are saved.

## Translation Tuning Variables

Most translation tuning lives near the top of `main.ino`, with the phase logic in `Motor_Control.ino` and RPM/heading timing in `Heading.ino`.

### Main Knobs

| Variable | Default | Where | What it affects |
| --- | --- | --- | --- |
| `motor_full_scale_percent` | `64.70588%` | NVS/WiFi config | Raw CH3 endpoint for a full logical request and final physical ceiling for spin, translation, and config output; adjustable from 1 to 100%. Tank output bypasses it. |
| `spin_ramp_start_rpm` | `1003.9606` | NVS/WiFi config | Knee where the fixed logical-200 ceiling starts rising; adjustable from 0 to 5000 RPM independently of the growth coefficient. |
| `acc_rate` | `0.05` | NVS/WiFi config | Growth coefficient above the selected spin-ramp knee; adjustable from 0.01 to 1.00 without moving the knee. |
| `heading_yaw_rate_dps` | `180.0` | `main.ino` | How quickly CH4 rotates the runtime drive-heading frame during normal driving. |
| `config_rpm_offset_rate` | `0.04` | `main.ino` | Maximum rate at which CH1 changes RPM correction; the RPM-stage stick response ramps smoothly from the deadband edge. |
| `config_led_offset_rate_dps` | `90.0` | `main.ino` | How quickly CH1 changes LED offset while config mode CH2 is pulled back. |
| `stick_deadzone_percent` | `8` | `main.ino` | Stick deadband around center. |
| `percentdecel` | `25` | `main.ino` | Live angular width of each opposing translation pulse. |
| `TRANSLATION_LED_ARC_WIDTH_DEG` | `54.0` | `main.ino` | Total width of each centered translation-direction LED arc in MELTY and heading calibration; independent of motor pulse width. |
| `translation_scale_factor` | `1.0` | NVS/WiFi config | Adjustable from 0.0 to 2.0: 0.0 disables the differential, 0.5 halves it, 1.0 keeps the full result, and values above 1.0 amplify it within the motor command limits. |
| `led_heading_offset_deg` | `315` | `main.ino` | Runtime LED-only phase offset. It starts at this default and is replaced by a valid saved NVS value at boot. |
| `top_led_phase_offset_deg` | `0` | NVS/WiFi config | Persistent display trim added only to the top heading marker after saved calibration; adjustable from -180 to +180 degrees. |
| `bottom_led_phase_offset_deg` | `0` | NVS/WiFi config | Persistent display trim added only to the bottom heading marker after its normal 180-degree underside orientation; adjustable from -180 to +180 degrees. |
| `ACCEL_X_OFFSET_MM` / `ACCEL_Y_OFFSET_MM` | `8.55` / `12.30` | `main.ino` | Accelerometer die coordinates relative to the rotation center. |
| `ACCEL_RADIUS_MM` | `14.9797` | `main.ino` | Radial distance derived from the X/Y mounting coordinates and used for RPM calculation. |
| `rpm_g_per_rpm2_per_mm` | `0.000001118` | `main.ino` | Centripetal acceleration conversion constant for radius in millimeters. |
| RPM correction table | 8 points | NVS/config mode | Corrects heading timing across different RPMs. |

### Symptom-Based Tuning

If translation is too weak or the bot barely walks:

- Increase `translation_scale_factor` within its `0.0–2.0` range or use more translation-stick displacement. Values above `1.0` amplify the pulse difference until a motor command reaches its limit.
- Increase `percentdecel` slightly to widen the square pulse.
- Make sure RPM correction points are saved across the RPM range you actually drive.
- Check that the bot has enough spin speed before expecting strong translation.

If translation is too violent, twitchy, or destabilizes spin:

- Reduce `translation_scale_factor` or translation-stick weight/rate in EdgeTX.
- Decrease `percentdecel` to shorten each square pulse.
- Increase `stick_deadzone_percent` if tiny stick inputs cause movement.
- Lower `acc_rate` if spin-up surges too hard.

If the bot translates in the wrong direction relative to the LED:

- First use config mode LED offset calibration.
- If right-stick-forward moves backward, invert CH2 in the transmitter or flip the sign of `field_forward` in `Motor_Control.ino`.
- If right-stick-left/right is reversed, invert CH1 in the transmitter or flip the sign of `field_right` in `Motor_Control.ino`.

If the heading LED slowly precesses at one RPM but not another:

- Add or replace RPM correction points near the RPM where drift happens.
- During the RPM stage, use CH1 until the heading LED stops precessing, then press CH8 to capture the point in RAM.
- Save points at low, mid, and high driving RPMs rather than all clustered together.

If the heading LED is stable but the bot advances at an angle:

- Triple-press CH8 to enter heading calibration, adjust with CH1, push CH2 forward to test, then exit with CH7 low.
- `led_heading_offset_deg` starts at its compiled default. Once NVS has a saved LED heading offset, config mode is the normal way to tune it.

If manual heading trim feels too slow or too fast:

- Increase `heading_yaw_rate_dps` for faster CH4 heading rotation.
- Decrease it for finer CH4 control.

If calibration adjustments are too coarse:

- Lower `config_rpm_offset_rate` for finer RPM correction adjustment.
- Lower `config_led_offset_rate_dps` for finer LED offset adjustment.

If RPM looks wrong:

- Verify `ACCEL_X_OFFSET_MM` and `ACCEL_Y_OFFSET_MM` locate the accelerometer die, not the breakout-board edge, relative to the real rotation center.
- The RPM formula expects millimeters. The conversion constant is the millimeter version of the centripetal acceleration formula.
- `get_accel_force_g()` projects zeroed X/Y acceleration onto the configured radial mounting vector. The accelerometer axes must have the same orientation as the robot X/Y axes.
- Re-enter config mode while the bot is still so accelerometer zero is refreshed.

### Translation Algorithm Notes

`translate_field_oriented()` first turns the centered sticks into a desired field vector:

```cpp
field_forward =  centered_CH2 / 50.0;
field_right   = -centered_CH1 / 50.0; // current transmitter polarity
magnitude     = max(abs(field_forward), abs(field_right));
vector_angle  = atan2(field_right, field_forward);
target_phase  = drive_heading_phase_deg + vector_angle;
led_phase     = drive_heading_phase_deg + led_heading_offset_deg;
```

Max-axis magnitude preserves full cardinal-stick authority without the diagonal magnitude exceeding 1.0. The requested direction is relative to the runtime drive-heading phase, not an external compass. CH4 rotates that drive frame. The saved LED offset affects only the displayed marker, so heading calibration can change the marker-to-motion alignment without moving the motor pulses at the same time.

The fixed-rate motor task compares the target phase with the accumulated shell phase. It applies opposite motor differentials in two square windows 180 degrees apart:

```cpp
pulse_width_deg = percentdecel * 3.6;
scaled_change   = stick_magnitude * translation_scale_factor;
slow_output     = base_spin * (1 - scaled_change);
fast_output     = base_spin + (same_sign_limit - base_spin) * scaled_change;
```

At the default `percentdecel = 25`, each live angular window is 90 degrees wide. The windows begin at the field-vector target phase and target phase + 180 degrees. While phase remains inside a window, the mapping moves one motor toward full command and the other toward zero, scaled by `translation_scale_factor` (range `0.0–2.0`, default `1.0`). Outside both windows, both motors immediately return to the base spin command. Phase is evaluated at 2 kHz and the resulting commands are transmitted at the existing 1 kHz DShot rate.

The dark-red translation-direction LEDs use separate fixed 54-degree arcs: ±27 degrees centered on the requested direction and its opposite. `TRANSLATION_LED_ARC_WIDTH_DEG` sets this display width in both MELTY operation and heading calibration. Changing `percentdecel` changes only the motor pulses; their default remains 25% of a rotation, or 90 degrees per pulse.

Translation pulses require estimated RPM above 400 even if the main loop has requested translation. At full stick with the default scale factor of `1.0`, the interpolation commands one motor to logical zero and the other to `999` in normal spin or `-999` in reversed spin. The final output scaling turns these into physical 0/646 at the default `motor_full_scale_percent`, or 0/500 at a 50% setting. Half a revolution later, the motors swap roles. Partial stick interpolates each motor between its logical base command and those endpoints before the single final scaling step.

An explicit sign-preservation guard clamps every translated output between zero and the full-scale endpoint matching its base command. Translation can slow a motor to zero but cannot command it into reverse. `translation_scale_factor` multiplies each motor's difference from base after stick scaling, while `percentdecel` controls pulse duration. Values above `1.0` amplify that difference, subject to the same command clamps. For example, if the unscaled pulse changes a base of 600 into 800/400, a factor of 0.5 produces 700/500, while a factor of 2.0 produces 999/200 after clamping.

### RPM Estimator

Every 2.5 ms (400 Hz), the firmware:

1. Reads the H3LIS331DL X/Y axes.
2. Subtracts the stored stationary zero offsets.
3. Projects X/Y acceleration onto the configured radial mounting vector.
4. Takes the absolute radial acceleration.
5. Rejects non-finite samples.
6. Applies a median-of-three filter followed by an EWMA with alpha `0.4084`.
7. Converts filtered acceleration to RPM:

```cpp
radial_g = abs(x_g * radial_unit_x + y_g * radial_unit_y);
rpm = sqrt(radial_g) /
      sqrt(rpm_g_per_rpm2_per_mm * ACCEL_RADIUS_MM);
```

Radial projection rejects much of the tangential acceleration produced during spin-up. Using total X/Y vector magnitude here would bias RPM upward whenever angular speed is changing.

The estimator also tracks an RPM-delta EWMA with alpha `0.1056`. That value drives the 750 ms “stable point” gate used when saving correction points.

### Phase Accumulator and Correction

The motor task is released every 500 microseconds (2 kHz). It integrates elapsed time rather than assuming every wake-up is exact:

```cpp
phase += dt_seconds * (estimated_rpm / rotation_offset) * 6.0;
```

One RPM equals six degrees per second. Below 200 RPM, phase resets to zero because a useful spinning reference is not established.

Heading-window LED entry/exit decisions run on this same 2 kHz phase clock. They publish a coalesced latest-state notification to an LED task at the Arduino loop's priority. The LED task yields after every blocking SPI frame, so it cannot accumulate obsolete phase frames or continuously preempt RPM, RC, and motor-snapshot updates. At RPM where the strips cannot physically render every edge, visual frames may be skipped; current motor timing takes priority over replaying stale LED transitions. Ordinary animations and non-phase status displays remain on the main loop.

`rotation_offset` is the correction factor calibrated in config mode. Outside config mode, the saved RPM table is linearly interpolated between adjacent points. Below or above the saved range, the firmware uses the nearest endpoint. With no table, it uses the single live/saved offset. Increasing the offset slows the predicted phase; decreasing it speeds the predicted phase.

This is an open-loop heading estimate based on measured speed. There is no magnetometer, optical index, or external heading sensor, so small RPM/geometry errors accumulate as heading precession. CH4 trim and the RPM correction table compensate for that drift.

### Current Physical and Timing Bounds

- The accelerometer range is ±400 g per axis. With the current 8.55/12.30 mm placement, the Y component reaches mathematical full scale at about 5,393 RPM. The firmware does not stop or gate the motors when an axis clips; the reported RPM instead plateaus or becomes inaccurate, so heading and translation timing are no longer trustworthy above the measurement range.
- At 6,000 RPM the shell would turn 100 times per second. The 2 kHz phase task provides about 20 phase decisions per revolution, while the 1 kHz DShot stream publishes about 10 motor updates per revolution.
- The 400 Hz estimator provides about four new accelerometer estimates per revolution at 6,000 RPM. The 2 kHz phase task continues between samples using the latest filtered RPM.
- Logical motor commands are limited to `-999` through `999`. For spin, translation, and config output, the final physical ceiling is `round(999 × motor_full_scale_percent / 100)`, currently 646 by default. Tank output retains the full ±999 limit. Translation interpolation stays between zero and the base command's same-direction logical full-scale endpoint before scaling.
- Firmware timing cannot overcome mechanical limits such as motor torque, wheel slip, shell inertia, battery sag, ESC response, accelerometer clipping, or vibration. Verify measured RPM and translation behavior before increasing speed.

## WiFi Mode

Holding CH6 above 50 continuously for 1 second enters WiFi mode. Entry uses the safe-control gate from config mode plus the RPM check in `motors_are_off()`:

- RC valid and fresh.
- CH3 at 5 or lower.
- CH1, CH2, and CH4 centered.
- Requested motors off, `motor_on` false, and RPM below 250.

The one-second timer resets if any condition fails. Config mode has priority, so WiFi cannot be entered while config mode is active. Motors are forced off while WiFi mode is active.

Entering WiFi mode freezes the rolling telemetry and event buffers before the access point starts. The bot then creates a local-only WiFi access point:

- SSID: `vibemelt`
- Password: `vibemelt`
- OTA hostname: `esp32-ap`
- OTA password: none

Connect a phone to `vibemelt`, then open `http://vibe.melt/`. The AP uses the fixed address `192.168.4.1`, disables WiFi power-save latency, and retries startup up to three times. Captive wildcard DNS resolves every hostname to the controller so browser reloads and phone connectivity checks return to the dashboard; `http://192.168.4.1/` remains the direct fallback. The dashboard and all API/download routes exist only while WiFi mode is active.

CH6 at 50 or lower exits WiFi mode, stops HTTP, and shuts off the access point, but only after a valid receiver frame explicitly reports that switch position. Network responses service the manually polled iBus receiver between chunks. An abandoned synchronous browser response can still outlast the normal 250 ms RC timeout, so temporary receiver-parser starvation no longer shuts down the AP. Motor output remains independently hard-disabled throughout WiFi mode. The controller checks AP health once per second, automatically restarts lost AP/HTTP services, and retries a failed startup every two seconds. A fixed-rate motor task continues sending explicit DShot stop packets throughout WiFi mode. Logging resumes after leaving WiFi; fixed schedule slots elapsed while the buffer was frozen are marked missed.

The one-second `loopTask` watchdog is temporarily unregistered while synchronous HTTP/OTA servicing is active, because a client request can legitimately wait longer than one second. It is registered again after HTTP and the AP stop. This does not enable motor output: the independent motor task continues its WiFi safety gate and explicit stop packets throughout the diagnostic session.

### Session logger and downloads

The RAM-only session logger starts at controller power-on and records at 10 Hz. It stores 3,000 fixed 16-byte telemetry rows, providing a five-minute circular FIFO. Once full, each new row replaces exactly the oldest row. Power cycling clears all telemetry and the separate 16 KB circular event log.

Each timeline row contains the power-on-relative timestamp, state, aligned filtered RPM/g-force, battery, normalized CH1/CH2/throttle, requested spin percent, post-limiter base spin command, derived acceleration ceiling, interval-averaged final left/right motor output, RPM variation, and validity/limiter/safety/translation flags.

`limit_flags` is a hexadecimal bit mask: bit 0 accelerometer valid, bit 1 battery valid, bit 2 RC valid, bit 3 missed/invalid/overflow, bit 4 acceleration limiter active, bit 5 motor safety gate closed, bit 6 translation pulse seen during the 100 ms interval, and bit 7 sensor fault active.

Local routes:

- `/` phone dashboard and paginated timeline
- `/api/live.json` current controller/WiFi status
- `/api/summary.json` since-boot and retained-window summaries
- `/api/timeline.json?start=0&count=200` paginated rows
- `/api/timeline.csv` complete retained FIFO download
- `/api/melty.csv` MELTY-only subset
- `/api/events.txt` captured event/debug messages, including explicit accelerometer `LOSS` and `RECOVERY` records with cause, sample age/RPM at loss, recovery downtime, and phase-reset status
- `/api/config.json` current/default drive and LED configuration
- `POST /api/config` save one drive or LED setting
- `POST /api/config/reset` reset one drive or LED setting to its firmware default

The DShot error statistic is the ESP32 DShot TX API error count. It reports local RMT/API failures, not acknowledgment that an ESC received or acted on a command.

### WiFi drive configurator

The dashboard loads the configurator before the larger diagnostic responses. It can change seven persistent drive and LED settings while the controller is safely stopped in WiFi mode. Each setting has its own Save and Reset-to-default button. Save requests are range-checked, written to NVS, read back for verification, and rejected outside the WiFi safety state.

- `motor_full_scale_percent` (**Full-output throttle (%)**): `1–100%`, default `64.70588%`. Sets both the raw spin-throttle endpoint that means logical 100% and the physical output ceiling after spin, translation, or config mixing. Full translation still reaches that ceiling with the opposite motor at zero. The RPM acceleration ceiling remains active. Ordinary and boosted tank output bypass this percentage.
- `translation_scale_factor`: `0.00–2.00`, default `1.00`. Scales the difference between the base spin command and the translation-pulse command. Zero disables translation modulation; one applies the full difference; values above one amplify it, with motor commands clamped between zero and the same-direction full-scale limit.
- `percentdecel`: `1–50%`, default `25%`. Sets the live angular width of each opposing translation pulse. Larger values apply the impulse longer but can increase braking, current, ESC stress, and motor heat.
- `spin_ramp_start_rpm`: `0–5000 RPM`, default approximately `1003.9606 RPM`. Sets where the logical command ceiling starts rising above its fixed 200 level. It can move independently of the growth coefficient.
- `acc_rate`: `0.01–1.00`, default `0.05`, adjustable in steps of `0.01`. Multiplies the `RPM^1.2` growth above the selected knee; a higher value makes that part of the ceiling steeper without moving the knee. The formula is `int(clamp(200 + acc_rate × (max(RPM, knee)^1.2 − knee^1.2), 200, 999))`.
- `top_led_phase_offset_deg`: `-180–+180°`, default `0°`. Adds a display-only phase trim to the top heading marker after the shared saved LED calibration.
- `bottom_led_phase_offset_deg`: `-180–+180°`, default `0°`. Adds a display-only phase trim to the bottom heading marker after its normal 180-degree underside orientation.

The spin-curve graph is a self-contained SVG served by the controller and works without an internet connection. Its axes stay fixed at 0–5000 body RPM and 0–1000 logical command. The dotted reference is the firmware-default curve; the solid curve previews the edited knee and coefficient as inputs change. Previewing does not change motor commands. Save persists a setting to the controller; until then, edits are an unsaved graph preview. The graph shows the logical acceleration ceiling for a full normalized spin request, before the final `motor_full_scale_percent` scaling; at the default 64.71% scale, logical 999 becomes physical 646.

Saved values are stored in the `vibemelt` NVS namespace and load at boot. Resetting a setting stores its firmware default. Configuration storage is separate from the volatile session telemetry, which remains RAM-only.

The full-output setting uses the NVS key `motor_full_pct`. A missing saved value loads the new default; later Save or Reset changes persist across reboots. Adding or changing this setting does not clear accelerometer zero, RPM correction points, or LED calibration.

The spin-ramp knee uses the NVS key `ramp_start_rpm`. When that key is missing, the firmware seeds the knee with `pow(200 / saved_acc_rate, 1 / 1.2)` so an existing valid saved coefficient keeps its legacy spin-ceiling curve. With the default coefficient, this gives approximately 1003.9606 RPM. Subsequent knee and coefficient changes persist independently. This addition preserves accelerometer, RPM, and LED calibration, full-output scaling, translation pulse width, and the separate LED arc width.

On the first boot after this tuning upgrade, the `drive_tune_v` migration installs `acc_rate = 0.05` and `percentdecel = 25` once. Accelerometer, RPM, and LED calibration are preserved. Later changes saved through the web UI load normally on subsequent boots.

When upgrading from firmware that used a different NVS namespace, saved settings are not imported. Recalibrate the accelerometer, RPM correction table, and LED heading offset, and reapply any custom drive settings after flashing.

## Safety States

The firmware uses explicit armed-idle, tank, spin-up, melty, config, WiFi, failsafe, low-battery, and sensor-fault states. The 2 kHz motor task accepts nonzero output only in the four motion states and only while RC, accelerometer, battery, and WiFi safety gates are valid. The ESC link is disarmed only by WiFi, failsafe, low-battery, or sensor-fault states.

- **RC failsafe:** Any of the eight channels outside 900-2100 microseconds rejects the whole frame and immediately substitutes centered sticks with throttle and switches low. A 250 ms fresh-frame timeout also enters failsafe.
- **Sensor fault:** The accelerometer configuration is verified every 500 ms. Three failed configuration reads, five consecutive incomplete axis reads, or a non-finite acceleration sample enters sensor fault. A readable sensor whose control registers reset is reconfigured immediately. Raw-axis clipping does not enter sensor fault.
- **Battery warning:** Battery voltage is filtered every 100 ms. Ten accumulated low samples below 3.60 V/cell enable the warning indication. The warning counter decays again when voltage recovers.
- **Critical battery halt:** Ten consecutive samples below 3.30 V/cell latch shutdown until reboot. A voltage above 5.5 V and a detected/configured cell count are required before battery-low logic is considered active.
- Set `BATTERY_CELL_COUNT` near the top of `main.ino` to `3` or `4` for a fixed pack. The default `0` retains 3S/4S auto-detection, which assumes packs above 13 V at startup are 4S.

Because shutdown is latched, recovering pack voltage after the critical threshold does not restart the motors. Lower all controls, correct the battery condition, and reboot.

LED feedback:

- Fast flashing red: critical battery halt.
- Flashing purple: accelerometer/sensor fault.
- Yellow chase: RC failsafe.
- Solid blue: WiFi mode.
- White at startup: boot/setup.
- Red/orange bar while stopped: battery-level display.
- Green/cyan at the field heading: heading indication; orange replaces it during a battery warning.
- Dim red in the centered 54-degree arcs: translation-direction indication.

Tank and spin-up do not have dedicated unique state colors. Enable serial debug when the exact selected state needs to be observed.

## Real-Time Control

Motor output and phase accumulation run in a dedicated task released every 500 microseconds by `esp_timer` (2 kHz). DShot packets are published every other task iteration (1 kHz), matching PotatoMelt-ESP32 while preserving the finer phase decisions. The normal Arduino loop publishes an atomic snapshot containing state, base throttle, filtered RPM, correction factor, and requested translation vector. LED rendering, RC parsing, sensor reads, NVS writes, and WiFi servicing cannot directly stretch a translation pulse.

MELTY phase lighting follows priority: the top strip's calibrated ±10-degree heading window is green, the upside-down strip's teal-green heading window starts 180 degrees later, the offset-adjusted copy of the translation-direction windows is dark red, and each strip is off outside its applicable windows. `top_led_phase_offset_deg` and `bottom_led_phase_offset_deg` then trim the two heading markers independently; if their windows overlap, both strips retain their heading colors. The displayed red windows are centered on the requested field vector, so the applicable heading marker bisects a forward window and takes color priority on that strip. This centering is display-only: the real motor windows retain their original leading-edge timing. `led_heading_offset_deg` rotates both heading windows and the red display copy together without changing the real field-vector motor target or motor timing. The per-strip trims affect only their heading markers. LED frames are coalesced and written by a separate priority-2 task using uninterrupted SPI DMA transfers, below the priority-4 motor scheduler. The priority-4 task only evaluates the small phase comparisons and sends a non-blocking task notification; all LED encoding and SPI output stays in the lower-priority task. The DMA transfer prevents the eighth LED from receiving a late split frame.

The H3LIS331DL is configured for a 1 kHz output data rate. RPM estimation reads it at 400 Hz, projects X/Y acceleration onto the known 8.55/12.30 mm radial mounting vector, applies a median-of-three filter followed by a 0.4084-alpha exponential low-pass filter, and advances phase by integrating corrected RPM over elapsed time. With the configured ±400 g range, the Y axis reaches full scale at approximately 5,393 RPM for this mounting geometry. Axis clipping is accepted as data and does not stop motor output.

If the scheduler wakes late, the phase accumulator uses the measured elapsed time and records missed periods. It does not emit a burst of old commands. The live motor gate checks RC frame age again on every 2 kHz task iteration. If that gate closes, the next 1 kHz DShot update sends zero to both ESCs.

## OTA Upload

1. Flash the firmware once over USB.
2. Power the bot.
3. Set CH6 high to enter WiFi mode.
4. Connect your computer to the `vibemelt` WiFi network.
5. In Arduino IDE, select the XIAO ESP32-C3 board and the network port for `esp32-ap`.
6. Upload while CH6 remains high.

## Phone Diagnostics

1. Safely stop the robot and hold CH6 high for one second.
2. Connect the phone to the `vibemelt` access point using password `vibemelt`.
3. Open `http://vibe.melt/` (`http://192.168.4.1/` is the fallback).
4. Inspect the frozen summary/table or download the full, MELTY-only, or event files.

## Serial Debug Mode

Serial event logging is compiled out by default. To enable it, uncomment this line near the top of `main.ino`:

```cpp
#define ENABLE_SERIAL_DEBUG
```

Alternatively, add the define only for a command-line debug build:

```sh
arduino-cli compile \
  --build-property compiler.cpp.extra_flags=-DENABLE_SERIAL_DEBUG \
  --fqbn esp32:esp32:XIAO_ESP32C3 \
  --libraries /path/to/libraries \
  /path/to/src/main
```

Open the USB serial monitor at 115200 baud. Debug mode logs:

- Boot, configuration load, accelerometer probe, ESC setup, and motor-task startup.
- Every robot state transition, including RPM, throttle, and RC health.
- RC acquisition, invalid frames, and frame timeouts.
- Battery pack detection, warning transitions, and critical shutdown.
- Accelerometer disconnect, recovery, and invalid samples.
- Config-mode entry/exit, accelerometer zero values, accepted correction points, and unstable-point rejection.
- Spin-direction changes and captured WiFi/session events.

No serial calls occur in the 2 kHz motor-output task. Debug printing can still slow the normal Arduino loop, so disable it for competition firmware after troubleshooting.

## Build

Open `src/main/main.ino` in Arduino IDE. The `main` sketch folder contains all firmware tabs; bundled dependencies are in the root-level `libraries` folder. Simulator tools are in `tools/sim`.

Install the **esp32 by Espressif Systems** board package and select **XIAO_ESP32C3**. The board package index is `https://espressif.github.io/arduino-esp32/package_esp32_index.json`. For Arduino IDE, install the bundled libraries into your Arduino sketchbook's `libraries` directory. The command below uses the repository's copies directly.

Example compile command, run from the repository root with `arduino-cli` on your PATH:

```sh
arduino-cli compile \
  --fqbn esp32:esp32:XIAO_ESP32C3 \
  --libraries ./libraries \
  ./src/main
```

Run the simulator's quick check from the repository root:

```sh
python tools/sim/robustness_test.py --quick
```

The simulator requires Python 3.10 or newer. Tkinter is needed only for the interactive GUI; headless checks use the Python standard library. Build output and Python caches are ignored by Git.
