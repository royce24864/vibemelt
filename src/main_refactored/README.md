# Refactored firmware

Open `main_refactored.ino` from this directory as a separate Arduino sketch.
Arduino combines every `.ino` tab in a sketch directory, so this version lives
in its own directory with its own supporting tabs. The original `src/main`
sketch remains independent and unchanged.

Build from the repository root with the same XIAO ESP32-C3 board package and
bundled libraries as the original firmware:

```sh
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 --libraries ./libraries --build-path ./build/refactored ./src/main_refactored
```

## Changes

- Valid low-throttle CH5 direction input is latched before idle-state returns.
  Direction remains fixed during calibration and WiFi mode.
- Calibration owns a snapshot of its entry settings. Only a healthy, deliberate
  CH7-low exit saves. Radio loss, sensor faults, critical battery, canceled or
  failed zeroing, and failed saves restore the entry settings and reset the
  estimator. After an abort, all motor output stays inhibited until CH7 is low,
  throttle is low, and translation/yaw sticks are centered on a healthy RC link.
  Failed saves and aborted zeroing produce red LED feedback.
- Accelerometer zeroing returns a result, services the receiver/watchdog/battery,
  and checks every sample before publishing new offsets. Moving controls or
  releasing CH7 during acquisition cancels the operation.
  Zeroing and storage publish explicitly disabled motor snapshots; a fresh
  estimator sample is required before normal output can resume.
- LED mode, frame, and transition number are read and published together under
  a short critical section. SPI rendering and task notifications run outside it.
- Configuration persistence uses a validated, versioned 140-byte record with a
  CRC32 and readback verification. Calibration and web settings share this save
  path. Runtime settings are restored if a save cannot be verified.
- Telemetry records the acceleration ceiling when each sample is captured.
  CSV/JSON exports do not recalculate it after configuration changes. Missed
  samples use `-1` for an unknown ceiling; the dashboard displays an em dash.
  Sample size grows from 16 to 18 bytes, adding 6,000 bytes to the existing ring.
- Controller decisions and drive calculations, accelerometer estimation, and
  calibration state now live in `Controller.cpp`, `Estimator.cpp`, and
  `Calibration.cpp`. These modules own explicit state and inputs; sketch tabs
  connect them to the receiver, I2C, storage, LEDs, and motor task.
- The separate sketch's DShot driver is locally modified from version 0.9.5.
  It polls completion without waiting, uses nonblocking queue submission, and
  retains each asynchronous payload until completion. A stop request submits
  one frame. Results distinguish queued, skipped, and failed frames, and each
  ESC has its own counters and accepted-frame averages.
- Motor snapshots carry independent demand-publication and actual sensor-sample
  timestamps. Enabled demand expires after 50 ms; sensor data expires after
  25 ms. Repeatedly publishing an old sensor value cannot refresh its age.
- Phase integration uses the full elapsed time. A gap over 20 ms invalidates
  phase and inhibits output until recovery resets it.
- Startup and ordinary RC, sensor, or motor faults require healthy inputs,
  low throttle, and centered translation/yaw controls before output can resume.
  A motor fault also requires two accepted zero frames per ESC after fault
  entry. The main loop waits for the motor task's recovery acknowledgement.
  Calibration aborts additionally require CH7 low. The critical-battery latch
  continues to require a restart.
- Motor-task readiness includes successful task creation, watchdog registration,
  and timer startup. Missing timer notifications for 20 ms triggers stop-frame
  attempts and stops feeding the motor watchdog, allowing the configured
  one-second watchdog to reset the controller. WiFi mode suspends only the
  main-loop watchdog; the motor watchdog remains active.
- Logger catch-up inserts at most 64 rows per service call and skips intervals
  outside the retained 3,000-row window arithmetically. Missing rows are marked
  `UNKNOWN` and excluded from state-specific summaries and MELTY exports.
  The live dashboard exposes output readiness, rearm status, motor faults, and
  separate queued/skipped/failed counts for each ESC.

## Stored settings

This sketch writes only to the `vibemelt_v2` NVS namespace. When its record is
absent, it reads the original `vibemelt` keys without modifying them, validates
them, and saves a complete new-format record. Existing original-firmware settings
therefore remain available when switching back to `src/main`.

After migration, the two sketches' settings evolve independently. Corrupt or
unsupported new-format records load defaults and are left intact until an
explicit save. A failed readback means persistence was not confirmed; a complete
new record may still have reached flash even though the running sketch restores
its previous settings.

## Regression checks

The tests in `tools/tests` compile the native modules directly and exercise
hardware adapters with deterministic hardware/storage/RTOS stubs. They do not
use the simulator. Generated files and executables go under `build`.

With a native C++17 compiler, run:

```sh
python tools/tests/run_firmware_tests.py --compiler /absolute/path/to/c++
```

On Windows, the runner also accepts a path to `zig.exe`.
These checks cover configuration corruption and rejected writes, input/state
transitions and rearming, estimator reset and filtering, calibration aborts and
rollback, phase/freshness boundaries, DShot payload ownership and failed/busy
transmits, LED publication during rendering, and telemetry export and bounded
catch-up across gaps up to seven days.

Build and native checks do not establish physical ESC reception. Queued frames
mean driver acceptance only; this firmware uses unidirectional DShot. Validate
startup, link/sensor interruptions, scheduler recovery, calibration, and WiFi
entry/exit on the controller and ESCs before using this version for driving.
