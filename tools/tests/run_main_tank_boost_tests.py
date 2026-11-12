"""Exercise the real main-sketch tank boost, mixer, and state-reset hook."""
import argparse
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "src/main"


def extract_function(source, name):
    match = re.search(r"^[^\n;{}]*\b" + re.escape(name) +
                      r"\([^;]*?^\{\n.*?^\}", source,
                      re.MULTILINE | re.DOTALL)
    if not match:
        raise ValueError(f"Missing firmware function: {name}")
    return match[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True)
    args = parser.parse_args()
    build = ROOT / "build/main-tank-boost-tests"
    build.mkdir(parents=True, exist_ok=True)
    main_source = (FIRMWARE / "main.ino").read_text()
    motor_source = (FIRMWARE / "Motor_Control.ino").read_text()
    constants = []
    for name in ("NUM_CHANNELS", "CH_TRANSLATE_LR", "CH_TRANSLATE_FB",
                 "CH_CONFIG_SAVE", "TANK_BOOST_SAMPLE_INTERVAL_MS",
                 "TANK_DRIVE_LIMIT", "TANK_STEERING_LIMIT", "TANK_BOOST_DRIVE_LIMIT",
                 "TANK_BOOST_STEERING_LIMIT", "DEFAULT_MOTOR_FULL_SCALE_PERCENT",
                 "MIN_MOTOR_FULL_SCALE_PERCENT", "MAX_MOTOR_FULL_SCALE_PERCENT"):
        match = re.search(r"^(?:constexpr|const) [^\n;]*\b" + name +
                          r"\s*=[^;]+;", main_source, re.MULTILINE)
        if not match:
            raise ValueError(f"Missing firmware constant: {name}")
        constants.append(match[0])
    state_enum = re.search(r"enum RobotState\s*\{[^}]*\};", main_source)[0]

    # Sampling belongs to the tank helper; calibration keeps its own CH8 logic.
    callers = []
    for source in (main_source, motor_source):
        for match in re.finditer(r"^[^\n;{}]*\b(\w+)\([^;]*?^\{\n.*?^\}",
                                 source, re.MULTILINE | re.DOTALL):
            name, body = match[1], match[0]
            if name != "update_tank_boost" and re.search(r"\bupdate_tank_boost\s*\(", body):
                callers.append(name)
    assert callers == ["tank_drive"], f"Boost sampling escaped TANK: {callers}"
    save_handler = extract_function(main_source, "handle_config_save_button")
    assert "CH_CONFIG_SAVE" in save_handler
    assert not re.search(r"\b(?:tank_drive|update_tank_boost)\s*\(", save_handler)
    loop = extract_function(main_source, "loop")
    assert loop.count("tank_drive()") == 1
    assert "unstick" not in loop.lower()
    motor_task = extract_function(motor_source, "motor_output_task")
    for output in ("output_l", "output_r"):
        assert re.search(r"scale_motor_output\(\s*" + output +
                         r"\s*,\s*control\.state\s*\)", motor_task)

    bodies = [extract_function(motor_source, name) for name in (
        "reset_tank_boost", "update_tank_boost", "tank_drive", "scale_motor_output")]
    bodies.append(extract_function(main_source, "set_robot_state"))
    source = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define DEBUG_LOGF(...) ((void)0)
'''
    source += "\n".join(constants) + "\n" + state_enum
    source += r'''
RobotState robot_state = ROBOT_ARMED_IDLE;
RobotState last_operating_state = ROBOT_ARMED_IDLE;
bool tank_boost_active = false, tank_boost_sampled = false;
unsigned long tank_boost_last_sample_ms = 0, now_ms = 0;
bool rc_status = true, accel_sensor_healthy = true;
bool config_mode = false, wifi_mode_active = false, battery_critical_latched = false;
bool reversed = false;
int duty[NUM_CHANNELS + 1] = {};
int motorL = 0, motorR = 0;
float motor_full_scale_percent = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
int state_transitions = 0;
unsigned long millis() { return now_ms; }
void session_on_state_transition(int, int) { ++state_transitions; }
long map(long value, long in_low, long in_high, long out_low, long out_high) {
  return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}
'''
    source += "\n".join(bodies)
    source += r'''
void healthy_tank(bool held) {
  config_mode = wifi_mode_active = battery_critical_latched = false;
  rc_status = accel_sensor_healthy = true;
  set_robot_state(ROBOT_ARMED_IDLE);
  set_robot_state(ROBOT_TANK);
  duty[CH_TRANSLATE_LR] = 50;
  duty[CH_TRANSLATE_FB] = 100;
  duty[CH_CONFIG_SAVE] = held ? 100 : 0;
  tank_drive();
}
void assert_boost_reset() {
  assert(!tank_boost_active && !tank_boost_sampled);
  assert(tank_boost_last_sample_ms == 0);
}
int main() {
  assert(TANK_BOOST_SAMPLE_INTERVAL_MS == 20);
  // Entry samples immediately, even at time zero; ordinary tank speed remains.
  healthy_tank(false);
  assert(!tank_boost_active && tank_boost_sampled);
  assert(motorL == -100 && motorR == 100);
  duty[CH_CONFIG_SAVE] = 100;
  now_ms = 19; tank_drive();
  assert(!tank_boost_active && tank_boost_last_sample_ms == 0);
  now_ms = 20; tank_drive();
  assert(tank_boost_active && tank_boost_last_sample_ms == 20);
  assert(motorL == -999 && motorR == 999);

  // Release is a level, not a latch: output returns to slow at the next 50 Hz sample.
  duty[CH_CONFIG_SAVE] = 0;
  for(now_ms = 21; now_ms < 40; ++now_ms) {
    tank_drive();
    assert(tank_boost_active && tank_boost_last_sample_ms == 20);
  }
  tank_drive();
  assert(!tank_boost_active && tank_boost_last_sample_ms == 40);
  assert(motorL == -100 && motorR == 100);
  duty[CH_CONFIG_SAVE] = 100;
  now_ms = 41; tank_drive(); assert(!tank_boost_active);
  now_ms = 60; tank_drive(); assert(tank_boost_active);
  now_ms = 80; tank_drive(); assert(tank_boost_active); // Holding does not toggle.
  duty[CH_CONFIG_SAVE] = 50; now_ms = 100; tank_drive();
  assert(!tank_boost_active);
  duty[CH_CONFIG_SAVE] = 51; now_ms = 120; tank_drive();
  assert(tank_boost_active);

  // Leaving TANK clears all sample state, including repeated selection of an
  // already active non-TANK state. A held button is sampled afresh on re-entry.
  for(RobotState state : {ROBOT_ARMED_IDLE, ROBOT_SPINUP, ROBOT_MELTY,
                          ROBOT_CONFIG, ROBOT_WIFI, ROBOT_FAILSAFE,
                          ROBOT_LOW_BATTERY, ROBOT_SENSOR_FAULT}) {
    healthy_tank(true);
    assert(tank_boost_active);
    set_robot_state(state); assert_boost_reset();
    tank_boost_active = tank_boost_sampled = true;
    tank_boost_last_sample_ms = now_ms;
    set_robot_state(state); assert_boost_reset();
    tank_boost_active = tank_boost_sampled = true;
    update_tank_boost(); assert_boost_reset();
    set_robot_state(ROBOT_TANK); tank_drive();
    assert(tank_boost_active); // No 20 ms wait after entry.
  }
  const int transitions_before = state_transitions;
  set_robot_state(ROBOT_TANK);
  assert(tank_boost_active && state_transitions == transitions_before);

  // Every safety condition overrides an already active boost immediately.
  for(int fault = 0; fault < 5; ++fault) {
    healthy_tank(true);
    if(fault == 0) config_mode = true;
    if(fault == 1) wifi_mode_active = true;
    if(fault == 2) rc_status = false;
    if(fault == 3) accel_sensor_healthy = false;
    if(fault == 4) battery_critical_latched = true;
    update_tank_boost(); assert_boost_reset();
  }

  // Unsigned clock rollover keeps the 20 ms interval correct.
  now_ms = (std::numeric_limits<unsigned long>::max)() - 10;
  healthy_tank(true);
  const unsigned long rollover_sample = tank_boost_last_sample_ms;
  duty[CH_CONFIG_SAVE] = 0;
  now_ms = rollover_sample + 19; tank_drive();
  assert(tank_boost_active && tank_boost_last_sample_ms == rollover_sample);
  now_ms = rollover_sample + 20; tank_drive();
  assert(!tank_boost_active && tank_boost_last_sample_ms == now_ms);

  // Reversing the melty-spin switch no longer grants automatic unstick output
  // or changes ordinary tank polarity. Steering retains the existing mixer.
  struct MixerCase { int forward, turn, slow_l, slow_r, boost_l, boost_r; };
  const MixerCase cases[] = {
    {50, 50, 0, 0, 0, 0}, {100, 50, -100, 100, -999, 999},
    {0, 50, 100, -100, 999, -999}, {50, 0, 20, 20, 200, 200},
    {50, 100, -20, -20, -200, -200},
    {100, 0, -80, 120, -799, 999}, {100, 100, -120, 80, -999, 799},
    {0, 0, 120, -80, 999, -799}, {0, 100, 80, -120, 799, -999},
    {75, 50, -50, 50, -499, 499},
  };
  for(bool reverse : {false, true}) {
    reversed = reverse;
    for(bool held : {false, true}) {
      healthy_tank(held);
      for(const auto &test : cases) {
        duty[CH_TRANSLATE_FB] = test.forward;
        duty[CH_TRANSLATE_LR] = test.turn;
        now_ms += 20; tank_drive();
        assert(motorL == (held ? test.boost_l : test.slow_l));
        assert(motorR == (held ? test.boost_r : test.slow_r));
        assert(motorL >= -999 && motorL <= 999 && motorR >= -999 && motorR <= 999);
      }
    }
  }

  healthy_tank(true);
  for(float full_scale : {100.0f, DEFAULT_MOTOR_FULL_SCALE_PERCENT, 50.0f}) {
    motor_full_scale_percent = full_scale;
    assert(scale_motor_output(motorL, ROBOT_TANK) == -999);
    assert(scale_motor_output(motorR, ROBOT_TANK) == 999);
    assert(scale_motor_output(-100, ROBOT_TANK) == -100);
    assert(scale_motor_output(100, ROBOT_TANK) == 100);
    assert(scale_motor_output(-10000, ROBOT_TANK) == -999);
    assert(scale_motor_output(10000, ROBOT_TANK) == 999);
  }
  for(RobotState state : {ROBOT_ARMED_IDLE, ROBOT_SPINUP, ROBOT_MELTY,
                          ROBOT_CONFIG, ROBOT_WIFI, ROBOT_FAILSAFE,
                          ROBOT_LOW_BATTERY, ROBOT_SENSOR_FAULT}) {
    motor_full_scale_percent = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
    assert(scale_motor_output(-999, state) == -646);
    assert(scale_motor_output(999, state) == 646);
    motor_full_scale_percent = 50;
    assert(scale_motor_output(-999, state) == -500);
    assert(scale_motor_output(999, state) == 500);
    assert(scale_motor_output(0, state) == 0);
  }
  reset_tank_boost(); assert_boost_reset();
  std::puts("PASS: TANK-only hold boost, 50 Hz sampling/release/rollover, state/fault resets, reverse-independent mixer, steering/clamping, calibration isolation, tank output-cap bypass and retained non-tank scaling");
}
'''
    cpp = build / "tests.cpp"
    cpp.write_text(source)
    exe = build / ("tests.exe" if os.name == "nt" else "tests")
    command = [args.compiler]
    if Path(args.compiler).stem.lower() == "zig":
        command += ["c++"]
        if os.name == "nt":
            command += ["-target", "x86_64-windows-gnu"]
    command += ["-std=c++17", "-Wall", "-Wextra", "-Werror", str(cpp),
                "-o", str(exe)]
    subprocess.run(command, check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
