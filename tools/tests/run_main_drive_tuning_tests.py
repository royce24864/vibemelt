"""Exercise the main sketch's drive tuning and motor math without hardware."""
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
    build = ROOT / "build/main-drive-tuning-tests"
    build.mkdir(parents=True, exist_ok=True)
    main_source = (FIRMWARE / "main.ino").read_text()
    constants = []
    for name in ("DEFAULT_ACC_RATE", "MIN_ACC_RATE", "MAX_ACC_RATE",
                 "DEFAULT_PERCENT_DECEL", "MIN_PERCENT_DECEL",
                 "MAX_PERCENT_DECEL", "DRIVE_TUNING_VERSION",
                 "SPIN_ACCEL_MIN_COMMAND", "SPIN_ACCEL_EXPONENT",
                 "DEFAULT_SPIN_RAMP_START_RPM", "MIN_SPIN_RAMP_START_RPM",
                 "MAX_SPIN_RAMP_START_RPM",
                 "ACCEL_SAMPLE_INTERVAL_US", "DEFAULT_MOTOR_FULL_SCALE_PERCENT",
                 "MIN_MOTOR_FULL_SCALE_PERCENT", "MAX_MOTOR_FULL_SCALE_PERCENT",
                 "DEFAULT_TRANSLATION_SCALE_FACTOR", "MIN_TRANSLATION_SCALE_FACTOR",
                 "MAX_TRANSLATION_SCALE_FACTOR", "DEFAULT_TOP_LED_PHASE_OFFSET_DEG",
                 "DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG", "MIN_LED_PHASE_OFFSET_DEG",
                 "MAX_LED_PHASE_OFFSET_DEG", "NUM_CHANNELS", "CH_SPIN_THROTTLE",
                 "CH_TRANSLATE_LR", "CH_TRANSLATE_FB", "CH_WIFI_MODE", "CH_SPIN_DIR",
                 "CH_CONFIG_MODE", "CH_CONFIG_SAVE", "CH_HEADING_YAW",
                 "stick_deadzone_percent", "throttle_low_percent", "wifi_entry_max_rpm",
                 "RC_VALID_MIN_US", "RC_VALID_MAX_US", "RC_FRAME_TIMEOUT_MS"):
        match = re.search(r"^(?:constexpr|const) [^\n;]*\b" + name +
                          r"\s*=[^;]+;", main_source, re.MULTILINE)
        if not match:
            raise ValueError(f"Missing firmware constant: {name}")
        constants.append(match[0])
    bodies = []
    for filename, names in {
        "Heading.ino": ["validStoredFloat", "wrap360", "stick_percent_centered",
                        "load_drive_tuning_settings", "load_spin_ramp_setting",
                        "load_motor_full_scale_setting"],
        "Motor_Control.ino": ["translation_command", "spin_acceleration_ceiling",
                              "spin_requested_command", "scale_motor_output", "spin",
                              "translation_active", "translate_field_oriented"],
        "RC_Handler.ino": ["set_safe_channels", "rc_frame_values_valid", "update_channels"],
        "main.ino": ["update_robot_state", "config_entry_controls_safe",
                     "motors_are_off", "update_spin_direction_input"],
        "Session_Logger.ino": ["session_acceleration_ceiling",
                               "session_spin_request_percent", "session_acceleration_limiter_active",
                               "drive_config_metadata",
                               "drive_config_parse_float", "drive_config_request_safe",
                               "session_send_config_error", "session_handle_config_update",
                               "session_send_config_json"],
    }.items():
        source = (FIRMWARE / filename).read_text()
        bodies.extend(extract_function(source, name) for name in names)
    source = r'''
#include <cassert>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <utility>
using std::isnan;
using std::isfinite;
using String = std::string;
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define DEBUG_LOG(...) ((void)0)
#define DEBUG_LOGF(...) ((void)0)
'''
    source += "\n".join(constants)
    source += r'''
struct FakeNvs {
  std::map<std::string, double> values;
  int writes = 0;
  float getFloat(const char *key, float fallback) {
    return values.count(key) ? values[key] : fallback;
  }
  int getInt(const char *key, int fallback) {
    return values.count(key) ? values[key] : fallback;
  }
  size_t putFloat(const char *key, float value) {
    assert(std::string(key).size() <= 15); values[key] = value; ++writes;
    return sizeof(float);
  }
  size_t putInt(const char *key, int value) {
    assert(std::string(key).size() <= 15); values[key] = value; ++writes;
    return sizeof(int);
  }
} config_store;
float acc_rate = DEFAULT_ACC_RATE, percentdecel = DEFAULT_PERCENT_DECEL;
float spin_ramp_start_rpm = DEFAULT_SPIN_RAMP_START_RPM;
float motor_full_scale_percent = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
float spin_throttle_input_percent = 0;
float translation_scale_factor = DEFAULT_TRANSLATION_SCALE_FACTOR;
float top_led_phase_offset_deg = DEFAULT_TOP_LED_PHASE_OFFSET_DEG;
float bottom_led_phase_offset_deg = DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG;
int duty[NUM_CHANNELS + 1] = {}, rpm = 0, spinspeed = 0, motorL = 0, motorR = 0;
constexpr double PI = 3.14159265358979323846;
bool reversed = false, pending_translation_enabled = false;
float pending_translation_magnitude = 0, pending_target_phase_deg = 0;
float pending_translation_display_phase_deg = 0, drive_heading_phase_deg = 0;
long map(long value, long in_low, long in_high, long out_low, long out_high) {
  return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}
bool near(float a, float b) { return std::fabs(a - b) < .0001f; }
struct FakeIBus {
  int cnt_rec = 0;
  std::array<int, NUM_CHANNELS> frame = {};
  void loop() {}
  int readChannel(int index) { return frame.at(index); }
} IBus;
unsigned long now_ms = 100, last_valid_rc_frame_ms = 0, rec_gap = 0;
int last_rec = 0;
unsigned int invalid_rc_frame_count = 0;
uint32_t total_invalid_rc_frame_count = 0;
bool rc_status = false;
unsigned long millis() { return now_ms; }
void reset_loop_task_watchdog() {}
void session_note_rc_status(bool) {}
bool wifi_mode_active = false, fake_motor_gate_open = false, config_mode = false;
bool motor_on = false, battery_critical_latched = false, accel_sensor_healthy = true;
enum RobotState { ROBOT_ARMED_IDLE, ROBOT_TANK, ROBOT_SPINUP, ROBOT_MELTY,
                  ROBOT_CONFIG, ROBOT_WIFI, ROBOT_FAILSAFE,
                  ROBOT_LOW_BATTERY, ROBOT_SENSOR_FAULT };
RobotState robot_state = ROBOT_ARMED_IDLE;
void set_robot_state(int value) { robot_state = static_cast<RobotState>(value); }
bool motor_output_gate_open() { return fake_motor_gate_open; }
struct FakeServer {
  std::map<std::string, std::string> arguments;
  int status = 0;
  std::string response;
  bool hasArg(const char *name) const { return arguments.count(name) != 0; }
  String arg(const char *name) const { return arguments.at(name); }
  void sendHeader(const char *, const char *) {}
  void send(int code, const char *, const char *body) {
    status = code; response = body;
  }
} diagnostic_server;
'''
    source += "\n".join(bodies)
    source += r'''
int main(int argc, char **argv) {
  // Upgrading replaces only the old drive tuning, keeping calibration intact.
  config_store.values = {{"acc_rate", .4}, {"pulse_pct", 15},
                         {"rpm_offset", 1.04}, {"r_rpm_offset", .96},
                         {"accel_zero_x", .8}, {"top_led_trim", 12},
                         {"trans_scale", .7}};
  load_drive_tuning_settings();
  assert(near(acc_rate, .05f) && near(percentdecel, 25));
  assert(config_store.getInt("drive_tune_v", 0) == DRIVE_TUNING_VERSION);
  assert(config_store.values.size() == 8 && config_store.writes == 3);
  assert(config_store.values.at("rpm_offset") == 1.04);
  assert(config_store.values.at("r_rpm_offset") == .96);
  assert(config_store.values.at("accel_zero_x") == .8);
  assert(config_store.values.at("top_led_trim") == 12);
  assert(config_store.values.at("trans_scale") == .7);

  // A later reboot retains saved web changes and does not rewrite NVS.
  config_store.putFloat("acc_rate", .07f);
  config_store.putFloat("pulse_pct", 32);
  const int writes_before_boot = config_store.writes;
  load_drive_tuning_settings();
  assert(near(acc_rate, .07f) && near(percentdecel, 32));
  assert(config_store.writes == writes_before_boot);
  for(float rate : {.01f, .05f, 1.0f}) {
    config_store.putFloat("acc_rate", rate);
    load_drive_tuning_settings();
    assert(near(acc_rate, rate));
  }
  for(float invalid : {0.0f, -1.0f, 1.01f,
                       std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity()}) {
    config_store.putFloat("acc_rate", invalid);
    load_drive_tuning_settings();
    assert(near(acc_rate, .05f));
  }
  for(float invalid : {0.0f, -1.0f, 50.01f,
                       std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity()}) {
    config_store.putFloat("pulse_pct", invalid);
    load_drive_tuning_settings();
    assert(near(percentdecel, 25));
  }
  config_store.values.clear();
  load_drive_tuning_settings();
  assert(near(acc_rate, .05f) && near(percentdecel, 25));
  assert(ACCEL_SAMPLE_INTERVAL_US <= 10000);

  // The added knee preserves an old saved slope's curve on first upgrade.
  // Once saved, changing either control must not silently move the other one.
  config_store.putFloat("acc_rate", .05f);
  const int writes_before_ramp = config_store.writes;
  load_spin_ramp_setting();
  assert(std::fabs(spin_ramp_start_rpm - DEFAULT_SPIN_RAMP_START_RPM) < .001f);
  assert(config_store.writes == writes_before_ramp + 1);
  load_spin_ramp_setting();
  assert(config_store.writes == writes_before_ramp + 1);
  config_store.putFloat("ramp_start_rpm", 2500);
  acc_rate = .2f;
  load_spin_ramp_setting();
  assert(near(spin_ramp_start_rpm, 2500) && near(acc_rate, .2f));
  assert(near(percentdecel, 25));
  for(float invalid : {-1.0f, 5000.01f, std::numeric_limits<float>::infinity()}) {
    config_store.putFloat("ramp_start_rpm", invalid);
    load_spin_ramp_setting();
    assert(near(spin_ramp_start_rpm, DEFAULT_SPIN_RAMP_START_RPM));
    assert(near(config_store.getFloat("ramp_start_rpm", -1), spin_ramp_start_rpm));
  }
  for(float rate : {.01f, .05f, .2f, 1.0f}) {
    acc_rate = rate;
    config_store.values.erase("ramp_start_rpm");
    load_spin_ramp_setting();
    for(int measured_rpm : {0, 500, 1000, 2000, 4000, 10000}) {
      const int legacy = (int)constrain(rate * powf((float)measured_rpm, 1.2f), 200.0f, 999.0f);
      assert(std::abs(spin_acceleration_ceiling(measured_rpm) - legacy) <= 1);
    }
  }
  acc_rate = .05f; spin_ramp_start_rpm = 2000;
  assert(spin_acceleration_ceiling(-100) == 200);
  assert(spin_acceleration_ceiling(2000) == 200);
  const int gentle_after_knee = spin_acceleration_ceiling(2500);
  acc_rate = .1f;
  assert(spin_acceleration_ceiling(2000) == 200);
  assert(spin_acceleration_ceiling(2500) > gentle_after_knee);
  spin_ramp_start_rpm = 3000;
  assert(spin_acceleration_ceiling(2500) == 200 && near(acc_rate, .1f));
  for(float start_rpm : {0.0f, 500.0f, DEFAULT_SPIN_RAMP_START_RPM, 2500.0f, 5000.0f}) {
    spin_ramp_start_rpm = start_rpm;
    for(float rate : {.01f, .05f, .2f, 1.0f}) {
      acc_rate = rate;
      int previous = 200;
      for(int measured_rpm = 0; measured_rpm <= 10000; measured_rpm += 25) {
        const int command = spin_acceleration_ceiling(measured_rpm);
        assert(command >= previous && command >= 200 && command <= 999);
        previous = command;
      }
      assert(spin_acceleration_ceiling((std::numeric_limits<int>::max)()) == 999);
    }
  }
  acc_rate = DEFAULT_ACC_RATE;
  spin_ramp_start_rpm = DEFAULT_SPIN_RAMP_START_RPM;

  // A missing output-scale setting is initialized independently of tuning and
  // calibration. Future boots preserve the chosen value without extra writes.
  config_store.putFloat("rpm_offset", 1.06f);
  const int writes_before_scale_load = config_store.writes;
  load_motor_full_scale_setting();
  assert(near(motor_full_scale_percent, 100.0f * 1100.0f / 1700.0f));
  assert(config_store.writes == writes_before_scale_load + 1);
  assert(near(config_store.getFloat("motor_full_pct", 0), motor_full_scale_percent));
  load_motor_full_scale_setting();
  assert(config_store.writes == writes_before_scale_load + 1);
  config_store.putFloat("motor_full_pct", 50);
  load_motor_full_scale_setting();
  assert(near(motor_full_scale_percent, 50));
  assert(near(config_store.getFloat("rpm_offset", 0), 1.06f));
  assert(near(acc_rate, .05f) && near(percentdecel, 25));
  for(float invalid : {0.0f, -1.0f, 100.01f,
                       std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity()}) {
    config_store.putFloat("motor_full_pct", invalid);
    load_motor_full_scale_setting();
    assert(near(motor_full_scale_percent, DEFAULT_MOTOR_FULL_SCALE_PERCENT));
  }

  // Preserve fractional RC throttle, including the 64.7% stock-speed endpoint.
  IBus.frame.fill(1500);
  IBus.frame[CH_SPIN_THROTTLE - 1] = 1647;
  ++IBus.cnt_rec; update_channels();
  assert(rc_status && duty[CH_SPIN_THROTTLE] == 64);
  assert(near(spin_throttle_input_percent, 64.7f));
  IBus.frame[CH_SPIN_THROTTLE - 1] = 1901;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  assert(near(spin_throttle_input_percent, 90.1f));
  IBus.frame[CH_SPIN_THROTTLE - 1] = 2100;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  assert(near(spin_throttle_input_percent, 100));
  IBus.frame[CH_SPIN_THROTTLE - 1] = 900;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  assert(near(spin_throttle_input_percent, 0));
  IBus.frame[CH_SPIN_THROTTLE - 1] = 1647;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  IBus.frame[0] = 2200;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  assert(!rc_status && duty[CH_SPIN_THROTTLE] == 0);
  assert(near(spin_throttle_input_percent, 0));
  IBus.frame[0] = 1500;
  ++IBus.cnt_rec; now_ms += 10; update_channels();
  assert(rc_status && near(spin_throttle_input_percent, 64.7f));
  now_ms = last_valid_rc_frame_ms + RC_FRAME_TIMEOUT_MS + 1;
  update_channels();
  assert(!rc_status && near(spin_throttle_input_percent, 0));

  // Known curve points cover startup, nonlinear growth, and DShot saturation.
  motor_full_scale_percent = 100;
  for(const auto &point : {std::pair<int, int>{-100, 200}, {0, 200},
                           {1000, 200}, {2000, 457}, {3000, 743},
                           {4000, 999}, {65535, 999},
                           {(std::numeric_limits<int>::max)(), 999}}) {
    rpm = point.first;
    assert(spin_acceleration_ceiling(rpm) == point.second);
    for(bool reverse : {false, true}) {
      reversed = reverse;
      duty[3] = 100; spin_throttle_input_percent = 100;
      pending_translation_enabled = true;
      pending_translation_magnitude = 1;
      spin();
      const int expected = reverse ? -point.second : point.second;
      assert(motorL == expected && motorR == expected);
      assert(!pending_translation_enabled && pending_translation_magnitude == 0);
      duty[3] = 10; spin_throttle_input_percent = 10; spin();
      assert(motorL == (reverse ? -100 : 100) && motorR == motorL);
      duty[3] = 0; spin_throttle_input_percent = 0; spin();
      assert(motorL == 0 && motorR == 0);
    }
  }
  assert(session_acceleration_ceiling(0) == 20);
  assert(session_acceleration_ceiling(2000) == 46);
  assert(session_acceleration_ceiling(3000) == 74);
  assert(session_acceleration_ceiling(4000) == 100);
  assert(session_acceleration_limiter_active(2000, 100));
  assert(!session_acceleration_limiter_active(2000, 40));
  assert(!session_acceleration_limiter_active(4000, 100));

  // The setting normalizes the spin endpoint and scales non-tank commands.
  // Full translation cannot raise the fast motor above the selected output.
  for(float full_scale : {100.0f, 50.0f, DEFAULT_MOTOR_FULL_SCALE_PERCENT}) {
    motor_full_scale_percent = full_scale;
    const int physical_full = full_scale == 100 ? 999 : full_scale == 50 ? 500 : 646;
    assert(spin_requested_command(0) == 0);
    assert(spin_requested_command(-10) == 0);
    assert(spin_requested_command(full_scale) == 999);
    assert(spin_requested_command(100) == 999);
    assert(spin_requested_command(200) == 999);
    assert(spin_requested_command(full_scale / 2) == 500);
    assert(session_spin_request_percent(full_scale) == 100);
    assert(session_spin_request_percent(full_scale / 2) == 50);
    assert(session_acceleration_limiter_active(2000, full_scale));
    assert(!session_acceleration_limiter_active(2000, full_scale * .4f));
    assert(scale_motor_output(0, ROBOT_SPINUP) == 0);
    assert(scale_motor_output(999, ROBOT_SPINUP) == physical_full);
    assert(scale_motor_output(10000, ROBOT_SPINUP) == physical_full);
    assert(scale_motor_output(-10000, ROBOT_SPINUP) == -physical_full);
    for(bool reverse : {false, true}) {
      reversed = reverse;
      const int sign = reverse ? -1 : 1;
      spin_throttle_input_percent = full_scale;
      duty[3] = static_cast<int>(full_scale);
      rpm = 5000; spin();
      assert(motorL == sign * 999 && motorR == motorL);
      assert(scale_motor_output(motorL, ROBOT_MELTY) == sign * physical_full);
      assert(scale_motor_output(translation_command(motorL, true, 1), ROBOT_MELTY) ==
             sign * physical_full);
      assert(scale_motor_output(translation_command(motorL, false, 1), ROBOT_MELTY) == 0);
      // The acceleration curve remains active below the requested endpoint.
      rpm = 2000; spin();
      assert(motorL == sign * 457 && motorR == motorL);
      assert(std::abs(scale_motor_output(motorL, ROBOT_MELTY)) < physical_full);
      for(float fraction : {.25f, .5f, .75f}) {
        const int slow = scale_motor_output(translation_command(motorL, false, fraction), ROBOT_MELTY);
        const int fast = scale_motor_output(translation_command(motorL, true, fraction), ROBOT_MELTY);
        assert(std::abs(slow) <= std::abs(scale_motor_output(motorL, ROBOT_MELTY)));
        assert(std::abs(fast) >= std::abs(scale_motor_output(motorL, ROBOT_MELTY)));
        assert(std::abs(fast) <= physical_full && slow * sign >= 0);
      }
    }
  }
  motor_full_scale_percent = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
  assert(spin_requested_command(64.7f) == 999);
  assert(std::abs(1700 * scale_motor_output(999, ROBOT_SPINUP) - 1100 * 999) < 1700);
  motor_full_scale_percent = 50;
  assert(scale_motor_output(spin_requested_command(25), ROBOT_SPINUP) == 250);
  assert(scale_motor_output(spin_requested_command(50), ROBOT_SPINUP) == 500);
  assert(scale_motor_output(spin_requested_command(100), ROBOT_SPINUP) == 500);

  // State selection and low-throttle guards use normalized demand even when a
  // full-output endpoint is below the former 10% raw spin threshold.
  rc_status = true;
  duty[CH_TRANSLATE_LR] = duty[CH_TRANSLATE_FB] = duty[CH_HEADING_YAW] = 50;
  for(float full_scale : {1.0f, 50.0f, DEFAULT_MOTOR_FULL_SCALE_PERCENT, 100.0f}) {
    motor_full_scale_percent = full_scale;
    motorL = motorR = 0; motor_on = false;
    spin_throttle_input_percent = full_scale;
    rpm = 5000; update_robot_state();
    assert(robot_state == ROBOT_MELTY);
    rpm = 0; update_robot_state();
    assert(robot_state == ROBOT_SPINUP);
    spin_throttle_input_percent = full_scale * .1f;
    update_robot_state();
    assert(robot_state == ROBOT_ARMED_IDLE);
    spin_throttle_input_percent = full_scale * .102f;
    update_robot_state();
    assert(robot_state == ROBOT_SPINUP);
    spin_throttle_input_percent = full_scale * .05f;
    assert(config_entry_controls_safe() && motors_are_off());
    spin_throttle_input_percent = full_scale * .051f;
    assert(!config_entry_controls_safe() && !motors_are_off());
    reversed = false; duty[CH_SPIN_DIR] = 100;
    spin_throttle_input_percent = full_scale * .199f;
    update_spin_direction_input();
    assert(reversed);
    reversed = false;
    spin_throttle_input_percent = full_scale * .2f;
    update_spin_direction_input();
    assert(!reversed);
    spin_throttle_input_percent = 0; config_mode = true;
    update_spin_direction_input(); assert(!reversed);
    config_mode = false; wifi_mode_active = true;
    update_spin_direction_input(); assert(!reversed);
    wifi_mode_active = false;
  }

  // Translation retains stick sign in both spin directions. Display direction
  // remains in the field frame; reverse-spin compensation changes motor phase.
  struct Direction { int ch1, ch2; float display, reverse_motor; };
  const Direction directions[] = {{50, 100, 0, 90}, {50, 0, 180, 270},
                                  {0, 50, 90, 0}, {100, 50, 270, 180}};
  motor_full_scale_percent = 100;
  rpm = 2000; duty[3] = 100; spin_throttle_input_percent = 100;
  drive_heading_phase_deg = 30;
  for(bool reverse : {false, true}) {
    reversed = reverse;
    for(const auto &direction : directions) {
      duty[1] = direction.ch1; duty[2] = direction.ch2;
      assert(translation_active());
      translate_field_oriented();
      assert(pending_translation_enabled && near(pending_translation_magnitude, 1));
      assert(near(pending_translation_display_phase_deg,
                  wrap360(30 + direction.display)));
      assert(near(pending_target_phase_deg,
                  wrap360(30 + (reverse ? direction.reverse_motor : direction.display))));
      assert(motorL == (reverse ? -457 : 457) && motorR == motorL);
    }
    duty[1] = duty[2] = 50;
    assert(!translation_active());
    translate_field_oriented();
    assert(!pending_translation_enabled && near(pending_translation_magnitude, 0));
  }
  for(int sign : {-1, 1}) {
    assert(translation_command(sign * 300, false, 1) == 0);
    assert(translation_command(sign * 300, true, 1) == sign * 999);
    assert(translation_command(sign * 300, false, .5f) == sign * 150);
    assert(translation_command(sign * 300, true, .5f) == sign * 650);
    assert(translation_command(sign * 300, true, 2) == sign * 999);
    assert(translation_command(sign * 300, true, -1) == sign * 300);
  }
  assert(translation_command(0, true, 1) == 0);

  // Exercise the real web metadata/parser/update/reset handler for the setting.
  motorL = motorR = 0;
  diagnostic_server.arguments = {{"name", "motor_full_scale_percent"}, {"value", "50"}};
  const int writes_before_unsafe = config_store.writes;
  session_handle_config_update(false);
  assert(diagnostic_server.status == 409 && config_store.writes == writes_before_unsafe);
  wifi_mode_active = true; robot_state = ROBOT_WIFI;
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(motor_full_scale_percent, 50));
  assert(near(config_store.getFloat("motor_full_pct", 0), 50));
  session_send_config_json();
  assert(diagnostic_server.response.find("\"motor_full_scale_percent\":50.000") != std::string::npos);
  assert(diagnostic_server.response.find("\"motor_full_scale_percent\":[1.000,100.000]") != std::string::npos);
  motor_full_scale_percent = 100; load_motor_full_scale_setting();
  assert(near(motor_full_scale_percent, 50));
  diagnostic_server.arguments["value"] = "64.7";
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(motor_full_scale_percent, 64.7f));
  for(const char *invalid : {"0", "100.01", "NaN", "Infinity", "50abc", ""}) {
    diagnostic_server.arguments["value"] = invalid;
    session_handle_config_update(false);
    assert(diagnostic_server.status == 400 || diagnostic_server.status == 422);
    assert(near(motor_full_scale_percent, 64.7f));
    assert(near(config_store.getFloat("motor_full_pct", 0), 64.7f));
  }
  diagnostic_server.arguments["value"] = "1";
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(motor_full_scale_percent, 1));
  fake_motor_gate_open = true;
  diagnostic_server.arguments["value"] = "50";
  session_handle_config_update(false);
  assert(diagnostic_server.status == 409 && near(motor_full_scale_percent, 1));
  fake_motor_gate_open = false;
  diagnostic_server.arguments.erase("value");
  session_handle_config_update(true);
  assert(diagnostic_server.status == 200);
  assert(near(motor_full_scale_percent, DEFAULT_MOTOR_FULL_SCALE_PERCENT));
  motor_full_scale_percent = 100; load_motor_full_scale_setting();
  assert(near(motor_full_scale_percent, DEFAULT_MOTOR_FULL_SCALE_PERCENT));
  assert(near(config_store.getFloat("rpm_offset", 0), 1.06f));

  diagnostic_server.arguments = {{"name", "spin_ramp_start_rpm"}, {"value", "2500"}};
  const float coefficient_before_knee_save = acc_rate;
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(spin_ramp_start_rpm, 2500));
  assert(near(acc_rate, coefficient_before_knee_save));
  assert(near(config_store.getFloat("ramp_start_rpm", -1), 2500));
  load_spin_ramp_setting(); assert(near(spin_ramp_start_rpm, 2500));
  diagnostic_server.arguments = {{"name", "acc_rate"}, {"value", ".08"}};
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(acc_rate, .08f));
  assert(near(spin_ramp_start_rpm, 2500));
  assert(near(config_store.getFloat("ramp_start_rpm", -1), 2500));
  diagnostic_server.arguments["value"] = ".05";
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(spin_ramp_start_rpm, 2500));
  diagnostic_server.arguments = {{"name", "spin_ramp_start_rpm"}, {"value", "2500"}};
  session_send_config_json();
  assert(diagnostic_server.response.find("\"spin_ramp_start_rpm\":2500.000") != std::string::npos);
  assert(diagnostic_server.response.find("\"spin_ramp_start_rpm\":[0.000,5000.000]") != std::string::npos);
  assert(diagnostic_server.response.find("\"spin_curve\":{") != std::string::npos);
  for(const char *invalid : {"-1", "5000.01", "NaN", "Infinity", "2000abc", ""}) {
    diagnostic_server.arguments["value"] = invalid;
    session_handle_config_update(false);
    assert(diagnostic_server.status == 400 || diagnostic_server.status == 422);
    assert(near(spin_ramp_start_rpm, 2500));
  }
  for(const char *valid : {"0", "5000"}) {
    diagnostic_server.arguments["value"] = valid;
    session_handle_config_update(false);
    assert(diagnostic_server.status == 200);
  }
  diagnostic_server.arguments.erase("value");
  session_handle_config_update(true);
  assert(diagnostic_server.status == 200);
  assert(near(spin_ramp_start_rpm, DEFAULT_SPIN_RAMP_START_RPM));
  assert(near(acc_rate, coefficient_before_knee_save));
  assert(near(config_store.getFloat("rpm_offset", 0), 1.06f));

  // Accepted fractional settings survive NVS reload and API serialization.
  diagnostic_server.arguments = {{"name", "spin_ramp_start_rpm"}, {"value", "1234.567"}};
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(spin_ramp_start_rpm, 1234.567f));
  diagnostic_server.arguments = {{"name", "acc_rate"}, {"value", ".055"}};
  session_handle_config_update(false);
  assert(diagnostic_server.status == 200 && near(acc_rate, .055f));
  acc_rate = 1; spin_ramp_start_rpm = 0;
  load_drive_tuning_settings(); load_spin_ramp_setting();
  assert(near(acc_rate, .055f) && near(spin_ramp_start_rpm, 1234.567f));
  session_send_config_json();
  const auto json_number = [&](const char *name) {
    const std::string marker = std::string("\"") + name + "\":";
    const size_t position = diagnostic_server.response.find(marker);
    assert(position != std::string::npos);
    return strtof(diagnostic_server.response.c_str() + position + marker.size(), nullptr);
  };
  assert(near(json_number("acc_rate"), .055f));
  assert(near(json_number("spin_ramp_start_rpm"), 1234.567f));

  // Export actual compiled float32 results for the offline JavaScript graph test.
  assert(argc == 2);
  FILE *fixture = std::fopen(argv[1], "w");
  assert(fixture != nullptr);
  session_send_config_json();
  std::fprintf(fixture, "{\"configuration\":%s,\"curves\":[", diagnostic_server.response.c_str());
  bool first = true;
  for(float start_rpm : {0.0f, 500.0f, DEFAULT_SPIN_RAMP_START_RPM, 2500.0f, 5000.0f}) {
    spin_ramp_start_rpm = start_rpm;
    for(float rate : {.01f, .05f, .2f, 1.0f}) {
      acc_rate = rate;
      for(int measured_rpm : {-100, 0, 500, 1000, 1004, 1500, 2000, 3000,
                              5000, 10000, (std::numeric_limits<int>::max)()}) {
        std::fprintf(fixture, "%s{\"rate\":%.9g,\"start\":%.9g,\"rpm\":%d,\"ceiling\":%d}",
                     first ? "" : ",", rate, start_rpm, measured_rpm,
                     spin_acceleration_ceiling(measured_rpm));
        first = false;
      }
    }
  }
  std::fputs("]}", fixture); std::fclose(fixture);
  std::puts("PASS: independent ramp knee/coefficient, curve monotonicity/clipping, migration/persistence, output normalization/scaling, RC/state safety, signed translation, telemetry and web save/reset/validation");
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
    subprocess.run([str(exe), str(build / "spin-curve-fixture.json")], check=True)


if __name__ == "__main__":
    main()
