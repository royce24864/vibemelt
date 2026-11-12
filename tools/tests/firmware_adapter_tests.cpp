#include "Controller.h"
#include "Estimator.h"
#include "Calibration.h"
#include "Config_Store.h"
#include "Phase_Led_State.h"
#include "Motor_Timing.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <cmath>

using String = std::string;
#define DEBUG_LOG(...) ((void)0)
#define DEBUG_LOGF(...) ((void)0)
constexpr int NUM_CHANNELS = 8, MAX_CORRECTION_POINTS = 8;
constexpr int CH_TRANSLATE_LR = 1, CH_TRANSLATE_FB = 2, CH_SPIN_THROTTLE = 3;
constexpr int CH_HEADING_YAW = 4, CH_SPIN_DIR = 5, CH_WIFI_MODE = 6, CH_CONFIG_MODE = 7, CH_CONFIG_SAVE = 8;
constexpr int stick_deadzone_percent = 8, throttle_low_percent = 5, ACCEL_MAX_SCALE = 400;
constexpr int RC_VALID_MIN_US = 900, RC_VALID_MAX_US = 2100, ACCEL_READ_FAILURE_LIMIT = 5;
constexpr unsigned long RC_FRAME_TIMEOUT_MS = 250, mode_switch_hold_ms = 1000, ACCEL_SAMPLE_INTERVAL_US = 2500;
float acc_rate, percentdecel, translation_scale_factor, offset, led_heading_offset_deg;
float accel_zero_x_g_offset, accel_zero_y_g_offset, correction_rpm[8], correction_offset[8];
int correction_point_count, correction_throttle[8], duty[9], rpm, motorL, motorR, max_rpm, max_gforce;
bool motor_on, rc_status, battery_critical_latched, accel_sensor_healthy, dshot_outputs_ready;
bool config_lifecycle_output_inhibited, wifi_mode_active, reversed, control_rearm_required;
controller::RearmState control_rearm_state;
calibration::Session calibration_session;
bool &config_mode = calibration_session.active;
ConfigCalibrationStage &config_calibration_stage = calibration_session.stage;
volatile bool &config_reentry_wait_for_low = calibration_session.wait_for_low;
bool pending_translation_enabled, reset_accel_filter_requested, reset_rotation_phase_requested, current_gforce_valid;
float pending_translation_magnitude, rpm_variation_ewma, current_gforce;
uint32_t accel_fault_generation, accel_sample_sequence;
unsigned long last_heading_update_ms, wifi_entry_high_start_ms, last_config_adjust_ms, rpm_stable_since_ms;
unsigned long led_feedback_until_ms, last_accel_sample_ms, last_valid_rc_frame_ms, rec_gap, accel_loss_started_ms;
int last_rec, accel_read_failure_count;
unsigned invalid_rc_frame_count, total_invalid_rc_frame_count;
int64_t loop_time, loop_start, loopstart, last_accel_sample_us;
estimator::State accel_estimator;
const estimator::Config accel_estimator_config{1, 0, 100, 0.4084f, 0.1056f, 750};
RobotState robot_state;
RuntimeConfig saved_config;
#include "adapter_prototypes.inc"

unsigned long clock_ms;
int polls, cancel_on_poll, fail_sample, samples, saves, watchdog_resets, config_loops, rearm_requests;
unsigned long sensor_read_delay_ms, receiver_poll_delay_ms;
bool storage_success, sensor_configuration_valid;
String feedback, LEDStatus;
motor_timing::Safety motor_safety;
int64_t published_us, published_sensor_us;
bool published_enabled;

unsigned long millis() { return clock_ms; }
int64_t esp_timer_get_time() { return static_cast<int64_t>(clock_ms) * 1000; }
void delay(unsigned long ms)
{
  // The motor task continues observing the old snapshot during blocking I/O.
  for(unsigned long elapsed = 0; elapsed < ms; ++elapsed)
  {
    ++clock_ms;
    motor_safety.check_inputs(esp_timer_get_time(), published_us, published_sensor_us,
                              published_enabled && motor_output_gate_open(), true);
  }
}
void reset_loop_task_watchdog() { ++watchdog_resets; }
void get_battery() {}
void update_accel_sensor_health() {}
void session_note_rc_status(bool) {}
void session_note_sensor_health(bool) {}
void service_wifi_mode() {}
void service_session_logger() {}
void update_heading_reference() {}
bool wifi_entry_requested() { return false; }
void wifi_mode() {}
void config_loop() { ++config_loops; }
void report_motor_output_status() {}
void spin() { motorL = motorR = controller::spin_demand(duty[3], rpm, acc_rate, reversed); }
void translate_field_oriented() { spin(); }
bool translation_active() { return controller::translation_active(duty[1], duty[2], 8); }
bool set_phase_led_mode(PhaseLedMode) { return true; }
bool motor_output_fault_latched() { return motor_safety.fault(); }
void request_motor_output_rearm() { ++rearm_requests; }
void update_motors()
{
  published_us = esp_timer_get_time();
  published_sensor_us = last_accel_sample_us;
  published_enabled = motor_output_gate_open();
}
void updateLED() {}
void show_led_feedback(String status, unsigned long, int = 0, int = -1) { feedback = status; }
void set_robot_state(int value) { robot_state = static_cast<RobotState>(value); }
struct Receiver
{
  int cnt_rec = 0;
  int frame[8] = {};
  bool auto_frame = true;
  void loop()
  {
    delay(receiver_poll_delay_ms);
    if(++polls == cancel_on_poll) frame[CH_CONFIG_MODE - 1] = 1000;
    if(auto_frame) ++cnt_rec;
  }
  int readChannel(int channel) { return frame[channel]; }
} IBus;
long map(long x, long a, long b, long c, long d) { return (x - a) * (d - c) / (b - a) + c; }
template<class T, class L, class H> T constrain(T value, L lo, H hi) { return value < lo ? lo : (value > hi ? hi : value); }
bool read_accel_axes_checked(int16_t &x, int16_t &y, int16_t &z)
{
  delay(sensor_read_delay_ms);
  x = 100; y = -200; z = 0;
  return ++samples != fail_sample;
}
bool accel_configuration_valid() { return sensor_configuration_valid; }
struct Sensor { float convertToG(int, int16_t raw) { return raw / 100.0f; } } xl;
bool save_config_settings()
{
  ++saves;
  assert(motorL == 0 && motorR == 0 && !motor_on && !pending_translation_enabled);
  assert(config_lifecycle_output_inhibited && !published_enabled);
  delay(200); // Flash latency must not age an enabled drive snapshot.
  saved_config = capture_runtime_config();
  return storage_success;
}

#include "adapters_under_test.inc"

void set_channel(int channel, int value)
{
  duty[channel] = value;
  IBus.frame[channel - 1] = 1000 + value * 10;
}
void reset_fixture()
{
  apply_runtime_config(config_record::defaults());
  for(int channel = 1; channel <= 8; ++channel) set_channel(channel, 50);
  set_channel(CH_SPIN_THROTTLE, 0);
  set_channel(CH_SPIN_DIR, 0);
  set_channel(CH_CONFIG_MODE, 100);
  rc_status = accel_sensor_healthy = sensor_configuration_valid = storage_success = dshot_outputs_ready = true;
  battery_critical_latched = wifi_mode_active = reversed = config_lifecycle_output_inhibited = false;
  calibration_session = {};
  control_rearm_state.required = control_rearm_required = false;
  pending_translation_enabled = motor_on = false;
  reset_accel_filter_requested = reset_rotation_phase_requested = false;
  current_gforce_valid = true;
  pending_translation_magnitude = 0;
  motorL = motorR = rpm = 0;
  accel_fault_generation = accel_sample_sequence = 0;
  polls = cancel_on_poll = fail_sample = samples = saves = watchdog_resets = config_loops = rearm_requests = 0;
  accel_read_failure_count = 0;
  sensor_read_delay_ms = receiver_poll_delay_ms = 0;
  clock_ms = 100;
  last_valid_rc_frame_ms = last_accel_sample_ms = clock_ms;
  loopstart = last_accel_sample_us = esp_timer_get_time();
  motor_safety = {};
  estimator::reset(accel_estimator);
  float sample = 0;
  estimator::accept_sample(accel_estimator, accel_estimator_config, 1, -2, 0, 0, clock_ms, sample);
  robot_state = ROBOT_ARMED_IDLE;
  IBus.auto_frame = true;
  IBus.cnt_rec = last_rec = 0;
  accel_loss_started_ms = 0;
  feedback.clear();
  update_motors();
}

int main()
{
  reset_fixture();
  enter_config_mode();
  assert(config_mode && feedback == "config zeroed" && samples == 101);
  assert(accel_zero_x_g_offset == 1 && accel_zero_y_g_offset == -2);
  assert(!motor_output_fault_latched() && !config_lifecycle_output_inhibited);
  assert(last_accel_sample_us == esp_timer_get_time() && watchdog_resets >= 101);
  update_control_rearm();
  update_robot_state();
  assert(!control_rearm_required && robot_state == ROBOT_CONFIG);

  reset_fixture();
  calibration_session.entry_pending = true;
  calibration_session.entry_start_ms = clock_ms - 1000U;
  loop(); // Actual outer dispatch must reach config after the blocking zero.
  assert(config_mode && config_loops == 1 && !motor_output_fault_latched());

  reset_fixture();
  fail_sample = 12;
  enter_config_mode();
  assert(!config_mode && saves == 0 && feedback == "config zero failed");
  assert(config_reentry_wait_for_low && control_rearm_required && accel_zero_x_g_offset == 0);

  reset_fixture();
  cancel_on_poll = 101;
  enter_config_mode();
  assert(!config_mode && saves == 0 && feedback == "config aborted");
  assert(config_reentry_wait_for_low && accel_zero_x_g_offset == 0);

  reset_fixture();
  enter_config_mode();
  acc_rate = 0.8f;
  set_channel(CH_CONFIG_MODE, 0);
  update_config_mode();
  assert(saves == 1 && !config_mode && feedback == "config final saved");
  assert(saved_config.acc_rate == 0.8f && !motor_output_fault_latched());

  reset_fixture();
  enter_config_mode();
  fail_sample = samples + 1; // First estimator read after a successful flash save fails.
  set_channel(CH_CONFIG_MODE, 0);
  update_config_mode();
  assert(saves == 1 && !config_mode && feedback == "config final saved");
  assert(control_rearm_required && !published_enabled && !motor_output_fault_latched());

  reset_fixture();
  enter_config_mode();
  acc_rate = 0.8f;
  storage_success = false;
  set_channel(CH_CONFIG_MODE, 0);
  update_config_mode();
  assert(saves == 1 && !config_mode && feedback == "config save failed");
  assert(acc_rate == 0.4f && accel_zero_x_g_offset == 0 && control_rearm_required);

  reset_fixture();
  enter_config_mode();
  rc_status = false;
  set_channel(CH_CONFIG_MODE, 0); // Synthetic failsafe CH7 must never commit.
  update_config_mode();
  assert(!config_mode && saves == 0 && config_reentry_wait_for_low);

  reset_fixture();
  set_channel(CH_CONFIG_MODE, 0);
  set_channel(CH_SPIN_DIR, 100);
  loop();
  assert(reversed && robot_state == ROBOT_ARMED_IDLE);
  set_channel(CH_SPIN_THROTTLE, 80);
  loop();
  assert(reversed && robot_state == ROBOT_SPINUP && motorL < 0);

  reset_fixture();
  set_channel(CH_CONFIG_MODE, 0);
  set_channel(CH_SPIN_THROTTLE, 80);
  clock_ms += 251;
  update_channels(); // Buffered recovery frame must not erase the gap.
  assert(rc_status && control_rearm_required);
  calcrpm();
  update_control_rearm();
  assert(control_rearm_required);
  update_robot_state();
  assert(robot_state == ROBOT_ARMED_IDLE);

  reset_fixture();
  set_channel(CH_CONFIG_MODE, 0);
  set_channel(CH_SPIN_THROTTLE, 80);
  clock_ms += 26;
  calcrpm(); // A successful sensor read also must not erase a stale interval.
  assert(accel_sensor_healthy && control_rearm_required);
  update_control_rearm();
  assert(control_rearm_required);
  set_channel(CH_SPIN_THROTTLE, 0);
  update_control_rearm();
  assert(!control_rearm_required);

  reset_fixture();
  set_channel(CH_SPIN_THROTTLE, 80);
  clock_ms += 3;
  sensor_read_delay_ms = 26;
  calcrpm(); // A slow successful read crosses expiry after the initial age check.
  assert(control_rearm_required && last_accel_sample_us == esp_timer_get_time());

  reset_fixture();
  set_channel(CH_SPIN_THROTTLE, 80);
  receiver_poll_delay_ms = 251;
  update_channels();
  assert(rc_status && control_rearm_required);

  reset_fixture();
  record_accel_loss("test transient");
  accel_sensor_healthy = true; // Same-turn recovery still requires neutral.
  assert(control_rearm_required && last_accel_sample_us == 0);
  set_channel(CH_SPIN_THROTTLE, 80);
  clock_ms += 3;
  calcrpm();
  update_control_rearm();
  assert(control_rearm_required);
  std::puts("PASS: real sketch adapters; zero/flash freshness interlock, save rollback, RC and sensor gap recovery, idle direction and outer dispatch");
}
