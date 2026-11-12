// Runs actual firmware functions, extracted by run_io_tests.py. These mocks
// provide hardware/RTOS boundaries and deterministic preemption points.
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Phase_Led_State.h"
#include "Session_Logger.h"
#include "Timeline_Cursor.h"
#include "Controller.h"

using std::min;
using std::max;
#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

using portMUX_TYPE = int;
using TaskHandle_t = void *;
using SemaphoreHandle_t = void *;
constexpr int portMUX_INITIALIZER_UNLOCKED = 0;
constexpr int pdTRUE = 1;
constexpr unsigned portMAX_DELAY = UINT32_MAX;
TaskHandle_t phase_led_task_handle = reinterpret_cast<void *>(1);
SemaphoreHandle_t led_output_mutex = reinterpret_cast<void *>(2);
uint32_t phase_led_render_count = 0;
int critical_depth = 0;
bool strip_mutex_held = false;
unsigned notifications = 0;
unsigned notification_takes = 0;
std::function<void()> at_critical_exit;
std::function<void()> at_notify;
std::function<void()> during_spi;
std::vector<PhaseLedFrame> rendered_frames;

void invoke_once(std::function<void()> &hook)
{
  auto invoke = std::move(hook);
  hook = {};
  if(invoke) invoke();
}

void portENTER_CRITICAL(portMUX_TYPE *)
{
  assert(critical_depth == 0);
  critical_depth++;
}

void portEXIT_CRITICAL(portMUX_TYPE *)
{
  assert(critical_depth == 1);
  critical_depth--;
  invoke_once(at_critical_exit);
}

void xTaskNotifyGive(TaskHandle_t handle)
{
  assert(handle != nullptr && critical_depth == 0);
  notifications++;
  invoke_once(at_notify);
}

struct TaskStopped {};
unsigned ulTaskNotifyTake(int, unsigned)
{
  assert(critical_depth == 0 && !strip_mutex_held);
  if(notification_takes++ != 0) throw TaskStopped{};
  return 1;
}

int xSemaphoreTake(SemaphoreHandle_t, unsigned)
{
  assert(critical_depth == 0 && !strip_mutex_held);
  strip_mutex_held = true;
  return pdTRUE;
}

void xSemaphoreGive(SemaphoreHandle_t)
{
  assert(critical_depth == 0 && strip_mutex_held);
  strip_mutex_held = false;
}

void taskYIELD()
{
  assert(critical_depth == 0 && !strip_mutex_held);
}

void render_phase_led_frame(PhaseLedFrame frame)
{
  assert(critical_depth == 0 && strip_mutex_held);
  rendered_frames.push_back(frame);
  invoke_once(during_spi);
}

#include "firmware_io_led.inc"

// Logger dependencies. Sensor/motor values below are sampled by the real
// session_capture_current_sample body, including its limiter flag handling.
TelemetrySample session_samples[SESSION_SAMPLE_CAPACITY];
uint32_t session_sample_head = 0;
uint32_t session_sample_count = 0;
uint64_t session_total_samples = 0;
SessionSummary session_summary = {};
bool session_initialized = true;
bool session_recording_frozen = false;
bool session_events_frozen = false;
int64_t session_epoch_us = 0;
int64_t mock_time_us = 0;
unsigned mock_time_reads = 0;
unsigned mock_motor_snapshots = 0;
std::function<void(unsigned)> before_time_read;
int8_t session_last_left_average = 0;
int8_t session_last_right_average = 0;
bool session_last_translation_seen = false;
int rpm = 1000;
float acc_rate = 0.4f;
float current_gforce = 42.0f;
float volts = 12.3f;
float rpm_variation_ewma = 5.0f;
float spinspeed = 499.5f;
bool current_gforce_valid = true;
bool accel_sensor_healthy = true;
bool rc_status = true;
int detected_battery_cells = 3;
constexpr float BATTERY_UNPLUGGED_THRESHOLD = 5.5f;
constexpr int CH_TRANSLATE_LR = 1;
constexpr int CH_TRANSLATE_FB = 2;
constexpr int CH_SPIN_THROTTLE = 3;
int duty[] = {0, 50, 50, 70};
RobotState robot_state = ROBOT_MELTY;

SessionState session_state_code() { return SESSION_MELTY; }
bool motor_output_gate_open() { return true; }
int64_t esp_timer_get_time()
{
  mock_time_reads++;
  if(before_time_read) before_time_read(mock_time_reads);
  return mock_time_us;
}
void reset_loop_task_watchdog() {}
void yield() {}
void snapshot_motor_output_interval(int &left, int &right, bool &translation)
{
  mock_motor_snapshots++;
  left = 400;
  right = 600;
  translation = false;
}
struct MockEsp
{
  uint32_t getFreeHeap() const { return 100000; }
} ESP;

struct MockArgument
{
  long toInt() const { return 0; }
};

struct MockDiagnosticServer
{
  std::string response;
  bool complete = false;
  bool hasArg(const char *) const { return false; }
  MockArgument arg(const char *) const { return {}; }
  void sendHeader(const char *, const char *) {}
  void chunkResponseBegin(const char *) { response.clear(); complete = false; }
  void chunkResponseEnd() { complete = true; }
} diagnostic_server;

void session_chunk_write(const char *data, size_t size)
{
  diagnostic_server.response.append(data, size);
}

#include "firmware_io_logger.inc"

void reset_led()
{
  phase_led_state = {PHASE_LED_DISABLED, PHASE_LED_FRAME_INVALID, 0};
  phase_led_task_handle = reinterpret_cast<void *>(1);
  critical_depth = 0;
  strip_mutex_held = false;
  notifications = notification_takes = phase_led_render_count = 0;
  at_critical_exit = at_notify = during_spi = {};
  rendered_frames.clear();
}

void render_one_task_iteration()
{
  try { phase_led_output_task(nullptr); }
  catch(const TaskStopped &) {}
  assert(critical_depth == 0 && !strip_mutex_held);
}

void test_mailbox_mode_and_notification()
{
  reset_led();
  assert(set_phase_led_mode(PHASE_LED_MELTY));
  auto mode = read_phase_led_snapshot();
  assert(mode.mode == PHASE_LED_MELTY && mode.frame == PHASE_LED_FRAME_INVALID);
  assert(mode.transition == 1);
  at_notify = [] {
    auto state = read_phase_led_snapshot();
    assert(state.mode == PHASE_LED_MELTY);
    assert(state.frame == PHASE_LED_FRAME_TOP_HEADING && state.transition == 2);
  };
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
  assert(notifications == 1);
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
  assert(notifications == 1 && read_phase_led_snapshot().transition == 2);
  assert(!set_phase_led_mode(PHASE_LED_DISABLED));
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_OFF);
  auto disabled = read_phase_led_snapshot();
  assert(disabled.mode == PHASE_LED_DISABLED && disabled.frame == PHASE_LED_FRAME_INVALID);
  assert(disabled.transition == 3 && notifications == 1);
  assert(set_phase_led_mode(PHASE_LED_CONFIG_STABLE));
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
  assert(read_phase_led_snapshot().frame == PHASE_LED_FRAME_INVALID);
  publish_phase_led_frame(PHASE_LED_CONFIG_STABLE, PHASE_LED_FRAME_CONFIG_STABLE);
  assert(read_phase_led_snapshot().frame == PHASE_LED_FRAME_CONFIG_STABLE);
  phase_led_task_handle = nullptr;
  assert(!set_phase_led_mode(PHASE_LED_MELTY));
  assert(read_phase_led_snapshot().mode == PHASE_LED_DISABLED);
}

void test_snapshot_preemption_keeps_frame_and_sequence_paired()
{
  reset_led();
  set_phase_led_mode(PHASE_LED_MELTY);
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
  auto before = read_phase_led_snapshot();
  // A motor tick becomes runnable immediately after the reader releases its
  // critical section. The returned snapshot must still be entirely old.
  at_critical_exit = [] {
    publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_OFF);
  };
  auto captured = read_phase_led_snapshot();
  auto latest = read_phase_led_snapshot();
  assert(captured.frame == PHASE_LED_FRAME_TOP_HEADING);
  assert(captured.transition == before.transition);
  assert(latest.frame == PHASE_LED_FRAME_OFF);
  assert(latest.transition == before.transition + 1);
}

void test_closing_edge_at_snapshot_and_during_spi()
{
  for(bool preempt_snapshot : {true, false})
  {
    reset_led();
    set_phase_led_mode(PHASE_LED_MELTY);
    publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
    auto close_window = [] {
      publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_OFF);
    };
    if(preempt_snapshot) at_critical_exit = close_window;
    else during_spi = close_window;
    render_one_task_iteration();
    assert((rendered_frames == std::vector<PhaseLedFrame>{
        PHASE_LED_FRAME_TOP_HEADING, PHASE_LED_FRAME_OFF}));
    assert(phase_led_render_count == 2);
  }
}

void test_coalesced_notifications_and_disabled_output()
{
  reset_led();
  set_phase_led_mode(PHASE_LED_MELTY);
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_TOP_HEADING);
  publish_phase_led_frame(PHASE_LED_MELTY, PHASE_LED_FRAME_OFF);
  render_one_task_iteration();
  assert((rendered_frames == std::vector<PhaseLedFrame>{PHASE_LED_FRAME_OFF}));
  rendered_frames.clear();
  notification_takes = 0;
  set_phase_led_mode(PHASE_LED_DISABLED);
  render_one_task_iteration();
  assert(rendered_frames.empty());
}

std::vector<std::string> split(const std::string &text, char delimiter)
{
  std::vector<std::string> result;
  std::istringstream stream(text);
  for(std::string part; std::getline(stream, part, delimiter);) result.push_back(part);
  return result;
}

void reset_logger()
{
  memset(session_samples, 0, sizeof(session_samples));
  session_sample_head = session_sample_count = 0;
  session_total_samples = 0;
  session_summary = {};
  session_initialized = true;
  session_recording_frozen = session_events_frozen = false;
  session_epoch_us = mock_time_us = 0;
  mock_time_reads = mock_motor_snapshots = 0;
  before_time_read = {};
  session_last_left_average = session_last_right_average = 0;
  session_last_translation_seen = false;
  rpm = 1000;
  acc_rate = 0.4f;
}

void test_historical_capture_resume_and_exports()
{
  reset_logger();
  session_capture_current_sample(false);
  auto original = session_sample_at(0);
  assert(original.acceleration_ceiling == 60);
  assert((original.flags & SAMPLE_ACCEL_LIMITER_ACTIVE) != 0);
  // Simulate WiFi changing acc_rate during a frozen log. The actual resume
  // code must backfill unobserved slots and use the new rate only for now.
  session_recording_frozen = session_events_frozen = true;
  acc_rate = 0.8f;
  mock_time_us = 400000;
  session_logger_resume();
  assert(!session_recording_frozen && !session_events_frozen);
  assert(session_sample_count == 5 && session_total_samples == 5);
  assert(session_sample_at(0).acceleration_ceiling == 60);
  assert(session_sample_at(0).flags == original.flags);
  for(unsigned i = 1; i < 4; i++)
  {
    const auto &missed = session_sample_at(i);
    assert(missed.acceleration_ceiling == -1);
    assert(missed.flags == SAMPLE_MISSED_OR_INVALID);
  }
  assert(session_sample_at(4).acceleration_ceiling == 100);
  assert((session_sample_at(4).flags & SAMPLE_ACCEL_LIMITER_ACTIVE) == 0);

  session_handle_timeline_csv(false);
  assert(diagnostic_server.complete);
  auto rows = split(diagnostic_server.response, '\n');
  assert(rows.size() == 6);
  assert(split(rows[1], ',')[10] == "60");
  for(unsigned i = 2; i < 5; i++) assert(split(rows[i], ',')[10] == "-1");
  assert(split(rows[5], ',')[10] == "100");
  session_handle_timeline_json();
  assert(diagnostic_server.complete);
  const auto &json = diagnostic_server.response;
  assert(json.find("\"count\":5") != std::string::npos);
  assert(json.find("\"acceleration_ceiling\":60") != std::string::npos);
  assert(json.find("\"acceleration_ceiling\":-1") != std::string::npos);
  assert(json.find("\"acceleration_ceiling\":100") != std::string::npos);

  // Overflow the real bounded ring and confirm captured values survive wrap.
  for(unsigned i = 0; i < SESSION_SAMPLE_CAPACITY; i++)
  {
    rpm = 1000;
    acc_rate = (i % 2 == 0) ? 0.4f : 0.8f;
    session_capture_current_sample(false);
  }
  assert(session_sample_count == SESSION_SAMPLE_CAPACITY);
  assert(session_sample_at(0).acceleration_ceiling == 60);
  assert(session_sample_at(SESSION_SAMPLE_CAPACITY - 1).acceleration_ceiling == 100);
  assert(sizeof(session_samples) == 54000);
}

void test_timeline_planner_boundaries()
{
  auto none = timeline_catch_up(1, 0, 3000, 64);
  assert(none.skipped_samples == 0 && none.append_samples == 0);
  assert(timeline_catch_up(0, 0, 0, 64).append_samples == 0);
  assert(timeline_catch_up(0, 0, 3000, 0).append_samples == 0);
  // Compile the actual module, and compare its cursor arithmetic with a
  // small exhaustive reference across ring and batch boundaries.
  for(uint32_t capacity = 1; capacity <= 17; capacity++)
    for(uint32_t budget = 1; budget <= 19; budget++)
      for(uint64_t next = 0; next <= 4; next++)
        for(uint64_t pending = 1; pending <= 40; pending++)
        {
          auto result = timeline_catch_up(next, next + pending - 1, capacity, budget);
          uint64_t expected_skip = pending > capacity ? pending - capacity : 0;
          uint32_t expected_append = (uint32_t)min<uint64_t>(pending, min(capacity, budget));
          assert(result.skipped_samples == expected_skip);
          assert(result.append_samples == expected_append);
        }
  auto endpoint = timeline_catch_up(0, UINT64_MAX, 3000, 64);
  assert(endpoint.skipped_samples == UINT64_MAX - 2999);
  assert(endpoint.append_samples == 64);
}

void seed_logger_history(uint32_t count)
{
  for(uint32_t i = 0; i < count; i++)
  {
    rpm = 1000 + i;
    session_capture_current_sample(false);
  }
  rpm = 12345;
  mock_motor_snapshots = 0;
}

void verify_ring(uint64_t initial_total, uint64_t latest_due)
{
  assert(session_sample_count <= SESSION_SAMPLE_CAPACITY);
  assert(session_sample_head == session_total_samples % SESSION_SAMPLE_CAPACITY);
  for(uint32_t i = 0; i < session_sample_count; i++)
  {
    uint64_t sequence = session_sequence_at(i);
    assert(sequence == session_total_samples - session_sample_count + i);
    const auto &sample = session_sample_at(i);
    if(sequence < initial_total)
    {
      assert(sample.rpm == 1000 + sequence);
      assert((sample.flags & SAMPLE_MISSED_OR_INVALID) == 0);
    }
    else if(sequence < latest_due)
    {
      assert(sample.state == SESSION_UNKNOWN);
      assert(sample.flags == SAMPLE_MISSED_OR_INVALID);
      assert(sample.rpm == 0 && sample.acceleration_ceiling == -1);
    }
    else
    {
      assert(sequence == latest_due);
      assert(sample.rpm == 12345 && sample.state == SESSION_MELTY);
      assert((sample.flags & SAMPLE_MISSED_OR_INVALID) == 0);
    }
  }
}

void service_logger_and_check_bound(uint64_t latest_due)
{
  uint64_t previous_total = session_total_samples;
  uint32_t previous_count = session_sample_count;
  unsigned previous_snapshots = mock_motor_snapshots;
  uint64_t pending = latest_due >= previous_total ? latest_due - previous_total + 1 : 0;
  uint64_t skipped = pending > SESSION_SAMPLE_CAPACITY ? pending - SESSION_SAMPLE_CAPACITY : 0;
  uint64_t inserted = min<uint64_t>(pending - skipped, SESSION_SAMPLE_SERVICE_BUDGET);
  service_session_logger();
  assert(session_total_samples == previous_total + skipped + inserted);
  uint32_t expected_count = min<uint64_t>(SESSION_SAMPLE_CAPACITY,
      (skipped ? 0 : previous_count) + inserted);
  assert(session_sample_count == expected_count);
  assert(mock_motor_snapshots - previous_snapshots <= 1);
}

void test_bounded_gaps_preserve_surviving_history()
{
  for(uint32_t history_count : {1u, SESSION_SAMPLE_CAPACITY + 17})
    for(uint32_t pending : {0u, 1u, 63u, 64u, 65u,
                           SESSION_SAMPLE_CAPACITY - 1, SESSION_SAMPLE_CAPACITY,
                           SESSION_SAMPLE_CAPACITY + 1})
    {
      reset_logger();
      seed_logger_history(history_count);
      uint64_t latest_due = history_count + pending - 1;
      mock_time_us = latest_due * SESSION_SAMPLE_INTERVAL_US;
      do
      {
        service_logger_and_check_bound(latest_due);
        verify_ring(history_count, latest_due);
      } while(session_total_samples <= latest_due);
      assert(session_total_samples == latest_due + 1);
      assert(session_sample_count == min<uint64_t>(SESSION_SAMPLE_CAPACITY,
                                                  history_count + pending));
      assert(mock_motor_snapshots == (pending != 0 ? 1u : 0u));
      // With capacity-1 pending rows exactly the last old sample survives.
      if(pending == SESSION_SAMPLE_CAPACITY - 1)
      {
        assert(session_sequence_at(0) == history_count - 1);
        assert(session_sample_at(0).rpm == 1000 + history_count - 1);
      }
    }
}

void test_hours_and_days_are_skipped_arithmetically()
{
  for(uint64_t latest_due : {72000ULL, 864000ULL, 6048000ULL})
  {
    reset_logger();
    seed_logger_history(SESSION_SAMPLE_CAPACITY + 17);
    uint64_t history_count = session_total_samples;
    session_recording_frozen = session_events_frozen = true;
    mock_time_us = latest_due * SESSION_SAMPLE_INTERVAL_US;
    service_session_logger();
    assert(session_total_samples == history_count);
    session_logger_resume();
    assert(!session_recording_frozen && !session_events_frozen);
    assert(session_sample_count == SESSION_SAMPLE_SERVICE_BUDGET);
    assert(session_sequence_at(0) == latest_due + 1 - SESSION_SAMPLE_CAPACITY);
    assert(mock_motor_snapshots == 0);
    verify_ring(history_count, latest_due);
    uint32_t services = 1;
    while(session_total_samples <= latest_due)
    {
      service_logger_and_check_bound(latest_due);
      verify_ring(history_count, latest_due);
      services++;
    }
    assert(services == (SESSION_SAMPLE_CAPACITY + SESSION_SAMPLE_SERVICE_BUDGET - 1) /
                       SESSION_SAMPLE_SERVICE_BUDGET);
    assert(session_sample_count == SESSION_SAMPLE_CAPACITY);
    assert(session_sequence_at(0) == latest_due + 1 - SESSION_SAMPLE_CAPACITY);
    assert(mock_motor_snapshots == 1);
    assert(session_summary.maximum_rpm_sequence == latest_due);
    WindowSummary window;
    session_compute_window_summary(window);
    assert(window.melty_samples == 1 && window.tank_samples == 0 && window.spinup_samples == 0);
    assert(window.maximum_rpm_sequence == latest_due);

    session_handle_timeline_csv(false);
    assert(diagnostic_server.complete);
    auto rows = split(diagnostic_server.response, '\n');
    assert(rows.size() == SESSION_SAMPLE_CAPACITY + 1);
    for(uint32_t i = 0; i < SESSION_SAMPLE_CAPACITY; i++)
    {
      auto columns = split(rows[i + 1], ',');
      char expected_time[40];
      snprintf(expected_time, sizeof(expected_time), "%.2f",
               session_sequence_at(i) * SESSION_SAMPLE_INTERVAL_SECONDS);
      assert(columns[0] == expected_time);
      bool missed = i + 1 < SESSION_SAMPLE_CAPACITY;
      assert(columns[1] == (missed ? "UNKNOWN" : "MELTY"));
      if(missed) assert(columns[10] == "-1" && columns[14] == "0x08");
    }
    session_handle_timeline_csv(true);
    rows = split(diagnostic_server.response, '\n');
    assert(rows.size() == 2); // unknown time must not become MELTY history
  }
}

void test_catchup_tracks_advancing_clock()
{
  reset_logger();
  seed_logger_history(1);
  for(uint64_t latest_due : {100ULL, 150ULL, 160ULL})
  {
    mock_time_us = latest_due * SESSION_SAMPLE_INTERVAL_US;
    service_logger_and_check_bound(latest_due);
    verify_ring(1, latest_due);
  }
  assert(session_total_samples == 161 && mock_motor_snapshots == 1);

  reset_logger();
  seed_logger_history(1);
  mock_time_us = 4 * SESSION_SAMPLE_INTERVAL_US;
  // Time crosses a sampling boundary just before the planned live row.
  before_time_read = [](unsigned read) {
    if(read == 5) mock_time_us = 5 * SESSION_SAMPLE_INTERVAL_US;
  };
  service_session_logger();
  assert(session_total_samples == 5 && mock_motor_snapshots == 0);
  assert(session_sample_at(4).state == SESSION_UNKNOWN);
  assert(session_sample_at(4).acceleration_ceiling == -1);
  before_time_read = {};
  service_logger_and_check_bound(5);
  verify_ring(1, 5);
  assert(session_total_samples == 6 && mock_motor_snapshots == 1);

  // A second multi-day interruption during catch-up must discard any newly
  // obsolete backlog without spending a loop on each intervening interval.
  reset_logger();
  seed_logger_history(1);
  mock_time_us = 864000LL * SESSION_SAMPLE_INTERVAL_US;
  service_logger_and_check_bound(864000);
  assert(session_sample_count == SESSION_SAMPLE_SERVICE_BUDGET);
  mock_time_us = 6048000LL * SESSION_SAMPLE_INTERVAL_US;
  service_logger_and_check_bound(6048000);
  assert(session_sample_count == SESSION_SAMPLE_SERVICE_BUDGET);
  verify_ring(1, 6048000);

  // Frozen recording and negative elapsed clocks do no catch-up work.
  auto previous_total = session_total_samples;
  auto previous_count = session_sample_count;
  session_recording_frozen = true;
  mock_time_us = 2 * 6048000LL * SESSION_SAMPLE_INTERVAL_US;
  service_session_logger();
  assert(session_total_samples == previous_total && session_sample_count == previous_count);
  session_recording_frozen = false;
  mock_time_us = -1;
  service_session_logger();
  assert(session_total_samples == previous_total && session_sample_count == previous_count);
}

void test_ceiling_boundaries()
{
  assert(telemetry_acceleration_ceiling(0, 0.4f) == 20);
  assert(telemetry_acceleration_ceiling(UINT16_MAX, 1.0f) == 100);
  assert(!telemetry_acceleration_limiter_active(100, -1));
  assert(!telemetry_acceleration_limiter_active(20, 60));
  assert(telemetry_acceleration_limiter_active(22, 60));
}

int main()
{
  test_mailbox_mode_and_notification();
  test_snapshot_preemption_keeps_frame_and_sequence_paired();
  test_closing_edge_at_snapshot_and_during_spi();
  test_coalesced_notifications_and_disabled_output();
  test_historical_capture_resume_and_exports();
  test_timeline_planner_boundaries();
  test_bounded_gaps_preserve_surviving_history();
  test_hours_and_days_are_skipped_arithmetically();
  test_catchup_tracks_advancing_clock();
  test_ceiling_boundaries();
  std::puts("PASS: 10 firmware IO regression groups (LED preemption, telemetry capture/export, bounded catch-up/ring/clock).");
}
