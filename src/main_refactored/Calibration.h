#pragma once
#include "Config_Store.h"
#include <cstdint>

enum ConfigCalibrationStage : std::uint8_t { CONFIG_STAGE_RPM, CONFIG_STAGE_HEADING };
enum ConfigExitReason : std::uint8_t
{
  CONFIG_EXIT_COMMIT, CONFIG_EXIT_RC_LOSS, CONFIG_EXIT_SENSOR_FAULT,
  CONFIG_EXIT_LOW_BATTERY, CONFIG_EXIT_ZERO_FAILED, CONFIG_EXIT_ZERO_CANCELLED,
  CONFIG_EXIT_CONTROLS_CHANGED, CONFIG_EXIT_MOTOR_FAULT
};
enum AccelZeroReason : std::uint8_t
{
  ACCEL_ZERO_SUCCESS, ACCEL_ZERO_RC_LOSS, ACCEL_ZERO_SENSOR_FAULT,
  ACCEL_ZERO_LOW_BATTERY, ACCEL_ZERO_SWITCH_RELEASED, ACCEL_ZERO_CONTROLS_CHANGED,
  ACCEL_ZERO_INVALID_OFFSET, ACCEL_ZERO_MOTOR_FAULT
};
struct AccelZeroResult { AccelZeroReason reason; int valid_samples; int requested_samples; };

namespace calibration
{
struct Safety
{
  bool rc_healthy;
  bool sensor_healthy;
  bool battery_critical;
  bool motor_fault;
  std::uint32_t sensor_fault_generation;
};
struct Session
{
  bool active = false;
  bool wait_for_low = false;
  ConfigCalibrationStage stage = CONFIG_STAGE_RPM;
  bool snapshot_valid = false;
  RuntimeConfig snapshot = {};
  std::uint32_t entry_sensor_fault_generation = 0;
  bool entry_pending = false;
  std::uint32_t entry_start_ms = 0;
  bool save_latched = false;
  bool gesture_pending = false;
  int press_count = 0;
  std::uint32_t press_start_ms = 0;
};
enum class Action { None, Enter, Exit };
struct Decision { Action action; ConfigExitReason reason; };
enum class Gesture { None, CapturePoint, ClearRpm, EnterHeading, HeadingReminder };
struct ZeroAccumulator { double x_sum = 0.0; double y_sum = 0.0; int samples = 0; };

void begin(Session &session, const RuntimeConfig &snapshot, std::uint32_t sensor_generation);
void finish(Session &session, bool saved);
ConfigExitReason fault_reason(const Session &session, const Safety &safety);
Decision update(Session &session, const Safety &safety, bool switch_high,
                bool controls_safe, bool rearm_required, std::uint32_t now_ms,
                std::uint32_t entry_hold_ms);
bool commit_allowed(const Session &session, const Safety &safety,
                    ConfigExitReason reason, bool switch_high);
Gesture update_gesture(Session &session, int save_channel, std::uint32_t now_ms,
                       std::uint32_t window_ms, int heading_presses, int clear_presses);
AccelZeroReason zero_cancel_reason(const Safety &safety, bool switch_high, bool controls_safe);
bool add_zero_sample(ZeroAccumulator &zero, float x_g, float y_g);
bool finish_zero(const ZeroAccumulator &zero, int required_samples, float &x_g, float &y_g);
void adjust(ConfigCalibrationStage stage, int lr_channel, int deadzone,
            float rpm_rate, float heading_rate, float elapsed_seconds,
            float &rpm_offset, float &heading_offset);
} // namespace calibration
