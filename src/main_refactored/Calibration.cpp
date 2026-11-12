#include "Calibration.h"
#include "Controller.h"
#include <algorithm>
#include <cmath>

namespace calibration
{
void begin(Session &session, const RuntimeConfig &snapshot, std::uint32_t sensor_generation)
{
  session = Session{};
  session.active = true;
  session.snapshot = snapshot;
  session.snapshot_valid = true;
  session.entry_sensor_fault_generation = sensor_generation;
}

void finish(Session &session, bool saved)
{
  session = Session{};
  session.wait_for_low = !saved;
}

ConfigExitReason fault_reason(const Session &session, const Safety &safety)
{
  if(!safety.rc_healthy) return CONFIG_EXIT_RC_LOSS;
  if(safety.battery_critical) return CONFIG_EXIT_LOW_BATTERY;
  if(!safety.sensor_healthy || (session.active && safety.sensor_fault_generation !=
      session.entry_sensor_fault_generation)) return CONFIG_EXIT_SENSOR_FAULT;
  if(safety.motor_fault) return CONFIG_EXIT_MOTOR_FAULT;
  return CONFIG_EXIT_COMMIT;
}

Decision update(Session &session, const Safety &safety, bool switch_high,
                bool controls_safe, bool rearm_required, std::uint32_t now_ms,
                std::uint32_t entry_hold_ms)
{
  const ConfigExitReason fault = fault_reason(session, safety);
  if(fault != CONFIG_EXIT_COMMIT)
  {
    session.entry_pending = false;
    return {session.active ? Action::Exit : Action::None, fault};
  }
  if(session.active && rearm_required)
    return {Action::Exit, CONFIG_EXIT_MOTOR_FAULT};
  if(!switch_high)
  {
    session.entry_pending = false;
    if(session.active) return {Action::Exit, CONFIG_EXIT_COMMIT};
    if(controls_safe) session.wait_for_low = false;
    return {Action::None, CONFIG_EXIT_COMMIT};
  }
  if(session.active || session.wait_for_low) return {Action::None, CONFIG_EXIT_COMMIT};
  if(!controls_safe || rearm_required)
  {
    session.entry_pending = false;
    return {Action::None, CONFIG_EXIT_COMMIT};
  }
  if(!session.entry_pending)
  {
    session.entry_pending = true;
    session.entry_start_ms = now_ms;
  }
  if(static_cast<std::uint32_t>(now_ms - session.entry_start_ms) >= entry_hold_ms)
  {
    session.entry_pending = false;
    return {Action::Enter, CONFIG_EXIT_COMMIT};
  }
  return {Action::None, CONFIG_EXIT_COMMIT};
}

bool commit_allowed(const Session &session, const Safety &safety,
                    ConfigExitReason reason, bool switch_high)
{
  return session.active && reason == CONFIG_EXIT_COMMIT && !switch_high &&
         fault_reason(session, safety) == CONFIG_EXIT_COMMIT;
}

Gesture update_gesture(Session &session, int save_channel, std::uint32_t now_ms,
                       std::uint32_t window_ms, int heading_presses, int clear_presses)
{
  Gesture action = Gesture::None;
  if(session.stage == CONFIG_STAGE_RPM && session.gesture_pending &&
     static_cast<std::uint32_t>(now_ms - session.press_start_ms) > window_ms)
  {
    const int count = session.press_count;
    session.press_count = 0;
    session.gesture_pending = false;
    if(count >= heading_presses)
    {
      session.stage = CONFIG_STAGE_HEADING;
      action = Gesture::EnterHeading;
    }
  }
  if(save_channel > 50 && !session.save_latched)
  {
    session.save_latched = true;
    if(session.stage == CONFIG_STAGE_HEADING)
      return action == Gesture::EnterHeading ? action : Gesture::HeadingReminder;
    if(!session.gesture_pending)
    {
      session.gesture_pending = true;
      session.press_start_ms = now_ms;
      session.press_count = 0;
    }
    ++session.press_count;
    if(session.press_count >= clear_presses)
    {
      session.press_count = 0;
      session.gesture_pending = false;
      return Gesture::ClearRpm;
    }
    if(session.press_count < heading_presses) return Gesture::CapturePoint;
  }
  else if(save_channel < 40) session.save_latched = false;
  return action;
}

AccelZeroReason zero_cancel_reason(const Safety &safety, bool switch_high, bool controls_safe)
{
  if(!safety.rc_healthy) return ACCEL_ZERO_RC_LOSS;
  if(safety.battery_critical) return ACCEL_ZERO_LOW_BATTERY;
  if(!safety.sensor_healthy) return ACCEL_ZERO_SENSOR_FAULT;
  if(safety.motor_fault) return ACCEL_ZERO_MOTOR_FAULT;
  if(!switch_high) return ACCEL_ZERO_SWITCH_RELEASED;
  if(!controls_safe) return ACCEL_ZERO_CONTROLS_CHANGED;
  return ACCEL_ZERO_SUCCESS;
}

bool add_zero_sample(ZeroAccumulator &zero, float x_g, float y_g)
{
  if(!std::isfinite(x_g) || !std::isfinite(y_g)) return false;
  zero.x_sum += x_g;
  zero.y_sum += y_g;
  ++zero.samples;
  return true;
}

bool finish_zero(const ZeroAccumulator &zero, int required_samples, float &x_g, float &y_g)
{
  if(required_samples <= 0 || zero.samples != required_samples) return false;
  const float x = static_cast<float>(zero.x_sum / zero.samples);
  const float y = static_cast<float>(zero.y_sum / zero.samples);
  if(!std::isfinite(x) || !std::isfinite(y) || std::fabs(x) > 20.0f || std::fabs(y) > 20.0f)
    return false;
  x_g = x;
  y_g = y;
  return true;
}

void adjust(ConfigCalibrationStage stage, int lr_channel, int deadzone,
            float rpm_rate, float heading_rate, float elapsed_seconds,
            float &rpm_offset, float &heading_offset)
{
  const float dt = std::max(0.0f, std::min(0.05f, elapsed_seconds));
  if(stage == CONFIG_STAGE_HEADING)
  {
    heading_offset = controller::wrap_degrees(heading_offset -
        controller::centered(lr_channel, deadzone) / 50.0f * heading_rate * dt);
    return;
  }
  const int centered = 50 - lr_channel;
  const int magnitude = std::abs(centered);
  float fraction = magnitude > deadzone ? (magnitude - deadzone) / static_cast<float>(50 - deadzone) : 0.0f;
  if(centered < 0) fraction = -fraction;
  rpm_offset = std::max(0.8f, std::min(1.2f, rpm_offset + fraction * rpm_rate * dt));
}
} // namespace calibration
