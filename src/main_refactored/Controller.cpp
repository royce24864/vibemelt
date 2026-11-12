#include "Controller.h"
#include <algorithm>
#include <cmath>

namespace controller
{
int centered(int value, int deadzone)
{
  const int result = value - 50;
  return std::abs(result) < deadzone ? 0 : result;
}

bool neutral(const Controls &controls, int deadzone, int low_throttle)
{
  return controls.throttle <= low_throttle && centered(controls.translate_lr, deadzone) == 0 &&
         centered(controls.translate_fb, deadzone) == 0 && centered(controls.yaw, deadzone) == 0;
}

bool translation_active(int lr, int fb, int deadzone)
{
  return centered(lr, deadzone) != 0 || centered(fb, deadzone) != 0;
}

bool spin_direction(bool previous, const Controls &controls, bool rc_healthy,
                    bool config_active, bool wifi_active)
{
  if(!rc_healthy || config_active || wifi_active || controls.throttle >= 20 ||
     controls.direction == 50) return previous;
  return controls.direction > 50;
}

RearmResult update_rearm(RearmState &state, const Health &health,
                         bool controls_neutral, bool calibration_wait_for_low,
                         bool special_mode_active)
{
  if(!health.rc || !health.sensor || health.battery_critical || !health.outputs_ready)
  {
    state.required = true;
    return {true, false};
  }
  if(health.motor_fault) state.required = true;
  if(!state.required) return {false, false};
  if(!controls_neutral || calibration_wait_for_low || special_mode_active)
    return {true, false};
  // The motor task clears its own latch only after fresh, accepted zero frames.
  // Keep the producer inhibited until that acknowledgement becomes visible.
  if(health.motor_fault) return {true, true};
  state.required = false;
  return {false, false};
}

RobotState select_state(const Health &health, bool wifi_active, bool config_active,
                        bool rearm_required, bool calibration_wait_for_low,
                        int rpm, const Controls &controls, int deadzone)
{
  if(wifi_active) return ROBOT_WIFI;
  if(health.battery_critical) return ROBOT_LOW_BATTERY;
  if(!health.sensor) return ROBOT_SENSOR_FAULT;
  if(!health.rc) return ROBOT_FAILSAFE;
  if(rearm_required || calibration_wait_for_low || health.motor_fault || !health.outputs_ready)
    return ROBOT_ARMED_IDLE;
  if(config_active) return ROBOT_CONFIG;
  if(rpm > 400 && controls.throttle > 10) return ROBOT_MELTY;
  if(controls.throttle > 10) return ROBOT_SPINUP;
  if(translation_active(controls.translate_lr, controls.translate_fb, deadzone)) return ROBOT_TANK;
  return ROBOT_ARMED_IDLE;
}

bool requires_zero(RobotState state)
{
  return state != ROBOT_TANK && state != ROBOT_SPINUP && state != ROBOT_MELTY && state != ROBOT_CONFIG;
}

int spin_demand(int throttle, int rpm, float acceleration_rate, bool reversed)
{
  const int ceiling = static_cast<int>(acceleration_rate * rpm + 200.0f);
  const int demand = std::min(throttle * 10, ceiling);
  return std::max(-999, std::min(999, reversed ? -demand : demand));
}

MotorDemand tank_demand(int lr, int fb, bool reversed)
{
  if(reversed && (fb > 55 || fb < 45)) return {1000 - fb * 20, fb * 20 - 1000};
  // Integer division intentionally matches Arduino map(), including truncation.
  const int steering = (lr * -40) / 100 + 20;
  return {100 - fb * 2 + steering, fb * 2 - 100 + steering};
}

float wrap_degrees(float degrees)
{
  if(!std::isfinite(degrees)) return 0.0f;
  const float value = std::fmod(degrees, 360.0f);
  return value < 0.0f ? value + 360.0f : value;
}

Translation translation(int lr, int fb, float heading, int deadzone)
{
  const float forward = centered(fb, deadzone) / 50.0f;
  const float right = -centered(lr, deadzone) / 50.0f;
  const float magnitude = std::max(std::fabs(forward), std::fabs(right));
  if(magnitude < 0.01f) return {false, 0.0f, 0.0f};
  return {true, std::min(magnitude, 1.0f),
          wrap_degrees(heading + std::atan2(right, forward) * 180.0f / 3.14159265358979323846f)};
}

float heading_after_yaw(float heading, int yaw, int deadzone, float rate_dps, float elapsed_seconds)
{
  const float dt = std::max(0.0f, std::min(elapsed_seconds, 0.05f));
  return wrap_degrees(heading - centered(yaw, deadzone) / 50.0f * rate_dps * dt);
}
} // namespace controller
