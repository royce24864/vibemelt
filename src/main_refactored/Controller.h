#pragma once

#include <cstdint>

enum RobotState
{
  ROBOT_ARMED_IDLE, ROBOT_TANK, ROBOT_SPINUP, ROBOT_MELTY, ROBOT_CONFIG,
  ROBOT_WIFI, ROBOT_FAILSAFE, ROBOT_LOW_BATTERY, ROBOT_SENSOR_FAULT
};

namespace controller
{
struct Controls
{
  int translate_lr;
  int translate_fb;
  int throttle;
  int yaw;
  int direction;
};

struct Health
{
  bool rc;
  bool sensor;
  bool battery_critical;
  bool motor_fault;
  bool outputs_ready;
};

struct RearmState { bool required = true; };
struct RearmResult { bool required; bool request_motor_rearm; };
struct MotorDemand { int left; int right; };
struct Translation { bool enabled; float magnitude; float target_phase_deg; };

int centered(int value, int deadzone);
bool neutral(const Controls &controls, int deadzone, int low_throttle);
bool translation_active(int lr, int fb, int deadzone);
bool spin_direction(bool previous, const Controls &controls, bool rc_healthy,
                    bool config_active, bool wifi_active);
RearmResult update_rearm(RearmState &state, const Health &health,
                         bool controls_neutral, bool calibration_wait_for_low,
                         bool special_mode_active);
RobotState select_state(const Health &health, bool wifi_active, bool config_active,
                        bool rearm_required, bool calibration_wait_for_low,
                        int rpm, const Controls &controls, int deadzone);
bool requires_zero(RobotState state);
int spin_demand(int throttle, int rpm, float acceleration_rate, bool reversed);
MotorDemand tank_demand(int lr, int fb, bool reversed);
Translation translation(int lr, int fb, float heading, int deadzone);
float wrap_degrees(float degrees);
float heading_after_yaw(float heading, int yaw, int deadzone,
                        float rate_dps, float elapsed_seconds);
} // namespace controller
