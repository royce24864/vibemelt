#pragma once

#include <Arduino.h>

constexpr uint32_t SESSION_SAMPLE_INTERVAL_US = 100000;
constexpr double SESSION_SAMPLE_INTERVAL_SECONDS =
    SESSION_SAMPLE_INTERVAL_US / 1000000.0;
constexpr uint32_t SESSION_SAMPLE_CAPACITY = 3000;
constexpr size_t SESSION_EVENT_CAPACITY = 16384;

enum SessionState : uint8_t
{
  SESSION_BOOT,
  SESSION_DISARMED,
  SESSION_TANK,
  SESSION_SPINUP,
  SESSION_MELTY,
  SESSION_CONFIG,
  SESSION_WIFI,
  SESSION_FAILSAFE,
  SESSION_LOW_BATTERY,
  SESSION_SENSOR_FAULT
};

enum SessionSampleFlags : uint8_t
{
  SAMPLE_ACCEL_VALID = 1 << 0,
  SAMPLE_BATTERY_VALID = 1 << 1,
  SAMPLE_RC_VALID = 1 << 2,
  SAMPLE_MISSED_OR_INVALID = 1 << 3,
  SAMPLE_ACCEL_LIMITER_ACTIVE = 1 << 4,
  SAMPLE_MOTOR_SAFETY_GATE_CLOSED = 1 << 5,
  SAMPLE_TRANSLATION_PULSE_SEEN = 1 << 6,
  SAMPLE_SENSOR_FAULT_ACTIVE = 1 << 7
};

struct TelemetrySample
{
  uint16_t rpm;
  uint16_t g_centi_g;
  uint16_t battery_centivolts;
  uint16_t rpm_variation;
  int8_t throttle_normalized;
  int8_t ch1_normalized;
  int8_t ch2_normalized;
  int8_t base_spin_command;
  int8_t left_dshot_average;
  int8_t right_dshot_average;
  uint8_t state;
  uint8_t flags;
};

static_assert(sizeof(TelemetrySample) == 16,
              "TelemetrySample must remain exactly 16 bytes");

struct SessionSummary
{
  uint16_t maximum_rpm;
  uint16_t maximum_g_centi_g;
  uint16_t minimum_battery_centivolts;
  uint64_t maximum_rpm_sequence;
  uint64_t maximum_g_sequence;
  uint64_t tank_ms;
  uint64_t spinup_ms;
  uint64_t melty_ms;
  uint32_t failsafe_episodes;
  uint32_t sensor_fault_episodes;
  uint32_t invalid_rc_frames;
  uint32_t motor_scheduler_overruns;
  uint32_t motor_scheduler_max_interval_us;
  uint32_t dshot_tx_api_errors;
  uint32_t minimum_free_heap;
};

struct WindowSummary
{
  uint16_t maximum_rpm;
  uint16_t maximum_g_centi_g;
  uint16_t minimum_battery_centivolts;
  uint64_t maximum_rpm_sequence;
  uint64_t maximum_g_sequence;
  uint32_t tank_samples;
  uint32_t spinup_samples;
  uint32_t melty_samples;
};
