#pragma once
#include <cstdint>

namespace estimator
{
struct Config
{
  float radial_unit_x;
  float radial_unit_y;
  float rpm_per_sqrt_g;
  float filter_alpha;
  float variation_alpha;
  std::uint32_t stable_hold_ms;
};

struct State
{
  float median_samples[3] = {};
  int median_index = 0;
  int valid_sample_count = 0;
  float filtered_g = 0.0f;
  float calculated_rpm = 0.0f;
  int rpm = 0;
  float variation = 0.0f;
  std::uint32_t stable_since_ms = 0;
  std::uint32_t last_sample_ms = 0;
  bool stability_started = false;
  bool valid = false;
};

void reset(State &state);
bool accept_sample(State &state, const Config &config, float x_g, float y_g,
                   float zero_x_g, float zero_y_g, std::uint32_t now_ms,
                   float &sample_g);
bool stable(const State &state, const Config &config, std::uint32_t now_ms);
} // namespace estimator
