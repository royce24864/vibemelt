#include "Estimator.h"
#include <algorithm>
#include <cmath>

namespace estimator
{
void reset(State &state) { state = State{}; }

bool accept_sample(State &state, const Config &config, float x_g, float y_g,
                   float zero_x_g, float zero_y_g, std::uint32_t now_ms,
                   float &sample_g)
{
  sample_g = std::fabs((x_g - zero_x_g) * config.radial_unit_x +
                       (y_g - zero_y_g) * config.radial_unit_y);
  if(!std::isfinite(sample_g)) return false;
  state.median_samples[state.median_index] = sample_g;
  state.median_index = (state.median_index + 1) % 3;
  state.valid_sample_count = std::min(state.valid_sample_count + 1, 3);
  float input = sample_g;
  if(state.valid_sample_count == 3)
  {
    const float a = state.median_samples[0], b = state.median_samples[1], c = state.median_samples[2];
    input = std::max(std::min(a, b), std::min(std::max(a, b), c));
  }
  state.filtered_g = state.valid_sample_count == 1 ? input :
      state.filtered_g * (1.0f - config.filter_alpha) + input * config.filter_alpha;
  const float next_rpm = std::sqrt(state.filtered_g) * config.rpm_per_sqrt_g;
  const float delta = std::fabs(next_rpm - state.calculated_rpm);
  state.variation = state.variation * (1.0f - config.variation_alpha) + delta * config.variation_alpha;
  const float stable_limit = std::max(12.0f, next_rpm * 0.01f);
  if(next_rpm > 400.0f && state.variation <= stable_limit)
  {
    if(!state.stability_started) state.stable_since_ms = now_ms;
    state.stability_started = true;
  }
  else
  {
    state.stability_started = false;
    state.stable_since_ms = 0;
  }
  state.calculated_rpm = next_rpm;
  state.rpm = static_cast<int>(std::round(next_rpm));
  state.last_sample_ms = now_ms;
  state.valid = true;
  return true;
}

bool stable(const State &state, const Config &config, std::uint32_t now_ms)
{
  return state.valid && state.stability_started &&
         static_cast<std::uint32_t>(now_ms - state.stable_since_ms) >= config.stable_hold_ms &&
         static_cast<std::uint32_t>(now_ms - state.last_sample_ms) < 50;
}
} // namespace estimator
