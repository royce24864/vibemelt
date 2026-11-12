#include "Motor_Timing.h"

#include <cmath>

namespace motor_timing {

bool fresh(int64_t now_us, int64_t sample_us, int64_t maximum_age_us) {
  return sample_us > 0 && now_us >= sample_us &&
         now_us - sample_us <= maximum_age_us;
}

void Phase::reset(int64_t now_us) {
  last_us_ = now_us;
  degrees_ = 0;
  valid_ = true;
}

bool Phase::advance(int64_t now_us, float rpm, float rotation_offset) {
  const int64_t elapsed_us = now_us - last_us_;
  last_us_ = now_us;
  if (elapsed_us < 0 || elapsed_us > PHASE_MAX_GAP_US ||
      !std::isfinite(rpm) || !std::isfinite(rotation_offset)) {
    valid_ = false;
  }
  if (!valid_) return false;
  if (rpm >= 200.0f && rotation_offset > 0.01f) {
    degrees_ = std::fmod(degrees_ + static_cast<float>(elapsed_us) * 0.000006f *
                        rpm / rotation_offset, 360.0f);
  } else {
    degrees_ = 0;
  }
  return true;
}

bool Safety::check_inputs(int64_t now_us, int64_t command_us, int64_t sensor_us,
                          bool output_requested, bool phase_valid) {
  const bool valid = fresh(now_us, command_us, COMMAND_MAX_AGE_US) &&
                     fresh(now_us, sensor_us, SENSOR_MAX_AGE_US);
  if (output_requested && (!valid || !phase_valid)) latch();
  return valid;
}

void Safety::latch() {
  if (!fault_) {
    left_.consecutive_zero_frames = 0;
    right_.consecutive_zero_frames = 0;
  }
  fault_ = true;
}

void Safety::record_tx(bool left, TxOutcome outcome, bool zero, int64_t now_us) {
  TransportHealth &health = left ? left_ : right_;
  if (outcome == TxOutcome::Queued) {
    ++health.queued;
    health.last_queued_us = now_us;
    health.healthy = true;
    health.consecutive_zero_frames = zero
        ? (health.consecutive_zero_frames < REARM_ZERO_FRAMES
               ? health.consecutive_zero_frames + 1 : REARM_ZERO_FRAMES)
        : 0;
  } else if (outcome == TxOutcome::Skipped) {
    ++health.skipped;
    if (!fresh(now_us, health.last_queued_us, TRANSPORT_MAX_SILENCE_US)) {
      health.healthy = false;
      health.consecutive_zero_frames = 0;
      latch();
    }
  } else {
    ++health.failed;
    health.healthy = false;
    health.consecutive_zero_frames = 0;
    latch();
  }
}

bool Safety::rearm(bool requested, bool output_disabled, bool inputs_healthy) {
  if (!requested || !output_disabled || !inputs_healthy ||
      !left_.healthy || !right_.healthy ||
      left_.consecutive_zero_frames < REARM_ZERO_FRAMES ||
      right_.consecutive_zero_frames < REARM_ZERO_FRAMES) return false;
  fault_ = false;
  return true;
}

} // namespace motor_timing
