#pragma once

#include <cstdint>

namespace motor_timing {

// Conservative starting limits for the 2 kHz output task. Validate against
// measured controller/sensor latency on hardware before increasing these.
constexpr int64_t COMMAND_MAX_AGE_US = 50000;
constexpr int64_t SENSOR_MAX_AGE_US = 25000;
constexpr int64_t PHASE_MAX_GAP_US = 20000;
constexpr int64_t TRANSPORT_MAX_SILENCE_US = 5000;
constexpr uint32_t REARM_ZERO_FRAMES = 2;
constexpr uint32_t NOTIFICATION_TIMEOUT_MS = 20;

bool fresh(int64_t now_us, int64_t sample_us, int64_t maximum_age_us);

class Phase {
 public:
  void reset(int64_t now_us);
  // Integrates the complete elapsed interval, or invalidates instead of
  // silently discarding time. Only an explicit reset restores validity.
  bool advance(int64_t now_us, float rpm, float rotation_offset);
  bool valid() const { return valid_; }
  float angle_degrees() const { return degrees_; }
 private:
  int64_t last_us_ = 0;
  float degrees_ = 0;
  bool valid_ = false;
};

enum class TxOutcome : uint8_t { Queued, Skipped, Failed };

struct TransportHealth {
  uint32_t queued = 0;
  uint32_t skipped = 0;
  uint32_t failed = 0;
  uint32_t consecutive_zero_frames = 0;
  int64_t last_queued_us = 0;
  bool healthy = false;
};

// Owned by the motor task; callers publish a read-only copy under their RTOS
// lock. Successful sends never implicitly clear an already latched fault.
class Safety {
 public:
  void latch();
  bool fault() const { return fault_; }
  bool check_inputs(int64_t now_us, int64_t command_us, int64_t sensor_us,
                    bool output_requested, bool phase_valid);
  void record_tx(bool left, TxOutcome outcome, bool zero, int64_t now_us);
  bool rearm(bool requested, bool output_disabled, bool inputs_healthy);
  const TransportHealth &left() const { return left_; }
  const TransportHealth &right() const { return right_; }
 private:
  bool fault_ = false;
  TransportHealth left_;
  TransportHealth right_;
};

} // namespace motor_timing
