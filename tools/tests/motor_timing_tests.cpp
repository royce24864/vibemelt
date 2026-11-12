#include "Motor_Timing.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace motor_timing;

static void zero_pair(Safety &safety, int64_t time) {
  safety.record_tx(true, TxOutcome::Queued, true, time);
  safety.record_tx(false, TxOutcome::Queued, true, time);
}

int main() {
  // Exact freshness limits, missing stamps, future stamps and long uptime.
  assert(fresh(50001, 1, COMMAND_MAX_AGE_US));
  assert(!fresh(50002, 1, COMMAND_MAX_AGE_US));
  assert(fresh(25001, 1, SENSOR_MAX_AGE_US));
  assert(!fresh(25002, 1, SENSOR_MAX_AGE_US));
  assert(!fresh(100, 0, SENSOR_MAX_AGE_US));
  assert(!fresh(100, 101, SENSOR_MAX_AGE_US));
  assert(fresh(INT64_C(9000000000), INT64_C(8999999000), SENSOR_MAX_AGE_US));

  Phase phase;
  assert(!phase.valid());
  phase.reset(100000);
  assert(phase.advance(105000, 1000, 1));
  assert(std::fabs(phase.angle_degrees() - 30) < 0.001f);
  assert(phase.advance(125000, 1000, 1));
  assert(std::fabs(phase.angle_degrees() - 150) < 0.001f);
  assert(!phase.advance(145001, 1000, 1));
  assert(!phase.advance(145501, 1000, 1)); // Normal ticks cannot restore phase.
  assert(std::fabs(phase.angle_degrees() - 150) < 0.001f);
  phase.reset(150000);
  assert(phase.advance(150500, 1000, 2));
  assert(std::fabs(phase.angle_degrees() - 1.5f) < 0.001f);
  assert(!phase.advance(150499, 1000, 1)); // A reversed clock is invalid too.
  phase.reset(160000);
  assert(!phase.advance(160500, std::numeric_limits<float>::quiet_NaN(), 1));

  Safety safety;
  assert(safety.check_inputs(100000, 99000, 99000, true, true));
  assert(!safety.fault());
  assert(!safety.check_inputs(100000, 99000, 74000, true, true));
  assert(safety.fault()); // Old actual sensor stamp cannot be refreshed by demand.
  assert(safety.check_inputs(100500, 100000, 100000, true, true));
  assert(safety.fault()); // Fresh data does not resume held throttle.
  zero_pair(safety, 101000);
  assert(!safety.rearm(true, true, true));
  zero_pair(safety, 101500);
  assert(!safety.rearm(false, true, true));
  assert(!safety.rearm(true, false, true));
  assert(!safety.rearm(true, true, false));
  assert(safety.rearm(true, true, true));
  assert(!safety.fault());

  // A new fault discards zero evidence recorded before the fault.
  safety.latch();
  assert(safety.left().consecutive_zero_frames == 0);
  assert(!safety.rearm(true, true, true));
  zero_pair(safety, 102000);
  safety.record_tx(false, TxOutcome::Failed, true, 102500);
  assert(safety.fault() && !safety.right().healthy);
  assert(safety.right().failed == 1 && safety.left().failed == 0);
  assert(!safety.rearm(true, true, true));
  zero_pair(safety, 103000);
  assert(!safety.rearm(true, true, true)); // The failed side needs two new zeros.
  zero_pair(safety, 103500);
  assert(safety.rearm(true, true, true));

  // Benign timing skips neither count as sent frames nor poison the transport.
  const uint32_t accepted = safety.left().queued;
  safety.record_tx(true, TxOutcome::Skipped, false, 104000);
  assert(safety.left().queued == accepted);
  assert(safety.left().skipped == 1 && !safety.fault());
  safety.record_tx(true, TxOutcome::Skipped, false, 108501);
  assert(safety.fault() && !safety.left().healthy);
  assert(safety.left().queued == accepted);
  assert(!safety.rearm(true, true, true));
  zero_pair(safety, 109000);
  zero_pair(safety, 109500);
  assert(safety.rearm(true, true, true));
  safety.record_tx(true, TxOutcome::Queued, false, 110000);
  assert(safety.left().consecutive_zero_frames == 0);

  Safety phase_fault;
  assert(phase_fault.check_inputs(1000, 500, 500, true, false));
  assert(phase_fault.fault());
  Safety disabled;
  assert(!disabled.check_inputs(100000, 1, 1, false, false));
  assert(!disabled.fault()); // Stale disabled demand cannot enable output.
  std::puts("Motor timing tests passed (freshness, phase gaps, transport, neutral rearm)");
}
