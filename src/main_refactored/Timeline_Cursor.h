#pragma once

#include <stdint.h>

struct TimelineCatchUp
{
  uint64_t skipped_samples;
  uint32_t append_samples;
};

// Plan one bounded service of a fixed-interval ring. next_sequence is the
// first interval not yet recorded; latest_due_sequence is inclusive.
// Skipped intervals are all older than the final retained window. If any
// are skipped, all previously stored rows are outside that window as well.
TimelineCatchUp timeline_catch_up(uint64_t next_sequence,
                                  uint64_t latest_due_sequence,
                                  uint32_t capacity, uint32_t service_budget);
