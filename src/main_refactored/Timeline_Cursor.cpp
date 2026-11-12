#include "Timeline_Cursor.h"

TimelineCatchUp timeline_catch_up(uint64_t next_sequence,
                                  uint64_t latest_due_sequence,
                                  uint32_t capacity, uint32_t service_budget)
{
  TimelineCatchUp result = {};
  if(capacity == 0 || service_budget == 0 || next_sequence > latest_due_sequence)
    return result;

  // Subtract before adding the inclusive endpoint, avoiding overflow even
  // for a caller whose latest sequence is UINT64_MAX.
  uint64_t preceding_intervals = latest_due_sequence - next_sequence;
  uint32_t retained_intervals;
  if(preceding_intervals >= capacity)
  {
    result.skipped_samples = preceding_intervals - (capacity - 1);
    retained_intervals = capacity;
  }
  else
  {
    retained_intervals = (uint32_t)preceding_intervals + 1;
  }
  result.append_samples = retained_intervals < service_budget
                            ? retained_intervals : service_budget;
  return result;
}
