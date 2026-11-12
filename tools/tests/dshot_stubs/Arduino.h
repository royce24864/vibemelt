#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#define IRAM_ATTR
using std::max;
using std::min;
template<class T> T constrain(T value, T lo, T hi) {
  return std::min(hi, std::max(lo, value));
}
int64_t esp_timer_get_time();
void delayMicroseconds(uint32_t delay);
struct Stream {
  template<class... Args> void printf(const char *, Args...) {}
  template<class... Args> void print(Args...) {}
  template<class... Args> void println(Args...) {}
};
inline Stream Serial;
struct EspStub {
  const char *getChipModel() { return "stub"; }
  int getChipRevision() { return 0; }
  int getCpuFreqMHz() { return 0; }
};
inline EspStub ESP;
inline int getXtalFrequencyMhz() { return 0; }
inline int getApbFrequency() { return 0; }
