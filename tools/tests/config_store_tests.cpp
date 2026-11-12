#include "../../src/main_refactored/Config_Store.h"
#include <cassert>
#include <cstdio>

struct FakeBlobStore
{
  config_record::Record stored = {};
  std::size_t length = 0;
  int writes = 0;
  bool reject_write = false;
  bool corrupt_read = false;
  bool short_read = false;

  std::size_t putBytes(const char *, const void *data, std::size_t size)
  {
    ++writes;
    if(reject_write) return 0;
    assert(size == sizeof(stored.bytes));
    std::memcpy(stored.bytes, data, size);
    length = size;
    return size;
  }
  std::size_t getBytesLength(const char *) { return length; }
  std::size_t getBytes(const char *, void *data, std::size_t size)
  {
    std::memcpy(data, stored.bytes, size);
    if(corrupt_read) static_cast<std::uint8_t *>(data)[20] ^= 0x10;
    return short_read ? size - 1 : size;
  }
};

int main()
{
  using namespace config_record;
  RuntimeConfig config = defaults();
  assert(valid(config));
  config.correction_point_count = 3;
  config.correction_points[0] = {2300.0f, 1.10f, 70};
  config.correction_points[1] = {700.0f, 0.90f, -40};
  config.correction_points[2] = {1400.0f, 1.03f, 0};
  config.accel_zero_x_g_offset = -2.5f;
  config.translation_scale_factor = 2.0f;

  Record record = {};
  assert(encode(config, record));
  RuntimeConfig decoded = defaults();
  assert(decode(record.bytes, sizeof(record.bytes), decoded));
  assert(decoded.correction_point_count == 3);
  assert(decoded.correction_points[0].rpm == 700.0f);
  assert(decoded.correction_points[0].offset == 0.90f);
  assert(decoded.correction_points[0].throttle == -40);
  assert(decoded.correction_points[2].rpm == 2300.0f);
  assert(decoded.correction_points[2].offset == 1.10f);
  assert(decoded.correction_points[2].throttle == 70);
  assert(decoded.accel_zero_x_g_offset == -2.5f);
  assert(decoded.translation_scale_factor == 2.0f);

  // A torn or corrupted record must never partially replace live settings.
  for(std::size_t i = 0; i < kRecordSize; ++i)
  {
    Record damaged = record;
    damaged.bytes[i] ^= 0x01;
    RuntimeConfig untouched = defaults();
    untouched.led_heading_offset_deg = 123.0f;
    assert(!decode(damaged.bytes, sizeof(damaged.bytes), untouched));
    assert(untouched.led_heading_offset_deg == 123.0f);
    assert(untouched.correction_point_count == 0);
  }
  assert(!decode(record.bytes, kRecordSize - 1, decoded));
  assert(!decode(nullptr, kRecordSize, decoded));
  Record future = record;
  write_u32(future.bytes + 4, 2 | (kPayloadSize << 16));
  write_u32(future.bytes + kRecordSize - 4, checksum(future.bytes, kRecordSize - 4));
  assert(!decode(future.bytes, kRecordSize, decoded));

  FakeBlobStore store;
  assert(save(store, config) == SaveResult::Saved);
  assert(store.writes == 1); // All settings are one write, including the table.
  Record saved = store.stored;
  store.reject_write = true;
  config.acc_rate = 0.8f;
  assert(save(store, config) == SaveResult::WriteFailed);
  assert(std::memcmp(saved.bytes, store.stored.bytes, kRecordSize) == 0);
  store.reject_write = false;
  store.corrupt_read = true;
  assert(save(store, config) == SaveResult::VerificationFailed);
  store.corrupt_read = false;
  store.short_read = true;
  assert(save(store, config) == SaveResult::VerificationFailed);
  store.short_read = false;

  RuntimeConfig invalid = config;
  invalid.rpm_offset = std::numeric_limits<float>::quiet_NaN();
  int writes_before = store.writes;
  assert(save(store, invalid) == SaveResult::InvalidConfig);
  assert(store.writes == writes_before);
  invalid = config;
  invalid.correction_point_count = 9;
  assert(!encode(invalid, record));
  invalid = config;
  invalid.correction_points[0].rpm = std::numeric_limits<float>::infinity();
  assert(!encode(invalid, record));
  invalid = config;
  invalid.led_heading_offset_deg = 360.0f;
  assert(!encode(invalid, record));

  RuntimeConfig legacy = defaults();
  legacy.acc_rate = -1.0f;
  legacy.accel_zero_y_g_offset = 21.0f;
  legacy.correction_point_count = 2;
  legacy.correction_points[0] = {1000.0f, 1.1f, 250};
  legacy.correction_points[1] = {std::numeric_limits<float>::infinity(), 0.0f, -250};
  sanitize_legacy(legacy);
  assert(valid(legacy));
  assert(legacy.acc_rate == defaults().acc_rate);
  assert(legacy.accel_zero_y_g_offset == 0.0f);
  assert(legacy.correction_points[0].rpm == 0.0f);
  assert(legacy.correction_points[0].offset == 1.0f);
  assert(legacy.correction_points[0].throttle == -100);
  assert(legacy.correction_points[1].throttle == 100);
  std::puts("PASS: configuration roundtrip, pairing, corruption, versioning, failed writes/readback, validation, legacy import");
}
