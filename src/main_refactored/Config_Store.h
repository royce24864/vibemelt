#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

// This model and record codec have no Arduino dependency. Calibration can keep
// a complete snapshot, and native tests exercise the same validation/save path.
struct CorrectionPoint
{
  float rpm;
  float offset;
  std::int32_t throttle;
};

struct RuntimeConfig
{
  float acc_rate;
  float percentdecel;
  float translation_scale_factor;
  float rpm_offset;
  float led_heading_offset_deg;
  float accel_zero_x_g_offset;
  float accel_zero_y_g_offset;
  std::int32_t correction_point_count;
  CorrectionPoint correction_points[8];
};

namespace config_record
{
constexpr std::size_t kCorrectionPoints = 8;
constexpr std::uint32_t kMagic = 0x324d4256u; // "VBM2", little endian
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kPayloadSize = 128;
constexpr std::size_t kRecordSize = 8 + kPayloadSize + 4;
constexpr const char *kKey = "record";
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Configuration records require 32-bit IEEE 754 floats");

struct Record
{
  std::uint8_t bytes[kRecordSize];
};

inline RuntimeConfig defaults()
{
  RuntimeConfig config = {};
  config.acc_rate = 0.40f;
  config.percentdecel = 15.0f;
  config.translation_scale_factor = 1.0f;
  config.rpm_offset = 1.0f;
  config.led_heading_offset_deg = 315.0f;
  for(std::size_t i = 0; i < kCorrectionPoints; ++i)
    config.correction_points[i].offset = 1.0f;
  return config;
}

inline bool in_range(float value, float minimum, float maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

inline bool valid(const RuntimeConfig &config)
{
  if(!in_range(config.acc_rate, 0.20f, 1.00f) ||
     !in_range(config.percentdecel, 1.0f, 50.0f) ||
     !in_range(config.translation_scale_factor, 0.0f, 2.0f) ||
     !in_range(config.rpm_offset, 0.8f, 1.2f) ||
     !in_range(config.led_heading_offset_deg, 0.0f, 360.0f) ||
     config.led_heading_offset_deg == 360.0f ||
     !in_range(config.accel_zero_x_g_offset, -20.0f, 20.0f) ||
     !in_range(config.accel_zero_y_g_offset, -20.0f, 20.0f) ||
     config.correction_point_count < 0 ||
     config.correction_point_count > static_cast<std::int32_t>(kCorrectionPoints))
    return false;

  for(std::size_t i = 0; i < kCorrectionPoints; ++i)
  {
    const CorrectionPoint &point = config.correction_points[i];
    if(!std::isfinite(point.rpm) || point.rpm < 0.0f ||
       !in_range(point.offset, 0.8f, 1.2f) ||
       point.throttle < -100 || point.throttle > 100)
      return false;
  }
  return true;
}

inline void sort_points(RuntimeConfig &config)
{
  if(config.correction_point_count < 0 ||
     config.correction_point_count > static_cast<std::int32_t>(kCorrectionPoints))
    return;
  for(std::int32_t i = 1; i < config.correction_point_count; ++i)
  {
    const CorrectionPoint point = config.correction_points[i];
    std::int32_t j = i;
    while(j > 0 && config.correction_points[j - 1].rpm > point.rpm)
    {
      config.correction_points[j] = config.correction_points[j - 1];
      --j;
    }
    config.correction_points[j] = point;
  }
}

// Legacy values were independent keys. Preserve its fallback/clamping behavior
// during one-time import, while also rejecting infinities in RPM values.
inline void sanitize_legacy(RuntimeConfig &config)
{
  const RuntimeConfig fallback = defaults();
  if(!in_range(config.acc_rate, 0.20f, 1.00f)) config.acc_rate = fallback.acc_rate;
  if(!in_range(config.percentdecel, 1.0f, 50.0f)) config.percentdecel = fallback.percentdecel;
  if(!in_range(config.translation_scale_factor, 0.0f, 2.0f))
    config.translation_scale_factor = fallback.translation_scale_factor;
  if(!in_range(config.rpm_offset, 0.8f, 1.2f)) config.rpm_offset = fallback.rpm_offset;
  if(!in_range(config.led_heading_offset_deg, 0.0f, 360.0f) ||
     config.led_heading_offset_deg == 360.0f)
    config.led_heading_offset_deg = fallback.led_heading_offset_deg;
  if(!in_range(config.accel_zero_x_g_offset, -20.0f, 20.0f)) config.accel_zero_x_g_offset = 0.0f;
  if(!in_range(config.accel_zero_y_g_offset, -20.0f, 20.0f)) config.accel_zero_y_g_offset = 0.0f;
  if(config.correction_point_count < 0 ||
     config.correction_point_count > static_cast<std::int32_t>(kCorrectionPoints))
  {
    config.correction_point_count = 0;
    for(std::size_t i = 0; i < kCorrectionPoints; ++i)
      config.correction_points[i] = fallback.correction_points[i];
  }
  for(std::size_t i = 0; i < kCorrectionPoints; ++i)
  {
    CorrectionPoint &point = config.correction_points[i];
    if(!std::isfinite(point.rpm) || point.rpm < 0.0f) point.rpm = 0.0f;
    if(!in_range(point.offset, 0.8f, 1.2f)) point.offset = 1.0f;
    if(point.throttle < -100) point.throttle = -100;
    if(point.throttle > 100) point.throttle = 100;
  }
  sort_points(config);
}

inline void write_u32(std::uint8_t *bytes, std::uint32_t value)
{
  for(unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (i * 8));
}

inline std::uint32_t read_u32(const std::uint8_t *bytes)
{
  std::uint32_t value = 0;
  for(unsigned i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(bytes[i]) << (i * 8);
  return value;
}

inline void write_float(std::uint8_t *bytes, float value)
{
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32(bytes, bits);
}

inline float read_float(const std::uint8_t *bytes)
{
  const std::uint32_t bits = read_u32(bytes);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline std::uint32_t checksum(const std::uint8_t *bytes, std::size_t length)
{
  std::uint32_t crc = 0xffffffffu;
  for(std::size_t i = 0; i < length; ++i)
  {
    crc ^= bytes[i];
    for(unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return ~crc;
}

inline bool encode(const RuntimeConfig &config, Record &record)
{
  if(!valid(config)) return false;
  write_u32(record.bytes, kMagic);
  write_u32(record.bytes + 4, kVersion | (static_cast<std::uint32_t>(kPayloadSize) << 16));
  const float fields[] = {config.acc_rate, config.percentdecel,
    config.translation_scale_factor, config.rpm_offset, config.led_heading_offset_deg,
    config.accel_zero_x_g_offset, config.accel_zero_y_g_offset};
  for(std::size_t i = 0; i < 7; ++i) write_float(record.bytes + 8 + i * 4, fields[i]);
  write_u32(record.bytes + 36, static_cast<std::uint32_t>(config.correction_point_count));
  for(std::size_t i = 0; i < kCorrectionPoints; ++i)
  {
    const CorrectionPoint &point = config.correction_points[i];
    write_float(record.bytes + 40 + i * 12, point.rpm);
    write_float(record.bytes + 44 + i * 12, point.offset);
    // Bias the signed throttle so decoding does not depend on signed casts.
    write_u32(record.bytes + 48 + i * 12, static_cast<std::uint32_t>(point.throttle + 100));
  }
  write_u32(record.bytes + kRecordSize - 4, checksum(record.bytes, kRecordSize - 4));
  return true;
}

inline bool decode(const void *data, std::size_t length, RuntimeConfig &config)
{
  if(data == nullptr || length != kRecordSize) return false;
  const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);
  if(read_u32(bytes) != kMagic ||
     read_u32(bytes + 4) != (kVersion | (static_cast<std::uint32_t>(kPayloadSize) << 16)) ||
     read_u32(bytes + kRecordSize - 4) != checksum(bytes, kRecordSize - 4))
    return false;
  RuntimeConfig decoded = defaults();
  float *fields[] = {&decoded.acc_rate, &decoded.percentdecel,
    &decoded.translation_scale_factor, &decoded.rpm_offset, &decoded.led_heading_offset_deg,
    &decoded.accel_zero_x_g_offset, &decoded.accel_zero_y_g_offset};
  for(std::size_t i = 0; i < 7; ++i) *fields[i] = read_float(bytes + 8 + i * 4);
  const std::uint32_t count = read_u32(bytes + 36);
  if(count > kCorrectionPoints) return false;
  decoded.correction_point_count = static_cast<std::int32_t>(count);
  for(std::size_t i = 0; i < kCorrectionPoints; ++i)
  {
    CorrectionPoint &point = decoded.correction_points[i];
    point.rpm = read_float(bytes + 40 + i * 12);
    point.offset = read_float(bytes + 44 + i * 12);
    const std::uint32_t throttle = read_u32(bytes + 48 + i * 12);
    if(throttle > 200u) return false;
    point.throttle = static_cast<std::int32_t>(throttle) - 100;
  }
  if(!valid(decoded)) return false;
  sort_points(decoded);
  config = decoded; // Do not expose a partial result on any failure.
  return true;
}

enum class SaveResult { Saved, InvalidConfig, WriteFailed, VerificationFailed };

// Preferences::putBytes performs nvs_set_blob plus nvs_commit. Keeping every
// field in one NVS blob prevents mixed generations of calibration point keys.
template<typename BlobStore>
inline SaveResult save(BlobStore &store, const RuntimeConfig &config)
{
  Record expected = {};
  if(!encode(config, expected)) return SaveResult::InvalidConfig;
  if(store.putBytes(kKey, expected.bytes, kRecordSize) != kRecordSize)
    return SaveResult::WriteFailed;
  Record actual = {};
  RuntimeConfig verified;
  if(store.getBytesLength(kKey) != kRecordSize ||
     store.getBytes(kKey, actual.bytes, kRecordSize) != kRecordSize ||
     std::memcmp(expected.bytes, actual.bytes, kRecordSize) != 0 ||
     !decode(actual.bytes, kRecordSize, verified))
    return SaveResult::VerificationFailed;
  return SaveResult::Saved;
}
} // namespace config_record

RuntimeConfig capture_runtime_config();
void apply_runtime_config(const RuntimeConfig &config);
bool initialize_config_store();
void load_config_settings();
bool save_config_settings();
