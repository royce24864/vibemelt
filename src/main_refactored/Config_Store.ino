// Keep this sketch's saves separate from the untouched original sketch. The
// legacy namespace is opened read-only, and only when no new record exists.
static Preferences config_v2_store;
static bool config_v2_ready = false;

static_assert(MAX_CORRECTION_POINTS == config_record::kCorrectionPoints,
              "Update the configuration record when changing table capacity");
static_assert(DEFAULT_ACC_RATE == 0.40f && MIN_ACC_RATE == 0.20f && MAX_ACC_RATE == 1.00f &&
              DEFAULT_PERCENT_DECEL == 15.0f && MIN_PERCENT_DECEL == 1.0f && MAX_PERCENT_DECEL == 50.0f &&
              DEFAULT_TRANSLATION_SCALE_FACTOR == 1.0f && MIN_TRANSLATION_SCALE_FACTOR == 0.0f &&
              MAX_TRANSLATION_SCALE_FACTOR == 2.0f,
              "Keep Config_Store.h defaults and validation in sync with drive settings");

RuntimeConfig capture_runtime_config()
{
  RuntimeConfig config = {};
  config.acc_rate = acc_rate;
  config.percentdecel = percentdecel;
  config.translation_scale_factor = translation_scale_factor;
  config.rpm_offset = offset;
  config.led_heading_offset_deg = led_heading_offset_deg;
  config.accel_zero_x_g_offset = accel_zero_x_g_offset;
  config.accel_zero_y_g_offset = accel_zero_y_g_offset;
  config.correction_point_count = correction_point_count;
  for(int i = 0; i < MAX_CORRECTION_POINTS; ++i)
  {
    config.correction_points[i].rpm = correction_rpm[i];
    config.correction_points[i].offset = correction_offset[i];
    config.correction_points[i].throttle = correction_throttle[i];
  }
  config_record::sort_points(config);
  return config;
}

void apply_runtime_config(const RuntimeConfig &config)
{
  acc_rate = config.acc_rate;
  percentdecel = config.percentdecel;
  translation_scale_factor = config.translation_scale_factor;
  offset = config.rpm_offset;
  led_heading_offset_deg = config.led_heading_offset_deg;
  accel_zero_x_g_offset = config.accel_zero_x_g_offset;
  accel_zero_y_g_offset = config.accel_zero_y_g_offset;
  correction_point_count = config.correction_point_count;
  for(int i = 0; i < MAX_CORRECTION_POINTS; ++i)
  {
    correction_rpm[i] = config.correction_points[i].rpm;
    correction_offset[i] = config.correction_points[i].offset;
    correction_throttle[i] = config.correction_points[i].throttle;
  }
}

bool initialize_config_store()
{
  if(config_v2_ready) return true;
  config_v2_ready = config_v2_store.begin("vibemelt_v2", false);
  if(!config_v2_ready) DEBUG_LOG("config: could not open configuration storage; using defaults");
  return config_v2_ready;
}

bool save_config_settings()
{
  if(!config_v2_ready)
  {
    DEBUG_LOG("config: save failed; storage is unavailable");
    return false;
  }
  const config_record::SaveResult result = config_record::save(config_v2_store, capture_runtime_config());
  switch(result)
  {
    case config_record::SaveResult::Saved:
      return true;
    case config_record::SaveResult::InvalidConfig:
      DEBUG_LOG("config: save rejected; runtime values are invalid");
      break;
    case config_record::SaveResult::WriteFailed:
      DEBUG_LOG("config: save failed; NVS blob write or commit failed");
      break;
    case config_record::SaveResult::VerificationFailed:
      DEBUG_LOG("config: save failed; NVS blob verification failed");
      break;
  }
  return false;
}

void load_config_settings()
{
  RuntimeConfig config = config_record::defaults();
  if(!config_v2_ready)
  {
    apply_runtime_config(config);
    return;
  }

  if(config_v2_store.isKey(config_record::kKey))
  {
    config_record::Record record = {};
    if(config_v2_store.getBytesLength(config_record::kKey) != config_record::kRecordSize ||
       config_v2_store.getBytes(config_record::kKey, record.bytes, sizeof(record.bytes)) != sizeof(record.bytes) ||
       !config_record::decode(record.bytes, sizeof(record.bytes), config))
    {
      // An invalid/future record must not silently resurrect old calibration.
      // Leave it intact for diagnosis; only a deliberate save replaces it.
      DEBUG_LOG("config: invalid or unsupported saved record; using defaults");
    }
    apply_runtime_config(config);
    return;
  }

  Preferences legacy;
  const bool legacy_available = legacy.begin("vibemelt", true);
  if(legacy_available)
  {
    config.acc_rate = legacy.getFloat("acc_rate", config.acc_rate);
    config.percentdecel = legacy.getFloat("pulse_pct", config.percentdecel);
    config.translation_scale_factor = legacy.getFloat("trans_scale", config.translation_scale_factor);
    config.rpm_offset = legacy.getFloat("rpm_offset", config.rpm_offset);
    config.led_heading_offset_deg = legacy.getFloat("led_heading", config.led_heading_offset_deg);
    config.accel_zero_x_g_offset = legacy.getFloat("accel_zero_x", 0.0f);
    config.accel_zero_y_g_offset = legacy.getFloat("accel_zero_y", legacy.getFloat("accel_zero", 0.0f));
    config.correction_point_count = legacy.getInt("corr_count", 0);
    for(int i = 0; i < MAX_CORRECTION_POINTS; ++i)
    {
      char rpm_key[8];
      char offset_key[8];
      char throttle_key[8];
      snprintf(rpm_key, sizeof(rpm_key), "rpm%d", i);
      snprintf(offset_key, sizeof(offset_key), "off%d", i);
      snprintf(throttle_key, sizeof(throttle_key), "thr%d", i);
      config.correction_points[i].rpm = legacy.getFloat(rpm_key, 0.0f);
      config.correction_points[i].offset = legacy.getFloat(offset_key, 1.0f);
      config.correction_points[i].throttle = legacy.getInt(throttle_key, 0);
    }
    legacy.end();
    config_record::sanitize_legacy(config);
  }
  apply_runtime_config(config);
  if(save_config_settings())
    DEBUG_LOG(legacy_available ? "config: legacy settings imported and verified" :
                                "config: default settings initialized and verified");
  else
    DEBUG_LOG("config: initial settings loaded in RAM only; persistence failed");
}
