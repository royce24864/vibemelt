void record_accel_loss(const char *reason)
{
  unsigned long now = millis();
  bool new_incident = accel_loss_started_ms == 0;
  if(new_incident)
  {
    accel_loss_started_ms = now == 0 ? 1 : now;
  }
  accel_sensor_healthy = false;
  session_note_sensor_health(false);
  if(new_incident)
  {
    unsigned long sample_age_ms = last_accel_sample_ms == 0
                                    ? 0 : now - last_accel_sample_ms;
    DEBUG_LOGF("accelerometer LOSS: reason=%s rpm=%d sample_age_ms=%lu",
               reason != nullptr ? reason : "unknown", rpm, sample_age_ms);
  }
}

void record_accel_recovery(const char *reason)
{
  unsigned long now = millis();
  bool recovering = accel_sensor_healthy == false || accel_loss_started_ms != 0;
  unsigned long downtime_ms = accel_loss_started_ms == 0
                                ? 0 : now - accel_loss_started_ms;
  accel_sensor_healthy = true;
  session_note_sensor_health(true);
  if(recovering)
  {
    DEBUG_LOGF("accelerometer RECOVERY: reason=%s downtime_ms=%lu phase_reset=1",
               reason != nullptr ? reason : "reinitialized", downtime_ms);
  }
  accel_loss_started_ms = 0;
}

bool read_accel_register_checked(uint8_t register_address, uint8_t &value)
{
  Wire.beginTransmission(ACCEL_I2C_ADDRESS);
  if(Wire.write(register_address) != 1 || Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if(Wire.requestFrom((uint8_t)ACCEL_I2C_ADDRESS, (uint8_t)1) != 1 ||
     Wire.available() < 1)
  {
    return false;
  }

  int data = Wire.read();
  if(data < 0)
  {
    return false;
  }
  value = (uint8_t)data;
  return true;
}

bool read_accel_axes_checked(int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t data[6];
  Wire.beginTransmission(ACCEL_I2C_ADDRESS);
  if(Wire.write(OUT_X_L | 0x80) != 1 || Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if(Wire.requestFrom((uint8_t)ACCEL_I2C_ADDRESS, (uint8_t)sizeof(data)) !=
     sizeof(data))
  {
    return false;
  }
  for(size_t i = 0; i < sizeof(data); i++)
  {
    if(Wire.available() < 1)
    {
      return false;
    }
    int byte_value = Wire.read();
    if(byte_value < 0)
    {
      return false;
    }
    data[i] = (uint8_t)byte_value;
  }

  x = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8)) >> 4;
  y = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)) >> 4;
  z = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8)) >> 4;
  return true;
}

bool accel_configuration_valid()
{
  uint8_t control_1 = 0;
  uint8_t control_4 = 0;
  return read_accel_register_checked(CTRL_REG1, control_1) &&
         read_accel_register_checked(CTRL_REG4, control_4) &&
         (control_1 & 0x3f) == 0x3f && // normal, 1 kHz, X/Y/Z enabled
         (control_4 & 0x30) == 0x30;   // +/-400 g range
}

bool read_accel_configuration(uint8_t &control_1, uint8_t &control_4)
{
  return read_accel_register_checked(CTRL_REG1, control_1) &&
         read_accel_register_checked(CTRL_REG4, control_4);
}

bool accel_configuration_values_valid(uint8_t control_1, uint8_t control_4)
{
  return (control_1 & 0x3f) == 0x3f &&
         (control_4 & 0x30) == 0x30;
}

bool configure_accel_sensor()
{
  Wire.beginTransmission(ACCEL_I2C_ADDRESS);
  if(Wire.endTransmission() != 0)
  {
    return false;
  }

  xl.setI2CAddr(ACCEL_I2C_ADDRESS);
  xl.begin(LIS331::USE_I2C);
  xl.setFullScale(ACCEL_RANGE);
  // The library otherwise leaves the sensor at its reset-default 50 Hz ODR.
  xl.setODR(LIS331::DR_1000HZ);
  delay(2);
  return accel_configuration_valid();
}

bool initialize_accel_sensor(int attempts)
{
  attempts = max(1, attempts);
  for(int attempt = 1; attempt <= attempts; attempt++)
  {
    if(configure_accel_sensor())
    {
      accel_health_failure_count = 0;
      accel_read_failure_count = 0;
      reset_accel_filter_requested = true;
      reset_rotation_phase_requested = true;
      last_accel_sample_ms = 0;
      DEBUG_LOGF("accelerometer: initialized on attempt %d/%d", attempt,
                 attempts);
      return true;
    }

    DEBUG_LOGF("accelerometer: initialization attempt %d/%d failed", attempt,
               attempts);
    if(attempt < attempts)
    {
      delay(20);
      service_session_logger();
    }
  }
  return false;
}

void calcrpm()  // calculates the rpm based on g-force
{
  static uint32_t last_processed_sequence = 0;
  static float previous_rpm = 0.0f;

  update_accel_sensor_health();
  float gforce = get_accel_force_g();
  if(accel_sensor_healthy == false || isnan(gforce) || isinf(gforce) || gforce < 0.0f)
  {
    current_gforce = 0.0f;
    current_gforce_valid = false;
    rpm = 0;
    rpm_stable_since_ms = 0;
    return;
  }

  // Keep the exact filtered radial-g estimator value aligned with this RPM.
  current_gforce = gforce;
  current_gforce_valid = true;
  float calculated_rpm = sqrt(gforce) * rpm_per_sqrt_g;
  rpm = round(calculated_rpm);

  if(accel_sample_sequence != last_processed_sequence)
  {
    float rpm_delta = fabs(calculated_rpm - previous_rpm);
    rpm_variation_ewma = (rpm_variation_ewma * (1.0f - RPM_VARIATION_ALPHA)) +
                         (rpm_delta * RPM_VARIATION_ALPHA);
    float stable_limit = max(12.0f, calculated_rpm * 0.01f);
    if(calculated_rpm > 400.0f && rpm_variation_ewma <= stable_limit)
    {
      if(rpm_stable_since_ms == 0) rpm_stable_since_ms = millis();
    }
    else
    {
      rpm_stable_since_ms = 0;
    }

    previous_rpm = calculated_rpm;
    last_processed_sequence = accel_sample_sequence;
  }

  // Update max_rpm if this is the highest so far
  if(rpm > max_rpm) 
  {
    max_rpm = rpm;
  }
}

bool rpm_is_stable()
{
  return rpm_stable_since_ms != 0 &&
         (millis() - rpm_stable_since_ms) >= RPM_STABLE_HOLD_MS &&
         (millis() - last_accel_sample_ms) < 50;
}

void update_accel_sensor_health()
{
  unsigned long now = millis();
  if((now - last_accel_health_check_ms) < ACCEL_HEALTH_CHECK_MS)
  {
    return;
  }
  last_accel_health_check_ms = now;

  if(accel_sensor_healthy == false)
  {
    if(initialize_accel_sensor(1))
    {
      record_accel_recovery("periodic I2C reinitialization succeeded");
    }
    return;
  }

  uint8_t control_1 = 0;
  uint8_t control_4 = 0;
  bool configuration_read = read_accel_configuration(control_1, control_4);
  if(configuration_read &&
     accel_configuration_values_valid(control_1, control_4))
  {
    accel_health_failure_count = 0;
  }
  else if(configuration_read)
  {
    DEBUG_LOGF("accelerometer: configuration lost ctrl1=0x%02X ctrl4=0x%02X; reinitializing",
               control_1, control_4);
    record_accel_loss("control-register configuration lost");
    if(initialize_accel_sensor(1))
    {
      record_accel_recovery("control registers restored");
    }
  }
  else
  {
    accel_health_failure_count++;
    if(accel_health_failure_count >= 3)
    {
      reset_accel_filter_requested = true;
      DEBUG_LOGF("accelerometer: sensor fault after %d failed configuration checks",
                 accel_health_failure_count);
      record_accel_loss("three consecutive configuration reads failed");
    }
  }
}

float get_accel_force_g() 
{
  static float median_samples[3] = {0.0f, 0.0f, 0.0f};
  static int median_index = 0;
  static int valid_sample_count = 0;
  static float filtered_g = 0.0f;

  if(reset_accel_filter_requested)
  {
    median_samples[0] = median_samples[1] = median_samples[2] = 0.0f;
    median_index = 0;
    valid_sample_count = 0;
    filtered_g = 0.0f;
    reset_accel_filter_requested = false;
  }

  int64_t now_us = esp_timer_get_time();
  if ((now_us - loopstart) >= ACCEL_SAMPLE_INTERVAL_US && accel_sensor_healthy)
  {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    if(read_accel_axes_checked(x, y, z) == false)
    {
      accel_read_failure_count++;
      loopstart = now_us;
      if(accel_read_failure_count >= ACCEL_READ_FAILURE_LIMIT)
      {
        reset_accel_filter_requested = true;
        DEBUG_LOGF("accelerometer: sensor fault after %d incomplete axis reads",
                   accel_read_failure_count);
        record_accel_loss("five consecutive axis reads were incomplete");
      }
      return filtered_g;
    }
    accel_read_failure_count = 0;
    float xg = xl.convertToG(ACCEL_MAX_SCALE, x) - accel_zero_x_g_offset;
    float yg = xl.convertToG(ACCEL_MAX_SCALE, y) - accel_zero_y_g_offset;
    // Project the planar acceleration onto the known radial mounting vector.
    // This rejects the orthogonal tangential acceleration present during
    // spin-up, unlike sqrt(x^2 + y^2), which biases the RPM estimate high.
    float radial_g = (xg * ACCEL_RADIAL_UNIT_X) +
                     (yg * ACCEL_RADIAL_UNIT_Y);
    float sample_g = fabs(radial_g);
    loopstart = now_us;
    last_accel_sample_ms = millis();

    if(isnan(sample_g) || isinf(sample_g))
    {
      if(accel_sensor_healthy)
      {
        DEBUG_LOGF("accelerometer: non-finite sample fault raw=(%d,%d,%d) g=%.2f",
                   x, y, z, sample_g);
      }
      record_accel_loss("non-finite acceleration sample");
      return filtered_g;
    }

    median_samples[median_index] = sample_g;
    median_index = (median_index + 1) % 3;
    if(valid_sample_count < 3) valid_sample_count++;

    float filtered_input = sample_g;
    if(valid_sample_count == 3)
    {
      float a = median_samples[0];
      float b = median_samples[1];
      float c = median_samples[2];
      filtered_input = max(min(a, b), min(max(a, b), c));
    }
    filtered_g = valid_sample_count == 1
                   ? filtered_input
                   : (filtered_g * (1.0f - ACCEL_FILTER_ALPHA)) +
                     (filtered_input * ACCEL_FILTER_ALPHA);
    accel_sample_sequence++;

    if (round(sample_g) > max_gforce) {
      max_gforce = round(sample_g);
    }
  }

  return filtered_g;
}

bool isAngleInRange(float center, float range)
{
  center = fmod((center + 360), 360);

  // Compute smallest difference, -180..180
  float diff = fabs(fmod((angle - center + 540), 360) - 180);

  return diff <= range; // within ±range degrees
}
float wrap360(float deg)
{
  while(deg >= 360.0) deg -= 360.0;
  while(deg < 0.0) deg += 360.0;
  return deg;
}

float current_led_heading_phase_deg()
{
  return wrap360(drive_heading_phase_deg + spin_calibration[reversed].led_heading_offset_deg);
}

int stick_percent_centered(int channel)
{
  int centered = duty[channel] - 50;
  if(abs(centered) < stick_deadzone_percent)
  {
    return 0;
  }
  return centered;
}

void update_heading_reference()
{
  unsigned long now = millis();
  if(last_heading_update_ms == 0)
  {
    last_heading_update_ms = now;
    return;
  }

  float dt = constrain((now - last_heading_update_ms) * 0.001f, 0.0f, 0.05f);
  last_heading_update_ms = now;

  int yaw = stick_percent_centered(CH_HEADING_YAW);
  float yaw_fraction = yaw / 50.0;
  // Invert CH4 so the drive heading follows the physical stick direction.
  drive_heading_phase_deg = wrap360(drive_heading_phase_deg -
                                    yaw_fraction * heading_yaw_rate_dps * dt);
}
bool validStoredFloat(float val, float min_value, float max_value)
{
  return !isnan(val) && val >= min_value && val <= max_value;
}

void load_spin_calibration(SpinCalibration &calibration, const char *prefix,
                           const SpinCalibration &defaults)
{
  char key[16]; // ESP32 NVS keys allow at most 15 characters.
  snprintf(key, sizeof(key), "%srpm_offset", prefix);
  calibration.offset = config_store.getFloat(key, defaults.offset);
  if(!validStoredFloat(calibration.offset, 0.8, 1.2)) calibration.offset = 1.0f;
  snprintf(key, sizeof(key), "%sled_heading", prefix);
  calibration.led_heading_offset_deg = config_store.getFloat(key, defaults.led_heading_offset_deg);
  if(!isfinite(calibration.led_heading_offset_deg) ||
     calibration.led_heading_offset_deg < 0.0f || calibration.led_heading_offset_deg >= 360.0f)
    calibration.led_heading_offset_deg = 315.0f;
  snprintf(key, sizeof(key), "%scorr_count", prefix);
  calibration.correction_point_count = config_store.getInt(key, defaults.correction_point_count);
  if(calibration.correction_point_count < 0 || calibration.correction_point_count > MAX_CORRECTION_POINTS)
  {
    clearCorrectionTable(calibration);
    return;
  }
  for(int i = 0; i < MAX_CORRECTION_POINTS; i++)
  {
    snprintf(key, sizeof(key), "%srpm%d", prefix, i);
    calibration.correction_rpm[i] = config_store.getFloat(key, defaults.correction_rpm[i]);
    snprintf(key, sizeof(key), "%soff%d", prefix, i);
    calibration.correction_offset[i] = config_store.getFloat(key, defaults.correction_offset[i]);
    snprintf(key, sizeof(key), "%sthr%d", prefix, i);
    calibration.correction_throttle[i] = constrain(config_store.getInt(key, defaults.correction_throttle[i]), -100, 100);
    if(!isfinite(calibration.correction_rpm[i]) || calibration.correction_rpm[i] < 0.0f)
      calibration.correction_rpm[i] = 0.0f;
    if(!validStoredFloat(calibration.correction_offset[i], 0.8, 1.2))
      calibration.correction_offset[i] = 1.0f;
  }
  sortCorrectionTable(calibration);
}

void save_spin_calibration(SpinCalibration &calibration, const char *prefix)
{
  sortCorrectionTable(calibration);
  char key[16];
  snprintf(key, sizeof(key), "%srpm_offset", prefix);
  config_store.putFloat(key, calibration.offset);
  snprintf(key, sizeof(key), "%sled_heading", prefix);
  config_store.putFloat(key, calibration.led_heading_offset_deg);
  snprintf(key, sizeof(key), "%scorr_count", prefix);
  config_store.putInt(key, calibration.correction_point_count);
  for(int i = 0; i < MAX_CORRECTION_POINTS; i++)
  {
    snprintf(key, sizeof(key), "%srpm%d", prefix, i);
    config_store.putFloat(key, calibration.correction_rpm[i]);
    snprintf(key, sizeof(key), "%soff%d", prefix, i);
    config_store.putFloat(key, calibration.correction_offset[i]);
    snprintf(key, sizeof(key), "%sthr%d", prefix, i);
    config_store.putInt(key, calibration.correction_throttle[i]);
  }
}

void load_drive_tuning_settings()
{
  if(config_store.getInt("drive_tune_v", 0) != DRIVE_TUNING_VERSION)
  {
    // The old coefficient used a different curve. Install the requested tuning
    // once, without clearing calibration or subsequent web UI adjustments.
    acc_rate = DEFAULT_ACC_RATE;
    percentdecel = DEFAULT_PERCENT_DECEL;
    config_store.putFloat("acc_rate", acc_rate);
    config_store.putFloat("pulse_pct", percentdecel);
    config_store.putInt("drive_tune_v", DRIVE_TUNING_VERSION);
    DEBUG_LOG("drive tuning: installed nonlinear spin-up and 25% pulses");
    return;
  }

  acc_rate = config_store.getFloat("acc_rate", DEFAULT_ACC_RATE);
  if(!validStoredFloat(acc_rate, MIN_ACC_RATE, MAX_ACC_RATE))
    acc_rate = DEFAULT_ACC_RATE;

  percentdecel = config_store.getFloat("pulse_pct", DEFAULT_PERCENT_DECEL);
  if(!validStoredFloat(percentdecel, MIN_PERCENT_DECEL, MAX_PERCENT_DECEL))
    percentdecel = DEFAULT_PERCENT_DECEL;
}

void load_spin_ramp_setting()
{
  spin_ramp_start_rpm = config_store.getFloat("ramp_start_rpm", NAN);
  if(isnan(spin_ramp_start_rpm))
  {
    // Preserve the existing saved coefficient's curve on the first upgrade.
    // Later coefficient edits leave this knee fixed and independent.
    spin_ramp_start_rpm = powf(SPIN_ACCEL_MIN_COMMAND / acc_rate,
                               1.0f / SPIN_ACCEL_EXPONENT);
    config_store.putFloat("ramp_start_rpm", spin_ramp_start_rpm);
  }
  else if(!validStoredFloat(spin_ramp_start_rpm, MIN_SPIN_RAMP_START_RPM,
                            MAX_SPIN_RAMP_START_RPM))
  {
    spin_ramp_start_rpm = DEFAULT_SPIN_RAMP_START_RPM;
    config_store.putFloat("ramp_start_rpm", spin_ramp_start_rpm);
  }
}

void load_motor_full_scale_setting()
{
  motor_full_scale_percent = config_store.getFloat("motor_full_pct", NAN);
  if(!validStoredFloat(motor_full_scale_percent, MIN_MOTOR_FULL_SCALE_PERCENT,
                       MAX_MOTOR_FULL_SCALE_PERCENT))
  {
    motor_full_scale_percent = DEFAULT_MOTOR_FULL_SCALE_PERCENT;
    config_store.putFloat("motor_full_pct", motor_full_scale_percent);
  }
}

void load_config_settings()
{
  load_drive_tuning_settings();
  load_spin_ramp_setting();
  load_motor_full_scale_setting();

  translation_scale_factor =
      config_store.getFloat("trans_scale", DEFAULT_TRANSLATION_SCALE_FACTOR);
  if(!validStoredFloat(translation_scale_factor, MIN_TRANSLATION_SCALE_FACTOR,
                       MAX_TRANSLATION_SCALE_FACTOR))
    translation_scale_factor = DEFAULT_TRANSLATION_SCALE_FACTOR;

  top_led_phase_offset_deg =
      config_store.getFloat("top_led_trim", DEFAULT_TOP_LED_PHASE_OFFSET_DEG);
  if(!validStoredFloat(top_led_phase_offset_deg, MIN_LED_PHASE_OFFSET_DEG,
                       MAX_LED_PHASE_OFFSET_DEG))
    top_led_phase_offset_deg = DEFAULT_TOP_LED_PHASE_OFFSET_DEG;

  bottom_led_phase_offset_deg =
      config_store.getFloat("bot_led_trim", DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG);
  if(!validStoredFloat(bottom_led_phase_offset_deg, MIN_LED_PHASE_OFFSET_DEG,
                       MAX_LED_PHASE_OFFSET_DEG))
    bottom_led_phase_offset_deg = DEFAULT_BOTTOM_LED_PHASE_OFFSET_DEG;

  accel_zero_x_g_offset = config_store.getFloat("accel_zero_x", 0.0);
  if(isnan(accel_zero_x_g_offset) || fabs(accel_zero_x_g_offset) > 20.0)
  {
    accel_zero_x_g_offset = 0.0;
  }

  accel_zero_y_g_offset = config_store.getFloat("accel_zero_y", config_store.getFloat("accel_zero", 0.0));
  if(isnan(accel_zero_y_g_offset) || fabs(accel_zero_y_g_offset) > 20.0)
  {
    accel_zero_y_g_offset = 0.0;
  }

  const SpinCalibration defaults;
  load_spin_calibration(spin_calibration[0], "", defaults);
  // Seed reverse from legacy calibration until it has its own saved values.
  load_spin_calibration(spin_calibration[1], "r_", spin_calibration[0]);
}

void set_accel_zero_offset()
{
  const int offset_samples = 100;
  float x_offset_sum = 0.0;
  float y_offset_sum = 0.0;
  int valid_samples = 0;
  for(int i = 0; i < offset_samples; i++)
  {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    if(read_accel_axes_checked(x, y, z))
    {
      x_offset_sum += xl.convertToG(ACCEL_MAX_SCALE, x);
      y_offset_sum += xl.convertToG(ACCEL_MAX_SCALE, y);
      valid_samples++;
    }
    delay(2);
  }
  if(valid_samples != offset_samples)
  {
    DEBUG_LOGF("accelerometer: zero rejected; only %d/%d complete samples",
               valid_samples, offset_samples);
    record_accel_loss("calibration zero did not receive all samples");
    return;
  }
  accel_zero_x_g_offset = x_offset_sum / valid_samples;
  accel_zero_y_g_offset = y_offset_sum / valid_samples;
  accel_sample_sequence = 0;
  reset_accel_filter_requested = true;
  rpm_variation_ewma = 0.0f;
  rpm_stable_since_ms = 0;
  reset_rotation_phase_requested = true;
}

void clearCorrectionTable(SpinCalibration &calibration)
{
  calibration.correction_point_count = 0;
  for(int i = 0; i < MAX_CORRECTION_POINTS; i++)
  {
    calibration.correction_rpm[i] = 0.0;
    calibration.correction_offset[i] = 1.0;
    calibration.correction_throttle[i] = 0;
  }
}

void sortCorrectionTable(SpinCalibration &calibration)
{
  for(int i = 1; i < calibration.correction_point_count; i++)
  {
    float rpm_key = calibration.correction_rpm[i];
    float offset_key = calibration.correction_offset[i];
    int throttle_key = calibration.correction_throttle[i];
    int j = i - 1;
    while(j >= 0 && calibration.correction_rpm[j] > rpm_key)
    {
      calibration.correction_rpm[j + 1] = calibration.correction_rpm[j];
      calibration.correction_offset[j + 1] = calibration.correction_offset[j];
      calibration.correction_throttle[j + 1] = calibration.correction_throttle[j];
      j--;
    }
    calibration.correction_rpm[j + 1] = rpm_key;
    calibration.correction_offset[j + 1] = offset_key;
    calibration.correction_throttle[j + 1] = throttle_key;
  }
}

int setCorrectionPoint(float point_rpm, float point_offset, int point_throttle,
                       bool *replaced_full_table)
{
  SpinCalibration &calibration = spin_calibration[reversed];
  point_offset = constrain(point_offset, 0.8, 1.2);
  point_throttle = constrain(point_throttle, -100, 100);
  bool table_was_full = calibration.correction_point_count >= MAX_CORRECTION_POINTS;

  int slot = calibration.correction_point_count;
  if(calibration.correction_point_count > 0)
  {
    int closest_slot = 0;
    float closest_delta = fabs(calibration.correction_rpm[0] - point_rpm);
    for(int i = 1; i < calibration.correction_point_count; i++)
    {
      float delta = fabs(calibration.correction_rpm[i] - point_rpm);
      if(delta < closest_delta)
      {
        closest_delta = delta;
        closest_slot = i;
      }
    }
    if(closest_delta <= 75.0f)
    {
      slot = closest_slot;
    }
  }

  if(slot == calibration.correction_point_count && calibration.correction_point_count < MAX_CORRECTION_POINTS)
  {
    calibration.correction_point_count++;
  }
  else if(slot == calibration.correction_point_count)
  {
    slot = 0;
    float best_delta = fabs(calibration.correction_rpm[0] - point_rpm);
    for(int i = 1; i < MAX_CORRECTION_POINTS; i++)
    {
      float delta = fabs(calibration.correction_rpm[i] - point_rpm);
      if(delta < best_delta)
      {
        best_delta = delta;
        slot = i;
      }
    }
  }

  calibration.correction_rpm[slot] = point_rpm;
  calibration.correction_offset[slot] = point_offset;
  calibration.correction_throttle[slot] = point_throttle;
  sortCorrectionTable(calibration);

  int sorted_index = 0;
  for(int i = 0; i < calibration.correction_point_count; i++)
  {
    if(calibration.correction_rpm[i] == point_rpm &&
       calibration.correction_offset[i] == point_offset &&
       calibration.correction_throttle[i] == point_throttle)
    {
      sorted_index = i;
      break;
    }
  }
  if(replaced_full_table != nullptr)
  {
    *replaced_full_table = table_was_full;
  }
  return sorted_index;
}

float getRotationOffsetForRpm(float current_rpm)
{
  const SpinCalibration &calibration = spin_calibration[reversed];
  // RPM calibration uses the live offset being adjusted. Heading calibration
  // uses the staged table so the heading is checked against the full curve.
  if((config_mode == true && config_calibration_stage == CONFIG_STAGE_RPM) ||
     calibration.correction_point_count == 0)
  {
    return constrain(calibration.offset, 0.8, 1.2);
  }

  if(current_rpm <= calibration.correction_rpm[0])
  {
    return calibration.correction_offset[0];
  }

  int last = calibration.correction_point_count - 1;
  if(current_rpm >= calibration.correction_rpm[last])
  {
    return calibration.correction_offset[last];
  }

  for(int i = 1; i < calibration.correction_point_count; i++)
  {
    if(current_rpm <= calibration.correction_rpm[i])
    {
      float low_rpm = calibration.correction_rpm[i - 1];
      float high_rpm = calibration.correction_rpm[i];
      if(high_rpm == low_rpm)
      {
        return calibration.correction_offset[i];
      }
      float frac = (current_rpm - low_rpm) / (high_rpm - low_rpm);
      return calibration.correction_offset[i - 1] + frac * (calibration.correction_offset[i] - calibration.correction_offset[i - 1]);
    }
  }

  return constrain(calibration.offset, 0.8, 1.2);
}
