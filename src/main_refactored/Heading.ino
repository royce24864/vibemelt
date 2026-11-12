void record_accel_loss(const char *reason)
{
  unsigned long now = millis();
  bool new_incident = accel_loss_started_ms == 0;
  if(new_incident)
  {
    accel_loss_started_ms = now == 0 ? 1 : now;
    accel_fault_generation++;
  }
  accel_sensor_healthy = false;
  latch_control_rearm();
  reset_accel_filter_requested = true;
  last_accel_sample_us = 0;
  current_gforce_valid = false;
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
      last_accel_sample_us = 0;
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

void calcrpm()
{
  update_accel_sensor_health();
  get_accel_force_g();
  if(!accel_sensor_healthy || !accel_estimator.valid)
  {
    current_gforce = 0.0f;
    current_gforce_valid = false;
    rpm = 0;
    rpm_variation_ewma = 0.0f;
    rpm_stable_since_ms = 0;
    return;
  }
  current_gforce = accel_estimator.filtered_g;
  current_gforce_valid = true;
  rpm = accel_estimator.rpm;
  rpm_variation_ewma = accel_estimator.variation;
  rpm_stable_since_ms = accel_estimator.stable_since_ms;
  if(rpm > max_rpm) max_rpm = rpm;
}

bool rpm_is_stable()
{
  return accel_sensor_healthy && estimator::stable(accel_estimator, accel_estimator_config, millis());
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
  if(reset_accel_filter_requested)
  {
    estimator::reset(accel_estimator);
    last_accel_sample_us = 0;
    last_accel_sample_ms = 0;
    reset_accel_filter_requested = false;
  }
  int64_t now_us = esp_timer_get_time();
  if(last_accel_sample_us != 0 && !config_lifecycle_output_inhibited &&
     !motor_timing::fresh(now_us, last_accel_sample_us, motor_timing::SENSOR_MAX_AGE_US))
    latch_control_rearm();
  if(now_us - loopstart < ACCEL_SAMPLE_INTERVAL_US || !accel_sensor_healthy)
    return accel_estimator.filtered_g;
  int16_t x = 0, y = 0, z = 0;
  loopstart = now_us;
  if(!read_accel_axes_checked(x, y, z))
  {
    if(++accel_read_failure_count >= ACCEL_READ_FAILURE_LIMIT)
      record_accel_loss("five consecutive axis reads were incomplete");
    return accel_estimator.filtered_g;
  }
  accel_read_failure_count = 0;
  float sample_g = 0.0f;
  const unsigned long sample_ms = millis();
  if(!estimator::accept_sample(accel_estimator, accel_estimator_config,
      xl.convertToG(ACCEL_MAX_SCALE, x), xl.convertToG(ACCEL_MAX_SCALE, y),
      accel_zero_x_g_offset, accel_zero_y_g_offset, sample_ms, sample_g))
  {
    record_accel_loss("non-finite acceleration sample");
    return accel_estimator.filtered_g;
  }
  // Only a complete, finite sample refreshes the command's sensor timestamp.
  // Include time spent in I2C: a read may cross the age limit before succeeding.
  const int64_t accepted_us = esp_timer_get_time();
  if(last_accel_sample_us != 0 && !config_lifecycle_output_inhibited &&
     !motor_timing::fresh(accepted_us, last_accel_sample_us, motor_timing::SENSOR_MAX_AGE_US))
    latch_control_rearm();
  last_accel_sample_ms = sample_ms;
  last_accel_sample_us = accepted_us;
  ++accel_sample_sequence;
  if(round(sample_g) > max_gforce) max_gforce = round(sample_g);
  return accel_estimator.filtered_g;
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
  return controller::wrap_degrees(deg);
}

float current_led_heading_phase_deg()
{
  return wrap360(drive_heading_phase_deg + led_heading_offset_deg);
}

int stick_percent_centered(int channel)
{
  return controller::centered(duty[channel], stick_deadzone_percent);
}

void update_heading_reference()
{
  unsigned long now = millis();
  if(last_heading_update_ms != 0)
    drive_heading_phase_deg = controller::heading_after_yaw(drive_heading_phase_deg,
        duty[CH_HEADING_YAW], stick_deadzone_percent, heading_yaw_rate_dps,
        (now - last_heading_update_ms) * 0.001f);
  last_heading_update_ms = now;
}
AccelZeroReason accel_zero_cancel_reason()
{
  update_channels();
  reset_loop_task_watchdog();
  get_battery();
  return calibration::zero_cancel_reason(current_calibration_safety(),
      duty[CH_CONFIG_MODE] > 50, config_entry_controls_safe());
}

AccelZeroResult set_accel_zero_offset()
{
  const int offset_samples = 100;
  AccelZeroResult result = {ACCEL_ZERO_SUCCESS, 0, offset_samples};
  calibration::ZeroAccumulator zero;
  for(int i = 0; i < offset_samples; i++)
  {
    result.reason = accel_zero_cancel_reason();
    if(result.reason != ACCEL_ZERO_SUCCESS) return result;
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    if(!read_accel_axes_checked(x, y, z))
    {
      record_accel_loss("calibration zero received an incomplete sample");
      result.reason = ACCEL_ZERO_SENSOR_FAULT;
      return result;
    }
    if(!calibration::add_zero_sample(zero, xl.convertToG(ACCEL_MAX_SCALE, x),
                                      xl.convertToG(ACCEL_MAX_SCALE, y)))
    {
      result.reason = ACCEL_ZERO_INVALID_OFFSET;
      return result;
    }
    result.valid_samples = zero.samples;
    delay(2);
  }

  result.reason = accel_zero_cancel_reason();
  if(result.reason != ACCEL_ZERO_SUCCESS) return result;
  if(!accel_configuration_valid())
  {
    record_accel_loss("sensor configuration invalid after calibration zero");
    result.reason = ACCEL_ZERO_SENSOR_FAULT;
    return result;
  }
  float x_zero = 0.0f, y_zero = 0.0f;
  if(!calibration::finish_zero(zero, offset_samples, x_zero, y_zero))
  {
    result.reason = ACCEL_ZERO_INVALID_OFFSET;
    return result;
  }

  // Publish offsets only after the complete acquisition passes every check.
  accel_zero_x_g_offset = x_zero;
  accel_zero_y_g_offset = y_zero;
  accel_sample_sequence = 0;
  last_accel_sample_us = 0;
  reset_accel_filter_requested = true;
  rpm_variation_ewma = 0.0f;
  rpm_stable_since_ms = 0;
  reset_rotation_phase_requested = true;
  return result;
}

void clearCorrectionTable()
{
  correction_point_count = 0;
  for(int i = 0; i < MAX_CORRECTION_POINTS; i++)
  {
    correction_rpm[i] = 0.0;
    correction_offset[i] = 1.0;
    correction_throttle[i] = 0;
  }
}

void sortCorrectionTable()
{
  for(int i = 1; i < correction_point_count; i++)
  {
    float rpm_key = correction_rpm[i];
    float offset_key = correction_offset[i];
    int throttle_key = correction_throttle[i];
    int j = i - 1;
    while(j >= 0 && correction_rpm[j] > rpm_key)
    {
      correction_rpm[j + 1] = correction_rpm[j];
      correction_offset[j + 1] = correction_offset[j];
      correction_throttle[j + 1] = correction_throttle[j];
      j--;
    }
    correction_rpm[j + 1] = rpm_key;
    correction_offset[j + 1] = offset_key;
    correction_throttle[j + 1] = throttle_key;
  }
}

int setCorrectionPoint(float point_rpm, float point_offset, int point_throttle,
                       bool *replaced_full_table)
{
  point_offset = constrain(point_offset, 0.8, 1.2);
  point_throttle = constrain(point_throttle, -100, 100);
  bool table_was_full = correction_point_count >= MAX_CORRECTION_POINTS;

  int slot = correction_point_count;
  if(correction_point_count > 0)
  {
    int closest_slot = 0;
    float closest_delta = fabs(correction_rpm[0] - point_rpm);
    for(int i = 1; i < correction_point_count; i++)
    {
      float delta = fabs(correction_rpm[i] - point_rpm);
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

  if(slot == correction_point_count && correction_point_count < MAX_CORRECTION_POINTS)
  {
    correction_point_count++;
  }
  else if(slot == correction_point_count)
  {
    slot = 0;
    float best_delta = fabs(correction_rpm[0] - point_rpm);
    for(int i = 1; i < MAX_CORRECTION_POINTS; i++)
    {
      float delta = fabs(correction_rpm[i] - point_rpm);
      if(delta < best_delta)
      {
        best_delta = delta;
        slot = i;
      }
    }
  }

  correction_rpm[slot] = point_rpm;
  correction_offset[slot] = point_offset;
  correction_throttle[slot] = point_throttle;
  sortCorrectionTable();

  int sorted_index = 0;
  for(int i = 0; i < correction_point_count; i++)
  {
    if(correction_rpm[i] == point_rpm &&
       correction_offset[i] == point_offset &&
       correction_throttle[i] == point_throttle)
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
  // RPM calibration uses the live offset being adjusted. Heading calibration
  // uses the staged table so the heading is checked against the full curve.
  if((config_mode == true && config_calibration_stage == CONFIG_STAGE_RPM) ||
     correction_point_count == 0)
  {
    return constrain(offset, 0.8, 1.2);
  }

  if(current_rpm <= correction_rpm[0])
  {
    return correction_offset[0];
  }

  int last = correction_point_count - 1;
  if(current_rpm >= correction_rpm[last])
  {
    return correction_offset[last];
  }

  for(int i = 1; i < correction_point_count; i++)
  {
    if(current_rpm <= correction_rpm[i])
    {
      float low_rpm = correction_rpm[i - 1];
      float high_rpm = correction_rpm[i];
      if(high_rpm == low_rpm)
      {
        return correction_offset[i];
      }
      float frac = (current_rpm - low_rpm) / (high_rpm - low_rpm);
      return correction_offset[i - 1] + frac * (correction_offset[i] - correction_offset[i - 1]);
    }
  }

  return constrain(offset, 0.8, 1.2);
}
