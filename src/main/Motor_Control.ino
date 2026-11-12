portMUX_TYPE motor_control_mux = portMUX_INITIALIZER_UNLOCKED;
volatile MotorControlSnapshot active_motor_control;
TaskHandle_t motor_output_task_handle = nullptr;
esp_timer_handle_t motor_output_timer_handle = nullptr;
volatile uint32_t motor_scheduler_overrun_count = 0;
volatile uint32_t motor_scheduler_max_interval_us = 0;
volatile uint32_t dshot_tx_error_count = 0;
volatile uint16_t last_dshot_left_value = 0;
volatile uint16_t last_dshot_right_value = 0;
volatile int last_motor_left_command = 0;
volatile int last_motor_right_command = 0;
portMUX_TYPE motor_interval_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t motor_interval_left_sum = 0;
volatile int64_t motor_interval_right_sum = 0;
volatile uint32_t motor_interval_output_count = 0;
volatile bool motor_interval_translation_seen = false;

// Publish every phase tick so narrow translation windows receive twice the
// former number of motor frames and do not favor 0/180 over 90/270 degrees.
const uint32_t DSHOT_OUTPUT_DIVIDER = 1; // 2 kHz task / 1 = 2 kHz DShot

int perk2dshot(int throttle)
{
  if(throttle == 0)
  {
    return 0;
  }
  if(throttle > 0)
  {
    return min(throttle, 999) + 48;
  }
  return min(throttle * -1, 998) + 1049;
}

int spin_requested_command(float raw_throttle_percent)
{
  const float endpoint = constrain(motor_full_scale_percent,
                                   MIN_MOTOR_FULL_SCALE_PERCENT,
                                   MAX_MOTOR_FULL_SCALE_PERCENT);
  const float fraction = constrain(raw_throttle_percent / endpoint, 0.0f, 1.0f);
  return (int)lroundf(fraction * 999.0f);
}

int scale_motor_output(int logical_command, RobotState state)
{
  logical_command = constrain(logical_command, -999, 999);
  if(state == ROBOT_TANK)
  {
    return logical_command;
  }
  const float scale = constrain(motor_full_scale_percent,
                                MIN_MOTOR_FULL_SCALE_PERCENT,
                                MAX_MOTOR_FULL_SCALE_PERCENT) * 0.01f;
  return (int)lroundf(logical_command * scale);
}

void reset_tank_boost()
{
  tank_boost_active = false;
  tank_boost_sampled = false;
  tank_boost_last_sample_ms = 0;
}

void update_tank_boost()
{
  if(robot_state != ROBOT_TANK || config_mode || wifi_mode_active ||
     !rc_status || !accel_sensor_healthy || battery_critical_latched)
  {
    reset_tank_boost();
    return;
  }

  const unsigned long now_ms = millis();
  if(tank_boost_sampled &&
     now_ms - tank_boost_last_sample_ms < TANK_BOOST_SAMPLE_INTERVAL_MS)
  {
    return;
  }
  tank_boost_last_sample_ms = now_ms;
  tank_boost_sampled = true;
  // Level-triggered: release restores the normal limit on the next sample.
  tank_boost_active = duty[CH_CONFIG_SAVE] > 50;
}

void tank_drive()
{
  update_tank_boost();
  const int drive_limit = tank_boost_active ? TANK_BOOST_DRIVE_LIMIT
                                           : TANK_DRIVE_LIMIT;
  const int steering_limit = tank_boost_active ? TANK_BOOST_STEERING_LIMIT
                                              : TANK_STEERING_LIMIT;
  const int steering = map(duty[CH_TRANSLATE_LR], 0, 100,
                           steering_limit, -steering_limit);
  // Preserve ordinary tank direction regardless of the selected spin direction.
  motorL = constrain((int)map(duty[CH_TRANSLATE_FB], 0, 100,
                              drive_limit, -drive_limit) + steering, -999, 999);
  motorR = constrain((int)map(duty[CH_TRANSLATE_FB], 0, 100,
                              -drive_limit, drive_limit) + steering, -999, 999);
}

int translation_command(int base_command, bool accelerate, float magnitude)
{
  base_command = constrain(base_command, -999, 999);
  magnitude = constrain(magnitude, 0.0f, 1.0f);
  if(base_command == 0)
  {
    return 0;
  }

  int target_command = 0;
  if(accelerate)
  {
    target_command = base_command > 0 ? 999 : -999;
  }

  int result = round(base_command + (target_command - base_command) * magnitude);

  // Translation may reduce a motor to zero, but it may never reverse it.
  return base_command > 0 ? constrain(result, 0, 999)
                          : constrain(result, -999, 0);
}

void motor_output_timer_callback(void *parameter)
{
  (void)parameter;
  if(motor_output_task_handle != nullptr)
  {
    xTaskNotifyGive(motor_output_task_handle);
  }
}

bool robot_state_allows_motor_output(RobotState state)
{
  return state == ROBOT_TANK || state == ROBOT_SPINUP ||
         state == ROBOT_MELTY || state == ROBOT_CONFIG;
}

bool motor_output_gate_open()
{
  return robot_state_allows_motor_output(robot_state) &&
         dshot_outputs_ready && rc_status && accel_sensor_healthy &&
         battery_critical_latched == false && wifi_mode_active == false;
}

void copy_motor_snapshot(MotorControlSnapshot &destination, const volatile MotorControlSnapshot &source)
{
  destination.state = source.state;
  destination.motor_l = source.motor_l;
  destination.motor_r = source.motor_r;
  destination.output_enabled = source.output_enabled;
  destination.translation_enabled = source.translation_enabled;
  destination.translation_magnitude = source.translation_magnitude;
  destination.target_phase_deg = source.target_phase_deg;
  destination.heading_phase_deg = source.heading_phase_deg;
  destination.translation_led_phase_deg = source.translation_led_phase_deg;
  destination.estimated_rpm = source.estimated_rpm;
  destination.rotation_offset = source.rotation_offset;
  destination.reversed_spin = source.reversed_spin;
  destination.phase_led_mode = source.phase_led_mode;
}

void update_motors()
{
  MotorControlSnapshot next;
  next.state = robot_state;
  next.motor_l = constrain(motorL, -999, 999);
  next.motor_r = constrain(motorR, -999, 999);
  next.output_enabled = motor_output_gate_open();
  next.translation_enabled = pending_translation_enabled;
  next.translation_magnitude = constrain(pending_translation_magnitude, 0.0, 1.0);
  next.target_phase_deg = wrap360(pending_target_phase_deg);
  next.heading_phase_deg = current_led_heading_phase_deg();
  // Apply the calibrated physical LED offset only to the displayed copy of
  // the requested translation direction. Reverse-spin compensation may use a
  // different internal motor phase, but the displayed vector stays referenced
  // to the field direction requested by the stick.
  next.translation_led_phase_deg = wrap360(pending_translation_display_phase_deg +
                                           spin_calibration[reversed].led_heading_offset_deg);
  next.estimated_rpm = max(0.0f, (float)rpm);
  next.rotation_offset = getRotationOffsetForRpm(next.estimated_rpm);
  next.reversed_spin = reversed;
  next.phase_led_mode = phase_led_mode;

  portENTER_CRITICAL(&motor_control_mux);
  active_motor_control.state = next.state;
  active_motor_control.motor_l = next.motor_l;
  active_motor_control.motor_r = next.motor_r;
  active_motor_control.output_enabled = next.output_enabled;
  active_motor_control.translation_enabled = next.translation_enabled;
  active_motor_control.translation_magnitude = next.translation_magnitude;
  active_motor_control.target_phase_deg = next.target_phase_deg;
  active_motor_control.heading_phase_deg = next.heading_phase_deg;
  active_motor_control.translation_led_phase_deg = next.translation_led_phase_deg;
  active_motor_control.estimated_rpm = next.estimated_rpm;
  active_motor_control.rotation_offset = next.rotation_offset;
  active_motor_control.reversed_spin = next.reversed_spin;
  active_motor_control.phase_led_mode = next.phase_led_mode;
  portEXIT_CRITICAL(&motor_control_mux);
}

void report_motor_output_status()
{
#ifdef ENABLE_SERIAL_DEBUG
  static unsigned long last_report_ms = 0;
  unsigned long now_ms = millis();
  bool tank_report = robot_state == ROBOT_TANK && now_ms - last_report_ms >= 500;
  bool idle_report = robot_state == ROBOT_ARMED_IDLE && now_ms - last_report_ms >= 1000;
  if(!tank_report && !idle_report)
  {
    return;
  }
  last_report_ms = now_ms;

  MotorControlSnapshot control;
  portENTER_CRITICAL(&motor_control_mux);
  copy_motor_snapshot(control, active_motor_control);
  portEXIT_CRITICAL(&motor_control_mux);

  if(idle_report)
  {
    DEBUG_LOGF("motor idle output: left_dshot=%u right_dshot=%u gate=%d dshot_errors=%lu",
               (unsigned int)last_dshot_left_value,
               (unsigned int)last_dshot_right_value,
               control.output_enabled ? 1 : 0,
               (unsigned long)dshot_tx_error_count);
  }
  else
  {
    DEBUG_LOGF("motor demand: left=%d right=%d gate=%d dshot_errors=%lu",
               control.motor_l, control.motor_r,
               control.output_enabled ? 1 : 0,
               (unsigned long)dshot_tx_error_count);
  }
#endif
}

void motor_output_task(void *parameter)
{
  (void)parameter;
  int64_t last_phase_us = esp_timer_get_time();
  uint32_t dshot_output_divider_count = 0;

  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    MotorControlSnapshot control;
    portENTER_CRITICAL(&motor_control_mux);
    copy_motor_snapshot(control, active_motor_control);
    portEXIT_CRITICAL(&motor_control_mux);

    int64_t now_us = esp_timer_get_time();
    int64_t raw_scheduler_interval_us = now_us - last_phase_us;
    uint32_t scheduler_interval_us =
        raw_scheduler_interval_us > 0 ? (uint32_t)raw_scheduler_interval_us : 0;
    if(scheduler_interval_us > motor_scheduler_max_interval_us)
    {
      motor_scheduler_max_interval_us = scheduler_interval_us;
    }
    uint32_t elapsed_periods =
        scheduler_interval_us / (uint32_t)MOTOR_TASK_PERIOD_US;
    if(elapsed_periods > 1)
    {
      motor_scheduler_overrun_count += elapsed_periods - 1;
    }
    float dt = constrain(scheduler_interval_us * 0.000001f, 0.0f, 0.02f);
    last_phase_us = now_us;

    if(reset_rotation_phase_requested)
    {
      rotation_phase_deg = 0.0f;
      reset_rotation_phase_requested = false;
    }
    else if(control.estimated_rpm >= 200.0f && control.rotation_offset > 0.01f)
    {
      rotation_phase_deg = wrap360(rotation_phase_deg +
                                   dt * (control.estimated_rpm / control.rotation_offset) * 6.0f);
    }
    else
    {
      rotation_phase_deg = 0.0f;
    }
    angle = (int)rotation_phase_deg;

    unsigned long last_rc_frame = last_valid_rc_frame_ms;
    bool esc_link_healthy = rc_status && last_rc_frame != 0 &&
                            (millis() - last_rc_frame) <= RC_FRAME_TIMEOUT_MS &&
                            accel_sensor_healthy &&
                            battery_critical_latched == false &&
                            wifi_mode_active == false;
    bool live_safety_gate = robot_state_allows_motor_output(control.state) &&
                            control.output_enabled && esc_link_healthy;

    int output_l = control.motor_l;
    int output_r = control.motor_r;
    bool pulse_active = false;

    bool translation_scheduler_enabled =
        live_safety_gate && control.translation_enabled &&
        control.estimated_rpm > 400.0f;
    bool positive_guide = false;
    bool negative_guide = false;

    if(translation_scheduler_enabled)
    {
      float phase_delta = control.reversed_spin
                            ? wrap360(control.target_phase_deg - rotation_phase_deg)
                            : wrap360(rotation_phase_deg - control.target_phase_deg);
      float pulse_width_deg = constrain(percentdecel, MIN_PERCENT_DECEL,
                                        MAX_PERCENT_DECEL) * 3.6f;
      positive_guide = phase_delta < pulse_width_deg;
      negative_guide = phase_delta >= 180.0f &&
                       phase_delta < 180.0f + pulse_width_deg;
    }

    // behavior: motor modulation is active only while the live phase is
    // inside either angular window. There is no rounded or paired frame quota.
    bool translation_frame_scheduled = positive_guide || negative_guide;
    if(translation_frame_scheduled)
    {
      float scaled_magnitude = control.translation_magnitude *
                               constrain(translation_scale_factor,
                                         MIN_TRANSLATION_SCALE_FACTOR,
                                         MAX_TRANSLATION_SCALE_FACTOR);
      bool positive_pulse = positive_guide;
      bool accelerate_right = positive_pulse != control.reversed_spin;
      output_r = translation_command(control.motor_r, accelerate_right,
                                          scaled_magnitude);
      output_l = translation_command(control.motor_l, !accelerate_right,
                                          scaled_magnitude);
      pulse_active = abs(output_r - control.motor_r) > 5 ||
                     abs(output_l - control.motor_l) > 5;
    }

    motor_on = pulse_active && esc_link_healthy;
    if(pulse_active)
    {
      portENTER_CRITICAL(&motor_interval_mux);
      motor_interval_translation_seen = true;
      portEXIT_CRITICAL(&motor_interval_mux);
    }

    // Decide phase-sensitive LED transitions on the same 2 kHz clock used by
    // translation. The lower-priority LED task performs the blocking SPI work.
    if(control.phase_led_mode != PHASE_LED_DISABLED)
    {
      // Apply the per-strip installation trims after the shared, saved heading
      // calibration. The bottom strip's baseline remains half a revolution
      // later because it faces the opposite side of the robot.
      float top_heading_phase_deg =
          wrap360(control.heading_phase_deg + top_led_phase_offset_deg);
      float bottom_heading_phase_deg =
          wrap360(control.heading_phase_deg + 180.0f + bottom_led_phase_offset_deg);
      float top_heading_phase_delta =
          wrap360(rotation_phase_deg - top_heading_phase_deg);
      float bottom_heading_phase_delta =
          wrap360(rotation_phase_deg - bottom_heading_phase_deg);
      float top_heading_diff =
          min(top_heading_phase_delta, 360.0f - top_heading_phase_delta);
      float bottom_heading_diff =
          min(bottom_heading_phase_delta, 360.0f - bottom_heading_phase_delta);
      bool heading_leds_enabled = control.estimated_rpm > 400.0f;
      bool top_heading_window_active =
          heading_leds_enabled && top_heading_diff <= HEADING_LED_HALF_WIDTH_DEG;
      // The underside LED faces the opposite way, so its field-heading marker
      // belongs exactly half a revolution after the top LED marker.
      bool bottom_heading_window_active =
          heading_leds_enabled && bottom_heading_diff <= HEADING_LED_HALF_WIDTH_DEG;
      float translation_led_phase_delta =
          control.reversed_spin
            ? wrap360(control.translation_led_phase_deg - rotation_phase_deg)
            : wrap360(rotation_phase_deg - control.translation_led_phase_deg);
      const float translation_led_half_width_deg =
          TRANSLATION_LED_ARC_WIDTH_DEG * 0.5f;
      float displayed_translation_magnitude =
          control.translation_magnitude *
          constrain(translation_scale_factor, MIN_TRANSLATION_SCALE_FACTOR,
                    MAX_TRANSLATION_SCALE_FACTOR);
      bool directional_led_requested =
          translation_scheduler_enabled && displayed_translation_magnitude > 0.01f;
      // Center the displayed vector windows on their requested field phases.
      // Motor modulation intentionally keeps the leading-edge windows
      // above; this visualization change cannot alter motor pulse timing.
      bool directional_led_pulse_active =
          directional_led_requested &&
          (translation_led_phase_delta <= translation_led_half_width_deg ||
           translation_led_phase_delta >= 360.0f - translation_led_half_width_deg ||
           (translation_led_phase_delta >= 180.0f - translation_led_half_width_deg &&
            translation_led_phase_delta <= 180.0f + translation_led_half_width_deg));
      PhaseLedFrame requested_frame = PHASE_LED_FRAME_OFF;

      if(live_safety_gate)
      {
        if(control.phase_led_mode == PHASE_LED_CONFIG_STABLE)
        {
          if(top_heading_window_active && bottom_heading_window_active)
          {
            requested_frame = PHASE_LED_FRAME_CONFIG_BOTH_HEADING;
          }
          else if(top_heading_window_active)
          {
            requested_frame = PHASE_LED_FRAME_CONFIG_TOP_HEADING;
          }
          else if(bottom_heading_window_active)
          {
            requested_frame = PHASE_LED_FRAME_CONFIG_BOTTOM_HEADING;
          }
          else
          {
            requested_frame = PHASE_LED_FRAME_CONFIG_STABLE;
          }
        }
        else if(control.phase_led_mode == PHASE_LED_CONFIG_DARK)
        {
          // During the heading-offset forward test, display the same two
          // phase-window translation vector used in normal MELTY operation.
          // Without a translation request, retain the solid dark-red
          // calibration background so the active stage remains obvious.
          if(top_heading_window_active && bottom_heading_window_active)
          {
            requested_frame = PHASE_LED_FRAME_BOTH_HEADING;
          }
          else if(top_heading_window_active)
          {
            requested_frame = directional_led_requested
                                ? (directional_led_pulse_active
                                     ? PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR
                                     : PHASE_LED_FRAME_TOP_HEADING)
                                : PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR;
          }
          else if(bottom_heading_window_active)
          {
            requested_frame = directional_led_requested
                                ? (directional_led_pulse_active
                                     ? PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING
                                     : PHASE_LED_FRAME_BOTTOM_HEADING)
                                : PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING;
          }
          else if(directional_led_requested)
          {
            requested_frame = directional_led_pulse_active
                                ? PHASE_LED_FRAME_MOTOR_ON
                                : PHASE_LED_FRAME_OFF;
          }
          else
          {
            requested_frame = PHASE_LED_FRAME_MOTOR_ON;
          }
        }
        else if(control.phase_led_mode == PHASE_LED_MELTY)
        {
          // LED priority with calibrated field display: green heading,
          // then the offset-adjusted translation direction in dark red.
          if(top_heading_window_active && bottom_heading_window_active)
          {
            requested_frame = PHASE_LED_FRAME_BOTH_HEADING;
          }
          else if(top_heading_window_active)
          {
            requested_frame = directional_led_pulse_active
                                ? PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR
                                : PHASE_LED_FRAME_TOP_HEADING;
          }
          else if(bottom_heading_window_active)
          {
            requested_frame = directional_led_pulse_active
                                ? PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING
                                : PHASE_LED_FRAME_BOTTOM_HEADING;
          }
          else if(directional_led_pulse_active)
          {
            requested_frame = PHASE_LED_FRAME_MOTOR_ON;
          }
          else
          {
            requested_frame = PHASE_LED_FRAME_OFF;
          }
        }
      }

      publish_phase_led_frame(control.phase_led_mode, requested_frame);
    }

    if(live_safety_gate == false)
    {
      output_l = 0;
      output_r = 0;
    }

    // Tank uses its own normal/boost limits and bypasses the configured ceiling.
    // Use the demand snapshot's state for scaling; safety still gates every mode.
    output_l = scale_motor_output(output_l, control.state);
    output_r = scale_motor_output(output_r, control.state);

    if(reverse_right_motor)
    {
      output_r = -output_r;
    }
    if(reverse_left_motor)
    {
      output_l = -output_l;
    }

    dshot_output_divider_count++;
    if(dshot_output_divider_count < DSHOT_OUTPUT_DIVIDER)
    {
      continue;
    }
    dshot_output_divider_count = 0;

    // Accumulate only the final signed command that reaches the DShot publish
    // boundary. The 10 Hz logger atomically snapshots and resets this interval.
    portENTER_CRITICAL(&motor_interval_mux);
    motor_interval_left_sum += output_l;
    motor_interval_right_sum += output_r;
    motor_interval_output_count++;
    portEXIT_CRITICAL(&motor_interval_mux);

    dshot_result_t right_tx_result;
    dshot_result_t left_tx_result;

    // Direct port of PotatoMelt-ESP32's output path.
    // Make the state gate dominant at the final frame boundary. In particular,
    // ARMED_IDLE always emits command 0 even if an older snapshot had demand.
    uint16_t right_dshot = live_safety_gate ? perk2dshot(output_r) : 0;
    uint16_t left_dshot = live_safety_gate ? perk2dshot(output_l) : 0;
    last_motor_right_command = live_safety_gate ? output_r : 0;
    last_motor_left_command = live_safety_gate ? output_l : 0;
    last_dshot_right_value = right_dshot;
    last_dshot_left_value = left_dshot;
    right_tx_result = escR.sendThrottle(right_dshot);
    left_tx_result = escL.sendThrottle(left_dshot);
    if(!right_tx_result.success) dshot_tx_error_count++;
    if(!left_tx_result.success) dshot_tx_error_count++;

  }
}

void snapshot_motor_output_interval(int &left_average, int &right_average,
                                    bool &translation_seen)
{
  int64_t left_sum;
  int64_t right_sum;
  uint32_t output_count;
  portENTER_CRITICAL(&motor_interval_mux);
  left_sum = motor_interval_left_sum;
  right_sum = motor_interval_right_sum;
  output_count = motor_interval_output_count;
  translation_seen = motor_interval_translation_seen;
  motor_interval_left_sum = 0;
  motor_interval_right_sum = 0;
  motor_interval_output_count = 0;
  motor_interval_translation_seen = false;
  portEXIT_CRITICAL(&motor_interval_mux);

  if(output_count == 0)
  {
    left_average = 0;
    right_average = 0;
    return;
  }
  left_average = constrain((int)(left_sum / (int64_t)output_count), -999, 999);
  right_average = constrain((int)(right_sum / (int64_t)output_count), -999, 999);
}

void init_motor_output_task()
{
  portENTER_CRITICAL(&motor_control_mux);
  active_motor_control.state = ROBOT_FAILSAFE;
  active_motor_control.motor_l = 0;
  active_motor_control.motor_r = 0;
  active_motor_control.output_enabled = false;
  active_motor_control.translation_enabled = false;
  active_motor_control.translation_magnitude = 0.0f;
  active_motor_control.target_phase_deg = 0.0f;
  active_motor_control.heading_phase_deg = 0.0f;
  active_motor_control.translation_led_phase_deg = 0.0f;
  active_motor_control.estimated_rpm = 0.0f;
  active_motor_control.rotation_offset = 1.0f;
  active_motor_control.reversed_spin = false;
  active_motor_control.phase_led_mode = PHASE_LED_DISABLED;
  portEXIT_CRITICAL(&motor_control_mux);

  if(motor_output_task_handle == nullptr)
  {
    BaseType_t task_result =
        xTaskCreate(motor_output_task, "motor-output", 4096, nullptr, 4,
                    &motor_output_task_handle);
    if(task_result != pdPASS)
    {
      motor_output_task_handle = nullptr;
      DEBUG_LOG("motor scheduler: task creation FAILED");
      return;
    }
  }

  if(motor_output_timer_handle == nullptr)
  {
    const esp_timer_create_args_t timer_args = {
      .callback = &motor_output_timer_callback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "motor-2khz",
      .skip_unhandled_events = true
    };
    esp_err_t timer_result =
        esp_timer_create(&timer_args, &motor_output_timer_handle);
    if(timer_result != ESP_OK)
    {
      DEBUG_LOGF("motor scheduler: timer creation failed (%d)", timer_result);
      motor_output_timer_handle = nullptr;
      return;
    }

    timer_result =
        esp_timer_start_periodic(motor_output_timer_handle,
                                 MOTOR_TASK_PERIOD_US);
    if(timer_result != ESP_OK)
    {
      DEBUG_LOGF("motor scheduler: timer start failed (%d)", timer_result);
      esp_timer_delete(motor_output_timer_handle);
      motor_output_timer_handle = nullptr;
      return;
    }
    DEBUG_LOGF("motor scheduler: running at %lu us",
               (unsigned long)MOTOR_TASK_PERIOD_US);
  }
}

int spin_acceleration_ceiling(int measured_rpm)
{
  // Hold the startup command up to the knee, then apply the curved rise.
  // The coefficient changes the rise without moving the configured knee.
  const float ramp_rpm = max((float)max(0, measured_rpm), spin_ramp_start_rpm);
  const float ceiling = SPIN_ACCEL_MIN_COMMAND + acc_rate *
      (powf(ramp_rpm, SPIN_ACCEL_EXPONENT) -
       powf(spin_ramp_start_rpm, SPIN_ACCEL_EXPONENT));
  return (int)constrain(ceiling, SPIN_ACCEL_MIN_COMMAND, 999.0f);
}

void spin()
{
  pending_translation_enabled = false;
  pending_translation_magnitude = 0.0f;
  const int magnitude = min(spin_requested_command(spin_throttle_input_percent),
                            spin_acceleration_ceiling(rpm));
  spinspeed = reversed ? -magnitude : magnitude;
  motorR = constrain(spinspeed, -999, 999);
  motorL = constrain(spinspeed, -999, 999);
}

bool translation_active()
{
  return stick_percent_centered(CH_TRANSLATE_FB) != 0 ||
         stick_percent_centered(CH_TRANSLATE_LR) != 0;
}

void translate_field_oriented()
{
  spin();

  float field_forward = stick_percent_centered(CH_TRANSLATE_FB) / 50.0f;
  // This transmitter's CH1 polarity is opposite the desired field-right axis.
  float field_right = -stick_percent_centered(CH_TRANSLATE_LR) / 50.0f;
  float magnitude = max(fabs(field_forward), fabs(field_right));
  if(magnitude < 0.01f)
  {
    return;
  }

  float display_vector_angle_deg =
      atan2(field_right, field_forward) * 180.0f / PI;
  pending_translation_display_phase_deg =
      wrap360(drive_heading_phase_deg + display_vector_angle_deg);

  float motor_forward = field_forward;
  float motor_right = field_right;
  if(reversed)
  {
    // Empirical inverse for this drivetrain's reverse-spin field mapping.
    // The previous crossed-and-negated transform produced an exact 180-degree
    // error (forward -> back and left -> right), so retain the axis exchange
    // while removing both sign inversions. Keep this motor-only so the LED
    // still shows the direction requested by the stick.
    motor_forward = field_right;
    motor_right = field_forward;
  }

  float motor_vector_angle_deg =
      atan2(motor_right, motor_forward) * 180.0f / PI;
  pending_target_phase_deg =
      wrap360(drive_heading_phase_deg + motor_vector_angle_deg);
  pending_translation_magnitude = constrain(magnitude, 0.0f, 1.0f);
  pending_translation_enabled = true;
}

void translate()
{
  translate_field_oriented();
}
