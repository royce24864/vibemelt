#include "Motor_Timing.h"

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
volatile bool motor_interval_translation_seen = false;
volatile uint32_t motor_interval_left_count = 0;
volatile uint32_t motor_interval_right_count = 0;
static bool motor_task_started = false;
static bool motor_fault_latched = false;
static bool motor_rearm_requested = false;
static motor_timing::TransportHealth motor_left_transport;
static motor_timing::TransportHealth motor_right_transport;

bool motor_output_fault_latched()
{
  portENTER_CRITICAL(&motor_control_mux);
  bool fault = motor_fault_latched;
  portEXIT_CRITICAL(&motor_control_mux);
  return fault;
}

void request_motor_output_rearm()
{
  portENTER_CRITICAL(&motor_control_mux);
  motor_rearm_requested = true;
  portEXIT_CRITICAL(&motor_control_mux);
}

void read_motor_transport_health(motor_timing::TransportHealth &left,
                                  motor_timing::TransportHealth &right)
{
  portENTER_CRITICAL(&motor_control_mux);
  left = motor_left_transport;
  right = motor_right_transport;
  portEXIT_CRITICAL(&motor_control_mux);
}

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
         !config_reentry_wait_for_low && !control_rearm_required &&
         !config_lifecycle_output_inhibited &&
         !motor_output_fault_latched() &&
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
  destination.published_at_us = source.published_at_us;
  destination.accel_sample_us = source.accel_sample_us;
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
  // the translation direction. The real motor window stays referenced to
  // target_phase_deg and its timing is unchanged.
  next.translation_led_phase_deg = wrap360(next.target_phase_deg +
                                           led_heading_offset_deg);
  next.estimated_rpm = max(0.0f, (float)rpm);
  next.rotation_offset = getRotationOffsetForRpm(next.estimated_rpm);
  next.reversed_spin = reversed;
  next.phase_led_mode = read_phase_led_snapshot().mode;
  next.published_at_us = esp_timer_get_time();
  next.accel_sample_us = last_accel_sample_us;

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
  active_motor_control.published_at_us = next.published_at_us;
  active_motor_control.accel_sample_us = next.accel_sample_us;
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

void send_motor_frame(bool left, int command, bool translation,
                      motor_timing::Safety &safety, int64_t now_us)
{
  const uint16_t value = perk2dshot(command);
  const dshot_result_t result = left ? escL.sendThrottle(value)
                                   : escR.sendThrottle(value);
  const motor_timing::TxOutcome outcome =
      result.tx_outcome == DSHOT_TX_QUEUED ? motor_timing::TxOutcome::Queued :
      result.tx_outcome == DSHOT_TX_SKIPPED ? motor_timing::TxOutcome::Skipped :
                                             motor_timing::TxOutcome::Failed;
  safety.record_tx(left, outcome, command == 0, now_us);
  if(outcome == motor_timing::TxOutcome::Queued)
  {
    // Accepted frames only, with independent denominators when one ESC skips.
    // These are driver-accepted commands, not measured physical motor output.
    portENTER_CRITICAL(&motor_interval_mux);
    if(left)
    {
      last_motor_left_command = command;
      last_dshot_left_value = value;
      motor_interval_left_sum += command;
      motor_interval_left_count++;
    }
    else
    {
      last_motor_right_command = command;
      last_dshot_right_value = value;
      motor_interval_right_sum += command;
      motor_interval_right_count++;
    }
    motor_interval_translation_seen |= translation;
    portEXIT_CRITICAL(&motor_interval_mux);
  }
  if(outcome == motor_timing::TxOutcome::Failed ||
     (outcome == motor_timing::TxOutcome::Queued && !result.success))
  {
    dshot_tx_error_count++;
    if(outcome == motor_timing::TxOutcome::Queued)
      safety.record_tx(left, motor_timing::TxOutcome::Failed, false, now_us);
  }
}

void motor_output_task(void *parameter)
{
  (void)parameter;
  // init_motor_output_task subscribes this task to the already-initialized
  // global watchdog before it releases this startup barrier.
  while(true)
  {
    portENTER_CRITICAL(&motor_control_mux);
    bool started = motor_task_started;
    portEXIT_CRITICAL(&motor_control_mux);
    if(started) break;
    vTaskDelay(1);
  }
  int64_t last_phase_us = esp_timer_get_time();
  motor_timing::Phase phase;
  motor_timing::Safety safety;
  safety.latch(); // Startup also requires accepted zero frames and neutral.
  phase.reset(last_phase_us);
  bool notification_starved = false;
  uint32_t dshot_output_divider_count = 0;

  while(true)
  {
    const uint32_t notifications = ulTaskNotifyTake(
        pdTRUE, max((TickType_t)1,
                    pdMS_TO_TICKS(motor_timing::NOTIFICATION_TIMEOUT_MS)));
    if(notifications == 0)
    {
      // Keep waking to attempt stop frames, but let the watchdog expire.
      // A later timer callback cannot erase evidence of scheduler starvation.
      notification_starved = true;
      safety.latch();
    }

    MotorControlSnapshot control;
    portENTER_CRITICAL(&motor_control_mux);
    copy_motor_snapshot(control, active_motor_control);
    bool rearm_requested = motor_rearm_requested;
    motor_rearm_requested = false;
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
    last_phase_us = now_us;

    phase.advance(now_us, control.estimated_rpm, control.rotation_offset);
    if(!phase.valid()) safety.latch();
    if(reset_rotation_phase_requested && !safety.fault())
    {
      phase.reset(now_us);
    }
    reset_rotation_phase_requested = false;
    rotation_phase_deg = phase.angle_degrees();
    angle = (int)rotation_phase_deg;

    unsigned long last_rc_frame = last_valid_rc_frame_ms;
    bool esc_link_healthy = rc_status && last_rc_frame != 0 &&
                            (millis() - last_rc_frame) <= RC_FRAME_TIMEOUT_MS &&
                            accel_sensor_healthy &&
                            battery_critical_latched == false &&
                            wifi_mode_active == false;
    bool fresh_inputs = safety.check_inputs(
        now_us, control.published_at_us, control.accel_sample_us,
        control.output_enabled, phase.valid());
    if(control.output_enabled && !esc_link_healthy) safety.latch();
    bool live_safety_gate = robot_state_allows_motor_output(control.state) &&
                            control.output_enabled && esc_link_healthy &&
                            fresh_inputs && !safety.fault() &&
                            !config_reentry_wait_for_low && !control_rearm_required;

    int output_l = control.motor_l;
    int output_r = control.motor_r;
    bool pulse_active = false;

    bool translation_scheduler_enabled =
        live_safety_gate && phase.valid() && control.translation_enabled &&
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

    // Decide phase-sensitive LED transitions on the same 2 kHz clock used by
    // translation. The lower-priority LED task performs the blocking SPI work.
    if(control.phase_led_mode != PHASE_LED_DISABLED)
    {
      float heading_phase_delta =
          wrap360(rotation_phase_deg - control.heading_phase_deg);
      float top_heading_diff =
          min(heading_phase_delta, 360.0f - heading_phase_delta);
      float bottom_heading_diff = fabs(heading_phase_delta - 180.0f);
      bool heading_leds_enabled = phase.valid() && control.estimated_rpm > 400.0f;
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
      float translation_led_width_deg =
          constrain(percentdecel, MIN_PERCENT_DECEL, MAX_PERCENT_DECEL) * 3.6f;
      float translation_led_half_width_deg = translation_led_width_deg * 0.5f;
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
          if(top_heading_window_active)
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
          if(top_heading_window_active)
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
          if(top_heading_window_active)
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

    live_safety_gate = live_safety_gate && motor_output_gate_open();
    send_motor_frame(false, live_safety_gate ? output_r : 0,
                     live_safety_gate && pulse_active, safety, now_us);
    // A right-side failure closes the left gate within this same cycle.
    send_motor_frame(true, live_safety_gate && !safety.fault() ? output_l : 0,
                     live_safety_gate && !safety.fault() && pulse_active,
                     safety, esp_timer_get_time());
    if(safety.fault())
    {
      motor_on = false;
      publish_phase_led_frame(control.phase_led_mode, PHASE_LED_FRAME_OFF);
      // One side might already have accepted a nonzero frame. Attempt a stop
      // immediately on both; timing skips are retried next tick, never waited on.
      if(live_safety_gate)
      {
        send_motor_frame(false, 0, false, safety, esp_timer_get_time());
        send_motor_frame(true, 0, false, safety, esp_timer_get_time());
      }
    }
    if(safety.rearm(rearm_requested, !control.output_enabled,
                    fresh_inputs && esc_link_healthy && !notification_starved))
    {
      phase.reset(esp_timer_get_time());
      rotation_phase_deg = 0;
      angle = 0;
    }
    portENTER_CRITICAL(&motor_control_mux);
    motor_fault_latched = safety.fault();
    motor_left_transport = safety.left();
    motor_right_transport = safety.right();
    portEXIT_CRITICAL(&motor_control_mux);

    if(!notification_starved && esp_task_wdt_reset() != ESP_OK)
    {
      notification_starved = true;
      safety.latch();
      portENTER_CRITICAL(&motor_control_mux);
      motor_fault_latched = true;
      portEXIT_CRITICAL(&motor_control_mux);
    }
  }
}

void snapshot_motor_output_interval(int &left_average, int &right_average,
                                    bool &translation_seen)
{
  int64_t left_sum;
  int64_t right_sum;
  uint32_t left_count;
  uint32_t right_count;
  portENTER_CRITICAL(&motor_interval_mux);
  left_sum = motor_interval_left_sum;
  right_sum = motor_interval_right_sum;
  left_count = motor_interval_left_count;
  right_count = motor_interval_right_count;
  translation_seen = motor_interval_translation_seen;
  motor_interval_left_sum = 0;
  motor_interval_right_sum = 0;
  motor_interval_left_count = 0;
  motor_interval_right_count = 0;
  motor_interval_translation_seen = false;
  portEXIT_CRITICAL(&motor_interval_mux);

  left_average = left_count == 0 ? 0 :
      constrain((int)(left_sum / (int64_t)left_count), -999, 999);
  right_average = right_count == 0 ? 0 :
      constrain((int)(right_sum / (int64_t)right_count), -999, 999);
}

bool init_motor_output_task()
{
  if(motor_output_task_handle != nullptr)
  {
    portENTER_CRITICAL(&motor_control_mux);
    bool ready = motor_task_started;
    portEXIT_CRITICAL(&motor_control_mux);
    return ready;
  }
  MotorControlSnapshot initial = {};
  initial.state = ROBOT_FAILSAFE;
  initial.rotation_offset = 1.0f;
  initial.phase_led_mode = PHASE_LED_DISABLED;
  initial.published_at_us = esp_timer_get_time();
  portENTER_CRITICAL(&motor_control_mux);
  // The aggregate is volatile, so use explicit field assignment.
  active_motor_control.state = initial.state;
  active_motor_control.motor_l = 0;
  active_motor_control.motor_r = 0;
  active_motor_control.output_enabled = false;
  active_motor_control.translation_enabled = false;
  active_motor_control.translation_magnitude = 0;
  active_motor_control.target_phase_deg = 0;
  active_motor_control.heading_phase_deg = 0;
  active_motor_control.translation_led_phase_deg = 0;
  active_motor_control.estimated_rpm = 0;
  active_motor_control.rotation_offset = initial.rotation_offset;
  active_motor_control.reversed_spin = false;
  active_motor_control.phase_led_mode = initial.phase_led_mode;
  active_motor_control.published_at_us = initial.published_at_us;
  active_motor_control.accel_sample_us = 0;
  motor_task_started = false;
  motor_fault_latched = true;
  portEXIT_CRITICAL(&motor_control_mux);

  const esp_timer_create_args_t timer_args = {
    .callback = &motor_output_timer_callback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "motor-2khz",
    .skip_unhandled_events = true
  };
  esp_err_t result = esp_timer_create(&timer_args, &motor_output_timer_handle);
  if(result != ESP_OK)
  {
    motor_output_timer_handle = nullptr;
    DEBUG_LOGF("motor scheduler: timer creation failed (%d)", result);
    return false;
  }
  BaseType_t task_result = xTaskCreate(
      motor_output_task, "motor-output", 4096, nullptr, 4,
      &motor_output_task_handle);
  if(task_result != pdPASS)
  {
    motor_output_task_handle = nullptr;
    esp_timer_delete(motor_output_timer_handle);
    motor_output_timer_handle = nullptr;
    DEBUG_LOG("motor scheduler: task creation failed");
    return false;
  }
  // Global WDT initialization must already have succeeded in setup().
  result = esp_task_wdt_add(motor_output_task_handle);
  bool watchdog_subscribed = result == ESP_OK;
  if(watchdog_subscribed)
    result = esp_timer_start_periodic(motor_output_timer_handle,
                                      MOTOR_TASK_PERIOD_US);
  if(result != ESP_OK)
  {
    DEBUG_LOGF("motor scheduler: watchdog/timer start failed (%d)", result);
    if(watchdog_subscribed) esp_task_wdt_delete(motor_output_task_handle);
    vTaskDelete(motor_output_task_handle);
    motor_output_task_handle = nullptr;
    esp_timer_delete(motor_output_timer_handle);
    motor_output_timer_handle = nullptr;
    return false;
  }
  portENTER_CRITICAL(&motor_control_mux);
  motor_task_started = true;
  portEXIT_CRITICAL(&motor_control_mux);
  DEBUG_LOGF("motor scheduler: running at %lu us with watchdog",
             (unsigned long)MOTOR_TASK_PERIOD_US);
  return true;
}
void spin()
{
  pending_translation_enabled = false;
  pending_translation_magnitude = 0.0f;
  spinspeed = controller::spin_demand(duty[CH_SPIN_THROTTLE], rpm,
                                      acc_rate, reversed);
  motorR = spinspeed;
  motorL = spinspeed;
}

bool translation_active()
{
  return controller::translation_active(duty[CH_TRANSLATE_LR],
                                         duty[CH_TRANSLATE_FB], stick_deadzone_percent);
}

void translate_field_oriented()
{
  spin();

  const controller::Translation vector = controller::translation(
      duty[CH_TRANSLATE_LR], duty[CH_TRANSLATE_FB], drive_heading_phase_deg,
      stick_deadzone_percent);
  pending_target_phase_deg = vector.target_phase_deg;
  pending_translation_magnitude = vector.magnitude;
  pending_translation_enabled = vector.enabled;
}

void translate()
{
  translate_field_oriented();
}
