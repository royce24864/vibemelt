spi_device_handle_t led_spi_device = nullptr;
int led_spi_current_data_pin = -1;

bool init_led_spi_output()
{
  if(led_spi_device != nullptr)
  {
    return true;
  }

  constexpr uint32_t LED_SPI_FREQUENCY = 2400000;
  constexpr size_t LED_BYTES_PER_PIXEL = 3;
  constexpr size_t SPI_BYTES_PER_LED_BYTE = 3;
  constexpr size_t MAX_LED_COUNT = NUMPIXELST > NUMPIXELSB ? NUMPIXELST : NUMPIXELSB;

  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = top_led_pin;
  bus_config.miso_io_num = -1;
  bus_config.sclk_io_num = led_spi_clock_pin;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz =
      MAX_LED_COUNT * LED_BYTES_PER_PIXEL * SPI_BYTES_PER_LED_BYTE;

  esp_err_t result =
      spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
  if(result != ESP_OK)
  {
    DEBUG_LOGF("LED SPI: persistent bus setup failed: %d", (int)result);
    return false;
  }

  spi_device_interface_config_t device_config = {};
  device_config.clock_speed_hz = LED_SPI_FREQUENCY;
  device_config.mode = 0;
  device_config.spics_io_num = -1;
  device_config.queue_size = 1;

  result = spi_bus_add_device(SPI2_HOST, &device_config, &led_spi_device);
  if(result != ESP_OK)
  {
    DEBUG_LOGF("LED SPI: persistent device setup failed: %d", (int)result);
    led_spi_device = nullptr;
    spi_bus_free(SPI2_HOST);
    return false;
  }

  led_spi_current_data_pin = top_led_pin;
  pinMode(bottom_led_pin, OUTPUT);
  digitalWrite(bottom_led_pin, LOW);
  DEBUG_LOG("LED SPI: persistent bus/device ready");
  return true;
}

void route_led_spi_to_pin(int data_pin)
{
  if(data_pin == led_spi_current_data_pin)
  {
    return;
  }

  if(led_spi_current_data_pin >= 0)
  {
    esp_rom_gpio_connect_out_signal(led_spi_current_data_pin,
                                    SIG_GPIO_OUT_IDX, false, false);
    pinMode(led_spi_current_data_pin, OUTPUT);
    digitalWrite(led_spi_current_data_pin, LOW);
  }

  esp_rom_gpio_pad_select_gpio(data_pin);
  esp_rom_gpio_connect_out_signal(data_pin,
                                  spi_periph_signal[SPI2_HOST].spid_out,
                                  false, false);
  led_spi_current_data_pin = data_pin;
}

void show_strip_spi(Adafruit_NeoPixel &strip, int data_pin)
{
  constexpr size_t LED_BYTES_PER_PIXEL = 3;
  constexpr size_t SPI_BYTES_PER_LED_BYTE = 3;
  constexpr size_t MAX_LED_COUNT = NUMPIXELST > NUMPIXELSB ? NUMPIXELST : NUMPIXELSB;
  // Eight LEDs encode to 72 bytes. Arduino's polling SPI writer divides that
  // at its 64-byte hardware FIFO boundary, which puts the split inside LED 8
  // (the first seven LEDs use 63 bytes). A higher-priority motor-task wakeup
  // during the refill can make the WS2812s see a reset-length low pulse and
  // leave only the final LED holding its previous color. Keep the full frame
  // in DMA-capable memory and submit it as one uninterrupted transaction.
  DMA_ATTR static uint8_t encoded[MAX_LED_COUNT * LED_BYTES_PER_PIXEL * SPI_BYTES_PER_LED_BYTE];

  const uint8_t *pixels = strip.getPixels();
  const size_t pixel_bytes = strip.numPixels() * LED_BYTES_PER_PIXEL;
  size_t encoded_index = 0;

  // At 2.4 MHz, 100 is a WS2812 zero and 110 is a WS2812 one.
  for(size_t byte_index = 0; byte_index < pixel_bytes; byte_index++)
  {
    uint32_t expanded = 0;
    for(uint8_t mask = 0x80; mask != 0; mask >>= 1)
    {
      expanded = (expanded << 3) | ((pixels[byte_index] & mask) ? 0x6 : 0x4);
    }
    encoded[encoded_index++] = (uint8_t)(expanded >> 16);
    encoded[encoded_index++] = (uint8_t)(expanded >> 8);
    encoded[encoded_index++] = (uint8_t)expanded;
  }

  if(led_spi_device == nullptr)
  {
    return;
  }

  route_led_spi_to_pin(data_pin);
  spi_transaction_t transaction = {};
  transaction.length = encoded_index * 8;
  transaction.tx_buffer = encoded;
  esp_err_t result = spi_device_transmit(led_spi_device, &transaction);

  if(result != ESP_OK)
  {
    DEBUG_LOGF("LED SPI DMA write failed on pin %d: %d", data_pin, (int)result);
  }

  // SPI mode 0 leaves MOSI low; this interval latches the WS2812 frame.
  delayMicroseconds(80);
}

void show_both_strips_spi()
{
  show_strip_spi(top_strip, top_led_pin);
  show_strip_spi(bottom_strip, bottom_led_pin);
}

void fill_top_heading_color()
{
  top_strip.fill(battery_warning
                   ? top_strip.Color(255, 100, 0)
                   : top_strip.Color(0, 255, 0),
                 0, NUMPIXELST);
}

void fill_bottom_heading_color()
{
  bottom_strip.fill(battery_warning
                      ? bottom_strip.Color(255, 100, 0)
                      : bottom_strip.Color(0, 255, 100),
                    0, NUMPIXELSB);
}

void render_phase_led_frame(PhaseLedFrame frame)
{
  switch(frame)
  {
    case PHASE_LED_FRAME_TOP_HEADING:
      fill_top_heading_color();
      bottom_strip.clear();
      break;

    case PHASE_LED_FRAME_BOTTOM_HEADING:
      top_strip.clear();
      fill_bottom_heading_color();
      break;

    case PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR:
      fill_top_heading_color();
      bottom_strip.fill(bottom_strip.Color(80, 0, 0), 0, NUMPIXELSB);
      break;

    case PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING:
      top_strip.fill(top_strip.Color(80, 0, 0), 0, NUMPIXELST);
      fill_bottom_heading_color();
      break;

    case PHASE_LED_FRAME_CONFIG_TOP_HEADING:
      top_strip.fill(top_strip.Color(255, 60, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, NUMPIXELSB);
      break;

    case PHASE_LED_FRAME_CONFIG_BOTTOM_HEADING:
      top_strip.fill(top_strip.Color(0, 180, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(255, 60, 0), 0, NUMPIXELSB);
      break;

    case PHASE_LED_FRAME_CONFIG_STABLE:
      top_strip.fill(top_strip.Color(0, 180, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, NUMPIXELSB);
      break;

    case PHASE_LED_FRAME_MOTOR_ON:
      top_strip.fill(top_strip.Color(80, 0, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(80, 0, 0), 0, NUMPIXELSB);
      break;

    case PHASE_LED_FRAME_OFF:
    default:
      top_strip.clear();
      bottom_strip.clear();
      break;
  }

  show_both_strips_spi();
}

bool phase_led_frame_contains_heading(PhaseLedFrame frame)
{
  return frame == PHASE_LED_FRAME_TOP_HEADING ||
         frame == PHASE_LED_FRAME_BOTTOM_HEADING ||
         frame == PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR ||
         frame == PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING ||
         frame == PHASE_LED_FRAME_CONFIG_TOP_HEADING ||
         frame == PHASE_LED_FRAME_CONFIG_BOTTOM_HEADING;
}

void phase_led_output_task(void *parameter)
{
  (void)parameter;
  uint32_t last_rendered_transition = UINT32_MAX;
  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(led_output_mutex == nullptr)
    {
      continue;
    }

    xSemaphoreTake(led_output_mutex, portMAX_DELAY);
    PhaseLedMode current_mode = phase_led_mode;
    PhaseLedFrame current_frame = phase_led_frame;
    uint32_t current_transition = phase_led_transition_count;
    if(current_mode != PHASE_LED_DISABLED &&
       current_frame != PHASE_LED_FRAME_INVALID &&
       current_transition != last_rendered_transition)
    {
      // Render only the newest phase state. Notifications that arrive while
      // SPI is busy coalesce, preventing obsolete frames from accumulating.
      render_phase_led_frame(current_frame);
      phase_led_render_count++;
      last_rendered_transition = current_transition;

      // At high RPM the logical heading window can close while the two-strip
      // SPI write is still in progress. Apply that trailing edge before
      // waiting for another notification so green cannot remain physically
      // latched into a much wider arc. This changes LEDs only--motor phase and
      // pulse timing remain in the priority-4 motor task.
      PhaseLedMode latest_mode = phase_led_mode;
      PhaseLedFrame latest_frame = phase_led_frame;
      uint32_t latest_transition = phase_led_transition_count;
      if(latest_mode == current_mode &&
         latest_transition != current_transition &&
         phase_led_frame_contains_heading(current_frame) &&
         !phase_led_frame_contains_heading(latest_frame) &&
         latest_frame != PHASE_LED_FRAME_INVALID)
      {
        render_phase_led_frame(latest_frame);
        phase_led_render_count++;
        last_rendered_transition = latest_transition;
      }
    }
    xSemaphoreGive(led_output_mutex);
    // Give any ready peer task an opportunity to run before checking for the
    // next coalesced phase transition.
    taskYIELD();
  }
}

void init_phase_led_output_task()
{
  if(!init_led_spi_output())
  {
    DEBUG_LOG("phase LED: persistent SPI initialization FAILED");
    return;
  }

  if(led_output_mutex == nullptr)
  {
    led_output_mutex = xSemaphoreCreateMutex();
  }

  if(led_output_mutex == nullptr)
  {
    DEBUG_LOG("phase LED: output mutex creation FAILED");
    return;
  }

  if(phase_led_task_handle == nullptr)
  {
    // Run above Arduino loopTask (priority 1) so a phase-window closing edge
    // cannot wait for the next equal-priority time slice and appear as a wide
    // arc at high RPM. The motor/translation scheduler remains dominant at
    // priority 4 and can preempt every LED transfer.
    BaseType_t task_result =
        xTaskCreate(phase_led_output_task, "phase-led", 3072, nullptr, 2,
                    &phase_led_task_handle);
    if(task_result != pdPASS)
    {
      phase_led_task_handle = nullptr;
      DEBUG_LOG("phase LED: task creation FAILED");
      return;
    }
  }
  DEBUG_LOG("phase LED: event-driven output task ready");
}

bool set_phase_led_mode(PhaseLedMode requested_mode)
{
  if(phase_led_task_handle == nullptr)
  {
    requested_mode = PHASE_LED_DISABLED;
  }

  if(phase_led_mode != requested_mode)
  {
    phase_led_mode = requested_mode;
    // Force the 2 kHz phase task to publish the first frame for a new mode.
    phase_led_frame = PHASE_LED_FRAME_INVALID;
    phase_led_transition_count++;
  }

  return requested_mode != PHASE_LED_DISABLED;
}

void publish_phase_led_frame(PhaseLedMode expected_mode,
                             PhaseLedFrame requested_frame)
{
  if(phase_led_task_handle == nullptr ||
     phase_led_mode != expected_mode ||
     expected_mode == PHASE_LED_DISABLED)
  {
    return;
  }

  if(phase_led_frame != requested_frame)
  {
    phase_led_frame = requested_frame;
    phase_led_transition_count++;
    xTaskNotifyGive(phase_led_task_handle);
  }
}

void updateLED() 
{
  bool output_locked = false;
  if(led_output_mutex != nullptr)
  {
    output_locked = xSemaphoreTake(led_output_mutex, portMAX_DELAY) == pdTRUE;
  }

  unsigned long currentMillis = millis();
  int graph = 0;
  String activeLEDStatus = LEDStatus;
  if(led_feedback_until_ms > currentMillis)
  {
    activeLEDStatus = led_feedback_status;
  }
  else if(led_feedback_status != "")
  {
    led_feedback_status = "";
    led_feedback_count = 0;
    led_feedback_index = -1;
  }

  const uint32_t ringColors[] = {
  top_strip.Color(100, 0, 0),   // red
  top_strip.Color(100, 80, 0),  // yellow
  top_strip.Color(0, 100, 0),   // green
  top_strip.Color(0, 80, 100),  // cyan
  top_strip.Color(80, 0, 100)   // purple
  };

  const int NUM_COLORS = sizeof(ringColors) / sizeof(ringColors[0]);

  // LED logic depending on status
    if (activeLEDStatus == "config zeroing")
    {
      float phase = (currentMillis % 400) / 400.0 * 2 * PI;
      int brightness = (sin(phase) * 70) + 90;
      top_strip.fill(top_strip.Color(0, 0, brightness), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 0, brightness), 0, NUMPIXELST);
    }
    else if (activeLEDStatus == "config zeroed")
    {
      top_strip.fill(top_strip.Color(0, 180, 80), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 180, 80), 0, NUMPIXELST);
    }
    else if (activeLEDStatus == "config rpm mode")
    {
      top_strip.fill(top_strip.Color(100, 80, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(100, 80, 0), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config heading mode")
    {
      top_strip.fill(top_strip.Color(0, 100, 140), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 100, 140), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config rpm stable")
    {
      top_strip.fill(top_strip.Color(0, 180, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config heading centered")
    {
      top_strip.fill(top_strip.Color(255, 60, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(255, 60, 0), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config point saved")
    {
      int savedCount = constrain(led_feedback_count, 0, NUMPIXELST);
      top_strip.clear();
      bottom_strip.clear();
      top_strip.fill(top_strip.Color(0, 180, 0), 0, savedCount);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, savedCount);
      if(savedCount < NUMPIXELST)
      {
        top_strip.setPixelColor(savedCount, top_strip.Color(100, 100, 100));
        bottom_strip.setPixelColor(savedCount, bottom_strip.Color(100, 100, 100));
      }
    }
    else if (activeLEDStatus == "config point replaced")
    {
      top_strip.fill(top_strip.Color(0, 180, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, NUMPIXELSB);
      if(led_feedback_index >= 0 && led_feedback_index < NUMPIXELST)
      {
        top_strip.setPixelColor(led_feedback_index,
                                top_strip.Color(255, 255, 255));
      }
      if(led_feedback_index >= 0 && led_feedback_index < NUMPIXELSB)
      {
        bottom_strip.setPixelColor(led_feedback_index,
                                   bottom_strip.Color(255, 255, 255));
      }
    }
    else if (activeLEDStatus == "config rpm cleared")
    {
      top_strip.fill(top_strip.Color(100, 0, 100), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(100, 0, 100), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config unstable")
    {
      float phase = (currentMillis % 300) / 300.0 * 2 * PI;
      int brightness = (sin(phase) * 60) + 100;
      top_strip.fill(top_strip.Color(brightness, 25, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(brightness, 25, 0), 0, NUMPIXELSB);
    }
    else if (activeLEDStatus == "config final saved")
    {
      int savedCount = constrain(led_feedback_count, 0, NUMPIXELST);
      top_strip.fill(top_strip.Color(100, 80, 0), 0, NUMPIXELST);
      bottom_strip.fill(bottom_strip.Color(100, 80, 0), 0, NUMPIXELST);
      top_strip.fill(top_strip.Color(0, 180, 0), 0, savedCount);
      bottom_strip.fill(bottom_strip.Color(0, 180, 0), 0, savedCount);
      if(savedCount < NUMPIXELST)
      {
        top_strip.setPixelColor(savedCount, top_strip.Color(100, 100, 100));
        bottom_strip.setPixelColor(savedCount, bottom_strip.Color(100, 100, 100));
      }
    }
    else if (activeLEDStatus == "failsafe") 
    {
      // advance animation index only on interval
    if (currentMillis - lastUpdate > animSpeed) 
    {
      animIndex += direction;

      // bounce logic for chase animation
      if (animIndex >= NUMPIXELST) {
        animIndex = NUMPIXELST;
        direction = -1;
      } else if (animIndex <= 0) {
        animIndex = 0;
        direction = 1;
      }

      lastUpdate = currentMillis;
    }
    if (rc_status == false) 
    {
      // yellow bounce chase
      top_strip.clear();
      bottom_strip.clear();
      top_strip.setPixelColor(animIndex, top_strip.Color(100, 80, 0));
      bottom_strip.setPixelColor(animIndex, bottom_strip.Color(100, 80, 0));
    }
    else
    {
    // breathing blue effect
    // use millis() directly for smoother breathing
    float phase = (millis() % 2000) / 2000.0 * 2 * PI; // 0 to 2π every 2 sec
    int brightness = (sin(phase) * 50) + 50; // sine wave 0–255

    uint32_t color = top_strip.Color(0, 0, brightness);
    top_strip.fill(color, 0, NUMPIXELST);
    bottom_strip.fill(color, 0, NUMPIXELST);
    }
  }
  else if (activeLEDStatus == "battery halt")
  {
    bool on = ((currentMillis / 150) % 2) == 0;
    top_strip.fill(on ? top_strip.Color(180, 0, 0) : 0, 0, NUMPIXELST);
    bottom_strip.fill(on ? bottom_strip.Color(180, 0, 0) : 0, 0, NUMPIXELSB);
  }
  else if (activeLEDStatus == "sensor fault")
  {
    bool on = ((currentMillis / 250) % 2) == 0;
    top_strip.fill(on ? top_strip.Color(160, 0, 160) : 0, 0, NUMPIXELST);
    bottom_strip.fill(on ? bottom_strip.Color(160, 0, 160) : 0, 0, NUMPIXELSB);
  }
  else if (activeLEDStatus == "wifi")
  {
    top_strip.fill(top_strip.Color(0, 0, 80), 0, NUMPIXELST);
    bottom_strip.fill(bottom_strip.Color(0, 0, 80), 0, NUMPIXELSB);
  }
  else if (activeLEDStatus == "export")
  {
    top_strip.fill(top_strip.Color(100, 100, 100), 0, NUMPIXELST); 
    bottom_strip.fill(bottom_strip.Color(100, 100, 100), 0, NUMPIXELST);
  }
  else if (activeLEDStatus == "armed")
  {
    if((millis() - stripstart) > 2000)
    {
    stripstart = millis();
    top_strip.clear();
    bottom_strip.clear();
    int battery_cells = detected_battery_cells > 0
                          ? detected_battery_cells
                          : (BATTERY_CELL_COUNT > 0 ? BATTERY_CELL_COUNT : 3);
    float bar_empty_volts = battery_cells * BATTERY_WARN_VOLTS_PER_CELL;
    float bar_full_volts = battery_cells == 3
                             ? BATTERY_BAR_3S_FULL_VOLTS
                             : battery_cells * BATTERY_BAR_STANDARD_FULL_VOLTS_PER_CELL;
    float bar_fill = (volts - bar_empty_volts) /
                     (bar_full_volts - bar_empty_volts);
    graph = constrain((int)round(1.0f +
                      constrain(bar_fill, 0.0f, 1.0f) * (NUMPIXELST - 1)),
                      1, NUMPIXELST);
    if(rvs_led_B == true)
    {
      bottom_strip.fill(bottom_strip.Color(100, 15, 0), NUMPIXELST - graph, NUMPIXELST);
    }
    else
    {
      bottom_strip.fill(bottom_strip.Color(100, 15, 0), 0, graph);
    }
    if(rvs_led_T == true)
    {
      top_strip.fill(top_strip.Color(100, 0, 0), NUMPIXELST - graph, NUMPIXELST); 
    }
    else
    {
      top_strip.fill(top_strip.Color(100, 0, 0), 0, graph);
    }
    }
  }
  else if (activeLEDStatus == "heading on") 
  {
    if (battery_warning) {
      top_strip.fill(top_strip.Color(255, 100, 0), 0, 10); 
      bottom_strip.fill(bottom_strip.Color(255, 100, 0), 0, 10);
    } else {
      top_strip.fill(top_strip.Color(0, 255, 0), 0, 10);
      bottom_strip.fill(bottom_strip.Color(0, 255, 100), 0, 10);
    }
  }
  else if (activeLEDStatus == "motor on") 
  {
    top_strip.fill(top_strip.Color(80, 0, 0), 0, 10);
    bottom_strip.fill(bottom_strip.Color(80, 0, 0), 0, 10);
  }
  else if (activeLEDStatus == "celebrate")
  {
    if (currentMillis - lastUpdate > 10)
    {
      ringRadius++;

      if (ringRadius >= NUMPIXELST)
      {
        ringRadius = 0;
        ringColorIndex = (ringColorIndex + 1) % NUM_COLORS;
      }

      lastUpdate = currentMillis;
    }

    top_strip.clear();
    bottom_strip.clear();

    uint32_t c = ringColors[ringColorIndex];

    top_strip.setPixelColor(ringRadius, c);
    bottom_strip.setPixelColor(ringRadius, c);
  }
  
  else if (activeLEDStatus == "boot") //white LED to indicating restart
  {
    top_strip.fill(top_strip.Color(50, 50, 50), 0, 10);
    bottom_strip.fill(bottom_strip.Color(50, 50, 50), 0, 10);
  }
  else 
  {
    top_strip.clear();
    bottom_strip.clear();
  }

  show_both_strips_spi();

  if(output_locked)
  {
    xSemaphoreGive(led_output_mutex);
  }
}
