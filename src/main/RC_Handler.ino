void set_safe_channels()
{
  for(int channel = 1; channel <= NUM_CHANNELS; channel++)
  {
    duty[channel] = 50;
  }
  duty[CH_SPIN_THROTTLE] = 0;
  duty[CH_WIFI_MODE] = 0;
  duty[CH_SPIN_DIR] = 0;
  duty[CH_CONFIG_MODE] = 0;
  duty[CH_CONFIG_SAVE] = 0;
}

bool rc_frame_values_valid(const int *frame)
{
  for(int channel = 1; channel <= NUM_CHANNELS; channel++)
  {
    if(frame[channel] < RC_VALID_MIN_US || frame[channel] > RC_VALID_MAX_US)
    {
      return false;
    }
  }
  return true;
}

void update_channels()
{
  reset_loop_task_watchdog();
  IBus.loop();

  if(IBus.cnt_rec != last_rec)
  {
    int frame[NUM_CHANNELS + 1];
    for(int channel = 1; channel <= NUM_CHANNELS; channel++)
    {
      frame[channel] = IBus.readChannel(channel - 1);
    }

    last_rec = IBus.cnt_rec;
    if(rc_frame_values_valid(frame))
    {
      bool rc_was_healthy = rc_status;
      for(int channel = 1; channel <= NUM_CHANNELS; channel++)
      {
        duty[channel] = constrain(map(frame[channel], 1000, 2000, 0, 100), 0, 100);
      }
      last_valid_rc_frame_ms = millis();
      rec_gap = 0;
      invalid_rc_frame_count = 0;
      rc_status = true;
      session_note_rc_status(true);
      if(rc_was_healthy == false)
      {
        DEBUG_LOG("RC: valid frame stream acquired");
      }
    }
    else
    {
      invalid_rc_frame_count++;
      total_invalid_rc_frame_count++;
      rc_status = false;
      session_note_rc_status(false);
      set_safe_channels();
      if(invalid_rc_frame_count == 1 || (invalid_rc_frame_count % 25) == 0)
      {
        DEBUG_LOGF("RC: invalid frame #%u values=[%d,%d,%d,%d,%d,%d,%d,%d]",
                   invalid_rc_frame_count, frame[1], frame[2], frame[3], frame[4],
                   frame[5], frame[6], frame[7], frame[8]);
      }
    }
  }

  rec_gap = millis() - last_valid_rc_frame_ms;
  if(last_valid_rc_frame_ms == 0 || rec_gap > RC_FRAME_TIMEOUT_MS)
  {
    if(rc_status)
    {
      DEBUG_LOGF("RC: frame timeout after %lu ms", rec_gap);
    }
    rc_status = false;
    session_note_rc_status(false);
    set_safe_channels();
  }
}
