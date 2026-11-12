DNSServer diagnostic_dns_server;
bool diagnostic_dns_active = false;
const IPAddress diagnostic_ap_ip(192, 168, 4, 1);
const IPAddress diagnostic_ap_gateway(192, 168, 4, 1);
const IPAddress diagnostic_ap_subnet(255, 255, 255, 0);

bool start_diagnostic_access_point()
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if(WiFi.softAPConfig(diagnostic_ap_ip, diagnostic_ap_gateway,
                       diagnostic_ap_subnet) == false)
  {
    DEBUG_LOG("WiFi: softAP static-IP configuration failed");
    return false;
  }

  for(int attempt = 1; attempt <= 3; attempt++)
  {
    if(WiFi.softAP(ssid, password, 6, 0, 4))
    {
      DEBUG_LOGF("WiFi: AP started on attempt %d/3", attempt);
      return true;
    }
    DEBUG_LOGF("WiFi: AP start attempt %d/3 failed", attempt);
    wifi_ap_start_failure_count++;
    WiFi.softAPdisconnect(true);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAPConfig(diagnostic_ap_ip, diagnostic_ap_gateway,
                      diagnostic_ap_subnet);
  }
  return false;
}

bool diagnostic_access_point_healthy()
{
  IPAddress current_ip = WiFi.softAPIP();
  return WiFi.getMode() == WIFI_AP && current_ip == diagnostic_ap_ip;
}

bool start_diagnostic_network_services(bool automatic_restart)
{
  wifi_ap_last_restart_attempt_ms = millis();
  if(automatic_restart) wifi_ap_restart_count++;
  if(start_diagnostic_access_point() == false)
  {
    wifi_ap_services_ready = false;
    DEBUG_LOGF("WiFi: AP unavailable; retry scheduled in %lu ms",
               WIFI_AP_RETRY_INTERVAL_MS);
    return false;
  }

  // Resolve every hostname to the controller. This makes mobile captive-portal
  // checks and browser reloads converge on the same local page even when the
  // client does not query vibe.melt first.
  diagnostic_dns_server.setTTL(0);
  diagnostic_dns_active = diagnostic_dns_server.start(
      DNS_DEFAULT_PORT, "*", WiFi.softAPIP());
  ArduinoOTA.begin();
  session_web_start();
  wifi_ap_services_ready = true;
  wifi_ap_last_health_check_ms = millis();
  DEBUG_LOGF("WiFi active: ssid=%s config=http://%s/ ip_fallback=http://%s/ dns=%s restart=%lu",
             ssid, config_hostname, WiFi.softAPIP().toString().c_str(),
             diagnostic_dns_active ? "ready" : "FAILED",
             (unsigned long)wifi_ap_restart_count);
  return true;
}

void restart_diagnostic_network_services()
{
  diagnostic_dns_server.stop();
  diagnostic_dns_active = false;
  session_web_stop();
  wifi_ap_services_ready = false;
  WiFi.softAPdisconnect(true);
  start_diagnostic_network_services(true);
}

void wifi_mode()
{
  if(wifi_mode_active)
  {
    return;
  }

  // Preserve the complete pre-diagnostic evidence before WiFi allocates heap
  // or any HTTP request can inspect it.
  session_logger_freeze();
  wifi_mode_active = true;
  wifi_rc_loss_since_ms = 0;
  wifi_ap_last_health_check_ms = 0;
  wifi_ap_last_restart_attempt_ms = 0;
  wifi_ap_restart_count = 0;
  wifi_ap_start_failure_count = 0;
  wifi_ap_services_ready = false;
  set_robot_state(ROBOT_WIFI);
  motorR = 0;
  motorL = 0;
  pending_translation_enabled = false;
  update_motors();
  suspend_loop_task_watchdog_for_wifi();

  start_diagnostic_network_services(false);
  LEDStatus = "wifi";
  updateLED();
}

void exit_wifi_mode()
{
  diagnostic_dns_server.stop();
  diagnostic_dns_active = false;
  session_web_stop();
  WiFi.softAPdisconnect(true);
  wifi_ap_services_ready = false;
  wifi_mode_active = false;
  wifi_entry_high_start_ms = 0;
  wifi_rc_loss_since_ms = 0;
  wifi_ap_last_health_check_ms = 0;
  wifi_ap_last_restart_attempt_ms = 0;
  session_logger_resume();
  DEBUG_LOGF("WiFi: AP/HTTP stopped (rc=%d switch=%d)",
             rc_status ? 1 : 0, duty[CH_WIFI_MODE]);
  set_robot_state(rc_status ? ROBOT_ARMED_IDLE : ROBOT_FAILSAFE);
  motorR = 0;
  motorL = 0;
  pending_translation_enabled = false;
  update_motors();
  resume_loop_task_watchdog_after_wifi();
}

void service_wifi_mode()
{
  if(wifi_mode_active == false)
  {
    return;
  }

  set_robot_state(ROBOT_WIFI);
  motorR = 0;
  motorL = 0;
  pending_translation_enabled = false;
  update_motors();
  unsigned long now_ms = millis();
  if(wifi_ap_services_ready)
  {
    ArduinoOTA.handle();
    if(diagnostic_dns_active) diagnostic_dns_server.processNextRequest();
    session_web_service();

    if((now_ms - wifi_ap_last_health_check_ms) >= WIFI_AP_HEALTH_INTERVAL_MS)
    {
      wifi_ap_last_health_check_ms = now_ms;
      if(diagnostic_access_point_healthy() == false)
      {
        DEBUG_LOG("WiFi: AP health check failed; restarting network services");
        restart_diagnostic_network_services();
      }
      else if(diagnostic_dns_active == false)
      {
        diagnostic_dns_active = diagnostic_dns_server.start(
            DNS_DEFAULT_PORT, "*", WiFi.softAPIP());
        DEBUG_LOGF("WiFi: DNS retry %s",
                   diagnostic_dns_active ? "succeeded" : "failed");
      }
    }
  }
  else if((now_ms - wifi_ap_last_restart_attempt_ms) >= WIFI_AP_RETRY_INTERVAL_MS)
  {
    start_diagnostic_network_services(true);
  }

  if(rc_status)
  {
    if(wifi_rc_loss_since_ms != 0)
    {
      DEBUG_LOGF("WiFi: receiver polling recovered after %lu ms; AP remained active",
                 millis() - wifi_rc_loss_since_ms);
    }
    wifi_rc_loss_since_ms = 0;
    if(duty[CH_WIFI_MODE] <= 50)
    {
      exit_wifi_mode();
    }
  }
  else
  {
    // WebServer writes are synchronous and can outlast the normal 250 ms RC
    // timeout when a browser reload abandons a response. Never couple that
    // temporary parser starvation to AP lifetime. Motors remain independently
    // hard-disabled by wifi_mode_active; a valid RC frame with CH6 low is the
    // only receiver-driven way to leave WiFi mode.
    if(wifi_rc_loss_since_ms == 0)
    {
      wifi_rc_loss_since_ms = millis();
      DEBUG_LOG("WiFi: receiver polling lost; keeping AP active and motors gated");
    }
  }
}
