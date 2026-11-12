#include <esp_task_wdt.h>
#include <Wire.h>
#include <SparkFun_LIS331.h>
#include <Adafruit_NeoPixel.h>
#include <driver/spi_master.h>
#include <esp_rom_gpio.h>
#include <soc/spi_periph.h>
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include "DShotRMT.h"
#include <Preferences.h>
#include "Session_Logger.h"
#include "Config_Store.h"
#include "Phase_Led_State.h"
#include "Controller.h"
#include "Estimator.h"
#include "Calibration.h"
#include "Motor_Timing.h"

// Compile with -DENABLE_SERIAL_DEBUG to enable
// transition/event logging at 115200 baud. Serial is never used by the 2 kHz
// motor-output task.

void session_log_event(const char *message);
void session_log_eventf(const char *format, ...);
void session_logger_begin();
void session_logger_delay(unsigned long delay_ms);
void session_logger_mark_setup_complete();
void service_session_logger();
void session_on_state_transition(int previous_state, int next_state);
void session_note_sensor_health(bool healthy);
void session_note_rc_status(bool healthy);
void session_web_register_routes();
void reset_loop_task_watchdog();
void suspend_loop_task_watchdog_for_wifi();
void resume_loop_task_watchdog_after_wifi();


//#define ENABLE_SERIAL_DEBUG


#ifdef ENABLE_SERIAL_DEBUG
  #define DEBUG_SERIAL_BEGIN() Serial.begin(115200)
  #define DEBUG_LOG(message) session_log_event(message)
  #define DEBUG_LOGF(format, ...) session_log_eventf(format, ##__VA_ARGS__)
#else
  #define DEBUG_SERIAL_BEGIN() do {} while(0)
  #define DEBUG_LOG(message) session_log_event(message)
  #define DEBUG_LOGF(format, ...) session_log_eventf(format, ##__VA_ARGS__)
#endif

//------accelerometer config------------
#define ACCEL_RANGE LIS331::HIGH_RANGE  //sets to 400g range
#define ACCEL_MAX_SCALE 400
#define ACCEL_I2C_ADDRESS 0x18 // Adafruit H3LIS331 default address with SDO open/GND
LIS331 xl; //accelerometer object
// Accelerometer die position relative to the physical rotation center.  The
// sensor X/Y axes are mounted in the same orientation as the robot X/Y axes.
const float ACCEL_X_OFFSET_MM = 8.55f;
const float ACCEL_Y_OFFSET_MM = 12.30f;
// Derive the radius so changing either mounting offset also updates RPM scaling.
const float ACCEL_RADIUS_MM = sqrtf(ACCEL_X_OFFSET_MM * ACCEL_X_OFFSET_MM +
                                    ACCEL_Y_OFFSET_MM * ACCEL_Y_OFFSET_MM);
const float ACCEL_RADIAL_UNIT_X = ACCEL_X_OFFSET_MM / ACCEL_RADIUS_MM;
const float ACCEL_RADIAL_UNIT_Y = ACCEL_Y_OFFSET_MM / ACCEL_RADIUS_MM;

//-------ESC config--------
#define escR_gpio GPIO_NUM_21 // XIAO ESP32-C3 D6/TX/GPIO21, right ESC signal
#define escL_gpio GPIO_NUM_10 // XIAO ESP32-C3 D10/MOSI/GPIO10, left ESC signal
DShotRMT escR(escR_gpio, DSHOT300, false);
DShotRMT escL(escL_gpio, DSHOT300, false);
// Software equivalents of swapping any two phase wires on each motor.
const bool reverse_right_motor = true;
const bool reverse_left_motor = true;
//library uses values between -999 and 999 to control speed of esc

//---------Other pin assignemnts----------
const int volt_pin = A0; // XIAO ESP32-C3 D0/A0/GPIO2, battery voltage divider

//------LED config--------
const int top_led_pin = D3;  // XIAO ESP32-C3 D3/A3/GPIO5, top LED strip
const int bottom_led_pin = D2;  // XIAO ESP32-C3 D2/A2/GPIO4, bottom LED strip
const int led_spi_clock_pin = D8;  // SPI clock is required internally; leave D8 unconnected
const int NUMPIXELST = 8; //number of leds in the top strip
const int NUMPIXELSB = 8; //number of leds in the bottom strip
const bool rvs_led_T = false;
const bool rvs_led_B = false;
const float HEADING_LED_HALF_WIDTH_DEG = 10.0f;
Adafruit_NeoPixel top_strip(NUMPIXELST, top_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bottom_strip(NUMPIXELSB, bottom_led_pin, NEO_GRB + NEO_KHZ800);
const int animSpeed = 100;   // ms between LED updates when in animation mode

//------Reciever config-----------
#define RXpin Serial1  // XIAO ESP32-C3 receiver iBus signal on D7/RX/GPIO20
IBusBM IBus;    // IBus object
const int NUM_CHANNELS = 8; //number of reciever channels to use

//------Field-oriented control map------
const int CH_TRANSLATE_LR = 1;     // right stick X: field left/right translation
const int CH_TRANSLATE_FB = 2;     // right stick Y: field forward/back translation
const int CH_SPIN_THROTTLE = 3;    // spin throttle
const int CH_HEADING_YAW = 4;      // left/right stick: rotate the runtime drive-heading frame
const int CH_SPIN_DIR = 5;         // spin direction switch
const int CH_WIFI_MODE = 6;        // WiFi/OTA mode switch
const int CH_CONFIG_MODE = 7;      // config/calibration mode switch
const int CH_CONFIG_SAVE = 8;      // capture RPM point; triple press advances to heading calibration

//------Driving characteristcs------
constexpr float DEFAULT_ACC_RATE = 0.40f;
constexpr float MIN_ACC_RATE = 0.20f;
constexpr float MAX_ACC_RATE = 1.00f;
constexpr float DEFAULT_PERCENT_DECEL = 15.0f;
constexpr float MIN_PERCENT_DECEL = 1.0f;
constexpr float MAX_PERCENT_DECEL = 50.0f;
constexpr float DEFAULT_TRANSLATION_SCALE_FACTOR = 1.0f;
constexpr float MIN_TRANSLATION_SCALE_FACTOR = 0.0f;
constexpr float MAX_TRANSLATION_SCALE_FACTOR = 2.0f;
float acc_rate = DEFAULT_ACC_RATE; //RPM-based spin acceleration ceiling slope
const float heading_yaw_rate_dps = 180.0; //max manual LED heading rotation speed, degrees/second
const float config_rpm_offset_rate = 0.04; //rotation-period correction factor change per second at full CH1 stick
const float config_led_offset_rate_dps = 90.0; //LED heading offset change per second at full yaw stick
const unsigned long config_stage_advance_window_ms = 3000;
const int config_stage_advance_press_count = 3;
const int config_clear_press_count = 5;
const int stick_deadzone_percent = 8;
const int throttle_low_percent = 5; //throttle stick low is ~-100, which maps to duty near 0
float percentdecel = DEFAULT_PERCENT_DECEL; //live angular width of each translation pulse
float translation_scale_factor = DEFAULT_TRANSLATION_SCALE_FACTOR; //0=no pulse change, 1=full change
const unsigned long mode_switch_hold_ms = 1000; //CH6/CH7 must be high this long before entering special modes
const unsigned long WIFI_AP_HEALTH_INTERVAL_MS = 1000;
const unsigned long WIFI_AP_RETRY_INTERVAL_MS = 2000;
const int wifi_entry_max_rpm =400;

//------Safety and estimator config------
const unsigned long RC_FRAME_TIMEOUT_MS = 250;
const int RC_VALID_MIN_US = 900;
const int RC_VALID_MAX_US = 2100;
const unsigned long ACCEL_SAMPLE_INTERVAL_US = 2500; //400 Hz sensor estimator
// These coefficients preserve approximately the same time constants used by
// the former 200 Hz estimator after doubling the sample rate.
const float ACCEL_FILTER_ALPHA = 0.4084f;
const float RPM_VARIATION_ALPHA = 0.1056f;
const unsigned long ACCEL_HEALTH_CHECK_MS = 500;
const int ACCEL_STARTUP_ATTEMPTS = 5;
const int ACCEL_READ_FAILURE_LIMIT = 5;
const unsigned long RPM_STABLE_HOLD_MS = 750;
const int BATTERY_CELL_COUNT = 3; //0 auto-detects 3S/4S; set explicitly when the pack is fixed
const float BATTERY_WARN_VOLTS_PER_CELL = 3.60;
const float BATTERY_HALT_VOLTS_PER_CELL = 3.20;
const float BATTERY_BAR_3S_FULL_VOLTS = 13.05; //fully charged 3S LiHV pack
const float BATTERY_BAR_STANDARD_FULL_VOLTS_PER_CELL = 4.20;
const float BATTERY_UNPLUGGED_THRESHOLD = 5.5;
const int BATTERY_LOW_SAMPLE_COUNT = 10;
const uint64_t MOTOR_TASK_PERIOD_US = 500; //2 kHz phase/motor scheduler

//-------Wifi config---------
const char *ssid = "vibemelt.ai"; //wifi ssid
const char *password = "vibemelt"; //wifi password
const char *config_hostname = "vibemelt.ai"; //local DNS name while connected to the AP

//-----other constants-------
const float rpm_g_per_rpm2_per_mm = 0.000001118; //g-force per rpm^2 per mm of accelerometer radius
const float rpm_per_sqrt_g =
    1.0f / sqrt(rpm_g_per_rpm2_per_mm * ACCEL_RADIUS_MM);
const int WDT_TIMEOUT = 1;  // seconds
const int MAX_CORRECTION_POINTS = 8;
const char *NVS_NAMESPACE = "vibemelt";

// Keep custom types visible to Arduino's generated cross-tab declarations.
ConfigExitReason config_abort_reason();
const char *config_exit_reason_name(ConfigExitReason reason);
void exit_config_mode(ConfigExitReason reason);
AccelZeroReason accel_zero_cancel_reason();
AccelZeroResult set_accel_zero_offset();
bool init_motor_output_task();
bool motor_output_fault_latched();
void request_motor_output_rearm();
controller::Controls current_controls();
controller::Health current_control_health();
calibration::Safety current_calibration_safety();
void latch_control_rearm();
void update_control_rearm();

struct MotorControlSnapshot
{
  int64_t published_at_us;
  int64_t accel_sample_us;
  RobotState state;
  int motor_l;
  int motor_r;
  bool output_enabled;
  bool translation_enabled;
  float translation_magnitude;
  float target_phase_deg;
  float heading_phase_deg;
  float translation_led_phase_deg;
  float estimated_rpm;
  float rotation_offset;
  bool reversed_spin;
  PhaseLedMode phase_led_mode;
};

//=============GLOBAL VARIABLES==================
//-------LED control-------
String LEDStatus = "armed"; //tells LED control what mode to be in
String led_feedback_status = "";
unsigned long led_feedback_until_ms = 0;
int led_feedback_count = 0;
int led_feedback_index = -1;
unsigned long lastUpdate = 0;   // last LED animation update time
unsigned long stripstart = 0;   //time of last update
int animIndex = 0;     // position in LED animation
int direction = 1; // 1 = forward, -1 = backward
int ringRadius = 0;
int ringColorIndex = 0;
TaskHandle_t phase_led_task_handle = nullptr;
SemaphoreHandle_t led_output_mutex = nullptr;
volatile uint32_t phase_led_render_count = 0;
//-------rpm and heading calc--------
int rpm;
float offset = 1.0; //creates variable to tune a stable heading, will update from NVS in setup
volatile int angle = 0; //phase accumulator maintained by the fixed-rate motor task
// Keep the commanded drive frame separate from the physical LED phase offset.
// CH4 rotates the drive frame at runtime; heading calibration changes only the
// LED offset so the marker can be aligned to the direction the bot actually moves.
float drive_heading_phase_deg = 0.0f;
float led_heading_offset_deg = 315.0f; //default LED-only offset; replaced by a valid saved value at boot
unsigned long last_heading_update_ms = 0;
unsigned long last_config_adjust_ms = 0;
float accel_zero_x_g_offset = 0.0;
float accel_zero_y_g_offset = 0.0;
volatile float rotation_phase_deg = 0.0;
volatile bool accel_sensor_healthy = false;
unsigned long last_accel_health_check_ms = 0;
unsigned long last_accel_sample_ms = 0;
int64_t last_accel_sample_us = 0;
estimator::State accel_estimator;
const estimator::Config accel_estimator_config = {
  ACCEL_RADIAL_UNIT_X, ACCEL_RADIAL_UNIT_Y, rpm_per_sqrt_g,
  ACCEL_FILTER_ALPHA, RPM_VARIATION_ALPHA, RPM_STABLE_HOLD_MS
};
int accel_health_failure_count = 0;
int accel_read_failure_count = 0;
unsigned long accel_loss_started_ms = 0;
uint32_t accel_fault_generation = 0;
unsigned long rpm_stable_since_ms = 0;
float rpm_variation_ewma = 0.0;
float current_gforce = 0.0f;
bool current_gforce_valid = false;
uint32_t accel_sample_sequence = 0;
bool reset_accel_filter_requested = false;
volatile bool reset_rotation_phase_requested = false;
int correction_point_count = 0;
float correction_rpm[MAX_CORRECTION_POINTS];
float correction_offset[MAX_CORRECTION_POINTS];
int correction_throttle[MAX_CORRECTION_POINTS];
int max_rpm = 0;  // stores the greatest rpm achieved
int max_gforce = 0; //for storing max gforce achieved
//--------translation----------
bool reversed = false;  //used for reverse spin direction
int64_t loopstart = 0; //stores last gforce read time in microseconds
int motorL;
int motorR;
int spinspeed;
//--------reciever----------
int duty[NUM_CHANNELS + 1]; //1-indexed list values from 0 - 100
int last_rec; //used to store update times for detecting signal loss
unsigned long rec_gap;
volatile bool rc_status = false; //whether getting rc signal, used for triggering failsafe
volatile unsigned long last_valid_rc_frame_ms = 0;
unsigned int invalid_rc_frame_count = 0;
uint32_t total_invalid_rc_frame_count = 0;
//----------other----------
float volts; //variable to store current battery voltage
unsigned long batloop; //variable for tracking battery update loop
bool battery_warning = false;
volatile bool battery_critical_latched = false;
int detected_battery_cells = 0;
int battery_detection_samples = 0;
int battery_warn_count = 0;
int battery_critical_count = 0;
int64_t loop_time;
int64_t loop_start;
volatile bool motor_on = false;
volatile bool dshot_outputs_ready = false;
volatile bool config_lifecycle_output_inhibited = false;
controller::RearmState control_rearm_state;
volatile bool control_rearm_required = true;
calibration::Session calibration_session;
bool &config_mode = calibration_session.active;
ConfigCalibrationStage &config_calibration_stage = calibration_session.stage;
volatile bool &config_reentry_wait_for_low = calibration_session.wait_for_low;
unsigned long wifi_entry_high_start_ms = 0;
unsigned long wifi_rc_loss_since_ms = 0;
unsigned long wifi_ap_last_health_check_ms = 0;
unsigned long wifi_ap_last_restart_attempt_ms = 0;
uint32_t wifi_ap_restart_count = 0;
uint32_t wifi_ap_start_failure_count = 0;
bool wifi_ap_services_ready = false;
volatile bool wifi_mode_active = false;
RobotState robot_state = ROBOT_FAILSAFE;
RobotState last_operating_state = ROBOT_FAILSAFE;
bool pending_translation_enabled = false;
float pending_translation_magnitude = 0.0f;
float pending_target_phase_deg = 0.0f;
bool loop_task_watchdog_registered = false;

const char *robot_state_name(int state)
{
  switch(state)
  {
    case ROBOT_ARMED_IDLE: return "ARMED_IDLE";
    case ROBOT_TANK: return "TANK";
    case ROBOT_SPINUP: return "SPINUP";
    case ROBOT_MELTY: return "MELTY";
    case ROBOT_CONFIG: return "CONFIG";
    case ROBOT_WIFI: return "WIFI";
    case ROBOT_FAILSAFE: return "FAILSAFE";
    case ROBOT_LOW_BATTERY: return "LOW_BATTERY";
    case ROBOT_SENSOR_FAULT: return "SENSOR_FAULT";
    default: return "UNKNOWN";
  }
}

void set_robot_state(int next_state_value)
{
  RobotState next_state = (RobotState)next_state_value;
  if(robot_state != next_state)
  {
    if(next_state == ROBOT_WIFI && robot_state != ROBOT_WIFI)
    {
      last_operating_state = robot_state;
    }
    session_on_state_transition((int)robot_state, (int)next_state);
    DEBUG_LOGF("state %s -> %s (rpm=%d throttle=%d rc=%d)",
               robot_state_name(robot_state), robot_state_name(next_state),
               rpm, duty[CH_SPIN_THROTTLE], rc_status ? 1 : 0);
    robot_state = next_state;
  }
}

void reset_loop_task_watchdog()
{
  if(loop_task_watchdog_registered)
  {
    esp_task_wdt_reset();
  }
}

void suspend_loop_task_watchdog_for_wifi()
{
  if(!loop_task_watchdog_registered) return;
  esp_err_t status = esp_task_wdt_delete(NULL);
  if(status == ESP_OK)
  {
    loop_task_watchdog_registered = false;
    DEBUG_LOG("watchdog: loopTask monitoring suspended for WiFi/HTTP");
  }
  else
  {
    DEBUG_LOGF("watchdog: failed to suspend loopTask monitoring: %d", (int)status);
  }
}

void resume_loop_task_watchdog_after_wifi()
{
  if(loop_task_watchdog_registered) return;
  esp_err_t status = esp_task_wdt_add(NULL);
  if(status == ESP_OK)
  {
    loop_task_watchdog_registered = true;
    DEBUG_LOG("watchdog: loopTask monitoring resumed after WiFi");
  }
  else
  {
    DEBUG_LOGF("watchdog: failed to resume loopTask monitoring: %d", (int)status);
  }
}

//--------------------------------------------------------------------------------------------
void setup() 
{
  session_logger_begin();
  DEBUG_SERIAL_BEGIN();
  DEBUG_LOG("boot: setup started");
  session_web_register_routes();
  if(!initialize_config_store())
  {
    DEBUG_LOG("config: persistent store unavailable; saves will report failure");
  }
  load_config_settings();
  DEBUG_LOGF("config loaded: led_offset=%.1f drive_heading=%.1f rpm_offset=%.4f correction_points=%d",
             led_heading_offset_deg, drive_heading_phase_deg, offset,
             correction_point_count);
  DEBUG_LOGF("drive config loaded: translation_scale=%.2f pulse_width=%.1f%% acc_rate=%.2f",
             translation_scale_factor, percentdecel, acc_rate);
#ifdef ENABLE_SERIAL_DEBUG
  session_logger_delay(1500); // Allow the serial monitor to reconnect before table output.
#endif
  if(correction_point_count == 0)
  {
    DEBUG_LOG("config table: empty");
  }
  else
  {
    for(int i = 0; i < correction_point_count; i++)
    {
      DEBUG_LOGF("config table[%d]: rpm=%.0f offset=%.4f throttle=%d",
                 i, correction_rpm[i], correction_offset[i], correction_throttle[i]);
    }
  }
  Wire.begin(D4, D5);  // XIAO ESP32-C3 D4/SDA/GPIO6 and D5/SCL/GPIO7
  Wire.setClock(400000);  //makes reading the sensor faster somehow
  bool accel_startup_ok = initialize_accel_sensor(ACCEL_STARTUP_ATTEMPTS);
  if(accel_startup_ok)
  {
    accel_sensor_healthy = true;
    session_note_sensor_health(true);
  }
  else
  {
    record_accel_loss("startup initialization failed after five attempts");
  }
  DEBUG_LOGF("accelerometer startup: address=0x%02X healthy=%d attempts=%d",
             ACCEL_I2C_ADDRESS, accel_sensor_healthy ? 1 : 0,
             ACCEL_STARTUP_ATTEMPTS);
  DEBUG_LOGF("accelerometer geometry: x=%.2f mm y=%.2f mm r=%.3f mm rpm/sqrt(g)=%.3f",
             ACCEL_X_OFFSET_MM, ACCEL_Y_OFFSET_MM, ACCEL_RADIUS_MM,
             rpm_per_sqrt_g);
  IBus.begin(RXpin, IBUSBM_NOTIMER, D7, -1);    // iBUS RX on D7; leave D6 free for ESC signal
  top_strip.begin(); //initializes LEDs
  bottom_strip.begin();
  top_strip.clear();  // Turn all LEDs off ASAP
  bottom_strip.clear();
  init_phase_led_output_task();
  LEDStatus = "boot";
  updateLED();
  ArduinoOTA.setHostname("esp32-ap"); //wifi ota config
  ArduinoOTA.setPassword(nullptr); //no password required to upload code
  dshot_result_t escR_status = escR.begin();
  dshot_result_t escL_status = escL.begin();
  DEBUG_LOGF("DShot setup: DShotRMT 0.9.5 protocol=300 right=%d left=%d",
             escR_status.success ? 0 : (int)escR_status.result_code,
             escL_status.success ? 0 : (int)escL_status.result_code);
  // Arm AM32 with a continuous zero-throttle stream before changing settings.
  bool esc_startup_ready = false;
  if(escR_status.success && escL_status.success)
  {
    session_logger_delay(250);
    bool startup_ok = true;
    for(int frame = 0; frame < 1500; frame++)
    {
      dshot_result_t right_status = escR.sendThrottle(0);
      dshot_result_t left_status = escL.sendThrottle(0);
      if(right_status.tx_outcome != DSHOT_TX_QUEUED ||
         left_status.tx_outcome != DSHOT_TX_QUEUED) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    // AM32 requires each setting command at least six times. Only accepted
    // transmissions count toward successful startup.
    DEBUG_LOG("DShot startup: enabling AM32 3D mode");
    for(int frame = 0; frame < 10; frame++)
    {
      dshot_result_t right_3d_status =
          escR.sendCommand(DSHOT_CMD_3D_MODE_ON, 1, 0);
      dshot_result_t left_3d_status =
          escL.sendCommand(DSHOT_CMD_3D_MODE_ON, 1, 0);
      if(right_3d_status.tx_outcome != DSHOT_TX_QUEUED ||
         left_3d_status.tx_outcome != DSHOT_TX_QUEUED) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    for(int frame = 0; frame < 100; frame++)
    {
      dshot_result_t right_status = escR.sendThrottle(0);
      dshot_result_t left_status = escL.sendThrottle(0);
      if(right_status.tx_outcome != DSHOT_TX_QUEUED ||
         left_status.tx_outcome != DSHOT_TX_QUEUED) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    DEBUG_LOGF("DShot startup: zero arm and AM32 3D setup complete status=%d",
               startup_ok ? 0 : 1);
    if(startup_ok)
    {
      esc_startup_ready = true;
    }
    else
    {
      DEBUG_LOG("DShot startup failed; motor scheduler disabled");
    }
  }
  else
  {
    DEBUG_LOG("DShot channel allocation failed; motor scheduler disabled");
  }

  esp_task_wdt_config_t wdt_config = {};
  wdt_config.timeout_ms = WDT_TIMEOUT * 1000U;
  wdt_config.idle_core_mask = 0;
  wdt_config.trigger_panic = true;

  esp_err_t loop_wdt_status = esp_task_wdt_status(NULL);
  loop_task_watchdog_registered = loop_wdt_status == ESP_OK;
  esp_err_t wdt_status;
  if(loop_wdt_status == ESP_ERR_INVALID_STATE)
  {
    wdt_status = esp_task_wdt_init(&wdt_config);
  }
  else
  {
    wdt_status = esp_task_wdt_reconfigure(&wdt_config);
  }
  if(wdt_status == ESP_OK)
  {
    if(loop_wdt_status != ESP_OK)
    {
      esp_err_t add_status = esp_task_wdt_add(NULL);
      if(add_status == ESP_OK)
      {
        loop_task_watchdog_registered = true;
      }
      else
      {
        DEBUG_LOGF("watchdog loopTask subscribe failed: %d", (int)add_status);
      }
    }
  }
  else
  {
    DEBUG_LOGF("watchdog setup failed: %d", (int)wdt_status);
  }
  // The output task subscribes to the watchdog before accepting commands.
  if(esc_startup_ready && wdt_status == ESP_OK && loop_task_watchdog_registered)
    dshot_outputs_ready = init_motor_output_task();
  if(!dshot_outputs_ready) DEBUG_LOG("motor scheduler unavailable; outputs remain inhibited");
  DEBUG_LOG("setup complete; waiting for healthy sensors and neutral RC controls");
  session_logger_mark_setup_complete();
  
}

//----------------------------------------------------------------------------------------
//functions

void get_battery()  //calculates battery voltage based on voltage divider input on pin A0
{
  if ((millis() - batloop) >= 100)
  {
    bool warning_before = battery_warning;
    bool critical_before = battery_critical_latched;
    int cells_before = detected_battery_cells;
    static bool filter_initialized = false;
    float measured_volts = (float(analogRead(volt_pin)) / 310.0f) * 1.29f;
    if(measured_volts >= 0.0f && measured_volts <= 20.0f)
    {
      volts = filter_initialized ? (volts * 0.8f) + (measured_volts * 0.2f) : measured_volts;
      filter_initialized = true;
    }

    if(BATTERY_CELL_COUNT > 0)
    {
      detected_battery_cells = BATTERY_CELL_COUNT;
    }
    else if(detected_battery_cells == 0 && volts > BATTERY_UNPLUGGED_THRESHOLD)
    {
      battery_detection_samples++;
      if(battery_detection_samples >= BATTERY_LOW_SAMPLE_COUNT)
      {
        detected_battery_cells = volts >= 13.0f ? 4 : 3;
      }
    }

    bool battery_present = detected_battery_cells > 0 && volts > BATTERY_UNPLUGGED_THRESHOLD;
    float warn_threshold = detected_battery_cells * BATTERY_WARN_VOLTS_PER_CELL;
    float halt_threshold = detected_battery_cells * BATTERY_HALT_VOLTS_PER_CELL;

    battery_warn_count = battery_present && volts < warn_threshold
                           ? min(battery_warn_count + 1, BATTERY_LOW_SAMPLE_COUNT)
                           : max(battery_warn_count - 1, 0);
    battery_critical_count = battery_present && volts < halt_threshold
                               ? min(battery_critical_count + 1, BATTERY_LOW_SAMPLE_COUNT)
                               : 0;
    battery_warning = battery_warn_count >= BATTERY_LOW_SAMPLE_COUNT;
    if(battery_critical_count >= BATTERY_LOW_SAMPLE_COUNT)
    {
      battery_critical_latched = true;
    }
    if(cells_before == 0 && detected_battery_cells > 0)
    {
      DEBUG_LOGF("battery: detected %dS pack at %.2f V", detected_battery_cells, volts);
    }
    if(warning_before != battery_warning)
    {
      DEBUG_LOGF("battery: warning %s at %.2f V (threshold %.2f V)",
                 battery_warning ? "active" : "cleared", volts, warn_threshold);
    }
    if(critical_before == false && battery_critical_latched)
    {
      DEBUG_LOGF("battery: CRITICAL halt latched at %.2f V (threshold %.2f V)",
                 volts, halt_threshold);
    }
    batloop = millis();
  }
}

controller::Controls current_controls()
{
  return {duty[CH_TRANSLATE_LR], duty[CH_TRANSLATE_FB], duty[CH_SPIN_THROTTLE],
          duty[CH_HEADING_YAW], duty[CH_SPIN_DIR]};
}

controller::Health current_control_health()
{
  const bool fresh_sensor = motor_timing::fresh(esp_timer_get_time(), last_accel_sample_us,
                                                motor_timing::SENSOR_MAX_AGE_US);
  return {rc_status, accel_sensor_healthy && fresh_sensor, battery_critical_latched,
          motor_output_fault_latched(), dshot_outputs_ready};
}

calibration::Safety current_calibration_safety()
{
  return {rc_status, accel_sensor_healthy, battery_critical_latched,
          motor_output_fault_latched(), accel_fault_generation};
}

void latch_control_rearm()
{
  control_rearm_state.required = true;
  control_rearm_required = true;
}

void update_control_rearm()
{
  const controller::RearmResult result = controller::update_rearm(
      control_rearm_state, current_control_health(),
      controller::neutral(current_controls(), stick_deadzone_percent, throttle_low_percent),
      config_reentry_wait_for_low && duty[CH_CONFIG_MODE] > 50,
      config_mode || wifi_mode_active);
  control_rearm_required = result.required;
  if(result.request_motor_rearm) request_motor_output_rearm();
}

void update_robot_state()
{
  set_robot_state(controller::select_state(current_control_health(), wifi_mode_active,
      config_mode, control_rearm_required, config_reentry_wait_for_low, rpm,
      current_controls(), stick_deadzone_percent));
}

bool state_requires_zero_motor_demand()
{
  return control_rearm_required || controller::requires_zero(robot_state);
}

void stop_for_current_state()
{
  set_phase_led_mode(PHASE_LED_DISABLED);
  motorR = 0;
  motorL = 0;
  pending_translation_enabled = false;
  motor_on = false;

  if(robot_state == ROBOT_LOW_BATTERY)
  {
    LEDStatus = "battery halt";
  }
  else if(robot_state == ROBOT_SENSOR_FAULT)
  {
    LEDStatus = "sensor fault";
  }
  else if(robot_state == ROBOT_FAILSAFE)
  {
    LEDStatus = "failsafe";
  }
  else if(robot_state == ROBOT_WIFI)
  {
    LEDStatus = "wifi";
  }
  else
  {
    LEDStatus = "armed";
  }

  update_motors();
  updateLED();
}

bool config_entry_controls_safe()
{
  // A sensor offset can report RPM while stationary before the first zero.
  return controller::neutral(current_controls(), stick_deadzone_percent, throttle_low_percent) &&
         motorL == 0 && motorR == 0 && !motor_on;
}

bool motors_are_off()
{
  return duty[CH_SPIN_THROTTLE] <= throttle_low_percent &&
         motorL == 0 &&
         motorR == 0 &&
         motor_on == false &&
         rpm < wifi_entry_max_rpm;
}

void show_led_feedback(String status, unsigned long duration_ms, int count = 0,
                       int index = -1)
{
  set_phase_led_mode(PHASE_LED_DISABLED);
  led_feedback_status = status;
  led_feedback_until_ms = millis() + duration_ms;
  led_feedback_count = count;
  led_feedback_index = index;
}

void stop_config_motors()
{
  config_lifecycle_output_inhibited = true;
  motorR = 0;
  motorL = 0;
  motor_on = false;
  pending_translation_enabled = false;
  pending_translation_magnitude = 0.0f;
  set_phase_led_mode(PHASE_LED_DISABLED);
  // Publish the zero-demand snapshot before zero sampling, restoring settings,
  // or writing flash; the independent motor task continues using this snapshot.
  update_motors();
}

ConfigExitReason config_abort_reason()
{
  return calibration::fault_reason(calibration_session, current_calibration_safety());
}

const char *config_exit_reason_name(ConfigExitReason reason)
{
  switch(reason)
  {
    case CONFIG_EXIT_COMMIT: return "operator CH7 release";
    case CONFIG_EXIT_RC_LOSS: return "RC loss";
    case CONFIG_EXIT_SENSOR_FAULT: return "sensor fault";
    case CONFIG_EXIT_LOW_BATTERY: return "critical battery";
    case CONFIG_EXIT_ZERO_FAILED: return "accelerometer zero failed";
    case CONFIG_EXIT_ZERO_CANCELLED: return "CH7 released during zero";
    case CONFIG_EXIT_CONTROLS_CHANGED: return "controls moved during zero";
    case CONFIG_EXIT_MOTOR_FAULT: return "motor output fault";
    default: return "unknown";
  }
}

void exit_config_mode(ConfigExitReason reason)
{
  if(!config_mode) return;
  stop_config_motors();
  // Faults take priority even if CH7 was also lowered in this frame.
  ConfigExitReason fault = config_abort_reason();
  if(reason == CONFIG_EXIT_COMMIT && fault != CONFIG_EXIT_COMMIT)
    reason = fault;

  bool commit_requested = calibration::commit_allowed(calibration_session,
      current_calibration_safety(), reason, duty[CH_CONFIG_MODE] > 50);
  bool saved = commit_requested && save_config_settings();
  if(!saved && calibration_session.snapshot_valid)
  {
    // On abort or save failure, the pre-session configuration remains active.
    // Neutral controls and a new CH7 low/high cycle permit a fresh session.
    apply_runtime_config(calibration_session.snapshot);
    reset_accel_filter_requested = true;
    last_accel_sample_us = 0;
    reset_rotation_phase_requested = true;
    rpm_variation_ewma = 0.0f;
    rpm_stable_since_ms = 0;
    current_gforce_valid = false;
    rpm = 0;
  }
  calibration::finish(calibration_session, saved);
  if(!saved) latch_control_rearm();
  last_heading_update_ms = millis();
  calcrpm();
  if(!current_control_health().sensor) latch_control_rearm();
  config_lifecycle_output_inhibited = false;
  // Republish any restored estimator values without restoring motor demand.
  update_motors();

  if(saved)
  {
    DEBUG_LOGF("config: saved complete calibration (%d correction points)",
               correction_point_count);
    show_led_feedback("config final saved", 1000, correction_point_count);
  }
  else if(commit_requested)
  {
    DEBUG_LOG("config: SAVE FAILED; pre-session settings restored");
    show_led_feedback("config save failed", 2000);
  }
  else
  {
    DEBUG_LOGF("config: aborted (%s); pre-session settings restored; lower CH7 with controls neutral before retry",
               config_exit_reason_name(reason));
    show_led_feedback(reason == CONFIG_EXIT_ZERO_FAILED
                        ? "config zero failed" : "config aborted", 1500);
  }
  updateLED();
}

void enter_config_mode()
{
  calibration::begin(calibration_session, capture_runtime_config(), accel_fault_generation);
  set_robot_state(ROBOT_CONFIG);
  last_config_adjust_ms = millis();
  stop_config_motors();
  DEBUG_LOG("config: entering; accelerometer zero starting");
  show_led_feedback("config zeroing", 600);
  updateLED();
  AccelZeroResult zero = set_accel_zero_offset();
  if(zero.reason != ACCEL_ZERO_SUCCESS)
  {
    DEBUG_LOGF("config: zero rejected reason=%d samples=%d/%d",
               (int)zero.reason, zero.valid_samples, zero.requested_samples);
    ConfigExitReason reason = CONFIG_EXIT_ZERO_FAILED;
    switch(zero.reason)
    {
      case ACCEL_ZERO_RC_LOSS: reason = CONFIG_EXIT_RC_LOSS; break;
      case ACCEL_ZERO_LOW_BATTERY: reason = CONFIG_EXIT_LOW_BATTERY; break;
      case ACCEL_ZERO_SWITCH_RELEASED: reason = CONFIG_EXIT_ZERO_CANCELLED; break;
      case ACCEL_ZERO_CONTROLS_CHANGED: reason = CONFIG_EXIT_CONTROLS_CHANGED; break;
      case ACCEL_ZERO_MOTOR_FAULT: reason = CONFIG_EXIT_MOTOR_FAULT; break;
      default: break;
    }
    exit_config_mode(reason);
    return;
  }
  // Zeroing invalidates the previous estimate. Acquire a complete sample with
  // the new offsets before publishing an enabled calibration snapshot.
  calcrpm();
  if(!current_control_health().sensor)
  {
    exit_config_mode(CONFIG_EXIT_ZERO_FAILED);
    return;
  }
  config_lifecycle_output_inhibited = false;
  update_motors();
  DEBUG_LOGF("config: accelerometer X/Y zero staged x=%.3f y=%.3f (%d/%d samples); entering RPM calibration",
             accel_zero_x_g_offset, accel_zero_y_g_offset,
             zero.valid_samples, zero.requested_samples);
  show_led_feedback("config zeroed", 700);
}

void update_config_mode()
{
  const calibration::Decision decision = calibration::update(calibration_session,
      current_calibration_safety(), duty[CH_CONFIG_MODE] > 50, config_entry_controls_safe(),
      control_rearm_required, millis(), mode_switch_hold_ms);
  if(decision.action == calibration::Action::Exit) exit_config_mode(decision.reason);
  else if(decision.action == calibration::Action::Enter) enter_config_mode();
}

void update_spin_direction_input()
{
  bool next = controller::spin_direction(reversed, current_controls(), rc_status,
                                         config_mode, wifi_mode_active);
  if(next != reversed)
  {
    reversed = next;
    DEBUG_LOG(reversed ? "spin direction: REVERSED" : "spin direction: NORMAL");
  }
}

bool wifi_entry_requested()
{
  static bool wifi_switch_was_high = false;
  static unsigned long last_blocked_log_ms = 0;
  static unsigned long last_pending_log_ms = 0;
  bool wifi_switch_high = duty[CH_WIFI_MODE] > 50;
  bool rc_safe = rc_status == true;
  bool throttle_safe = duty[CH_SPIN_THROTTLE] <= throttle_low_percent;
  int ch1_centered_value = stick_percent_centered(CH_TRANSLATE_LR);
  int ch2_centered_value = stick_percent_centered(CH_TRANSLATE_FB);
  int ch4_centered_value = stick_percent_centered(CH_HEADING_YAW);
  bool ch1_safe = ch1_centered_value == 0;
  bool ch2_safe = ch2_centered_value == 0;
  bool ch4_safe = ch4_centered_value == 0;
  bool motor_request_safe = motorL == 0 && motorR == 0 && motor_on == false;
  bool rpm_safe = rpm < wifi_entry_max_rpm;
  bool config_clear = config_mode == false;
  bool all_entry_checks_pass = rc_safe && throttle_safe && ch1_safe && ch2_safe &&
                               ch4_safe && motor_request_safe && rpm_safe &&
                               config_clear;

  if(wifi_switch_high && !wifi_switch_was_high)
  {
    DEBUG_LOGF("WiFi entry request: CH%d high value=%d", CH_WIFI_MODE,
               duty[CH_WIFI_MODE]);
  }
  wifi_switch_was_high = wifi_switch_high;

  if(wifi_switch_high && all_entry_checks_pass)
  {
    if(wifi_entry_high_start_ms == 0)
    {
      wifi_entry_high_start_ms = millis();
      last_pending_log_ms = wifi_entry_high_start_ms;
      DEBUG_LOGF("WiFi entry hold started: CH%d=%d required=%lu ms",
                 CH_WIFI_MODE, duty[CH_WIFI_MODE], mode_switch_hold_ms);
    }
    unsigned long held_ms = millis() - wifi_entry_high_start_ms;
    if(held_ms >= mode_switch_hold_ms)
    {
      DEBUG_LOGF("WiFi entry accepted: CH%d=%d held=%lu ms",
                 CH_WIFI_MODE, duty[CH_WIFI_MODE], held_ms);
      return true;
    }
    if((millis() - last_pending_log_ms) >= 250)
    {
      last_pending_log_ms = millis();
      DEBUG_LOGF("WiFi entry pending: held=%lu/%lu ms", held_ms,
                 mode_switch_hold_ms);
    }
    return false;
  }

  if(wifi_switch_high)
  {
    if((millis() - last_blocked_log_ms) >= 500)
    {
      last_blocked_log_ms = millis();
      DEBUG_LOGF("WiFi entry blocked CH%d=%d: rc=%s throttle=%s(%d<=%d) ch1=%s(%d) ch2=%s(%d) ch4=%s(%d) motors=%s(L%d R%d active%d) rpm=%s(%d<%d) config_clear=%s",
                 CH_WIFI_MODE, duty[CH_WIFI_MODE], rc_safe ? "PASS" : "FAIL",
                 throttle_safe ? "PASS" : "FAIL", duty[CH_SPIN_THROTTLE],
                 throttle_low_percent, ch1_safe ? "PASS" : "FAIL",
                 ch1_centered_value, ch2_safe ? "PASS" : "FAIL",
                 ch2_centered_value, ch4_safe ? "PASS" : "FAIL",
                 ch4_centered_value, motor_request_safe ? "PASS" : "FAIL",
                 motorL, motorR, motor_on ? 1 : 0, rpm_safe ? "PASS" : "FAIL",
                 rpm, wifi_entry_max_rpm, config_clear ? "PASS" : "FAIL");
    }
    if(wifi_entry_high_start_ms != 0)
    {
      DEBUG_LOGF("WiFi entry hold reset: a safety check stopped passing after %lu ms",
                 millis() - wifi_entry_high_start_ms);
    }
  }
  else if(wifi_entry_high_start_ms != 0)
  {
    DEBUG_LOGF("WiFi entry cancelled: CH%d lowered after %lu/%lu ms",
               CH_WIFI_MODE, millis() - wifi_entry_high_start_ms,
               mode_switch_hold_ms);
  }

  wifi_entry_high_start_ms = 0;
  return false;
}

void handle_config_save_button()
{
  calibration::Gesture gesture = calibration::update_gesture(calibration_session,
      duty[CH_CONFIG_SAVE], millis(), config_stage_advance_window_ms,
      config_stage_advance_press_count, config_clear_press_count);
  if(gesture == calibration::Gesture::EnterHeading || gesture == calibration::Gesture::HeadingReminder)
  {
    show_led_feedback("config heading mode", gesture == calibration::Gesture::EnterHeading ? 1000 : 400,
                      correction_point_count);
    return;
  }
  if(gesture == calibration::Gesture::ClearRpm)
  {
    clearCorrectionTable();
    offset = 1.0f;
    DEBUG_LOGF("config: RPM calibration cleared; LED heading preserved at %.1f degrees", led_heading_offset_deg);
    show_led_feedback("config rpm cleared", 1000);
    return;
  }
  if(gesture != calibration::Gesture::CapturePoint) return;
  bool stable_rpm = rpm_is_stable();
  DEBUG_LOGF("config: RPM point capture requested rpm=%d stable=%d variation=%.2f", rpm, stable_rpm ? 1 : 0, rpm_variation_ewma);
  if(rpm > 400 && stable_rpm)
  {
    int saved_throttle = map(constrain(duty[CH_SPIN_THROTTLE], 0, 100), 0, 100, -100, 100);
    bool replaced_full_table = false;
    int saved_index = setCorrectionPoint(rpm, offset, saved_throttle, &replaced_full_table);
    DEBUG_LOGF("config: correction point staged rpm=%d offset=%.4f throttle=%d count=%d index=%d replaced_full=%d",
               rpm, offset, saved_throttle, correction_point_count, saved_index, replaced_full_table ? 1 : 0);
    show_led_feedback(replaced_full_table ? "config point replaced" : "config point saved", 600,
                      correction_point_count, replaced_full_table ? saved_index : -1);
  }
  else if(rpm > 200) show_led_feedback("config unstable", 700, correction_point_count);
  else show_led_feedback("config rpm mode", 400, correction_point_count);
}

void handle_config_adjustments()
{
  unsigned long now = millis();
  calibration::adjust(config_calibration_stage, duty[CH_TRANSLATE_LR], stick_deadzone_percent,
      config_rpm_offset_rate, config_led_offset_rate_dps,
      (now - last_config_adjust_ms) * 0.001f, offset, led_heading_offset_deg);
  last_config_adjust_ms = now;
  handle_config_save_button();
}

void config_loop()
{
  set_robot_state(ROBOT_CONFIG);
  calcrpm();
  ConfigExitReason fault = config_abort_reason();
  if(fault != CONFIG_EXIT_COMMIT)
  {
    exit_config_mode(fault);
    update_robot_state();
    stop_for_current_state();
    return;
  }
  handle_config_adjustments();

  if(duty[CH_SPIN_THROTTLE] > 10)
  {
    // Translation testing belongs only to heading-offset calibration.
    if(config_calibration_stage == CONFIG_STAGE_HEADING &&
       stick_percent_centered(CH_TRANSLATE_FB) > 0)
    {
      translate_field_oriented();
    }
    else
    {
      spin();
    }
  }
  else
  {
    motorR = 0;
    motorL = 0;
    pending_translation_enabled = false;
    motor_on = false;
  }

  bool config_rpm_stable = rpm > 400 && rpm_is_stable();
  bool feedback_active = led_feedback_until_ms > millis();
  PhaseLedMode requested_phase_led_mode = PHASE_LED_DISABLED;

  if(!feedback_active && rpm > 400)
  {
    if(config_calibration_stage == CONFIG_STAGE_RPM && config_rpm_stable)
    {
      requested_phase_led_mode = PHASE_LED_CONFIG_STABLE;
    }
    else
    {
      requested_phase_led_mode = PHASE_LED_CONFIG_DARK;
    }
  }

  bool phase_led_rendering = set_phase_led_mode(requested_phase_led_mode);

  if(!phase_led_rendering && rpm > 400 && isAngleInRange(current_led_heading_phase_deg(), HEADING_LED_HALF_WIDTH_DEG) &&
     config_calibration_stage == CONFIG_STAGE_RPM && config_rpm_stable)
  {
    LEDStatus = "config heading centered";
  }
  else if(!phase_led_rendering && config_calibration_stage == CONFIG_STAGE_RPM && config_rpm_stable)
  {
    LEDStatus = "config rpm stable";
  }
  else if(!phase_led_rendering && rpm > 400 && isAngleInRange(current_led_heading_phase_deg(), HEADING_LED_HALF_WIDTH_DEG))
  {
    LEDStatus = "heading on";
  }
  else if(!phase_led_rendering && duty[CH_SPIN_THROTTLE] > 10)
  {
    LEDStatus = "motor on";
  }
  else if(!phase_led_rendering && config_calibration_stage == CONFIG_STAGE_HEADING)
  {
    LEDStatus = "config heading mode";
  }
  else if(!phase_led_rendering)
  {
    LEDStatus = "config rpm mode";
  }

  if(!phase_led_rendering)
  {
    updateLED();
  }
  update_motors();
}

//----------------------------------------------------------------------------------------
void loop() 
{
  get_battery();
  update_channels();
  calcrpm();

  if(wifi_mode_active)
  {
    service_wifi_mode();
    update_robot_state();
    service_session_logger();
    stop_for_current_state();
    return;
  }

  update_control_rearm();
  update_config_mode();
  if(reset_accel_filter_requested) calcrpm();
  update_control_rearm();
  update_spin_direction_input();
  update_robot_state();
  service_session_logger();

  if(config_mode == false)
  {
    // Keep the integration timestamp fresh while armed and idle so the first
    // yaw command after a long idle period cannot create a large heading jump.
    update_heading_reference();
  }

  if(wifi_entry_requested())
  {
    wifi_entry_high_start_ms = 0;
    wifi_mode();
    stop_for_current_state();
    return;
  }

  if(state_requires_zero_motor_demand())
  {
    stop_for_current_state();
    loop_time = esp_timer_get_time() - loop_start;
    loop_start = esp_timer_get_time();
    return;
  }

  if(config_mode == true)
  {
    config_loop();
    return;
  }

    bool phase_led_rendering = false;
    pending_translation_enabled = false;
    if(robot_state == ROBOT_MELTY)
    {
     if((reversed == false && duty[CH_SPIN_DIR] > 50) || (reversed == true && duty[CH_SPIN_DIR] < 50)) //activates celebration led display if reverse switch is toggled while spinning
     {
      set_phase_led_mode(PHASE_LED_DISABLED);
      LEDStatus = "celebrate";
     }
     else if(led_feedback_until_ms <= millis())
     {
       phase_led_rendering = set_phase_led_mode(PHASE_LED_MELTY);
     }
     else
     {
       set_phase_led_mode(PHASE_LED_DISABLED);
     }
     if(translation_active())
     {
      translate_field_oriented();
     }
     else 
     {
      spin();
     }
    
    } //-------------------------------------------
    else if(robot_state == ROBOT_SPINUP)
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      spin();
    }
    else if(robot_state == ROBOT_TANK)
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      const controller::MotorDemand demand = controller::tank_demand(
          duty[CH_TRANSLATE_LR], duty[CH_TRANSLATE_FB], reversed);
      motorL = demand.left;
      motorR = demand.right;
    }
    else
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      motorL = 0;
      motorR = 0;
    }
    if(rpm < 400)
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      phase_led_rendering = false;
      LEDStatus = "armed";
    }
  if(!phase_led_rendering)
  {
    updateLED();
  }
  loop_time = esp_timer_get_time() - loop_start;
  update_motors();
  report_motor_output_status();
  loop_start = esp_timer_get_time();

}
