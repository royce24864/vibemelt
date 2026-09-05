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

enum RobotState
{
  ROBOT_ARMED_IDLE,
  ROBOT_TANK,
  ROBOT_SPINUP,
  ROBOT_MELTY,
  ROBOT_CONFIG,
  ROBOT_WIFI,
  ROBOT_FAILSAFE,
  ROBOT_LOW_BATTERY,
  ROBOT_SENSOR_FAULT
};

enum PhaseLedMode : uint8_t
{
  PHASE_LED_DISABLED,
  PHASE_LED_MELTY,
  PHASE_LED_CONFIG_STABLE,
  PHASE_LED_CONFIG_DARK
};

enum PhaseLedFrame : uint8_t
{
  PHASE_LED_FRAME_INVALID,
  PHASE_LED_FRAME_OFF,
  PHASE_LED_FRAME_TOP_HEADING,
  PHASE_LED_FRAME_BOTTOM_HEADING,
  PHASE_LED_FRAME_TOP_HEADING_BOTTOM_MOTOR,
  PHASE_LED_FRAME_MOTOR_TOP_BOTTOM_HEADING,
  PHASE_LED_FRAME_CONFIG_TOP_HEADING,
  PHASE_LED_FRAME_CONFIG_BOTTOM_HEADING,
  PHASE_LED_FRAME_CONFIG_STABLE,
  PHASE_LED_FRAME_MOTOR_ON
};

enum ConfigCalibrationStage : uint8_t
{
  CONFIG_STAGE_RPM,
  CONFIG_STAGE_HEADING
};

struct MotorControlSnapshot
{
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
volatile PhaseLedMode phase_led_mode = PHASE_LED_DISABLED;
volatile PhaseLedFrame phase_led_frame = PHASE_LED_FRAME_INVALID;
volatile uint32_t phase_led_transition_count = 0;
volatile uint32_t phase_led_render_count = 0;
Preferences config_store;
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
int accel_health_failure_count = 0;
int accel_read_failure_count = 0;
unsigned long accel_loss_started_ms = 0;
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
bool config_mode = false;
ConfigCalibrationStage config_calibration_stage = CONFIG_STAGE_RPM;
bool config_save_latched = false;
int config_stage_press_count = 0;
unsigned long config_stage_press_start_ms = 0;
unsigned long config_entry_high_start_ms = 0;
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
  config_store.begin(NVS_NAMESPACE, false);
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
  if(escR_status.success && escL_status.success)
  {
    session_logger_delay(250);
    bool startup_ok = true;
    for(int frame = 0; frame < 1500; frame++)
    {
      dshot_result_t right_status = escR.sendThrottle(0);
      dshot_result_t left_status = escL.sendThrottle(0);
      if(!right_status.success || !left_status.success) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    // AM32 requires setting commands at least six times. Keep this behavior in
    // the main firmware so DShotRMT itself remains at its upstream 0.9.5 implementation.
    DEBUG_LOG("DShot startup: enabling AM32 3D mode");
    for(int frame = 0; frame < 10; frame++)
    {
      dshot_result_t right_3d_status =
          escR.sendCommand(DSHOT_CMD_3D_MODE_ON, 1, 0);
      dshot_result_t left_3d_status =
          escL.sendCommand(DSHOT_CMD_3D_MODE_ON, 1, 0);
      if(!right_3d_status.success || !left_3d_status.success) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    for(int frame = 0; frame < 100; frame++)
    {
      dshot_result_t right_status = escR.sendThrottle(0);
      dshot_result_t left_status = escL.sendThrottle(0);
      if(!right_status.success || !left_status.success) startup_ok = false;
      delay(1);
      service_session_logger();
    }

    DEBUG_LOGF("DShot startup: zero arm and AM32 3D setup complete status=%d",
               startup_ok ? 0 : 1);
    if(startup_ok)
    {
      dshot_outputs_ready = true;
      // The 2 kHz phase task publishes DShot300 at 2 kHz on both independent
      // RMT channels.
      init_motor_output_task();
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
  DEBUG_LOG("setup complete; waiting for valid RC");
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

void update_robot_state()
{
  if(wifi_mode_active)
  {
    set_robot_state(ROBOT_WIFI);
  }
  else if(battery_critical_latched)
  {
    set_robot_state(ROBOT_LOW_BATTERY);
  }
  else if(accel_sensor_healthy == false)
  {
    set_robot_state(ROBOT_SENSOR_FAULT);
  }
  else if(rc_status == false)
  {
    set_robot_state(ROBOT_FAILSAFE);
  }
  else if(config_mode)
  {
    set_robot_state(ROBOT_CONFIG);
  }
  else if(rpm > 400 && duty[CH_SPIN_THROTTLE] > 10)
  {
    set_robot_state(ROBOT_MELTY);
  }
  else if(duty[CH_SPIN_THROTTLE] > 10)
  {
    set_robot_state(ROBOT_SPINUP);
  }
  else if(translation_active())
  {
    set_robot_state(ROBOT_TANK);
  }
  else
  {
    set_robot_state(ROBOT_ARMED_IDLE);
  }
}

bool state_requires_zero_motor_demand()
{
  return robot_state == ROBOT_ARMED_IDLE || robot_state == ROBOT_WIFI ||
         robot_state == ROBOT_FAILSAFE || robot_state == ROBOT_LOW_BATTERY ||
         robot_state == ROBOT_SENSOR_FAULT;
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
  // Do not gate accelerometer calibration on the RPM estimate. Before the
  // first successful zero, sensor offset can make a stationary robot appear
  // to be spinning and would make config mode impossible to enter.
  return duty[CH_SPIN_THROTTLE] <= throttle_low_percent &&
         stick_percent_centered(CH_TRANSLATE_LR) == 0 &&
         stick_percent_centered(CH_TRANSLATE_FB) == 0 &&
         stick_percent_centered(CH_HEADING_YAW) == 0 &&
         motorL == 0 && motorR == 0 && motor_on == false;
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

void save_config_settings()
{
  sortCorrectionTable();
  config_store.putFloat("rpm_offset", offset);
  // Keep the existing NVS key for compatibility with previously saved values.
  config_store.putFloat("led_heading", led_heading_offset_deg);
  config_store.putFloat("accel_zero_x", accel_zero_x_g_offset);
  config_store.putFloat("accel_zero_y", accel_zero_y_g_offset);
  config_store.putFloat("acc_rate", acc_rate);
  config_store.putFloat("pulse_pct", percentdecel);
  config_store.putFloat("trans_scale", translation_scale_factor);
  config_store.putInt("corr_count", correction_point_count);
  for(int i = 0; i < MAX_CORRECTION_POINTS; i++)
  {
    char rpm_key[8];
    char offset_key[8];
    char throttle_key[8];
    snprintf(rpm_key, sizeof(rpm_key), "rpm%d", i);
    snprintf(offset_key, sizeof(offset_key), "off%d", i);
    snprintf(throttle_key, sizeof(throttle_key), "thr%d", i);
    config_store.putFloat(rpm_key, correction_rpm[i]);
    config_store.putFloat(offset_key, correction_offset[i]);
    config_store.putInt(throttle_key, correction_throttle[i]);
  }
}

void enter_config_mode()
{
  config_mode = true;
  config_calibration_stage = CONFIG_STAGE_RPM;
  set_robot_state(ROBOT_CONFIG);
  DEBUG_LOG("config: entering; accelerometer zero starting");
  config_save_latched = false;
  config_stage_press_count = 0;
  config_stage_press_start_ms = 0;
  last_config_adjust_ms = millis();
  motorR = 0;
  motorL = 0;
  update_motors();
  show_led_feedback("config zeroing", 600);
  updateLED();
  set_accel_zero_offset();
  DEBUG_LOGF("config: accelerometer X/Y zero staged x=%.3f y=%.3f; entering RPM calibration",
             accel_zero_x_g_offset, accel_zero_y_g_offset);
  show_led_feedback("config zeroed", 700);
}

void exit_config_mode()
{
  DEBUG_LOGF("config: exiting and saving (%d correction points)", correction_point_count);
  save_config_settings();
  show_led_feedback("config final saved", 1000, correction_point_count);
  updateLED();
  config_mode = false;
  config_calibration_stage = CONFIG_STAGE_RPM;
  config_save_latched = false;
  config_stage_press_count = 0;
  config_stage_press_start_ms = 0;
}

void update_config_mode()
{
  static bool config_switch_was_high = false;
  bool config_switch_high = duty[CH_CONFIG_MODE] > 50;

  if(config_switch_high != config_switch_was_high)
  {
    DEBUG_LOGF("config: CH7 %s value=%d",
               config_switch_high ? "high" : "low", duty[CH_CONFIG_MODE]);
    config_switch_was_high = config_switch_high;
  }

  if(config_switch_high)
  {
    if(config_mode == false && rc_status == true && config_entry_controls_safe())
    {
      if(config_entry_high_start_ms == 0)
      {
        config_entry_high_start_ms = millis();
        DEBUG_LOG("config: entry hold started");
      }
      if((millis() - config_entry_high_start_ms) >= mode_switch_hold_ms)
      {
        config_entry_high_start_ms = 0;
        enter_config_mode();
      }
    }
    else if(config_mode == false)
    {
      static unsigned long last_blocked_log_ms = 0;
      if((millis() - last_blocked_log_ms) >= 500)
      {
        last_blocked_log_ms = millis();
        DEBUG_LOGF("config: entry blocked rc=%d throttle=%d ch1=%d ch2=%d ch4=%d motors=(%d,%d) active=%d",
                   rc_status ? 1 : 0, duty[CH_SPIN_THROTTLE],
                   duty[CH_TRANSLATE_LR], duty[CH_TRANSLATE_FB],
                   duty[CH_HEADING_YAW], motorL, motorR, motor_on ? 1 : 0);
      }
      config_entry_high_start_ms = 0;
    }
  }
  else
  {
    config_entry_high_start_ms = 0;
    if(config_mode == true)
    {
      exit_config_mode();
    }
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
  unsigned long now = millis();

  // A three-press action cannot be committed on the third edge because the
  // same sequence may continue to the five-press clear action. Resolve the
  // shorter gesture only after its window closes.
  if(config_calibration_stage == CONFIG_STAGE_RPM &&
     config_stage_press_start_ms != 0 &&
     (now - config_stage_press_start_ms) > config_stage_advance_window_ms)
  {
    int completed_press_count = config_stage_press_count;
    config_stage_press_count = 0;
    config_stage_press_start_ms = 0;

    if(completed_press_count >= config_stage_advance_press_count)
    {
      config_calibration_stage = CONFIG_STAGE_HEADING;
      DEBUG_LOGF("config: entering heading-offset calibration after %d CH8 presses with %d staged RPM points",
                 completed_press_count, correction_point_count);
      show_led_feedback("config heading mode", 1000, correction_point_count);
    }
  }

  if(duty[CH_CONFIG_SAVE] > 50 && config_save_latched == false)
  {
    config_save_latched = true;

    if(config_calibration_stage == CONFIG_STAGE_HEADING)
    {
      DEBUG_LOG("config: CH8 ignored in heading stage; lower CH7 to commit calibration");
      show_led_feedback("config heading mode", 400);
      return;
    }

    if(config_stage_press_start_ms == 0 ||
       (now - config_stage_press_start_ms) > config_stage_advance_window_ms)
    {
      config_stage_press_start_ms = now;
      config_stage_press_count = 1;
    }
    else
    {
      config_stage_press_count++;
    }
    DEBUG_LOGF("config: RPM-stage CH8 sequence %d/%d (heading) %d/%d (clear)",
               config_stage_press_count, config_stage_advance_press_count,
               config_stage_press_count, config_clear_press_count);

    if(config_stage_press_count >= config_clear_press_count)
    {
      clearCorrectionTable();
      offset = 1.0f;
      config_stage_press_count = 0;
      config_stage_press_start_ms = 0;
      DEBUG_LOGF("config: RPM calibration cleared; LED heading preserved at %.1f degrees",
                 led_heading_offset_deg);
      show_led_feedback("config rpm cleared", 1000);
      return;
    }

    // Three and four presses remain pending until the sequence window closes,
    // leaving time for a fifth press to select clear instead.
    if(config_stage_press_count >= config_stage_advance_press_count)
    {
      return;
    }

    bool stable_rpm = rpm_is_stable();
    DEBUG_LOGF("config: RPM point capture requested rpm=%d stable=%d variation=%.2f",
               rpm, stable_rpm ? 1 : 0, rpm_variation_ewma);

    if(rpm > 400 && stable_rpm)
    {
      int saved_throttle = map(constrain(duty[CH_SPIN_THROTTLE], 0, 100),
                               0, 100, -100, 100);
      bool replaced_full_table = false;
      int saved_index = setCorrectionPoint(rpm, offset, saved_throttle,
                                           &replaced_full_table);
      DEBUG_LOGF("config: correction point staged rpm=%d offset=%.4f throttle=%d count=%d index=%d replaced_full=%d",
                 rpm, offset, saved_throttle, correction_point_count,
                 saved_index, replaced_full_table ? 1 : 0);
      if(replaced_full_table)
      {
        show_led_feedback("config point replaced", 600,
                          correction_point_count, saved_index);
      }
      else
      {
        show_led_feedback("config point saved", 600, correction_point_count);
      }
    }
    else if(rpm > 200)
    {
      DEBUG_LOGF("config: correction capture rejected; RPM unstable rpm=%d variation=%.2f",
                 rpm, rpm_variation_ewma);
      show_led_feedback("config unstable", 700, correction_point_count);
    }
    else
    {
      show_led_feedback("config rpm mode", 400, correction_point_count);
    }
  }
  else if(duty[CH_CONFIG_SAVE] < 40)
  {
    config_save_latched = false;
  }
}

void handle_config_adjustments()
{
  unsigned long now = millis();
  float dt = constrain((now - last_config_adjust_ms) * 0.001f, 0.0f, 0.05f);
  last_config_adjust_ms = now;

  // Reverse CH1 for calibration adjustments without changing its polarity in
  // normal driving, translation tests, or config-entry safety checks.
  int adjust = -stick_percent_centered(CH_TRANSLATE_LR);
  float adjust_fraction = adjust / 50.0;

  if(config_calibration_stage == CONFIG_STAGE_HEADING)
  {
    led_heading_offset_deg = wrap360(led_heading_offset_deg +
                                     adjust_fraction * config_led_offset_rate_dps * dt);
  }
  else
  {
    // Rescale the RPM-calibration response after the deadband. This removes
    // the old jump from zero directly to ~16% of full adjustment speed while
    // retaining the same maximum rate at full CH1 stick.
    int centered = 50 - duty[CH_TRANSLATE_LR];
    int magnitude = abs(centered);
    float rpm_adjust_fraction = 0.0f;
    if(magnitude > stick_deadzone_percent)
    {
      rpm_adjust_fraction = (magnitude - stick_deadzone_percent) /
                            (float)(50 - stick_deadzone_percent);
      if(centered < 0)
      {
        rpm_adjust_fraction = -rpm_adjust_fraction;
      }
    }

    offset = constrain(offset + rpm_adjust_fraction * config_rpm_offset_rate * dt,
                       0.8, 1.2);
  }

  handle_config_save_button();
}

void config_loop()
{
  set_robot_state(ROBOT_CONFIG);
  calcrpm();
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

  update_config_mode();
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

  if(duty[CH_SPIN_DIR] > 50 && duty[CH_SPIN_THROTTLE] < 20) //only reverses spin direction if throttle is under 20%
  {
    if(reversed == false)
    {
      reversed = true;
      DEBUG_LOG("spin direction: REVERSED");
    }
  }
  else if(duty[CH_SPIN_DIR] < 50 && duty[CH_SPIN_THROTTLE] < 20)
  {
    if(reversed)
    {
      reversed = false;
      DEBUG_LOG("spin direction: NORMAL");
    }
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
    else if(robot_state == ROBOT_TANK && reversed == true &&
            (duty[CH_TRANSLATE_FB] > 55 || duty[CH_TRANSLATE_FB] < 45)) //unstick using the right stick
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      motorL = map(duty[CH_TRANSLATE_FB], 0, 100, 1000, -1000);
      motorR = map(duty[CH_TRANSLATE_FB], 0, 100, -1000, 1000);
    }
    else if(robot_state == ROBOT_TANK) //tank drive mode
    {
      set_phase_led_mode(PHASE_LED_DISABLED);
      motorL = map(duty[CH_TRANSLATE_FB], 0, 100, 100, -100) + map(duty[CH_TRANSLATE_LR], 0, 100, 20, -20);
      motorR = map(duty[CH_TRANSLATE_FB], 0, 100, -100, 100) + map(duty[CH_TRANSLATE_LR], 0, 100, 20, -20);
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
