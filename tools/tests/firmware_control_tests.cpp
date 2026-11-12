#include "Controller.h"
#include "Estimator.h"
#include "Calibration.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

static bool near(float a, float b) { return std::fabs(a - b) < 0.001f; }

static void test_controller()
{
  controller::Controls controls{50, 50, 0, 50, 100};
  controller::Health healthy{true, true, false, false, true};
  controller::RearmState rearm;
  assert(rearm.required); // Startup also requires a deliberate neutral observation.
  controls.throttle = 80;
  assert(controller::update_rearm(rearm, healthy, false, false, false).required);
  assert(controller::select_state(healthy, false, false, rearm.required, false, 0, controls, 8) == ROBOT_ARMED_IDLE);
  controls.throttle = 0;
  assert(controller::neutral(controls, 8, 5));
  assert(!controller::update_rearm(rearm, healthy, true, false, false).required);
  bool reversed = controller::spin_direction(false, controls, true, false, false);
  assert(reversed);
  controls.direction = 0;
  controls.throttle = 80;
  assert(controller::spin_direction(reversed, controls, true, false, false));
  assert(controller::select_state(healthy, false, false, false, false, 0, controls, 8) == ROBOT_SPINUP);
  assert(controller::select_state(healthy, false, false, false, false, 401, controls, 8) == ROBOT_MELTY);
  controls.throttle = 0;
  assert(controller::spin_direction(true, controls, false, false, false));
  assert(controller::spin_direction(true, controls, true, true, false));
  controls.direction = 50;
  assert(controller::spin_direction(true, controls, true, false, false));

  for(int fault = 0; fault < 4; ++fault)
  {
    auto health = healthy;
    rearm.required = false;
    if(fault == 0) health.rc = false;
    if(fault == 1) health.sensor = false;
    if(fault == 2) health.outputs_ready = false;
    if(fault == 3) health.motor_fault = true;
    assert(controller::update_rearm(rearm, health, false, false, false).required);
    // Clearing a transient fault cannot restart a held command.
    assert(controller::update_rearm(rearm, healthy, false, false, false).required);
    assert(controller::update_rearm(rearm, healthy, true, true, false).required);
    assert(controller::update_rearm(rearm, healthy, true, false, true).required);
    assert(!controller::update_rearm(rearm, healthy, true, false, false).required);
  }
  auto motor_fault = healthy;
  motor_fault.motor_fault = true;
  auto decision = controller::update_rearm(rearm, motor_fault, true, false, false);
  assert(decision.required && decision.request_motor_rearm);
  // Repeated fault observations do not prevent the motor task's acknowledgement.
  decision = controller::update_rearm(rearm, motor_fault, true, false, false);
  assert(decision.required && decision.request_motor_rearm);
  decision = controller::update_rearm(rearm, healthy, true, false, false);
  assert(!decision.required && !decision.request_motor_rearm);
  healthy.battery_critical = true;
  assert(controller::update_rearm(rearm, healthy, true, false, false).required);
  assert(controller::select_state(healthy, false, true, false, false, 1000, controls, 8) == ROBOT_LOW_BATTERY);
  healthy.battery_critical = false;
  assert(controller::select_state(healthy, false, true, false, false, 1000, controls, 8) == ROBOT_CONFIG);
  assert(controller::select_state(healthy, true, true, false, false, 1000, controls, 8) == ROBOT_WIFI);
  controls.translate_lr = 58;
  assert(!controller::neutral(controls, 8, 5));
  assert(controller::select_state(healthy, false, false, false, false, 0, controls, 8) == ROBOT_TANK);
  controls.translate_lr = 57;
  assert(controller::neutral(controls, 8, 5));
  controls.yaw = 42;
  assert(!controller::neutral(controls, 8, 5));
  controls.yaw = 50;
  controls.translate_fb = 42;
  assert(!controller::neutral(controls, 8, 5));
  controls.translate_fb = 50;
  controls.throttle = 6;
  assert(!controller::neutral(controls, 8, 5));
  for(RobotState state : {ROBOT_ARMED_IDLE, ROBOT_WIFI, ROBOT_FAILSAFE, ROBOT_SENSOR_FAULT, ROBOT_LOW_BATTERY})
    assert(controller::requires_zero(state));
  for(RobotState state : {ROBOT_TANK, ROBOT_SPINUP, ROBOT_MELTY, ROBOT_CONFIG})
    assert(!controller::requires_zero(state));

  assert(controller::spin_demand(100, 1000, 0.4f, false) == 600);
  assert(controller::spin_demand(100, 1000, 0.4f, true) == -600);
  assert(controller::spin_demand(100, 5000, 0.4f, false) == 999);
  assert(controller::spin_demand(10, 0, 0.4f, true) == -100);
  auto tank = controller::tank_demand(50, 100, false);
  assert(tank.left == -100 && tank.right == 100);
  tank = controller::tank_demand(51, 50, false);
  assert(tank.left == 0 && tank.right == 0); // Arduino integer map truncation.
  tank = controller::tank_demand(0, 0, true);
  assert(tank.left == 1000 && tank.right == -1000);
  auto translation = controller::translation(50, 100, 37.0f, 8);
  assert(translation.enabled && near(translation.magnitude, 1.0f) && near(translation.target_phase_deg, 37.0f));
  translation = controller::translation(100, 50, 0.0f, 8);
  assert(translation.enabled && near(translation.target_phase_deg, 270.0f));
  translation = controller::translation(50, 0, 270.0f, 8);
  assert(near(translation.target_phase_deg, 90.0f));
  translation = controller::translation(57, 43, 0.0f, 8);
  assert(!translation.enabled);
  assert(near(controller::heading_after_yaw(0, 100, 8, 180.0f, 30.0f), 351.0f));
}

static void test_estimator()
{
  estimator::Config config{1.0f, 0.0f, 100.0f, 0.4084f, 0.1056f, 750};
  estimator::State state;
  float sample = 0;
  assert(!estimator::stable(state, config, 0));
  assert(estimator::accept_sample(state, config, 101, 900, 1, 0, 0, sample));
  assert(near(sample, 100) && state.rpm == 1000); // Tangential acceleration does not affect radial estimate.
  assert(near(state.variation, 105.6f));
  for(std::uint32_t ms = 3; ms <= 1500; ms += 3)
    assert(estimator::accept_sample(state, config, 101, 900, 1, 0, ms, sample));
  assert(estimator::stable(state, config, 1500));
  assert(!estimator::stable(state, config, 1550));
  const auto before = state;
  assert(!estimator::accept_sample(state, config, std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 1550, sample));
  assert(state.last_sample_ms == before.last_sample_ms && state.rpm == before.rpm);
  // One radial spike is rejected by the populated median filter.
  assert(estimator::accept_sample(state, config, 401, 0, 1, 0, 1503, sample));
  assert(state.rpm == 1000);
  estimator::reset(state);
  assert(!state.valid && state.rpm == 0 && !state.stability_started);
  assert(estimator::accept_sample(state, config, 25, 0, 0, 0, 2000, sample));
  assert(state.rpm == 500 && near(state.variation, 52.8f)); // Reset removes old RPM history too.
  assert(!estimator::stable(state, config, 2800));
  estimator::reset(state);
  // Elapsed calculations continue across the uint32 millisecond wrap.
  config.variation_alpha = 0.0f;
  const std::uint32_t start = 0xffffff00U;
  assert(estimator::accept_sample(state, config, 100, 0, 0, 0, start, sample));
  assert(estimator::accept_sample(state, config, 100, 0, 0, 0, start + 800U, sample));
  assert(estimator::stable(state, config, start + 800U));
}

static void test_calibration()
{
  using calibration::Action;
  using calibration::Gesture;
  calibration::Session session;
  calibration::Safety safety{true, true, false, false, 7};
  auto config = config_record::defaults();
  auto tick = [&](bool high, bool safe, bool rearm, std::uint32_t now) {
    return calibration::update(session, safety, high, safe, rearm, now, 1000);
  };
  assert(tick(true, true, false, 0).action == Action::None);
  assert(tick(true, true, false, 999).action == Action::None);
  assert(tick(true, true, false, 1000).action == Action::Enter);
  calibration::begin(session, config, 7);
  assert(session.active && session.snapshot_valid && session.stage == CONFIG_STAGE_RPM);
  config.acc_rate = 0.8f;
  assert(near(session.snapshot.acc_rate, 0.4f));
  assert(!calibration::commit_allowed(session, safety, CONFIG_EXIT_COMMIT, true));
  assert(calibration::commit_allowed(session, safety, CONFIG_EXIT_COMMIT, false));
  safety.rc_healthy = false;
  assert(tick(false, true, false, 1010).reason == CONFIG_EXIT_RC_LOSS);
  assert(!calibration::commit_allowed(session, safety, CONFIG_EXIT_COMMIT, false));
  safety.rc_healthy = true;
  ++safety.sensor_fault_generation;
  assert(tick(false, true, false, 1010).reason == CONFIG_EXIT_SENSOR_FAULT);
  calibration::finish(session, false);
  assert(!session.active && session.wait_for_low && !session.snapshot_valid);
  tick(true, true, true, 2000);
  assert(session.wait_for_low);
  tick(false, false, true, 2000);
  assert(session.wait_for_low);
  tick(false, true, true, 2000);
  assert(!session.wait_for_low);
  assert(tick(true, true, true, 4000).action == Action::None);
  assert(tick(true, true, false, 5000).action == Action::None);
  assert(tick(true, true, false, 6000).action == Action::Enter);
  calibration::begin(session, config, safety.sensor_fault_generation);
  assert(tick(true, true, true, 6100).reason == CONFIG_EXIT_MOTOR_FAULT);
  safety.motor_fault = true;
  assert(!calibration::commit_allowed(session, safety, CONFIG_EXIT_COMMIT, false));
  safety.motor_fault = false;
  safety.battery_critical = true;
  assert(calibration::fault_reason(session, safety) == CONFIG_EXIT_LOW_BATTERY);
  safety.battery_critical = false;
  calibration::finish(session, true);
  assert(!session.active && !session.wait_for_low);

  calibration::begin(session, config, safety.sensor_fault_generation);
  auto press = [&](std::uint32_t ms) {
    calibration::update_gesture(session, 0, ms, 3000, 3, 5);
    return calibration::update_gesture(session, 100, ms, 3000, 3, 5);
  };
  assert(press(0) == Gesture::CapturePoint);
  assert(press(500) == Gesture::CapturePoint);
  assert(press(1000) == Gesture::None);
  assert(session.stage == CONFIG_STAGE_RPM);
  assert(calibration::update_gesture(session, 0, 3001, 3000, 3, 5) == Gesture::EnterHeading);
  assert(session.stage == CONFIG_STAGE_HEADING);
  assert(press(3100) == Gesture::HeadingReminder);
  calibration::begin(session, config, safety.sensor_fault_generation);
  press(0); press(400); press(800); press(1200);
  assert(press(1600) == Gesture::ClearRpm);
  assert(session.stage == CONFIG_STAGE_RPM);
  assert(calibration::update_gesture(session, 0, 4000, 3000, 3, 5) == Gesture::None);

  calibration::ZeroAccumulator zero;
  float x = 17, y = 18;
  for(int i = 0; i < 99; ++i) assert(calibration::add_zero_sample(zero, 1, -2));
  assert(!calibration::finish_zero(zero, 100, x, y) && x == 17 && y == 18);
  assert(!calibration::add_zero_sample(zero, std::numeric_limits<float>::infinity(), 0));
  assert(zero.samples == 99);
  assert(calibration::add_zero_sample(zero, 1, -2));
  assert(calibration::finish_zero(zero, 100, x, y) && x == 1 && y == -2);
  assert(calibration::zero_cancel_reason(safety, true, true) == ACCEL_ZERO_SUCCESS);
  assert(calibration::zero_cancel_reason(safety, false, true) == ACCEL_ZERO_SWITCH_RELEASED);
  assert(calibration::zero_cancel_reason(safety, true, false) == ACCEL_ZERO_CONTROLS_CHANGED);
  safety.rc_healthy = false;
  assert(calibration::zero_cancel_reason(safety, false, false) == ACCEL_ZERO_RC_LOSS);
  safety.rc_healthy = true;
  safety.sensor_healthy = false;
  assert(calibration::zero_cancel_reason(safety, true, true) == ACCEL_ZERO_SENSOR_FAULT);
  safety.sensor_healthy = true;
  safety.motor_fault = true;
  assert(calibration::zero_cancel_reason(safety, true, true) == ACCEL_ZERO_MOTOR_FAULT);
  float rpm_offset = 1.0f, heading = 315.0f;
  calibration::adjust(CONFIG_STAGE_RPM, 42, 8, 0.04f, 90.0f, 1, rpm_offset, heading);
  assert(near(rpm_offset, 1.0f));
  calibration::adjust(CONFIG_STAGE_RPM, 0, 8, 0.04f, 90.0f, 1, rpm_offset, heading);
  assert(near(rpm_offset, 1.002f) && heading == 315.0f);
  calibration::adjust(CONFIG_STAGE_HEADING, 0, 8, 0.04f, 90.0f, 1, rpm_offset, heading);
  assert(near(heading, 319.5f) && near(rpm_offset, 1.002f));
}

int main()
{
  test_controller();
  test_estimator();
  test_calibration();
  std::puts("PASS: real controller, estimator and calibration modules; fault rearm, demand math, filtering, lifecycle, zeroing and gestures");
}
