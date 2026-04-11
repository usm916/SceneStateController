#include "elevator_module.h"

#include "config.h"
#include "shared_serial.h"

#include <AccelStepper.h>
#include <Arduino.h>
#include <Preferences.h>
#include <TMCStepper.h>

namespace {
constexpr uint16_t kMotorFullStepsPerRev = 200;
constexpr uint8_t kMotorMicrosteps = 16;

constexpr float kMoveMaxSpeedDefault = 7500.0f;
constexpr float kMoveAccelerationDefault = 1300.0f;
constexpr int32_t kSpinHorizonSteps = 32768;
constexpr int32_t kMaxTravelSteps = 8 * (int32_t)SSC_STEPS_PER_FLOOR;

constexpr float kHomingFastSpeed = 4200.0f;
constexpr float kHomingSlowSpeed = 900.0f;

constexpr char kPrefsNs[] = "ev_calib";
constexpr char kPrefsKeyVersion[] = "ver";
constexpr char kPrefsKeyTopStep[] = "top_step";
constexpr char kPrefsKeyTopMargin[] = "top_margin";
constexpr char kPrefsKeyBottomMargin[] = "btm_margin";
constexpr uint32_t kCalibVersion = 2;

HardwareSerial& s_tmc_serial = Serial1;
TMC2209Stepper s_driver(&s_tmc_serial, SSC_TMC_RSENSE_OHM, 0);
AccelStepper s_stepper(AccelStepper::DRIVER, SSC_PIN_STEP, SSC_PIN_DIR);
Preferences s_prefs;

String s_input_line = "";

int32_t s_current_floor = 0;
int32_t s_target_floor = 0;
EvState s_state = EV_IDLE;

bool s_move_active = false;
bool s_spin_mode = false;
bool s_position_mode = false;
int8_t s_spin_dir = 0;  // 1:cw, -1:ccw

bool s_is_homed_zero = false;
bool s_calibration_armed = false;
bool s_has_calibration = false;
int32_t s_top_limit_steps = (int32_t)SSC_STEPS_PER_FLOOR;
int32_t s_top_margin_steps = 0;
int32_t s_bottom_margin_steps = 0;

uint32_t s_move_start_ms = 0;
uint32_t s_last_check_ms = 0;
uint32_t s_move_timeout_ms = 20000;

struct HomingSession {
  bool active = false;
  bool toward_top = false;
  bool capture_for_calibration = false;
  uint8_t phase = 0;  // 1 seek, 2 slow release-seek, 3 margin settle
  int8_t dir = 0;
  int8_t backoff_dir = 0;
  int32_t phase_start_pos = 0;
  int32_t hit_pos = 0;
  int32_t release_pos = 0;
  int32_t release_steps = 0;
  int32_t margin_steps = 0;
};

HomingSession s_homing;

float rpm_to_steps_per_sec(uint16_t rpm) {
  return ((float)rpm * (float)kMotorFullStepsPerRev * (float)kMotorMicrosteps) / 60.0f;
}

bool endstop_hit_up_no_pullup() {
  return digitalRead(SSC_PIN_ENDSTOP_UP) == HIGH;
}

bool endstop_hit_down_no_pullup() {
  return digitalRead(SSC_PIN_ENDSTOP_DOWN) == HIGH;
}

bool endstop_hit_up() {
#if SSC_ENDSTOP_USE_INPUT_PULLUP
  return digitalRead(SSC_PIN_ENDSTOP_UP) == LOW;
#else
  return endstop_hit_up_no_pullup();
#endif
}

bool endstop_hit_down() {
#if SSC_ENDSTOP_USE_INPUT_PULLUP
  return digitalRead(SSC_PIN_ENDSTOP_DOWN) == LOW;
#else
  return endstop_hit_down_no_pullup();
#endif
}

bool active_homing_switch_hit() {
  return s_homing.toward_top ? endstop_hit_up() : endstop_hit_down();
}

void zero_at_bottom_endstop() {
  s_stepper.setCurrentPosition(0);
  s_current_floor = 0;
  s_target_floor = 0;
  s_is_homed_zero = true;
}

void begin_spin(int8_t dir, uint16_t rpm) {
  const float spin_speed = rpm_to_steps_per_sec(rpm);
  if (spin_speed <= 0.0f) return;

  if (s_spin_mode && s_spin_dir == dir && !s_homing.active) {
    s_stepper.setMaxSpeed(spin_speed);
    return;
  }

  s_spin_mode = true;
  s_position_mode = false;
  s_spin_dir = dir;
  s_move_active = true;
  s_homing.active = false;

  s_stepper.setAcceleration(kMoveAccelerationDefault);
  s_stepper.setMaxSpeed(spin_speed);

  const int32_t now_pos = s_stepper.currentPosition();
  const int32_t horizon = (dir > 0) ? kSpinHorizonSteps : -kSpinHorizonSteps;
  s_stepper.moveTo(now_pos + horizon);

  s_move_start_ms = millis();
  s_last_check_ms = s_move_start_ms;
  s_state = (dir > 0) ? EV_MOVING_UP : EV_MOVING_DOWN;
}

void keep_spin_target_ahead() {
  if (!s_spin_mode || s_spin_dir == 0) return;

  const int32_t dist = s_stepper.distanceToGo();
  if ((s_spin_dir > 0 && dist < (kSpinHorizonSteps / 2)) ||
      (s_spin_dir < 0 && dist > -(kSpinHorizonSteps / 2))) {
    const int32_t extension = (s_spin_dir > 0) ? kSpinHorizonSteps : -kSpinHorizonSteps;
    s_stepper.moveTo(s_stepper.targetPosition() + extension);
  }
}

bool save_calibration() {
  if (s_top_limit_steps <= 0) return false;
  if (!s_prefs.begin(kPrefsNs, false)) return false;
  s_prefs.putUInt(kPrefsKeyVersion, kCalibVersion);
  s_prefs.putInt(kPrefsKeyTopStep, s_top_limit_steps);
  s_prefs.putInt(kPrefsKeyTopMargin, s_top_margin_steps);
  s_prefs.putInt(kPrefsKeyBottomMargin, s_bottom_margin_steps);
  s_prefs.end();
  return true;
}

void load_calibration() {
  s_has_calibration = false;
  if (!s_prefs.begin(kPrefsNs, true)) return;
  const uint32_t version = s_prefs.getUInt(kPrefsKeyVersion, 0);
  const int32_t top_step = s_prefs.getInt(kPrefsKeyTopStep, 0);
  const int32_t top_margin = s_prefs.getInt(kPrefsKeyTopMargin, 0);
  const int32_t bottom_margin = s_prefs.getInt(kPrefsKeyBottomMargin, 0);
  s_prefs.end();

  if ((version == 1 || version == kCalibVersion) && top_step > 0) {
    s_top_limit_steps = top_step;
    if (version == kCalibVersion) {
      s_top_margin_steps = (top_margin > 0) ? top_margin : 0;
      s_bottom_margin_steps = (bottom_margin > 0) ? bottom_margin : 0;
    } else {
      s_top_margin_steps = 0;
      s_bottom_margin_steps = 0;
    }
    s_has_calibration = true;
  }
}

void configure_motion_defaults() {
  s_stepper.setMaxSpeed(kMoveMaxSpeedDefault);
  s_stepper.setAcceleration(kMoveAccelerationDefault);
}

void begin_homing(bool toward_top, bool capture_for_calibration) {
  s_homing = {};
  s_homing.active = true;
  s_homing.toward_top = toward_top;
  s_homing.capture_for_calibration = capture_for_calibration;
  s_homing.phase = 1;
  s_homing.dir = toward_top ? 1 : -1;
  s_homing.backoff_dir = -s_homing.dir;
  s_homing.phase_start_pos = s_stepper.currentPosition();

  s_spin_mode = false;
  s_position_mode = false;
  s_spin_dir = 0;
  s_move_active = true;

  s_stepper.setAcceleration(kMoveAccelerationDefault);
  s_stepper.setMaxSpeed(kHomingFastSpeed);
  s_stepper.moveTo(s_homing.phase_start_pos + (int32_t)s_homing.dir * kMaxTravelSteps);

  s_move_start_ms = millis();
  s_last_check_ms = s_move_start_ms;
  if (s_calibration_armed && !toward_top) {
    s_state = EV_CALIBRATING;
  } else {
    s_state = toward_top ? EV_HOMING_TOP : EV_HOMING_ZERO;
  }
}

void finish_homing_at_margin_reference(uint32_t now_ms, Event* out_event) {
  if (s_homing.toward_top) {
    s_top_margin_steps = s_homing.margin_steps;
    if (s_homing.capture_for_calibration && s_calibration_armed && s_is_homed_zero) {
      s_top_limit_steps = s_stepper.currentPosition();
      s_has_calibration = save_calibration();
    }
    s_state = EV_IDLE;
  } else {
    s_bottom_margin_steps = s_homing.margin_steps;
    zero_at_bottom_endstop();
    s_state = s_calibration_armed ? EV_CALIBRATING : EV_IDLE;
  }

  s_homing.active = false;
  s_move_active = false;
  configure_motion_defaults();
  s_stepper.stop();

  if (s_homing.capture_for_calibration && s_homing.toward_top) {
    s_calibration_armed = false;
  }

  if (out_event) {
    out_event->type = EVT_EV_ARRIVED;
    out_event->ts_ms = now_ms;
  }
}

void homing_fail(uint32_t now_ms, Event* out_event, int32_t error_code) {
  s_homing.active = false;
  s_move_active = false;
  s_calibration_armed = false;
  s_state = EV_ERROR;
  s_stepper.stop();
  if (out_event) {
    out_event->type = EVT_EV_ERROR;
    out_event->ts_ms = now_ms;
    out_event->data.error_code = error_code;
  }
}

void handle_homing_tick(uint32_t now_ms, Event* out_event) {
  if (!s_homing.active) return;

  const int32_t traveled = abs(s_stepper.currentPosition() - s_homing.phase_start_pos);
  if ((s_homing.phase == 1 || s_homing.phase == 3) && traveled > kMaxTravelSteps) {
    homing_fail(now_ms, out_event, s_homing.toward_top ? 1101 : 1102);
    return;
  }

  if (s_homing.phase == 1) {
    if (active_homing_switch_hit()) {
      s_homing.hit_pos = s_stepper.currentPosition();
      s_homing.phase = 2;
      s_homing.phase_start_pos = s_homing.hit_pos;
      s_stepper.setMaxSpeed(kHomingSlowSpeed);
      s_stepper.moveTo(s_homing.phase_start_pos + (int32_t)s_homing.backoff_dir * kMaxTravelSteps);
    }
    return;
  }

  if (s_homing.phase == 2) {
    if (!active_homing_switch_hit()) {
      s_homing.release_pos = s_stepper.currentPosition();
      s_homing.release_steps = abs(s_homing.release_pos - s_homing.hit_pos);
      if (s_homing.release_steps <= 0) s_homing.release_steps = 1;
      s_homing.margin_steps = s_homing.release_steps * 2;

      const int32_t extra_margin_steps = s_homing.margin_steps - s_homing.release_steps;
      if (extra_margin_steps > 0) {
        s_homing.phase = 3;
        s_homing.phase_start_pos = s_homing.release_pos;
        s_stepper.setMaxSpeed(kHomingSlowSpeed);
        s_stepper.moveTo(s_homing.phase_start_pos + (int32_t)s_homing.backoff_dir * extra_margin_steps);
      } else {
        finish_homing_at_margin_reference(now_ms, out_event);
      }
    }
    return;
  }

  if (s_homing.phase == 3 && !s_stepper.isRunning()) {
    finish_homing_at_margin_reference(now_ms, out_event);
  }
}

void emergency_stop_with_error(uint32_t now_ms, Event* out_event, int32_t error_code) {
  elevator_stop();
  s_move_active = false;
  s_homing.active = false;
  s_state = EV_ERROR;
  if (out_event) {
    out_event->type = EVT_EV_ERROR;
    out_event->ts_ms = now_ms;
    out_event->data.error_code = error_code;
  }
}
}  // namespace

void elevator_setup() {
  pinMode(SSC_PIN_STEP, OUTPUT);
  pinMode(SSC_PIN_DIR, OUTPUT);
  digitalWrite(SSC_PIN_STEP, LOW);
  digitalWrite(SSC_PIN_DIR, LOW);

#if SSC_ENDSTOP_USE_INPUT_PULLUP
  pinMode(SSC_PIN_ENDSTOP_UP, INPUT_PULLUP);
  pinMode(SSC_PIN_ENDSTOP_DOWN, INPUT_PULLUP);
#else
  pinMode(SSC_PIN_ENDSTOP_UP, INPUT);
  pinMode(SSC_PIN_ENDSTOP_DOWN, INPUT);
#endif

  s_tmc_serial.begin(SSC_TMC_UART_BAUD, SERIAL_8N1, SSC_TMC_UART_RX_PIN, SSC_TMC_UART_TX_PIN);

  pinMode(SSC_PIN_EN, OUTPUT);
  digitalWrite(SSC_PIN_EN, LOW);

  s_driver.begin();
  s_driver.pdn_disable(true);
  s_driver.I_scale_analog(false);
  s_driver.toff(SSC_TMC_TOFF);
  s_driver.blank_time(SSC_TMC_BLANK_TIME);
  s_driver.rms_current(SSC_TMC_RMS_CURRENT_MA, SSC_TMC_RMS_HOLD_MULT);
  s_driver.microsteps(SSC_TMC_MICROSTEPS);
  s_driver.en_spreadCycle(SSC_TMC_ENABLE_SPREADCYCLE != 0);
  s_driver.TPWMTHRS(SSC_TMC_TPWMTHRS);

  configure_motion_defaults();
  s_stepper.setCurrentPosition(0);

  load_calibration();

  s_state = EV_IDLE;
  s_move_active = false;
  s_spin_mode = false;
  s_position_mode = false;
  s_homing = {};
}

EvState elevator_state() { return s_state; }

int32_t elevator_floor() { return s_current_floor; }

bool elevator_calibration_in_progress() { return s_calibration_armed || s_state == EV_CALIBRATING; }

bool elevator_has_valid_calibration() { return s_has_calibration; }

int32_t elevator_top_limit_steps() { return s_top_limit_steps; }
int32_t elevator_top_margin_steps() { return s_top_margin_steps; }
int32_t elevator_bottom_margin_steps() { return s_bottom_margin_steps; }
bool elevator_is_homed_zero() { return s_is_homed_zero; }
int32_t elevator_target_floor() { return s_target_floor; }
int32_t elevator_current_position_steps() { return s_stepper.currentPosition(); }
int32_t elevator_target_position_steps() { return s_stepper.targetPosition(); }
int32_t elevator_distance_to_go_steps() { return s_stepper.distanceToGo(); }
bool elevator_is_moving() { return s_move_active; }

void elevator_stop() {
  s_spin_mode = false;
  s_position_mode = false;
  s_spin_dir = 0;
  s_homing.active = false;

  if (s_move_active) {
    s_stepper.stop();
  }
}

void handleInput(int32_t target_steps) {
  if (s_has_calibration) {
    if (target_steps < 0) target_steps = 0;
    if (target_steps > s_top_limit_steps) target_steps = s_top_limit_steps;
  }

  s_spin_mode = false;
  s_position_mode = true;
  s_spin_dir = 0;
  s_homing.active = false;

  configure_motion_defaults();
  s_stepper.moveTo(target_steps);

  s_target_floor = target_steps / (int32_t)SSC_STEPS_PER_FLOOR;
  s_move_active = true;
  s_move_start_ms = millis();
  s_last_check_ms = s_move_start_ms;
  const int32_t travel_steps = abs(s_stepper.distanceToGo());
  const float max_speed = s_stepper.maxSpeed();
  const uint32_t dynamic_timeout_ms =
      (max_speed > 1.0f) ? (uint32_t)((1000.0f * (float)travel_steps) / max_speed) + 5000 : 20000;
  s_move_timeout_ms = (dynamic_timeout_ms < 20000) ? 20000 : dynamic_timeout_ms;
  s_state = (target_steps >= s_stepper.currentPosition()) ? EV_MOVING_UP : EV_MOVING_DOWN;

  Serial.print("New target: ");
  Serial.println(target_steps);
  Serial.print("Current position: ");
  Serial.println(s_stepper.currentPosition());
  Serial.print("Distance to go: ");
  Serial.println(s_stepper.distanceToGo());
}

void handleSerialInput() {
  static SharedSerialCursor serial_cursor = {0};
  static bool serial_cursor_initialized = false;
  static char line[32] = {0};

  if (!serial_cursor_initialized) {
    shared_serial_cursor_init(&serial_cursor);
    serial_cursor_initialized = true;
  }

  while (shared_serial_read_line(&serial_cursor, line, sizeof(line))) {
    s_input_line = line;
    s_input_line.trim();
    if (s_input_line.length() > 0) {
      if (s_input_line[0] == 'e' || s_input_line[0] == 'E') {
        const long target_steps = s_input_line.substring(1).toInt();
        handleInput((int32_t)target_steps);
      }
    }
    s_input_line = "";
  }
}

void elevator_command_move_to(int32_t target_floor) {
  if (target_floor == s_current_floor) {
    s_state = EV_ARRIVED;
    return;
  }
  s_target_floor = target_floor;
  handleInput(target_floor * (int32_t)SSC_STEPS_PER_FLOOR);
}

void elevator_command_spin_cw(uint16_t rpm) {
  begin_spin(1, rpm);
}

void elevator_command_spin_ccw(uint16_t rpm) {
  begin_spin(-1, rpm);
}

void elevator_command_home_zero() {
  begin_homing(false, false);
}

void elevator_command_home_top() {
  const bool capture = s_calibration_armed;
  begin_homing(true, capture);
}

void elevator_command_start_calibration() {
  s_calibration_armed = true;
  begin_homing(false, false);
}

void elevator_command_emergency_stop() {
  elevator_stop();
  s_move_active = false;
  s_homing.active = false;
  s_calibration_armed = false;
  s_state = EV_ERROR;
}

void elevator_tick(uint32_t now_ms, Event* out_event) {
  if (out_event) out_event->type = EVT_NONE;

  keep_spin_target_ahead();
  s_stepper.run();

  if (now_ms - s_last_check_ms < 20) return;
  s_last_check_ms = now_ms;

  handle_homing_tick(now_ms, out_event);
  if (s_homing.active) return;
  if (s_state == EV_ERROR) return;

  if (!s_spin_mode && !s_homing.active && endstop_hit_up() && s_stepper.distanceToGo() > 0) {
    emergency_stop_with_error(now_ms, out_event, 1001);
    return;
  }
  if (!s_spin_mode && !s_homing.active && endstop_hit_down() && s_stepper.distanceToGo() < 0) {
    zero_at_bottom_endstop();
    emergency_stop_with_error(now_ms, out_event, 1002);
    return;
  }

  if (s_move_active && s_position_mode && (now_ms - s_move_start_ms > s_move_timeout_ms)) {
    elevator_stop();
    s_state = EV_ERROR;
    if (out_event) {
      out_event->type = EVT_EV_ERROR;
      out_event->ts_ms = now_ms;
      out_event->data.error_code = 1003;
    }
    return;
  }

  if (!s_stepper.isRunning()) {
    if (s_move_active) {
      s_move_active = false;
      if (s_position_mode && (s_state == EV_MOVING_UP || s_state == EV_MOVING_DOWN)) {
        s_current_floor = s_stepper.currentPosition() / (int32_t)SSC_STEPS_PER_FLOOR;
        s_state = EV_ARRIVED;
        if (out_event) {
          out_event->type = EVT_EV_ARRIVED;
          out_event->ts_ms = now_ms;
        }
      } else {
        s_state = EV_IDLE;
      }
    }

    if (s_state == EV_ARRIVED && s_position_mode) {
      s_stepper.setCurrentPosition((int32_t)s_current_floor * (int32_t)SSC_STEPS_PER_FLOOR);
    }
  }
}
