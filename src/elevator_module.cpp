#include "elevator_module.h"

#include "config.h"
#include "shared_serial.h"

#include <AccelStepper.h>
#include <Arduino.h>
#include <TMCStepper.h>

namespace {
constexpr uint16_t kMotorFullStepsPerRev = 200;
constexpr uint8_t kMotorMicrosteps = 16;
constexpr uint16_t kStartupSpinRpm = 100;
constexpr uint32_t kStartupSpinDurationMs = 2000;

constexpr float kMoveMaxSpeedDefault = 8400.0f;
constexpr float kMoveAccelerationDefault = 2000.0f;
constexpr int32_t kSpinHorizonSteps = 32768;

HardwareSerial& s_tmc_serial = Serial1;
TMC2209Stepper s_driver(&s_tmc_serial, SSC_TMC_RSENSE_OHM, 0);

AccelStepper s_stepper(AccelStepper::DRIVER, SSC_PIN_STEP, SSC_PIN_DIR);
String s_input_line = "";

int32_t s_current_floor = 0;
int32_t s_target_floor = 0;
EvState s_state = EV_IDLE;

bool s_move_active = false;
bool s_spin_mode = false;
bool s_position_mode = false;
int8_t s_spin_dir = 0;  // 1:cw, -1:ccw

bool s_startup_spin_scheduled = true;
bool s_startup_spin_active = false;
uint32_t s_startup_spin_until_ms = 0;

uint32_t s_move_start_ms = 0;
uint32_t s_last_check_ms = 0;
uint32_t s_move_timeout_ms = 20000;

float rpm_to_steps_per_sec(uint16_t rpm) {
  return ((float)rpm * (float)kMotorFullStepsPerRev * (float)kMotorMicrosteps) / 60.0f;
}

void begin_spin(int8_t dir, uint16_t rpm) {
  const float spin_speed = rpm_to_steps_per_sec(rpm);
  if (spin_speed <= 0.0f) return;

  s_spin_mode = true;
  s_position_mode = false;
  s_spin_dir = dir;
  s_move_active = true;

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

bool endstop_hit_up() {
  return digitalRead(SSC_PIN_ENDSTOP_UP) == LOW;
}

bool endstop_hit_down() {
  return digitalRead(SSC_PIN_ENDSTOP_DOWN) == LOW;
}
}  // namespace

void elevator_setup() {
  pinMode(SSC_PIN_STEP, OUTPUT);
  pinMode(SSC_PIN_DIR, OUTPUT);
  digitalWrite(SSC_PIN_STEP, LOW);
  digitalWrite(SSC_PIN_DIR, LOW);

  pinMode(SSC_PIN_ENDSTOP_UP, INPUT_PULLUP);
  pinMode(SSC_PIN_ENDSTOP_DOWN, INPUT_PULLUP);

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

  s_stepper.setMaxSpeed(kMoveMaxSpeedDefault);
  s_stepper.setAcceleration(kMoveAccelerationDefault);
  s_stepper.setCurrentPosition(0);

  s_state = EV_IDLE;
  s_startup_spin_scheduled = true;
  s_startup_spin_active = false;
}

EvState elevator_state() { return s_state; }

int32_t elevator_floor() { return s_current_floor; }

void elevator_stop() {
  s_spin_mode = false;
  s_position_mode = false;
  s_spin_dir = 0;

  if (s_move_active) {
    s_stepper.stop();
  }
}

void handleInput(int32_t target_steps) {
  s_startup_spin_scheduled = false;
  s_startup_spin_active = false;
  s_spin_mode = false;
  s_position_mode = true;
  s_spin_dir = 0;

  s_stepper.setMaxSpeed(kMoveMaxSpeedDefault);
  s_stepper.setAcceleration(kMoveAccelerationDefault);
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

void elevator_tick(uint32_t now_ms, Event* out_event) {
  if (out_event) out_event->type = EVT_NONE;

  if (s_startup_spin_scheduled) {
    begin_spin(1, kStartupSpinRpm);
    s_startup_spin_scheduled = false;
    s_startup_spin_active = true;
    s_startup_spin_until_ms = now_ms + kStartupSpinDurationMs;
  }

  if (s_startup_spin_active && now_ms >= s_startup_spin_until_ms) {
    elevator_stop();
    s_startup_spin_active = false;
  }

  keep_spin_target_ahead();
  s_stepper.run();

  if (now_ms - s_last_check_ms < 20) return;
  s_last_check_ms = now_ms;

  if (s_state == EV_MOVING_UP && endstop_hit_up()) {
    elevator_stop();
    s_state = EV_ERROR;
    if (out_event) {
      out_event->type = EVT_EV_ERROR;
      out_event->ts_ms = now_ms;
      out_event->data.error_code = 1001;
    }
    return;
  }
  if (s_state == EV_MOVING_DOWN && endstop_hit_down()) {
    elevator_stop();
    s_state = EV_ERROR;
    if (out_event) {
      out_event->type = EVT_EV_ERROR;
      out_event->ts_ms = now_ms;
      out_event->data.error_code = 1002;
    }
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
