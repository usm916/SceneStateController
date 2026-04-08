#include "elevator_module.h"
#include "config.h"
#include "tmc2209_module.h"
#include <Arduino.h>

static volatile int32_t s_step_count = 0;
static volatile bool s_step_high = false;
static volatile bool s_running = false;

static int32_t s_target_steps = 0;
static int32_t s_current_floor = 0;
static EvState s_state = EV_IDLE;
static uint32_t s_move_start_ms = 0;
static uint32_t s_last_check_ms = 0;
static bool s_startup_spin_scheduled = true;
static bool s_startup_spin_active = false;

static hw_timer_t* s_timer = nullptr;
static portMUX_TYPE s_timerMux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint16_t kMotorFullStepsPerRev = 200;
static constexpr uint8_t kMotorMicrosteps = 16;
static constexpr uint16_t kStartupSpinRpm = 100;
static constexpr uint32_t kStartupSpinDurationMs = 15000;

static void ARDUINO_ISR_ATTR on_timer() {
  portENTER_CRITICAL_ISR(&s_timerMux);
  if (!s_running) {
    if (s_step_high) {
      digitalWrite(SSC_PIN_STEP, LOW);
      s_step_high = false;
    }
    portEXIT_CRITICAL_ISR(&s_timerMux);
    return;
  }

  if (!s_step_high) {
    digitalWrite(SSC_PIN_STEP, HIGH);
    s_step_high = true;
  } else {
    digitalWrite(SSC_PIN_STEP, LOW);
    s_step_high = false;
    s_step_count++;
    if (s_step_count >= s_target_steps) {
      s_running = false;
    }
  }
  portEXIT_CRITICAL_ISR(&s_timerMux);
}

static bool endstop_hit_up() {
  return digitalRead(SSC_PIN_ENDSTOP_UP) == LOW;
}
static bool endstop_hit_down() {
  return digitalRead(SSC_PIN_ENDSTOP_DOWN) == LOW;
}

void elevator_setup() {
  pinMode(SSC_PIN_STEP, OUTPUT);
  pinMode(SSC_PIN_DIR, OUTPUT);
  digitalWrite(SSC_PIN_STEP, LOW);
  digitalWrite(SSC_PIN_DIR, LOW);

  pinMode(SSC_PIN_ENDSTOP_UP, INPUT_PULLUP);
  pinMode(SSC_PIN_ENDSTOP_DOWN, INPUT_PULLUP);

  Serial.println("Setting up into TMC2209...");

  tmc2209_setup();
  tmc2209_set_enable(true);
  Serial.println("finishing setup into TMC2209...");
  s_timer = timerBegin(1000000); // 1 MHz timer frequency (ESP32 core 3.x)
  timerAttachInterrupt(s_timer, &on_timer);
  timerAlarm(s_timer, 1000, true, 0); // 1 tick = 1 us at 1 MHz

  s_state = EV_IDLE;
  s_startup_spin_scheduled = true;
  s_startup_spin_active = false;
}

EvState elevator_state() { return s_state; }
int32_t elevator_floor() { return s_current_floor; }

void elevator_stop() {
  portENTER_CRITICAL(&s_timerMux);
  s_running = false;
  s_step_count = 0;
  s_target_steps = 0;
  portEXIT_CRITICAL(&s_timerMux);
  s_state = EV_IDLE;
}

void elevator_command_move_to(int32_t target_floor) {
  if (target_floor == s_current_floor) {
    s_state = EV_ARRIVED;
    return;
  }

  const bool up = (target_floor > s_current_floor);
  digitalWrite(SSC_PIN_DIR, up ? HIGH : LOW);

  const int32_t floors = abs(target_floor - s_current_floor);
  const int32_t steps_total = floors * (int32_t)SSC_STEPS_PER_FLOOR;

  const uint32_t step_hz = SSC_STEP_HZ_DEFAULT;
  const uint32_t alarm_us = (uint32_t)(1000000UL / (step_hz * 2UL));
  timerAlarm(s_timer, alarm_us, true, 0);

  portENTER_CRITICAL(&s_timerMux);
  s_step_count = 0;
  s_target_steps = steps_total;
  s_running = true;
  portEXIT_CRITICAL(&s_timerMux);

  s_move_start_ms = millis();
  s_last_check_ms = s_move_start_ms;
  s_state = up ? EV_MOVING_UP : EV_MOVING_DOWN;
}

void elevator_tick(uint32_t now_ms, Event* out_event) {
  if (out_event) out_event->type = EVT_NONE;

  if (s_startup_spin_scheduled) {
    const uint32_t step_hz = ((uint32_t)kStartupSpinRpm * kMotorFullStepsPerRev * kMotorMicrosteps) / 60UL;
    const uint32_t alarm_us = (uint32_t)(1000000UL / (step_hz * 2UL));
    const uint32_t target_steps = (step_hz * kStartupSpinDurationMs) / 1000UL;

    digitalWrite(SSC_PIN_DIR, LOW);
    timerAlarm(s_timer, alarm_us, true, 0);

    portENTER_CRITICAL(&s_timerMux);
    s_step_count = 0;
    s_target_steps = (int32_t)target_steps;
    s_running = true;
    portEXIT_CRITICAL(&s_timerMux);

    s_startup_spin_scheduled = false;
    s_startup_spin_active = true;
    s_move_start_ms = now_ms;
    s_last_check_ms = now_ms;
    return;
  }

  if (s_startup_spin_active) {
    bool running;
    portENTER_CRITICAL(&s_timerMux);
    running = s_running;
    portEXIT_CRITICAL(&s_timerMux);
    if (!running) {
      s_startup_spin_active = false;
      s_state = EV_IDLE;
    }
    return;
  }

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

  const uint32_t timeout_ms = 20000;
  if ((s_state == EV_MOVING_UP || s_state == EV_MOVING_DOWN) && (now_ms - s_move_start_ms > timeout_ms)) {
    elevator_stop();
    s_state = EV_ERROR;
    if (out_event) {
      out_event->type = EVT_EV_ERROR;
      out_event->ts_ms = now_ms;
      out_event->data.error_code = 1003;
    }
    return;
  }

  bool running;
  int32_t step_count, target_steps;
  portENTER_CRITICAL(&s_timerMux);
  running = s_running;
  step_count = s_step_count;
  target_steps = s_target_steps;
  portEXIT_CRITICAL(&s_timerMux);

  if (!running && (s_state == EV_MOVING_UP || s_state == EV_MOVING_DOWN) && target_steps > 0 && step_count >= target_steps) {
    if (s_state == EV_MOVING_UP) s_current_floor += (target_steps / (int32_t)SSC_STEPS_PER_FLOOR);
    if (s_state == EV_MOVING_DOWN) s_current_floor -= (target_steps / (int32_t)SSC_STEPS_PER_FLOOR);

    s_state = EV_ARRIVED;
    if (out_event) {
      out_event->type = EVT_EV_ARRIVED;
      out_event->ts_ms = now_ms;
    }
  }
}
