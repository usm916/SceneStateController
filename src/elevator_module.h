#pragma once
#include <stdint.h>
#include "scene_state.h"
#include "events.h"

void elevator_setup();
void handleSerialInput();
void handleInput(int32_t target_steps);
void elevator_command_move_to(int32_t target_floor);
void elevator_command_spin_cw(uint16_t rpm);
void elevator_command_spin_ccw(uint16_t rpm);
void elevator_command_home_zero();
void elevator_command_home_top();
void elevator_command_start_calibration();
bool elevator_calibration_in_progress();
bool elevator_has_valid_calibration();
int32_t elevator_top_limit_steps();
void elevator_stop();
void elevator_tick(uint32_t now_ms, Event* out_event);
EvState elevator_state();
int32_t elevator_floor();
