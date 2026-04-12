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
void elevator_command_emergency_stop();
bool elevator_calibration_in_progress();
bool elevator_has_valid_calibration();
int32_t elevator_top_limit_steps();
int32_t elevator_top_margin_steps();
int32_t elevator_bottom_margin_steps();
bool elevator_is_homed_zero();
void elevator_stop();
void elevator_tick(uint32_t now_ms, Event* out_event);
EvState elevator_state();
int32_t elevator_floor();
int32_t elevator_target_floor();
int32_t elevator_current_position_steps();
int32_t elevator_target_position_steps();
int32_t elevator_distance_to_go_steps();
bool elevator_is_moving();
uint32_t elevator_motor_lag_count();
uint32_t elevator_motor_lag_last_interval_ms();
uint32_t elevator_motor_lag_max_interval_ms();
uint32_t elevator_motor_lag_accumulated_ms();
