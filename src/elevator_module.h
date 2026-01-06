#pragma once
#include <stdint.h>
#include "scene_state.h"
#include "events.h"

void elevator_setup();
void elevator_command_move_to(int32_t target_floor);
void elevator_stop();
void elevator_tick(uint32_t now_ms, Event* out_event);
EvState elevator_state();
int32_t elevator_floor();
