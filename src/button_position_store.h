#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "ir_module.h"

void button_position_store_setup();
bool button_position_store_load();
bool button_position_store_save();

void button_position_store_set_zero(int32_t current_steps);
int32_t button_position_store_zero_steps();

bool button_position_store_parse_button_token(const char* token, uint8_t* out_index);
bool button_position_store_index_from_remote(RemoteButton button, uint8_t* out_index);

bool button_position_store_record_relative(uint8_t index, int32_t relative_steps);
bool button_position_store_record_current(uint8_t index, int32_t current_steps);
bool button_position_store_has(uint8_t index);
bool button_position_store_relative(uint8_t index, int32_t* out_relative_steps);
bool button_position_store_target(uint8_t index, int32_t* out_target_steps);

void button_position_store_print_info(Stream& out);
