#pragma once

#include <stdint.h>

static constexpr uint8_t RUNTIME_MODE_IR = 0x01;
static constexpr uint8_t RUNTIME_MODE_LED = 0x02;
static constexpr uint8_t RUNTIME_MODE_ELEVATOR = 0x04;
static constexpr uint8_t RUNTIME_MODE_SCENE = 0x08;
static constexpr uint8_t RUNTIME_MODE_ALL =
    RUNTIME_MODE_IR | RUNTIME_MODE_LED | RUNTIME_MODE_ELEVATOR | RUNTIME_MODE_SCENE;

uint8_t runtime_mode_get();
void runtime_mode_set(uint8_t mode);
