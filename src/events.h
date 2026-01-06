#pragma once
#include <stdint.h>

enum EventType : uint8_t {
  EVT_NONE = 0,
  EVT_IR_BUTTON,
  EVT_PI_CMD_MOVE,
  EVT_PI_CMD_LED,
  EVT_EV_ARRIVED,
  EVT_EV_ERROR,
  EVT_TIMEOUT,
};

struct IrEvent {
  uint8_t protocol;
  uint16_t addr;
  uint16_t cmd;
};

struct PiMoveCmd {
  int32_t target_floor;
};

struct PiLedCmd {
  uint8_t pattern_id;
};

struct Event {
  EventType type;
  uint32_t ts_ms;
  union {
    IrEvent ir;
    PiMoveCmd move;
    PiLedCmd led;
    int32_t error_code;
  } data;
};
