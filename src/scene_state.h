#pragma once
#include <stdint.h>

enum SceneId : uint8_t {
  SCENE_IDLE = 0,
  SCENE_MOVE,
  SCENE_ARRIVED,
  SCENE_ERROR,
};

enum EvState : uint8_t {
  EV_IDLE = 0,
  EV_MOVING_UP,
  EV_MOVING_DOWN,
  EV_HOMING_ZERO,
  EV_HOMING_TOP,
  EV_CALIBRATING,
  EV_ARRIVED,
  EV_ERROR,
};

static inline const char* scene_name(SceneId s) {
  switch (s) {
    case SCENE_IDLE: return "SCENE_IDLE";
    case SCENE_MOVE: return "SCENE_MOVE";
    case SCENE_ARRIVED: return "SCENE_ARRIVED";
    case SCENE_ERROR: return "SCENE_ERROR";
    default: return "SCENE_UNKNOWN";
  }
}

static inline const char* ev_state_name(EvState s) {
  switch (s) {
    case EV_IDLE: return "EV_IDLE";
    case EV_MOVING_UP: return "EV_MOVING_UP";
    case EV_MOVING_DOWN: return "EV_MOVING_DOWN";
    case EV_HOMING_ZERO: return "EV_HOMING_ZERO";
    case EV_HOMING_TOP: return "EV_HOMING_TOP";
    case EV_CALIBRATING: return "EV_CALIBRATING";
    case EV_ARRIVED: return "EV_ARRIVED";
    case EV_ERROR: return "EV_ERROR";
    default: return "EV_UNKNOWN";
  }
}
