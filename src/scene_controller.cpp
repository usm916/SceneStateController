#include "scene_controller.h"
#include "led_module.h"
#include "elevator_module.h"
#include "ir_module.h"
#include "pi_link.h"
#include "pi_protocol.h"

static SceneId s_scene = SCENE_IDLE;
static int32_t s_target_floor = 0;

SceneId scene_current() { return s_scene; }

static void set_scene(SceneId s) {
  if (s_scene == s) return;
  s_scene = s;
  ssc_send_scene(Serial, scene_name(s_scene));

  switch (s_scene) {
    case SCENE_IDLE:    led_set_pattern(LEDP_IDLE);    break;
    case SCENE_MOVE:    led_set_pattern(LEDP_MOVING);  break;
    case SCENE_ARRIVED: led_set_pattern(LEDP_ARRIVED); break;
    case SCENE_ERROR:   led_set_pattern(LEDP_ERROR);   break;
    default: break;
  }
}

void scene_setup() {
  s_scene = SCENE_IDLE;
  s_target_floor = 0;
  set_scene(SCENE_IDLE);
}

void scene_handle_event(const Event& e) {
  pi_link_send_event(e);

  switch (s_scene) {
    case SCENE_IDLE:
      if (e.type == EVT_PI_CMD_MOVE) {
        s_target_floor = e.data.move.target_floor;
        elevator_command_move_to(s_target_floor);
        set_scene(SCENE_MOVE);
      }
      break;

    case SCENE_MOVE:
      if (e.type == EVT_EV_ARRIVED) set_scene(SCENE_ARRIVED);
      if (e.type == EVT_EV_ERROR)   set_scene(SCENE_ERROR);
      break;

    case SCENE_ARRIVED:
      break;

    case SCENE_ERROR:
      if (e.type == EVT_PI_CMD_MOVE) {
        s_target_floor = e.data.move.target_floor;
        elevator_command_move_to(s_target_floor);
        set_scene(SCENE_MOVE);
      }
      break;

    default:
      break;
  }
}

void scene_tick(uint32_t now_ms) {
  static uint32_t s_arrived_ms = 0;

  if (s_scene == SCENE_ARRIVED) {
    if (s_arrived_ms == 0) s_arrived_ms = now_ms;
    if (now_ms - s_arrived_ms > 3000) {
      s_arrived_ms = 0;
      set_scene(SCENE_IDLE);
    }
  } else {
    s_arrived_ms = 0;
  }
}
