#pragma once
#include <stdint.h>
#include "scene_state.h"
#include "events.h"

void scene_setup();
SceneId scene_current();
bool scene_select(SceneId s);
void scene_handle_event(const Event& e);
void scene_tick(uint32_t now_ms);
