#pragma once
#include "events.h"
#include <Arduino.h>

void pi_link_setup();
bool pi_link_poll(Event& out);
void pi_link_send_event(const Event& e);
