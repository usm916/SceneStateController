#pragma once
#include <stdint.h>
#include <Arduino.h>

static inline void ssc_send_scene(Print& io, const char* name) {
  io.print("SCENE "); io.println(name);
}
static inline void ssc_send_ev(Print& io, const char* name) {
  io.print("EV "); io.println(name);
}
static inline void ssc_send_ir(Print& io, uint8_t proto, uint16_t addr, uint16_t cmd) {
  io.print("IR ");
  io.print(proto);
  io.print(" 0x"); io.print(addr, HEX);
  io.print(" 0x"); io.println(cmd, HEX);
}
static inline void ssc_send_err(Print& io, int32_t code) {
  io.print("ERR "); io.println(code);
}
