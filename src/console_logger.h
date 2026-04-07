#ifndef CONSOLE_LOGGER_H
#define CONSOLE_LOGGER_H

#include <Arduino.h>

#include "config.h"
#include "events.h"

class ConsoleLogger {
 public:
  explicit ConsoleLogger(Stream& out);

  void print_banner();
  void print_mode(uint8_t mode);
  void print_mode_change(uint8_t before, uint8_t after);
  void print_serial_echo(char c);
  void print_mode_usage();
  void print_mode_cmd_too_long();
  void print_startup(uint8_t mode);

#if SSC_IR_LOG_ENABLE
  void print_ir_event(const Event& event);
#endif

 private:
  Stream& out_;
};

#endif
