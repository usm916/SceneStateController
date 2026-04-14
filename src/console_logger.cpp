#include "console_logger.h"

ConsoleLogger::ConsoleLogger(Stream& out) : out_(out) {}

void ConsoleLogger::print_banner() {
  out_.println();
  out_.println("===== SceneStateController (Arduino Prototype) =====");
  out_.print("Serial Monitor baud=");
  out_.println(SSC_USB_SERIAL_BAUD);
  out_.print("SSC_MODE=");
  out_.println(SSC_MODE);
  out_.println("Pi5 -> ESP32 commands:");
  out_.println("  MOVE <floor>");
  out_.println("  LED <pattern>  (0=IDLE 1=MOVING 2=ARRIVED 3=ERROR)");
  out_.println("====================================================");
}

void ConsoleLogger::print_mode_change(uint8_t before, uint8_t after) {
  out_.print("MODE CHANGED ");
  out_.print(before);
  out_.print(" -> ");
  out_.println(after);
}

void ConsoleLogger::print_serial_echo(char c) {
  out_.print("RX: ");
  if (c == '\r') {
    out_.println("\\r");
    return;
  }
  if (c == '\n') {
    out_.println("\\n");
    return;
  }
  out_.write((uint8_t)c);
  out_.println();
}

void ConsoleLogger::print_mode_usage() {
  out_.println("Type help to show this command list.");
  out_.println("MODE_CMD usage: s0..s4");
  out_.println("IR decode mode: m0..m3");
  out_.println("Elevator target (mode 3): e<steps>  (example: e3200)");
  out_.println("TMC usage: TMC RUN <mA> | TMC HOLD <0..100> | TMC INFO");
  out_.println("System info: INFO");
  out_.println("Preset cmds: mute | rec_<btn> | rec_<btn>_<steps> | save_pref");
  out_.println("Motion cmds: speed_<steps/s> | accel_<steps/s2>");
  out_.println("Current cmd: current_<1..2000>  (TMC run current mA)");
  out_.println("Manual prev/next max speed follows speed_<steps/s>");
}

void ConsoleLogger::print_mode_cmd_too_long() {
  out_.println("MODE_CMD too long");
}

void ConsoleLogger::print_startup(uint8_t mode) {
  out_.print("INITIAL MODE=");
  out_.println(mode);
  out_.println("Type s0..s4 + Enter to switch mode at runtime.");
  out_.println("Type m0..m3 + Enter to switch IR decode mode.");
  out_.println("In mode 3, type e<steps> + Enter to move elevator.");
  out_.println("READY");
}

#if SSC_IR_LOG_ENABLE
void ConsoleLogger::print_ir_event(const Event& event) {
  out_.print("IR RX protocol=");
  out_.print(event.data.ir.protocol);
  out_.print(" addr=0x");
  out_.print(event.data.ir.addr, HEX);
  out_.print(" cmd=0x");
  out_.println(event.data.ir.cmd, HEX);
}
#endif
