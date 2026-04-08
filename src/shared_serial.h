#ifndef SHARED_SERIAL_H
#define SHARED_SERIAL_H

#include <Arduino.h>

struct SharedSerialCursor {
  uint16_t next_seq;
};

void shared_serial_setup();
void shared_serial_pump();
void shared_serial_cursor_init(SharedSerialCursor* cursor);
bool shared_serial_read_line(SharedSerialCursor* cursor, char* out_line, size_t out_size);

#endif
