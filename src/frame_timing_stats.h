#ifndef FRAME_TIMING_STATS_H
#define FRAME_TIMING_STATS_H

#include <stdint.h>

void frame_timing_on_loop(uint32_t now_us);
uint32_t frame_timing_min_us();
uint32_t frame_timing_max_us();
uint8_t frame_timing_recent_count();
uint32_t frame_timing_recent_us(uint8_t index_from_oldest);

#endif
