#ifndef FRAME_TIMING_STATS_H
#define FRAME_TIMING_STATS_H

#include <stdint.h>

void frame_timing_on_loop(uint32_t now_us);
uint32_t frame_timing_min_us();
uint32_t frame_timing_max_us();
uint32_t frame_timing_avg_last_10_us();

#endif
