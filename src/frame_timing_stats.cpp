#include "frame_timing_stats.h"

namespace {
constexpr uint8_t kFrameWindowSize = 10;

uint32_t s_prev_now_us = 0;
bool s_prev_initialized = false;
uint32_t s_min_frame_us = 0;
uint32_t s_max_frame_us = 0;
uint32_t s_recent_frames_us[kFrameWindowSize] = {0};
uint8_t s_recent_frame_count = 0;
uint8_t s_recent_frame_next = 0;

void push_recent_frame(uint32_t frame_us) {
  s_recent_frames_us[s_recent_frame_next] = frame_us;
  s_recent_frame_next = (uint8_t)((s_recent_frame_next + 1) % kFrameWindowSize);
  if (s_recent_frame_count < kFrameWindowSize) {
    ++s_recent_frame_count;
  }
}
}  // namespace

void frame_timing_on_loop(uint32_t now_us) {
  if (!s_prev_initialized) {
    s_prev_now_us = now_us;
    s_prev_initialized = true;
    return;
  }

  const uint32_t frame_us = now_us - s_prev_now_us;
  s_prev_now_us = now_us;

  if (s_min_frame_us == 0 || frame_us < s_min_frame_us) {
    s_min_frame_us = frame_us;
  }
  if (frame_us > s_max_frame_us) {
    s_max_frame_us = frame_us;
  }

  push_recent_frame(frame_us);
}

uint32_t frame_timing_min_us() {
  return s_min_frame_us;
}

uint32_t frame_timing_max_us() {
  return s_max_frame_us;
}

uint32_t frame_timing_avg_last_10_us() {
  if (s_recent_frame_count == 0) return 0;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < s_recent_frame_count; ++i) {
    sum += s_recent_frames_us[i];
  }

  return sum / s_recent_frame_count;
}
