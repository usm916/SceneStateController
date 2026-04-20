#include "frame_timing_stats.h"

namespace {
constexpr uint8_t kFrameWindowSize = 10;

uint32_t s_prev_now_ms = 0;
bool s_prev_initialized = false;
uint32_t s_min_frame_ms = 0;
uint32_t s_max_frame_ms = 0;
uint32_t s_recent_frames_ms[kFrameWindowSize] = {0};
uint8_t s_recent_frame_count = 0;
uint8_t s_recent_frame_next = 0;

void push_recent_frame(uint32_t frame_ms) {
  s_recent_frames_ms[s_recent_frame_next] = frame_ms;
  s_recent_frame_next = (uint8_t)((s_recent_frame_next + 1) % kFrameWindowSize);
  if (s_recent_frame_count < kFrameWindowSize) {
    ++s_recent_frame_count;
  }
}
}  // namespace

void frame_timing_on_loop(uint32_t now_ms) {
  if (!s_prev_initialized) {
    s_prev_now_ms = now_ms;
    s_prev_initialized = true;
    return;
  }

  const uint32_t frame_ms = now_ms - s_prev_now_ms;
  s_prev_now_ms = now_ms;

  if (s_min_frame_ms == 0 || frame_ms < s_min_frame_ms) {
    s_min_frame_ms = frame_ms;
  }
  if (frame_ms > s_max_frame_ms) {
    s_max_frame_ms = frame_ms;
  }

  push_recent_frame(frame_ms);
}

uint32_t frame_timing_min_ms() {
  return s_min_frame_ms;
}

uint32_t frame_timing_max_ms() {
  return s_max_frame_ms;
}

uint32_t frame_timing_avg_last_10_ms() {
  if (s_recent_frame_count == 0) return 0;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < s_recent_frame_count; ++i) {
    sum += s_recent_frames_ms[i];
  }

  return sum / s_recent_frame_count;
}
