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

uint8_t frame_timing_recent_count() {
  return s_recent_frame_count;
}

uint32_t frame_timing_recent_us(uint8_t index_from_oldest) {
  if (index_from_oldest >= s_recent_frame_count) return 0;

  const uint8_t oldest_index =
      (uint8_t)((s_recent_frame_next + kFrameWindowSize - s_recent_frame_count) % kFrameWindowSize);
  const uint8_t idx = (uint8_t)((oldest_index + index_from_oldest) % kFrameWindowSize);
  return s_recent_frames_us[idx];
}
