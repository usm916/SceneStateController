#pragma once
#include <stddef.h>
#include <stdint.h>

#define SSC_PIN_IR            23
#define SSC_PIN_WS2812B_0     18
#define SSC_PIN_WS2812B_1     19
#define SSC_PIN_WS2812B_2     21
#define SSC_PIN_WS2812B_3     22
#define SSC_PIN_WS2812B_4     32
#define SSC_PIN_WS2812B_5     33

#define SSC_PIN_STEP          26
#define SSC_PIN_DIR           27
#define SSC_PIN_EN            14
// TMC2209 UART (added for UART switch)
#define SSC_TMC_UART_BAUD 115200
#define SSC_TMC_UART_ONEWIRE  0
#define SSC_TMC_UART_ONEWIRE_PIN  17
#define SSC_TMC_UART_RX_PIN  16
#define SSC_TMC_UART_TX_PIN  17

#define SSC_PIN_ENDSTOP_UP    25
#define SSC_PIN_ENDSTOP_DOWN  13

// Endstop input mode:
// 1: INPUT_PULLUP + active LOW switch (default)
// 0: INPUT (no pull-up) + active HIGH switch
#ifndef SSC_ENDSTOP_USE_INPUT_PULLUP
#define SSC_ENDSTOP_USE_INPUT_PULLUP 1
#endif

// #define SSC_TMC_UART_PORT     1
// #define SSC_TMC_UART_PIN      17
// #define SSC_TMC_UART_BAUD     115200

#define SSC_LED_STRIP_COUNT   6
#define SSC_LED_STRIP_LEN     72
#define SSC_LED_BRIGHTNESS    255
#ifndef SSC_LED_TARGET_FPS
#define SSC_LED_TARGET_FPS    60
#endif

#ifndef SSC_LED_ACTIVE_STRIP_COUNT
#define SSC_LED_ACTIVE_STRIP_COUNT 6
#endif

#define SSC_STEPS_PER_FLOOR   25600
#define SSC_STEP_HZ_DEFAULT   800
#define SSC_STEP_PULSE_US     2

#ifndef SSC_MODE
#define SSC_MODE 0
#endif

#ifndef SSC_IR_LOG_ENABLE
#define SSC_IR_LOG_ENABLE 1
#endif

#ifndef SSC_USB_SERIAL_BAUD
#define SSC_USB_SERIAL_BAUD 115200
#endif

// ESP-NOW link:
// - ENABLE=1 on both devices
// - ROLE is selected from Web UI and stored in Preferences
// - PEER_MAC should be the STA MAC of the other device.
#ifndef SSC_ESPNOW_LINK_ENABLE
#define SSC_ESPNOW_LINK_ENABLE 1
#endif

#ifndef SSC_ESPNOW_LINK_CHANNEL
#define SSC_ESPNOW_LINK_CHANNEL 1
#endif

#ifndef SSC_ESPNOW_LINK_PEER_MAC
#define SSC_ESPNOW_LINK_PEER_MAC {0x24, 0x6F, 0x28, 0x00, 0x00, 0x01}
#endif

#define SSC_TMC_MOTOR_CURRENT_MA 1000
#define SSC_TMC_HOLD_CURRENT_PCT 60
#define SSC_TMC_HOLD_CURRENT_MULT ((float)SSC_TMC_HOLD_CURRENT_PCT / 100.0f)

// Elevator / TMC2209 defaults (single place for tuning)
#define SSC_TMC_RSENSE_OHM 0.11f
#define SSC_TMC_TOFF 4
#define SSC_TMC_BLANK_TIME 24
#define SSC_TMC_MICROSTEPS 16
#define SSC_TMC_ENABLE_SPREADCYCLE 0
#define SSC_TMC_TPWMTHRS 0

// Motor lag monitoring (lightweight loop delay watchdog)
#ifndef SSC_MOTOR_LAG_MONITOR_ENABLE
#define SSC_MOTOR_LAG_MONITOR_ENABLE 1
#endif

#ifndef SSC_MOTOR_LAG_WARN_THRESHOLD_MS
#define SSC_MOTOR_LAG_WARN_THRESHOLD_MS 35
#endif

#ifndef SSC_MOTOR_LAG_WARN_MIN_INTERVAL_MS
#define SSC_MOTOR_LAG_WARN_MIN_INTERVAL_MS 1000
#endif

// Web remote UI customization
// - Change labels in SSC_WEB_REMOTE_BUTTON_LABELS to rename displayed button text.
// - Change colors in SSC_WEB_REMOTE_BUTTON_COLORS to set each button color in webColor (0xRRGGBB).
// - Keep count/order aligned across KEYS/LABELS/COLORS.
struct SscWebColor
{
  uint32_t webColor;
};

struct SscRgbColor
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

constexpr SscRgbColor SSC_LED_STRIP_BASE_COLORS[SSC_LED_STRIP_COUNT] = {
  {56, 32, 32},  // strip 0 (warm white red tint)
  {32, 56, 32},  // strip 1 (warm white green tint)
  {32, 32, 56},  // strip 2 (cool white blue tint)
  {56, 48, 32},  // strip 3 (warm amber-ish white)
  {48, 32, 56},  // strip 4 (violet-ish white)
  {32, 52, 52},  // strip 5 (cyan-ish white)
};
static_assert(sizeof(SSC_LED_STRIP_BASE_COLORS) / sizeof(SSC_LED_STRIP_BASE_COLORS[0]) == SSC_LED_STRIP_COUNT,
              "SSC_LED_STRIP_BASE_COLORS count mismatch");

constexpr size_t SSC_WEB_REMOTE_BUTTON_COUNT = 21;

constexpr const char* SSC_WEB_REMOTE_BUTTON_KEYS[SSC_WEB_REMOTE_BUTTON_COUNT] = {
  "BTN_POWER", "BTN_MODE", "BTN_MUTE",
  "BTN_PLAYPAUSE", "BTN_PREV", "BTN_NEXT",
  "BTN_EQ", "BTN_VOL_DOWN", "BTN_VOL_UP",
  "BTN_0", "BTN_RPT", "BTN_CLOCK",
  "BTN_1", "BTN_2", "BTN_3",
  "BTN_4", "BTN_5", "BTN_6",
  "BTN_7", "BTN_8", "BTN_9",
};

constexpr const char* SSC_WEB_REMOTE_BUTTON_LABELS[SSC_WEB_REMOTE_BUTTON_COUNT] = {
  "POWER", "MODE", "MUTE",
  ">> []", "PREV", "NEXT",
  "EQ", "VOL_DOWN", "VOL_UP",
  "0", "RPT", "CLOCK",
  "1", "2", "3",
  "4", "5", "6",
  "7", "8", "9",
};

constexpr SscWebColor SSC_WEB_REMOTE_BUTTON_COLORS[SSC_WEB_REMOTE_BUTTON_COUNT] = {
  {0xF58080}, {0xF5F5F5}, {0xF5F5F5},
  {0xF5F5F5}, {0xA6C4D9}, {0xA6C4D9},
  {0xF5F5F5}, {0xA6D9D2}, {0xA6D9D2},
  {0xF5F5F5}, {0xF5FFF5}, {0xF5FFF5},
  {0xF5F5F5}, {0xF5F5F5}, {0xF5F5F5},
  {0xF5F5F5}, {0xF5F5F5}, {0xF5F5F5},
  {0xF5F5F5}, {0xF5F5F5}, {0xF5F5F5},
};

static_assert(sizeof(SSC_WEB_REMOTE_BUTTON_KEYS) / sizeof(SSC_WEB_REMOTE_BUTTON_KEYS[0]) == SSC_WEB_REMOTE_BUTTON_COUNT,
              "SSC_WEB_REMOTE_BUTTON_KEYS count mismatch");
static_assert(sizeof(SSC_WEB_REMOTE_BUTTON_LABELS) / sizeof(SSC_WEB_REMOTE_BUTTON_LABELS[0]) == SSC_WEB_REMOTE_BUTTON_COUNT,
              "SSC_WEB_REMOTE_BUTTON_LABELS count mismatch");
static_assert(sizeof(SSC_WEB_REMOTE_BUTTON_COLORS) / sizeof(SSC_WEB_REMOTE_BUTTON_COLORS[0]) == SSC_WEB_REMOTE_BUTTON_COUNT,
              "SSC_WEB_REMOTE_BUTTON_COLORS count mismatch");
