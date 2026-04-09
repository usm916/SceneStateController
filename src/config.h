#pragma once

#define SSC_PIN_IR            23
#define SSC_PIN_WS2812B       18

#define SSC_PIN_STEP          26
#define SSC_PIN_DIR           27
#define SSC_PIN_EN            14

#define SSC_PIN_ENDSTOP_UP    32
#define SSC_PIN_ENDSTOP_DOWN  33

// #define SSC_TMC_UART_PORT     1
// #define SSC_TMC_UART_PIN      17
// #define SSC_TMC_UART_BAUD     115200

#define SSC_LED_COUNT         100
#define SSC_LED_BRIGHTNESS    64

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


// TMC2209 UART (added for UART switch)
#define SSC_TMC_UART_BAUD 115200
#define SSC_TMC_UART_ONEWIRE  0
#define SSC_TMC_UART_ONEWIRE_PIN  17
#define SSC_TMC_UART_RX_PIN  16
#define SSC_TMC_UART_TX_PIN  17

#define SSC_TMC_MOTOR_CURRENT_MA 330
#define SSC_TMC_HOLD_CURRENT_PCT 60

// Elevator / TMC2209 defaults (single place for tuning)
#define SSC_TMC_RSENSE_OHM 0.11f
#define SSC_TMC_TOFF 4
#define SSC_TMC_BLANK_TIME 24
#define SSC_TMC_RMS_CURRENT_MA 300
#define SSC_TMC_RMS_HOLD_MULT 0.6f
#define SSC_TMC_MICROSTEPS 16
#define SSC_TMC_ENABLE_SPREADCYCLE 0
#define SSC_TMC_TPWMTHRS 0
