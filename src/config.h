#pragma once

#define SSC_PIN_IR            23
#define SSC_PIN_WS2812B       18

#define SSC_PIN_STEP          26
#define SSC_PIN_DIR           27
#define SSC_PIN_EN            14

#define SSC_PIN_ENDSTOP_UP    32
#define SSC_PIN_ENDSTOP_DOWN  33

#define SSC_TMC_UART_PORT     1
#define SSC_TMC_UART_PIN      17
#define SSC_TMC_UART_BAUD     115200

#define SSC_LED_COUNT         100
#define SSC_LED_BRIGHTNESS    64

#define SSC_STEPS_PER_FLOOR   3200
#define SSC_STEP_HZ_DEFAULT   800
#define SSC_STEP_PULSE_US     2

#ifndef SSC_MODE
#define SSC_MODE 0
#endif


// TMC2209 UART (added for UART switch)
#define SSC_TMC_UART_BAUD 115200
#define SSC_TMC_UART_ONEWIRE  1
#define SSC_TMC_UART_ONEWIRE_PIN  17
#define SSC_TMC_UART_RX_PIN  16
#define SSC_TMC_UART_TX_PIN  17
