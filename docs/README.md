# SceneStateController – Arduino Prototype

## What this ZIP contains (requested order)
2) Scene/State enums + Event definitions: src/scene_state.h, src/events.h
1) Arduino prototype scaffold: SceneStateController.ino + src/* modules
3) TMC2209 init/config: src/tmc2209_module.* (TMCStepper)
4) Integrated flow: SceneController drives EV+LED and sends events to Pi5 over USB Serial

## Libraries (Arduino IDE Library Manager)
- Arduino-IRremote (4.x)
- FastLED
- TMCStepper

## Modes
Set build flag or edit SSC_MODE in src/config.h:
- 0: Integrated
- 1: IR only
- 2: LED only
- 3: EV only
- 4: Pi link only


## Wiring defaults
IR OUT -> GPIO23
WS2812B DATA -> GPIO18
STEP -> GPIO26, DIR -> GPIO27, EN -> GPIO14 (active low on many boards)
PDN_UART (one-wire) -> GPIO17 (may require diode/resistor network)
Endstop UP -> GPIO32 (active low), Endstop DOWN -> GPIO33 (active low)


## UART wiring modes
- One-wire (default): SSC_TMC_UART_ONEWIRE=1, use SSC_TMC_UART_ONEWIRE_PIN.
- Two-wire: set SSC_TMC_UART_ONEWIRE=0 and define SSC_TMC_UART_RX_PIN / SSC_TMC_UART_TX_PIN.
