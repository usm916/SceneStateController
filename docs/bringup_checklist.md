# Bring-up Checklist (Pi5 + ESP32 + TMC2209 + WS2812B + IR)

## 0. Libraries
Install (Arduino IDE Library Manager):
- Arduino-IRremote (4.x)
- FastLED
- TMCStepper

## 1. Wiring sanity
- Common GND between ESP32, TMC2209, WS2812B power.
- WS2812B: add 470–1000uF capacitor across 5V/GND near the strip.
- WS2812B DATA: add 330–470 ohm series resistor near ESP32 pin.
- IR receiver OUT -> SSC_PIN_IR (default GPIO23). VCC per module spec.
- TMC2209: STEP/DIR/EN -> SSC_PIN_STEP/DIR/EN.
- Endstops: wire to GND when pressed (active LOW), pins default GPIO25/13 with INPUT_PULLUP.

## 2. Power-up order (recommended)
- Power ESP32 first or at the same time as WS2812B to reduce random LED states.
- In code, LED is cleared on led_setup().

## 3. UART mode selection
### A) One-wire PDN_UART (default)
- Set SSC_TMC_UART_ONEWIRE=1
- Connect PDN_UART to SSC_TMC_UART_ONEWIRE_PIN (default GPIO17)
- Many modules need a resistor/diode network for stable one-wire UART. If UART read/write fails:
  - Try 1k series resistor from ESP32 TX to PDN_UART
  - Or separate RX/TX (two-wire) if your module supports it

### B) Two-wire UART
- Set SSC_TMC_UART_ONEWIRE=0
- Connect ESP32 TX -> TMC PDN_UART
- Connect ESP32 RX <- TMC PDN_UART (if module provides separate RX path; otherwise use one-wire)
- Set SSC_TMC_UART_RX_PIN / SSC_TMC_UART_TX_PIN accordingly

## 4. First boot check
- Open Serial Monitor at `SSC_USB_SERIAL_BAUD` (default 115200).
- Confirm you see:
  - "SceneStateController (Arduino Prototype)"
  - "READY"

## 5. LED-only test
- Set SSC_MODE=2
- Expect: dim blue solid (IDLE)

## 6. IR-only test
- Set SSC_MODE=1
- Press remote keys.
- Expect: "IR <proto> 0x<addr> 0x<cmd>" lines.
- Functional checks:
  - `VOL-`: zero homing starts
  - `EQ`: calibration mode starts (zero homing)
  - `VOL+`: top homing starts; when calibration is armed, saves top range
- If nothing:
  - Confirm IR OUT pin matches SSC_PIN_IR
  - Confirm IR module VCC (3.3V vs 5V)
  - Try enabling IRremote examples to validate hardware

## 7. EV-only test (motor disabled safe)
- Set SSC_MODE=3
- Temporarily disconnect motor power (VMOT) for the first run.
- Confirm no boot loops and no overheating on driver.

## 8. EV movement test (with VMOT)
- Connect VMOT and motor.
- Send from Serial Monitor (or Pi5) command:
  - MOVE 1
- Expect EV starts stepping (direction depends on wiring).
- If direction is reversed, swap motor direction by flipping DIR logic or swapping a coil pair.

## 9. Integrated test (recommended)
- Set SSC_MODE=0
- Send:
  - MOVE 1
- Expect:
  - LED switches to MOVING pattern
  - EV runs
  - On arrival, LED changes to ARRIVED then back to IDLE

## 10. Safety checks
- Press endstops while moving to validate error handling.
- Confirm it transitions to ERROR (red blink) and reports ERR code.
