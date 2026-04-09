# SceneStateController – Scene / State Design

Scene and State are separated layers.
- **Scene**: meaning/presentation layer for LED + upper-level behavior.
- **Elevator State**: mechanical state for stepper motion, homing, and calibration.

## Elevator control model (2026-04 update)

### Position model
- Bottom endstop (`ENDSTOP_DOWN`) is treated as **absolute zero**.
- Top range limit is stored as `top_step` in NVS (`Preferences`, namespace `ev_calib`).
- Normal runtime target is clamped to `0..top_step` when calibration exists.

This design intentionally fixes `min=0`, so only `top_step` is persisted.

### IR control mapping
- `BTN_EQ`      : Start calibration flow (immediately runs zero homing).
- `BTN_VOL_DOWN`: Zero homing only.
- `BTN_VOL_UP`  : Top homing. If calibration is armed, captures `top_step` and saves it.
- `BTN_0..BTN_3`: Direct move to floor index (`0..3`).
- `BTN_PREV/NEXT`: Manual CW/CCW spin while held.

## Elevator state transition diagram

```mermaid
stateDiagram-v2
  [*] --> EV_IDLE

  EV_IDLE --> EV_MOVING_UP: Move target > current
  EV_IDLE --> EV_MOVING_DOWN: Move target < current

  EV_IDLE --> EV_HOMING_ZERO: BTN_VOL_DOWN
  EV_IDLE --> EV_CALIBRATING: BTN_EQ (calibration arm)
  EV_CALIBRATING --> EV_HOMING_TOP: BTN_VOL_UP

  EV_IDLE --> EV_HOMING_TOP: BTN_VOL_UP (manual top homing)

  EV_MOVING_UP --> EV_ARRIVED: target reached
  EV_MOVING_DOWN --> EV_ARRIVED: target reached
  EV_ARRIVED --> EV_IDLE: settle

  EV_HOMING_ZERO --> EV_IDLE: zero established
  EV_HOMING_ZERO --> EV_CALIBRATING: zero established while calibration armed

  EV_HOMING_TOP --> EV_IDLE: top reached
  EV_HOMING_TOP --> EV_IDLE: top_step saved (if calibration armed)

  EV_MOVING_UP --> EV_ERROR: endstop/timeout
  EV_MOVING_DOWN --> EV_ERROR: endstop/timeout
  EV_HOMING_ZERO --> EV_ERROR: switch not found / invalid travel
  EV_HOMING_TOP --> EV_ERROR: switch not found / invalid travel
  EV_CALIBRATING --> EV_ERROR: homing failed

  EV_ERROR --> EV_IDLE: next valid command
```

## Calibration procedure
1. Press **EQ** to arm calibration and run zero homing.
2. After zero is established, press **VOL_UP**.
3. On top endstop detect, current position is captured as `top_step` and saved via Preferences.
4. Normal move commands then run in the calibrated `0..top_step` range.

## Failure policy
- Homing phases use max travel guard; if exceeded, state transitions to `EV_ERROR`.
- Endstop conflict during regular position move transitions to `EV_ERROR`.
- Calibration flag is cleared after top capture/save.
