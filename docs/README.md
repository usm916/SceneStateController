# SceneStateController – Arduino Prototype

## What this ZIP contains (requested order)
2) Scene/State enums + Event definitions: src/scene_state.h, src/events.h
1) Arduino prototype scaffold: SceneStateController.ino + src/* modules
3) TMC2209 init/config: src/tmc2209_module.* (TMCStepper)
4) Integrated flow: SceneController drives EV+LED and sends events to Pi5 over USB Serial

## Libraries (Arduino IDE Library Manager)
- Arduino-IRremote (4.x)
- FastLED
- AccelStepper
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

## TMC2209 current tuning (USB Serial commands)
- `TMC MOTOR <mA>`: sets motor RMS current (same as run current in this prototype).
- `TMC RUN <mA>`: alias of `TMC MOTOR` for compatibility with common naming.
- `TMC HOLD <0..100>`: hold current percentage relative to run current.
- `TMC INFO`: prints current settings.

Meaning of values:
- `motor_current` / `run_current`: TMC2209 coil RMS current in mA.
- `hold_current`: percentage used while stopped; lower values reduce heat.

Typical starting points for NEMA17-class steppers:
- `run_current`: 500–800 mA
- `hold_current`: 30–50%

## Elevator control (AccelStepper版) 使いかた

このプロトタイプでは、EV（ステッピングモーター）制御に `AccelStepper` を使い、
`moveTo` + `run` で加減速つきの移動を行います。

### 挙動の概要
- 起動時に短いウォームアップ回転（startup spin）を実施します。
- 位置移動は floor 指定で行い、内部で `SSC_STEPS_PER_FLOOR` を使って step 数へ変換します。
- 手動スピン（CW/CCW）は連続回転ターゲットを前方へ延長しながら回します。
- 停止時は即停止ではなく、加速度設定に従って減速停止します。

### 操作方法

#### 1) EV only モードで位置移動（シリアル）
1. `SSC_MODE` を `3`（EV only）に設定して書き込み。
2. USB Serial を開き、次のコマンドを送信:
   - `MOVE <floor>`
   - 例: `MOVE 3`
3. モーターが加減速しながら指定フロアへ移動します。

#### 2) Integrated モードでIR手動回転
1. `SSC_MODE` を `0`（Integrated）に設定。
2. IRリモコンの:
   - `PREV` 押下: CW回転
   - `NEXT` 押下: CCW回転
   - どちらも押していない: 減速停止

### 調整パラメータ（まずはここ）
- `SSC_STEPS_PER_FLOOR`（`src/config.h`）:
  フロアあたりのステップ数。機構に合わせて最優先で調整。
- `SSC_STEP_HZ_DEFAULT`（`src/config.h`）:
  位置移動の最大速度（step/s相当）。
- `kMoveAccelerationDefault`（`src/elevator_module.cpp`）:
  加速度（step/s^2）。大きすぎると脱調しやすいので徐々に上げる。
- `kStartupSpinRpm` / `kStartupSpinDurationMs`（`src/elevator_module.cpp`）:
  起動時ウォームアップ回転の速度と時間。

### 初期チューニング手順（推奨）
1. `TMC RUN 600` / `TMC HOLD 30` あたりで電流を設定。
2. `SSC_STEP_HZ_DEFAULT` を低め（例: 800）から開始。
3. `kMoveAccelerationDefault` を低め（例: 1000〜2000）から開始。
4. 徐々に速度・加速度を上げ、脱調や発熱を見ながら詰める。
