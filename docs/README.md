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
Endstop UP -> GPIO25 (active low), Endstop DOWN -> GPIO13 (active low)

If you want endstops without internal pull-up:
- Set `SSC_ENDSTOP_USE_INPUT_PULLUP` to `0` in `src/config.h`.
- Endstop input mode changes to `INPUT` and pressed-level is treated as `HIGH`.


## UART wiring modes
- One-wire (default): SSC_TMC_UART_ONEWIRE=1, use SSC_TMC_UART_ONEWIRE_PIN.
- Two-wire: set SSC_TMC_UART_ONEWIRE=0 and define SSC_TMC_UART_RX_PIN / SSC_TMC_UART_TX_PIN.

## TMC2209 current tuning (USB Serial commands)
- `TMC MOTOR <mA>`: sets motor RMS current (same as run current in this prototype).
- `TMC RUN <mA>`: alias of `TMC MOTOR` for compatibility with common naming.
- `TMC HOLD <0..100>`: hold current percentage relative to run current.
- `TMC INFO`: prints current settings.

## INFO系コマンド一覧（Serial）

ステータス確認で使うコマンドをここにまとめます。

- `INFO` / `info`
  システムの統合ステータスを表示します。主な出力:
  - scene / elevator_state
  - floor_current / floor_target
  - pos_current_steps / pos_target_steps / distance_to_go_steps
  - moving
  - motor_lag_*（監視値）
  - calibration_* / homed_zero / top_limit_steps / top_margin_steps / bottom_margin_steps
  - tmc_motor_current_ma / tmc_run_current_ma / tmc_hold_current_pct
  - move_max_speed_steps_per_sec / move_accel_steps_per_sec2
  - button preset情報（btn_zero_steps, btn_0..btn_9 の登録状態と値）

- `TMC INFO`
  TMC2209 の電流設定だけを簡易表示します。

### 併用する設定コマンド（INFOで反映確認）
- `mute`
- `rec_<btn>`
- `rec_<btn>_<steps>`
- `save_pref`
- `speed_<steps_per_sec>`
- `accel_<steps_per_sec2>`
- `TMC MOTOR <mA>`
- `TMC RUN <mA>`
- `TMC HOLD <0..100>`

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
- 起動時の位置確定は `VOL-` によるゼロホーミングで行います（必要に応じて起動シーケンスへ組み込み）。
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
   - `PREV` 押下: CCW回転
   - `NEXT` 押下: CW回転
   - どちらも押していない: 減速停止

#### 3) Homing / Calibration（IR）
- `VOL-` (`BTN_VOL_DOWN`): ゼロホーミング（DOWN endstopへ復帰して0位置を確定）
- `VOL+` (`BTN_VOL_UP`): トップホーミング（UP endstopまで移動）
- `EQ` (`BTN_EQ`): キャリブレーション開始（ゼロホーミング実行後、`VOL+`でトップ位置を保存）

保存データは `Preferences`（NVS, namespace: `ev_calib`）に `top_step` として保持されます。

### 調整パラメータ（まずはここ）
- `SSC_STEPS_PER_FLOOR`（`src/config.h`）:
  フロアあたりのステップ数。機構に合わせて最優先で調整。
- `SSC_STEP_HZ_DEFAULT`（`src/config.h`）:
  位置移動の最大速度（step/s相当）。
- `kMoveAccelerationDefault`（`src/elevator_module.cpp`）:
  加速度（step/s^2）。大きすぎると脱調しやすいので徐々に上げる。
- `kHomingFastSpeed` / `kHomingSlowSpeed`（`src/elevator_module.cpp`）:
  ホーミング時の高速/低速の速度。
- `kHomingBackoffSteps`（`src/elevator_module.cpp`）:
  一度スイッチ検出後に戻る量（再接触で精度を上げる）。

### 初期チューニング手順（推奨）
1. `TMC RUN 600` / `TMC HOLD 30` あたりで電流を設定。
2. `SSC_STEP_HZ_DEFAULT` を低め（例: 800）から開始。
3. `kMoveAccelerationDefault` を低め（例: 1000〜2000）から開始。
4. 徐々に速度・加速度を上げ、脱調や発熱を見ながら詰める。

## IRボタンが「効いたり効かなかったり」見えるとき

以下のような `else if` 連鎖で `ev_state != BTN_X` を条件にしている場合、
**一度 `ev_state` が `BTN_X` になると、どこかで明示的に解除しない限り再押下が無視されます。**

- 典型的な問題:
  - `ev_state` を `BTN_NONE` に戻す処理がない
  - ボタン押下の「レベル判定（押されている間 true）」を使っている
  - 同じボタンの再押下で `ev_state != BTN_X` が常に false になる

### 対策（推奨）
- 全ボタンが離された瞬間に `ev_state = BTN_NONE` を入れる。
- 可能ならレベル判定ではなく、**立ち上がりエッジ（just pressed）**で `handleInput()` を呼ぶ。
- `1.5 * SSC_STEPS_PER_FLOOR` のような小数フロア指定は、丸め規則を明示する（`lroundf` など）。

### 最小修正例
```cpp
if (!m00_pressed && !m10_pressed && !m15_pressed && !m20_pressed) {
  ev_state = BTN_NONE;
}

if (m00_pressed && ev_state != BTN_0) {
  handleInput(0);
  ev_state = BTN_0;
} else if (m10_pressed && ev_state != BTN_1) {
  handleInput(1 * (int32_t)SSC_STEPS_PER_FLOOR);
  ev_state = BTN_1;
} else if (m15_pressed && ev_state != BTN_2) {
  handleInput((int32_t)lroundf(1.5f * (float)SSC_STEPS_PER_FLOOR));
  ev_state = BTN_2;
} else if (m20_pressed && ev_state != BTN_3) {
  handleInput(2 * (int32_t)SSC_STEPS_PER_FLOOR);
  ev_state = BTN_3;
}
```
