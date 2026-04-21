R"HTML(
<div class='box compact'>
<details class='collapsible'>
<summary><h2>Controller Settings</h2></summary>
<form method='POST' action='/save-control'>
<label>TMC run current (mA)</label>
<input name='tmc_run_current_ma' type='number' min='1' max='2000' value='{{TMC_RUN_CURRENT_MA}}'>
<label>TMC hold current (%)</label>
<input name='tmc_hold_current_pct' type='number' min='0' max='100' value='{{TMC_HOLD_CURRENT_PCT}}'>
<label>Move max speed (steps/sec)</label>
<input name='move_max_speed_steps_per_sec' type='number' min='1' value='{{MOVE_MAX_SPEED_STEPS_PER_SEC}}'>
<label>Move acceleration (steps/sec²)</label>
<input name='move_accel_steps_per_sec2' type='number' min='1' value='{{MOVE_ACCEL_STEPS_PER_SEC2}}'>
<label>Button zero steps (mute equivalent)</label>
<input name='btn_zero_steps' type='number' value='{{BTN_ZERO_STEPS}}'>
{{BTN_RELATIVE_ROWS}}
<p class='small'>INFOやserial/IRで更新可能な値をここから書き換えできます。保存時にNVSへ永続化します。</p>
<button type='submit'>Save Controller Settings</button>
</form>
</details>
</div>
)HTML"
