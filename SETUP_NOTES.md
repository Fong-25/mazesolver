# Setup / Tuning / Hardware Notes

## A. One-time physical measurements (measure once with tools, rarely revisit)
- `RobotConfig::WHEEL_DIAMETER_MM`, `WHEEL_BASE_MM`, `WHEEL_TRACK_MM`
  — Calipers/ruler on the assembled robot.
- `BoardConfig::BATTERY_DIVIDER_R1_OHM` / `R2_OHM`
  — Multimeter the *actual* populated resistors (manufacturing tolerance
  means the nominal value on the schematic may be off by a few %). Verify
  by comparing `BatteryMonitor::getVoltage()` against a multimeter reading
  on the battery directly; if they disagree, the resistor values (not the
  math) are the thing to correct.
- `BoardConfig::RGB_LED_COUNT`
  — Count the actual WS2812B chain length on the PCB.

## B. Simple direction/polarity checks (one test, flip a bool, done)
- `RobotConfig::LEFT_MOTOR_REVERSED` / `RIGHT_MOTOR_REVERSED`
  — Command a small forward PWM to ONE motor at a time (motors disabled
  otherwise). Watch which way the wheel spins. Wrong direction -> flip the
  bool for that side. Do this before touching encoders — you need correct
  motor direction as the reference point.
- `RobotConfig::LEFT_ENCODER_REVERSED` / `RIGHT_ENCODER_REVERSED`
  — With motor polarity already correct, command that motor forward and
  watch `LEFT_ENCODER_COUNT()`/`RIGHT_ENCODER_COUNT()` via `LOG E` (DEBUG_LOG
  mode). If commanding forward makes the count DEcrease, flip the encoder's
  REVERSED flag so the API always reports positive = forward, regardless of
  physical wiring.
- `RobotConfig::IMU_YAW_AXIS` / `IMU_YAW_SIGN`
  — Hold the robot still and rotate it by hand (e.g. 90° clockwise viewed
  from above). Watch `LOG I`. If the axis barely moves, wrong axis — try
  the other one. If the axis moves but the sign is backwards (CW should
  read negative or positive depending on your convention — pick one and be
  consistent), flip `IMU_YAW_SIGN`.

## C. Iterative tuning (start conservative, adjust based on observed behavior)
- `ControlConfig::LEFT_KP/KI/KD`, `RIGHT_KP/KI/KD` (wheel velocity PID)
  — Classic manual tuning: set I=D=0, raise P until the wheel oscillates
  around the target speed, back off ~30-50%. Then add D to damp overshoot.
  Add a small I last, only if there's persistent steady-state error. Use
  `MOTION_TEST`'s `TEST CRUISE` + `LOG E`/`LOG M` to watch the response.
- `ControlConfig::HEADING_KP/KI/KD`
  — Same method, using `TEST TURNL`/`TEST TURNR`/`TEST TURN180` + `LOG I`
  to compare commanded turn angle against actual integrated yaw.
- `ControlConfig::WALL_KP/KI/KD`
  — Same method, but needs the physical maze/walls to test against (ToF
  involved) — defer until ToF wall-following logic exists.
- `RobotConfig::MAX_LINEAR_SPEED_MM_S`, `MAX_LINEAR_ACCEL_MM_S2`,
  `MAX_TURN_RATE_DEG_S`
  — Start at the conservative defaults already set. Raise gradually using
  `TEST ACCEL`/`TEST DECEL`, watching for wheel slip (visually, or a sudden
  encoder/IMU mismatch) as the ceiling.

## D. Bench calibration against a known reference
- `RobotConfig::ENCODER_PULSES_PER_OUTPUT_REV`
  — Ground truth beats datasheet math: rotate the output wheel by hand
  exactly one full turn, read the raw count via `LOG E`, divide by 4
  (quadrature). Compare against the datasheet's 350/rev as a sanity check,
  but trust the physical measurement if they disagree.
- `SensorConfig::TOF_MIN_VALID_MM/TOF_MAX_VALID_MM`,
  `FRONT_WALL_MM`/`SIDE_WALL_MAX_MM`
  — Place a flat object at a known measured distance, compare against
  `LOG T`. Adjust thresholds to match real wall-detection distances for
  your maze cell size, not the datasheet's generic range.
- `SensorConfig::IMU_GYRO_FS_SEL` / `IMU_GYRO_SENSITIVITY_LSB_PER_DPS`
  — Currently ±500°/s. If `TEST TURNL`/`TURNR`/`TURN180` show the yaw rate
  pinned at a maximum value (clipping) during fast turns, raise the range
  (keep the paired sensitivity constant in sync — see the comment in
  `SensorConfig.h`).
- `RobotConfig::BATTERY_LOW_VOLTAGE` / `BATTERY_CRITICAL_VOLTAGE`
  — Current defaults (7.0V / 6.6V) are standard-ish for a 2S LiPo, but
  confirm against your specific pack/BMS's actual safe cutoff voltage.

## E. Feel/comfort tuning (no wrong answer, just preference)
- `UserConfig::BUTTON_DEBOUNCE_MS`, `BUTTON_LONG_PRESS_MS`
  — Adjust if the physical button feels laggy or double-fires.
- `UserConfig::STANDBY_BLINK_INTERVAL_MS`
  — Whatever blink rate reads clearly as "standby" to you.
- `UserConfig::BATTERY_SAMPLE_INTERVAL_MS`, `TOF_POLL_INTERVAL_MS`,
  `DIP_DEBOUNCE_MS`
  — These affect responsiveness vs. I2C/CPU load, not correctness. Lower
  only if you notice lag; there's no accuracy reason to tune these.
- `RobotConfig::ENCODER_GLITCH_SAFETY_MARGIN` (currently 1.5x)
  — Tighten only if `consumeGlitchFlag()` never fires during genuinely
  aggressive runs and you want tighter fault detection; loosen if it
  false-positives during normal hard acceleration.

## Open questions — sensor layout
- **RESOLVED**: 4 ToF sensors — 2 straight-ahead-facing (front-left,
  front-right, both parallel forward), 1 diagonal-front-left, 1
  diagonal-front-right. No AUXILIARY/rear sensor.
- **Action needed** (deferred until after MotorDriver, per plan): rename
  `ToFManager::SensorRole` from {FRONT, DIAGONAL_LEFT, DIAGONAL_RIGHT,
  AUXILIARY} to {FRONT_LEFT, FRONT_RIGHT, DIAGONAL_LEFT, DIAGONAL_RIGHT}.
  Single-place change: the enum in ToFManager.h + the 4
  `RobotConfig::TOF_ROLE_INDEX_*` constants. No other code touches roles
  yet, so this is contained.