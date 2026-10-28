# Tuning & Configuration Guide

Order matters. Each phase depends on the one before it. Do not skip ahead.

## Ground rules
- **Battery connected + SW1 on for everything below.** USB-only reads ~0 V on
  the battery ADC -> `BATTERY_CRITICAL` -> ERROR (solid red) immediately.
- **Wheels lifted** until a step says put it on the floor.
- **`STOP` or any fault = ERROR until reset.** Nothing calls
  `Safety::clearTrip()`. Power-cycle to recover.
- Terminal: 115200 baud, newline line ending. Flash with the HC-06
  disconnected.
- `PID L|R kp ki kd` changes are **lost on reboot**. Copy final values into
  `ControlConfig.h`. Heading/wall gains have no runtime command (reflash).

### Entering a mode over Bluetooth (used in every phase)
1. Be in LOCKED (setting DIP off). `MODE <n>` (0 EXPLORE, 1 FAST_RUN,
   2 DIAGNOSTIC, 3 DEBUG_LOG, 4 MOTION_TEST). RGB shows the mode color.
2. Hold the button >= 3 s -> STANDBY (color blinks).
3. Cover the start sensor (FRONT_LEFT by default) <= 30 mm, then release ->
   RUNNING. Color goes solid when the cover registers.
4. Leave with `EXIT`, then a short button press -> LOCKED.

## Phase 0 - Boot, comms, power
- **0.1 Boot.** ~1 s rainbow, then serial `OK`, `READY`. A `TOF FAIL` or
  `IMU FAIL` line means red; check I2C wiring, XSHUT pins,
  addresses. Keep the robot **still** during boot (gyro bias calibration).
- **0.2 HC-06 baud.** HC-06 defaults to 9600. Set it to 115200 (`AT+BAUD8`)
  or change `UserConfig::BLUETOOTH_BAUD`. Test `PING` -> `PONG`, `STATUS`.
- **0.3 Battery.** `BAT` vs a multimeter on the battery. If off, fix in this
  order: `BATTERY_DIVIDER_R1/R2_OHM` (measure the real resistors), then
  `ADC_REFERENCE_V` (measure the actual 5 V rail while running on the
  battery, not USB). Target +-0.05 V. Confirm `BATTERY_LOW_VOLTAGE` /
  `BATTERY_CRITICAL_VOLTAGE` against your pack's real safe cutoff.
- **0.4 Facts to verify, not tune:** `MazeConfig` (16x16, start (0,0), 2x2
  goal), `CELL_SIZE_MM` (measure your real maze pitch),
  `RGB_LED_COUNT`, `TOF_INSTALLED_COUNT`, `SensorConfig` I2C addresses.

## Phase 1 - Direction & identity (wheels lifted)
Order: motors, encoders, IMU, ToF roles.
- **1.1 Motors.** DIAGNOSTIC: `MOTOR L 100`. It should spin forward; else flip
  `LEFT_MOTOR_REVERSED`. `MOTOR STOP`. Repeat for R. If nothing turns, raise
  the PWM.
- **1.2 Encoders.** `LOG E`, then `MOTOR L 100`: the count must **increase**;
  else flip `LEFT_ENCODER_REVERSED`. Repeat for R. No change at all is
  wiring, not a flag. Spin one wheel and confirm the other's count stays put.
- **1.3 IMU.** `LOG I`. Still: rate ~0, yaw drift < ~1 deg/10 s. Rotate the
  robot **counter-clockwise viewed from above** by 90 deg: yaw must read
  ~ +90. (Convention is fixed in code: positive = left/CCW; turns and pose
  depend on it.) Barely moves -> change `IMU_YAW_AXIS`. Moves but negative ->
  flip `IMU_YAW_SIGN`. Do not continue until right.
- **1.4 ToF roles.** `SENSOR` or `LOG T`. Cover each sensor; the role that
  reacts must match its physical position. If not, change
  `TOF_ROLE_INDEX_*` in `RobotConfig` (do not rewire).
- **1.5 Start sensor.** Covering `START_SENSOR_ROLE_INDEX` must read
  reliably <= `TOF_COVER_THRESHOLD_MM` (30) and above `TOF_MIN_VALID_MM`
  (20). If flaky, raise the threshold or pick another sensor. Keep it well
  below your wall readings.

## Phase 2 - Geometry & scale (on the floor, push by hand, motors idle)
Order: gyro scale, encoder PPR, wheel diameters, wheel base. The IMU is the
angle reference; the ruler is the distance reference.
- **2.1 Gyro scale.** Rotate by hand exactly 360 deg against a marked line,
  `LOG I`: expect 360 +-1-2 %. Average 3 runs, keep each under ~20 s to limit
  drift. Off by more: check `IMU_GYRO_FS_SEL` and
  `IMU_GYRO_SENSITIVITY_LSB_PER_DPS` are still a matched pair (0x08 / 65.5).
- **2.2 `ENCODER_PULSES_PER_OUTPUT_REV`.** Turn the output wheel exactly N
  revolutions (N = 5), read `ENC`. PPR = raw / (4 x N).
  (`MODE_GESTURE_MIN_COUNTS` follows automatically.)
- **2.3 Wheel diameters.** Caliper each wheel as a start. Then push the
  robot straight 1000 mm along a ruler and read `LOG P` (pose zeroes at boot,
  so use differences or reset the board). Left and right counts should match
  over straight travel: set `D_L / D_R` so `counts_L x D_L = counts_R x D_R`.
  Then scale both so pose distance = ruler distance. Target: within 1 %.
- **2.4 `WHEEL_BASE_MM`.** Rotate in place by hand ~90-135 deg (avoid the +-180
  wrap). Compare `LOG P` `th` with `LOG I` yaw:
  `WHEEL_BASE_new = WHEEL_BASE_old x (th_encoder / yaw_imu)`. Repeat both
  directions. Target: agree within ~1-2 deg.
- (`WHEEL_TRACK_MM` is unused. Ignore it.)

## Phase 3 - Actuators (motors enabled; needs the MotionTest fixes)
- **3.1 `MOTOR_DEADBAND_PWM`.** On the **floor** (weight matters). DIAGNOSTIC:
  `MOTOR L n` from 10 up in steps of 5 until the wheel starts reliably from
  rest, both directions, both wheels. Set the deadband to the larger value
  plus ~10 %.
- **3.2 Wheel velocity PID** (`LEFT_/RIGHT_ KP/KI/KD`,
  `WHEEL_PID_INTEGRAL_LIMIT`, `WHEEL_PID_DERIVATIVE_FILTER_ALPHA`).
  - Units: error in mm/s, output PWM +-255. Integral is error x seconds,
    clamped +-150, so the max I-term is Ki x 150 PWM. Keep that under ~255.
  - No feedforward: P-only leaves a big steady-state error, so Ki is
    required.
  - Speed from `LOG E`: v = dCounts x mm_per_count / dt, where mm_per_count =
    pi x D / (4 x PPR) (~0.0763 for D = 34, PPR = 350). Set
    `LOG_STREAM_INTERVAL_MS` to ~50 while tuning; do not leave streams on
    during real runs (the 32-byte TX buffer makes prints block the loop).
  - At 300 mm/s each 2 ms tick sees only ~8 counts, so speed is noisy. Keep
    Kd ~0 (or lower the alpha).
  - Method: `TEST CRUISE`, watch `LOG E` + `LOG M`, adjust live with `PID`.
    Start Kp 0.3, Ki 0, Kd 0. Raise Kp until the speed oscillates, back off
    30-50 %. Add Ki until steady speed is within ~3 % of target in ~0.3 s
    without oscillation. Tune left, copy to right, verify. Target: both
    wheels within 2 % of each other.
- **3.3 Heading hold** (`HEADING_KP/KI/KD`, `HEADING_CORRECTION_LIMIT_MM_S`).
  Units: mm/s per degree. Start Kp 2-3, Ki 0, Kd 0. `TEST CRUISE`
  (720 mm, needs >= 1 m clear straight): yaw at the end minus at the start
  should be < ~1 deg. Nudge the robot ~10 deg mid-run: it should recover
  without oscillating. Oscillates -> lower Kp. Drifts -> raise. To see the
  baseline drift, set `ENABLE_IMU_HEADING_HOLD` false for one run. If
  heading hold makes the veer **worse**, the yaw sign/axis (1.3) is wrong;
  do not flip PID gains.
- **3.4 Turns** (`TURN_SPEED_DEG_S`, `TURN_ANGLE_TOLERANCE_DEG`,
  `MOTION_SETTLE_MS`). `TEST TURNL/TURNR/TURN180` with `LOG I`. The turn is
  IMU-terminated with no decel ramp, so speed is the main lever against
  overshoot. Repeat 10x. Check repeatability (< 1-2 deg), left/right
  symmetry, and that it rotates about its center (mark the floor). Confirm
  `LOG P` `th` agrees with the IMU. Raise `MOTION_SETTLE_MS` if the wheels
  are still moving when the next primitive starts.
- **3.5 Distance & accel.** `TEST CRUISE` 720 mm: measure the actual travel
  with a ruler. A systematic error means go back to 2.3. Then raise
  `MAX_LINEAR_ACCEL_MM_S2` (1500) gradually: too high = slip on the
  accel edge / overshoot on the brake edge. Re-check at full **and** low
  battery.
- **3.6 Speed constants.** `FORWARD_BASE_SPEED_MM_S` (EXPLORE) and
  `FAST_RUN_SPEED_MM_S` stay conservative until Phase 5.
  `MAX_LINEAR_SPEED_MM_S` only sets the encoder-glitch limit (x
  `ENCODER_GLITCH_SAFETY_MARGIN`); raise it with any speed above ~1000
  mm/s or you get false `ENCODER_INVALID` trips. `MAX_TURN_RATE_DEG_S` is
  unused.

## Phase 4 - Sensing (needs a real maze section: straight corridor, an end
wall, one open side)
- **4.1 ToF offsets** (`TOF_OFFSET_MM_1..4`, indexed by physical XSHUT order).
  Procedure in `MODES_AND_BLUETOOTH_PROTOCOL.md`. Check at 2-3 distances
  (50/100/150 mm). A constant error is an offset. An error that grows with
  distance is gain, which config can't fix; choose a mid-range offset.
- **4.2 `TOF_DIAGONAL_MOUNT_ANGLE_DEG`.** Hold a flat wall parallel to the
  robot side at a known perpendicular distance d. If the raw slant reading
  is r, then angle = asin(d / r). Average two distances. Confirm the
  `LOG T` diagonal value now equals d.
- **4.3 `TOF_MIN_VALID_MM` / `TOF_MAX_VALID_MM`.** Check open-side readings
  come out invalid or above `SIDE_WALL_MAX_MM`. Leave the defaults unless
  `LOG T` shows a problem.
- **4.4 `FRONT_WALL_MM`, `SIDE_WALL_MAX_MM`.** Procedure in the MODES doc,
  from a centered pose. Redo after any change in 4.1 or 4.2.
- **4.5 Freshness.** Each sensor refreshes every ~80 ms (4 x
  `TOF_POLL_INTERVAL_MS`), and EXPLORE senses right after
  `MOTION_SETTLE_MS` (80). That is borderline. Confirm with `LOG T` that all
  four values are post-stop; if stale, raise `MOTION_SETTLE_MS`.

## Phase 5 - Start alignment & first maze runs
- **5.1 `ROBOT_CENTER_TO_REAR_MM`.** Measure rear-most point to axle center.
  `FIRST_MOVE_DISTANCE_MM = 90 - value` (must be <= 90). Verify: start
  EXPLORE with the rear against the wall; after the alignment move the axle
  must sit over the cell center (mark it).
- **5.2 EXPLORE.** Start with a tiny closed layout (2x2 cells), `LOG P`.
  After the run: `MAP`, `SAVE`, power-cycle, `LOAD` -> `LOAD OK`. Grow the
  maze gradually.
- **5.3 FAST_RUN.** Run the saved map at the default speed and confirm it
  follows the expected path with batched straights. Only then raise
  `FAST_RUN_SPEED_MM_S` in steps (500 -> 600 ...), re-checking heading hold
  each step and raising `MAX_LINEAR_ACCEL_MM_S2` if the ramps dominate. No
  advanced fast run until this is solid.

## Phase 6 - Wall centering (last, only after 3-5 are solid)
1. Flip **both**: `ENABLE_TOF_WALL_CENTERING = true` and
   `#define MAZESOLVER_COMPILE_TOF_WALL_CENTERING 1`. Rebuild and check the
   `pio run` RAM/Flash lines (+46 B RAM).
2. It only acts on moves that pass `allowWallCentering = true` (EXPLORE and
   FAST_RUN cell moves). Never MotionTest or the alignment moves.
3. Lower `WALL_CORRECTION_LIMIT_MM_S` to ~50 for the first runs.
4. Sign check: place the robot deliberately off-center toward the left wall;
   it must steer away. If it steers into the wall, swap `rightMm - leftMm`
   to `leftMm - rightMm` in `Motion.cpp`'s `combinedCorrection()`.
5. Tune `WALL_KP/KI/KD` (mm/s per mm of left-minus-right difference). Start
   Kp 1.0, Ki/Kd 0. Heading hold opposes lateral correction, so centering
   is slower with a high `HEADING_KP`.

## Phase 7 - System & comfort
- **Loop timing.** `CONTROL_PERIOD_US` 2000, `IMU_PERIOD_US` 5000: toggle a
  spare pin at loop start/end and scope it with everything active, logging
  off. Then tighten `Safety`'s `WDTO_500MS` to a measured margin.
- **Battery:** `BATTERY_SAMPLE_INTERVAL_MS`, `BATTERY_FILTER_ALPHA`.
- **UX:** `BUTTON_DEBOUNCE_MS`, `BUTTON_LONG_PRESS_MS`, `DIP_DEBOUNCE_MS`,
  `STANDBY_BLINK_INTERVAL_MS`, `RGB_BRIGHTNESS`, `BOOT_ANIMATION_STEP_MS`,
  all `RGB_*` colors, `MODE_SEQUENCES`, `MODE_GESTURE_TIMEOUT_MS`.
  Gestures turn the LEFT wheel >= one stroke of at least 1/8 revolution, then a ≥150 ms pause
- **Other:** `LOG_STREAM_INTERVAL_MS`, `TOF_POLL_INTERVAL_MS`,
  `WHEEL_PID_DERIVATIVE_FILTER_ALPHA`, `ENCODER_GLITCH_SAFETY_MARGIN`.

## Constant -> phase index
| Constants | Phase |
|---|---|
| `BATTERY_DIVIDER_*`, `ADC_REFERENCE_V`, `BATTERY_*_VOLTAGE` | 0.3 |
| `MazeConfig`, `CELL_SIZE_MM`, `RGB_LED_COUNT`, ToF addresses | 0.4 |
| `*_MOTOR_REVERSED`, `*_ENCODER_REVERSED`, `IMU_YAW_*`, `TOF_ROLE_INDEX_*`, `START_SENSOR_ROLE_INDEX`, `TOF_COVER_THRESHOLD_MM` | 1 |
| `IMU_GYRO_*`, `ENCODER_PULSES_PER_OUTPUT_REV`, `*_WHEEL_DIAMETER_MM`, `WHEEL_BASE_MM` | 2 |
| `MOTOR_DEADBAND_PWM`, wheel PID + limits, `HEADING_*`, `TURN_*`, `MOTION_SETTLE_MS`, `MOTION_TEST_FORWARD_DISTANCE_MM`, `MAX_LINEAR_*`, `*_SPEED_MM_S` | 3 |
| `TOF_OFFSET_MM_*`, `TOF_DIAGONAL_MOUNT_ANGLE_DEG`, `TOF_MIN/MAX_VALID_MM`, `FRONT_WALL_MM`, `SIDE_WALL_MAX_MM` | 4 |
| `ROBOT_CENTER_TO_REAR_MM`, `FAST_RUN_SPEED_MM_S` | 5 |
| `WALL_*`, `ENABLE_TOF_WALL_CENTERING`, `MAZESOLVER_COMPILE_TOF_WALL_CENTERING` | 6 |
| periods, watchdog, UX/RGB/button constants | 7 |
| unused: `MAX_TURN_RATE_DEG_S`, `WHEEL_TRACK_MM` | never |