# Setup / Tuning / Hardware Notes

Running log of things to verify or configure once the physical robot is available.
Add to this as we go

## Open questions — sensor layout
- ToF: 4 sensors installed, **2 facing straight forward** (not the 1-front +
  2-diagonal layout implied by FIRMWARE_SPECS.md section 14's ASCII diagram).
  `ToFManager`'s `SensorRole` enum (FRONT / DIAGONAL_LEFT / DIAGONAL_RIGHT /
  AUXILIARY) is a placeholder based on the doc's diagram — needs to be
  revisited once the real geometry (what are the OTHER two sensors doing —
  side-facing for wall-following? still diagonal?) is confirmed. Because
  ToFManager exposes roles, not sensor indices, this is a rename in ONE
  place (RobotConfig.h's TOF_ROLE_INDEX_* + the enum names) whenever we get
  there — not a rewrite.

## Architecture notes
- `Wire.begin()` must be called exactly once, in `main.cpp` `setup()`,
  before any driver that uses I2C (`ToFManager::begin()`, `IMU::begin()`).
  Do not add it inside individual driver `begin()` functions.

## To calibrate once hardware exists
- `RobotConfig::WHEEL_DIAMETER_MM`, `WHEEL_BASE_MM`, `WHEEL_TRACK_MM`
- `RobotConfig::ENCODER_PULSES_PER_OUTPUT_REV`
- `RobotConfig::BATTERY_LOW_VOLTAGE` / `BATTERY_CRITICAL_VOLTAGE`
- `UserConfig::BUTTON_DEBOUNCE_MS` / `BUTTON_LONG_PRESS_MS` (feel-check)
- `SensorConfig::FRONT_WALL_MM` / `SIDE_WALL_MAX_MM` (wall thresholds)
- IMU gyro full-scale range (`SensorConfig::IMU_GYRO_FS_SEL`) — currently
  set to ±500°/s, raise if fast-run turning exceeds that
- IMU yaw axis + sign (`RobotConfig::IMU_YAW_AXIS/IMU_YAW_SIGN`) — must be
  verified against actual mounting orientation