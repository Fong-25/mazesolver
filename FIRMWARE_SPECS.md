# MICROMOUSE ROBOT — COMPLETE FIRMWARE / PROGRAMMING SPECIFICATION
=====================================================================

This document is the **standalone programming specification** for the
Micromouse robot described by the hardware description used to build the PCB.

It intentionally includes the hardware information that firmware developers
need, so this file can be given to another programmer without requiring the
separate hardware-description document.

---

# 1. PROJECT OVERVIEW

## 1.1 Robot

Two-wheel differential-drive Micromouse / autonomous maze-solving robot.

The robot uses:

- Classic Arduino Nano / ATmega328P.
- 2 × GA12 N20 geared DC motors with Hall quadrature encoders.
- DRV8833 dual H-bridge.
- 3–4 × VL53L0X/VL53L1X ToF modules.
- 1 × MPU6500 IMU module.
- HC-06 Bluetooth module.
- WS2812B RGB LED chain.
- 1 × momentary push button.
- 1 × 2-bit DIP switch.
- Battery voltage measurement through an ADC divider.

The firmware must be designed as a **modular embedded application**, not as
one large Arduino sketch.

The core goals are:

1. Reliable hardware abstraction.
2. Closed-loop left/right motor control.
3. Encoder-based speed and distance estimation.
4. IMU-based heading estimation.
5. ToF-based wall/corridor sensing.
6. Maze exploration and map storage.
7. Re-running a known maze using a faster path.
8. Clear user-mode selection and startup state handling.
9. Easy adjustment of hardware-dependent constants without searching through
   the control code.
10. Easy replacement or extension of individual subsystems.

---

# 2. PROGRAMMING PLATFORM

The firmware must support either of these development workflows:

## 2.1 Arduino IDE

Recommended for:

- First bring-up.
- Simple sensor testing.
- Initial motor/encoder validation.
- Quick manual experiments.

The project should remain compatible with a normal Arduino Nano / ATmega328P
board definition.

## 2.2 VS Code + PlatformIO

Recommended for the main project.

PlatformIO should be the preferred long-term environment because it makes a
multi-file firmware architecture easier to maintain, test, configure, and
scale.

### IMPORTANT: All the example / placeholder code below are just for example.
Follow modern, reliable Arduino syntax, but the rules must stay the same (unless
there are specific reasons listed).

Recommended project concept:

```text
micromouse-firmware/
│
├── platformio.ini
│
├── src/
│   ├── main.cpp
│   │
│   ├── app/
│   │   ├── App.cpp
│   │   └── App.h
│   │
│   ├── config/
│   │   ├── BoardConfig.h
│   │   ├── RobotConfig.h
│   │   ├── ControlConfig.h
│   │   ├── SensorConfig.h
│   │   └── UserConfig.h
│   │
│   ├── drivers/
│   │   ├── MotorDriver.cpp
│   │   ├── MotorDriver.h
│   │   ├── Encoder.cpp
│   │   ├── Encoder.h
│   │   ├── ToFManager.cpp
│   │   ├── ToFManager.h
│   │   ├── IMU.cpp
│   │   ├── IMU.h
│   │   ├── RGB.cpp
│   │   ├── RGB.h
│   │   ├── Button.cpp
│   │   ├── Button.h
│   │   ├── DipSwitch.cpp
│   │   ├── DipSwitch.h
│   │   ├── BatteryMonitor.cpp
│   │   └── BatteryMonitor.h
│   │
│   ├── control/
│   │   ├── PID.cpp
│   │   ├── PID.h
│   │   ├── MotorControl.cpp
│   │   ├── MotorControl.h
│   │   ├── PoseEstimator.cpp
│   │   └── PoseEstimator.h
│   │
│   ├── navigation/
│   │   ├── Maze.cpp
│   │   ├── Maze.h
│   │   ├── FloodFill.cpp
│   │   ├── FloodFill.h
│   │   ├── Explorer.cpp
│   │   ├── Explorer.h
│   │   ├── PathPlanner.cpp
│   │   ├── PathPlanner.h
│   │   ├── FastRun.cpp
│   │   └── FastRun.h
│   │
│   ├── modes/
│   │   ├── ModeManager.cpp
│   │   ├── ModeManager.h
│   │   ├── SettingMode.cpp
│   │   ├── SettingMode.h
│   │   ├── StandbyMode.cpp
│   │   └── StandbyMode.h
│   │
│   ├── system/
│   │   ├── Scheduler.cpp
│   │   ├── Scheduler.h
│   │   ├── Safety.cpp
│   │   ├── Safety.h
│   │   ├── Diagnostics.cpp
│   │   └── Diagnostics.h
│   │
│   └── communication/
│       ├── Bluetooth.cpp
│       └── Bluetooth.h
│
└── lib/
    └── (optional local libraries)
```

The exact file names may change, but the rule should remain:

> **One module = one responsibility.**

Do not put all hardware drivers, PID, maze logic, state machines, and constants
inside `main.cpp`.

---

# 3. FIRMWARE ARCHITECTURE

The firmware should be layered.

```text
Application / State Machine
            |
            v
Navigation / Modes / User Interaction
            |
            v
Control Algorithms
            |
            v
Hardware Drivers
            |
            v
Arduino / ATmega328P Hardware
```

## 3.1 Hardware layer

Responsible only for interfacing with physical devices.

Examples:

- `MotorDriver`
- `Encoder`
- `ToFManager`
- `IMU`
- `RGB`
- `Button`
- `DipSwitch`
- `BatteryMonitor`
- `Bluetooth`

Hardware drivers should not contain maze-solving decisions.

Example:

```cpp
MotorDriver.setLeftPWM(120);
```

is appropriate.

```cpp
MotorDriver.solveMaze();
```

is not.

## 3.2 Control layer

Responsible for continuous motion control.

Examples:

- Wheel velocity PID.
- Wheel distance tracking.
- Straight-line correction.
- Heading correction.
- Turning control.
- Odometry.
- Sensor-based wall correction.

## 3.3 Navigation layer

Responsible for understanding the maze.

Examples:

- Current cell.
- Current orientation.
- Wall map.
- Visited map.
- Flood-fill values.
- Exploration decisions.
- Shortest/fast path generation.

The explored maze must survive modes change 
(see below for changing mode). But it should 
be removed if the robot is turned off. This act
as a way to reset the maze if error occured.

## 3.4 Application layer

Responsible for robot behavior as a whole.

Examples:

- Startup.
- Sensor initialization.
- Settings selection.
- Standby.
- Running.
- Goal reached.
- Error state.
- User abort.

---

# 4. HARDWARE / PIN REFERENCE

This section is intentionally included here so the firmware document is
standalone.

## 4.1 MCU

- Arduino Nano.
- ATmega328P.
- 5 V logic system.

## 4.2 Recommended pinout

| Nano Pin | Function |
|---|---|
| D0 / RX | HC-06 TX |
| D1 / TX | HC-06 RX |
| D2 | Left encoder C1 |
| D3 | Left encoder C2 |
| D4 | Right encoder C1 |
| D5 | DRV8833 AIN2 |
| D6 | DRV8833 AIN1 |
| D7 | Right encoder C2 |
| D8 | **SW1 momentary push button** |
| D9 | DRV8833 BIN1 |
| D10 | DRV8833 BIN2 |
| D11 | WS2812B data |
| D12 | ToF #1 XSHUT |
| D13 | ToF #2 XSHUT |
| A0 | Spare ADC |
| A1 | ToF #3 XSHUT |
| A2 | ToF #4 XSHUT |
| A3 | **SW2 DIP bit 2 — SETTING** |
| A4 | I2C SDA |
| A5 | I2C SCL |
| A6 | Spare ADC |
| A7 | Battery voltage ADC |

### Important

`SW2 DIP bit 1` is the **true power switch** and does not connect to a Nano
GPIO.

Therefore:

- SW2 bit 1 OFF -> robot electronics physically unpowered.
- SW2 bit 1 ON -> robot electronics powered.
- Firmware does not read bit 1.

`SW2 DIP bit 2` is the firmware-controlled **SETTING switch**.

- A3 -> SW2 bit 2 -> GND.
- Use `INPUT_PULLUP`.
- OFF/open -> HIGH.
- ON/closed -> LOW.

`SW1` is the separate momentary push button:

- D8 -> button -> GND.
- Use `INPUT_PULLUP`.
- Released -> HIGH.
- Pressed -> LOW.

This naming is intentional and must be kept consistent in firmware.

---

# 5. CONFIGURATION PHILOSOPHY

Any value that may change because of:

- PCB revisions.
- Motor wiring.
- Encoder wiring.
- Left/right motor orientation.
- Mechanical dimensions.
- PID tuning.
- Sensor mounting.
- IMU mounting orientation.
- Battery configuration.
- Different module versions.
- Calibration.
- Competition tuning.

must be stored in a dedicated configuration location.

Do **not** bury such values inside control algorithms.

---

# 6. CONFIGURATION FILES

Use small, meaningful configuration headers.

## 6.1 BoardConfig.h

Contains physical pin assignments.

Example:

```cpp
#pragma once

namespace Board
{
    constexpr uint8_t PIN_LEFT_ENC_A  = 2;
    constexpr uint8_t PIN_LEFT_ENC_B  = 3;
    constexpr uint8_t PIN_RIGHT_ENC_A = 4;
    constexpr uint8_t PIN_RIGHT_ENC_B = 7;

    constexpr uint8_t PIN_LEFT_IN1  = 6;
    constexpr uint8_t PIN_LEFT_IN2  = 5;
    constexpr uint8_t PIN_RIGHT_IN1 = 9;
    constexpr uint8_t PIN_RIGHT_IN2 = 10;

    constexpr uint8_t PIN_BUTTON = 8;
    constexpr uint8_t PIN_RGB    = 11;

    constexpr uint8_t PIN_TOF_XSHUT_1 = 12;
    constexpr uint8_t PIN_TOF_XSHUT_2 = 13;
    constexpr uint8_t PIN_TOF_XSHUT_3 = A1;
    constexpr uint8_t PIN_TOF_XSHUT_4 = A2;

    constexpr uint8_t PIN_SETTING_SWITCH = A3;
    constexpr uint8_t PIN_BATTERY_ADC    = A7;

    constexpr uint8_t PIN_I2C_SDA = A4;
    constexpr uint8_t PIN_I2C_SCL = A5;
}
```

Only board-level pin changes belong here.

## 6.2 RobotConfig.h

Contains mechanical and polarity settings.

Example:

```cpp
#pragma once

namespace RobotConfig
{
    // -------- Motor / encoder polarity --------

    constexpr bool LEFT_MOTOR_REVERSED  = false;
    constexpr bool RIGHT_MOTOR_REVERSED = true;

    constexpr bool LEFT_ENCODER_REVERSED  = false;
    constexpr bool RIGHT_ENCODER_REVERSED = true;

    // -------- Mechanical geometry --------

    constexpr float WHEEL_DIAMETER_MM = 34.0f;

    // Center-to-center distance between left and right wheels.
    constexpr float WHEEL_BASE_MM = 72.0f;

    // Axle position / wheel geometry can be calibrated separately.
    constexpr float WHEEL_TRACK_MM = 72.0f;

    // -------- Encoder --------

    constexpr float ENCODER_PULSES_PER_OUTPUT_REV = 350.0f;

    // Using x4 quadrature decoding.
    constexpr float ENCODER_COUNTS_PER_OUTPUT_REV =
        ENCODER_PULSES_PER_OUTPUT_REV * 4.0f;

    // -------- Motion limits --------

    constexpr float MAX_LINEAR_SPEED_MM_S = 800.0f;
    constexpr float MAX_LINEAR_ACCEL_MM_S2 = 1500.0f;

    constexpr float MAX_TURN_RATE_DEG_S = 500.0f;
}
```

The numerical values here are **starting placeholders unless mechanically
confirmed**.

The actual wheel diameter and wheel-base/track must be measured from the
assembled robot.

## 6.3 ControlConfig.h

All PID parameters belong here.

```cpp
#pragma once

namespace ControlConfig
{
    // Wheel velocity PID
    constexpr float LEFT_KP  = 1.0f;
    constexpr float LEFT_KI  = 0.0f;
    constexpr float LEFT_KD  = 0.0f;

    constexpr float RIGHT_KP = 1.0f;
    constexpr float RIGHT_KI = 0.0f;
    constexpr float RIGHT_KD = 0.0f;

    // Heading PID
    constexpr float HEADING_KP = 1.0f;
    constexpr float HEADING_KI = 0.0f;
    constexpr float HEADING_KD = 0.0f;

    // Wall centering
    constexpr float WALL_KP = 1.0f;
    constexpr float WALL_KI = 0.0f;
    constexpr float WALL_KD = 0.0f;
}
```

Do not tune constants in `MotorControl.cpp`, `Explorer.cpp`, or random sections
of `main.cpp`.

## 6.4 SensorConfig.h

```cpp
#pragma once

namespace SensorConfig
{
    constexpr uint8_t TOF_DEFAULT_ADDRESS = 0x29;

    constexpr uint8_t TOF_ADDR_1 = 0x30;
    constexpr uint8_t TOF_ADDR_2 = 0x31;
    constexpr uint8_t TOF_ADDR_3 = 0x32;
    constexpr uint8_t TOF_ADDR_4 = 0x33;

    constexpr uint32_t I2C_CLOCK_HZ = 100000;

    // Example sensor validity limits.
    constexpr uint16_t TOF_MIN_VALID_MM = 20;
    constexpr uint16_t TOF_MAX_VALID_MM = 2000;

    // Wall thresholds must be calibrated on the real robot.
    constexpr uint16_t FRONT_WALL_MM = 90;
    constexpr uint16_t SIDE_WALL_MAX_MM = 120;
}
```

## 6.5 UserConfig.h

Contains settings that are likely to change during normal robot development.

Examples:

- Mode-selection sequence.
- Button hold time.
- Standby indication rate.
- RGB colors.
- Goal coordinates.
- Exploration behavior.
- Fast-run speed.
- Sensor-covered start threshold.

---

# 7. HARDWARE-SPECIFIC VALUES THAT MUST BE CONFIGURABLE

At minimum, make all of these constants configurable:

### Motor / encoder

- Left motor polarity.
- Right motor polarity.
- Left encoder polarity.
- Right encoder polarity.
- Encoder counts per wheel revolution.
- PWM limits.
- Minimum motor PWM.
- Maximum motor PWM.
- Brake/coast behavior.
- Motor deadband compensation.

### Mechanical

- Wheel diameter.
- Wheel circumference.
- Wheel-to-wheel center distance.
- Effective wheel track.
- Maze-cell size.
- Robot center offset if required.
- Sensor offsets from robot center.
- Sensor angles if used by geometry calculations.

### Motion control

- Wheel velocity PID.
- Straight-line PID.
- Heading PID.
- Turn PID.
- Acceleration limit.
- Deceleration limit.
- Maximum velocity.
- Maximum turn rate.
- Turn completion tolerance.
- Encoder distance tolerance.

### IMU

- IMU axis mapping.
- X/Y/Z sign inversion.
- Gyro axis used as yaw.
- Gyro sign.
- Accelerometer sign.
- Calibration offsets.
- Gyro bias.
- Sensor update rate.
- Complementary-filter coefficient.

### ToF

- I2C addresses.
- XSHUT pins.
- Sensor role assignment.
- Timeout.
- Measurement timing.
- Valid-distance limits.
- Front wall threshold.
- Left wall threshold.
- Right wall threshold.
- Diagonal sensor thresholds.
- Sensor mounting offsets.

### Battery

- Divider R1.
- Divider R2.
- ADC reference.
- Battery low-voltage threshold.
- Battery critical threshold.

### User interface

- Button hold duration.
- Start trigger duration.
- RGB colors.
- Blink rate.
- Mode-selection sequence.

---

# 8. MOTOR / ENCODER ABSTRACTION

The physical connector can be manually rewired, so firmware must never assume
that physical motor direction automatically equals logical robot direction.

The firmware should expose:

```cpp
setLeftMotorReversed(...)
setRightMotorReversed(...)
setLeftEncoderReversed(...)
setRightEncoderReversed(...)
```

or equivalent compile-time constants.

The driver should convert:

```text
requested logical direction
        |
        v
motor polarity correction
        |
        v
DRV8833 PWM / direction output
```

Encoder counts must pass through the corresponding encoder polarity correction
before being used by odometry or PID.

## 8.1 Example

If the left motor physically spins backward when the program requests
"forward", change:

```cpp
constexpr bool LEFT_MOTOR_REVERSED = true;
```

Do not reverse random signs inside the PID code.

Likewise, if the encoder's A/B orientation causes positive physical motion to
produce negative counts:

```cpp
constexpr bool LEFT_ENCODER_REVERSED = true;
```

---

# 9. ENCODER PROCESSING

The motors provide Hall A/B quadrature signals.

Given:

- 350 pulses/channel/output-shaft revolution.
- x4 quadrature decoding.

Nominal software resolution is:

```text
350 × 4 = 1400 counts/output-shaft revolution
```

The maximum no-load motor speed listed for the motor is approximately
300 RPM at 6 V.

Approximate count rate:

```text
350 × 300 / 60 = 1750 pulses/s/channel

1750 × 4 = 7000 counts/s/motor
```

Two motors therefore produce approximately:

```text
14,000 quadrature counts/s total
```

The ATmega328P can handle this when the interrupt/edge handling remains
lightweight.

## 9.1 ISR rule

Encoder interrupt routines must do as little as possible.

Good ISR responsibilities:

- Read the relevant encoder state.
- Determine direction.
- Increment/decrement a volatile count.

Avoid inside ISR:

- Serial printing.
- Floating-point calculations.
- I2C transactions.
- PID calculations.
- Maze logic.
- RGB updates.
- Long loops.

## 9.2 Noise handling

Motor PWM can introduce spurious edges.

Possible firmware protections:

1. Gray-code/state transition validation.
2. Reject impossible quadrature transitions.
3. Optional minimum-edge-time filter.
4. Simple debounce/state filter.
5. Require 2 consistent reads before accepting an edge.
6. Cross-check measured speed against physically possible speed.
7. Log counts during aggressive motor operation.
8. Bench-test both motors under high load and verify encoder counts 
remain clean.

Do not use a long software debounce delay that destroys genuine high-speed
encoder transitions.

---

# 10. MOTOR CONTROL

The robot should use **closed-loop wheel velocity control** rather than
simply applying a fixed PWM.

Basic architecture:

```text
Target left speed  ----\
                        +--> Left PID --> Motor Driver --> Left Motor
Measured left speed ---/

Target right speed ----\
                         +--> Right PID --> Motor Driver --> Right Motor
Measured right speed ----/
```

## 10.1 PID loop

Recommended control period:

```text
1–5 ms
```

The exact value should be selected after profiling CPU load and encoder
processing.

The PID module should support:

```cpp
update(target, measurement, dt)
```

and provide:

- Integral limiting.
- Output limiting.
- Optional derivative filtering.
- Reset function.
- Enable/disable.
- Sign-safe behavior.

## 10.2 Motor deadband compensation

Small DC motors may not move at low PWM values.

A configurable minimum-output compensation may therefore be used:

```text
PID output
    |
    +-- deadband compensation
    |
    +-- clamp to PWM range
    |
    v
DRV8833
```

Keep this in configuration.

---

# 11. DIFFERENTIAL-DRIVE ODOMETRY

Let:

- `dL` = left wheel distance.
- `dR` = right wheel distance.
- `B` = effective wheel-base / track.

Then:

```text
dCenter = (dR + dL) / 2
dTheta  = (dR - dL) / B
```

Update pose:

```text
x += dCenter * cos(theta + dTheta / 2)
y += dCenter * sin(theta + dTheta / 2)

theta += dTheta
```

Use radians internally.

Because mechanical tolerances affect turning precision, the effective
wheel-base must be calibrated experimentally and stored in `RobotConfig.h`.

---

# 12. IMU ORIENTATION CONFIGURATION

The IMU module may be installed in a different orientation after a PCB
revision or mechanical rebuild.

Never hard-code assumptions such as:

```cpp
yawRate = imu.gyroZ;
```

inside the control algorithm.

Instead use configurable axis mapping.

Conceptually:

```cpp
struct ImuAxisConfig
{
    Axis yawAxis;
    int8_t yawSign;
    Axis pitchAxis;
    int8_t pitchSign;
    Axis rollAxis;
    int8_t rollSign;
};
```

Example configuration:

```cpp
constexpr Axis YAW_AXIS = Axis::Z;
constexpr int8_t YAW_SIGN = +1;
```

The final axis/sign values must be verified on the assembled robot.

## 12.1 Heading estimation

For a first implementation:

- Use gyro yaw rate for short-term turn control.
- Fuse or correct heading with encoder odometry.
- Perform gyro bias calibration while stationary.

A complementary filter can be introduced later.

The architecture should allow replacing the estimator without rewriting the
navigation code.

---

# 13. TOF SENSOR ARCHITECTURE

The robot uses 3–4 VL53L0X/VL53L1X module boards.

All sensors share:

```text
A4 = SDA
A5 = SCL
```

Individual XSHUT lines:

```text
ToF #1 -> D12
ToF #2 -> D13
ToF #3 -> A1
ToF #4 -> A2
```

Addresses:

```text
ToF #1 -> 0x30
ToF #2 -> 0x31
ToF #3 -> 0x32
ToF #4 -> 0x33
```

Typical default address:

```text
0x29
```

## 13.1 Initialization

On startup:

1. Configure all XSHUT pins as outputs.
2. Drive all XSHUT pins LOW.
3. Wait briefly for all sensors to shut down.
4. Enable sensor 1.
5. Initialize sensor 1 at the default address.
6. Change sensor 1 to `0x30`.
7. Enable sensor 2.
8. Initialize sensor 2.
9. Change it to `0x31`.
10. Repeat for sensor 3 and sensor 4.
11. Start normal ranging.

Never bring all identical-address sensors online before giving them unique
addresses.

## 13.2 Logical sensor names

Do not make navigation code depend on "sensor #2".

Instead define logical roles:

```cpp
enum class SensorRole
{
    FRONT,
    DIAGONAL_LEFT,
    DIAGONAL_RIGHT,
    AUXILIARY
};
```

Then configure which physical ToF address belongs to each role.

This allows the PCB wiring or sensor population to change without rewriting
maze logic.

---

# 14. WALL SENSOR GEOMETRY

Primary sensing arrangement:

```text
          FRONT OF ROBOT

          \    |    /
           \   |   /
            DL F DR

          [ROBOT]
```

The intended roles are:

- `FRONT`: directly ahead.
- `DIAGONAL_LEFT`: angled toward the left wall.
- `DIAGONAL_RIGHT`: angled toward the right wall.

The diagonal sensors are especially useful for:

- Looking ahead while moving quickly.
- Detecting approaching corners.
- Detecting wall openings earlier.
- Estimating heading error.
- Comparing left/right corridor geometry.

## 14.1 Wall error

A basic diagonal-heading error can be estimated from the difference between
left and right measurements:

```text
error = distance_right - distance_left
```

The exact mathematical relationship depends on sensor angle and physical
mounting, so the conversion from raw distance difference to steering correction
must be calibrated on the real robot.

Do not assume the raw difference in millimetres is already a perfect angular
error.

---

# 15. I2C

Because the bus capacitance / pull-up arrangement may be relatively heavy,
use:

```cpp
Wire.setClock(100000);
```

unless the final hardware is measured and validated for faster operation.

Target:

```text
I2C = 100 kHz
```

Keep sensor polling scheduled rather than blocking the entire firmware.

One slow I2C operation should not freeze:

- Motor control.
- Encoder processing.
- Safety.
- Button handling.

---

# 16. BATTERY MONITORING

Hardware:

```text
Battery +
   |
  R1
   |
   +---- A7
   |
  R2
   |
  GND
```

Example divider:

```text
R1 = 10 kΩ
R2 = 6.8 kΩ
```

At 8.4 V battery voltage, the ADC node is approximately:

```text
3.40 V
```

These resistor values are planned to use on the final robot, 
but the exact battery-voltage calculation must use configurable 
resistor values rather than a hard-coded multiplier.

Example:

```cpp
float adcVoltage = adcRaw * ADC_REFERENCE_V / ADC_MAX_COUNTS;

float batteryVoltage =
    adcVoltage * (R1 + R2) / R2;
```

Add filtering in software.

Suggested strategy:

- Read periodically.
- Use a small moving average or exponential filter.
- Apply low-voltage warning thresholds.
- Stop or enter safe state at the critical threshold.

Thresholds must be configurable.

---

# 17. RGB STATUS SYSTEM

The WS2812B is the primary user-visible status indicator.

Do not allow arbitrary modules to directly manipulate the LED.

Use a centralized status controller:

```cpp
RgbStatus.set(Status::SETTING);
RgbStatus.set(Status::STANDBY);
RgbStatus.set(Status::EXPLORING);
RgbStatus.set(Status::FAST_RUN);
RgbStatus.set(Status::ERROR);
```

Suggested state meanings:

| State | RGB behavior |
|---|---|
| Booting | short startup animation |
| Setting | solid selected-mode color |
| Standby | selected-mode color blinking |
| Exploring | dedicated run color |
| Map stored / exploration complete | completion indication |
| Fast run | dedicated fast-run color |
| Finished | completion indication |
| Error | red or other error indication |

The exact colors are configuration, not hard-coded throughout the program.

---

# 18. USER CONTROL / OPERATING STATE MACHINE

The robot has three relevant user controls:

## SW2 DIP bit 1

True electrical system power.

```text
OFF -> no firmware execution
ON  -> system powered
```

The firmware does not read this switch.

## SW2 DIP bit 2

SETTING switch:

```text
OFF -> normal locked mode
ON  -> mode-selection state
```

GPIO:

```text
A3
```

Use:

```cpp
pinMode(A3, INPUT_PULLUP);
```

## SW1 momentary button

GPIO:

```text
D8
```

Use:

```cpp
pinMode(D8, INPUT_PULLUP);
```

The button is used to enter STANDBY after a long press and may also be used
for future user actions such as abort/reset.

---

# 19. MAIN APPLICATION STATES

Use an explicit state machine.

Minimum states:

```text
BOOT
  |
  v
INITIALIZE
  |
  v
SETTING / LOCKED
  |
  v
STANDBY
  |
  +----> EXPLORATION
  |
  +----> FAST_RUN
  |
  +----> DIAGNOSTIC / CALIBRATION
  |
  v
FINISHED
```

Error handling should be able to transition into:

```text
ERROR
```

without directly resetting the MCU.

---

# 20. BOOT SEQUENCE

Recommended startup order:

```text
1. MCU startup
2. Configure GPIO
3. Stop motor outputs
4. Initialize encoder hardware
5. Initialize I2C at 100 kHz
6. Initialize ToF sensors one-by-one
7. Initialize IMU
8. Initialize RGB
9. Initialize button
10. Initialize setting switch
11. Initialize battery monitor
12. Initialize Bluetooth
13. Load persistent maze/configuration data
14. Perform self-check
15. Enter user selection / locked idle state
```

At no point should motor outputs become active merely because the MCU has
booted.

---

# 21. SETTING MODE

## 21.1 Purpose

When `SW2 DIP bit 2` is ON:

```text
SETTINGS = ACTIVE
```

The robot must not start autonomous motion.

The user can select the operating mode using a deliberate wheel-movement
sequence.

The selected mode is shown using a **solid RGB color**.

## 21.2 Mode selector input

Use one wheel only for mode-selection gestures.

The firmware should detect:

- Direction.
- Approximate movement size.
- Separation between movements.
- End of a gesture.
- Timeout between sequence elements.

The gesture should be recognized from encoder movement, not from arbitrary
PWM commands.

Suggested abstraction:

```cpp
enum class WheelGesture
{
    FORWARD,
    BACKWARD
};
```

A gesture is emitted after the selected wheel moves by a configurable
minimum encoder count. Recommended a quarter of wheel revolution for 
each gesture, after encoder calibration for high precision.

## 21.3 Example selection protocol

A simple extensible default sequence table:

```text
Mode 0 — EXPLORE
    FORWARD, FORWARD, BACKWARD

Mode 1 — FAST RUN
    FORWARD, BACKWARD, FORWARD

Mode 2 — DIAGNOSTIC
    BACKWARD, BACKWARD, FORWARD
```

The sequence table is only a firmware-level default and must be stored in a
configuration file so it can be changed without rewriting the state machine.

For example:

```cpp
constexpr WheelGesture MODE_EXPLORE[] =
{
    WheelGesture::FORWARD,
    WheelGesture::FORWARD,
    WheelGesture::BACKWARD
};
```

The selector should provide feedback after a valid sequence by leaving the
corresponding mode color continuously ON.

## 21.4 Selection rules

While setting is active:

- Motors must remain disabled for normal motion.
- Only the selected wheel is accepted as a mode-input source.
- Accidental tiny movements are ignored using a configurable minimum count.
- A gesture timeout resets an incomplete sequence.
- Invalid sequences should either reset or return to the nearest valid state.
- The selected mode must remain visible using solid RGB.

---

# 22. SETTING LOCK

When the user switches:

```text
SW2 DIP bit 2: ON -> OFF
```

the selected mode becomes locked.

Example:

```text
SETTING ACTIVE
      |
      | DIP bit 2 OFF
      v
SELECTED MODE LOCKED
```

The RGB remains **solid with the selected mode color**.

At this point the robot still must not move automatically.

The robot is waiting for the user to arm it.

---

# 23. STANDBY ENTRY

After a mode has been selected and locked:

If `SW1` (the push button) is pressed and held for approximately:

```text
3 seconds
```

the system enters `STANDBY`.

The hold duration must be configurable.

RGB behavior:

```text
selected mode color
        +
     blinking
```

The blink must use the same selected mode color rather than replacing it
with another arbitrary color.

Example:

```text
Fast Run selected
-> solid FAST-RUN color

Button held 3 s
-> blinking FAST-RUN color
-> STANDBY
```

---

# 24. RUN START TRIGGER

In STANDBY, the robot waits for a deliberate ToF start gesture.

Required behavior:

```text
ToF sensor covered
        |
        v
ToF sees object very close
        |
        v
User releases / uncovers sensor
        |
        v
RUN
```

Therefore:

1. Enter STANDBY.
2. Robot waits.
3. Detect a valid cover event.
4. Require the sensor to subsequently return to its normal/open state.
5. Start the selected mode only after the release event.

This prevents the robot from starting immediately when the user enters
standby.

The start sensor and cover threshold must be configurable.

Recommended state machine:

```text
WAIT_FOR_COVER
      |
      | valid cover
      v
COVERED
      |
      | valid release
      v
RUN
```

---

# 25. COMPLETE USER FLOW

Normal operation:

```text
POWER OFF
   |
   | SW2 bit 1 ON
   v
BOOT
   |
   v
INITIALIZE
   |
   v
IDLE / WAIT FOR SETTING
   |
   | SW2 bit 2 ON
   v
SETTING MODE
   |
   | wheel gesture sequence
   v
MODE SELECTED
   |
   | solid RGB
   |
   | SW2 bit 2 OFF
   v
MODE LOCKED
   |
   | hold SW1 for 3 s
   v
STANDBY
   |
   | cover ToF
   v
COVERED
   |
   | release ToF
   v
SELECTED RUN MODE
```

The robot must never start driving simply because:

- Power was turned on.
- A mode was selected.
- The setting switch was turned off.
- The button was pressed briefly.
- The ToF was already covered before entering standby.

A valid cover-then-release sequence is required after entering STANDBY.

---

# 26. OPERATING MODES

The firmware should include at least these conceptual modes.

## MODE 0 — EXPLORE

Purpose:

- Explore an unknown maze.
- Build a wall map.
- Track the robot's position and orientation.
- Determine reachable paths.
- Update the maze model as new walls are observed.
- Continue until the goal condition is satisfied.
- Store the learned map.
- Go back to the starting point.

Core components:

```text
ToF + encoders + IMU
        |
        v
Wall detection + pose
        |
        v
Maze map
        |
        v
Flood fill / path decision
        |
        v
Motion controller
```

The exploration algorithm should not be tied directly to motor PWM.

---

# 27. MAZE MODEL

A classic Micromouse maze can be represented as a grid of cells.

A common target is:

```text
16 × 16 cells
```

but maze dimensions must be configurable.

Suggested constants:

```cpp
constexpr uint8_t MAZE_WIDTH  = 16;
constexpr uint8_t MAZE_HEIGHT = 16;
```

## 27.1 Per-cell information

At minimum store:

- North wall.
- East wall.
- South wall.
- West wall.
- Visited flag.
- Optional goal flag.

Use compact bit fields to conserve ATmega328P RAM.

Example:

```text
bit 0 = NORTH wall
bit 1 = EAST wall
bit 2 = SOUTH wall
bit 3 = WEST wall
bit 4 = visited
bit 5 = confirmed
```

Keep the representation compact because the ATmega328P has limited RAM.

---

# 28. MAP STORAGE

There are two different concepts:

## Runtime map

Lives in RAM while the robot is operating.

Contains:

- Current known walls.
- Visited cells.
- Flood-fill values.
- Current planned path.

## Persistent map

May be stored in EEPROM after a successful exploration.

Purpose:

- Preserve a learned maze between power cycles.
- Allow a later fast run without exploring again.

Do not continuously write EEPROM during every sensor update.

EEPROM has finite write endurance.

Recommended strategy:

```text
Explore in RAM
    |
    v
Goal reached / exploration complete
    |
    v
Validate map
    |
    v
Save one version to EEPROM
```

Use a version number / checksum so corrupted or unrelated EEPROM contents are
not mistaken for a valid maze.

---

# 29. EXPLORATION ALGORITHM

Use a maintainable graph/grid approach.

Recommended first implementation:

## Flood-fill exploration

At each cell:

1. Read front/diagonal/available wall information.
2. Convert sensor observations into logical walls.
3. Update the maze map.
4. Recompute or update flood-fill distances.
5. Determine reachable neighboring cells.
6. Select the next target cell.
7. Generate a motion command.
8. Execute the motion using closed-loop control.
9. Update robot cell/orientation.
10. Repeat.

Tie-breaking should favor behavior that improves exploration rather than
simply minimizing current movement.

A simple first priority can be:

```text
1. Unvisited reachable cell.
2. Lowest flood-fill distance.
3. Least recently visited / tie-break rule.
4. Safe fallback direction.
```

---

# 30. GOAL CONDITION

The goal must be configurable.

For a standard Micromouse implementation, the goal may be the center region.

Support either:

```cpp
isGoalCell(x, y)
```

or a goal-region definition.

Do not hard-code "cell 7,7" throughout the algorithm.

---

# 31. FAST RUN

After the maze has been explored and stored, the robot can perform a fast
run on the known maze.

Purpose:

- Use the stored map.
- Generate a more efficient path.
- Run at higher speed.
- Use aggressive but controlled acceleration/deceleration.
- Use known turns rather than exploring.

Fast run should **not** perform the same decision-making loop as exploration.

Recommended flow:

```text
Load map
   |
   v
Generate best known path
   |
   v
Compress path / merge straight segments
   |
   v
Generate motion primitives
   |
   v
Execute high-speed run
```

---

# 32. PATH REPRESENTATION

Use a direction sequence internally:

```text
N E E S S E N ...
```

or relative commands:

```text
STRAIGHT
LEFT
RIGHT
UTURN
```

For execution, relative motion primitives are often easier:

```cpp
enum class MotionCommand
{
    CELL_FORWARD,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_180,
    FAST_STRAIGHT,
    STOP
};
```

A path can then be converted from grid directions into motion commands.

---

# 33. FAST-RUN PATH OPTIMIZATION

The fast-run path should remove unnecessary pauses.

Possible optimization:

```text
Forward
Forward
Forward
```

becomes:

```text
Forward × 3 cells
```

and can be executed as one longer controlled segment.

Further optimization may later combine compatible sequences around turns.

The first implementation should prioritize **reliability and determinism** over
maximum speed.

---

# 34. MOTION PRIMITIVES

The navigation layer should not directly control PWM.

It should request motion primitives.

Examples:

```cpp
moveForwardCell();
turnLeft90();
turnRight90();
turn180();
stop();
```

Each primitive is implemented by the control layer.

## 34.1 Forward cell

Control using:

- Encoder distance.
- Wheel velocity PID.
- Heading correction.
- Optional wall/diagonal correction.

## 34.2 90-degree turn

Use:

- Encoder differential motion.
- Gyro angle/rate.
- Final heading tolerance.
- Velocity profile.

The wheel-base constant is critical here.

---

# 35. PRECISE TURNING

Because this is a differential-drive robot, turn accuracy depends strongly on:

- Wheel diameter.
- Wheel-to-wheel distance.
- Encoder scale.
- Motor matching.
- Wheel slip.
- Floor conditions.
- Gyro calibration.

Do not tune turning only by changing the PWM.

Recommended control hierarchy:

```text
Desired angle
      |
      v
Turn controller
      |
      +------ gyro heading feedback
      |
      +------ encoder differential feedback
      |
      v
Left/right wheel targets
      |
      v
Wheel PID
      |
      v
Motor driver
```

The actual effective wheel-base should be measured by performing repeated
controlled rotations and adjusting:

```cpp
WHEEL_BASE_MM
```

until the measured angle matches the commanded angle.

---

# 36. STRAIGHT-LINE CONTROL

A straight segment should not simply use:

```text
left PWM = right PWM
```

Instead:

```text
target speed
     |
     +----> left wheel PID
     |
     +----> right wheel PID

heading error / wall error
     |
     +----> differential correction
```

Example:

```text
leftTarget  = baseSpeed - correction
rightTarget = baseSpeed + correction
```

The sign must be verified physically and made configurable if the final motor
orientation differs.

---

# 37. WALL FOLLOWING / CORRIDOR CONTROL

During a straight corridor segment, the robot may use diagonal ToF sensors
to reduce yaw error.

Basic structure:

```text
diagonal-left distance
          |
          +----\
                > heading correction
          +----/
          |
diagonal-right distance
```

The controller should use valid-data checks.

If a diagonal sensor loses the wall:

- Do not treat an invalid/very large reading as a huge angular error.
- Mark that measurement as unavailable.
- Fall back to other sensors / IMU / encoders.

---

# 38. SENSOR FUSION STRATEGY

Avoid making one sensor responsible for the entire pose solution.

Use:

### Encoders

Best for:

- Distance traveled.
- Wheel speed.
- Short-term odometry.

### IMU

Best for:

- Angular rate.
- Short-term heading changes.
- Turn completion.

### ToF

Best for:

- Wall presence.
- Corridor geometry.
- Corner detection.
- Wall-relative correction.

The navigation pose should be based on a controlled combination of these
sources.

---

# 39. BUTTON HANDLING

The push button is `SW1` and is on D8.

Use debouncing.

Required events:

```text
PRESS
RELEASE
LONG_PRESS
```

Long press threshold:

```text
~3000 ms
```

but keep it configurable.

The button system should not block the firmware while waiting.

Do not write:

```cpp
delay(3000);
```

to detect a long press.

Use timestamps instead.

---

# 40. DIP SWITCH HANDLING

Only:

```text
SW2 bit 2 -> A3
```

is software-visible.

Use:

```cpp
pinMode(A3, INPUT_PULLUP);
```

Debounce or stability-filter the switch before changing state.

The setting switch should cause deterministic state transitions:

```text
OFF -> ON
    enter Setting Mode

ON -> OFF
    lock selected mode
```

Avoid repeatedly re-entering Setting Mode every loop while the switch remains
ON.

Trigger on an actual state transition.

---

# 41. NON-BLOCKING FIRMWARE

The main firmware loop should be cooperative and time-based.

Avoid large blocking calls such as:

```cpp
delay(1000);
```

especially during normal robot operation.

Instead schedule tasks:

```text
1 kHz-ish:
    motor control

hundreds of Hz:
    encoder processing / pose

tens to hundreds of Hz:
    IMU

tens of Hz:
    ToF

tens of Hz:
    UI / buttons

lower rate:
    battery

as required:
    Bluetooth
```

The exact rates should be measured against ATmega328P CPU and I2C load.

---

# 42. MAIN LOOP STRUCTURE

Conceptual:

```cpp
void loop()
{
    uint32_t now = micros();

    Encoder::service();
    Safety::update(now);

    if (scheduler.controlReady(now))
    {
        MotorControl::update();
        PoseEstimator::update();
    }

    if (scheduler.imuReady(now))
    {
        IMU::update();
    }

    if (scheduler.tofReady(now))
    {
        ToFManager::update();
    }

    Button::update(now);
    DipSwitch::update(now);
    BatteryMonitor::update(now);
    Bluetooth::update();

    ModeManager::update();
}
```

The exact implementation can change, but responsibilities should remain
separated.

---

# 43. SAFETY SYSTEM

Safety must override navigation.

Possible immediate stop conditions:

- Critical battery voltage.
- Sensor subsystem failure.
- IMU initialization failure if required for the selected mode.
- Encoder failure.
- Impossible pose / speed condition.
- Motor-control watchdog timeout.
- Explicit user abort.
- Severe software fault.

Priority:

```text
Safety
  >
Motor control
  >
Motion command
  >
Navigation
  >
User interface / telemetry
```

When a safety stop occurs:

1. Stop motor outputs.
2. Record the error.
3. Show error through RGB.
4. Optionally report over Bluetooth.
5. Remain stationary until a clear recovery action occurs.

---

# 44. WATCHDOG / FAIL-SAFE

The system should use a watchdog where practical.

The code should ensure that a temporary fault does not leave motors running
indefinitely.

A healthy control loop should regularly service the watchdog.

Do not kick the watchdog from only one deeply nested subsystem without checking
that the whole system is healthy.

---

# 45. BLUETOOTH

HC-06 is connected to hardware UART:

```text
D0 RX <- HC-06 TX
D1 TX -> HC-06 RX
```

Use it for:

- PID tuning.
- Sensor telemetry.
- Encoder count inspection.
- Battery monitoring.
- Debug logs.
- Map dump.
- Mode/status information.
- Future remote control.

Do not make the robot dependent on Bluetooth for normal autonomous operation.
The rule of standard Micromouse doesnt allow Bluetooth or other wireless 
communications.

The firmware must still boot and run if no Bluetooth device is connected.

## 45.1 Suggested text commands

Example:

```text
PING
STATUS
MOTOR
ENC
SENSOR
IMU
BAT
PID
MAP
MODE
STOP
```

Commands should be implemented in `Bluetooth.cpp`, not scattered through the
control modules.

---

# 46. CALIBRATION MODES

A diagnostic/calibration mode is strongly recommended.

Useful calibration actions:

## Encoder calibration

Measure:

```text
commanded distance
vs.
measured distance
```

to calibrate effective:

```text
counts per millimetre
```

## Wheel-base calibration

Rotate the robot by a known commanded angle and measure actual heading.

Adjust:

```text
WHEEL_BASE_MM
```

## Motor balance

Command the same velocity to both wheels and compare measured velocities.

Adjust PID and optional feed-forward independently.

## IMU calibration

Robot stationary:

- Estimate gyro bias.
- Store offsets.

## ToF calibration

Compare sensor readings against known wall distances.

Adjust:

- offset.
- filtering.
- valid range.
- wall thresholds.

---

# 47. CONFIGURATION STORAGE

The firmware should distinguish between:

## Compile-time constants

Use headers for:

- Pin mapping.
- Mechanical constants.
- PID parameters.
- Sensor roles.
- Default mode sequences.

## Runtime calibration data

Can be stored in EEPROM if desired:

- Encoder scale.
- Wheel diameter calibration.
- Wheel-base calibration.
- IMU bias.
- ToF offsets.

Use a versioned structure.

Example:

```cpp
struct PersistentConfig
{
    uint16_t version;
    float wheelScaleLeft;
    float wheelScaleRight;
    float wheelBase;
    int16_t gyroBias;
    uint16_t checksum;
};
```

Include a checksum/CRC.

If invalid:

```text
load defaults
```

rather than using corrupt values.

---

# 48. PERSISTENT MAZE FORMAT

A stored maze should contain enough information to verify and reconstruct the
map.

Example:

```cpp
struct MazeStorage
{
    uint16_t magic;
    uint8_t version;
    uint8_t width;
    uint8_t height;
    uint8_t cells[256];
    uint16_t checksum;
};
```

Exact packing can be optimized later.

The EEPROM write should happen only at controlled events such as:

```text
successful exploration complete
explicit save command
```

not continuously.

---

# 49. MODE MANAGER

`ModeManager` owns the top-level mode state.

Example:

```cpp
enum class RunMode
{
    EXPLORE,
    FAST_RUN,
    DIAGNOSTIC
};

enum class SystemState
{
    BOOT,
    INIT,
    SETTING,
    LOCKED,
    STANDBY,
    RUNNING,
    FINISHED,
    ERROR
};
```

Do not combine every concept into one enormous enum if that makes transitions
hard to reason about.

A clean design is:

```text
SystemState = what the robot is currently doing
RunMode     = what operating algorithm is selected
```

---

# 50. EXAMPLE STATE TRANSITIONS

```text
BOOT
  -> INIT
  -> LOCKED

LOCKED
  -> SETTING       if DIP bit 2 turns ON
  -> STANDBY       after 3 s SW1 hold

SETTING
  -> LOCKED        if DIP bit 2 turns OFF

STANDBY
  -> COVER_WAIT
  -> RUNNING       after valid cover + release

RUNNING
  -> FINISHED      after goal/run completion
  -> ERROR         on critical failure
  -> LOCKED/STOP   after user abort, according to safety policy

FINISHED
  -> LOCKED        after acknowledgement/power-cycle
```

The exact transition graph can evolve, but all transitions should be explicit.

---

# 51. RUN MODE DETAILS

## 51.1 Explore mode

Sequence:

```text
START
  |
  v
Initialize pose at start cell
  |
  v
Read walls
  |
  v
Update maze
  |
  v
Select next cell
  |
  v
Move
  |
  v
Update pose
  |
  v
Repeat
```

At goal:

```text
stop
save maze
show completion RGB state
```

## 51.2 Fast-run mode

Sequence:

```text
START
  |
  v
Load verified map
  |
  v
Generate optimized route
  |
  v
Execute route
  |
  v
Stop at goal
```

If no valid stored maze exists:

```text
do NOT blindly execute fast-run
```

Instead:

- enter error state, or
- require exploration first.

The selected behavior should be configurable.

## 51.3 Diagnostic mode

Provides controlled tests such as:

- Encoder test.
- Motor test.
- ToF readout.
- IMU readout.
- RGB test.
- Button/DIP test.
- Battery ADC readout.
- Bluetooth telemetry.

Diagnostic mode should default to motors disabled until a specific test is
requested.

---

# 52. DIAGNOSTIC OUTPUT

Suggested Bluetooth/status output:

```text
BOOT OK
TOF1 OK 0x30
TOF2 OK 0x31
TOF3 OK 0x32
IMU OK 0x68
ENC L OK
ENC R OK
BAT 7.92V
MODE EXPLORE
READY
```

During development:

```text
LSPD=...
RSPD=...
LENC=...
RENC=...
YAW=...
DL=...
F=...
DR=...
BAT=...
```

Telemetry frequency must be limited so serial I/O does not interfere with
control.

---

# 53. ERROR HANDLING

Use explicit error codes.

Example:

```cpp
enum class ErrorCode
{
    NONE,
    TOF_INIT_FAILED,
    IMU_INIT_FAILED,
    ENCODER_INVALID,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    MAP_INVALID,
    MOTOR_TIMEOUT,
    START_SEQUENCE_TIMEOUT
};
```

`Diagnostics` or `Safety` should store the current error.

RGB can indicate the major error class while Bluetooth provides the detailed
code.

---

# 54. MEMORY CONSIDERATIONS — ATMEGA328P

The ATmega328P has limited RAM and program memory.

Avoid:

- Dynamic allocation in control loops.
- `String` objects for high-frequency telemetry.
- Large temporary arrays.
- Excessive logging.
- Duplicate copies of the maze.
- Full floating-point object graphs.

Prefer:

- `constexpr`.
- Fixed-size arrays.
- Small structs.
- `uint8_t`, `uint16_t`, `int16_t` where appropriate.
- Static allocation for major buffers.
- Compact maze representation.

Be especially careful with:

```text
Maze
+
Flood-fill
+
Path buffer
+
Sensor buffers
+
Serial buffers
```

all existing simultaneously.

---

# 55. FLOAT VS FIXED-POINT

Floating-point is acceptable for slower control and geometry calculations on
the ATmega328P if CPU load is measured.

However:

- Avoid unnecessary floating-point work inside encoder ISRs.
- Avoid repeatedly recalculating constants.
- Consider fixed-point or integer math for high-frequency pieces if profiling
  shows CPU pressure.

Use the simplest implementation that meets the timing requirement.

---

# 56. FILE RESPONSIBILITY RULES

Every source file should have one obvious reason to change.

Examples:

### `MotorDriver.cpp`

Changes when:

- Driver interface changes.
- DRV8833 behavior changes.
- PWM pin implementation changes.

### `Encoder.cpp`

Changes when:

- Encoder decoding changes.
- Interrupt implementation changes.
- Quadrature filtering changes.

### `MotorControl.cpp`

Changes when:

- PID algorithm changes.
- Velocity controller changes.
- Straight-line correction changes.

### `Maze.cpp`

Changes when:

- Maze representation changes.

### `FloodFill.cpp`

Changes when:

- Flood-fill algorithm changes.

### `Explorer.cpp`

Changes when:

- Exploration policy changes.

### `FastRun.cpp`

Changes when:

- Fast-run planner/executor changes.

### `ModeManager.cpp`

Changes when:

- User operating-state behavior changes.

### `RobotConfig.h`

Changes when:

- Hardware/mechanical calibration changes.

This separation is one of the most important maintainability rules of the
project.

---

# 57. DEPENDENCY DIRECTION

Prefer:

```text
App
 |
 +--> ModeManager
       |
       +--> Explorer
       +--> FastRun
       +--> Diagnostic
                 |
                 v
              Control
                 |
                 v
              Drivers
```

Avoid circular dependencies.

For example:

Bad:

```text
MotorControl -> Explorer
Explorer -> MotorControl
```

Better:

```text
Explorer -> Motion interface
Motion interface -> MotorControl
```

Navigation asks for a motion action; it does not manipulate motor PWM directly.

---

# 58. PUBLIC INTERFACES

Keep module APIs small.

Example:

```cpp
namespace Motors
{
    void begin();
    void setTargetVelocity(float left, float right);
    void stop();
    bool isStopped();
}
```

Encoder:

```cpp
namespace Encoders
{
    void begin();
    int32_t leftCount();
    int32_t rightCount();
    float leftVelocity();
    float rightVelocity();
    void reset();
}
```

ToF:

```cpp
namespace ToF
{
    bool begin();
    bool update();
    uint16_t front();
    uint16_t diagonalLeft();
    uint16_t diagonalRight();
}
```

Do not expose internal implementation data unless necessary.

---

# 59. TESTING STRATEGY

Develop from the bottom upward.

## Test 1 — RGB

Verify:

- Solid colors.
- Blink.
- Status transitions.

## Test 2 — Button

Verify:

- Press.
- Release.
- 3-second hold.

## Test 3 — DIP setting switch

Verify:

- OFF.
- ON.
- Transition detection.

## Test 4 — Battery ADC

Compare displayed voltage to a multimeter.

## Test 5 — ToF

Read each sensor individually.

Verify:

- Correct XSHUT.
- Correct address.
- Correct logical role.

## Test 6 — IMU

Verify:

- Axis direction.
- Yaw sign.
- Stationary bias.
- Turn direction.

## Test 7 — Encoder

Turn each wheel manually.

Verify:

- Forward = positive.
- Backward = negative.
- A/B polarity.

## Test 8 — Motor driver

Lift the robot from the floor.

Verify:

- Left forward.
- Left backward.
- Right forward.
- Right backward.

Never begin with full-power ground testing.

## Test 9 — Closed-loop speed

Command equal wheel speeds.

Verify that actual measured speeds converge.

## Test 10 — Distance

Command one cell.

Calibrate wheel diameter / encoder scale.

## Test 11 — Turning

Command repeated 90° turns.

Calibrate effective wheel-base.

## Test 12 — Wall correction

Drive slowly alongside a wall.

Verify diagonal sensor correction.

## Test 13 — User state machine

Verify exact sequence:

```text
Power
-> Setting
-> Select
-> Lock
-> 3 s hold
-> Standby
-> Cover
-> Release
-> Run
```

## Test 14 — Maze exploration

Use a simple test maze before full-speed operation.

## Test 15 — Stored-map fast run

Only after the map format and exploration behavior are reliable.

---

# 60. RECOMMENDED DEVELOPMENT PHASES

## Phase 1 — Bring-up

- PlatformIO / Arduino build.
- GPIO.
- RGB.
- Button.
- DIP.
- Battery ADC.
- I2C.

## Phase 2 — Sensors

- ToF initialization.
- Unique addresses.
- IMU.
- Telemetry.

## Phase 3 — Motors

- Motor polarity.
- Encoder direction.
- Encoder counting.
- Open-loop low-speed motor tests.

## Phase 4 — Closed-loop control

- Wheel speed PID.
- Distance control.
- Straight-line correction.

## Phase 5 — Turning / pose

- Gyro.
- Odometry.
- Wheel-base calibration.
- Accurate 90° turns.

## Phase 6 — User interface

- Setting mode.
- Wheel gesture selector.
- Solid mode color.
- DIP lock.
- 3-second standby entry.
- ToF cover/release start.

## Phase 7 — Maze exploration

- Maze model.
- Wall detection.
- Flood-fill.
- Exploration policy.

## Phase 8 — Persistent map

- EEPROM format.
- CRC/checksum.
- Save/load.

## Phase 9 — Fast run

- Path generation.
- Segment compression.
- Motion profile.
- Higher speed.

## Phase 10 — Optimization

Only after reliable operation:

- Higher speeds.
- Better wall correction.
- More aggressive acceleration.
- More advanced route optimization.
- Better sensor fusion.

---

# 61. IMPORTANT CONSTANTS QUICK-REFERENCE

The following should be easy to find, preferably near the top of the config
files.

```text
BOARD
-----
Motor pins
Encoder pins
ToF XSHUT pins
RGB pin
Button pin
Setting DIP pin
Battery ADC pin
I2C pins

MOTOR
-----
Left motor reverse
Right motor reverse
Left encoder reverse
Right encoder reverse
PWM min
PWM max
Motor deadband

ENCODER
-------
Pulses/rev
Quadrature multiplier
Counts/rev

MECHANICAL
----------
Wheel diameter
Wheel base / effective track
Maze cell size

CONTROL
-------
Left PID Kp/Ki/Kd
Right PID Kp/Ki/Kd
Heading PID Kp/Ki/Kd
Wall PID Kp/Ki/Kd
Acceleration limits
Velocity limits
Turn tolerances

IMU
---
Yaw axis
Yaw sign
Gyro bias
Filter coefficient

TOF
---
Addresses
Logical roles
Measurement timing
Valid range
Wall thresholds
Start cover threshold

BATTERY
-------
R1
R2
ADC reference
Low threshold
Critical threshold

USER
----
Button hold time
Gesture threshold
Gesture timeout
Mode sequences
RGB colors
Blink period

MAZE
----
Width
Height
Start cell
Goal cell/region
Persistent-map version
```

---

# 62. DEFAULT MODE DEFINITIONS

The initial firmware should provide:

### `EXPLORE`

RGB: solid color A.

Behavior:

- Explore an unknown maze.
- Learn walls.
- Build map.
- Stop at goal.
- Save valid map.

### `FAST_RUN`

RGB: solid color B.

Behavior:

- Require a valid stored map.
- Generate efficient route.
- Run the known path at higher speed.
- Stop at goal.

### `DIAGNOSTIC`

RGB: solid color C.

Behavior:

- Provide hardware tests.
- No autonomous maze motion.
- Motors disabled except during explicit motor tests.

Additional modes can be added later without changing the low-level drivers.

---

# 63. SETTINGS / MODE-SELECTION EXAMPLE

Default example:

```text
DIP bit 2 = ON
        |
        v
SETTING ACTIVE
        |
        +-- wheel gesture: F, F, B
        |       -> EXPLORE
        |
        +-- wheel gesture: F, B, F
        |       -> FAST RUN
        |
        +-- wheel gesture: B, B, F
                -> DIAGNOSTIC
```

After selection:

```text
RGB = solid selected-mode color
```

Then:

```text
DIP bit 2 = OFF
```

locks the selection.

Then:

```text
hold SW1 for ~3 s
```

enters STANDBY.

RGB becomes:

```text
selected-mode color
+
blink
```

Then:

```text
cover ToF
   ->
release / uncover ToF
   ->
run selected mode
```

---

# 64. DESIGN RULES FOR FUTURE SCALING

The firmware architecture must make it possible to add:

- More ToF sensors.
- Better IMU filtering.
- Different motor drivers.
- Different motors.
- Another Bluetooth interface.
- USB serial commands.
- Additional operation modes.
- More sophisticated maze planners.
- Speed profiles.
- Calibration storage.
- Advanced telemetry.

without rewriting unrelated modules.

For example, adding a new fast-run planner should not require editing:

- Encoder driver.
- RGB driver.
- Button driver.
- Battery monitor.

Likewise, replacing the MPU6500 with another IMU should primarily affect the
IMU driver / adapter and configuration.

---

# 65. HARDWARE REFERENCE — MOTOR

GA12 N20 geared DC motor:

- 50:1 gearbox.
- Nominal operating point: 6 V.
- No-load speed: approximately 300 RPM at 6 V.
- Rated-load speed: approximately 250 RPM.
- No-load current: approximately 50 mA.
- Rated current: approximately 95 mA.
- Stall current: approximately 400 mA.
- Stall torque: approximately 1.30 kg·cm.
- Rated torque: approximately 0.35 kg·cm.
- Hall quadrature encoder.
- 7 pulses/channel/motor-shaft revolution.
- Approximately 350 pulses/channel/output-shaft revolution after gearbox.
- Encoder supply: 3.3–5 V.

Motor connector signals:

```text
M1
GND
C1
C2
VCC
M2
```

Firmware must treat motor/encoder orientation as configurable because the
physical connection may be manually reversed at the motor connector.

---

# 66. HARDWARE REFERENCE — DRV8833

- Dual H-bridge.
- One bridge per motor.
- Left bridge:
  - AIN1 = D6
  - AIN2 = D5
- Right bridge:
  - BIN1 = D9
  - BIN2 = D10

The driver is controlled by the motor driver module.

Keep motor-current management out of navigation code.

---

# 67. HARDWARE REFERENCE — IMU

Module:

```text
MPU6500
```

Typical address:

```text
0x68
```

Connected to:

```text
SDA = A4
SCL = A5
```

The module is physically plug-in.

Firmware must support configurable orientation because the module may be
installed with a different orientation after mechanical/PCB changes.

---

# 68. HARDWARE REFERENCE — TOF

Up to four module boards:

```text
ToF #1 = 0x30
ToF #2 = 0x31
ToF #3 = 0x32
ToF #4 = 0x33
```

Default:

```text
0x29
```

XSHUT:

```text
#1 = D12
#2 = D13
#3 = A1
#4 = A2
```

If only three modules are installed, the fourth sensor can remain disabled.

---

# 69. HARDWARE REFERENCE — POWER

SW2 DIP bit 1 is true system power:

```text
Battery + -> SW2 bit 1 -> system power distribution
```

Firmware does not control or monitor this switch directly.

Recommended 2S power concept:

```text
Battery
  |
  +-- SW2 bit 1 SYSTEM POWER
  |
  +-- motor power path -> suitable motor rail -> DRV8833
  |
  +-- logic regulator -> 5 V -> Nano / sensors / encoders
```

The motor nominal point is around 6 V and the listed motor operating range is
3–12 V.

---

# 70. HARDWARE REFERENCE — BATTERY ADC

```text
Battery + -> R1 -> A7 -> R2 -> GND
```

Example:

```text
R1 = 10 kΩ
R2 = 6.8 kΩ
```

Optional:

```text
100 nF
```

from the ADC node to ground.

---

# 71. HARDWARE REFERENCE — MECHANICS

The reference maze geometry is:

```text
Cell = 180 × 180 mm
Typical wall = 12 mm
Clear passage ≈ 168 mm
```

A practical initial robot target from the hardware design is approximately:

```text
PCB ≈ 125 × 110 mm
Robot overall ≈ 135 × 120 mm
```

The actual firmware must not assume these dimensions are exact.

For motion calculations, use measured/calibrated:

```text
wheel diameter
effective wheel-base
encoder scale
```

stored in configuration.

---

# 72. NON-NEGOTIABLE IMPLEMENTATION RULES

1. **Do not write the entire firmware in one `.ino` / `main.cpp` file.**
2. **Keep hardware pins in board configuration.**
3. **Keep mechanical dimensions in robot configuration.**
4. **Keep PID values in control configuration.**
5. **Keep sensor addresses/thresholds in sensor configuration.**
6. **Keep motor and encoder direction corrections configurable.**
7. **Keep IMU axis/sign corrections configurable.**
8. **Do not use blocking delays in autonomous control.**
9. **Keep encoder ISRs extremely short.**
10. **Do not perform maze logic inside hardware drivers.**
11. **Navigation must request motion through a control interface.**
12. **Safety must be able to stop the robot independently of navigation.**
13. **SW2 DIP bit 1 is power only; firmware must not wait for/read it.**
14. **SW2 DIP bit 2 controls SETTING.**
15. **SW1 is the push button.**
16. **Mode selection happens only while SETTING is active.**
17. **Turning DIP bit 2 OFF locks the selected mode.**
18. **A ~3 s SW1 hold enters STANDBY.**
19. **Run begins only after a valid ToF cover-then-release sequence.**
20. **Fast run must require a valid stored maze.**
21. **All hardware-dependent values must be easy to find and change.**
22. **The firmware must remain usable without Bluetooth.**
23. **Use appropriate comments. For code explaination, tuning / calibration instruction for the equivalent variables.**

---

# 73. FINAL EXPECTED USER EXPERIENCE

From the user's perspective, the intended behavior is:

```text
1. Turn robot power ON using SW2 DIP bit 1.

2. Robot boots and performs self-check.

3. Turn SW2 DIP bit 2 ON.

4. Robot enters SETTING.

5. Rotate ONE wheel using the predefined forward/backward gesture sequence.

6. Robot identifies the desired mode.

7. RGB becomes a solid color representing that mode.

8. Turn SW2 DIP bit 2 OFF.

9. Mode is now locked.
   RGB remains solid.

10. Press and hold SW1 for about 3 seconds.

11. Robot enters STANDBY.
    RGB blinks using the same selected-mode color.

12. Cover the designated ToF sensor.

13. Release/uncover the sensor.

14. Robot starts the selected mode.

15. EXPLORE:
      learn maze -> reach goal -> store map.

    or

    FAST RUN:
      load stored map -> calculate best known route -> run it quickly.

16. At completion, stop safely and show a clear completion state.
```

This behavior should be implemented as an explicit state machine rather than a
collection of nested `if` statements.

---

# 74. STARTING CHECKLIST FOR THE PROGRAMMER

Before autonomous motion:

```text
[ ] Arduino Nano detected
[ ] PlatformIO / Arduino build works
[ ] All pin definitions verified
[ ] RGB working
[ ] SW1 button working
[ ] SW2 bit 2 working
[ ] 3-second long press working
[ ] I2C = 100 kHz
[ ] ToF addresses assigned correctly
[ ] Logical ToF roles verified
[ ] MPU6500 working
[ ] IMU axes verified
[ ] Battery ADC calibrated
[ ] Left motor polarity verified
[ ] Right motor polarity verified
[ ] Left encoder polarity verified
[ ] Right encoder polarity verified
[ ] Encoder counts validated
[ ] Wheel diameter calibrated
[ ] Wheel-base calibrated
[ ] Left PID tuned
[ ] Right PID tuned
[ ] Heading control tuned
[ ] Straight control tuned
[ ] ToF wall thresholds tuned
[ ] Cover/release start trigger validated
[ ] Maze representation tested
[ ] Flood-fill tested
[ ] Explore mode tested on a simple maze
[ ] EEPROM map save/load tested
[ ] Fast-run path tested at low speed
[ ] Safety stop tested
[ ] Full autonomous test performed only after all above pass
```

---

# 75. RECOMMENDED FIRST IMPLEMENTATION ORDER

The most reliable order is:

```text
Board configuration
      ↓
Drivers
      ↓
Sensor bring-up
      ↓
Encoder validation
      ↓
Motor direction validation
      ↓
Wheel PID
      ↓
Odometry
      ↓
IMU heading
      ↓
ToF wall model
      ↓
Motion primitives
      ↓
User state machine
      ↓
Maze representation
      ↓
Flood-fill exploration
      ↓
Persistent map
      ↓
Fast run
      ↓
Speed optimization
```

Do not begin by writing the maze algorithm.

The robot needs trustworthy low-level motion and sensing first.

---

# 76. END OF SPECIFICATION

This file is intended to be the **single programming reference** for the
robot.

The programmer should be able to begin from this document and build the
firmware without needing the separate hardware-description document.

The exact calibration values may change after physical assembly and testing,
but all such values should be changed through the dedicated configuration
files rather than by modifying the underlying algorithms.
