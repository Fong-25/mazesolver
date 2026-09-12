MICROMOUSE ROBOT — COMPLETE HARDWARE DESCRIPTION
=====================================================

Project type
------------
Two-wheel differential-drive micromouse / autonomous maze-solving robot.

The PCB is the main controller/interconnect board. The Arduino Nano,
four ToF sensors, and MPU6500 are used as plug-in modules rather than
being represented as individual ICs in the main schematic.

PRIMARY HARDWARE
----------------
MCU:
- Arduino Nano, classic ATmega328P version.
- Plug-in module mounted on female headers.
- 5 V logic system.

Motors:
- 2 × Hshop GA12 N20 geared DC motors with integrated Hall quadrature
  encoders.
- 50:1 gearbox.
- Motor nominal operating point: 6 V.
- No-load speed: 300 RPM @ 6 V.
- Rated-load speed: 250 RPM.
- No-load current: 50 mA.
- Rated-load current: 95 mA.
- Stall current: 400 mA.
- Encoder: Hall, 2-channel AB quadrature.
- Encoder resolution: 7 pulses/channel/motor-shaft revolution,
  350 pulses/channel/output-shaft revolution after the 50:1 gearbox.
- Motor connector signals: M1, GND, C1, C2, VCC, M2.
- Encoder VCC: 3.3–5 V.
Source: Hshop product page:
https://hshop.vn/dong-co-dc-giamtoc-ga12-n20-encoder

Motor driver:
- DRV8833 dual H-bridge.
- One H-bridge per motor.
- Motor supply connected to the motor rail.
- Logic/control inputs driven by four Nano PWM pins.

Distance sensors:
- 3–4 × VL53L0X/VL53L1X module boards.
- Each sensor is plugged into a dedicated PCB header.
- All sensors share the same I2C SDA/SCL bus.
- No TCA9548A multiplexer.
- Individual XSHUT GPIO lines are used to initialize sensors one at a
  time and assign unique I2C addresses.
- Four-sensor addressing plan:
    ToF #1 = 0x30
    ToF #2 = 0x31
    ToF #3 = 0x32
    ToF #4 = 0x33
- If only three sensors are fitted, the fourth header/XSHUT can remain
  unused.

IMU:
- 1 × MPU6500 module.
- Plug-in module on a dedicated PCB header.
- Shares the main I2C bus.
- Typical I2C address: 0x68.

Bluetooth:
- HC-06 module.
- Connected to the Nano's hardware UART.
- Used for configuration, telemetry, debugging, or manual/control
  functions as desired.

LED:
- WS2812B RGB LED chain.
- All LEDs use one data GPIO.
- Number of LEDs is not fixed by the controller architecture.

User controls:
- 2-position DIP switch, two completely independent switches:
    SW1 = SYSTEM ENABLE / OFF
    SW2 = TOGGLE / MODE FUNCTION
- SW1 and SW2 are intentionally unrelated.
- 1 × momentary push button, unchanged from the previous design.

Battery monitoring:
- Battery voltage measured with a resistor divider into Nano A7.
- Optional RC filtering capacitor at the ADC input.

RECOMMENDED NANO PINOUT
-----------------------
D0 / RX   -> HC-06 TX
D1 / TX   -> HC-06 RX 

D2        -> Left encoder C1
D3        -> Left encoder C2

D4        -> Right encoder C1
D7        -> Right encoder C2

D6        -> DRV8833 AIN1  (PWM)
D5        -> DRV8833 AIN2  (PWM)

D9        -> DRV8833 BIN1  (PWM)
D10       -> DRV8833 BIN2  (PWM)

D8        -> Push button
D11       -> WS2812B data

D12       -> ToF #1 XSHUT
D13       -> ToF #2 XSHUT
A1        -> ToF #3 XSHUT
A2        -> ToF #4 XSHUT

A7        -> Battery voltage ADC
A3        -> SW2 toggle/mode switch

A4        -> I2C SDA
A5        -> I2C SCL

A6        -> Spare ADC
A0        -> Spare ADC

IMPORTANT NOTE ABOUT SW1 (SYSTEM ON/OFF)
-----------------------------------------
SW1 should NOT consume a Nano GPIO if it is intended to be a real
system power switch.

Recommended power architecture:
Battery -> SW1 -> protected/system power rail -> regulators + electronics

External regulator can be skipped since the Nano accept 7-12V to it VIN then drop to 5V by an on-board LDO.

This makes SW1 a true electrical system ON/OFF control and keeps it
independent from SW2.

SW2 is a normal GPIO input:
- SW2 -> A3
- Other side of SW2 -> GND
- Use INPUT_PULLUP
- Open = HIGH
- Closed = LOW

The push button is similarly:
- D8 -> button -> GND
- Use INPUT_PULLUP
- Released = HIGH
- Pressed = LOW

I2C CONNECTIONS
---------------
Nano A4 (SDA) -> all ToF module SDA pins + MPU6500 SDA
Nano A5 (SCL) -> all ToF module SCL pins + MPU6500 SCL

All I2C modules share common GND.

Each ToF additionally has:
ToF #1 XSHUT -> D12
ToF #2 XSHUT -> D13
ToF #3 XSHUT -> A1
ToF #4 XSHUT -> A2

ToF startup sequence:
1. Hold all XSHUT lines LOW.
2. Enable ToF #1.
3. Initialize it and change its address from 0x29 to 0x30.
4. Enable ToF #2 and change it to 0x31.
5. Enable ToF #3 and change it to 0x32.
6. Enable ToF #4 and change it to 0x33.

Do not enable all identical-address ToF sensors simultaneously before
assigning their addresses.

MOTOR CONNECTIONS
-----------------
Left motor:
- DRV8833 AOUT1/AOUT2 -> left motor M1/M2.
- Left encoder VCC -> 5 V.
- Left encoder GND -> GND.
- Left encoder C1 -> D2.
- Left encoder C2 -> D3.

Right motor:
- DRV8833 BOUT1/BOUT2 -> right motor M1/M2.
- Right encoder VCC -> 5 V.
- Right encoder GND -> GND.
- Right encoder C1 -> D4.
- Right encoder C2 -> D7.

Motor direction can be inverted in firmware if the physical motor
polarity/orientation requires it.

POWER ARCHITECTURE
------------------
Recommended structure for a 2S battery system:

Battery
  |
  +-- SW1 (SYSTEM ON/OFF)
  |
  +-- motor power path -> suitable motor rail -> DRV8833 VM
  |
  +-- logic regulator -> 5 V rail -> Nano/sensors/encoder modules

The motor's nominal point is 6 V and can work at the voltage range of 3-12V.
Use an appropriate motor regulator if needed.

The DRV8833 must have adequate local bulk/ceramic decoupling close to
its supply pins.

Keep the high-current motor path physically separate from sensitive
sensor/logic routing.

WALL SENSOR ORIENTATION
------------------------
A common Micromouse sensor configuration uses three (or four) forward 
facing sensors: one (or two) pointing straight ahead and two angled diagonally 
toward the left and the right wall.

      ↙[DL]    [F]    [DR]↘
              Front
                ↓
## Why use diagonal sensors?
Diagonal sensors provide look-ahead information about the side walls, 
which is especially useful when the robot is moving at high speed.

Unlike purely side-facing sensors, which mainly measure the robot's 
lateral distance from the walls, diagonal sensors can also help determine 
the robot's heading relative to the corridor.

By comparing the left and right diagonal measurements, the controller 
can detect yaw error and correct the robot's orientation. They can also 
detect approaching corners and wall openings earlier, giving the robot 
more time to adjust its motion.
## Sensor roles
- Front sensor: Detects walls directly ahead and determines whether 
the next cell is open.
- Left/right diagonal sensors: Provide look-ahead information for 
heading correction, wall tracking, and corner detection.

Side-facing sensors can still be useful and are sometimes used together
with diagonal sensors. However, diagonal sensors are particularly valuable 
for fast and accurate Micromouse motion.

GROUNDING / PCB LAYOUT
----------------------
Use a common electrical ground, but organize the physical layout so
motor current does not share narrow traces with sensor/logic return
currents.

Recommended placement:
- DRV8833 close to the two motor connectors.
- Battery/power entry close to the motor driver and regulators.
- Nano near the center of the logic section.
- MPU6500 close to the robot's mechanical center.
- ToF headers at the perimeter/front/sides according to sensing geometry.
- HC-06 away from noisy motor-current paths and preferably near a PCB
  edge for RF exposure.
- WS2812B near the desired visible location.
- Encoder connector traces routed away from motor outputs/PWM nodes.

For encoder signals, consider optional 47–220 ohm series resistor
footprints near the Nano if the final wiring proves noisy.

For I2C, keep SDA/SCL reasonably short and avoid routing them next to
motor outputs.

BATTERY MEASUREMENT
-------------------
Battery + -> R1 -> A7 -> R2 -> GND

Example for a 2S battery:
R1 = 10 kOhm
R2 = 6.8 kOhm

At 8.4 V:
A7 sees approximately 3.40 V.

Optionally add 100 nF from A7 to GND at the divider output.

SYSTEM ON/OFF
-------------
The first DIP switch position is a TRUE POWER switch, not a GPIO input.

SW1:
Battery + -> SW1 -> system power distribution

This means:
SW1 OFF = robot electronics physically unpowered
SW1 ON  = robot powered

SW2:
A3 -> SW2 -> GND
INPUT_PULLUP

SW2 is completely independent of SW1.

MECHANICAL / MICROMOUSE SIZE
----------------------------
Standard micromouse cells are 180 mm × 180 mm with 12 mm walls,
leaving 168 mm of clear passage width.

For a first practical micromouse, do NOT design the robot at the
absolute 168 mm maximum width.

Recommended PCB/chassis target:
- PCB width: approximately 120–130 mm
- PCB length: approximately 100–120 mm
- Overall robot envelope: approximately 125–140 mm wide and
  110–140 mm long, depending on motor/wheel/battery arrangement.

A very reasonable starting target is:
    PCB: 125 mm × 110 mm
    Robot overall: about 135 mm × 120 mm

This gives useful clearance inside the 168 mm maze passage and leaves
room for mechanical mounting, wheel clearance, sensor placement and
wiring.

If diagonal travel is a future goal, the robot needs to be much
narrower. A commonly cited target is below roughly 118 mm overall
width, because the diagonal clearance becomes the limiting geometry.

The absolute competition limits depend on the competition rules.
Many micromouse rules use 250 mm × 250 mm as the maximum robot size,
while some competitions impose a stricter 168 mm width/length limit.
Always design against the rules of the specific competition.

DESIGN INTENT
-------------
This PCB is intended to be a compact controller/interconnect board for
a differential-drive micromouse.

Core capabilities:
- Closed-loop left/right motor control using quadrature encoders.
- 3–4 directional ToF distance sensors.
- IMU-based orientation/gyro information.
- Autonomous maze navigation.
- Bluetooth telemetry/configuration.
- Battery monitoring.
- RGB status indication.
- Physical system power switch.
- Independent mode/function toggle.
- Momentary user button.
- The Arduino Nano, ToF modules and MPU6500 remain replaceable plug-in
modules. The PCB therefore contains their headers and supporting
power/signal circuitry rather than reproducing the modules' internal
schematics.

FIRMWARE NOTICES
----------------
## Encoder noise / spurious edges
- Motor/PWM noise may couple into encoder signals and cause false 
quadrature transitions.
- Handle this in firmware with a simple debounce/state filter or 
require two consistent reads before accepting an edge.
- Bench-test both motors under high load and verify encoder counts 
remain clean.
## I2C rise-time
- The estimated bus capacitance and 2.5 kΩ pull-up give a rise time 
of roughly 550 ns, which is too slow for 400 kHz Fast Mode.
- Firmware solution: use I2C at 100 kHz only (Wire.setClock(100000) 
if needed, else just Wire.begin() and running the 100kHz of the Nano default).
- At 100 kHz, the rise time is within the I2C specification and 
should avoid marginal sensor communication.

PCB DESIGN CHECKLIST
--------------------
[ ] Nano female headers <br>
[ ] 3–4 ToF module headers <br>
[ ] MPU6500 module header <br>
[ ] HC-06 header <br> 
[ ] Two motor/encoder connectors <br>
[ ] DRV8833 <br>
[ ] SW1 = system power switch <br>
[ ] SW2 = independent mode/function switch <br>
[ ] Push button <br>
[ ] Battery connector <br>
[ ] WS2812B connector/header or onboard LED <br>
[ ] Battery divider + RC filter <br>
[ ] Motor supply bulk capacitor <br>
[ ] 5 V regulator <br>
[ ] Appropriate motor regulator if using a battery above the desired 
    motor voltage <br>
[ ] I2C pull-up strategy checked against the actual ToF/MPU modules <br>
[ ] Optional encoder series resistor footprints <br>
[ ] Test points for GND, 5 V, battery, motor rail, SDA, SCL <br>
[ ] Clear silkscreen labels for every connector <br>
[ ] Reverse-polarity / overcurrent protection as appropriate <br>
[ ] Mounting holes and mechanical envelope verified before routing <br>

REFERENCE NOTES
---------------
Hshop motor:
https://hshop.vn/dong-co-dc-giamtoc-ga12-n20-encoder

Micromouse geometry reference:
- Standard maze cell: 180 × 180 mm.
- Typical wall thickness: 12 mm.
- Clear passage: 168 × 168 mm.

A compact robot is strongly preferable to using the full available
168 mm width because clearance improves wall sensing, turning and
error tolerance.

Sources checked:
Hshop GA12 N20 specifications and pinout.
Micromouse Online maze/chassis dimensions.
