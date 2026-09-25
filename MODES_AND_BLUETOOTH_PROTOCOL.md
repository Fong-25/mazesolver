# Modes & Bluetooth Protocol

## Full mode table (wheel-gesture selection stays the primary path; BT is a bench-testing shortcut)

| # | Mode         | Wheel gesture | BT command | RGB (solid, `UserConfig::RGB_*`) |
|---|--------------|----------------|------------|----------------------------------|
| 0 | EXPLORE      | F, F, B        | `MODE 0`   | green |
| 1 | FAST_RUN     | F, B, F        | `MODE 1`   | blue |
| 2 | DIAGNOSTIC   | B, B, F        | `MODE 2`   | yellow |
| 3 | DEBUG_LOG    | F, F, F        | `MODE 3`   | cyan |
| 4 | MOTION_TEST  | B, B, B        | `MODE 4`   | magenta |

DEBUG_LOG and MOTION_TEST sit alongside EXPLORE/FAST_RUN/DIAGNOSTIC as full
sibling modes in ModeManager — not nested inside DIAGNOSTIC — so no extra
state-machine layer needed. Both inherit DIAGNOSTIC's existing rule from
spec section 62: **no autonomous maze motion, motors only move during an
explicit, requested action.**

## RGB status behavior (`RgbStatus`)

`RgbStatus` is the only module allowed to touch `RGB` directly -- it reads
`ModeManager`/`SettingMode`/`StandbyMode` and translates that into color +
blink. Colors are `UserConfig::RGB_*` constants (first-pass placeholders,
tune once the LED's on the bench).

| SystemState | RGB |
|---|---|
| BOOT / INIT | off (TODO: startup animation, spec §17) |
| SETTING | off until a gesture sequence matches this session, then solid selected-mode color |
| LOCKED | solid locked-mode color (off if nothing's ever been locked yet) |
| STANDBY, waiting for cover | selected-mode color, blinking |
| STANDBY, cover confirmed | selected-mode color, **solid** |
| RUNNING | solid mode color (same color as LOCKED/SETTING -- no visual seam at the transition) |
| FINISHED | solid white (`RGB_FINISHED_*`) |
| ERROR | solid red (`RGB_ERROR_*`) |

### Standby: solid-on-cover feedback

Not in the original spec -- added because blinking the whole time gives no
signal for whether a cover gesture actually registered until after you've
already released and the robot either did or didn't start moving.

Now: RGB blinks through `WAIT_FOR_COVER`, then switches to **solid** the
instant `StandbyMode` sees a valid cover (`isCoverConfirmed()` becomes
true) -- while the sensor is still covered, before release. Solid means
"that cover counted, go ahead and release." It stays solid through the
release and into RUNNING (same color, so there's no seam), and only starts
blinking again the next time `STANDBY` is freshly entered.

## Modes 0 & 1 — start-cell alignment (EXPLORE and FAST_RUN)

Both runs start with the robot parked **against the start cell's rear wall**,
so its wheel-axle centerline (the point every "move one cell" is measured
from) is `RobotConfig::ROBOT_CENTER_TO_REAR_MM` from that wall — not
`CELL_SIZE_MM / 2`. Every move after the first is a fixed `CELL_SIZE_MM`
delta from wherever the robot actually is, so without a correction that
offset would carry into every "cell center" for the whole run.

`RobotConfig::FIRST_MOVE_DISTANCE_MM` (= `CELL_SIZE_MM/2 - ROBOT_CENTER_TO_REAR_MM`)
is the extra forward travel that centers the robot in the start cell. It is
an **addition**, never a replacement for a cell move, and it does not
advance the cell counter — the robot is still in the start cell, just
centered now.

| Mode | How the alignment is applied |
|---|---|
| EXPLORE | Standalone alignment move (at `FORWARD_BASE_SPEED_MM_S`) **before the first sense** — the front ToF threshold is calibrated for a centered robot, so sensing from the parked position would read the front wall too far away. Then the normal DECIDE loop starts. |
| FAST_RUN | Added onto the first straight run: `FIRST_MOVE_DISTANCE_MM + n × CELL_SIZE_MM`, so there is no extra stop. If the very first step needs a **turn** instead, a standalone alignment move happens first (turning in place from the un-centered position would put every later cell center off), then it re-decides. |

Assumes the standard start: start cell open only toward the initial heading
(NORTH), robot facing it. `ROBOT_CENTER_TO_REAR_MM` is still the `0.0f`
placeholder until measured — see `SETUP_NOTES.md` section A. A
`static_assert` in `RobotConfig.h` rejects a value above `CELL_SIZE_MM / 2`.

## Mode 1 — FAST_RUN

Purpose: run the already-known map (from a completed EXPLORE this session, or
`LOAD`ed from EEPROM at boot) start → goal as one timed, one-way attempt. No
wall sensing, no `Maze`/`FloodFill` updates during the run, no return leg.
ModeManager entering FAST_RUN is the implicit contract that a valid map
exists.

Behavior (`navigation/FastRun.cpp`), all running at `FAST_RUN_SPEED_MM_S`:

- **Path rule:** at each cell, step to the open neighbor with the lowest
  `FloodFill` distance. Ties between equal-distance neighbors prefer
  **straight over turning** — same length, fewer turns.
- **Straight-run batching (spec §33):** consecutive same-direction steps are
  issued as **one** `Motion::moveForwardCell(speed, n × CELL_SIZE_MM)`
  primitive — no stop between cells. It looks ahead with the same rule the
  real decision uses, so the batch is exactly what step-by-step would have
  done. A batch ends at any turn, or at the first goal cell.
- **Turns** still stop, rotate in place (IMU-tracked), then continue. Corner
  smoothing / diagonals are **not** implemented — deliberately deferred
  until this baseline is confirmed working on hardware.
- **Ends** on entering any goal-region cell (motors disabled). No known path
  from the current cell → `SOFTWARE_FAULT`.
- Start alignment as in the section above.

Known limits (not bugs): `Motion` has no velocity-profile shaping yet, so a
batch still starts and stops with a step change in target speed rather than a
ramp. Longer batches expose that more at high speed — the next thing to watch
when raising `FAST_RUN_SPEED_MM_S`.

## Mode 2 — DIAGNOSTIC (now with concrete commands)

Purpose: isolated hardware bring-up/testing. Motors only move on an
explicit command, never autonomously.

| Command       | Action |
|---------------|--------|
| `MOTOR L <pwm>` | Set left motor to raw PWM, -255 to 255 (sign = direction) |
| `MOTOR R <pwm>` | Set right motor to raw PWM, -255 to 255 |
| `MOTOR BRAKE`   | Active brake both motors |
| `MOTOR STOP`    | Coast both motors (also reachable via the global `STOP`) |
| `LOG <channel>` | Same streaming channels as DEBUG_LOG (E/M/I/B/T/A/OFF) — available here too, since motor-polarity bring-up needs to drive AND watch encoders simultaneously |
| `LOG P`   | Pose (x, y, theta) from PoseEstimator |

**This is the procedure `SETUP_NOTES.md` section B refers to**: enter
DIAGNOSTIC, run `LOG E`, then `MOTOR L 100` — watch the terminal to confirm
direction and encoder sign together in real time.

## Mode 3 — DEBUG_LOG

Purpose: stream requested telemetry channels over Bluetooth. Tuning/config
only, never used during EXPLORE/FAST_RUN.

BT commands while in this mode:

| Command   | Streams |
|-----------|---------|
| `LOG E`   | Encoder counts (left, right) |
| `LOG M`   | Commanded motor PWM (left, right) |
| `LOG I`   | IMU yaw rate + integrated yaw |
| `LOG B`   | Battery voltage |
| `LOG T`   | ToF distances (all installed roles) |
| `LOG A`   | All of the above, one line per tick |
| `LOG OFF` | Stop streaming |

Multiple channels can likely be OR'd together later (e.g. `LOG E I` for
encoder+IMU side by side) — not required for v1, easy to extend since each
channel is just a bit in a mask.

## Mode 4 — MOTION_TEST

Purpose: validate PID (wheel-speed, heading, distance) using only encoders
+ IMU. **No ToF involved by design** — this tests the control loop itself,
not obstacle awareness.

Individually selectable primitives, plus a "run all in sequence" option:

| Command        | Action |
|----------------|--------|
| `TEST ACCEL`   | Straight line, ramp from 0 to cruise speed, hold briefly, stop |
| `TEST CRUISE`  | Straight line at constant cruise speed only (isolates steady-state PID from accel/decel transients) |
| `TEST DECEL`   | Cruise speed to full stop, isolates deceleration behavior |
| `TEST TURNL`   | In-place 90° left turn |
| `TEST TURNR`   | In-place 90° right turn |
| `TEST TURN180` | In-place 180° turn ("turning backward") |
| `TEST ALL`     | Runs the full sequence above, in order, with a pause between each |

Safety, non-negotiable for this mode:
- Every test has a **hard max-duration and max-distance cap** (config values,
  TBD once MotorControl/PID exist) — if a test doesn't self-terminate
  cleanly within that cap, motors force-stop and an error is reported, not
  left running.
- `STOP` (already in the spec's suggested BT command list) works at all
  times in this mode and kills motors immediately, regardless of which
  test is mid-run.
- This mode should probably also auto-arm a physical safety expectation:
  robot needs clearance space, not maze-adjacent, given zero ToF awareness.

## `MAP` command — maze snapshot over Bluetooth

Prints the current runtime `Maze` state as ASCII art. One renderer for
every stage of a run — it doesn't know or care whether EXPLORE has run
yet, it just reflects what's actually in `Maze`:

- **Before any exploration**: nothing is `visited`, so every interior wall
  renders as `UNKNOWN` (`...` / `?`) and only the four outer border walls
  (stamped by `Maze::reset()`) show up solid.
- **After a successful explore + save**: walls adjacent to a `visited`
  cell resolve out of `UNKNOWN` into real `WALL`/`OPEN`. Cells FloodFill
  never reached stay `UNKNOWN` — since flood-fill exploration doesn't
  have to visit the whole grid to reach the goal, this naturally shows
  only the explored subset, not the full 16x16.

Legend:

| Symbol | Meaning |
|---|---|
| `---` / `\|` | known wall |
| (blank) | known open (no wall) |
| `...` / `?` | not yet explored (neither adjacent cell visited) |
| `S` | start cell |
| `G` | goal region |
| `.` | visited cell (not start/goal) |
| ` ` (cell interior) | unvisited cell |

Example, fresh boot (4x4 shown for brevity — real grid is
`MazeConfig::WIDTH` x `MazeConfig::HEIGHT`):

MAP
+---+---+---+---+ <br>
| . . . |         <br>
+...+...+...+...+ <br>
| . . . |         <br>
+...+...+...+...+ <br>
| . . . |         <br>
+...+...+...+...+ <br>
| S . . |         <br>
+---+---+---+---+ <br>
END MAP


Send `MAP` any time over Bluetooth (works in any `ModeManager` state,
same as `STATUS`/`PING`) to get a fresh snapshot.

## `SAVE` / `LOAD` commands — persist/restore the map over Bluetooth
 
Bench-testing shortcut for `MazePersistence`'s two triggers (spec section
48): normally the EEPROM write happens automatically the moment EXPLORE
reaches the goal, but these let you force it (or restore it) directly.
 
| Command | Action |
|---|---|
| `SAVE` | Writes the current runtime `Maze` to EEPROM, replies `SAVE OK` |
| `LOAD` | Loads EEPROM into `Maze` if magic/version/width/height/checksum all match, replies `LOAD OK` or `LOAD FAIL: ...` and leaves `Maze` untouched |
 
Same as `MAP`: works in any `ModeManager` state, no gating — these are
debug/ops tools, not part of the run state machine. `LOAD` failing is
expected and harmless the very first time, before anything has ever been
saved.
 
## ToF sensor offset calibration (`SensorConfig::TOF_OFFSET_MM_*`)
 
The four VL53L0X units don't agree with each other raw — confirmed on the
bench (two agreeing, one under-reading, one over-reading, by roughly
10-20mm at the same distance). `ToFManager` corrects for this once, per
physical sensor, before any reading leaves the driver — `Explorer`,
`FloodFill`, everything above it always sees an already-corrected value
and needs no knowledge that the four units differ.
 
**Procedure:**
1. Place a target at a known reference distance (e.g. 100mm).
2. For each sensor in turn, point it straight down its *own* optical axis
   at the target — for the two diagonal sensors this means aligning the
   target to that sensor's angle, not the robot's body heading.
3. Read the raw distance (e.g. via `LOG T` in DEBUG_LOG/DIAGNOSTIC).
4. `offset[i] = reference - reading[i]`. Negative offsets are expected
   and correct for any sensor that over-reports distance.
5. Set `SensorConfig::TOF_OFFSET_MM_1..4`, indexed by physical sensor
   position (`XSHUT_PINS`/`TARGET_ADDR` order), NOT by logical role — the
   bias belongs to that specific unit, not to where it's mounted.
Applied in `ToFManager::pollOneSensor()`, added to the raw reading before
the validity check and everything downstream (`FRONT_WALL_MM`,
`SIDE_WALL_MAX_MM`) ever sees it.

## Wall-detection threshold calibration (`SensorConfig::FRONT_WALL_MM`, `SIDE_WALL_MAX_MM`)

Do the ToF offset calibration above first — `LOG T` (DEBUG_LOG/DIAGNOSTIC)
prints values **after** the per-sensor offset and, for the diagonals, the
slant→perpendicular correction, and these thresholds are compared against
exactly those values. Measure the real thing directly rather than computing
it from geometry; the point is what the sensors read in a real maze cell.

Explorer senses with the robot at the **center of a cell** (this is why the
start-cell alignment exists), so calibrate from that pose: robot's wheel-axle
centerline over the cell center, aligned with the cell walls.

**`FRONT_WALL_MM`**
1. Place a real wall across the front of that cell. The wall face is
   ~`CELL_SIZE_MM/2` (less half the wall thickness) ahead of the axle
   centerline; each front sensor sits some distance ahead of the axle, so
   its reading is that minus its forward offset — read it, don't compute it.
2. `LOG T`: note `FRONT_LEFT` and `FRONT_RIGHT` with the wall present. Front
   detection is **either sensor** (OR), so take the **larger** of the two.
3. Remove the wall and note the readings with the front open (they will be at
   least about one `CELL_SIZE_MM` larger — the next wall, or out of range).
4. Set `FRONT_WALL_MM` above the wall-present reading by a margin that covers
   stopping-position error (roughly ±10 mm — check by nudging the robot
   forward/back), and well below the wall-absent reading. A false "wall"
   only costs a wasted re-evaluation; a missed real wall costs a collision,
   so bias toward the wall-present side.

**`SIDE_WALL_MAX_MM`**
1. Center the robot in a real corridor (both side walls present).
2. `LOG T`: read `DIAGONAL_LEFT` and `DIAGONAL_RIGHT` — already corrected to
   perpendicular distance, so no angle math needed.
3. Open one side (remove that wall) and note the reading; repeat for the
   other side. Expect it far above the wall-present value.
4. Set `SIDE_WALL_MAX_MM` above the larger wall-present reading with margin
   for lateral drift (shove the robot ~10 mm toward each side and confirm the
   wall is still detected there), and below the wall-absent readings.
5. Re-check after any change to `TOF_DIAGONAL_MOUNT_ANGLE_DEG` or the
   offsets — the reading these compare against moves with them.

## Open items (deferred until Encoder/MotorControl/PID/Bluetooth exist)
- Exact cruise speed, accel ramp, and turn parameters for each test — these
  should probably pull from `ControlConfig`/`RobotConfig` rather than being
  hardcoded per-test, so tuning one place updates both normal operation and
  the test.
- Whether `TEST ALL` needs a per-segment pass/fail readout (e.g. comparing
  commanded vs. IMU-measured turn angle) or just runs and lets you eyeball
  the `LOG` stream — leaning toward "log stream is enough for v1."