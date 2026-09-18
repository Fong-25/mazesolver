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

## Open items (deferred until Encoder/MotorControl/PID/Bluetooth exist)
- Exact cruise speed, accel ramp, and turn parameters for each test — these
  should probably pull from `ControlConfig`/`RobotConfig` rather than being
  hardcoded per-test, so tuning one place updates both normal operation and
  the test.
- Whether `TEST ALL` needs a per-segment pass/fail readout (e.g. comparing
  commanded vs. IMU-measured turn angle) or just runs and lets you eyeball
  the `LOG` stream — leaning toward "log stream is enough for v1."