# Modes & Bluetooth Protocol

## Full mode table (wheel-gesture selection stays the primary path; BT is a bench-testing shortcut)

| # | Mode         | Wheel gesture | BT command | RGB (TBD once RgbStatus exists) |
|---|--------------|----------------|------------|----------------------------------|
| 0 | EXPLORE      | F, F, B        | `MODE 0`   | solid color A |
| 1 | FAST_RUN     | F, B, F        | `MODE 1`   | solid color B |
| 2 | DIAGNOSTIC   | B, B, F        | `MODE 2`   | solid color C |
| 3 | DEBUG_LOG    | F, F, F        | `MODE 3`   | solid color D (TBD) |
| 4 | MOTION_TEST  | B, B, B        | `MODE 4`   | solid color E (TBD) |

DEBUG_LOG and MOTION_TEST sit alongside EXPLORE/FAST_RUN/DIAGNOSTIC as full
sibling modes in ModeManager — not nested inside DIAGNOSTIC — so no extra
state-machine layer needed. Both inherit DIAGNOSTIC's existing rule from
spec section 62: **no autonomous maze motion, motors only move during an
explicit, requested action.**

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

## Open items (deferred until Encoder/MotorControl/PID/Bluetooth exist)
- Exact cruise speed, accel ramp, and turn parameters for each test — these
  should probably pull from `ControlConfig`/`RobotConfig` rather than being
  hardcoded per-test, so tuning one place updates both normal operation and
  the test.
- Whether `TEST ALL` needs a per-segment pass/fail readout (e.g. comparing
  commanded vs. IMU-measured turn angle) or just runs and lets you eyeball
  the `LOG` stream — leaning toward "log stream is enough for v1."