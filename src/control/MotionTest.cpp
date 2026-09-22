#include "MotionTest.h"

#include <string.h>

#include "../communication/Bluetooth.h"
#include "../config/ControlConfig.h"
#include "Motion.h"
#include "MotorControl.h"

namespace {
    enum class Phase : uint8_t { WAITING, RUNNING_STEP, DONE };
    enum class TestKind : uint8_t {
        NONE,
        ACCEL,
        CRUISE,
        DECEL,
        TURNL,
        TURNR,
        TURN180,
        ALL
    };

    constexpr uint8_t TEST_NAME_BUFFER_SIZE = 16;
    constexpr uint8_t ALL_SEQUENCE_LEN = 6;
    const TestKind ALL_SEQUENCE[ALL_SEQUENCE_LEN] = {
        TestKind::ACCEL, TestKind::CRUISE, TestKind::DECEL,
        TestKind::TURNL, TestKind::TURNR,  TestKind::TURN180};

    Phase phase = Phase::WAITING;
    bool runningAll = false;
    uint8_t allIndex = 0;

    TestKind parseTestName(const char* name) {
        if (strcasecmp(name, "ACCEL") == 0) return TestKind::ACCEL;
        if (strcasecmp(name, "CRUISE") == 0) return TestKind::CRUISE;
        if (strcasecmp(name, "DECEL") == 0) return TestKind::DECEL;
        if (strcasecmp(name, "TURNL") == 0) return TestKind::TURNL;
        if (strcasecmp(name, "TURNR") == 0) return TestKind::TURNR;
        if (strcasecmp(name, "TURN180") == 0) return TestKind::TURN180;
        if (strcasecmp(name, "ALL") == 0) return TestKind::ALL;
        return TestKind::NONE;
    }

    void startStep(TestKind kind) {
        switch (kind) {
            case TestKind::ACCEL:
            case TestKind::CRUISE:
            case TestKind::DECEL:
                // Same primitive for all three -- see header TODO. Longer
                // than one cell so CRUISE actually gets distance to hold
                // steady-state speed at, not just accel-then-stop.
                Motion::moveForwardCell(
                    ControlConfig::FORWARD_BASE_SPEED_MM_S,
                    ControlConfig::MOTION_TEST_FORWARD_DISTANCE_MM);
                break;
            case TestKind::TURNL:
                Motion::turnLeft90();
                break;
            case TestKind::TURNR:
                Motion::turnRight90();
                break;
            case TestKind::TURN180:
                Motion::turn180();
                break;
            default:
                break;
        }
    }

    void finishRun() {
        Motion::stop();
        MotorControl::disable();
        runningAll = false;
        allIndex = 0;
        phase = Phase::DONE;
    }
}

namespace MotionTest {
    void begin() {
        phase = Phase::WAITING;
        runningAll = false;
        allIndex = 0;
        // Armed but idle: nothing here calls a Motion primitive until a
        // real TEST command lands in WAITING, matching DIAGNOSTIC's
        // "motors only move on an explicit command" rule.
        MotorControl::enable();
    }

    void update(uint32_t nowMs) {
        (void)nowMs;
        switch (phase) {
            case Phase::DONE:
                return;

            case Phase::WAITING: {
                char name[TEST_NAME_BUFFER_SIZE];
                if (!Bluetooth::consumeTestRequest(name, sizeof(name))) return;

                TestKind kind = parseTestName(name);
                if (kind == TestKind::NONE)
                    return;  // unrecognized -- stay
                             // waiting for a
                             // valid one instead
                             // of silently
                             // finishing

                if (kind == TestKind::ALL) {
                    runningAll = true;
                    allIndex = 0;
                    startStep(ALL_SEQUENCE[allIndex]);
                } else {
                    startStep(kind);
                }
                phase = Phase::RUNNING_STEP;
                break;
            }

            case Phase::RUNNING_STEP:
                if (Motion::isBusy()) return;

                if (runningAll) {
                    allIndex++;
                    if (allIndex >= ALL_SEQUENCE_LEN) {
                        finishRun();
                    } else {
                        startStep(ALL_SEQUENCE[allIndex]);
                    }
                } else {
                    finishRun();
                }
                break;
        }
    }

    bool isDone() { return phase == Phase::DONE; }
}