#include "Bluetooth.h"

#include <ctype.h>
#include <string.h>

#include "../config/MazeConfig.h"
#include "../config/UserConfig.h"
#include "../control/MotorControl.h"
#include "../control/PoseEstimator.h"
#include "../drivers/BatteryMonitor.h"
#include "../drivers/Encoder.h"
#include "../drivers/IMU.h"
#include "../drivers/ToFManager.h"
#include "../navigation/Maze.h"
#include "../navigation/MazePersistence.h"
#include "../system/Diagnostics.h"
#include "../system/Safety.h"

namespace {
    constexpr uint8_t LINE_BUFFER_SIZE = 64;
    char lineBuffer[LINE_BUFFER_SIZE];
    uint8_t lineLength = 0;

    constexpr uint8_t LOG_ENCODER = 1 << 0;
    constexpr uint8_t LOG_MOTOR = 1 << 1;
    constexpr uint8_t LOG_IMU = 1 << 2;
    constexpr uint8_t LOG_BATTERY = 1 << 3;
    constexpr uint8_t LOG_TOF = 1 << 4;
    constexpr uint8_t LOG_POSE = 1 << 5;

    uint8_t logMask = 0;
    uint32_t lastLogMs = 0;

    bool hasPendingMode = false;
    uint8_t pendingModeIndex = 0;

    bool hasPendingTest = false;
    char pendingTestName[8] = {0};

    bool hasPendingMotor = false;
    Bluetooth::MotorAction pendingMotorAction = Bluetooth::MotorAction::STOP;
    int16_t pendingMotorPwm = 0;

    // WALL is a known wall, OPEN is a known gap, UNKNOWN means neither side
    // of that edge has been visited yet -- only ever UNKNOWN for interior
    // edges, never the border (see below).
    enum class WallState : uint8_t { OPEN, WALL, UNKNOWN };

    WallState wallStateAt(uint8_t x, uint8_t y, Maze::Direction dir) {
        bool present = Maze::hasWall(x, y, dir);
        uint8_t nx, ny;
        if (!Maze::neighborOf(x, y, dir, nx, ny)) {
            // Grid boundary -- Maze::reset() stamps these unconditionally, so
            // they're always known regardless of what's been explored.
            return present ? WallState::WALL : WallState::OPEN;
        }
        if (!Maze::isVisited(x, y) && !Maze::isVisited(nx, ny)) {
            return WallState::UNKNOWN;  // neither side explored yet
        }
        return present ? WallState::WALL : WallState::OPEN;
    }

    char mazeCellChar(uint8_t x, uint8_t y) {
        if (x == MazeConfig::START_X && y == MazeConfig::START_Y) return 'S';
        if (x >= MazeConfig::GOAL_X_MIN && x <= MazeConfig::GOAL_X_MAX &&
            y >= MazeConfig::GOAL_Y_MIN && y <= MazeConfig::GOAL_Y_MAX) {
            return 'G';
        }
        return Maze::isVisited(x, y) ? '.' : ' ';
    }

    void printMazeHorizontalEdge(uint8_t y, Maze::Direction dir) {
        for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
            Serial.print('+');
            switch (wallStateAt(x, y, dir)) {
                case WallState::WALL:
                    Serial.print(F("---"));
                    break;
                case WallState::OPEN:
                    Serial.print(F("   "));
                    break;
                case WallState::UNKNOWN:
                    Serial.print(F("..."));
                    break;
            }
        }
        Serial.println('+');
    }

    void printMazeCellRow(uint8_t y) {
        for (uint8_t x = 0; x < MazeConfig::WIDTH; x++) {
            switch (wallStateAt(x, y, Maze::Direction::WEST)) {
                case WallState::WALL:
                    Serial.print('|');
                    break;
                case WallState::OPEN:
                    Serial.print(' ');
                    break;
                case WallState::UNKNOWN:
                    Serial.print('?');
                    break;
            }
            Serial.print(' ');
            Serial.print(mazeCellChar(x, y));
            Serial.print(' ');
        }
        switch (wallStateAt(MazeConfig::WIDTH - 1, y, Maze::Direction::EAST)) {
            case WallState::WALL:
                Serial.println('|');
                break;
            case WallState::OPEN:
                Serial.println(' ');
                break;
            case WallState::UNKNOWN:
                Serial.println('?');
                break;
        }
    }

    // On-demand ASCII snapshot of the current runtime map -- one renderer for
    // every stage, see rationale above.
    void printMaze() {
        Serial.println(F("MAP"));
        for (int8_t y = MazeConfig::HEIGHT - 1; y >= 0; y--) {
            printMazeHorizontalEdge((uint8_t)y, Maze::Direction::NORTH);
            printMazeCellRow((uint8_t)y);
        }
        printMazeHorizontalEdge(0, Maze::Direction::SOUTH);
        Serial.println(F("END MAP"));
    }

    void printToF() {
        Serial.print(F("SENSOR FL="));
        Serial.print(
            ToFManager::getDistanceMm(ToFManager::SensorRole::FRONT_LEFT));
        Serial.print(F(" FR="));
        Serial.print(
            ToFManager::getDistanceMm(ToFManager::SensorRole::FRONT_RIGHT));
        Serial.print(F(" DL="));
        Serial.print(
            ToFManager::getDistanceMm(ToFManager::SensorRole::DIAGONAL_LEFT));
        Serial.print(F(" DR="));
        Serial.println(
            ToFManager::getDistanceMm(ToFManager::SensorRole::DIAGONAL_RIGHT));
    }

    void printPidGains() {
        float kp, ki, kd;
        MotorControl::getLeftGains(kp, ki, kd);
        Serial.print(F("PID L kp="));
        Serial.print(kp);
        Serial.print(F(" ki="));
        Serial.print(ki);
        Serial.print(F(" kd="));
        Serial.println(kd);

        MotorControl::getRightGains(kp, ki, kd);
        Serial.print(F("PID R kp="));
        Serial.print(kp);
        Serial.print(F(" ki="));
        Serial.print(ki);
        Serial.print(F(" kd="));
        Serial.println(kd);
    }

    void printStatus() {
        Serial.print(F("STATUS bat="));
        Serial.print(BatteryMonitor::getVoltage());
        Serial.print(F(" tripped="));
        Serial.print(Safety::isTripped() ? 1 : 0);
        Serial.print(F(" err="));
        Serial.print((uint8_t)Diagnostics::getError());
        Serial.print(F(" spdL="));
        Serial.print(MotorControl::getMeasuredLeftSpeedMmS());
        Serial.print(F(" spdR="));
        Serial.print(MotorControl::getMeasuredRightSpeedMmS());
        Serial.print(F(" x="));
        Serial.print(PoseEstimator::getXMm());
        Serial.print(F(" y="));
        Serial.print(PoseEstimator::getYMm());
        Serial.print(F(" th="));
        Serial.println(PoseEstimator::getThetaDeg());
    }

    void handleMotorCommand(char* rest) {
        char* savePtr = nullptr;
        char* sub = strtok_r(rest, " ", &savePtr);
        if (sub == nullptr) {
            Serial.println(F("ERR MOTOR needs L/R/BRAKE/STOP"));
            return;
        }

        // Queued only -- never applied here. Diagnostic is the sole
        // consumer, and only while ModeManager is actually in
        // DIAGNOSTIC's own RUNNING session (see Bluetooth.h).
        if (strcasecmp(sub, "BRAKE") == 0) {
            pendingMotorAction = Bluetooth::MotorAction::BRAKE;
            hasPendingMotor = true;
            Serial.println(
                F("MOTOR BRAKE queued (applied only in DIAGNOSTIC)"));
        } else if (strcasecmp(sub, "STOP") == 0) {
            pendingMotorAction = Bluetooth::MotorAction::STOP;
            hasPendingMotor = true;
            Serial.println(F("MOTOR STOP queued (applied only in DIAGNOSTIC)"));
        } else if (strcasecmp(sub, "L") == 0 || strcasecmp(sub, "R") == 0) {
            char* valStr = strtok_r(nullptr, " ", &savePtr);
            if (valStr == nullptr) {
                Serial.println(F("ERR MOTOR needs a PWM value"));
                return;
            }
            int16_t pwm = (int16_t)atoi(
                valStr);  // MotorDriver clamps out-of-range internally
            pendingMotorAction = (strcasecmp(sub, "L") == 0)
                                     ? Bluetooth::MotorAction::SET_LEFT
                                     : Bluetooth::MotorAction::SET_RIGHT;
            pendingMotorPwm = pwm;
            hasPendingMotor = true;
            Serial.print(F("MOTOR "));
            Serial.print(sub);
            Serial.print(F(" = "));
            Serial.print(pwm);
            Serial.println(F(" queued (applied only in DIAGNOSTIC)"));
        } else {
            Serial.println(F("ERR unknown MOTOR subcommand"));
        }
    }

    void handlePidCommand(char* rest) {
        char* savePtr = nullptr;
        char* side = strtok_r(rest, " ", &savePtr);

        if (side == nullptr) {
            printPidGains();
            return;
        }

        char* kpStr = strtok_r(nullptr, " ", &savePtr);
        char* kiStr = strtok_r(nullptr, " ", &savePtr);
        char* kdStr = strtok_r(nullptr, " ", &savePtr);

        if (kpStr == nullptr || kiStr == nullptr || kdStr == nullptr) {
            Serial.println(F("ERR PID needs: PID <L|R> <kp> <ki> <kd>"));
            return;
        }

        float kp = atof(kpStr), ki = atof(kiStr), kd = atof(kdStr);

        if (strcasecmp(side, "L") == 0) {
            MotorControl::setLeftGains(kp, ki, kd);
            Serial.println(F("PID L updated"));
        } else if (strcasecmp(side, "R") == 0) {
            MotorControl::setRightGains(kp, ki, kd);
            Serial.println(F("PID R updated"));
        } else
            Serial.println(F("ERR PID side must be L or R"));
    }

    void handleLogCommand(char* rest) {
        char* savePtr = nullptr;
        char* channel = strtok_r(rest, " ", &savePtr);
        if (channel == nullptr) {
            Serial.println(F("ERR LOG needs a channel (E/M/I/B/T/P/A/OFF)"));
            return;
        }

        if (strcasecmp(channel, "OFF") == 0) {
            logMask = 0;
            Serial.println(F("LOG OFF"));
            return;
        }
        if (strcasecmp(channel, "A") == 0) {
            logMask = 0x3F;
            Serial.println(F("LOG ALL"));
            return;
        }

        uint8_t bit = 0;
        switch (toupper(channel[0])) {
            case 'E':
                bit = LOG_ENCODER;
                break;
            case 'M':
                bit = LOG_MOTOR;
                break;
            case 'I':
                bit = LOG_IMU;
                break;
            case 'B':
                bit = LOG_BATTERY;
                break;
            case 'T':
                bit = LOG_TOF;
                break;
            case 'P':
                bit = LOG_POSE;
                break;
            default:
                Serial.println(F("ERR unknown LOG channel"));
                return;
        }

        logMask ^=
            bit;  // toggle: sending the same channel again turns it back off
        Serial.print(F("LOG mask = 0x"));
        Serial.println(logMask, HEX);
    }

    void handleModeCommand(char* rest) {
        char* savePtr = nullptr;
        char* numStr = strtok_r(rest, " ", &savePtr);
        if (numStr == nullptr) {
            Serial.println(F("ERR MODE needs a number"));
            return;
        }

        pendingModeIndex = (uint8_t)atoi(numStr);
        hasPendingMode = true;
        Serial.print(F("MODE request queued: "));
        Serial.println(pendingModeIndex);
    }

    void handleTestCommand(char* rest) {
        if (rest == nullptr || rest[0] == '\0') {
            Serial.println(
                F("ERR TEST needs a name "
                  "(ACCEL/CRUISE/DECEL/TURNL/TURNR/TURN180/ALL)"));
            return;
        }
        strncpy(pendingTestName, rest, sizeof(pendingTestName) - 1);
        pendingTestName[sizeof(pendingTestName) - 1] = '\0';
        hasPendingTest = true;
        Serial.print(F("TEST request queued: "));
        Serial.println(pendingTestName);
    }

    void handleLine(char* line) {
        char* savePtr = nullptr;
        char* cmd = strtok_r(line, " ", &savePtr);
        if (cmd == nullptr) return;

        if (strcasecmp(cmd, "PING") == 0)
            Serial.println(F("PONG"));
        else if (strcasecmp(cmd, "STATUS") == 0)
            printStatus();
        else if (strcasecmp(cmd, "STOP") == 0) {
            Safety::triggerUserAbort();
            Serial.println(F("STOPPED"));
        } else if (strcasecmp(cmd, "MOTOR") == 0)
            handleMotorCommand(savePtr);
        else if (strcasecmp(cmd, "ENC") == 0) {
            Serial.print(F("ENC L="));
            Serial.print(Encoder::LEFT_ENCODER_COUNT());
            Serial.print(F(" R="));
            Serial.println(Encoder::RIGHT_ENCODER_COUNT());
        } else if (strcasecmp(cmd, "SENSOR") == 0)
            printToF();
        else if (strcasecmp(cmd, "IMU") == 0) {
            Serial.print(F("IMU rate="));
            Serial.print(IMU::getYawRateDegPerSec());
            Serial.print(F(" yaw="));
            Serial.println(IMU::getYawDeg());
        } else if (strcasecmp(cmd, "BAT") == 0) {
            Serial.print(F("BAT "));
            Serial.println(BatteryMonitor::getVoltage());
        } else if (strcasecmp(cmd, "PID") == 0)
            handlePidCommand(savePtr);
        else if (strcasecmp(cmd, "LOG") == 0)
            handleLogCommand(savePtr);
        else if (strcasecmp(cmd, "MODE") == 0)
            handleModeCommand(savePtr);
        else if (strcasecmp(cmd, "TEST") == 0)
            handleTestCommand(savePtr);
        else if (strcasecmp(cmd, "MAP") == 0)
            printMaze();
        else if (strcasecmp(cmd, "SAVE") == 0) {
            MazePersistence::save();
            Serial.println(F("SAVE OK"));
        } else if (strcasecmp(cmd, "LOAD") == 0) {
            if (MazePersistence::load()) {
                Serial.println(F("LOAD OK"));
            } else {
                Serial.println(
                    F("LOAD FAIL: no valid saved maze (magic/version/"
                      "checksum mismatch)"));
            }
        } else
            Serial.println(F("ERR unknown command"));
    }

    void serviceLogStream(uint32_t nowMs) {
        if (logMask == 0) return;
        if ((nowMs - lastLogMs) < UserConfig::LOG_STREAM_INTERVAL_MS) return;
        lastLogMs = nowMs;

        if (logMask & LOG_ENCODER) {
            Serial.print(F("E L="));
            Serial.print(Encoder::LEFT_ENCODER_COUNT());
            Serial.print(F(" R="));
            Serial.println(Encoder::RIGHT_ENCODER_COUNT());
        }
        if (logMask & LOG_MOTOR) {
            Serial.print(F("M L="));
            Serial.print(MotorControl::getLastLeftPwm());
            Serial.print(F(" R="));
            Serial.println(MotorControl::getLastRightPwm());
        }
        if (logMask & LOG_IMU) {
            Serial.print(F("I rate="));
            Serial.print(IMU::getYawRateDegPerSec());
            Serial.print(F(" yaw="));
            Serial.println(IMU::getYawDeg());
        }
        if (logMask & LOG_BATTERY) {
            Serial.print(F("B "));
            Serial.println(BatteryMonitor::getVoltage());
        }
        if (logMask & LOG_TOF) printToF();
        if (logMask & LOG_POSE) {
            Serial.print(F("P x="));
            Serial.print(PoseEstimator::getXMm());
            Serial.print(F(" y="));
            Serial.print(PoseEstimator::getYMm());
            Serial.print(F(" th="));
            Serial.println(PoseEstimator::getThetaDeg());
        }
    }
}

namespace Bluetooth {
    void begin() {
        Serial.begin(UserConfig::BLUETOOTH_BAUD);
        lineLength = 0;
        logMask = 0;
        lastLogMs = millis();
        hasPendingMode = false;
        hasPendingTest = false;
    }

    void update() {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (lineLength > 0) {
                    lineBuffer[lineLength] = '\0';
                    handleLine(lineBuffer);
                    lineLength = 0;
                }
            } else if (lineLength < (LINE_BUFFER_SIZE - 1)) {
                lineBuffer[lineLength++] = c;
            }
            // else: silently drop overflow chars — a too-long line just
            // truncates, never overruns the buffer.
        }

        serviceLogStream(millis());
    }

    bool consumeModeRequest(uint8_t& outModeIndex) {
        if (!hasPendingMode) return false;
        outModeIndex = pendingModeIndex;
        hasPendingMode = false;
        return true;
    }

    bool consumeTestRequest(char* outBuffer, uint8_t bufferSize) {
        if (!hasPendingTest) return false;
        strncpy(outBuffer, pendingTestName, bufferSize - 1);
        outBuffer[bufferSize - 1] = '\0';
        hasPendingTest = false;
        return true;
    }

    bool consumeMotorRequest(MotorAction& outAction, int16_t& outPwm) {
        if (!hasPendingMotor) return false;
        outAction = pendingMotorAction;
        outPwm = pendingMotorPwm;
        hasPendingMotor = false;
        return true;
    }
}