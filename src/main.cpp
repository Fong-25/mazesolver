#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "communication/Bluetooth.h"
#include "config/BoardConfig.h"
#include "config/ControlConfig.h"
#include "config/RobotConfig.h"
#include "control/Motion.h"
#include "control/MotorControl.h"
#include "control/PoseEstimator.h"
#include "drivers/BatteryMonitor.h"
#include "drivers/Button.h"
#include "drivers/DipSwitch.h"
#include "drivers/Encoder.h"
#include "drivers/IMU.h"
#include "drivers/MotorDriver.h"
#include "drivers/RGB.h"
#include "drivers/ToFManager.h"
#include "modes/ModeManager.h"
#include "modes/RgbStatus.h"
#include "navigation/Maze.h"
#include "navigation/MazePersistence.h"
#include "system/Diagnostics.h"
#include "system/Safety.h"
#include "system/Scheduler.h"

namespace {
    Scheduler scheduler;
}

void setup() {
    // Must be the very first thing that runs, before Serial/driver init --
    // see SETUP_NOTES.md. A watchdog left armed across reset can look like
    // a bricked board to the upload tool otherwise.
    MCUSR = 0;
    wdt_disable();

    // Mitigation for a leaky power switch (see TUNING_GUIDE.md "Dirty
    // boot"): sample the battery divider directly and require several
    // CONSECUTIVE readings above a sane threshold before doing anything
    // else. This catches a slow/noisy voltage ramp on switch-on; it does
    // NOT replace fixing BOD -- if VCC genuinely never drops below the
    // chip's own internal POR threshold between power cycles, no code
    // running after that point can be trusted to detect it, this loop
    // included. No watchdog is armed yet, so this can't trigger a reset
    // by sitting here.
    pinMode(Board::PIN_BATTERY_ADC, INPUT);
    uint8_t stableCount = 0;
    while (stableCount < 5) {
        int raw = analogRead(Board::PIN_BATTERY_ADC);
        float adcV = raw * Board::ADC_REFERENCE_V / Board::ADC_MAX_COUNTS;
        float battV =
            adcV *
            (Board::BATTERY_DIVIDER_R1_OHM + Board::BATTERY_DIVIDER_R2_OHM) /
            Board::BATTERY_DIVIDER_R2_OHM;
        if (battV >= RobotConfig::BATTERY_LOW_VOLTAGE) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        delay(10);
    }

    // --- Boot sequence, FIRMWARE_SPECS.md section 20 ---
    MotorDriver::begin();  // stops motor outputs (step 3) before anything
    // else can possibly command them
    RGB::begin();  // step 8
    RgbStatus::begin();
    Encoder::begin();  // step 4

    Wire.begin();  // step 5 -- shared bus, must happen exactly once, before
                   // either I2C device below
    bool tofOk = ToFManager::begin();  // step 6 (also sets the I2C clock)
    bool imuOk = IMU::begin();         // step 7
    if (imuOk) {
        IMU::calibrateBias();  // "once during boot while stationary" --
                               // robot isn't moving yet, this is that
                               // moment
    }

    Button::begin();          // step 9
    DipSwitch::begin();       // step 10
    BatteryMonitor::begin();  // step 11
    Bluetooth::begin();       // step 12 -- Serial is live from here on

    // Persisted for STATUS -- see Diagnostics.h. The prints below are
    // still useful over a wired USB serial monitor (no connect delay
    // there), but DON'T rely on catching them over Bluetooth: send
    // STATUS once connected instead, any time after boot.
    Diagnostics::setSelfCheck(tofOk, imuOk);
    if (tofOk && imuOk) {
        Serial.println(F("OK"));
    } else {
        if (!tofOk) Serial.println(F("TOF FAIL"));
        if (!imuOk) Serial.println(F("IMU FAIL"));
    }

    // Step 13, load persistent maze/config data: TODO, no persistence
    // layer exists yet.

    Diagnostics::begin();
    RgbStatus::playBootAnimation(1000);
    Safety::begin();  // arms the watchdog -- deliberately last of the
                      // slow/I2C inits above, so setup() itself doesn't
                      // eat into the 500ms margin before loop() starts
                      // kicking it

    MotorControl::begin();
    PoseEstimator::begin();
    MotorControl::disable();  // never drive just because we booted --
                              // ModeManager arms this explicitly once a
                              // run mode actually starts

    scheduler.begin(ControlConfig::CONTROL_PERIOD_US,
                    ControlConfig::IMU_PERIOD_US);

    // Step 14, self-check: minimal version until a real Diagnostics
    // self-check step exists -- a failed sensor init trips Safety
    // immediately, which ModeManager's first update() will turn into
    // ERROR on its own.
    if (!tofOk) Safety::triggerFault(Diagnostics::ErrorCode::TOF_INIT_FAILED);
    if (!imuOk) Safety::triggerFault(Diagnostics::ErrorCode::IMU_INIT_FAILED);

    if (!MazePersistence::load()) {
        Maze::reset();
    }
    ModeManager::begin();  // step 15 -- LOCKED (or ERROR, corrected on the
                           // first loop() tick if a fault was just
                           // triggered above)

    Serial.println(F("READY"));
}

void loop() {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    Encoder::service(nowMs);
    Safety::update(nowUs);

    if (scheduler.controlReady(nowUs)) {
        Motion::update(nowUs);  // arms -> sets wheel targets; no-op when idle
        MotorControl::update(nowUs);
        PoseEstimator::update(nowUs);
    }

    if (scheduler.imuReady(nowUs)) {
        IMU::update(nowMs);
    }

    ToFManager::update(nowMs);  // internally rate-limited + round-robin,
                                // no scheduler slot needed

    Button::update(nowMs);
    DipSwitch::update(nowMs);
    BatteryMonitor::update(nowMs);
    Bluetooth::update();

    ModeManager::update(nowMs);
    RgbStatus::update(nowMs);  // reads this tick's ModeManager state

    // Never kick unconditionally -- see SETUP_NOTES.md, that defeats the
    // entire point of having a watchdog.
    if (!Safety::isTripped()) {
        Safety::kickWatchdog();
    }
}