#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "communication/Bluetooth.h"
#include "config/ControlConfig.h"
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

    // --- Boot sequence, FIRMWARE_SPECS.md section 20 ---
    MotorDriver::begin();  // stops motor outputs (step 3) before anything
                           // else can possibly command them
    Encoder::begin();      // step 4

    Wire.begin();  // step 5 -- shared bus, must happen exactly once, before
                   // either I2C device below
    bool tofOk = ToFManager::begin();  // step 6 (also sets the I2C clock)
    bool imuOk = IMU::begin();         // step 7
    if (imuOk) {
        IMU::calibrateBias();  // "once during boot while stationary" --
                               // robot isn't moving yet, this is that
                               // moment
    }

    RGB::begin();  // step 8
    RgbStatus::begin();
    Button::begin();          // step 9
    DipSwitch::begin();       // step 10
    BatteryMonitor::begin();  // step 11
    Bluetooth::begin();       // step 12 -- Serial is live from here on

    Serial.println(F("BOOT OK"));
    Serial.println(tofOk ? F("TOF OK") : F("TOF FAIL"));
    Serial.println(imuOk ? F("IMU OK") : F("IMU FAIL"));

    // Step 13, load persistent maze/config data: TODO, no persistence
    // layer exists yet.

    Diagnostics::begin();
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

    Maze::reset();
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