#pragma once
#include <Arduino.h>

// UNDER HERE ARE MOSTLY PIN DEFINITIONS
namespace Board {
    // ENCODER
    constexpr uint8_t PIN_LEFT_ENC_A = 2;   // INT0 - true external interrupt
    constexpr uint8_t PIN_LEFT_ENC_B = 3;   // INT1 - true external interrupt
    constexpr uint8_t PIN_RIGHT_ENC_A = 4;  // PCINT2 group (shared w/ D7)
    constexpr uint8_t PIN_RIGHT_ENC_B = 7;  // PCINT2 group (shared w/ D4)

    // MOTOR DRIVER (DRV8833)
    constexpr uint8_t PIN_LEFT_IN1 = 6;    // AIN1
    constexpr uint8_t PIN_LEFT_IN2 = 5;    // AIN2
    constexpr uint8_t PIN_RIGHT_IN1 = 9;   // BIN1
    constexpr uint8_t PIN_RIGHT_IN2 = 10;  // BIN2

    // UI
    constexpr uint8_t PIN_BUTTON = 8;
    constexpr uint8_t PIN_RGB = 11;
    constexpr uint8_t RGB_LED_COUNT = 1;

    // ToF XSHUT
    constexpr uint8_t PIN_TOF_XSHUT_1 = 12;
    constexpr uint8_t PIN_TOF_XSHUT_2 = 13;
    constexpr uint8_t PIN_TOF_XSHUT_3 = A1;
    constexpr uint8_t PIN_TOF_XSHUT_4 = A2;

    // DIP SWITCH
    constexpr uint8_t PIN_SETTING_SWITCH = A3;  // 2-bit DIP
    // BATTERY
    constexpr uint8_t PIN_BATTERY_ADC = A7;

    // SHARED I2C BUS
    constexpr uint8_t PIN_I2C_SDA = A4;
    constexpr uint8_t PIN_I2C_SCL = A5;

    // UART (HC06, USB program)
    constexpr uint8_t PIN_UART_RX = 0;
    constexpr uint8_t PIN_UART_TX = 1;
}