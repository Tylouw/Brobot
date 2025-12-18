#pragma once
#include <Arduino.h>

// ============================
// Global robot configuration
// ============================

constexpr uint8_t MOTOR_COUNT = 6;

// ============================
// Emergency stop
// ============================

constexpr gpio_num_t ESTOP_PIN = GPIO_NUM_19;

// ============================
// Shift register / serial bits
// ============================

constexpr gpio_num_t SR_DATA_PIN  = GPIO_NUM_17;
constexpr gpio_num_t SR_CLOCK_PIN = GPIO_NUM_18;
constexpr gpio_num_t SR_LATCH_PIN = GPIO_NUM_5;

// ============================
// Stepper motor pins
// Order MUST match logical axis order
// ============================

struct StepperPinConfig {
    gpio_num_t step;
    gpio_num_t dir;
};

constexpr StepperPinConfig STEPPER_PINS[MOTOR_COUNT] = {
    { GPIO_NUM_13, GPIO_NUM_12 }, // M1 – A4/A5/A6 coupling
    { GPIO_NUM_14, GPIO_NUM_27 }, // M2 – A4/A5/A6 coupling
    { GPIO_NUM_26, GPIO_NUM_25 }, // M3 – A4
    { GPIO_NUM_33, GPIO_NUM_32 }, // M4 – A3
    { GPIO_NUM_16, GPIO_NUM_4  }, // M5 – A2
    { GPIO_NUM_15, GPIO_NUM_2  }  // M6 – A1
};

// ============================
// Stepper motor parameters
// ============================

constexpr int STEPS_PER_REV = 800;

constexpr float GEAR_RATIOS[MOTOR_COUNT] = {
    6.0f,
    6.0f,
    6.0f,
    (56.0f / 16.0f) * (120.0f / 23.0f),
    42.0f,
    192.0f / 16.0f
};
