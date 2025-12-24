#include <Arduino.h>
#include "robot.h"
#include "hardware/hardware_init.h"

#include "config/pin_config.h"
#include "utils/debug.h"
#include "motion/motion_controller.h"

HardwareInit hardware;
MotionController* motion = nullptr;

void Robot::init() {
    debug_init();
    LOG_INFO("Robot init starting");
    hardware.initPins();
    motion = hardware.initMotors_getMotionController();
}

bool readAnglesFromSerial(float angles[6]) {
    if (Serial.available() <= 0) return false;

    String text = Serial.readStringUntil('\n');
    text.trim();
    if (text.length() == 0) return false;

    int startIndex = 0;
    for (int i = 0; i < 6; ++i) {
        int endIndex = text.indexOf(',', startIndex);
        if (endIndex < 0) endIndex = text.length();

        String part = text.substring(startIndex, endIndex);
        part.trim();
        angles[i] = part.toFloat();

        startIndex = endIndex + 1;
        if (endIndex == text.length() && i < 5) {
            // Not enough comma-separated values
            return false;
        }
    }
    return true;
}

void Robot::update() {
    if (!motion) return;

    float angles[6];
    if (readAnglesFromSerial(angles)) {
        Serial.print("angles: ");
        for (int i = 0; i < 6; ++i) {
            Serial.print(angles[i]);
            Serial.print(i < 5 ? "," : "\n");
        }
        motion->moveJointsDeg(angles, 60.0f, 180.0f);
    }

    motion->update();
}


