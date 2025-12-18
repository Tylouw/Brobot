#include <Arduino.h>
#include "robot.h"
#include "hardware/hardware_init.h"

#include "config/pin_config.h"
#include "utils/debug.h"
#include "motion/motion_controller.h"

HardwareInit hardware;
MotionController motion;

void Robot::init() {
    debug_init();
    LOG_INFO("Robot init starting");
    hardware.initPins();
    motion = hardware.initMotors_getMotionController();
    // disableMotors();
    // motor.move(6.0f, 0.7f, 1.0f);
    float target[6] = {0, 0, 0, 0, 45, 0};
    motion.moveJointsDeg(target,
                            60.0f,   // deg/s
                            180.0f); // deg/s^2
}

void Robot::update() {
    motion.update();
}