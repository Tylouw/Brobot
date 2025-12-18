#include "motion_controller.h"
#include "motion/stepper_motor.h"
#include <math.h>

static constexpr float WRIST_RATIO = 30.0f / 33.0f;

MotionController::MotionController(StepperMotor* motors[6]) {
    for (int i = 0; i < 6; ++i) {
        motors_[i]  = motors[i];
        jointRad_[i] = 0.0f;
    }
}

void MotionController::init() {
    stopJog();
}

void MotionController::update() {
    for (int i = 0; i < 6; ++i) {
        motors_[i]->update();
    }
}

bool MotionController::isBusy() const {
    for (int i = 0; i < 6; ++i) {
        if (motors_[i]->isRunning()) return true;
    }
    return false;
}

/* ===========================
   Joint state access
   =========================== */

void MotionController::getJointAnglesRad(float outRad[6]) const {
    for (int i = 0; i < 6; ++i) {
        outRad[i] = jointRad_[i];
    }
}

void MotionController::getJointAnglesDeg(float outDeg[6]) const {
    for (int i = 0; i < 6; ++i) {
        outDeg[i] = radToDeg(jointRad_[i]);
    }
}

/* =====================================================
   Joint → Motor coupling
   Joint order:  A1 A2 A3 A4 A5 A6
   Motor order:  M1 M2 M3 M4 M5 M6
   ===================================================== */
void MotionController::jointsToMotorsRad(const float q[6], float m[6]) {
    const float q1 = q[0];
    const float q2 = q[1];
    const float q3 = q[2];
    const float q4 = q[3];
    const float q5 = q[4];
    const float q6 = q[5];

    // Wrist coupling
    m[0] = (-1.0f * q4) + (1.0f * q5) + (-2.0f * q6);
    m[1] = ( 1.0f * q4) + (WRIST_RATIO * q5) + (2.0f * WRIST_RATIO * q6);
    m[2] = ( 1.0f * q4);

    // Direct joints
    m[3] = q3;
    m[4] = q2;
    m[5] = q1;
}

/* ===========================
   Planned motion
   =========================== */

void MotionController::moveJointsDeg(const float targetDeg[6],
                                     float maxVelDeg,
                                     float accelDeg)
{
    float targetRad[6];
    for (int i = 0; i < 6; ++i) {
        targetRad[i] = degToRad(targetDeg[i]);
    }
    moveJointsRad(targetRad,
                  degToRad(maxVelDeg),
                  degToRad(accelDeg));
}

void MotionController::moveJointsRad(const float targetRad[6],
                                     float maxVelRad,
                                     float accelRad)
{
    if (isBusy()) return;

    float dqRad[6];
    for (int i = 0; i < 6; ++i) {
        dqRad[i] = targetRad[i] - jointRad_[i];
        jointRad_[i] = targetRad[i];
    }

    float dmRad[6];
    jointsToMotorsRad(dqRad, dmRad);

    // Find longest motor move for synchronization
    float maxDist = 0.0f;
    for (int i = 0; i < 6; ++i) {
        maxDist = fmaxf(maxDist, fabsf(dmRad[i]));
    }
    if (maxDist < 1e-9f) return;

    for (int i = 0; i < 6; ++i) {
        const float scale = fabsf(dmRad[i]) / maxDist;
        motors_[i]->move(dmRad[i],
                         maxVelRad * scale,
                         accelRad  * scale);
    }
}

/* ===========================
   Jogging
   =========================== */

void MotionController::setJogJointVelDeg(const float jointVelDeg[6],
                                         float accelDeg)
{
    float jointVelRad[6];
    for (int i = 0; i < 6; ++i) {
        jointVelRad[i] = degToRad(jointVelDeg[i]);
    }
    setJogJointVelRad(jointVelRad, degToRad(accelDeg));
}

void MotionController::setJogJointVelRad(const float jointVelRad[6],
                                         float accelRad)
{
    float motorVelRad[6];
    jointsToMotorsRad(jointVelRad, motorVelRad);

    for (int i = 0; i < 6; ++i) {
        motors_[i]->jog(motorVelRad[i], accelRad);
    }
}

void MotionController::stopJog() {
    for (int i = 0; i < 6; ++i) {
        motors_[i]->stopJog();
    }
}
