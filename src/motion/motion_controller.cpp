#include "motion_controller.h"
#include "motion/stepper_motor.h"
#include <math.h>

static constexpr float WRIST_RATIO = 30.0f / 33.0f;

MotionController::MotionController(StepperMotor* motors[6])
: lastUpdateMicros_(0)
{
    for (int i = 0; i < 6; ++i) {
        motors_[i] = motors[i];
        motorRad_[i] = 0.0f;
        jointRad_[i] = 0.0f;
        jointVelRad_[i] = 0.0f;
    }
}

void MotionController::init() {
    stopJog();
    lastUpdateMicros_ = micros();

    // Initialize state from motors
    refreshStateFromMotors_(0.0f);
}

void MotionController::update() {
    // Update motor stepping first
    for (int i = 0; i < 6; ++i) {
        motors_[i]->update();
    }

    // Then refresh derived state
    const uint32_t now = micros();
    float dt = 0.0f;
    if (lastUpdateMicros_ != 0) {
        dt = (now - lastUpdateMicros_) * 1e-6f;
    }
    lastUpdateMicros_ = now;

    refreshStateFromMotors_(dt);
}

bool MotionController::isBusy() const {
    for (int i = 0; i < 6; ++i) {
        if (motors_[i]->isRunning()) return true;
    }
    return false;
}

/* ===========================
   State access (derived)
   =========================== */

void MotionController::getJointAnglesRad(float outRad[6]) const {
    for (int i = 0; i < 6; ++i) outRad[i] = jointRad_[i];
}

void MotionController::getJointAnglesDeg(float outDeg[6]) const {
    for (int i = 0; i < 6; ++i) outDeg[i] = radToDeg(jointRad_[i]);
}

/* =====================================================
   Joint ↔ Motor coupling (radians)
   Joint order: A1 A2 A3 A4 A5 A6
   Motor order: M1 M2 M3 M4 M5 M6
   ===================================================== */

void MotionController::jointsToMotorsRad(const float q[6], float m[6]) {
    // IMPORTANT: Keep your existing sign convention.
    // These negations are part of your project-wide direction convention.
    const float q1 = -q[0];
    const float q2 = -q[1];
    const float q3 = -q[2];
    const float q4 = -q[3];
    const float q5 = -q[4];
    const float q6 = -q[5];

    // Wrist coupling
    m[0] = (-1.0f * q4) + (1.0f * q5) + (-2.0f * q6);
    m[1] = ( 1.0f * q4) + (WRIST_RATIO * q5) + (2.0f * WRIST_RATIO * q6);
    m[2] = ( 1.0f * q4);

    // Direct joints
    m[3] = q3;
    m[4] = q2;
    m[5] = q1;
}

void MotionController::motorsToJointsRad(const float m[6], float q[6]) {
    // Invert the wrist coupling, then invert the sign convention back.
    //
    // Given:
    //   m2 = q4
    //   m0 = -q4 + q5 - 2 q6
    //   m1 =  q4 + r q5 + 2 r q6
    //
    // Let q4 = m2
    // A = m0 + q4 = q5 - 2 q6
    // B = (m1 - q4)/r = q5 + 2 q6
    // q5 = (A+B)/2
    // q6 = (B-A)/4

    const float q4 = m[2];
    const float A  = m[0] + q4;
    const float B  = (m[1] - q4) / WRIST_RATIO;

    const float q5 = 0.5f * (A + B);
    const float q6 = 0.25f * (B - A);

    const float q3 = m[3];
    const float q2 = m[4];
    const float q1 = m[5];

    // Now undo the earlier negation convention:
    q[0] = -q1; // A1
    q[1] = -q2; // A2
    q[2] = -q3; // A3
    q[3] = -q4; // A4
    q[4] = -q5; // A5
    q[5] = -q6; // A6
}

/* ===========================
   Internal: refresh from motors
   =========================== */

void MotionController::refreshStateFromMotors_(float dt) {
    float newMotorRad[6];
    for (int i = 0; i < 6; ++i) {
        // StepperMotor::positionUnits() is in "units" you planned for.
        // With your stepsPerUnit = stepsPerRev*gearratio/(2π),
        // units are radians of the motor-axis variable (i.e., m[i] in coupling equations).
        newMotorRad[i] = motors_[i]->positionUnits();
    }

    float newJointRad[6];
    motorsToJointsRad(newMotorRad, newJointRad);

    if (dt > 1e-6f) {
        for (int i = 0; i < 6; ++i) {
            jointVelRad_[i] = (newJointRad[i] - jointRad_[i]) / dt;
        }
    } else {
        for (int i = 0; i < 6; ++i) jointVelRad_[i] = 0.0f;
    }

    for (int i = 0; i < 6; ++i) motorRad_[i] = newMotorRad[i];
    for (int i = 0; i < 6; ++i) jointRad_[i] = newJointRad[i];
}

/* ===========================
   Planned motion
   =========================== */

void MotionController::moveJointsDeg(const float targetDeg[6],
                                     float maxVelDeg,
                                     float accelDeg)
{
    float targetRad[6];
    for (int i = 0; i < 6; ++i) targetRad[i] = degToRad(targetDeg[i]);

    moveJointsRad(targetRad, degToRad(maxVelDeg), degToRad(accelDeg));
}

void MotionController::moveJointsRad(const float targetRad[6],
                                     float maxVelRad,
                                     float accelRad)
{
    if (isBusy()) return;

    // Ensure we’re using the freshest measured state before computing deltas
    refreshStateFromMotors_(0.0f);

    // Convert absolute joint target to absolute motor target
    float targetMotorRad[6];
    jointsToMotorsRad(targetRad, targetMotorRad);

    // Compute motor deltas from measured motor positions
    float deltaMotorRad[6];
    for (int i = 0; i < 6; ++i) {
        deltaMotorRad[i] = targetMotorRad[i] - motorRad_[i];
    }

    // Sync: scale so longest motor move reaches maxVelRad
    float maxDist = 0.0f;
    for (int i = 0; i < 6; ++i) maxDist = fmaxf(maxDist, fabsf(deltaMotorRad[i]));
    if (maxDist < 1e-9f) return;

    for (int i = 0; i < 6; ++i) {
        const float dist = deltaMotorRad[i];
        const float scale = fabsf(dist) / maxDist;

        motors_[i]->move(dist,
                         maxVelRad * scale,
                         accelRad  * scale);
    }

    // NOTE: We do NOT set jointRad_ here.
    // jointRad_ will converge naturally as motors move (refreshStateFromMotors_ in update()).
}

/* ===========================
   Jogging (velocity mode)
   =========================== */

void MotionController::setJogJointVelDeg(const float jointVelDeg[6],
                                         float accelDeg)
{
    float jointVelRad[6];
    for (int i = 0; i < 6; ++i) jointVelRad[i] = degToRad(jointVelDeg[i]);
    setJogJointVelRad(jointVelRad, degToRad(accelDeg));
}

void MotionController::setJogJointVelRad(const float jointVelRad[6],
                                         float accelRad)
{
    // Convert joint velocities to motor velocities via same coupling
    float motorVelRad[6];
    jointsToMotorsRad(jointVelRad, motorVelRad);

    // For now: apply same accelRad to all motors.
    // If you later want exact coupled accel: pass a joint accel vector and map it too.
    const float a = fabsf(accelRad);

    for (int i = 0; i < 6; ++i) {
        motors_[i]->jog(motorVelRad[i], a);
    }
}

void MotionController::stopJog() {
    for (int i = 0; i < 6; ++i) motors_[i]->stopJog();
}
