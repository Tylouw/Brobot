#pragma once

#include <Arduino.h>

class StepperMotor;

/*
  MotionController
  ----------------
  Joint order (always):
    A1, A2, A3, A4, A5, A6

  Internal units:
    - angles: radians
    - velocities: rad/s
    - accelerations: rad/s^2

  IMPORTANT:
    - Motor positions are the ground truth (from StepperMotor step counter).
    - Joint angles are DERIVED from motor positions by inverting the coupling.
*/
class MotionController {
public:
    MotionController(StepperMotor* motors[6]);

    void init();
    void update();

    /* ===== Planned joint move ===== */

    // Absolute joint targets (degrees)
    void moveJointsDeg(const float targetDeg[6],
                       float maxVelDeg,
                       float accelDeg);

    // Absolute joint targets (radians)
    void moveJointsRad(const float targetRad[6],
                       float maxVelRad,
                       float accelRad);

    /* ===== Jogging ===== */

    // Continuous jog velocities (degrees/s)
    void setJogJointVelDeg(const float jointVelDeg[6],
                           float accelDeg);

    // Continuous jog velocities (rad/s)
    void setJogJointVelRad(const float jointVelRad[6],
                           float accelRad);

    void stopJog();

    bool isBusy() const;

    /* ===== State access (derived from motors) ===== */
    void getJointAnglesRad(float outRad[6]) const;
    void getJointAnglesDeg(float outDeg[6]) const;
    static void jointsToMotorsRad(const float q[6], float m[6]);
    static void motorsToJointsRad(const float m[6], float q[6]);

    void printMotorPositions() const;
    void printMotorSteps() const;

private:
    StepperMotor* motors_[6];

    // Ground truth motor positions (radians) from step counters
    float motorRad_[6];

    // Derived joint state (radians)
    float jointRad_[6];

    // Optional derived joint velocities (rad/s)
    float jointVelRad_[6];

    uint32_t lastUpdateMicros_;

    /* ===== Coupling maps =====
       Joint order:  A1 A2 A3 A4 A5 A6   (q)
       Motor order:  M1 M2 M3 M4 M5 M6   (m)
       All in radians.
    */

    // Refresh motorRad_ and jointRad_ from StepperMotor positions
    void refreshStateFromMotors_(float dt);

    static constexpr float degToRad(float d) { return d * DEG_TO_RAD; }
    static constexpr float radToDeg(float r) { return r * RAD_TO_DEG; }
};
