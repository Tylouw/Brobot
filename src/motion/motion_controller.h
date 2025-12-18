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

  User-facing API:
    - degrees where explicitly stated
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

    void getJointAnglesRad(float outRad[6]) const;
    void getJointAnglesDeg(float outDeg[6]) const;

private:
    StepperMotor* motors_[6];

    // current joint state (radians)
    float jointRad_[6];

    /* ===== Joint → Motor coupling =====
       Input:  joint radians
       Output: motor radians
    */
    static void jointsToMotorsRad(const float q[6], float m[6]);

    static constexpr float degToRad(float d) { return d * DEG_TO_RAD; }
    static constexpr float radToDeg(float r) { return r * RAD_TO_DEG; }
};
