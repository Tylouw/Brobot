#pragma once
#include <Arduino.h>

enum class MotionMode {
    Idle,
    Planned,
    Jog
};

class StepperMotionTiming {
public:
    StepperMotionTiming();

    void planMove(
        int steps,
        float maxVelSteps,     // steps / second
        float accelSteps       // steps / second^2
    );

    // Call this frequently (e.g. every loop)
    // Returns true exactly when ONE step should be executed
    void startJog(float targetVelSteps, float accelSteps);
    void stopJog();   // smooth deceleration to zero

    bool update();
    bool isRunning() const;
    float currentVelocity() const;

private:
    MotionMode mode;

    // time
    uint32_t lastMicros;
    uint32_t interval;

    // planned move state
    int32_t stepCount;
    int32_t stepsTotal;
    int32_t rampUpEnd;
    int32_t rampDownStart;

    // velocity state
    float currentVel;
    float targetVel;
    float accel;
    float maxVel;

};
