#pragma once
#include "hardware/stepper_driver.h"
#include "stepper_motion_timing.h"

class StepperMotor {
public:
    StepperMotor(uint8_t stepPin,
                 uint8_t dirPin,
                 int stepsPerRev,
                 float gearratio);

    // planned move
    void move(float distanceUnits,
              float velocityUnits,
              float accelUnits);

    // jogging
    void jog(float velocityUnits,
             float accelUnits);
    void stopJog();

    void update();
    bool isRunning() const;
    float positionUnits() const;

private:
    StepperDriver driver;
    StepperMotionTiming timing;
    int64_t stepPosition;
    int stepsPerUnit;
    uint32_t stepHighStartMicros;
    static constexpr uint32_t STEP_PULSE_US = 3;
};
