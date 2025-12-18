#include <math.h>
#include "stepper_motor.h"
#include "config/pin_config.h"

StepperMotor::StepperMotor(uint8_t stepPin, uint8_t dirPin, int stepsPerRev, float gearratio)
: driver(stepPin, dirPin),
  timing(),
  stepPosition(0),
  stepsPerUnit(stepsPerRev * gearratio / (2.0f * M_PI)),
  stepHighStartMicros(0)
{}

void StepperMotor::move(float distanceUnits, float velocityUnits, float accelUnits)
{
    int32_t steps = (int32_t)(fabsf(distanceUnits) * stepsPerUnit);
    float velSteps = fabsf(velocityUnits) * stepsPerUnit;
    float accSteps = fabsf(accelUnits) * stepsPerUnit;

    driver.setDirection(distanceUnits >= 0.0f);
    timing.planMove(steps, velSteps, accSteps);
}

void StepperMotor::jog(float velocityUnits,float accelUnits)
{
    float velSteps = velocityUnits * stepsPerUnit;
    float accSteps = fabsf(accelUnits) * stepsPerUnit;

    driver.setDirection(velSteps >= 0.0f);
    timing.startJog(velSteps, accSteps);
}

void StepperMotor::stopJog()
{
    timing.stopJog();
}

void StepperMotor::update()
{
    uint32_t now = micros();

    // Finish pulse if active
    if (driver.stepActive()) {
        if (now - stepHighStartMicros >= STEP_PULSE_US) {
            driver.endStep();
        }
        return;
    }

    // Request new step
    if (timing.update()) {
        driver.beginStep();
        stepHighStartMicros = now;

        if (timing.currentVelocity() >= 0.0f) {
            stepPosition++;
        } else {
            stepPosition--;
        }
    }
}


float StepperMotor::positionUnits() const
{
    return (float)stepPosition / stepsPerUnit;
}


bool StepperMotor::isRunning() const
{
    return timing.isRunning();
}
