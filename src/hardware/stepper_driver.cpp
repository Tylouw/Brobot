#include "stepper_driver.h"

StepperDriver::StepperDriver(uint8_t step, uint8_t dir)
: stepPin(step), dirPin(dir), stepIsHigh(false)
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(stepPin, LOW);
}

void StepperDriver::setDirection(bool dir)
{
    digitalWrite(dirPin, dir ? HIGH : LOW);
}

void StepperDriver::beginStep()
{
    digitalWrite(stepPin, HIGH);
    stepIsHigh = true;
}

void StepperDriver::endStep()
{
    digitalWrite(stepPin, LOW);
    stepIsHigh = false;
}

bool StepperDriver::stepActive() const
{
    return stepIsHigh;
}