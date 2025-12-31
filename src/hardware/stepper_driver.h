#pragma once
#include <Arduino.h>

class StepperDriver {
public:
    StepperDriver(uint8_t stepPin, uint8_t dirPin, bool isReversed);

    void setDirection(bool dir);

    // called by motor when a step is requested
    void beginStep();

    // called later to finish the pulse
    void endStep();

    bool stepActive() const;

private:
    uint8_t stepPin;
    uint8_t dirPin;
    bool stepIsHigh;
    bool isReversed;
};
