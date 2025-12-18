#include <math.h>
#include <Arduino.h>
#include "stepper_motion_timing.h"

StepperMotionTiming::StepperMotionTiming()
: mode(MotionMode::Idle),
  lastMicros(0),
  interval(0),
  stepCount(0),
  stepsTotal(0),
  rampUpEnd(0),
  rampDownStart(0),
  currentVel(0.0f),
  targetVel(0.0f),
  accel(0.0f),
  maxVel(0.0f)
{}

void StepperMotionTiming::planMove(int32_t steps,
                                   float maxVelSteps,
                                   float accelSteps)
{
    if (steps <= 0 || accelSteps <= 0.0f) {
        mode = MotionMode::Idle;
        return;
    }

    stepsTotal = steps;
    stepCount  = 0;

    accel  = accelSteps;
    maxVel = maxVelSteps;

    // steps needed to reach max velocity
    rampUpEnd = (int32_t)((maxVel * maxVel) / (2.0f * accel));
    if (rampUpEnd > stepsTotal / 2) {
        rampUpEnd = stepsTotal / 2;
    }

    rampDownStart = stepsTotal - rampUpEnd;

    lastMicros = micros();
    interval   = 0;

    mode = MotionMode::Planned;
}

void StepperMotionTiming::startJog(float targetVelSteps,
                                   float accelSteps)
{
    accel     = fabsf(accelSteps);
    targetVel = targetVelSteps;
    mode      = MotionMode::Jog;
}

void StepperMotionTiming::stopJog()
{
    targetVel = 0.0f;
}

bool StepperMotionTiming::update()
{
    if (mode == MotionMode::Idle) return false;

    uint32_t now = micros();

    // =======================
    // JOG MODE (time-based)
    // =======================
    if (mode == MotionMode::Jog) {
        float dt = (now - lastMicros) * 1e-6f;
        if (dt <= 0.0f) return false;

        float dv = accel * dt;
        if (currentVel < targetVel)
            currentVel = min(currentVel + dv, targetVel);
        else if (currentVel > targetVel)
            currentVel = max(currentVel - dv, targetVel);

        if (fabsf(currentVel) < 1e-4f && targetVel == 0.0f) {
            mode = MotionMode::Idle;
            return false;
        }

        interval = (uint32_t)(1e6f / fabsf(currentVel));

        if ((now - lastMicros) < interval) return false;
        lastMicros = now;
        return true;
    }

    // ==========================
    // PLANNED MOVE (step-based)
    // ==========================
    if (mode == MotionMode::Planned) {
        if ((now - lastMicros) < interval) return false;
        lastMicros = now;

        stepCount++;

        // acceleration
        if (stepCount <= rampUpEnd) {
            float v = sqrtf(2.0f * accel * stepCount);
            interval = (uint32_t)(1e6f / min(v, maxVel));
        }
        // deceleration
        else if (stepCount >= rampDownStart) {
            int32_t s = stepsTotal - stepCount;
            if (s <= 0) {
                mode = MotionMode::Idle;
                return true;
            }
            float v = sqrtf(2.0f * accel * s);
            interval = (uint32_t)(1e6f / v);
        }
        // cruise
        else {
            interval = (uint32_t)(1e6f / maxVel);
        }

        if (stepCount >= stepsTotal) {
            mode = MotionMode::Idle;
        }

        return true;
    }

    return false;
}

bool StepperMotionTiming::isRunning() const
{
    return mode != MotionMode::Idle;
}

float StepperMotionTiming::currentVelocity() const
{
    return currentVel;
}
