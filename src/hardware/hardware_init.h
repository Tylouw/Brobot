#include <Arduino.h>
#include "config/pin_config.h"
#include "motion/stepper_motor.h"
#include "motion/motion_controller.h"

class HardwareInit {
  public:
  HardwareInit()
  : motors_{
      StepperMotor(STEPPER_PINS[0].step, STEPPER_PINS[0].dir, true, STEPS_PER_REV, GEAR_RATIOS[0]), // A4/A5/A6 coupling
      StepperMotor(STEPPER_PINS[1].step, STEPPER_PINS[1].dir, true, STEPS_PER_REV, GEAR_RATIOS[1]), // A4/A5/A6 coupling
      StepperMotor(STEPPER_PINS[2].step, STEPPER_PINS[2].dir, true, STEPS_PER_REV, GEAR_RATIOS[2]), // A4 normal
      StepperMotor(STEPPER_PINS[3].step, STEPPER_PINS[3].dir, true, STEPS_PER_REV, GEAR_RATIOS[3]), // A3 normal
      StepperMotor(STEPPER_PINS[4].step, STEPPER_PINS[4].dir, true, STEPS_PER_REV, GEAR_RATIOS[4]), // A2 normal
      StepperMotor(STEPPER_PINS[5].step, STEPPER_PINS[5].dir, true, STEPS_PER_REV, GEAR_RATIOS[5]) // A1 reversed
    }
  {
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      motorPtrs_[i] = &motors_[i];
    }
  }
  void initPins() {
      pinMode(ESTOP_PIN, INPUT_PULLUP);
      pinMode(SR_DATA_PIN, OUTPUT);
      pinMode(SR_CLOCK_PIN, OUTPUT);
      pinMode(SR_LATCH_PIN, OUTPUT);
  }

  void sendSerialBits(int value){
    for (int i = 11; i < 32; i++) {
      digitalWrite(SR_DATA_PIN, (value & (1 << i)) > 0 );
      // Serial.print((value & (1 << i)) > 0);
      digitalWrite(SR_CLOCK_PIN, HIGH);
      // delayMicroseconds(30);
      digitalWrite(SR_CLOCK_PIN, LOW);
    }
    // Serial.println();
    digitalWrite(SR_LATCH_PIN, HIGH);
    digitalWrite(SR_LATCH_PIN, LOW);
  }

  MotionController* initMotors_getMotionController(){
    //M2: MS1,MS2,MS3,EN: 0100
    //M1: MS1,MS2,MS3,EN: 0100
    //M4: MS1,MS2,MS3,EN: 0100
    //M3: MS1,MS2,MS3,EN: 0100
    //M5: MS3,EN,MS1,MS2: 0001
    //M6: EN: 0
    //0b01000100010001000001000000000000 - enable motors
    //0b01010101010101010101000000000000 - disable motors
    sendSerialBits(0b01000100010001000001000000000000);

    MotionController* motion = new MotionController(motorPtrs_);
    return motion;
  }

  void disableMotors(){
      sendSerialBits(0b01010101010101010101000000000000);
  }

  private:
  StepperMotor motors_[MOTOR_COUNT];
  StepperMotor* motorPtrs_[MOTOR_COUNT];

};