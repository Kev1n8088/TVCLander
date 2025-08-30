#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class MotorController {
private:
    Adafruit_PWMServoDriver pwm;

public:
    MotorController();
    void begin();
    void stop();
    void setSpeed(float speed);
};

#endif // MotorController_H