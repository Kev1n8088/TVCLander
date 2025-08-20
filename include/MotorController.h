#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "Constants.h"

class MotorController {
public:
    MotorController();
    void begin();
    void stop();
    void setSpeed(float speed);
};

#endif // MotorController_H