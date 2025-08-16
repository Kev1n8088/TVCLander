#include "MotorController.h"


/**
 * @brief Constructor for MotorController. Initializes member variables.
 */

MotorController::MotorController() {
    pinMode(ROLL_MINUS, OUTPUT);
    pinMode(ROLL_PLUS, OUTPUT);
}

void MotorController::stop(){
    // Stop the motor by setting both pins to LOW
    digitalWrite(ROLL_MINUS, LOW);
    digitalWrite(ROLL_PLUS, LOW);
}


/**
 * @brief Sets the speed of the motor.
 * @param speed Speed value in rad/s
 */
void MotorController::setSpeed(float speed) {
    // Set the speed of the motor
    
    speed = min(max(speed, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED); // Limit speed to max wheel speed

    int a = static_cast<int>(speed); // Convert speed to PWM value (0-255)
    // TODO : adjust conversion based on measured values

    if (a > 0) {
        digitalWrite(ROLL_MINUS, LOW);
        analogWrite(ROLL_PLUS, a); // Set speed on ROLL_PLUS pin
    } else if (a < 0) {
        digitalWrite(ROLL_PLUS, LOW);
        analogWrite(ROLL_MINUS, -a); // Set speed on ROLL_MINUS pin
    } else {
        stop(); // Stop the motor if speed is zero
    }
}