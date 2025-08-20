#include "MotorController.h"


/**
 * @brief Constructor for MotorController. Initializes member variables.
 */

MotorController::MotorController() {
}

/**
 * @brief Initializes the motor controller pins.
 */
void MotorController::begin() {
    // Set the motor control pins as outputs
    pinMode(ROLL_MINUS, OUTPUT);
    pinMode(ROLL_PLUS, OUTPUT);
    
    // Reducing frequency to improve low speed control
    analogWriteFrequency(ROLL_MINUS, 25); // Set PWM frequency for ROLL_MINUS pin
    analogWriteFrequency(ROLL_PLUS, 25); // Set PWM frequency for ROLL_PLUS pin
    stop();
    
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

    int a = static_cast<int>(speed / 3.9); // Convert speed to PWM value (0-4095)
    // TODO : adjust conversion based on measured values
    
    a = min(max(a, -4095), 4095); // Limit PWM value to range -4095 to 4095

    analogWriteResolution(12);
    if (a > 0) {
        digitalWrite(ROLL_MINUS, LOW);
        analogWrite(ROLL_PLUS, a); // Set speed on ROLL_PLUS pin
    } else if (a < 0) {
        digitalWrite(ROLL_PLUS, LOW);
        analogWrite(ROLL_MINUS, -a); // Set speed on ROLL_MINUS pin
    } else {
        stop(); // Stop the motor if speed is zero
    }
    analogWriteResolution(8); // Reset resolution to 8 bits for other uses
}