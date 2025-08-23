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
    //analogWriteFrequency(ROLL_MINUS, 25); // Set PWM frequency for ROLL_MINUS pin
    //analogWriteFrequency(ROLL_PLUS, 25); // Set PWM frequency for ROLL_PLUS pin
    stop();
    
}

void MotorController::stop(){
    // Stop the motor by setting both pins to HIGH (brake mode)
    analogWrite(ROLL_MINUS, 255);
    analogWrite(ROLL_PLUS, 255);
}

/**
 * @brief Sets the speed of the motor using slow decay mode.
 * @param speed Speed value in rad/s
 */
void MotorController::setSpeed(float speed) {
    // Set the speed of the motor using slow decay (brake) mode
    
    speed = min(max(speed, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED); // Limit speed to max wheel speed

    int a = static_cast<int>(speed / 6.275); // Convert speed to PWM value (0-255)
    // TODO : adjust conversion based on measured values

    a = min(max(a, -255), 255); // Limit PWM value to range -255 to 255

    if (a > 0) {
        // Forward direction with slow decay
        analogWrite(ROLL_MINUS, 255);        // Keep high-side ON
        analogWrite(ROLL_PLUS, 255 - a);     // PWM the other side (inverted)
    } else if (a < 0) {
        // Reverse direction with slow decay  
        analogWrite(ROLL_PLUS, 255);         // Keep high-side ON
        analogWrite(ROLL_MINUS, 255 - (-a)); // PWM the other side (inverted)
    } else {
        stop(); // Stop the motor if speed is zero (brake mode)
    }
}