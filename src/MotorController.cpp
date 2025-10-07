#include "MotorController.h"




/**
 * @brief Constructor for MotorController. Initializes member variables.
 */

MotorController::MotorController() {
    pwm = Adafruit_PWMServoDriver();
}

/**
 * @brief Initializes the motor controller pins.
 */
void MotorController::begin() {

    // Set the motor control pins as outputs
    pwm.begin();
    pwm.setPWMFreq(100); // low freq for better control
    Wire.setClock(400000);

    stop();
    
}

void MotorController::stop(){
    // Stop the motor by setting both pins to LOW
    pwm.setPin(ROLL_MINUS, 4096);
    pwm.setPin(ROLL_PLUS, 4096);
}

/**
 * @brief Sets the speed of the motor using slow decay mode.
 * @param speed Speed value in rad/s
 */
void MotorController::setSpeed(float speed) {
    // Set the speed of the motor using slow decay (brake) mode
    
    speed = min(max(speed, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED); // Limit speed to max wheel speed

    int a = static_cast<int>(speed / 0.39); // Convert speed to PWM value (0-4095)
    // TODO : adjust conversion based on measured values

    a = min(max(a, -4095), 4095); // Limit PWM value to range -4095 to 4095

    if (a > 0) {
        pwm.setPin(ROLL_MINUS, 4096);
        pwm.setPin(ROLL_PLUS, 4095 - (a));
    } else if (a < 0) {
        pwm.setPin(ROLL_PLUS, 4096);
        pwm.setPin(ROLL_MINUS, 4095 - (-a));
    } else {
        stop(); // Stop the motor if speed is zero (brake mode)
    }
}