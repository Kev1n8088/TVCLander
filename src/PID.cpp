#include "PID.h"
#include <Arduino.h>


/**
 * @brief PID constructor
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param dt Time interval in microseconds
 * @param integralMax Maximum value for the integral term
 */
PID::PID(float Kp, float Ki, float Kd, unsigned long dt, float integralMax){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->dt = dt;
    this->integral = 0.0;
    this->output = 0.0;
    this->previousTime = 0;
    this->integralMax = integralMax;
    this->lastError = 0.0;
}

/**
 * @brief Compute the PID output at set dt interval
 * @param setpoint Desired value
 * @param measuredValue Current value
 * @param derivative Derivative of the error (if useExternalDerivative is true)
 * @param useExternalDerivative If true, use provided derivative; otherwise, compute internally
 */
void PID::compute(float setpoint, float measuredValue, float derivative, bool useExternalDerivative){
    if(previousTime == 0){
        previousTime = micros(); // Initialize previous time if not set
        return; // Skip first computation
    }

    float time = micros() - previousTime; // time in microseconds
    
    if (time < dt){
        return; // not enough time has passed
    }
    previousTime = micros();

    float error = setpoint - measuredValue;

    if(!useExternalDerivative){
        derivative = (error - lastError) / (time / 1000000.0); // convert dt to seconds
    }else{
        derivative = -derivative ; // Invert derivative, as its derivative of plant rather than derivative of error, for PID control
    }

    lastError = error;
    integral += Ki * error * time / 1000000.0; // convert dt to seconds
    // Limit integral to prevent windup
    integral = constrain(integral, -integralMax, integralMax);
    output = Kp * error + integral + Kd * derivative;

    // Limit output to a certain range if needed
    // output = constrain(output, min_output, max_output);
}

/**
 * @brief Get the PID output
 * @return PID output
 */
float PID::getOutput(){
    return output;
}

/**
 * @brief Change PID gains
 * @param Kp New proportional gain
 * @param Ki New integral gain
 * @param Kd New derivative gain
 */
void PID::changeKs(float Kp, float Ki, float Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}