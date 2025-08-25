#include "PID.h"
#include <Arduino.h>


/**
 * @brief PID constructor
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param dt Time interval in microseconds
 * @param integralMax Maximum value for the integral term
 * @param N Filter coefficient for derivative filtering (higher = less filtering). 0 means no filter
 */
PID::PID(float Kp, float Ki, float Kd, unsigned long dt, float integralMax, float N){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->dt = dt;
    this->integral = 0.0;
    this->output = 0.0;
    this->previousTime = 0;
    this->integralMax = integralMax;
    this->lastError = 0.0;
    this->N = N;
    this->filteredDerivative = 0.0;
}

/**
 * @brief Compute the PID output at set dt interval
 * @param setpoint Desired value
 * @param measuredValue Current value
 */
void PID::compute(float setpoint, float measuredValue){
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
    float timeInSeconds = time / 1000000.0; // convert to seconds

    // Calculate raw derivative
    float rawDerivative = (error - lastError) / timeInSeconds;
    
    // Apply derivative filter using first-order low-pass filter
    // filteredDerivative(k) = (N * rawDerivative(k) + filteredDerivative(k-1)) / (N + 1)
    // This is equivalent to the discrete form of the continuous filter: N/(s+N)
    if (N > 0) {
        filteredDerivative = (N * rawDerivative + filteredDerivative) / (N + 1.0);
    } else {
        // If N = 0, no filtering (use raw derivative)
        filteredDerivative = rawDerivative;
    }

    lastError = error;
    
    // Update integral with anti-windup
    integral += Ki * error * timeInSeconds;
    integral = constrain(integral, -integralMax, integralMax);
    
    // Calculate PID output using filtered derivative
    output = Kp * error + integral + Kd * filteredDerivative;

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

/**
 * @brief Change the derivative filter coefficient
 * @param N New filter coefficient (higher = less filtering, 0 = no filtering)
 */
void PID::setFilterCoefficient(float N) {
    this->N = N;
}

/**
 * @brief Get the current filter coefficient
 * @return Current filter coefficient
 */
float PID::getFilterCoefficient() {
    return N;
}

/**
 * @brief Reset PID controller state
 */
void PID::reset(){
    integral = 0.0; // Reset integral term
    output = 0.0; // Reset output
    previousTime = 0; // Reset previous time
    lastError = 0.0; // Reset last error
    filteredDerivative = 0.0; // Reset filtered derivative
}