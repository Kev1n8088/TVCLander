#ifndef PID_H
#define PID_H
#include <Arduino.h>

class PID
{
private:
    float Kp, Ki, Kd;
    unsigned long dt; // time interval in microseconds 
    float integralMax;
    float integral;
    float output;
    unsigned long previousTime;
    float lastError;
    float N; // Filter coefficient for derivative filtering
    float filteredDerivative; // Filtered derivative value
    bool derivativeOnMeasurement;

public:
    /**
     * @brief PID constructor
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param dt Time interval in microseconds
     * @param integralMax Maximum value for the integral term
     * @param N Filter coefficient (default 100, higher = less filtering, 0 = no filtering)
     */
    PID(float Kp, float Ki, float Kd, unsigned long dt, float integralMax, float N = 100.0, bool derivativeOnMeasurement = false);

    void compute(float setpoint, float measuredValue, float derivative=0, bool useExternalDerivative=false);
    void changeKs(float Kp, float Ki, float Kd);
    void setFilterCoefficient(float N);
    float getFilterCoefficient();
    float getOutput();
    void clearIntegral() { integral = 0.0; } // Reset integral term
    void clearDerivative() { lastError = 0.0; filteredDerivative = 0.0; } // Reset derivative terms
    void reset();

};

#endif // PID_H