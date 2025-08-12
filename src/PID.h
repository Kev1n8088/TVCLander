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

public:
    PID(float Kp, float Ki, float Kd, unsigned long dt, float integralMax);
    void compute(float setpoint, float measuredValue, float derivative, bool useExternalDerivative = true);
    void changeKs(float Kp, float Ki, float Kd);
    float getOutput();
    void clearIntegral() { integral = 0.0; } // Reset integral term
    void clearDerivative() { lastError = 0.0; } // Reset derivative term

};

#endif // PID_H