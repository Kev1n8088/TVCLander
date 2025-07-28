
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
    float getOutput();

};