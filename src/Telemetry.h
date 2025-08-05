#ifndef TELEMETRY_H
#define TELEMETRY_H


#include <Arduino.h>
#include "StateEstimation.h"
#include "Constants.h"

class Telemetry{
private:
    uint64_t lastLogMillis;
    uint64_t lastTelemetryMillis;

    
    size_t telemetryBufferUsed;
    String logFileName;

    int oldVehicleState;
    int SDGood;

    void dumpToSD();
    void dataLog(float timeSec, float quaternion[4], float worldAccel[3],
                 float worldVelocity[3], float worldPosition[3], float rawAccel[3],
                float rawGyro[3], float gyroBias[3], float attitudeSetpoint[2], float servoCommand[2], 
                float thrust, float reactionWheelSpeed, int vehicleState, int sensorStatus, int SDGood);
    void sendTelemetry(float timeSec, float quaternion[4], float worldAccel[3],
                       float worldVelocity[3], float worldPosition[3], float rawAccel[3],
                      float rawGyro[3], float gyroBias[3], float attitudeSetpoint[2], 
                      float servoCommand[2], float thrust, float reactionWheelSpeed, 
                      int vehicleState, int sensorStatus, int SDGood);
public:
    Telemetry();
    void begin();
    void telemetryLoop(StateEstimation& state);
    void setLogFileName(const String& name) { logFileName = name; } // Setter for file name
};

#endif // TELEMETRY_H