#ifndef GPSHANDLER_H
#define GPSHANDLER_H

#include <Arduino.h>
#include <SparkFun_LG290P_GNSS.h>
#include "Constants.h"

typedef struct positionAndVelocity {
        double latitude;
        double longitude;
        double altitude;
        double velocityNorth;
        double velocityEast;
        double velocityDown;
} positionAndVelocity;

typedef struct XYZ { // Falcon 9 convention
    double x;
    double y;
    double z;
} XYZ;

typedef struct GPSInfo{
    int fixType; //"No-Fix", "3D-Fix", "DGPS-Fix", "GPS-PPS", "RTK-Fix", "RTK-Flt" 
    positionAndVelocity pos;
    positionAndVelocity home;
    XYZ xyz;
    int satsInView;
    int satsUsed;
    float pdop;
    float error2D;
    float error3D;
    uint32_t timeOfWeek; // Time of week in milliseconds
    uint32_t rtcmAge; // Age of RTCM correction in milliseconds
}  GPSInfo;

class GPSHandler {
private:
    LG290P gps; 
    uint64_t lastUpdateMillis;
    bool DRY;
    uint32_t lastRTCMMillis; // Last time RTCM correction was received
    uint32_t lastMemoryCleanup; // Last time GPS memory was cleaned

    positionAndVelocity home; // Home position for GPS
    positionAndVelocity current;
    XYZ getDistance(positionAndVelocity pos1, positionAndVelocity pos2);
    GPSInfo gpsInfo;

    void busyWait(int seconds);
    
public:
    float lastUpdateTime;
    float updateInterval;

    GPSHandler();
    int begin();
    void gpsLoop();
    bool dataReady();
    void setCurrentAsHome();
    void resetHome();

    GPSInfo& getGPSInfo() {
        return gpsInfo;
    }

    void sendRTCMCorrection(const uint8_t* data, size_t length);
    void forceMemoryCleanup(); // Force immediate memory cleanup

    float getUpdateInterval() const {
        return updateInterval;
    }
};

#endif // GPSHANDLER_H