#ifndef GPSHANDLER_H
#define GPSHANDLER_H

#include <Arduino.h>
#include <SparkFun_LG290P_GNSS.h>
#include "Constants.h"

typedef struct positionAndVelocity {
        float latitude;
        float longitude;
        float altitude;
        float velocityNorth;
        float velocityEast;
        float velocityDown;
} positionAndVelocity;

typedef struct XYZ { // Falcon 9 convention
    float x;
    float y;
    float z;
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
    int homeAverageCount;
    uint32_t lastRTCMMillis; // Last time RTCM correction was received

    positionAndVelocity home; // Home position for GPS
    positionAndVelocity current;
    XYZ getDistance(positionAndVelocity pos1, positionAndVelocity pos2);
    GPSInfo gpsInfo;

    void busyWait(int seconds);
    
public:
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

};

#endif // GPSHANDLER_H