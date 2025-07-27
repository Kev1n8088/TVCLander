#include <Orientation.h>
#include <Quaternion.h>
#include <Arduino.h>
#include <Constants.h>
#include <SPI.h>
#include <SCH1.h>
#include <Adafruit_NeoPixel.h>
#include "ICM45686.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LIS2MDL.h>

class StateEstimation
{
private:
    SCH1 IMU0;
    ICM456xx IMU1;
    Adafruit_BMP3XX bmp;
    Adafruit_LIS2MDL lis2mdl;

    uint64_t oriLoopMicros;
    uint64_t lastOriUpdate;

    uint64_t accelLoopMicros;
    uint64_t lastAccelUpdate;

    Orientation ori;
    EulerAngles oriEuler;

    SCH1_raw_data rawIMU0Data;
    SCH1_result resultIMU0Data;

    uint64_t preLaunchLastUpdateMillis; //PRELAUNCH, last time gyro biases/orientation were updated in milliseconds

    float gyroBias[3]; // Gyro bias in rad/s

    int vehicleState; // 0 = disarmed, 1 armed, 2 launched before apogee, 3 past apogee, 4 landing burn;
    float launchTime; // Time of launch in seconds 

    float worldAccel[3];
    float worldVelocity[3]; // World frame velocity in m/s
    float worldPosition[3]; // World frame position in m 

    float thrust;


    int beginBaro();
    int beginIMU0();
    int beginIMU1();
    int beginMag();

    void oriLoop();
    void accelLoop();
    void readIMU0();

    void updatePrelaunch(); // Update gyro biases and orientation before launch 

    void resetVariables();
    void resetLinearVariables();

public:
    StateEstimation();
    void estimateState();
    int begin();
    float getMass();
    float getPitchYawMMOI(); 
    float getRollMMOI();
    float getMomentArm();
    float getThrust();

    
};