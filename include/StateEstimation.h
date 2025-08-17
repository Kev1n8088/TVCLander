#ifndef STATEESTIMATION_H
#define STATEESTIMATION_H

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
#include "PID.h"
#include "GPSHandler.h"
#include "KalmanFilter.h"
#include <Servo.h>
#include "MotorController.h"

class StateEstimation
{
private:
    SCH1 IMU0;
    ICM456xx IMU1;
    Adafruit_BMP3XX bmp;
    Adafruit_LIS2MDL lis2mdl;
    GPSHandler gps;

    PID PitchPID;
    PID YawPID;
    PID RollPID;
    PID PitchStabilizationPID;
    PID YawStabilizationPID;
    PID YAscentPID;
    PID ZAscentPID;
    PID YDescentPID;
    PID ZDescentPID;

    KalmanFilter XPos;
    KalmanFilter YPos;
    KalmanFilter ZPos;



    MotorController RollMotor; // Motor controller for roll motor

    uint64_t lastStateEstimateMicros; // Last time state estimation was run in microseconds

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
    float gyroRemovedBias[3]; // Gyro data with bias removed in rad/s

    int vehicleState; // 0 = disarmed, 1 armed, 2 launched characterizing misalign, 3 launched guidance, 4 launched returning to vertical, //5 past apogee, 6 landing burn, 7 landed, -1 abort state;
    float launchTime; // Time of launch in seconds 
    float timeSinceLaunch; // Time since launch in seconds

    float accelCalibrated[3]; // Calibrated acceleration in body frame
    float measuredWorldAccel[3];
    float worldAccel[3]; // Up is X
    float worldVelocity[3]; // World frame velocity in m/s
    float worldPosition[3]; // World frame position in m 

    uint64_t lastGimbalMisalignMicros;
    float gimbalMisalignAccumulator[2]; //Rate accumulator, gets current angular displacement from start to current of calculation
    float gimbalMisalignTime;
    float gimbalMisalign[2]; // Misalign yaw and pitch
    float gimbalForceAccumulator;

    float attitudeSetpoint[2]; // Yaw and Pitch setpoints for gimbal
    float angularAccelCommand[2]; // before transform applied for servo command

    float gimbalAngle[2]; // after transform applied for servo command, before mapping
    
    float projectedLandingPosition[2];

    float wheelSpeed;

    uint64_t lastActuatorMicros;
    uint64_t lastWheelMicros;

    float thrust;

    uint8_t sensorStatus;

    float apogeeAltitude;
    float landingIgnitionAltitude;

    float accelUncertainty[3];
    float velocityUncertainty[3];
    float positionUncertainty[3];


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

    void detectLaunch();
    void detectApogee();
    void computeGimbalMisalign();
    void PIDLoop();
    void actuateServos(bool actuate = true, bool includePID = true); // Actuate servos based on attitude setpoints
    void actuateWheel();
    void GPSLoop();
    void firePyroWhenReady();
    

public: 
    StateEstimation();
    void estimateState();
    int begin();
    float getMass();
    float getPitchYawMMOI(); 
    float getMomentArm();
    float getThrust();
    int getVehicleState() { return vehicleState; }
    float getTimeSinceLaunch() { return timeSinceLaunch; }
    uint8_t getSensorStatus() { return sensorStatus; }
    uint8_t setVehicleState(int state); 

    bool getPyroCont();

    const float* getEulerAngle();

    // Telemetry getters
    const Quaternion& getOrientationQuaternion() const { return ori.orientation; }
    void getOrientationQuaternionArray(float out[4]) const {
        out[0] = ori.orientation.a;
        out[1] = ori.orientation.b;
        out[2] = ori.orientation.c;
        out[3] = ori.orientation.d;
    }
    const float* getWorldAccel() const { return worldAccel; }
    const float* getWorldVelocity() const { return worldVelocity; }
    const float* getWorldPosition() const { return worldPosition; }
    // Returns raw accel in body frame, up right back
    void getRawAccel(float out[3]) const {
        out[0] = resultIMU0Data.Acc1[1]; // X (UP)
        out[1] = resultIMU0Data.Acc1[0]; // Y (RIGHT)
        out[2] = resultIMU0Data.Acc1[2]; // Z (BACK)
    }
    // Returns raw gyro in body frame
    void getRawGyro(float out[3]) const {
        out[0] = resultIMU0Data.Rate1[2]; // (yaw)
        out[1] = -resultIMU0Data.Rate1[0]; // (PITCH)
        out[2] = resultIMU0Data.Rate1[1]; // (roll)
    }
    const float* getGyroBias() const { return gyroBias; }
    const float* getGyroRemovedBias() const { return gyroRemovedBias; }
    const float* getAccelCalibrated() const { return accelCalibrated; }
    float getLaunchTime() { return launchTime; }
    float getIgnitionAltitude() { return landingIgnitionAltitude; }
    float getApogeeAltitude() { return apogeeAltitude; }

    const float* getAttitudeSetpoint() const { return attitudeSetpoint; } // Yaw, pitch
    const float* getGimbalMisalign() const { return gimbalMisalign; } // Yaw, pitch
    const float* getGimbalAngle() const { return gimbalAngle; } // Yaw, pitch
    const float* getAngularAccelCommand() const { return angularAccelCommand; } // Yaw, pitch

    float getWheelSpeed() { return wheelSpeed; } // Returns the current wheel speed in rad/s
    const float* getProjectedLandingPosition() const { return projectedLandingPosition; } // Returns the projected landing position in Y, Z

    const float* getRawRotatedAccel() const {return measuredWorldAccel; } // Returns the raw rotated acceleration in world frame

    const float* getAccelUncertainty() const { return accelUncertainty; } // Returns the acceleration uncertainty in world frame
    const float* getVelocityUncertainty() const { return velocityUncertainty; } // Returns the velocity uncertainty in world frame
    const float* getPositionUncertainty() const { return positionUncertainty; } // Returns the position uncertainty in world frame

    GPSInfo& getGPSInfo() { return gps.getGPSInfo(); }
    GPSHandler& getGPSHandler() { return gps; }
};

#endif // STATEESTIMATION_H