#include "StateEstimation.h"
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


StateEstimation::StateEstimation()
    : IMU0(IMU0_RST_PIN), // Initialize SCH1 with the reset pin
      IMU1(SPI, IMU1_CS_PIN), // Initialize ICM456xx with SPI and CS pin
      bmp(),
      lis2mdl(12345)
{
    SPI.begin();
}

int StateEstimation::begin(){
    int failMask = 0;
    if (beginBaro() != 0)  failMask |= 0x01;
    if (beginIMU0() != 0)  failMask |= 0x02;
    if (beginIMU1() != 0)  failMask |= 0x04;
    if (beginMag()  != 0)  failMask |= 0x08;
    return failMask;
}

void StateEstimation::resetVariables(){
    
    // init variables
    resetLinearVariables();

    oriLoopMicros = 0;
    lastOriUpdate = 0;

    ori.zero(); // Reset orientation to initial conditions (1, 0, 0, 0)

    // Initialize gyro bias to zero
    gyroBias[0] = 0.0f; // Gyro bias for X axis in rad/s
    gyroBias[1] = 0.0f; // Gyro bias for Y axis in rad/s
    gyroBias[2] = 0.0f; // Gyro bias for Z axis in rad/s
}

void StateEstimation::resetLinearVariables(){
    // Reset linear variables
    accelLoopMicros = 0;
    lastAccelUpdate = 0;

    // Reset world frame acceleration, velocity, and position
    for (int i = 0; i < 3; i++) {
        worldAccel[i] = 0.0f; // Reset acceleration in world frame
        worldVelocity[i] = 0.0f; // Reset velocity in world frame
        worldPosition[i] = 0.0f; // Reset position in world frame
    }
}

float StateEstimation::getMass(){
    //Returns current expected mass of the vehicle in kg

    // TODO: needs to be dynamic
    return 1.0f; 
}

float StateEstimation::getMomentArm(){
    // Returns the moment arm of the vehicle in meters

    // TODO: needs to be dynamic
    return 0.15f;
}

float StateEstimation::getPitchYawMMOI(){
    // Returns the pitch and yaw moment of inertia in kg*m^2

    // TODO: needs to be dynamic
    return 0.026f;
}

float StateEstimation::getRollMMOI(){
    // Returns the roll moment of inertia in kg*m^2

    // TODO: needs to be dynamic

    return 0.011f;
}

float StateEstimation::getThrust(){
    return thrust;
}

int StateEstimation::beginBaro(){
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    return -1;
  }else{
    Serial.println("BMP3 sensor initialized successfully!");
    return 0;
  }
}


int StateEstimation::beginIMU0(){
    // Uses Interpolated

    SCH1_filter Filter;
    SCH1_sensitivity Sensitivity;
    SCH1_decimation Decimation;

    // SCH1600 filter settings
    Filter.Rate12 = FILTER_RATE; // Should be correct for 500hz
    Filter.Acc12 = FILTER_ACC12;
    Filter.Acc3 = FILTER_ACC3;

    // SCH1600 sensitivity settings
    Sensitivity.Rate1 = SENSITIVITY_RATE1;
    Sensitivity.Rate2 = SENSITIVITY_RATE2; 
    Sensitivity.Acc1 = SENSITIVITY_ACC1;
    Sensitivity.Acc2 = SENSITIVITY_ACC2;
    Sensitivity.Acc3 = SENSITIVITY_ACC3;

    // SCH1600 decimation settings (for Rate2 and Acc2 channels).
    Decimation.Rate2 = DECIMATION_RATE;
    Decimation.Acc2 = DECIMATION_ACC;

    int ret = IMU0.SCH1_init(IMU0_CS_PIN, Filter, Sensitivity, Decimation, true, 10000000, &SPI);
    if(ret != SCH1_OK) {
        Serial.println("SCH1 initialization failed!");
        return ret; // Return the error code
    } else {
        Serial.println("SCH1 initialized successfully!");
        return 0;
    }
}

int StateEstimation::beginIMU1(){
    int ret = IMU1.begin();
    if (ret != 0) {
        Serial.print("ICM456xx initialization failed: ");
        return ret; // Return the error code
    }else{
        Serial.println("ICM456xx initialized successfully!");
        return 0;
    }
}

int StateEstimation::beginMag(){
    lis2mdl.enableAutoRange(true);
    if (! lis2mdl.begin_SPI(LIS2MDL_CS)) {  // hardware SPI mode
        Serial.println("No LIS2MDL Detected!");
        return -1; // Return error code
    } else {
        Serial.println("LIS2MDL initialized successfully!");
        return 0; // Return success code
    }
}

void StateEstimation::estimateState(){  
    switch (vehicleState){
        case 0: // Disarmed State
            if (digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                updatePrelaunch(); // Update gyro biases and orientation before launch
            }
            break;
        case 1: // Armed State. < 30 seconds before launch, do not update gyro bias.
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
            }
            break;
    }
        
}

void StateEstimation::readIMU0(){
    IMU0.SCH1_getData(&rawIMU0Data);
    IMU0.SCH1_convert_data(&rawIMU0Data, &resultIMU0Data);
}

//Axes: X axis is UP (Gravity is -X), Y axis is RIGHT, Z axis is BACK (Right hand rule). Matches Falcon9 standard
//Rotation Axes: X is ROLL, Y is PITCH, Z is YAW
void StateEstimation::oriLoop(){
    if (lastOriUpdate == 0) { // If this is the first update, set lastOriUpdate to current time
        lastOriUpdate = micros();
        return;
    }

    oriLoopMicros = micros();
    float dtOri = (float)(oriLoopMicros - lastOriUpdate) / 1000000.0f; // Convert to seconds
    lastOriUpdate = oriLoopMicros;

    // TODO: Check for inverts, should be correct
    //Uses Rate2 = decimated rate data
    ori.update(radians(resultIMU0Data.Rate2[1]) - gyroBias[0], radians(resultIMU0Data.Rate2[0]) - gyroBias[1], radians(resultIMU0Data.Rate2[2]) - gyroBias[2], dtOri); 
}

void StateEstimation::accelLoop(){
    if (lastAccelUpdate == 0) { // If this is the first update, set lastAccelUpdate to current time
        lastAccelUpdate = micros();
        return;
    }

    
    ori.updateAccel(resultIMU0Data.Acc2[1], resultIMU0Data.Acc2[0], resultIMU0Data.Acc2[2]); // Update the orientation with the measured acceleration in body frame

    worldAccel[0] = ori.worldAccel.b + WORLD_GRAVITY_X; // X acceleration in world frame
    worldAccel[1] = ori.worldAccel.c + WORLD_GRAVITY_Y; // Y acceleration in world frame
    worldAccel[2] = ori.worldAccel.d + WORLD_GRAVITY_Z; // Z acceleration in world frame

    accelLoopMicros = micros();
    float dtAccel = (float)(accelLoopMicros - lastAccelUpdate) / 1000000.0f; // Convert to seconds
    lastAccelUpdate = accelLoopMicros;

    // Update world velocity and position using simple integration
    worldVelocity[0] += worldAccel[0] * dtAccel; // Update X velocity in world frame
    worldVelocity[1] += worldAccel[1] * dtAccel; // Update Y velocity in world frame
    worldVelocity[2] += worldAccel[2] * dtAccel; // Update Z velocity in world frame
    worldPosition[0] += worldVelocity[0] * dtAccel; // Update X position in world frame
    worldPosition[1] += worldVelocity[1] * dtAccel; // Update Y position in world frame
    worldPosition[2] += worldVelocity[2] * dtAccel; // Update Z position in world frame

    thrust = resultIMU0Data.Acc2[1] * getMass(); // Calculate thrust in Newtons based on acceleration in body frame and mass of the vehicle
}

void StateEstimation::updatePrelaunch(){
    // Update gyro biases and orientation before launch. Can be improved with iterative gyro bias estimation and complementary filter but is not necessary for now.

    if (preLaunchLastUpdateMillis == 0) { // If this is the first update, set preLaunchLastUpdateMillis to current time
        preLaunchLastUpdateMillis = millis();
        return;
    }

    if (millis() - preLaunchLastUpdateMillis < PRELAUNCH_UPDATE_INTERVAL) { // If not enough time has passed since last update, return
        return;
    }

    preLaunchLastUpdateMillis = millis(); // Update last update time

    int waitTime = 0; //timeout for waiting for IMU DRY

    gyroBias[0] = 0.0f; // Reset gyro bias for X axis in rad/s
    gyroBias[1] = 0.0f; // Reset gyro bias for Y axis in rad/s
    gyroBias[2] = 0.0f; // Reset gyro bias for Z axis in rad/s

    float accelReading[3]; // Temporary array to store acceleration readings

    Quaternion expectedGravity = Quaternion(-WORLD_GRAVITY_X, -WORLD_GRAVITY_Y, -WORLD_GRAVITY_Z); // Expected gravity vector as experienced by accelerometer
    expectedGravity = expectedGravity.normalize();

    for(int i = 0; i < PRELAUNCH_AVERAGE_COUNT; i++){
        while(digitalRead(IMU0_DRY_PIN) == LOW) { // Wait for DRY pin to be HIGH, default behavior DRY pin is active HIGH
            delay(1);
            waitTime++;
            if (waitTime > 1000) { // If wait time exceeds 1 second, break out of loop
                Serial.println("Timeout waiting for IMU DRY pin to go HIGH");
                return;
            }
        }
        readIMU0(); // Read IMU data only if DRY pin is HIGH
        gyroBias[0] += radians(resultIMU0Data.Rate2[1]); // Accumulate gyro bias for X axis in rad/s
        gyroBias[1] += radians(resultIMU0Data.Rate2[0]); // Accumulate gyro bias for Y axis in rad/s
        gyroBias[2] += radians(resultIMU0Data.Rate2[2]); // Accumulate gyro bias for Z axis in rad/s

        accelReading[0] += resultIMU0Data.Acc2[1]; // X acceleration in body frame
        accelReading[1] += resultIMU0Data.Acc2[0]; // Y acceleration in body frame
        accelReading[2] += resultIMU0Data.Acc2[2]; // Z acceleration in body frame

        delay(PRELAUNCH_AVERAGE_INTERVAL); // Wait for the specified interval before next sample
    }

    // Average the accumulated gyro biases
    gyroBias[0] /= PRELAUNCH_AVERAGE_COUNT; // Average gyro bias for X axis in rad/s
    gyroBias[1] /= PRELAUNCH_AVERAGE_COUNT; // Average gyro bias for Y axis in rad/s
    gyroBias[2] /= PRELAUNCH_AVERAGE_COUNT; // Average gyro bias for Z axis in rad/s

    // Average the acceleration readings
    //accelReading[0] /= PRELAUNCH_AVERAGE_COUNT; // Average X acceleration in body frame
    //accelReading[1] /= PRELAUNCH_AVERAGE_COUNT; // Average Y acceleration in body frame
    //accelReading[2] /= PRELAUNCH_AVERAGE_COUNT; // Average Z acceleration in body frame
    Quaternion actualAccel = Quaternion(accelReading[1], accelReading[0], accelReading[2]); // Create quaternion from acceleration readings in body frame
    actualAccel = actualAccel.normalize(); // Normalize the quaternion to get unit vector

    ori.orientation = actualAccel.rotation_between_vectors(expectedGravity); // Compute the orientation quaternion by rotating expected gravity vector to actual acceleration vector
    ori.orientation = ori.orientation.normalize(); // Normalize the orientation quaternion
}