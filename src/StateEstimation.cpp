#include "StateEstimation.h"
#include <Orientation.h>
#include <Quaternion.h>
#include <Arduino.h>
#include <Constants.h>
#include <SPI.h>
#include <SCH1.h>
#include <Adafruit_NeoPixel.h>
#include "ICM45686.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LIS2MDL.h>
#include "GPSHandler.h"
#include <PWMServo.h>

PWMServo PitchServo;
PWMServo YawServo;


/**
 * @brief Constructor for StateEstimation. Initializes sensors and SPI.
 */
StateEstimation::StateEstimation()
    : IMU0(IMU0_RST_PIN), // Initialize SCH1 with the reset pin
      IMU1(SPI, IMU1_CS_PIN), // Initialize ICM456xx with SPI and CS pin
      bmp(),
      lis2mdl(12345), 
      gps(),
      PitchPID(18,0,6,1000,0, 100.0, true),
        YawPID(18,0,6,1000,0, 100.0, true),
        YPID(0.1,0.00005,0.1,1000,0.005),
        ZPID(0.1,0.00005,0.1,1000,0.005),
        XPos(),
        YPos(),
        ZPos()
{
}

/**
 * @brief Initializes all sensors and returns a bitmask of failures.
 * @return Bitmask indicating which sensors failed to initialize.
 */
int StateEstimation::begin(){
    XPos.setGPSPosNoise(0.01); //Higher pos noise for vertical axis
    XPos.setProcessNoise(0.01, 0.01, 0.3); // Higher process noise for vertical axis
    SPI.begin();
    resetVariables();
    pinMode(IMU0_DRY_PIN, INPUT); // Set the dry pin for SCH1 as input
    pinMode(LAND_CONTINUITY, INPUT); // Set the land continuity pin as input
    pinMode(LAND_PYRO, OUTPUT); // Set the land pyro pin as output
    pinMode(CHUTE_CONTINUITY, INPUT); // Set the chute continuity pin as input
    pinMode(CHUTE_PYRO, OUTPUT); // Set the chute pyro pin as output


    digitalWrite(LAND_PYRO, LOW); // Initialize the land pyro pin to LOW
    digitalWrite(CHUTE_PYRO, LOW); // Initialize the chute pyro pin to LOW
    
    //RollMotor.begin();
    PitchServo.attach(PITCH_SERVO); // Attach the pitch servo to the specified pin
    YawServo.attach(YAW_SERVO); // Attach the yaw servo to the specified pin

    //debug stuff
    dt_loop = 0;
    lastIMUReadMicros = micros(); // Initialize last IMU read time

    uint8_t failMask = 0;
    //if (beginBaro() != 0)  failMask |= 0x01;
    if (beginIMU0() != 0)  failMask |= 0x02;
    //if (beginIMU1() != 0)  failMask |= 0x04;
    //if (beginMag()  != 0)  failMask |= 0x08;
    GPSStatus = gps.begin();
    if (GPSStatus != 0)  failMask |= 0x10;
    sensorStatus = failMask; // Store the sensor status in the class variable
    return failMask;
}

/**
 * @brief Resets all state estimation variables to initial conditions.
 */
void StateEstimation::resetVariables(){
    maxMillisPerLoop = 0;
    lastBodyAngAccelCommand[0] = 0;
    lastBodyAngAccelCommand[1] = 0;
    filteredActualAngularAccel[0] = 0;
    filteredActualAngularAccel[1] = 0;

    lastGyroRemovedBias[0] = 0;
    lastGyroRemovedBias[1] = 0;

    expectedGravity = Quaternion(-WORLD_GRAVITY_X, -WORLD_GRAVITY_Y, -WORLD_GRAVITY_Z);
    actualAccel = Quaternion(0, 0, 0,0 ); // Reset actual acceleration vector in body frame to zero
    GPSLocation = Quaternion(0, 0.35, 0, 0); 
    worldGPSLocation = Quaternion(0, 0, 0, 0);
    GPSVelocityWorld = Quaternion(0, 0, 0, 0);
    GPSVelocityBody = Quaternion(0, 0, 0, 0);
    angularAccelCommandVector = Quaternion(0, 0, 0, 0);
    bodyAngularAccelCommandVector = Quaternion(0, 0, 0, 0);
    rollRateCompensatedBodyToWorld = Quaternion(0, 0, 0, 0);

    lastStateEstimateMicros = 0; // Last time state estimation was run in microseconds

    
    // init variables
    resetLinearVariables();

    timeSinceLaunch = 0.0f; // Time since launch in seconds
    launchTime = 0.0f; // Launch time in seconds
    landingIgnitionTime = 0.0f; // Landing ignition time in seconds

    wheelSpeed = 0.0f; // Wheel speed in rad/s

    gimbalMisalign[0] = 0;
    gimbalMisalign[1] = 0;

    gimbalMisalignAccumulator[0] = 0;
    gimbalMisalignAccumulator[1] = 0;

    gimbalMisalignTime = 0;

    gimbalForceAccumulator = 0;
    MMOIAccumulator = 0;
    momentArmAccumulator = 0;

    oriLoopMicros = 0;
    lastOriUpdate = 0;

    lastGimbalMisalignMicros = 0;
    lastActuatorMicros = 0;
    lastWheelMicros = 0;

    ori.zero(); // Reset orientation to initial conditions (1, 0, 0, 0)

    // Initialize gyro bias to zero
    gyroBias[0] = 0.0f; // Gyro bias for X axis in rad/s
    gyroBias[1] = 0.0f; // Gyro bias for Y axis in rad/s
    gyroBias[2] = 0.0f; // Gyro bias for Z axis in rad/s

    gyroRemovedBias[0] = 0.0f; // Gyro bias removed for X axis in rad/s
    gyroRemovedBias[1] = 0.0f; // Gyro bias removed for Y axis in rad/s
    gyroRemovedBias[2] = 0.0f; // Gyro bias removed for Z axis in rad/s

    gyroLowPassed[0] = 0.0f; // Low passed gyro for X axis in rad/s
    gyroLowPassed[1] = 0.0f; // Low passed gyro for Y axis in rad/s
    gyroLowPassed[2] = 0.0f; // Low passed gyro for Z axis in rad/s


    attitudeSetpoint[0] = 0.0f; // Yaw setpoint in rad
    attitudeSetpoint[1] = 0.0f; // Pitch setpoint in rad

    angularAccelCommand[0] = 0.0f; // Yaw angular acceleration command in rad/s^2
    angularAccelCommand[1] = 0.0f; // Pitch angular acceleration command in rad/s^2

    gimbalAngle[0] = 0.0f; // Yaw angle in rad
    gimbalAngle[1] = 0.0f; // Pitch angle in rad

    thrust = 12.5f; // Initial thrust in N
}

/**
 * @brief Resets linear state variables (acceleration, velocity, position).
 */
void StateEstimation::resetLinearVariables(){
    // Reset linear variables
    accelLoopMicros = 0;
    lastAccelUpdate = 0;

    apogeeAltitude = 0;
    landingIgnitionAltitude = 0;

    positionSetpoint[0] = 0.0f; // Projected landing position Y
    positionSetpoint[1] = 0.0f; // Projected landing position Z

    // Reset world frame acceleration, velocity, and position
    for (int i = 0; i < 3; i++) {
        measuredWorldAccel[i] = 0.0f; // Reset measured acceleration in world frame
        worldAccel[i] = 0.0f; // Reset acceleration in world frame
        worldVelocity[i] = 0.0f; // Reset velocity in world frame
        worldPosition[i] = 0.0f; // Reset position in world frame

        accelCalibrated[i] = 0.0f; // Reset calibrated acceleration in body frame
        accelUncertainty[i] = 0.0f; // Reset acceleration uncertainty in world frame
        velocityUncertainty[i] = 0.0f; // Reset velocity uncertainty in world frame
        positionUncertainty[i] = 0.0f; // Reset position uncertainty in world frame
        adjustedGPSPosition[i] = 0.0f; // Reset adjusted GPS position in world frame
        adjustedGPSVelocity[i] = 0.0f; // Reset adjusted GPS velocity in world frame

    }
}

/**
 * @brief Returns the current expected mass of the vehicle in kg.
 */
float StateEstimation::getMass(){
    //Returns current expected mass of the vehicle in kg

    // TODO: adjust nums
    if (vehicleState < 5){
        return max(1.174, 1.234 - 0.01806 * (timeSinceLaunch + 0.26)); // 1.2kg at launch, losing 18.06g/s, min 1.14kg
    }else{
        if(landingIgnitionTime < 0.05f){
            return 1.174; // If landing ignition time not set, landing burn has not yet started, return 1.188kg
        }
        return max(1.061, 1.121 - 0.01806 * max(0, timeSinceLaunch - landingIgnitionTime - 0.1));
    }
}

/**
 * @brief Returns the moment arm of the vehicle in meters.
 */
float StateEstimation::getMomentArm(){
    // Returns the moment arm of the vehicle in meters

    // TODO: adjust nums
    if (vehicleState < 5){
        return min(0.151906, 0.138444 + 0.0038463 * (timeSinceLaunch + 0.26)); 
    }else{
        if(landingIgnitionTime < 0.05f){
            return 0.151906; 
        }
        return min(0.173, 0.163422 + 0.0027366 * max(0, timeSinceLaunch - landingIgnitionTime - 0.1));
    }
}

/**
 * @brief Returns the pitch and yaw moment of inertia in kg*m^2.
 */
float StateEstimation::getPitchYawMMOI(){
    // Returns the pitch and yaw moment of inertia in kg*m^2

    // TODO: adjust nums
    if (vehicleState < 5){
        return max(0.05767, 0.06208 - 0.00126 * (timeSinceLaunch + 0.26));
    }else{
        if(landingIgnitionTime < 0.05f){
            return 0.05767;
        }
        return max(0.05243, 0.05429 - 0.0005314 * max(0, timeSinceLaunch - landingIgnitionTime - 0.1));
    }
}

/**
 * @brief Returns the current thrust value. Minimum is 12.5 N, maximum is 25.0 N.
 */
float StateEstimation::getThrust(){

    return min(max(8.0f, thrust), 35.0f);
}

/**
 * @brief Initializes the barometric sensor.
 * @return 0 on success, -1 on failure.
 */
int StateEstimation::beginBaro(){
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    return -1;
  }else{
    Serial.println("BMP3 sensor initialized successfully!");
    return 0;
  }
}


/**
 * @brief Initializes the primary IMU (SCH1).
 * @return 0 on success, error code otherwise.
 */
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
        if(DEBUG_SERIAL){
            Serial.println("SCH1 initialization failed!");
        }
        return ret; // Return the error code
    } else {
        Serial.println("SCH1 initialized successfully!");
        return 0;
    }
}

/**
 * @brief Initializes the secondary IMU (ICM456xx).
 * @return 0 on success, error code otherwise.
 */
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

/**
 * @brief Initializes the magnetometer.
 * @return 0 on success, error code otherwise.
 */
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

/**
 * @brief Main state estimation loop. Updates state variables periodically.
 */
void StateEstimation::estimateState(){  
    // Main state estimation loop. This function should be called periodically to update the state estimation.
    if (lastStateEstimateMicros == 0) { // If this is the first update, set lastStateEstimateMicros to current time
        lastStateEstimateMicros = micros();
        return;
    }

    if (micros() - lastStateEstimateMicros < STATE_ESTIMATION_INTERVAL_US) { // If not enough time has passed since last update, return
        return;
        
    }

    maxMillisPerLoop = maxMillisPerLoop * 0.5 + 0.5 * (micros() - lastStateEstimateMicros) / 1000; // Update typical loop time in milliseconds
    lastStateEstimateMicros = micros(); // Update last state estimate time

    if (launchTime != 0.0f){
        timeSinceLaunch = millis() / 1000.0f - launchTime; // Calculate time since launch in seconds
    }

    gps.gpsLoop(); // Update GPS data

    if(vehicleState > 4 && vehicleState < 6){
        if (gps.getGPSInfo().fixType < 4) { // If lost GPS
            abort();
        }
    }


    // State machine
    switch (vehicleState){
        case 0: // Disarmed State
            if (digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                updatePrelaunch(); // Update gyro biases and orientation before launch
                digitalWrite(LAND_PYRO, LOW);
                digitalWrite(CHUTE_PYRO, LOW);
                //RollMotor.stop(); // Stop roll motor
            }
            break;
        case 1: // Armed State. < 30 seconds before launch, do not update gyro bias.
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH

                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                actuateServos(false); // center servos without actuating them
                //RollMotor.stop(); // Stop roll motor
                digitalWrite(LAND_PYRO, LOW);
                digitalWrite(CHUTE_PYRO, LOW);
                detectLaunch();
            }
            break;
        case 2: // No gimbal command state, characterizing misalign
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                computeGimbalMisalign();
                actuateServos(false); // center servos without actuating them
                digitalWrite(LAND_PYRO, LOW);
                digitalWrite(CHUTE_PYRO, LOW);
            }
            if (timeSinceLaunch > MISALIGN_CHARACTERIZATION_TIME){
                vehicleState = 3;
            }
            break;
        case 3: //Standard guidance state
        case 4: //return to vertical state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints 
                adaptiveGimbalMisalignEstimation();
                actuateServos();
                digitalWrite(LAND_PYRO, LOW);
                digitalWrite(CHUTE_PYRO, LOW);
                detectApogee();
            }
            // if(vehicleState == 3 && timeSinceLaunch > GIMBAL_STABILIZATION_TIME){ // After gimbal stabilization time, switch to return to vertical state
            //     vehicleState = 4;
            // }
        case 5: // Past apogee state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                adaptiveGimbalMisalignEstimation();
                actuateServos();
                firePyroWhenReady(); // Fire pyro when ready
            }
            break;
        case 6: // landing burn state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                adaptiveGimbalMisalignEstimation();
                actuateServos();
                firePyroWhenReady(); // Keep firing pyro to ensure ignition
            }
            if(timeSinceLaunch > 15.0f) { // If time since launch is greater than 15 seconds, assume landing. Very simple because no harm in staying in this state for longer than necessary.
                vehicleState = 7; // Transition to landed state
            }
            break;
        case 7: // landed state
                actuateServos(false);
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            break;
        // below are test states - they do not contain transition logic and are used for testing purposes only'
        case 64: //Adaptive gimbal misalignment estimation test state (no use of PIDs, only has gimbal misalignment estimation running)
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                actuateServos(true, false);
                adaptiveGimbalMisalignEstimation();
                //digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        // Servo test cases - no wheel
        case 65: // Stabilization test state (no use of positional PID)
        case 66:// Positional PID test state (setpoint 0)
        case 67:// Trajectory Positional PID test state (setpoint follows trajectory)
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos();
                //digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        case 68: //misalignment test - operation: rotate the vehicle and the gimbal should move in the opposite direction 
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
            
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                computeGimbalMisalign();
                actuateServos(true, false); // Actuate but do not include PID outputs
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        case 69: //Testing arm phase (kalman gets reset)
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                dt_loop = micros() - lastIMUReadMicros; // Calculate time since last IMU read in microseconds
                lastIMUReadMicros = micros(); // Update last IMU read time
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                actuateServos(false); // center servos without actuating them
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        case 70: //Pyro fire:
            digitalWrite(LAND_PYRO, HIGH);
            readIMU0(); // Read IMU data only if DRY
            oriLoop(); // Call orientation loop to update orientation
            accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
            GPSLoop();
            actuateServos(false); // center servos without actuating them
            break;
        case 71:
            digitalWrite(CHUTE_PYRO, HIGH);
            break;
    }
        
}

/**
 * @brief Reads data from GPS and updates kalman filter states.
 */
void StateEstimation::GPSLoop(){
    if (!gps.dataReady()){
        return; // If GPS data is not ready, return
    }

    // Update GPS data
    worldGPSLocation = ori.orientation.rotate(GPSLocation); // Rotate GPS location vector to body frame
    adjustedGPSPosition[0] = gps.getGPSInfo().xyz.x - worldGPSLocation.b;
    adjustedGPSPosition[1] = gps.getGPSInfo().xyz.y - worldGPSLocation.c;
    adjustedGPSPosition[2] = gps.getGPSInfo().xyz.z - worldGPSLocation.d; // Adjust GPS position for lever arm and orientation

    GPSVelocityBody = Quaternion(0, gyroLowPassed[2], gyroLowPassed[1], gyroLowPassed[0]) * GPSLocation; // Compute cross product of GPS location and gyro low passed to get velocity in body frame
    GPSVelocityBody.a = 0; // Scalar part zero to ensure pure vector quaternion
    GPSVelocityWorld = ori.orientation.rotate(GPSVelocityBody); // Rotate GPS velocity vector to world frame

    adjustedGPSVelocity[0] = (-gps.getGPSInfo().pos.velocityDown) - GPSVelocityWorld.b; // Adjust GPS velocity for lever arm and orientation
    adjustedGPSVelocity[1] = gps.getGPSInfo().pos.velocityEast - GPSVelocityWorld.c; // Adjust GPS velocity for lever arm and orientation
    adjustedGPSVelocity[2] = gps.getGPSInfo().pos.velocityNorth - GPSVelocityWorld.d; // Adjust GPS velocity for lever arm and orientation

    if(gps.getGPSInfo().fixType > 3){ //Keep estimating even if we lost RTK fix - just use rtk float
        XPos.updateGPS(adjustedGPSPosition[0], adjustedGPSVelocity[0]); // Update X position and velocity from GPS data
        YPos.updateGPS(adjustedGPSPosition[1], adjustedGPSVelocity[1]); // Update Y position and velocity from GPS data
        ZPos.updateGPS(adjustedGPSPosition[2], adjustedGPSVelocity[2]); // Update Z position and velocity from GPS data
    }

    // Update world frame position and velocity from GPS data
    worldPosition[0] = XPos.getPosition(); // Update world frame X position
    worldPosition[1] = YPos.getPosition(); // Update world frame Y position
    worldPosition[2] = ZPos.getPosition(); // Update world frame Z position

    worldVelocity[0] = XPos.getVelocity(); // Update world frame X velocity
    worldVelocity[1] = YPos.getVelocity(); // Update world frame Y velocity
    worldVelocity[2] = ZPos.getVelocity(); // Update world frame Z velocity

    // Update world frame acceleration from GPS data
    worldAccel[0] = XPos.getAcceleration(); // Update world frame X acceleration
    worldAccel[1] = YPos.getAcceleration(); // Update world frame Y acceleration
    worldAccel[2] = ZPos.getAcceleration(); // Update world frame Z acceleration

    // Update accel uncertainties
    accelUncertainty[0] = XPos.getAccelerationUncertainty(); // Update X acceleration uncertainty
    accelUncertainty[1] = YPos.getAccelerationUncertainty(); // Update Y acceleration uncertainty
    accelUncertainty[2] = ZPos.getAccelerationUncertainty(); // Update Z acceleration uncertainty

    // Update position uncertainties
    positionUncertainty[0] = XPos.getPositionUncertainty(); // Update X position uncertainty
    positionUncertainty[1] = YPos.getPositionUncertainty(); // Update Y position uncertainty
    positionUncertainty[2] = ZPos.getPositionUncertainty(); // Update Z position uncertainty

    // Update velocity uncertainties
    velocityUncertainty[0] = XPos.getVelocityUncertainty(); // Update X velocity uncertainty
    velocityUncertainty[1] = YPos.getVelocityUncertainty(); // Update Y velocity uncertainty
    velocityUncertainty[2] = ZPos.getVelocityUncertainty(); // Update Z velocity uncertainty
}


/**
 * @brief Computes the current trajectory setpoint according to the target and current time
 */
float StateEstimation::calculateTrajectory(float target, float time){
    if (time <= 0) {
        return 0;
    }

    if(time >= T_END){
        return target; 
    }

    float v_max = target / (0.5 * T_ACCEL + T_COAST + 0.5 * T_DECEL); // Maximum velocity to reach target in time

    if(time <= T_ACCEL){
        //accel phase
        float phaseProgress = time / T_ACCEL; // Progress in acceleration phase
        float velocityIntegral = v_max * 0.5 * (phaseProgress - sin(PI * phaseProgress) / PI); // Integral of velocity over acceleration phase
        return velocityIntegral * T_ACCEL;
    }else if (time <= T_ACCEL + T_COAST){
        float dAccel = v_max * T_ACCEL * 0.5;
        float dCoast = v_max * (time - T_ACCEL); // Distance traveled during coast phase
        return dAccel + dCoast; // Total distance is sum of acceleration and coast distances
    }else{
        //decel phase
        float dAccel = v_max * T_ACCEL * 0.5;
        float dCoast = v_max * T_COAST; // Distance traveled during coast phase

        float tDecelCurrent = time - (T_ACCEL + T_COAST); // Time in deceleration phase
        float phaseProgress = tDecelCurrent / T_DECEL; // Progress in deceleration phase

        float velocityIntegral = v_max * 0.5 * (phaseProgress + sin(PI * phaseProgress) / PI); // Integral of velocity over deceleration phases
        float dDecel = velocityIntegral * T_DECEL; // Distance traveled during deceleration phase
        return dAccel + dCoast + dDecel; // Total distance is sum of acceleration, coast, and deceleration distances
    }
}

/**
 * @brief Computes the velocity setpoint (derivative of trajectory)
 */
float StateEstimation::calculateTrajectoryVelocity(float target, float time) {
    if (time <= 0 || time >= T_END) {
        return 0; // No movement before start or after end
    }

    float v_max = target / (0.5 * T_ACCEL + T_COAST + 0.5 * T_DECEL);

    if (time <= T_ACCEL) {
        // Acceleration phase - sinusoidal velocity profile
        float phaseProgress = time / T_ACCEL;
        return v_max * 0.5 * (1 - cos(PI * phaseProgress));
        
    } else if (time <= T_ACCEL + T_COAST) {
        // Coast phase - constant velocity
        return v_max;
        
    } else {
        // Deceleration phase - sinusoidal velocity profile (decreasing)
        float tDecelCurrent = time - (T_ACCEL + T_COAST);
        float phaseProgress = tDecelCurrent / T_DECEL;
        return v_max * 0.5 * (1 + cos(PI * phaseProgress));
    }
}

/**
 * @brief PID control loop for attitude and position control.
 */
void StateEstimation::PIDLoop(){

    positionSetpoint[0] = calculateTrajectory(Y_TARGET, timeSinceLaunch); // Calculate Y position setpoint based on trajectory
    positionSetpoint[1] = calculateTrajectory(Z_TARGET, timeSinceLaunch); // Calculate Z position setpoint based on trajectory

    float velocitySetpoint[2];
    velocitySetpoint[0] = calculateTrajectoryVelocity(Y_TARGET, timeSinceLaunch);
    velocitySetpoint[1] = calculateTrajectoryVelocity(Z_TARGET, timeSinceLaunch);

    if(vehicleState == 66 || vehicleState == 65){
        // In stabilization and position hold test state, set position setpoints to zero
        positionSetpoint[0] = 0.0f; // Y position setpoint
        positionSetpoint[1] = 0.0f; // Z position setpoint

        velocitySetpoint[0] = 0.0f; // Y velocity setpoint
        velocitySetpoint[1] = 0.0f; // Z velocity setpoint
    }

    YPID.compute(positionSetpoint[0], worldPosition[1], velocitySetpoint[0] - worldVelocity[1], true);
    ZPID.compute(positionSetpoint[1], worldPosition[2], velocitySetpoint[1] - worldVelocity[2], true);

    float alpha = 0.1; // Low pass filter constant
    

    attitudeSetpoint[0] = attitudeSetpoint[0] * (1 - alpha) + (min(max(YPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD) * alpha); // Yaw
    attitudeSetpoint[1] = attitudeSetpoint[1] * (1 - alpha) + (-min(max(ZPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD) * alpha); // Pitch

    if(vehicleState == 4 || vehicleState == 65){ // only in return to vertical state or test state
        attitudeSetpoint[0] = 0.0f; // Yaw setpoint to zero after 2.5 seconds post launch
        attitudeSetpoint[1] = 0.0f; // Pitch setpoint to zero after 2.5 seconds post launch
    }

    YawPID.compute(attitudeSetpoint[0], getEulerAngle()[0]);
    PitchPID.compute(attitudeSetpoint[1], getEulerAngle()[1]);

}


/** 
 * @brief Actuates servos based on PID outputs and gimbal misalignment.
 * @param actuate If true, actuate servos; if false, center servos.
 * @param includePID If true, include PID outputs in actuation; if false, do not include PID outputs and only has gimbal misalignment.
 */
void StateEstimation::actuateServos(bool actuate, bool includePID){
    if(lastActuatorMicros == 0) { // If this is the first update, set lastActuatorMicros to current time
        lastActuatorMicros = micros();
        return;
    }
    if (micros() - lastActuatorMicros < ACTUATOR_INTERVAL_US) { // If not enough time has passed since last update, return
        return;
    }
    if(!actuate) { // If actuate is false, do not actuate servos
        YawServo.write(90);
        PitchServo.write(90); // Center servos
        return;
    }

    float dt = (float)(micros() - lastActuatorMicros) / 1000000.0f; // Convert to seconds
    lastActuatorMicros = micros(); // Update last actuator time


    angularAccelCommand[0] = YawPID.getOutput(); // Yaw angular acceleration command
    angularAccelCommand[1] = PitchPID.getOutput(); // Pitch angular acceleration command

    // roll mixer
    angularAccelCommandVector = Quaternion(0, 0, angularAccelCommand[1], angularAccelCommand[0]); // pitch yaw

    float roll_feedforward = gyroLowPassed[2] * ROLL_FEEDFORWARD_T;

    rollRateCompensatedBodyToWorld = ori.orientation * Quaternion().from_axis_angle(roll_feedforward, 1, 0, 0);

    bodyAngularAccelCommandVector = rollRateCompensatedBodyToWorld.conj().rotate(angularAccelCommandVector);

    float newGimbalAngle[2];

    newGimbalAngle[1] =  bodyAngularAccelCommandVector.c; // Pitch gimbal angle command
    newGimbalAngle[0] = bodyAngularAccelCommandVector.d; // Yaw gimbal angle command

    // convert from angular acceleration to gimbal angle
    float modifier = getPitchYawMMOI() / (getThrust() * getMomentArm()); // Modifier to convert angular acceleration to gimbal angle

    newGimbalAngle[0] = asin(max(min(newGimbalAngle[0] * modifier, 1), -1));
    newGimbalAngle[1] = asin(max(min(newGimbalAngle[1] * modifier, 1), -1));

    if(includePID == false){ // If includePID is false, do not include PID outputs and only has gimbal misalignment
        newGimbalAngle[0] = 0;
        newGimbalAngle[1] = 0;
    }
    newGimbalAngle[0] += gimbalMisalign[0]; // Add gimbal misalignment to yaw gimbal angle command
    newGimbalAngle[1] += gimbalMisalign[1]; // Add gimbal misalignment to pitch gimbal angle command

    float gimbalRate[2];
    gimbalRate[0] = (newGimbalAngle[0] - gimbalAngle[0]) / dt; // Yaw gimbal rate
    gimbalRate[1] = (newGimbalAngle[1] - gimbalAngle[1]) / dt; // Pitch gimbal rate
    
    gimbalRate[0] = min(max(gimbalRate[0], -MAX_GIMBAL_RATE_RAD_S), MAX_GIMBAL_RATE_RAD_S); // Limit gimbal rate to +/- MAX_GIMBAL_RATE_RAD_S
    gimbalRate[1] = min(max(gimbalRate[1], -MAX_GIMBAL_RATE_RAD_S), MAX_GIMBAL_RATE_RAD_S); // Limit gimbal rate to +/- MAX_GIMBAL_RATE_RAD_S

    gimbalAngle[0] += gimbalRate[0] * dt; // Update yaw gimbal angle
    gimbalAngle[1] += gimbalRate[1] * dt; // Update pitch gimbal angle

    gimbalAngle[0] = min(max(gimbalAngle[0], -GIMBAL_LIMIT_RAD), GIMBAL_LIMIT_RAD); // Limit gimbal angle to +/- GIMBAL_LIMIT_RAD
    gimbalAngle[1] = min(max(gimbalAngle[1], -GIMBAL_LIMIT_RAD), GIMBAL_LIMIT_RAD); // Limit gimbal angle to +/- GIMBAL_LIMIT_RAD
    // signs CORRECT up to here - note that positive pitch gimbal angle has bottom of gimbal towards +Z axis
    // positive yaw gimbal angle has bottom of gimbal towards -Y axis. 
    // Matches up with right hand rule


    //some code for actuation here - TODO may need sign flips
    // TODO Check signs
    float servoAngle[2];
    servoAngle[0] = 586.58888 * pow(gimbalAngle[0], 3) + 217.8076 * gimbalAngle[0]; // in degrees Yaw
    servoAngle[1] = 586.58888 * pow(gimbalAngle[1], 3) + 217.8076 * gimbalAngle[1]; // in degrees Pitch

    YawServo.write(90 - servoAngle[0]); // Set yaw servo angle, 90 degrees is center position
    PitchServo.write(90 + servoAngle[1]); // Set pitch servo angle, 90 degrees is center position

}

/**
 * @brief Fires pyro charges when the vehicle is ready for landing.
 */
void StateEstimation::firePyroWhenReady(){
    // Fire pyros when ready
    if(landingIgnitionAltitude == 0.0){
        digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
        return; // If landing ignition altitude is not set, do not fire pyro
    }

    if(worldPosition[0] > landingIgnitionAltitude){
        digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
        return; // If world position is above landing ignition altitude, do not fire pyro
    }

    //Only start firing if vehicle is in landing state and has passed landing ignition altitude

    if (vehicleState == 5 || vehicleState == 6) { // If vehicle is past apogee or in landing burn state
        if(landingIgnitionTime == 0.0f){ //Has not committed to ignition yet, can still abort
            if(abs(ori.toEuler().yaw) > PI / 4 || abs(ori.toEuler().pitch) > PI / 4){ // ABORT if too tilted or pointing backward
                abort();
                return;
            }
            if(!getPyroCont()){ // ABORT if pyro is not ready
                abort();
                return;
            }
        }
        
        if(worldPosition[0] > PYRO_LOCKOUT_ALT){
            digitalWrite(LAND_PYRO, HIGH); // Fire pyro
            if(landingIgnitionTime == 0.0f){
                landingIgnitionTime = timeSinceLaunch; // Record landing ignition time
            }
        }else{
            digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
        }
        
        if(vehicleState == 5){ // If vehicle is past apogee
            vehicleState = 6; // Change state to landing burn
        }
    }else{
        digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
    }
}


bool StateEstimation::getPyroCont(){
    // Returns true if pyro continuity is detected, false otherwise
    return digitalRead(LAND_CONTINUITY) == HIGH; // Check if land continuity pin is HIGH
}

bool StateEstimation::getChuteCont(){
    // Returns true if chute continuity is detected, false otherwise
    return digitalRead(CHUTE_CONTINUITY) == HIGH; // Check if chute continuity pin is HIGH
}


/**
 * @brief Reads IMU0 sensor data and converts it to usable format.
 */
void StateEstimation::readIMU0(){
    IMU0.SCH1_getData(&rawIMU0Data);
    IMU0.SCH1_convert_data(&rawIMU0Data, &resultIMU0Data);

    gyroRemovedBias[0] = radians(resultIMU0Data.Rate1[2]) - gyroBias[0]; // Gyro X in rad/s with bias removed
    gyroRemovedBias[1] = -(radians(resultIMU0Data.Rate1[0]) - gyroBias[1]); // Gyro Y in rad/s with bias removed
    gyroRemovedBias[2] = (radians(resultIMU0Data.Rate1[1]) - gyroBias[2]); // Gyro Z in rad/s with bias removed

    //single axis calibrated, maybe will implement 3x3 matrix later
    accelCalibrated[0] = (resultIMU0Data.Acc1[1] - -0.021470) * 1.001336; // X acceleration in body frame
    accelCalibrated[1] = (resultIMU0Data.Acc1[0] - -0.008415) * 1.001126; // Y acceleration in body frame
    accelCalibrated[2] = (resultIMU0Data.Acc1[2] - 0.047871) * 1.000885; // Z acceleration in body frame 

}

//Axes: X axis is UP (Gravity is -X), Y axis is RIGHT, Z axis is BACK (Right hand rule). Matches Falcon9 standard
//Rotation Axes: X is ROLL, Y is PITCH, Z is YAW
/**
 * @brief Updates orientation using gyro data.
 */
void StateEstimation::oriLoop(){
    if (lastOriUpdate == 0) { // If this is the first update, set lastOriUpdate to current time
        lastOriUpdate = micros();
        return;
    }

    oriLoopMicros = micros();
    float dtOri = (float)(oriLoopMicros - lastOriUpdate) / 1000000.0f; // Convert to seconds
    lastOriUpdate = oriLoopMicros;
    ori.update(gyroRemovedBias[0], gyroRemovedBias[1], gyroRemovedBias[2], dtOri); 
    
    float filterAlpha = 0.1;
    //Simple low pass, used for lever arm correction
    gyroLowPassed[0] = filterAlpha * gyroRemovedBias[0] + (1 - filterAlpha) * gyroLowPassed[0];
    gyroLowPassed[1] = filterAlpha * gyroRemovedBias[1] + (1 - filterAlpha) * gyroLowPassed[1];
    gyroLowPassed[2] = filterAlpha * gyroRemovedBias[2] + (1 - filterAlpha) * gyroLowPassed[2];
}

/**
 * @brief Updates acceleration, velocity, and position using IMU data.
 */
void StateEstimation::accelLoop(){
    if (lastAccelUpdate == 0) { // If this is the first update, set lastAccelUpdate to current time
        lastAccelUpdate = micros();
        return;
    }
    
    ori.updateAccel(accelCalibrated[0], -accelCalibrated[1], accelCalibrated[2]); // Update the orientation with the measured acceleration in body frame

    measuredWorldAccel[0] = ori.worldAccel.b + WORLD_GRAVITY_X; // X acceleration in world frame
    measuredWorldAccel[1] = ori.worldAccel.c + WORLD_GRAVITY_Y; // Y acceleration in world frame
    measuredWorldAccel[2] = ori.worldAccel.d + WORLD_GRAVITY_Z; // Z acceleration in world frame



    accelLoopMicros = micros();
    //float dtAccel = (float)(accelLoopMicros - lastAccelUpdate) / 1000000.0f; // Convert to seconds
    lastAccelUpdate = accelLoopMicros;

    thrust = accelCalibrated[0] * getMass(); // Calculate thrust in Newtons based on acceleration in body frame and mass of the vehicle

    if(gps.getGPSInfo().fixType > 3){ //Ensure RTK Fix or RTK float
        XPos.updateAccelerometer(measuredWorldAccel[0]);
        YPos.updateAccelerometer(measuredWorldAccel[1]);
        ZPos.updateAccelerometer(measuredWorldAccel[2]);
    }

    worldPosition[0] = XPos.getPosition(); // Get X position in world frame
    worldPosition[1] = YPos.getPosition(); // Get Y position in world frame
    worldPosition[2] = ZPos.getPosition(); // Get Z position in world frame

    worldVelocity[0] = XPos.getVelocity(); // Get X velocity in world frame
    worldVelocity[1] = YPos.getVelocity(); // Get Y velocity in world frame
    worldVelocity[2] = ZPos.getVelocity(); // Get Z velocity in world frame

    worldAccel[0] = XPos.getAcceleration(); // Get X acceleration in world frame
    worldAccel[1] = YPos.getAcceleration(); // Get Y acceleration in world frame
    worldAccel[2] = ZPos.getAcceleration(); // Get Z acceleration in world frame
    
    // Update accel uncertainties
    accelUncertainty[0] = XPos.getAccelerationUncertainty(); // Update X acceleration uncertainty
    accelUncertainty[1] = YPos.getAccelerationUncertainty(); // Update Y acceleration uncertainty
    accelUncertainty[2] = ZPos.getAccelerationUncertainty(); // Update Z acceleration uncertainty

    // Update position uncertainties
    positionUncertainty[0] = XPos.getPositionUncertainty(); // Update X position uncertainty
    positionUncertainty[1] = YPos.getPositionUncertainty(); // Update Y position uncertainty
    positionUncertainty[2] = ZPos.getPositionUncertainty(); // Update Z position uncertainty

    // Update velocity uncertainties
    velocityUncertainty[0] = XPos.getVelocityUncertainty(); // Update X velocity uncertainty
    velocityUncertainty[1] = YPos.getVelocityUncertainty(); // Update Y velocity uncertainty
    velocityUncertainty[2] = ZPos.getVelocityUncertainty(); // Update Z velocity uncertainty
}

/**
 * @brief Updates gyro biases and orientation before launch.
 */
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
    accelReading[0] = 0.0f; // Reset X acceleration in body frame
    accelReading[1] = 0.0f; // Reset Y acceleration in body frame
    accelReading[2] = 0.0f; // Reset Z acceleration in body frame

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
        gyroBias[0] += radians(resultIMU0Data.Rate1[2]); // Accumulate gyro bias for X axis in rad/s
        gyroBias[1] += radians(resultIMU0Data.Rate1[0]); // Accumulate gyro bias for Y axis in rad/s
        gyroBias[2] += radians(resultIMU0Data.Rate1[1]); // Accumulate gyro bias for Z axis in rad/s

        accelReading[0] += accelCalibrated[0]; // X acceleration in body frame
        accelReading[1] += accelCalibrated[1]; // Y acceleration in body frame
        accelReading[2] += -accelCalibrated[2]; // Z acceleration in body frame

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
    actualAccel = Quaternion(accelReading[0], accelReading[1], accelReading[2]); // Create quaternion from acceleration readings in body frame
    actualAccel = actualAccel.normalize(); // Normalize the quaternion to get unit vector

    ori.orientation = expectedGravity.rotation_between_vectors(actualAccel); // Compute the orientation quaternion by rotating expected gravity vector to actual acceleration vector
    //ori.zeroRoll();
    ori.orientation = ori.orientation.normalize(); // Normalize the orientation quaternion

    gps.setCurrentAsHome(); // Set current GPS position as home position
}


/**
 * @brief Sets the vehicle state.
 * Error codes:
 * 1 Invalid state
 * 2 Invalid transition
 * 3 Bad sensors
 * 4 Pyro continuity not detected
 */
uint8_t StateEstimation::setVehicleState(int state){
    // 0: Disarmed, 1: Armed, 2: Launching, 3: In Flight, 4: Landing, 5: Landed
    maxMillisPerLoop = 0; // Reset max loop time for performance monitoring
    if (state >= -1 && state <= 7) {
        switch (state){
            case 1: // armed process
                if (vehicleState == 0){
                    if(sensorStatus == 0 && gps.getGPSInfo().fixType == 4){ // allow arm with RTKfix, and all sensors functioning
                        // if(getPyroCont() == false){ // Check if pyro continuity is not detected
                        //     return 4; // Cannot arm if pyro continuity is not detected
                        // }
                        vehicleState = 1;
                        XPos.begin(); // Initialize X Kalman
                        YPos.begin(); // Initialize Y Kalman
                        ZPos.begin(); // Initialize Z Kalman
                    } else {
                        return 3; // Cannot arm if sensors are not all functioning
                    }
                }else{
                    return 2; // Invalid transition
                }
                break;
            case 0: //disarm process
                if (vehicleState > 1 && vehicleState < 7){ // Somewhat unsafe but should not disarm in flight
                    return 2; // Invalid transition
                }
                resetVariables(); // Reset all variables to initial conditions
                gps.resetHome();
                XPos.reset();
                YPos.reset();
                ZPos.reset();
                YPID.reset();
                ZPID.reset();
                YawPID.reset();
                PitchPID.reset();
                //RollMotor.stop(); // Stop roll motor
                vehicleState = 0; // Set vehicle state to disarmed
                break;
            default:
                vehicleState = state;
                break;
        }
    }else if(state == 69){
        if(vehicleState != 0){
            return 2; // Invalid transition, cannot set to testing state if not disarmed
        }
        vehicleState = 69; // Set vehicle state to testing state
        XPos.begin(); // Initialize X Kalman
        YPos.begin(); // Initialize Y Kalman
        ZPos.begin(); // Initialize Z Kalman
    } else if (state >= 64){
        if (vehicleState != 69){
            return 2; // Invalid transition, cannot set to actual testing state if not in test prep
        }
        launchTime = millis() / 1000.0; // Set launch time to current time in seconds, to progress trajectory 
        if(state == 68 || state == 64){
            lastGimbalMisalignMicros = micros();
        }  
        vehicleState = state; // Set vehicle state to testing states
    } else {
        return 1; // Invalid state
    }
    return 0;
}



// Returns the Euler angles in radians
// Yaw pitch roll
const float* StateEstimation::getEulerAngle(){
    static float euler[3];
    EulerAngles a = ori.toEuler(); // Get Euler angles from orientation quaternion
    euler[0] = a.yaw;   // yaw (X axis)
    euler[1] = a.pitch;  // Pitch (Y axis)
    euler[2] = a.roll;    // roll (Z axis)
    return euler; // Return the Euler angles
}

// Detects Launch via absolute value of acceleration
void StateEstimation::detectLaunch(){
    if (vehicleState == 1){
        if (sqrt(pow(resultIMU0Data.Acc1[1], 2) + pow(resultIMU0Data.Acc1[0], 2) + pow(resultIMU0Data.Acc1[2], 2)) > LAUNCH_ACCEL_THRESHOLD){
            vehicleState = 2;       
            launchTime = millis() / 1000.0;
            lastGimbalMisalignMicros = micros();
        }
    }
}

// Uses accel integration only to detect apogee altitude and calculates correct ignition altitude
void StateEstimation::detectApogee(){
    if (worldPosition[0] > apogeeAltitude){
        apogeeAltitude = worldPosition[0];
    }   
    if (worldPosition[0] < apogeeAltitude - BELOW_APOGEE_THRESHOLD){
        // TODO: Smarter ignition altitude adjustment
        if (vehicleState == 4 || vehicleState == 3){
            if(abs(ori.toEuler().yaw) > PI / 4 || abs(ori.toEuler().pitch) > PI / 4){ // ABORT if too tilted or pointing backward
                abort();
                return;
            }
            if(!getPyroCont()){ // ABORT if pyro is not ready
                abort();
                return;
            }

            landingIgnitionAltitude = 0.81 * apogeeAltitude;
            vehicleState = 5;
            
        }
    }
}

void StateEstimation::abort(){
    if(vehicleState > 2 && vehicleState < 7 && accelCalibrated[0] < 11.0){ //Not under thrust, and in flight
        digitalWrite(CHUTE_PYRO, HIGH);
        delay(1000);
        vehicleState = 7;
    }
}

// gets gimbal misalign, assumes that it starts at launch time = 0
void StateEstimation::computeGimbalMisalign(){
    if(timeSinceLaunch < 0.05f){
        return; // Do not compute gimbal misalign in first 50 ms after launch - disturbed by launch dynamics
    }
    if(lastGimbalMisalignMicros == 0) { // If this is the first update, set lastGimbalMisalignMicros to current time
        lastGimbalMisalignMicros = micros();
        return;
    }

    if ((vehicleState == 2  || vehicleState == 68) && timeSinceLaunch > 0 && timeSinceLaunch < MISALIGN_CHARACTERIZATION_TIME)  { // If in flight state and time since launch is within gimbal stabilization time
        float dt = (float)(micros() - lastGimbalMisalignMicros) / 1000000.0f; // Convert to seconds
        lastGimbalMisalignMicros = micros(); // Update last gimbal misalign time
        gimbalForceAccumulator += getThrust() * dt;
        gimbalMisalignAccumulator[0] += gyroRemovedBias[0] * dt; // Accumulate gimbal misalign for yaw
        gimbalMisalignAccumulator[1] += gyroRemovedBias[1] * dt;
        gimbalMisalignTime += dt;
        float f = gimbalForceAccumulator / gimbalMisalignTime;

        if(f < 0.01f) { // If force is too low, do not calculate misalign
            return;
        }

        MMOIAccumulator += getPitchYawMMOI() * dt;
        momentArmAccumulator += getMomentArm() * dt; //averages MMOI and moment arm over time

        float i = MMOIAccumulator / gimbalMisalignTime;
        float r = momentArmAccumulator / gimbalMisalignTime;

        float y = (2 * gimbalMisalignAccumulator[0] * i)/(r * f * gimbalMisalignTime * gimbalMisalignTime);
        float p = (2 * gimbalMisalignAccumulator[1] * i)/(r * f * gimbalMisalignTime * gimbalMisalignTime);

        if (abs(y) > 1 || abs(p) > 1) { // If gimbal misalign is outside of asin domain, do not calculate
            return;
        }

        gimbalMisalign[0] = -asin(y);
        gimbalMisalign[1] = -asin(p);

        gimbalMisalign[0] = min(max(gimbalMisalign[0], -GIMBAL_LIMIT_RAD/8), GIMBAL_LIMIT_RAD/8); // Limit gimbal misalignment to +/- GIMBAL_LIMIT_RAD/8
        gimbalMisalign[1] = min(max(gimbalMisalign[1], -GIMBAL_LIMIT_RAD/8), GIMBAL_LIMIT_RAD/8); // Limit gimbal misalignment to +/- GIMBAL_LIMIT_RAD/8

    }
    
}

void StateEstimation::adaptiveGimbalMisalignEstimation(){
    //return; // Disabled for now
    if(timeSinceLaunch <= MISALIGN_CHARACTERIZATION_TIME){
        return;
    }

    float dt = (float)(micros() - lastGimbalMisalignMicros) / 1000000.0f;
    if (dt < 0.005f){
        return;
    }
    lastGimbalMisalignMicros = micros();

    float currentBodyAngAccelCommand[2] = {
        bodyAngularAccelCommandVector.d, //yaw
        bodyAngularAccelCommandVector.c  //pitch
    };

    bool isSteady = isServoCommandSteady(currentBodyAngAccelCommand, dt);

    // get angular accel
    float actualAngularAccel[2] = {
        (gyroRemovedBias[0] - lastGyroRemovedBias[0]) / dt,
        (gyroRemovedBias[1] - lastGyroRemovedBias[1]) / dt
    };
    
    lastGyroRemovedBias[0] = gyroRemovedBias[0];
    lastGyroRemovedBias[1] = gyroRemovedBias[1];

    //low pass
    float filterAlpha = 0.3f; // Adjust this value as needed
    filteredActualAngularAccel[0] = filterAlpha * filteredActualAngularAccel[0] + (1.0f - filterAlpha) * actualAngularAccel[0];
    filteredActualAngularAccel[1] = filterAlpha * filteredActualAngularAccel[1] + (1.0f - filterAlpha) * actualAngularAccel[1];
    //filteredActualAngularAccel[2] = 0; // Roll, not used

    if(dt > 0.1f){
        return; 
    }

    float angAccelError[2] = {
        filteredActualAngularAccel[0] - currentBodyAngAccelCommand[0],
        filteredActualAngularAccel[1] - currentBodyAngAccelCommand[1]
    };

    if(getThrust() > 10.0f && isSteady){
        float modifier = getPitchYawMMOI() / (getThrust() * getMomentArm());
        float requiredGimbalCorrection[2] = {
            -asin(constrain(angAccelError[0] * modifier, -1.0f, 1.0f)),
            -asin(constrain(angAccelError[1] * modifier, -1.0f, 1.0f))
        };

        float adaptiveGain = 0.001f;

        gimbalMisalign[0] += requiredGimbalCorrection[0] * adaptiveGain;
        gimbalMisalign[1] += requiredGimbalCorrection[1] * adaptiveGain;

        gimbalMisalign[0] = constrain(gimbalMisalign[0], -GIMBAL_LIMIT_RAD/8, GIMBAL_LIMIT_RAD/8);
        gimbalMisalign[1] = constrain(gimbalMisalign[1], -GIMBAL_LIMIT_RAD/8, GIMBAL_LIMIT_RAD/8);

    }
}

bool StateEstimation::isServoCommandSteady(float currentCommand[2], float dt){
    float commandRate[2] = {
        abs(currentCommand[0] - lastBodyAngAccelCommand[0]) / dt,
        abs(currentCommand[1] - lastBodyAngAccelCommand[1]) / dt
    };

    lastBodyAngAccelCommand[0] = currentCommand[0];
    lastBodyAngAccelCommand[1] = currentCommand[1];

    float maxCommandRate = 3.0f; // rad/s³ - tune this threshold
    return (commandRate[0] < maxCommandRate && commandRate[1] < maxCommandRate);
}