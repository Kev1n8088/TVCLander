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
#include "GPSHandler.h"


/**
 * @brief Constructor for StateEstimation. Initializes sensors and SPI.
 */
StateEstimation::StateEstimation()
    : IMU0(IMU0_RST_PIN), // Initialize SCH1 with the reset pin
      IMU1(SPI, IMU1_CS_PIN), // Initialize ICM456xx with SPI and CS pin
      bmp(),
      lis2mdl(12345), 
      PitchPID(8,0,-4,10000,0.5),
        YawPID(8,0,-4,10000,0.5),
        PitchStabilizationPID(8,0,-6,10000,0.5),
        YawStabilizationPID(8,0,-6,10000,0.5),
        RollPID(0.02,0,-0.005,10000,0.5),
        YAscentPID(0.02,0,-0.01,10000,0.5),
        ZAscentPID(0.02,0,-0.01,10000,0.5),
        YDescentPID(0.03,0,-0.08,10000,0.5),
        ZDescentPID(0.03,0,-0.08,10000,0.5),
        gps(),
        XPos(),
        YPos(),
        ZPos(),
        RollMotor()
{
    SPI.begin();
    resetVariables();
    PitchServo.attach(PITCH_SERVO); // Attach the pitch servo to the specified pin
    YawServo.attach(YAW_SERVO); // Attach the yaw servo to the specified pin
}

/**
 * @brief Initializes all sensors and returns a bitmask of failures.
 * @return Bitmask indicating which sensors failed to initialize.
 */
int StateEstimation::begin(){
    pinMode(IMU0_DRY_PIN, INPUT); // Set the dry pin for SCH1 as input
    pinMode(LAND_CONTINUITY, INPUT); // Set the land continuity pin as input
    pinMode(LAND_PYRO, OUTPUT); // Set the land pyro pin as output

    int failMask = 0;
    if (beginBaro() != 0)  failMask |= 0x01;
    if (beginIMU0() != 0)  failMask |= 0x02;
    if (beginIMU1() != 0)  failMask |= 0x04;
    if (beginMag()  != 0)  failMask |= 0x08;
    if (gps.begin() != 0)  failMask |= 0x10;
    sensorStatus = failMask; // Store the sensor status in the class variable
    return failMask;
}

/**
 * @brief Resets all state estimation variables to initial conditions.
 */
void StateEstimation::resetVariables(){
    
    // init variables
    resetLinearVariables();

    timeSinceLaunch = 0.0f; // Time since launch in seconds
    launchTime = 0.0f; // Launch time in seconds

    gimbalMisalign[0] = 0;
    gimbalMisalign[1] = 0;

    gimbalMisalignAccumulator[0] = 0;
    gimbalMisalignAccumulator[1] = 0;

    gimbalMisalignTime = 0;

    gimbalForceAccumulator = 0;

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

    // Reset world frame acceleration, velocity, and position
    for (int i = 0; i < 3; i++) {
        measuredWorldAccel[i] = 0.0f; // Reset measured acceleration in world frame
        worldAccel[i] = 0.0f; // Reset acceleration in world frame
        worldVelocity[i] = 0.0f; // Reset velocity in world frame
        worldPosition[i] = 0.0f; // Reset position in world frame
    }
}

/**
 * @brief Returns the current expected mass of the vehicle in kg.
 */
float StateEstimation::getMass(){
    //Returns current expected mass of the vehicle in kg

    // TODO: adjust nums
    if (vehicleState < 5){
        return 1.0f;
    }else{
        return 0.9f;
    }
}

/**
 * @brief Returns the moment arm of the vehicle in meters.
 */
float StateEstimation::getMomentArm(){
    // Returns the moment arm of the vehicle in meters

    // TODO: adjust nums
    if (vehicleState < 5){
        return 0.5f;
    }else{
        return 0.7f;
    }
}

/**
 * @brief Returns the pitch and yaw moment of inertia in kg*m^2.
 */
float StateEstimation::getPitchYawMMOI(){
    // Returns the pitch and yaw moment of inertia in kg*m^2

    // TODO: adjust nums
    if (vehicleState < 5){
        return 0.023f;
    }else{
        return 0.020f;
    }
}

/**
 * @brief Returns the current thrust value. Minimum is 12.5 N, maximum is 25.0 N.
 */
float StateEstimation::getThrust(){

    return min(max(12.5, thrust), 25.0);
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

    lastStateEstimateMicros = micros(); // Update last state estimate time

    if (launchTime != 0.0f){
        timeSinceLaunch = millis() / 1000.0f - launchTime; // Calculate time since launch in seconds
    }

    gps.gpsLoop(); // Update GPS data

    // State machine
    switch (vehicleState){
        case 0: // Disarmed State
            if (digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                updatePrelaunch(); // Update gyro biases and orientation before launch
                RollMotor.stop(); // Stop roll motor
            }
            break;
        case 1: // Armed State. < 30 seconds before launch, do not update gyro bias.
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos(false); // center servos without actuating them
                actuateWheel(); 
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
                //detectLaunch();
            }
            break;
        case 2: // No gimbal command state, characterizing misalign
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                getGimbalMisalign();
                actuateServos(false); // center servos without actuating them
                actuateWheel(); 
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            // TODO: Lock servos in center position
            if (timeSinceLaunch > MISALIGN_CHARACTERIZATION_TIME){
                vehicleState = 3;
            }
            break;
        case 3: //Standard guidance state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints 
                actuateServos();
                actuateWheel(); 
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            // TODO: insert code for updating servos
            if (timeSinceLaunch > GIMBAL_STABILIZATION_TIME){
                vehicleState = 4;
            }
            break;
        case 4: // Return to vertical state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos();
                actuateWheel(); 
                detectApogee();
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        case 5: // Past apogee state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos();
                actuateWheel(); 
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
                actuateServos();
                actuateWheel(); 
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
        // below are test states - they do not contain transition logic and are used for testing purposes only
        case 64: // Wheel testing state
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos(false); // center servos without actuating them
                actuateWheel(); 
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        // Servo test cases - no wheel
        case 65: // Stabilization test state (no use of positional PID)
        case 66:// Positional PID test state (uses descent positional PID)
        case 67:// Predictive Positional PID test state (uses ascent positional PID)
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                PIDLoop(); // Call PID loop to compute attitude setpoints
                actuateServos();
                RollMotor.stop(); // Stop roll motor
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
            break;
        case 68: //misalignment test - operation: rotate the vehicle and the gimbal should move in the opposite direction 
            if(digitalRead(IMU0_DRY_PIN) == HIGH) { // Check if DRY pin is HIGH, default behavior DRY pin is active HIGH
                readIMU0(); // Read IMU data only if DRY
                oriLoop(); // Call orientation loop to update orientation
                accelLoop(); // Call acceleration loop to update world frame acceleration, velocity, and position
                GPSLoop();
                getGimbalMisalign();
                actuateServos(true, false); // Actuate but do not include PID outputs
                RollMotor.stop(); // Stop roll motor
                digitalWrite(LAND_PYRO, LOW); // Ensure land pyro is not fired
            }
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

    XPos.updateGPS(gps.getGPSInfo().XYZ.x, -gps.getGPSInfo().pos.velocityDown); // Update X position and velocity from GPS data
    YPos.updateGPS(gps.getGPSInfo().XYZ.y, gps.getGPSInfo().pos.velocityEast); // Update Y position and velocity from GPS data
    ZPos.updateGPS(gps.getGPSInfo().XYZ.z, gps.getGPSInfo().pos.velocityNorth); // Update Z position and velocity from GPS data

    // Update world frame position and velocity from GPS data
    worldPosition[0] = XPos.getPosition(); // Update world frame X position
    worldPosition[1] = YPos.getPosition(); // Update world frame Y position
    worldPosition[2] = ZPos.getPosition(); // Update world frame Z position

    worldVelocity[0] = XPos.getVelocity(); // Update world frame X velocity
    worldVelocity[1] = YPos.getVelocity(); // Update world frame Y velocity
    worldVelocity[2] = ZPos.getVelocity(); // Update world frame Z velocity

    // Update world frame acceleration from GPS data
    measuredWorldAccel[0] = XPos.getAcceleration(); // Update world frame X acceleration
    measuredWorldAccel[1] = YPos.getAcceleration(); // Update world frame Y acceleration
    measuredWorldAccel[2] = ZPos.getAcceleration(); // Update world frame Z acceleration
}

/**
 * @brief PID control loop for attitude and position control.
 */
void StateEstimation::PIDLoop(){
    // TODO may need sign flips
    float projectedLandingPosition[2];

    projectedLandingPosition[0] = worldPosition[1] + worldVelocity[1] * (PROJECTED_LANDING_TIME - timeSinceLaunch); // Projected landing position in Y
    projectedLandingPosition[1] = worldPosition[2] + worldVelocity[2] * (PROJECTED_LANDING_TIME - timeSinceLaunch); // Projected landing position in Z

    YDescentPID.compute(Y_TARGET, worldPosition[1], worldVelocity[1], true);
    ZDescentPID.compute(X_TARGET, worldPosition[2], worldVelocity[2], true);

    YAscentPID.compute(Y_TARGET, projectedLandingPosition[0], worldVelocity[1], true);
    ZAscentPID.compute(X_TARGET, projectedLandingPosition[1], worldVelocity[2], true);

    if(vehicleState < 5 || vehicleState == 67){ // use ascent PID controllers before apogee and in predictive PID test state
        attitudeSetpoint[0] = min(max(YAscentPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD); // Yaw
        attitudeSetpoint[1] = -min(max(ZAscentPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD);// Pitch
    }else{ // use descent PID controllers after apogee
        attitudeSetpoint[0] = min(max(YDescentPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD); // Yaw
        attitudeSetpoint[1] = -min(max(ZDescentPID.getOutput(), -MAX_ATTITIDE_SETPOINT_RAD), MAX_ATTITIDE_SETPOINT_RAD);// Pitch
    }

    YawPID.compute(attitudeSetpoint[0], getEulerAngle()[0], 0, false);
    PitchPID.compute(attitudeSetpoint[1], getEulerAngle()[1], 0, false);

    YawStabilizationPID.compute(0, getEulerAngle()[0], 0, false);
    PitchStabilizationPID.compute(0, getEulerAngle()[1], 0, false);

    RollPID.compute(0, gyroRemovedBias[2], 0, false);

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
    }

    float dt = (float)(micros() - lastActuatorMicros) / 1000000.0f; // Convert to seconds
    lastActuatorMicros = micros(); // Update last actuator time

    //seperate controllers instead of changing constants to prevent derivative kick
    if (vehicleState == 4 || vehicleState == 65){ // // If in return to vertical state or stabilization test state
        angularAccelCommand[0] = YawStabilizationPID.getOutput(); // Yaw angular acceleration command
        angularAccelCommand[1] = PitchStabilizationPID.getOutput(); // Pitch angular acceleration command
    }else{
        angularAccelCommand[0] = YawPID.getOutput(); // Yaw angular acceleration command
        angularAccelCommand[1] = PitchPID.getOutput(); // Pitch angular acceleration command
    }

    // roll mixer
    gimbalAngle[0] = sin(getEulerAngle()[2]) * (angularAccelCommand[1]) + cos(getEulerAngle()[2]) * (angularAccelCommand[0]); // Yaw gimbal angle command
    gimbalAngle[1] = cos(getEulerAngle()[2]) * (angularAccelCommand[1]) - sin(getEulerAngle()[2]) * (angularAccelCommand[0]); // Pitch gimbal angle command

    // convert from angular acceleration to gimbal angle
    float modifier = getPitchYawMMOI() / (getThrust() * getMomentArm()); // Modifier to convert angular acceleration to gimbal angle

    gimbalAngle[0] = asin(max(min(gimbalAngle[0] * modifier, 1), -1));
    gimbalAngle[1] = asin(max(min(gimbalAngle[1] * modifier, 1), -1));

    if(includePID == false){ // If includePID is false, do not include PID outputs and only has gimbal misalignment
        gimbalAngle[0] = 0;
        gimbalAngle[1] = 0;
    }
    gimbalAngle[0] += gimbalMisalign[0]; // Add gimbal misalignment to yaw gimbal angle command
    gimbalAngle[1] += gimbalMisalign[1]; // Add gimbal misalignment to pitch gimbal angle command

    gimbalAngle[0] = min(max(gimbalAngle[0], -GIMBAL_LIMIT_RAD), GIMBAL_LIMIT_RAD); // Limit gimbal angle to +/- GIMBAL_LIMIT_RAD
    gimbalAngle[1] = min(max(gimbalAngle[1], -GIMBAL_LIMIT_RAD), GIMBAL_LIMIT_RAD); // Limit gimbal angle to +/- GIMBAL_LIMIT_RAD
    // signs CORRECT up to here - note that positive pitch gimbal angle has bottom of gimbal towards +Z axis
    // positive yaw gimbal angle has bottom of gimbal towards -Y axis. 
    // Matches up with right hand rule


    //some code for actuation here - TODO may need sign flips
    // TODO Check signs
    float servoAngle[2];
    servoAngle[0] = 333.57967 * pow(gimbalAngle[0], 3) + 192.1374 * gimbalAngle[0]; // in degrees Yaw
    servoAngle[1] = 333.57967 * pow(gimbalAngle[1], 3) + 192.1374 * gimbalAngle[1]; // in degrees Pitch

    YawServo.write(90 + servoAngle[0]); // Set yaw servo angle, 90 degrees is center position
    PitchServo.write(90 + servoAngle[1]); // Set pitch servo angle, 90 degrees is center position

    // Actuate roll motor
}

void StateEstimation::actuateWheel(){
    if(lastWheelMicros == 0) { // If this is the first update, set lastWheelMicros to current time
        lastWheelMicros = micros();
        return;
    }
    if (micros() - lastWheelMicros < ACTUATOR_INTERVAL_US) { // If not enough time has passed since last update, return
        return;
    }

    float dt = (float)(micros() - lastWheelMicros) / 1000000.0f; // Convert to seconds

    lastWheelMicros = micros(); // Update last wheel time
    // Actuate roll motor
    // reaction wheel controller
    wheelSpeed += (RollPID.getOutput() / WHEEL_MOI) * dt;  // Roll wheel speed command, because PID requests torque and torque is only generated by changing wheel speed
    wheelSpeed = min(max(wheelSpeed, -MAX_WHEEL_SPEED), MAX_WHEEL_SPEED); // Limit wheel speed to max
    RollMotor.setSpeed(wheelSpeed); // Set roll motor speed based on wheel speed command

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
        if(worldPosition[0] > PYRO_LOCKOUT_ALT){
            digitalWrite(LAND_PYRO, HIGH); // Fire pyro
            if(vehicleState == 5){ // If vehicle is past apogee
                vehicleState = 6; // Change state to landing burn
            }
        }else{
            digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
        }
    }else{
        digitalWrite(LAND_PYRO, LOW); // Do not fire pyro
    }
}


bool StateEstimation::getPyroCont(){
    // Returns true if pyro continuity is detected, false otherwise
    return digitalRead(LAND_CONTINUITY) == HIGH; // Check if land continuity pin is HIGH
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
}

/**
 * @brief Updates acceleration, velocity, and position using IMU data.
 */
void StateEstimation::accelLoop(){
    if (lastAccelUpdate == 0) { // If this is the first update, set lastAccelUpdate to current time
        lastAccelUpdate = micros();
        return;
    }
    
    ori.updateAccel(resultIMU0Data.Acc1[1], resultIMU0Data.Acc1[0], resultIMU0Data.Acc1[2]); // Update the orientation with the measured acceleration in body frame

    measuredWorldAccel[0] = ori.worldAccel.b + WORLD_GRAVITY_X; // X acceleration in world frame
    measuredWorldAccel[1] = -ori.worldAccel.c + WORLD_GRAVITY_Y; // Y acceleration in world frame
    measuredWorldAccel[2] = ori.worldAccel.d + WORLD_GRAVITY_Z; // Z acceleration in world frame

    accelLoopMicros = micros();
    float dtAccel = (float)(accelLoopMicros - lastAccelUpdate) / 1000000.0f; // Convert to seconds
    lastAccelUpdate = accelLoopMicros;

    thrust = resultIMU0Data.Acc1[1] * getMass(); // Calculate thrust in Newtons based on acceleration in body frame and mass of the vehicle

    XPos.updateAccelerometer(measuredWorldAccel[0]);
    YPos.updateAccelerometer(measuredWorldAccel[1]);
    ZPos.updateAccelerometer(measuredWorldAccel[2]);

    worldPosition[0] = XPos.getPosition(); // Get X position in world frame
    worldPosition[1] = YPos.getPosition(); // Get Y position in world frame
    worldPosition[2] = ZPos.getPosition(); // Get Z position in world frame

    worldVelocity[0] = XPos.getVelocity(); // Get X velocity in world frame
    worldVelocity[1] = YPos.getVelocity(); // Get Y velocity in world frame
    worldVelocity[2] = ZPos.getVelocity(); // Get Z velocity in world frame

    worldAccel[0] = XPos.getAcceleration(); // Get X acceleration in world frame
    worldAccel[1] = YPos.getAcceleration(); // Get Y acceleration in world frame
    worldAccel[2] = ZPos.getAcceleration(); // Get Z acceleration in world frame
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
    Quaternion actualAccel = Quaternion(accelReading[0], accelReading[1], accelReading[2]); // Create quaternion from acceleration readings in body frame
    actualAccel = actualAccel.normalize(); // Normalize the quaternion to get unit vector

    ori.orientation = expectedGravity.rotation_between_vectors(actualAccel); // Compute the orientation quaternion by rotating expected gravity vector to actual acceleration vector
    //ori.zeroRoll();
    ori.orientation = ori.orientation.normalize(); // Normalize the orientation quaternion
    
    gps.setCurrentAsHome(); // Set current GPS position as home position
}

/**
 * @brief Sets the vehicle state.
 * Error codes:
 * -1 Invalid state
 * -2 Invalid transition
 * -3 Bad sensors
 * -4 Pyro continuity not detected
 */
int StateEstimation::setVehicleState(int state){
    // 0: Disarmed, 1: Armed, 2: Launching, 3: In Flight, 4: Landing, 5: Landed
    if (state >= -1 && state <= 7) {
        switch (state){
            case 1: // armed process
                if (vehicleState == 0){
                    if(sensorStatus == 0 && gps.getGPSInfo().fixType > 3){ // allow arm with RTKfix or RTK float, and all sensors functioning
                        if(getPyroCont() == false){ // Check if pyro continuity is not detected
                            return -4; // Cannot arm if pyro continuity is not detected
                        }
                        vehicleState = 1;
                        XPos.begin(); // Initialize X Kalman
                        YPos.begin(); // Initialize Y Kalman
                        ZPos.begin(); // Initialize Z Kalman
                    } else {
                        return -3; // Cannot arm if sensors are not all functioning
                    }
                }else{
                    return -2;
                }
                break;
            case 0: //disarm process
                resetVariables(); // Reset all variables to initial conditions
                XPos.reset();
                YPos.reset();
                ZPos.reset();
                vehicleState = 0; // Set vehicle state to disarmed
                break;
            default:
                vehicleState = state;
                break;
        }
    } else if (state >= 64){
        vehicleState = state; // Set vehicle state to testing states
    } else {
        return -1; // Invalid state
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
        if (abs(pow(resultIMU0Data.Acc1[1], 2) + pow(resultIMU0Data.Acc1[0], 2) + pow(resultIMU0Data.Acc1[2], 2)) > LAUNCH_ACCEL_THRESHOLD){
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
        if (vehicleState == 4){
            landingIgnitionAltitude = 0.6 * apogeeAltitude;
            vehicleState = 5;
        }
    }
}

// gets gimbal misalign, assumes that it starts at launch time = 0
void StateEstimation::getGimbalMisalign(){
    if(lastGimbalMisalignMicros == 0) { // If this is the first update, set lastGimbalMisalignMicros to current time
        lastGimbalMisalignMicros = micros();
        return;
    }
    if (vehicleState == 2 && timeSinceLaunch > 0){
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

        float i = getPitchYawMMOI();
        float r = getMomentArm();

        gimbalMisalign[0] = asin((2 * gimbalMisalignAccumulator[0] * i)/(r * f * timeSinceLaunch * timeSinceLaunch));
        gimbalMisalign[1] = asin((2 * gimbalMisalignAccumulator[1] * i)/(r * f * timeSinceLaunch * timeSinceLaunch));

    }
    
}


