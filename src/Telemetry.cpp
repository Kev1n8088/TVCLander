#include <SD.h>
#include <Telemetry.h>
#include <Arduino.h>
#include <Constants.h>
#include "StateEstimation.h"

EXTMEM uint8_t telemetryBuffer[MAX_DATA_LOGS * BYTES_PER_LOG];
static uint8_t serialBuffer[32 * 1024];

/**
 * @brief Telemetry constructor. Initializes member variables.
 */
Telemetry::Telemetry(){
    lastLogMillis = 0;
    lastTelemetryMillis = 0;
    oldVehicleState = 0;
    telemetryBufferUsed = 0;
    SDGood = 0;
    logFileName = "flightlog.bin";
}

/**
 * @brief Initializes telemetry serial, SD card, and debug serial.
 */
void Telemetry::begin(){
    TELEMETRY_SERIAL.begin(TELEMETRY_BAUD); // Initialize telemetry serial port with specified baud rate
    if (!SD.begin()) { // Initialize SD card
        SDGood = 0; // Set SDGood to 0 if SD card initialization fails
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("SD Card initialization failed!");
        }
    } else {
        SDGood = 1; // Set SDGood to 1 if SD card initialization is successful
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("SD Card initialized successfully!");
        }
    }
    
    TELEMETRY_SERIAL.addMemoryForWrite(serialBuffer, sizeof(serialBuffer));

}

/**
 * @brief Main telemetry loop. Handles logging and sending telemetry data.
 * @param state Reference to StateEstimation object for data access.
 */
void Telemetry::telemetryLoop(StateEstimation& state){
    uint64_t currentMillis = millis();

    if (currentMillis - lastLogMillis >= DATALOG_INTERVAL) {
        lastLogMillis = currentMillis;
        int vehicleState = state.getVehicleState();

        // Prepare telemetry data
        float quat[4];
        state.getOrientationQuaternionArray(quat);
        float worldAccel[3];
        float worldVelocity[3];
        float worldPosition[3];
        memcpy(worldAccel, state.getWorldAccel(), 3 * sizeof(float));
        memcpy(worldVelocity, state.getWorldVelocity(), 3 * sizeof(float));
        memcpy(worldPosition, state.getWorldPosition(), 3 * sizeof(float));
        float rawAccel[3];
        float rawGyro[3];
        state.getRawAccel(rawAccel);
        state.getRawGyro(rawGyro);
        float gyroBias[3];
        memcpy(gyroBias, state.getGyroBias(), 3 * sizeof(float));
        float attitudeSetpoint[2] = {0, 0}; // Placeholder
        float servoCommand[2] = {0, 0}; // Placeholder
        float thrust = state.getThrust();
        float reactionWheelSpeed = 0; // Placeholder
        int sensorStatus = state.getSensorStatus();
        int SDGoodVal = SDGood;

        // Log data
        // dataLog(
        //     millis() / 1000.0f, // timeSec
        //     quat,
        //     worldAccel,
        //     worldVelocity,
        //     worldPosition,
        //     rawAccel,
        //     rawGyro,
        //     gyroBias,
        //     attitudeSetpoint,
        //     servoCommand,
        //     thrust,
        //     reactionWheelSpeed,
        //     vehicleState,
        //     sensorStatus,
        //     SDGoodVal
        // );

        if (vehicleState != oldVehicleState) {
            oldVehicleState = vehicleState;
            if (vehicleState == 7) {
                dumpToSD();
            }
        }
        if (DEBUG_MODE) {
            //DEBUG_SERIAL.println("Data logged");
        }
    }

    if (currentMillis - lastTelemetryMillis >= TELEMETRY_INTERVAL) {
        lastTelemetryMillis = currentMillis;
        // Prepare telemetry data
        float quat[4];
        state.getOrientationQuaternionArray(quat);
        float worldAccel[3];
        float worldVelocity[3];
        float worldPosition[3];
        memcpy(worldAccel, state.getWorldAccel(), 3 * sizeof(float));
        memcpy(worldVelocity, state.getWorldVelocity(), 3 * sizeof(float));
        memcpy(worldPosition, state.getWorldPosition(), 3 * sizeof(float));
        float rawAccel[3];
        float rawGyro[3];
        state.getRawAccel(rawAccel);
        state.getRawGyro(rawGyro);
        float gyroBias[3];
        memcpy(gyroBias, state.getGyroBias(), 3 * sizeof(float));
        float attitudeSetpoint[2] = {0, 0}; // Placeholder
        float servoCommand[2] = {0, 0}; // Placeholder
        float thrust = state.getThrust();
        float reactionWheelSpeed = 0; // Placeholder
        int vehicleState = state.getVehicleState();
        int sensorStatus = state.getSensorStatus();
        int SDGoodVal = SDGood;
        sendTelemetry(
            millis() / 1000.0f,
            quat,
            worldAccel,
            worldVelocity,
            worldPosition,
            rawAccel,
            rawGyro,
            gyroBias,
            attitudeSetpoint,
            servoCommand,
            thrust,
            reactionWheelSpeed,
            vehicleState,
            sensorStatus,
            SDGoodVal
        );

        float eulerAngles[3]; // yaw pitch roll
        memcpy(eulerAngles, state.getEulerAngle(), 3 * sizeof(float));
        if (DEBUG_MODE){
            DEBUG_SERIAL.print("Orientation: ");
            DEBUG_SERIAL.print(degrees(eulerAngles[0]),4);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print(degrees(eulerAngles[1]),4);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.print(degrees(eulerAngles[2]),4);
            DEBUG_SERIAL.println();
        }
    }   
}

/**
 * @brief Logs telemetry data to PSRAM buffer in binary format.
 */
void Telemetry::dataLog(float timeSec, float quaternion[4], float worldAccel[3],
                 float worldVelocity[3], float worldPosition[3], float rawAccel[3],
                float rawGyro[3], float gyroBias[3], float attitudeSetpoint[2], float servoCommand[2], 
                float thrust, float reactionWheelSpeed, int vehicleState, int sensorStatus, int SDGood) {
    if (telemetryBufferUsed + BYTES_PER_LOG > sizeof(telemetryBuffer)) {
        // Buffer full, skip logging
        return;
    }         

    uint8_t* ptr = telemetryBuffer + telemetryBufferUsed;

    // Write separator
    uint32_t sep = LOG_SEPARATOR;
    memcpy(ptr, &sep, sizeof(sep)); ptr += sizeof(sep);

    // Write floats in order
    memcpy(ptr, &timeSec, sizeof(float)); ptr += sizeof(float);
    memcpy(ptr, quaternion, 4 * sizeof(float)); ptr += 4 * sizeof(float);
    memcpy(ptr, worldAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, worldVelocity, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, worldPosition, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, rawAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, rawGyro, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, gyroBias, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, attitudeSetpoint, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    memcpy(ptr, servoCommand, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    memcpy(ptr, &thrust, sizeof(float)); ptr += sizeof(float);
    memcpy(ptr, &reactionWheelSpeed, sizeof(float)); ptr += sizeof(float);

    // Write int
    memcpy(ptr, &vehicleState, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, &sensorStatus, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, &SDGood, sizeof(int)); ptr += sizeof(int);

    telemetryBufferUsed += BYTES_PER_LOG;
}

/**
 * @brief Dumps logged telemetry data from PSRAM to SD card as binary file.
 */
void Telemetry::dumpToSD(){
    if (telemetryBufferUsed == 0) return;

    File logFile = SD.open(logFileName.c_str(), FILE_WRITE); // Use configurable file name
    if (logFile) {
        logFile.write(telemetryBuffer, telemetryBufferUsed);
        logFile.close();
    }

    // Clear buffer for next use
    telemetryBufferUsed = 0;
}

/**
 * @brief Packs and sends telemetry data over serial in binary format.
 */
void Telemetry::sendTelemetry(float timeSec, float quaternion[4], float worldAccel[3],
                 float worldVelocity[3], float worldPosition[3], float rawAccel[3],
                float rawGyro[3], float gyroBias[3], float attitudeSetpoint[2], float servoCommand[2], 
                float thrust, float reactionWheelSpeed, int vehicleState, int sensorStatus, int SDGood) {
    // Pack data in the same binary format as dataLog
    uint8_t buffer[BYTES_PER_LOG];
    uint8_t* ptr = buffer;
    uint32_t sep = LOG_SEPARATOR;
    memcpy(ptr, &sep, sizeof(sep)); ptr += sizeof(sep);
    memcpy(ptr, &timeSec, sizeof(float)); ptr += sizeof(float);
    memcpy(ptr, quaternion, 4 * sizeof(float)); ptr += 4 * sizeof(float);
    memcpy(ptr, worldAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, worldVelocity, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, worldPosition, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, rawAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, rawGyro, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, gyroBias, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    memcpy(ptr, attitudeSetpoint, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    memcpy(ptr, servoCommand, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    memcpy(ptr, &thrust, sizeof(float)); ptr += sizeof(float);
    memcpy(ptr, &reactionWheelSpeed, sizeof(float)); ptr += sizeof(float);
    memcpy(ptr, &vehicleState, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, &sensorStatus, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, &SDGood, sizeof(int)); ptr += sizeof(int);
    TELEMETRY_SERIAL.write(buffer, BYTES_PER_LOG);

    if (DEBUG_MODE){
        // DEBUG_SERIAL.print("Quaternion: ");
        // DEBUG_SERIAL.print(quaternion[0], 4);
        // DEBUG_SERIAL.print(", ");
        // DEBUG_SERIAL.print(quaternion[1], 4);
        // DEBUG_SERIAL.print(", ");
        // DEBUG_SERIAL.print(quaternion[2], 4);
        // DEBUG_SERIAL.print(", ");
        // DEBUG_SERIAL.print(quaternion[3], 4);
        // DEBUG_SERIAL.println();
    }
}