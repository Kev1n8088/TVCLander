#include <SD.h>
#include <Telemetry.h>
#include <Arduino.h>
#include <Constants.h>
#include "StateEstimation.h"
#include "LINK80.cpp"

EXTMEM uint8_t telemetryBuffer[MAX_DATA_LOGS * BYTES_PER_LOG];
static uint8_t serialBuffer[32 * 1024];

enum PacketType {
    PACKET_TELEMETRY = 0x01,
    PACKET_COMMAND   = 0x02
};

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

    //handleReceive(state);

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
            // DEBUG_SERIAL.print("Orientation: ");
            // DEBUG_SERIAL.print(degrees(eulerAngles[0]),4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(degrees(eulerAngles[1]),4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(degrees(eulerAngles[2]),4);
            // DEBUG_SERIAL.println();

            
            // DEBUG_SERIAL.print("Acceleration: ");
            // DEBUG_SERIAL.print(worldAccel[0],4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(worldAccel[1],4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(worldAccel[2],4);
            // DEBUG_SERIAL.println();

            // DEBUG_SERIAL.print("Quaternion: ");
            // DEBUG_SERIAL.print(quat[0], 4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(quat[1], 4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(quat[2], 4);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.println(quat[3], 4);

            // DEBUG_SERIAL.print("Quaternion: ");
            // DEBUG_SERIAL.print(1.000);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(0);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.print(0);
            // DEBUG_SERIAL.print(", ");
            // DEBUG_SERIAL.println(0);

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
    // uint8_t buffer[BYTES_PER_LOG + 4];
    // uint8_t* ptr = buffer;
    // uint32_t sep = LOG_SEPARATOR;
    // memcpy(ptr, &sep, sizeof(sep)); ptr += sizeof(sep);
    // memcpy(ptr, &timeSec, sizeof(float)); ptr += sizeof(float);
    // memcpy(ptr, quaternion, 4 * sizeof(float)); ptr += 4 * sizeof(float);
    // memcpy(ptr, worldAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, worldVelocity, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, worldPosition, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, rawAccel, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, rawGyro, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, gyroBias, 3 * sizeof(float)); ptr += 3 * sizeof(float);
    // memcpy(ptr, attitudeSetpoint, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    // memcpy(ptr, servoCommand, 2 * sizeof(float)); ptr += 2 * sizeof(float);
    // memcpy(ptr, &thrust, sizeof(float)); ptr += sizeof(float);
    // memcpy(ptr, &reactionWheelSpeed, sizeof(float)); ptr += sizeof(float);
    // memcpy(ptr, &vehicleState, sizeof(int)); ptr += sizeof(int);
    // memcpy(ptr, &sensorStatus, sizeof(int)); ptr += sizeof(int);
    // memcpy(ptr, &SDGood, sizeof(int)); ptr += sizeof(int);

    
    // uint32_t crc = crc32(buffer, BYTES_PER_LOG);
    // memcpy(ptr, &crc, sizeof(crc));
    // ptr += sizeof(crc);

    LINK80::StateTelemetry state = {
        .vehicle_state = (int8_t)vehicleState,
        .quat_w = quaternion[0],
        .quat_x = quaternion[1],
        .quat_y = quaternion[2],
        .quat_z = quaternion[3],
        .accel_x = worldAccel[0],
        .accel_y = worldAccel[1],
        .accel_z = worldAccel[2],
        .vel_x = worldVelocity[0],
        .vel_y = worldVelocity[1],
        .vel_z = worldVelocity[2],
        .pos_x = worldPosition[0],
        .pos_y = worldPosition[1],
        .pos_z = worldPosition[2],
        .time_since_launch = timeSec
    };

    uint8_t packet_buffer[255];
    size_t packet_size = LINK80::packStateTelemetry(state, packet_buffer);

    // packet_size = LINK80::packCommandAck(51, 0, packet_buffer);

    if (packet_size == 0) {
        // Packing failed
        return;
    }

    TELEMETRY_SERIAL.write(packet_buffer, packet_size);

    if (DEBUG_MODE){
        //DEBUG_SERIAL.write(packet_buffer, packet_size);
        //for 6 point calibration
        DEBUG_SERIAL.print(rawAccel[1], 4);
        DEBUG_SERIAL.print(", ");
        DEBUG_SERIAL.print(rawAccel[0], 4);
        DEBUG_SERIAL.print(", ");
        DEBUG_SERIAL.println(rawAccel[2], 4);
    }
}

// void Telemetry::handleReceive(StateEstimation& state) {
//     static uint8_t rxBuffer[256];
//     static size_t rxIndex = 0;

//     while (TELEMETRY_SERIAL.available()) {
//         uint8_t byte = TELEMETRY_SERIAL.read();
//         rxBuffer[rxIndex++] = byte;

//         // Wait for at least header
//         if (rxIndex >= 4) {
//             uint16_t delim = (rxBuffer[0] << 8) | rxBuffer[1];
//             uint8_t type = rxBuffer[2];
//             uint8_t len = rxBuffer[3];

//             // Check start delimiter
//             if (delim != RECV_START_DELIM) {
//                 // Shift buffer left by one
//                 memmove(rxBuffer, rxBuffer + 1, --rxIndex);
//                 continue;
//             }

//             // Wait for full packet
//             if (rxIndex < 4 + len + 4) continue;

//             // CRC check
//             uint32_t receivedCrc;
//             memcpy(&receivedCrc, rxBuffer + 4 + len, 4);
//             uint32_t calcCrc = crc32(rxBuffer, 4 + len);

//             if (receivedCrc == calcCrc) {
//                 // Valid packet
//                 if (type == PACKET_COMMAND) {
//                     // Extract first 4 bytes of payload as command ID
//                     if (len >= 4) {
//                         uint32_t cmdId;
//                         memcpy(&cmdId, rxBuffer + 4, 4);
//                         switch (cmdId) {
//                             case 0xAAAAAAAA: // Arm command
//                                     state.setVehicleState(1); 
//                                 break;
//                             case 0xDDDDDDDD: // Disarm command
//                                     state.setVehicleState(0);
//                                 break;
//                             default:
//                                 // Unknown command
//                                 break;
//                         }
//                     }
//                 } else if (type == PACKET_TELEMETRY) {
//                     // Handle telemetry
//                 }
//             }
//             // Remove processed packet from buffer
//             size_t packetSize = 4 + len + 4;
//             memmove(rxBuffer, rxBuffer + packetSize, rxIndex - packetSize);
//             rxIndex -= packetSize;
//         }
//     }
// }

// uint32_t Telemetry::crc32(const uint8_t* data, size_t length) {
//     uint32_t crc = 0xFFFFFFFF;
//     for (size_t i = 0; i < length; ++i) {
//         crc ^= data[i];
//         for (int j = 0; j < 8; ++j) {
//             if (crc & 1)
//                 crc = (crc >> 1) ^ 0xEDB88320;
//             else
//                 crc >>= 1;
//         }
//     }
//     return ~crc;
// }