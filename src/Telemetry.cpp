#include <SD.h>
#include <Telemetry.h>
#include <Arduino.h>
#include <Constants.h>
#include "StateEstimation.h"

static uint8_t telemetryBuffer[MAX_BYTES_PER_LOG];
static uint8_t serialBuffer[32 * 1024];

DMAMEM static uint8_t telemetryPacketBuffer[LINK80::MAX_PACKET_SIZE * 8];

enum PacketType {
    PACKET_TELEMETRY = 0x01,
    PACKET_COMMAND   = 0x02
};


struct RTCMBuffer {
    uint8_t data[1030];
    size_t length;
    bool active;
    uint32_t lastUpdateMillis;
};

DMAMEM static RTCMBuffer rtcmBuffers[128]; // RTCM ID is uint8_t

DMAMEM static uint8_t receiveBuffer[LINK80::MAX_PACKET_SIZE * 8]; // Buffer for received packets

/**
 * @brief Telemetry constructor. Initializes member variables.
 */
Telemetry::Telemetry(){
    lastLogMillis = 0;
    lastTelemetryMillis = 0;
    telemetryBufferUsed = 0;
    SDGood = false;
    downCount = 0;
    downCountRadio = 0;
    packetBufferLen = 0;
    currentPacketType = 0;
    newestRTCMID = 0;
    numRTCMDropped = 0;
}

/**
 * @brief Initializes telemetry serial, SD card, and debug serial.
 */
void Telemetry::begin(){
    strcpy(logFileName, "flightlog.bin");
    TELEMETRY_SERIAL.begin(TELEMETRY_BAUD); // Initialize telemetry serial port with specified baud rate
    pinMode(VBAT_SENSE_PIN, INPUT); // Set VBAT sense pin as input
    if (!SD.begin(BUILTIN_SDCARD)) { // Initialize SD card
        SDGood = false; // Set SDGood to 0 if SD card initialization fails
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("SD Card initialization failed!");
        }
    } else {
        SDGood = true; // Set SDGood to 1 if SD card initialization is successful
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("SD Card initialized successfully!");
        }
    }

    String filename;
    int fileNumber = 1;
    do {
        filename = "flight" + String(fileNumber) + ".bin";
        fileNumber++;
    } while (SD.exists(filename.c_str()) && fileNumber < 10000); // Prevent infinite loop
    
    if (fileNumber >= 10000) {
        // Fallback to timestamp-based filename if too many files exist
        filename = "flight_" + String(millis()) + ".bin";
    }

    logFile = SD.open(filename.c_str(), FILE_WRITE);
    if (!logFile) {
        SDGood = false;
    }
    
    TELEMETRY_SERIAL.addMemoryForWrite(serialBuffer, sizeof(serialBuffer));

}

/**
 * @brief Main telemetry loop. Handles logging and sending telemetry data.
 * @param state Reference to StateEstimation object for data access.
 */
void Telemetry::telemetryLoop(StateEstimation& state){
    uint64_t currentMillis = millis();

    handleReceive(state);
    
    if(state.getVehicleState() == 0 || state.getVehicleState() == 7) {
        logFile.flush();
        sdCounter = 0; // Reset SD counter when disarmed   
    }

    if(state.getVehicleState() > 1 && state.getVehicleState() < 7) {
        if (currentMillis - lastLogMillis >= DATALOG_INTERVAL) {
            lastLogMillis = currentMillis;
            dataLog(state);
        }
    }

    if (currentMillis - lastTelemetryMillis >= TELEMETRY_INTERVAL) {
        lastTelemetryMillis = currentMillis;
        // Prepare telemetry data
        sendTelemetry(state);

        if (DEBUG_MODE){    
            // DEBUG_SERIAL.print("Yaw: ");
            // DEBUG_SERIAL.print(state.getEulerAngle()[0]);
            // DEBUG_SERIAL.print(" Pitch: ");
            // DEBUG_SERIAL.print(state.getEulerAngle()[1]);
            // DEBUG_SERIAL.print(" Roll: ");
            // DEBUG_SERIAL.println(state.getEulerAngle()[2]);
            if(state.getVehicleState() == 69){
                //DEBUG_SERIAL.println(state.getDt());
            }
        }
    }   
}




/**
 * @brief Packs and sends telemetry data over serial in binary format.
 */
void Telemetry::sendTelemetry(StateEstimation& state) {

    downCountRadio = (downCountRadio + 1) % UINT16_MAX; // Increment downCount and wrap around at 2^32


    currentPacketType = (currentPacketType + 1) % 2;
    //currentPacketType = 2; //testing

    size_t packet_size = 0;

    LINK80::StateTelemetry stateTelem;
    LINK80::GPSData gps;
    GPSInfo& gpsInfo = state.getGPSInfo();

    float vbat = (((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER;


    uint8_t pyroMask = (state.getChuteCont() ? (1 << 0) : 0) | (state.getPyroCont() ? (1 << 1) : 0); // Create a bitmask for pyro status. First bit is chute, second bit is landing motor
    switch (currentPacketType){
        case 0:
            stateTelem = {
                .vehicle_state = (int8_t)state.getVehicleState(),
                .quat_w = state.getOrientationQuaternion().a,
                .quat_x = state.getOrientationQuaternion().b,
                .quat_y = state.getOrientationQuaternion().c,
                .quat_z = state.getOrientationQuaternion().d,
                .accel_x = state.getWorldAccel()[0],
                .accel_y = state.getWorldAccel()[1],
                .accel_z = state.getWorldAccel()[2],
                .vel_x = state.getWorldVelocity()[0],
                .vel_y = state.getWorldVelocity()[1],
                .vel_z = state.getWorldVelocity()[2],
                .pos_x = state.getWorldPosition()[0],
                .pos_y = state.getWorldPosition()[1],
                .pos_z = state.getWorldPosition()[2],
                .time_since_launch = state.getTimeSinceLaunch(),
                .failmask = state.getSensorStatus(),
                .sd_good = SDGood,
                .gyro_yaw = state.getGyroRemovedBias()[0],
                .gyro_pitch = state.getGyroRemovedBias()[1],
                .gyro_roll = state.getGyroRemovedBias()[2],
                .accelerometer_x = state.getAccelCalibrated()[0],
                .accelerometer_y = state.getAccelCalibrated()[1],
                .accelerometer_z = state.getAccelCalibrated()[2],
                .baro_altitude = state.getGPSHandler().getUpdateInterval(), // zero for now since barometer is not used
                .gyro_bias_yaw = state.getGyroBias()[0],
                .gyro_bias_pitch = state.getGyroBias()[1],
                .gyro_bias_roll = state.getGyroBias()[2],
                .accUncX = state.getAccelUncertainty()[0],
                .accUncY = state.getAccelUncertainty()[1],
                .accUncZ = state.getAccelUncertainty()[2],
                .velUncX = state.getVelocityUncertainty()[0],
                .velUncY = state.getVelocityUncertainty()[1],
                .velUncZ = state.getVelocityUncertainty()[2],
                .posUncX = state.getPositionUncertainty()[0],
                .posUncY = state.getPositionUncertainty()[1],
                .posUncZ = state.getPositionUncertainty()[2],
                .accelMeasuredX = state.getMeasuredRotatedAccel()[0],
                .accelMeasuredY = state.getMeasuredRotatedAccel()[1],
                .accelMeasuredZ = state.getMeasuredRotatedAccel()[2],
                .velMeasuredX = state.getAdjustedGPSVelocity()[0],
                .velMeasuredY = state.getAdjustedGPSVelocity()[1],
                .velMeasuredZ = state.getAdjustedGPSVelocity()[2],
                .posMeasuredX = state.getAdjustedGPSPosition()[0],
                .posMeasuredY = state.getAdjustedGPSPosition()[1],
                .posMeasuredZ = state.getAdjustedGPSPosition()[2],
                .vehicle_ms = millis(),
                .down_count = downCountRadio
            };
            break;
        case 1:
            gps = {
                .satsInView = (uint8_t)gpsInfo.satsInView,
                .satsUsed = (uint8_t)gpsInfo.satsUsed,
                .fixType = (uint8_t)gpsInfo.fixType,
                .latitude = (float)gpsInfo.pos.latitude,
                .longitude = (float)gpsInfo.pos.longitude,
                .altitude = (float)gpsInfo.pos.altitude,
                .accuracy2D = gpsInfo.error2D,
                .accuracy3D = gpsInfo.error3D,
                .PDOP = gpsInfo.pdop,
                .gps_ms = gpsInfo.timeOfWeek,
                .lastRTCM = gpsInfo.rtcmAge,
                .latHome = (float)gpsInfo.home.latitude,
                .lonHome = (float)gpsInfo.home.longitude,
                .altHome = (float)gpsInfo.home.altitude,
                .downVel = (float)gpsInfo.pos.velocityDown,
                .eastVel = (float)gpsInfo.pos.velocityEast,
                .northVel = (float)gpsInfo.pos.velocityNorth,
                .relX = (float)gpsInfo.xyz.x,
                .relY = (float)gpsInfo.xyz.y,
                .relZ = (float)gpsInfo.xyz.z,
                .YTarget = Y_TARGET,
                .ZTarget = Z_TARGET,
                .ignitionAlt = state.getIgnitionAltitude(),
                .apogeeAlt = state.getApogeeAltitude(),
                .yawSetpoint = state.getAttitudeSetpoint()[0],
                .pitchSetpoint = state.getAttitudeSetpoint()[1],
                .yawCommand = state.getAngularAccelCommand()[0],
                .pitchCommand = state.getAngularAccelCommand()[1],
                .rollMixedYaw = state.getGimbalAngle()[0],
                .rollMixedPitch = state.getGimbalAngle()[1],
                .yawMisalign = state.getGimbalMisalign()[0],
                .pitchMisalign = state.getGimbalMisalign()[1],
                .rollCommand = state.getWheelSpeed(),
                .YProjected = state.getPositionSetpoints()[0],
                .ZProjected = state.getPositionSetpoints()[1],
                .VBAT = vbat, 
                .thrust = state.getThrust(),
                .mass = state.getMass(),
                .MMOI = state.getPitchYawMMOI(),
                .momentArm = state.getMomentArm(),
                .pyroStatus = pyroMask,
                .vehicle_ms = millis(),
                .down_count = downCountRadio
            };
            break;
    };

    // Use static buffer for packing
    if(currentPacketType == 0){
        packet_size = LINK80::packStateTelemetry(stateTelem, telemetryPacketBuffer);
    }else if(currentPacketType == 1){
        packet_size = LINK80::packGPSData(gps, telemetryPacketBuffer);
    }
    if (packet_size == 0) {
        // Packing failed
        DEBUG_SERIAL.println("Failed to pack telemetry data");
        return;
    }

    if(TELEMETRY_SERIAL.availableForWrite() > packet_size){

        TELEMETRY_SERIAL.write(telemetryPacketBuffer, packet_size); //ensure nonblocking

    }

    // if (DEBUG_MODE){
    //     DEBUG_SERIAL.write(telemetryPacketBuffer, packet_size);
    //     //DEBUG_SERIAL.println((((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER); // Send battery voltage
    // }
}

void Telemetry::returnAck(uint8_t messageType, uint8_t commandID, uint8_t errorCode) {
    
    downCount = (downCount + 1) % 4294967296; // Increment downCount and wrap around at 2^32

    packet_size = LINK80::packCommandAck(messageType, commandID, errorCode, telemetryPacketBuffer, millis(), downCount);
    if (packet_size > 0) {

        if (TELEMETRY_SERIAL.availableForWrite() > packet_size) {
            TELEMETRY_SERIAL.write(telemetryPacketBuffer, packet_size);
        }
        // if (DEBUG_MODE){
        //     DEBUG_SERIAL.write(telemetryPacketBuffer, packet_size);
        // }
    }
}

void Telemetry::handleReceive(StateEstimation& state) {
    // Read incoming data with proper bounds checking
    while (TELEMETRY_SERIAL.available() > 0 && packetBufferLen < sizeof(receiveBuffer)) {
        receiveBuffer[packetBufferLen++] = TELEMETRY_SERIAL.read();
    }
    // Process all complete packets
    while (packetBufferLen >= LINK80::HEADER_SIZE + LINK80::CHECKSUM_SIZE) {
        size_t packetSize = findAndExtractPacket();
        if (packetSize > 0) {
            LINK80::UnpackedPacket unpacked = LINK80::unpackPacket(receiveBuffer, packetSize);
            DEBUG_SERIAL.println(unpacked.message_type);
            DEBUG_SERIAL.println(unpacked.error);
            if (unpacked.message_type >= 10 && unpacked.message_type <= 30) {
                // Handle command packet
                uint8_t commandID = LINK80::parseCommand(unpacked);
                uint8_t error = 0; // Default to no error
                switch (unpacked.message_type){
                    case LINK80::MessageType::PING:
                        state.getGPSHandler().resetHome();
                        returnAck(LINK80::MessageType::PING, commandID, 0);
                        break;
                    case LINK80::MessageType::DISARM:
                        error = state.setVehicleState(0);
                        //DEBUG_SERIAL.println("Trying Disarming");
                        returnAck(LINK80::MessageType::DISARM, commandID, error);
                        break;
                    case (LINK80::MessageType::ARM):
                        error = state.setVehicleState(1);
                        returnAck(LINK80::MessageType::ARM, commandID, error);
                        break;
                    case (LINK80::MessageType::TEST_PREP):
                        error = state.setVehicleState(69);
                        returnAck(LINK80::MessageType::TEST_PREP, commandID, error);
                        break;
                    case (LINK80::MessageType::WHEEL_TEST):
                        error = state.setVehicleState(64);
                        returnAck(LINK80::MessageType::WHEEL_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::STAB_TEST):
                        error = state.setVehicleState(65);
                        returnAck(LINK80::MessageType::STAB_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::POS_TEST):
                        error = state.setVehicleState(66);
                        returnAck(LINK80::MessageType::POS_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::TRAJ_TEST):
                        error = state.setVehicleState(67);
                        returnAck(LINK80::MessageType::TRAJ_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::MISALIGN_TEST):
                        error = state.setVehicleState(68);
                        returnAck(LINK80::MessageType::MISALIGN_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::PYRO_TEST):
                        error = state.setVehicleState(70);
                        returnAck(LINK80::MessageType::PYRO_TEST, commandID, error);
                        break;
                    case (LINK80::MessageType::ABORT):
                        error = state.setVehicleState(71);
                        returnAck(LINK80::MessageType::ABORT, commandID, error);
                        break;
                }
                packetBufferLen = 0;
                break;
            }
            
            LINK80::UnpackedPacket unpacked = LINK80::unpackPacket(receiveBuffer, packet_size);
            
            if (DEBUG_MODE) {
                DEBUG_SERIAL.print("MSG Type: ");
                DEBUG_SERIAL.print(unpacked.message_type);
                DEBUG_SERIAL.print(" Error: ");
                DEBUG_SERIAL.println(unpacked.error);
            }
            
            if (unpacked.valid) {
                if (unpacked.message_type >= 10 && unpacked.message_type <= 30) {
                    // Handle command packet
                    uint8_t commandID = LINK80::parseCommand(unpacked);
                    uint8_t error = 0; // Default to no error
                    switch (unpacked.message_type){
                        case LINK80::MessageType::PING:
                            // Respond to ping with a command ack
                            returnAck(LINK80::MessageType::PING, commandID, 0);
                            break;
                        case LINK80::MessageType::DISARM:
                            error = state.setVehicleState(0);
                            DEBUG_SERIAL.println("Trying Disarming");
                            returnAck(LINK80::MessageType::DISARM, commandID, error);
                            break;
                        case (LINK80::MessageType::ARM):
                            error = state.setVehicleState(1);
                            returnAck(LINK80::MessageType::ARM, commandID, error);
                            break;
                        case (LINK80::MessageType::TEST_PREP):
                            error = state.setVehicleState(69);
                            returnAck(LINK80::MessageType::TEST_PREP, commandID, error);
                            break;
                        case (LINK80::MessageType::WHEEL_TEST):
                            error = state.setVehicleState(64);
                            returnAck(LINK80::MessageType::WHEEL_TEST, commandID, error);
                            break;
                        case (LINK80::MessageType::STAB_TEST):
                            error = state.setVehicleState(65);
                            returnAck(LINK80::MessageType::STAB_TEST, commandID, error);
                            break;
                    }
                }
                if (unpacked.message_type >= 51 && unpacked.message_type <= 55) {
                    handleRTCM(unpacked, state);
                }
            } else if (DEBUG_MODE) {
                DEBUG_SERIAL.println("Invalid packet received");
            }
            
            // Remove processed packet from buffer
            if (packet_size <= packetBufferLen) {
                memmove(receiveBuffer, receiveBuffer + packet_size, packetBufferLen - packet_size);
                packetBufferLen -= packet_size;
            } else {
                // Safety: clear buffer if packet size is inconsistent
                packetBufferLen = 0;
            }
            
            processedPackets++;
        } else {
            break; // No complete packet found
        }
    }
}

size_t Telemetry::findAndExtractPacket() {
    // Look for packet header
    headerIndex = SIZE_MAX;
    for (size_t i = 0; i < packetBufferLen; ++i) {
        if (receiveBuffer[i] == LINK80::PACKET_HEADER) {
            headerIndex = i;
            break;
        }
    }
    if (headerIndex == SIZE_MAX) {
        // No header found, clear buffer if it's getting full
        if (packetBufferLen > sizeof(receiveBuffer) * 0.75) {
            packetBufferLen = 0;
            if (DEBUG_MODE) {
                DEBUG_SERIAL.println("No header found - buffer cleared");
            }
        }
        return 0;
    }

    // Remove any data before the header
    if (headerIndex > 0) {
        memmove(receiveBuffer, receiveBuffer + headerIndex, packetBufferLen - headerIndex);
        packetBufferLen -= headerIndex;
    }

    // Check if we have enough data for header
    if (packetBufferLen < LINK80::HEADER_SIZE) {
        return 0;
    }

    uint8_t payloadLength = receiveBuffer[1];
    
    // Validate payload length to prevent buffer overflow
    const uint8_t MAX_REASONABLE_PAYLOAD = 200; // Adjust based on your protocol
    if (payloadLength > MAX_REASONABLE_PAYLOAD) {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.print("Invalid payload length: ");
            DEBUG_SERIAL.println(payloadLength);
        }
        // Remove the bad header and try to find next one
        if (packetBufferLen > 1) {
            memmove(receiveBuffer, receiveBuffer + 1, packetBufferLen - 1);
            packetBufferLen--;
        } else {
            packetBufferLen = 0;
        }
        return 0;
    }
    
    size_t expectedPacketSize = LINK80::HEADER_SIZE + payloadLength;

    // Additional safety check against buffer overflow
    if (expectedPacketSize > sizeof(receiveBuffer)) {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("Packet too large for buffer");
        }
        packetBufferLen = 0; // Clear buffer
        return 0;
    }

    // Check if we have the complete packet
    if (packetBufferLen < expectedPacketSize) {
        return 0;
    }

    // Return the size of the complete packet
    return expectedPacketSize;
}



void Telemetry::handleRTCM(const LINK80::UnpackedPacket& packet, StateEstimation& state) {
    if (!packet.valid || packet.message_type < 51 || packet.message_type > 55) return;

    // Validate packet has at least 1 byte for RTCM ID
    if (packet.data_length < 1) {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("RTCM packet too short");
        }
        return;
    }

    uint8_t rtcm_id = packet.data[0];
    if (rtcm_id >= 128) {
        // Invalid RTCM ID, ignore packet
        if (DEBUG_MODE) {
            DEBUG_SERIAL.print("Invalid RTCM ID: ");
            DEBUG_SERIAL.println(rtcm_id);
        }
        return;
    }

    size_t fragment_len = packet.data_length - 1; // Exclude RTCM ID byte
    
    // Validate fragment length
    if (fragment_len > 1024) { // Reasonable max fragment size
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("RTCM fragment too large");
        }
        return;
    }

    RTCMBuffer& buf = rtcmBuffers[rtcm_id];
    if (!buf.active) {
        buf.length = 0;
        buf.active = true;
    }

    // Append fragment with bounds checking
    if (buf.length + fragment_len <= sizeof(buf.data)) {
        memcpy(buf.data + buf.length, packet.data + 1, fragment_len);
        buf.length += fragment_len;
    } else {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("RTCM buffer overflow - discarding");
        }
        // Reset buffer on overflow
        buf.active = false;
        buf.length = 0;
        return;
    }

    buf.lastUpdateMillis = millis();

    // If message_type == 55, packet is complete
    if (packet.message_type == 55) {
        state.getGPSHandler().sendRTCMCorrection(buf.data, buf.length);
        buf.active = false;
        buf.length = 0;
        buf.lastUpdateMillis = 0;
        //returnAck(121, 0, 0);
    }

    // Cleanup stale buffers
    for (int i = 0; i < 128; ++i) {
        if (rtcmBuffers[i].active && millis() - rtcmBuffers[i].lastUpdateMillis > 3000) { // 3 second timeout
            rtcmBuffers[i].active = false;
            rtcmBuffers[i].length = 0;
            rtcmBuffers[i].lastUpdateMillis = 0;
        }
    }
}

/**
 * @brief Logs telemetry data to SD Card in binary format.
 */
void Telemetry::dataLog(StateEstimation& state) {

    telemetryBufferUsed = 0;


    downCount = (downCount + 1) % UINT16_MAX; // Increment downCount and wrap around

    size_t packet_size = 0;
    uint8_t tempBuffer[LINK80::MAX_PACKET_SIZE]; // Temporary buffer for packing

    LINK80::StateTelemetry stateTelem;
    LINK80::GPSData gps;
    GPSInfo& gpsInfo = state.getGPSInfo();
    uint8_t pyroMask = (state.getChuteCont() ? (1 << 0) : 0) | (state.getPyroCont() ? (1 << 1) : 0); // Create a bitmask for pyro status. First bit is chute, second bit is landing motor

    float vbat = (((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER;

    // 1. State telemetry packet
    stateTelem = {
        .vehicle_state = (int8_t)state.getVehicleState(),
        .quat_w = state.getOrientationQuaternion().a,
        .quat_x = state.getOrientationQuaternion().b,
        .quat_y = state.getOrientationQuaternion().c,
        .quat_z = state.getOrientationQuaternion().d,
        .accel_x = state.getWorldAccel()[0],
        .accel_y = state.getWorldAccel()[1],
        .accel_z = state.getWorldAccel()[2],
        .vel_x = state.getWorldVelocity()[0],
        .vel_y = state.getWorldVelocity()[1],
        .vel_z = state.getWorldVelocity()[2],
        .pos_x = state.getWorldPosition()[0],
        .pos_y = state.getWorldPosition()[1],
        .pos_z = state.getWorldPosition()[2],
        .time_since_launch = state.getTimeSinceLaunch(),
        .failmask = state.getSensorStatus(),
        .sd_good = SDGood,
        .gyro_yaw = state.getGyroRemovedBias()[0],
        .gyro_pitch = state.getGyroRemovedBias()[1],
        .gyro_roll = state.getGyroRemovedBias()[2],
        .accelerometer_x = state.getAccelCalibrated()[0],
        .accelerometer_y = state.getAccelCalibrated()[1],
        .accelerometer_z = state.getAccelCalibrated()[2],
        .baro_altitude = state.getGPSHandler().getUpdateInterval(), // zero for now since barometer is not used
        .gyro_bias_yaw = state.getGyroBias()[0],
        .gyro_bias_pitch = state.getGyroBias()[1],
        .gyro_bias_roll = state.getGyroBias()[2],
        .accUncX = state.getAccelUncertainty()[0],
        .accUncY = state.getAccelUncertainty()[1],
        .accUncZ = state.getAccelUncertainty()[2],
        .velUncX = state.getVelocityUncertainty()[0],
        .velUncY = state.getVelocityUncertainty()[1],
        .velUncZ = state.getVelocityUncertainty()[2],
        .posUncX = state.getPositionUncertainty()[0],
        .posUncY = state.getPositionUncertainty()[1],
        .posUncZ = state.getPositionUncertainty()[2],
        .accelMeasuredX = state.getMeasuredRotatedAccel()[0],
        .accelMeasuredY = state.getMeasuredRotatedAccel()[1],
        .accelMeasuredZ = state.getMeasuredRotatedAccel()[2],
        .velMeasuredX = state.getAdjustedGPSVelocity()[0],
        .velMeasuredY = state.getAdjustedGPSVelocity()[1],
        .velMeasuredZ = state.getAdjustedGPSVelocity()[2],
        .posMeasuredX = state.getAdjustedGPSPosition()[0],
        .posMeasuredY = state.getAdjustedGPSPosition()[1],
        .posMeasuredZ = state.getAdjustedGPSPosition()[2],
        .vehicle_ms = millis(),
        .down_count = downCount
    };
    packet_size = LINK80::packStateTelemetry(stateTelem, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }

    // GPS data packet
    gps = {
        .satsInView = (uint8_t)gpsInfo.satsInView,
        .satsUsed = (uint8_t)gpsInfo.satsUsed,
        .fixType = (uint8_t)gpsInfo.fixType,
        .latitude = (float)gpsInfo.pos.latitude,
        .longitude = (float)gpsInfo.pos.longitude,
        .altitude = (float)gpsInfo.pos.altitude,
        .accuracy2D = gpsInfo.error2D,
        .accuracy3D = gpsInfo.error3D,
        .PDOP = gpsInfo.pdop,
        .gps_ms = gpsInfo.timeOfWeek,
        .lastRTCM = gpsInfo.rtcmAge,
        .latHome = (float)gpsInfo.home.latitude,
        .lonHome = (float)gpsInfo.home.longitude,
        .altHome = (float)gpsInfo.home.altitude,
        .downVel = (float)gpsInfo.pos.velocityDown,
        .eastVel = (float)gpsInfo.pos.velocityEast,
        .northVel = (float)gpsInfo.pos.velocityNorth,
        .relX = (float)gpsInfo.xyz.x,
        .relY = (float)gpsInfo.xyz.y,
        .relZ = (float)gpsInfo.xyz.z,
        .YTarget = Y_TARGET,
        .ZTarget = Z_TARGET,
        .ignitionAlt = state.getIgnitionAltitude(),
        .apogeeAlt = state.getApogeeAltitude(),
        .yawSetpoint = state.getAttitudeSetpoint()[0],
        .pitchSetpoint = state.getAttitudeSetpoint()[1],
        .yawCommand = state.getAngularAccelCommand()[0],
        .pitchCommand = state.getAngularAccelCommand()[1],
        .rollMixedYaw = state.getGimbalAngle()[0],
        .rollMixedPitch = state.getGimbalAngle()[1],
        .yawMisalign = state.getGimbalMisalign()[0],
        .pitchMisalign = state.getGimbalMisalign()[1],
        .rollCommand = state.getWheelSpeed(),
        .YProjected = state.getPositionSetpoints()[0],
        .ZProjected = state.getPositionSetpoints()[1],
        .VBAT = vbat, 
        .thrust = state.getThrust(),
        .mass = state.getMass(),
        .MMOI = state.getPitchYawMMOI(),
        .momentArm = state.getMomentArm(),
        .pyroStatus = pyroMask,
        .vehicle_ms = millis(),
        .down_count = downCount
    };
    packet_size = LINK80::packGPSData(gps, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }


    logFile.write(telemetryBuffer, telemetryBufferUsed);
    if(downCount % 10 == 0){ //flush every 200ms if logging at 20ms
        logFile.flush();
    }

}