#include <SD.h>
#include <Telemetry.h>
#include <Arduino.h>
#include <Constants.h>
#include "StateEstimation.h"

EXTMEM uint8_t telemetryBuffer[MAX_DATA_LOGS * MAX_BYTES_PER_LOG];
static uint8_t serialBuffer[32 * 1024];

static uint8_t telemetryPacketBuffer[LINK80::MAX_PACKET_SIZE];

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

static RTCMBuffer rtcmBuffers[128]; // RTCM ID is uint8_t

static uint8_t receiveBuffer[600]; // Buffer for received packets

/**
 * @brief Telemetry constructor. Initializes member variables.
 */
Telemetry::Telemetry(){
    lastLogMillis = 0;
    lastTelemetryMillis = 0;
    oldVehicleState = 0;
    telemetryBufferUsed = 0;
    SDGood = false;
    logFileName = "flightlog.bin";
    downCount = 0;
    packetBufferLen = 0;
    currentPacketType = 0;
}

/**
 * @brief Initializes telemetry serial, SD card, and debug serial.
 */
void Telemetry::begin(){
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
    
    TELEMETRY_SERIAL.addMemoryForWrite(serialBuffer, sizeof(serialBuffer));

}

/**
 * @brief Main telemetry loop. Handles logging and sending telemetry data.
 * @param state Reference to StateEstimation object for data access.
 */
void Telemetry::telemetryLoop(StateEstimation& state){
    uint64_t currentMillis = millis();

    handleReceive(state);

    if(state.getVehicleState() > 1 && state.getVehicleState() < 7) {
        if (currentMillis - lastLogMillis >= DATALOG_INTERVAL) {
            lastLogMillis = currentMillis;
            dataLog(state);
        }
    }

    // Check for vehicle state changes that require saving data
    if (state.getVehicleState() != oldVehicleState) {
        oldVehicleState = state.getVehicleState();
        if (state.getVehicleState() == 0 || state.getVehicleState() == 7) {
            // Save data and reset buffer when disarmed (0) or landed (7)
            dumpToSD();
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

    downCount = (downCount + 1) % UINT16_MAX; // Increment downCount and wrap around at 2^32

    //DEBUG_SERIAL.println(downCount);

    currentPacketType = (currentPacketType + 1) % 7;
    //currentPacketType = 2; //testing

    size_t packet_size = 0;

    LINK80::StateTelemetry stateTelem;
    LINK80::SensorData sensors;
    LINK80::LanderData lander;
    LINK80::GPSData gps;
    LINK80::KalmanData kalman;
    GPSInfo& gpsInfo = state.getGPSInfo();

    float vbat = (((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER;


    uint8_t pyroMask = (state.getChuteCont() ? (1 << 0) : 0) | (state.getPyroCont() ? (1 << 1) : 0); // Create a bitmask for pyro status. First bit is chute, second bit is landing motor
    switch (currentPacketType){
        case 0:
        case 4: //state telem packet
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
                .vehicle_ms = millis(),
                .down_count = downCount
            };
            break;
        case 1: // sensor telem packet
            sensors = {
                .failmask = state.getSensorStatus(),
                .sd_good = SDGood,
                .gyro_yaw = state.getGyroRemovedBias()[0],
                .gyro_pitch = state.getGyroRemovedBias()[1],
                .gyro_roll = state.getGyroRemovedBias()[2],
                .accel_x = state.getAccelCalibrated()[0],
                .accel_y = state.getAccelCalibrated()[1],
                .accel_z = state.getAccelCalibrated()[2],
                .baro_altitude = state.getGPSHandler().getUpdateInterval(), // zero for now since barometer is not used
                .gyro_bias_yaw = state.getGyroBias()[0],
                .gyro_bias_pitch = state.getGyroBias()[1],
                .gyro_bias_roll = state.getGyroBias()[2],
                .vehicle_ms = millis(),
                .down_count = downCount
            };
            break;
        case 2: 
        case 6: // lander telem packet
            lander = {
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
            break;
        case 3:
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
                .vehicle_ms = millis(),
                .down_count = downCount
            };
            break;
        case 5: // Kalman telem packet
            kalman = {
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
            break;
    };

    // Use static buffer for packing
    if(currentPacketType == 0 || currentPacketType == 4){
        packet_size = LINK80::packStateTelemetry(stateTelem, telemetryPacketBuffer);
    }else if(currentPacketType == 1){
        packet_size = LINK80::packSensorData(sensors, telemetryPacketBuffer);
    }else if(currentPacketType == 2 || currentPacketType == 6){
        packet_size = LINK80::packLanderData(lander, telemetryPacketBuffer);
    }else if(currentPacketType == 3){
        packet_size = LINK80::packGPSData(gps, telemetryPacketBuffer);
    }else if(currentPacketType == 5){
        packet_size = LINK80::packKalmanData(kalman, telemetryPacketBuffer);
    }

    if (packet_size == 0) {
        // Packing failed
        DEBUG_SERIAL.println("Failed to pack telemetry data");
        return;
    }

    TELEMETRY_SERIAL.write(telemetryPacketBuffer, packet_size);

    // if (DEBUG_MODE){
    //     DEBUG_SERIAL.write(telemetryPacketBuffer, packet_size);
    //     //DEBUG_SERIAL.println((((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER); // Send battery voltage
    // }
}

void Telemetry::returnAck(uint8_t messageType, uint8_t commandID, uint8_t errorCode) {
    
    downCount = (downCount + 1) % 4294967296; // Increment downCount and wrap around at 2^32

    size_t packet_size = LINK80::packCommandAck(messageType, commandID, errorCode, telemetryPacketBuffer, millis(), downCount);
    if (packet_size > 0) {
        TELEMETRY_SERIAL.write(telemetryPacketBuffer, packet_size);
        // if (DEBUG_MODE){
        //     DEBUG_SERIAL.write(telemetryPacketBuffer, packet_size);
        // }
    }
}

void Telemetry::handleReceive(StateEstimation& state) {
    while (TELEMETRY_SERIAL.available() > 0 && packetBufferLen < RX_BUFFER_SIZE) {
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
                }
            }
            if (unpacked.message_type >= 51 && unpacked.message_type <= 55) {
                handleRTCM(unpacked, state);
            }
            // Remove processed packet from buffer
            memmove(receiveBuffer, receiveBuffer + packetSize, packetBufferLen - packetSize);
            packetBufferLen -= packetSize;
        } else {
            break; // No complete packet found
        }
    }
}

size_t Telemetry::findAndExtractPacket() {
    // Look for packet header
    size_t headerIndex = SIZE_MAX;
    for (size_t i = 0; i < packetBufferLen; ++i) {
        if (receiveBuffer[i] == LINK80::PACKET_HEADER) {
            headerIndex = i;
            break;
        }
    }
    if (headerIndex == SIZE_MAX) {
        // No header found, clear buffer
        packetBufferLen = 0;
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
    size_t expectedPacketSize = LINK80::HEADER_SIZE + payloadLength;

    // Check if we have the complete packet
    if (packetBufferLen < expectedPacketSize) {
        return 0;
    }

    // Return the size of the complete packet
    return expectedPacketSize;
}



void Telemetry::handleRTCM(const LINK80::UnpackedPacket& packet, StateEstimation& state) {
    if (!packet.valid || packet.message_type < 51 || packet.message_type > 55) return;



    uint8_t rtcm_id = packet.data[0];
    if (rtcm_id >= 128) {
        // Invalid RTCM ID, ignore packet
        return;
    }

    size_t fragment_len = packet.data_length - 1; // Exclude RTCM ID byte

    RTCMBuffer& buf = rtcmBuffers[rtcm_id];
    if (!buf.active) {
        buf.length = 0;
        buf.active = true;
    }

    // Append fragment
    if (buf.length + fragment_len <= sizeof(buf.data)) {
        memcpy(buf.data + buf.length, packet.data + 1, fragment_len);
        buf.length += fragment_len;
    }

    buf.lastUpdateMillis = millis();

    // If message_type == 55, packet is complete
    if (packet.message_type == 55) {
        state.getGPSHandler().sendRTCMCorrection(buf.data, buf.length);
        buf.active = false;
        buf.length = 0;
        buf.lastUpdateMillis = 0;
    }

    for (int i = 0; i < 128; ++i) {
        if (rtcmBuffers[i].active && millis() - rtcmBuffers[i].lastUpdateMillis > 3000) { // 1 second timeout
            rtcmBuffers[i].active = false;
            rtcmBuffers[i].length = 0;
            rtcmBuffers[i].lastUpdateMillis = 0;
        }
    }
}

/**
 * @brief Logs telemetry data to PSRAM buffer in binary format.
 */
void Telemetry::dataLog(StateEstimation& state) {
    // Reset buffer if vehicle state is disarmed
    if (state.getVehicleState() == 0) {
        telemetryBufferUsed = 0;
        return;
    }

    downCount = (downCount + 1) % UINT16_MAX; // Increment downCount and wrap around

    size_t packet_size = 0;
    uint8_t tempBuffer[LINK80::MAX_PACKET_SIZE]; // Temporary buffer for packing

    LINK80::StateTelemetry stateTelem;
    LINK80::SensorData sensors;
    LINK80::LanderData lander;
    LINK80::GPSData gps;
    LINK80::KalmanData kalman;
    GPSInfo& gpsInfo = state.getGPSInfo();

    float vbat = (((float)analogRead(VBAT_SENSE_PIN)) / 1023.0f) * VBAT_DIVIDER;

    // Check if we have enough space for all packet types (rough estimate)
    if (telemetryBufferUsed + (5 * LINK80::MAX_PACKET_SIZE) > sizeof(telemetryBuffer)) {
        return; // Not enough space
    }

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
        .vehicle_ms = millis(),
        .down_count = downCount
    };
    packet_size = LINK80::packStateTelemetry(stateTelem, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }

    // 2. Sensor data packet
    sensors = {
        .failmask = state.getSensorStatus(),
        .sd_good = SDGood,
        .gyro_yaw = state.getGyroRemovedBias()[0],
        .gyro_pitch = state.getGyroRemovedBias()[1],
        .gyro_roll = state.getGyroRemovedBias()[2],
        .accel_x = state.getAccelCalibrated()[0],
        .accel_y = state.getAccelCalibrated()[1],
        .accel_z = state.getAccelCalibrated()[2],
        .baro_altitude = state.getGPSHandler().getUpdateInterval(),
        .gyro_bias_yaw = state.getGyroBias()[0],
        .gyro_bias_pitch = state.getGyroBias()[1],
        .gyro_bias_roll = state.getGyroBias()[2],
        .vehicle_ms = millis(),
        .down_count = downCount
    };
    packet_size = LINK80::packSensorData(sensors, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }

    uint8_t pyroMask = (state.getChuteCont() ? (1 << 0) : 0) | (state.getPyroCont() ? (1 << 1) : 0); // Create a bitmask for pyro status. First bit is chute, second bit is landing motor

    // 3. Lander data packet
    lander = {
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
    packet_size = LINK80::packLanderData(lander, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }

    // 4. GPS data packet
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
        .vehicle_ms = millis(),
        .down_count = downCount
    };
    packet_size = LINK80::packGPSData(gps, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }

    // 5. Kalman data packet
    kalman = {
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
    packet_size = LINK80::packKalmanData(kalman, tempBuffer);
    if (packet_size > 0 && telemetryBufferUsed + packet_size < sizeof(telemetryBuffer)) {
        memcpy(telemetryBuffer + telemetryBufferUsed, tempBuffer, packet_size);
        telemetryBufferUsed += packet_size;
    }
}

/**
 * @brief Dumps logged telemetry data from PSRAM to SD card as binary file with unique filename.
 */
void Telemetry::dumpToSD(){
    if (telemetryBufferUsed == 0) return;

    // Generate unique filename by finding the next available number
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

    File logFile = SD.open(filename.c_str(), FILE_WRITE);
    if (logFile) {
        logFile.write(telemetryBuffer, telemetryBufferUsed);
        logFile.close();
        
        if (DEBUG_MODE) {
            DEBUG_SERIAL.print("Telemetry data saved to: ");
            DEBUG_SERIAL.println(filename);
        }
    } else {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.print("Failed to create log file: ");
            DEBUG_SERIAL.println(filename);
        }
    }

    // Clear buffer for next use
    telemetryBufferUsed = 0;
}