#include "LINK80.h"


uint8_t LINK80::payloadBuffer[LINK80::MAX_PAYLOAD_SIZE + LINK80::CHECKSUM_SIZE] = {0};
uint8_t LINK80::packetBuffer[LINK80::MAX_PACKET_SIZE + 5] = {0};
uint8_t LINK80::payloadAndChecksumBuffer[LINK80::MAX_PAYLOAD_SIZE + LINK80::CHECKSUM_SIZE] = {0};

// Pack a command acknowledgement
size_t LINK80::packCommandAck(uint8_t command_type, uint8_t command_id, uint8_t error, uint8_t* p, uint32_t vehicle_ms, uint32_t down_count) {
    size_t offset = 0;
    payloadBuffer[offset++]  = 0; // Reserved addressing byte
    payloadBuffer[offset++]  = command_type + 100; // Ack message type
    payloadBuffer[offset++]  = command_id;
    payloadBuffer[offset++]  = error; // Error code

    packUINT32(payloadBuffer, offset, vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, down_count); 

    size_t payload_size = 12;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);
    
    memcpy(p, packetBuffer, a);

    return a;
}

// Pack state telemetry
size_t LINK80::packStateTelemetry(const StateTelemetry& state, uint8_t* p) {
    size_t offset = 0;
    
    payloadBuffer[offset++] = 0; // Reserved addressing
    payloadBuffer[offset++] = STATE_TELEMETRY;
    payloadBuffer[offset++] = state.vehicle_state;
    
    // Pack floats in little endian
    packFloat(payloadBuffer, offset, state.quat_w); offset += 4;
    packFloat(payloadBuffer, offset, state.quat_x); offset += 4;
    packFloat(payloadBuffer, offset, state.quat_y); offset += 4;
    packFloat(payloadBuffer, offset, state.quat_z); offset += 4;
    packFloat(payloadBuffer, offset, state.accel_x); offset += 4;
    packFloat(payloadBuffer, offset, state.accel_y); offset += 4;
    packFloat(payloadBuffer, offset, state.accel_z); offset += 4;
    packFloat(payloadBuffer, offset, state.vel_x); offset += 4;
    packFloat(payloadBuffer, offset, state.vel_y); offset += 4;
    packFloat(payloadBuffer, offset, state.vel_z); offset += 4;
    packFloat(payloadBuffer, offset, state.pos_x); offset += 4;
    packFloat(payloadBuffer, offset, state.pos_y); offset += 4;
    packFloat(payloadBuffer, offset, state.pos_z); offset += 4;
    packFloat(payloadBuffer, offset, state.time_since_launch); offset += 4;
    packUINT32(payloadBuffer, offset, state.vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, state.down_count); 
    
    size_t payload_size = 67;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);

    memcpy(p, packetBuffer, a);

    return a;
}

// Pack sensor data
size_t LINK80::packSensorData(const SensorData& sensors, uint8_t* p) {
    size_t offset = 0;
    
    payloadBuffer[offset++] = 0; // Reserved addressing
    payloadBuffer[offset++] = SENSORS;
    payloadBuffer[offset++] = sensors.failmask;
    payloadBuffer[offset++] = sensors.sd_good ? 1 : 0;
    
    packFloat(payloadBuffer, offset, sensors.gyro_yaw); offset += 4;
    packFloat(payloadBuffer, offset, sensors.gyro_pitch); offset += 4;
    packFloat(payloadBuffer, offset, sensors.gyro_roll); offset += 4;
    packFloat(payloadBuffer, offset, sensors.accel_x); offset += 4;
    packFloat(payloadBuffer, offset, sensors.accel_y); offset += 4;
    packFloat(payloadBuffer, offset, sensors.accel_z); offset += 4;
    packFloat(payloadBuffer, offset, sensors.baro_altitude); offset += 4;
    packFloat(payloadBuffer, offset, sensors.gyro_bias_yaw); offset += 4;
    packFloat(payloadBuffer, offset, sensors.gyro_bias_pitch); offset += 4;
    packFloat(payloadBuffer, offset, sensors.gyro_bias_roll); offset += 4;
    packUINT32(payloadBuffer, offset, sensors.vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, sensors.down_count); 
    
    size_t payload_size = 52;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);
    
    memcpy(p, packetBuffer, a);

    return a;
}

size_t LINK80::packGPSData(const GPSData& gps, uint8_t* p) {
    size_t offset = 0;

    payloadBuffer[offset++] = 0; // Reserved addressing
    payloadBuffer[offset++] = GPS; 
    payloadBuffer[offset++] = gps.satsInView;
    payloadBuffer[offset++] = gps.satsUsed;
    payloadBuffer[offset++] = gps.fixType;

    packFloat(payloadBuffer, offset, gps.latitude); offset += 4;
    packFloat(payloadBuffer, offset, gps.longitude); offset += 4;
    packFloat(payloadBuffer, offset, gps.altitude); offset += 4;
    packFloat(payloadBuffer, offset, gps.accuracy2D); offset += 4;
    packFloat(payloadBuffer, offset, gps.accuracy3D); offset += 4;
    packFloat(payloadBuffer, offset, gps.PDOP); offset += 4;
    packUINT32(payloadBuffer, offset, gps.gps_ms); offset += 4;
    packUINT32(payloadBuffer, offset, gps.lastRTCM); offset += 4;
    packFloat(payloadBuffer, offset, gps.latHome); offset += 4;
    packFloat(payloadBuffer, offset, gps.lonHome); offset += 4;
    packFloat(payloadBuffer, offset, gps.altHome); offset += 4;
    packFloat(payloadBuffer, offset, gps.downVel); offset += 4;
    packFloat(payloadBuffer, offset, gps.eastVel); offset += 4;
    packFloat(payloadBuffer, offset, gps.northVel); offset += 4;
    packFloat(payloadBuffer, offset, gps.relX); offset += 4;
    packFloat(payloadBuffer, offset, gps.relY); offset += 4;
    packFloat(payloadBuffer, offset, gps.relZ); offset += 4;
    packUINT32(payloadBuffer, offset, gps.vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, gps.down_count);

    size_t payload_size = 81;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);
    memcpy(p, packetBuffer, a);
    return a;
}

// Pack Lander (TVC) data
size_t LINK80::packLanderData(const LanderData& lander, uint8_t* p) {
    size_t offset = 0;

    payloadBuffer[offset++] = 0; // Reserved addressing
    payloadBuffer[offset++] = LANDER;

    packFloat(payloadBuffer, offset, lander.YTarget); offset += 4;
    packFloat(payloadBuffer, offset, lander.ZTarget); offset += 4;
    packFloat(payloadBuffer, offset, lander.ignitionAlt); offset += 4;
    packFloat(payloadBuffer, offset, lander.apogeeAlt); offset += 4;
    packFloat(payloadBuffer, offset, lander.yawSetpoint); offset += 4;
    packFloat(payloadBuffer, offset, lander.pitchSetpoint); offset += 4;
    packFloat(payloadBuffer, offset, lander.yawCommand); offset += 4;
    packFloat(payloadBuffer, offset, lander.pitchCommand); offset += 4;
    packFloat(payloadBuffer, offset, lander.rollMixedYaw); offset += 4;
    packFloat(payloadBuffer, offset, lander.rollMixedPitch); offset += 4;
    packFloat(payloadBuffer, offset, lander.yawMisalign); offset += 4;
    packFloat(payloadBuffer, offset, lander.pitchMisalign); offset += 4;
    packFloat(payloadBuffer, offset, lander.rollCommand); offset += 4;
    packFloat(payloadBuffer, offset, lander.YProjected); offset += 4;
    packFloat(payloadBuffer, offset, lander.ZProjected); offset += 4;
    packFloat(payloadBuffer, offset, lander.VBAT); offset += 4;
    packFloat(payloadBuffer, offset, lander.thrust); offset += 4;
    packFloat(payloadBuffer, offset, lander.mass); offset += 4;
    packFloat(payloadBuffer, offset, lander.MMOI); offset += 4;
    packFloat(payloadBuffer, offset, lander.momentArm); offset += 4;
    payloadBuffer[offset++] = lander.pyroStatus;
    packUINT32(payloadBuffer, offset, lander.vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, lander.down_count);

    size_t payload_size = 91;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);
    memcpy(p, packetBuffer, a);
    return a;
}

// Pack Kalman data
size_t LINK80::packKalmanData(const KalmanData& kalman, uint8_t* p) {
    size_t offset = 0;

    payloadBuffer[offset++] = 0; // Reserved addressing
    payloadBuffer[offset++] = KALMAN; 

    packFloat(payloadBuffer, offset, kalman.accUncX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.accUncY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.accUncZ); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velUncX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velUncY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velUncZ); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posUncX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posUncY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posUncZ); offset += 4;
    packFloat(payloadBuffer, offset, kalman.accelMeasuredX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.accelMeasuredY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.accelMeasuredZ); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velMeasuredX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velMeasuredY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.velMeasuredZ); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posMeasuredX); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posMeasuredY); offset += 4;
    packFloat(payloadBuffer, offset, kalman.posMeasuredZ); offset += 4;
    packUINT32(payloadBuffer, offset, kalman.vehicle_ms); offset += 4;
    packUINT32(payloadBuffer, offset, kalman.down_count);

    size_t payload_size = 82;
    size_t a = packPacket(payloadBuffer, payload_size, packetBuffer, MAX_PACKET_SIZE);
    memcpy(p, packetBuffer, a);
    return a;
}

// Unpack received packet (for ground control commands)
LINK80::UnpackedPacket LINK80::unpackPacket(const uint8_t* packet, size_t packet_size) {
    UnpackedPacket result;
    result.valid = false;
    result.data_length = 0;
    result.error = nullptr;
    
    if (packet_size < HEADER_SIZE + CHECKSUM_SIZE) {
        result.error = "Packet too short";
        return result;
    }
    
    if (packet[0] != PACKET_HEADER) {
        result.error = "Invalid packet header";
        return result;
    }
    
    uint8_t payload_length = packet[1];
    uint8_t cobs_offset = packet[2];
    
    if (packet_size != HEADER_SIZE + payload_length) {
        result.error = "Packet size mismatch";
        return result;
    }
    
    // Note: payload_length now includes the checksum
    if (payload_length < CHECKSUM_SIZE + 2) {
        result.error = "Payload too short";
        return result;
    }
    
    // Apply reverse COBS transformation to [payload + checksum] - use global buffer
    memcpy(payloadAndChecksumBuffer, packet + HEADER_SIZE, payload_length);
    
    if (cobs_offset > 0) {
        applyCOBSDecoding(payloadAndChecksumBuffer, payload_length, cobs_offset);
    }
    
    // Split payload and checksum - use global buffer
    size_t actual_payload_size = payload_length - CHECKSUM_SIZE;
    memcpy(payloadBuffer, payloadAndChecksumBuffer, actual_payload_size);
    
    // Verify checksum (CRC32 of original payload)
    uint32_t calculated_crc = calculateCRC32(payloadBuffer, actual_payload_size);
    uint32_t received_crc;
    memcpy(&received_crc, payloadAndChecksumBuffer + actual_payload_size, CHECKSUM_SIZE);
    
    if (calculated_crc != received_crc) {
        result.error = "CRC mismatch";
        return result;
    }
    
    if (actual_payload_size < 2) {
        result.error = "Payload too short for message type";
        return result;
    }
    
    result.valid = true;
    result.message_type = payloadBuffer[1];
    result.data_length = actual_payload_size - 2;
    if (result.data_length > 0) {
        memcpy(result.data, payloadBuffer + 2, result.data_length);
    }
    
    return result;
}

// Parse arm/disarm commands. Returns command ID
uint8_t LINK80::parseCommand(const UnpackedPacket& packet) {
    if (!packet.valid || packet.data_length == 0) {
        return 0; // Return 0 for invalid packets instead of throwing
    }
    return packet.data[0]; // Command ID
}


// Private helper methods implementation

void LINK80::packFloat(uint8_t* buffer, size_t offset, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(float));
    
    // Pack in little endian
    buffer[offset] = bits & 0xFF;
    buffer[offset + 1] = (bits >> 8) & 0xFF;
    buffer[offset + 2] = (bits >> 16) & 0xFF;
    buffer[offset + 3] = (bits >> 24) & 0xFF;
}

float LINK80::unpackFloat(const uint8_t* buffer, size_t offset) {
    uint32_t bits = buffer[offset] | 
                   (buffer[offset + 1] << 8) | 
                   (buffer[offset + 2] << 16) | 
                   (buffer[offset + 3] << 24);
    
    float value;
    memcpy(&value, &bits, sizeof(float));
    return value;
}

void LINK80::packUINT32(uint8_t* buffer, size_t offset, uint32_t value) {
    buffer[offset] = value & 0xFF;
    buffer[offset + 1] = (value >> 8) & 0xFF;
    buffer[offset + 2] = (value >> 16) & 0xFF;
    buffer[offset + 3] = (value >> 24) & 0xFF;
}

uint32_t LINK80::unpackUINT32(const uint8_t* buffer, size_t offset) {
    return (buffer[offset] | 
            (buffer[offset + 1] << 8) | 
            (buffer[offset + 2] << 16) | 
            (buffer[offset + 3] << 24));
}

size_t LINK80::packPacket(const uint8_t* payload, size_t payload_size, uint8_t* buffer, size_t buffer_size) {
    if (payload_size > MAX_PAYLOAD_SIZE) {
        return 0; // Return 0 for error instead of throwing
    }
    
    // Calculate CRC32 of original payload first
    uint32_t crc = calculateCRC32(payload, payload_size);
    
    // Create [payload + checksum] section - use global buffer
    memcpy(payloadAndChecksumBuffer, payload, payload_size);
    
    // Append checksum in little endian
    size_t crc_offset = payload_size;
    payloadAndChecksumBuffer[crc_offset++] = crc & 0xFF;
    payloadAndChecksumBuffer[crc_offset++] = (crc >> 8) & 0xFF;
    payloadAndChecksumBuffer[crc_offset++] = (crc >> 16) & 0xFF;
    payloadAndChecksumBuffer[crc_offset++] = (crc >> 24) & 0xFF;
    
    size_t total_data_size = payload_size + CHECKSUM_SIZE;
    size_t required_size = HEADER_SIZE + total_data_size;
    
    if (buffer_size < required_size) {
        return 0; // Buffer too small
    }
    
    // Find COBS offset (first occurrence of 0xAA in [payload + checksum])
    uint8_t cobs_offset = 0;
    for (size_t i = 0; i < total_data_size; ++i) {
        if (payloadAndChecksumBuffer[i] == PACKET_HEADER) {
            cobs_offset = (uint8_t)i;
            break;
        }
    }
    
    // Apply COBS encoding to [payload + checksum]
    if (cobs_offset > 0) {
        applyCOBSEncoding(payloadAndChecksumBuffer, total_data_size);
    }
    
    // Build final packet
    size_t packet_pos = 0;
    buffer[packet_pos++] = PACKET_HEADER;
    buffer[packet_pos++] = (uint8_t)total_data_size;
    buffer[packet_pos++] = cobs_offset;
    
    // Add COBS-encoded [payload + checksum]
    memcpy(buffer + packet_pos, payloadAndChecksumBuffer, total_data_size);
    packet_pos += total_data_size;
    
    return packet_pos;
}

void LINK80::applyCOBSEncoding(uint8_t* data, size_t data_size) {
    size_t current_pos = 0;
    
    while (current_pos < data_size) {
        if (data[current_pos] == PACKET_HEADER) {
            // Find next 0xAA or end of data
            size_t next_aa = current_pos + 1;
            while (next_aa < data_size && data[next_aa] != PACKET_HEADER) {
                next_aa++;
            }
            
            if (next_aa < data_size) {
                // Replace current 0xAA with offset to next 0xAA
                data[current_pos] = (uint8_t)(next_aa - current_pos);
            } else {
                // No more 0xAA bytes, replace with 0
                data[current_pos] = 0;
            }
        }
        current_pos++;
    }
}

void LINK80::applyCOBSDecoding(uint8_t* data, size_t data_size, uint8_t first_cobs_offset) {
    size_t cobs_pos = first_cobs_offset;
    size_t max_iterations = data_size + 8; // Safety margin
    size_t iterations = 0;
    while (cobs_pos < data_size && iterations < max_iterations) {
        uint8_t offset = data[cobs_pos];
        data[cobs_pos] = PACKET_HEADER; // Restore original 0xAA
        if (offset == 0) {
            break; // No more encoded bytes
        }
        cobs_pos += offset;
        iterations++;
    }
    // Optional: If iterations >= max_iterations, could log an error or set a flag for corrupted data
}

uint32_t LINK80::calculateCRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}