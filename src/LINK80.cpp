#include <Arduino.h>
#include <stdint.h>
#include <string.h>

class LINK80 {
public:
    static constexpr uint8_t    PACKET_HEADER = 0xAA;
    static constexpr uint32_t CRC_POLYNOMIAL = 0xEDB88320;
    static constexpr size_t MAX_PACKET_SIZE = 255;
    static constexpr size_t MAX_PAYLOAD_SIZE = 247;
    static constexpr size_t HEADER_SIZE = 3;
    static constexpr size_t CHECKSUM_SIZE = 4;
    
    // Message types for vehicle -> ground control
    enum MessageType {
        // Command acknowledgements (command type + 50)
        ARM_ACK = 61,
        DISARM_ACK = 62,
        
        // Vehicle -> Ground data
        STATE_TELEMETRY = 155,
        SENSORS = 156
    };
    
    // Vehicle state structure for telemetry
    struct StateTelemetry {
        int8_t vehicle_state;
        float quat_w, quat_x, quat_y, quat_z;
        float accel_x, accel_y, accel_z;
        float vel_x, vel_y, vel_z;
        float pos_x, pos_y, pos_z;
        float time_since_launch;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };
    
    // Sensor data structure
    struct SensorData {
        int8_t failmask;
        bool sd_good;
        float gyro_yaw, gyro_pitch, gyro_roll;
        float accel_x, accel_y, accel_z;
        float baro_altitude;
        float gyro_bias_yaw, gyro_bias_pitch, gyro_bias_roll;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };
    
    // Pack a command acknowledgement
    static size_t packCommandAck(uint8_t command_type, uint8_t command_id, uint8_t* p, uint32_t vehicle_ms, uint32_t down_count) {
        uint8_t payload[11];
        size_t offset = 0;
        payload[offset++]  = 0; // Reserved addressing byte
        payload[offset++]  = command_type + 50; // Ack message type
        payload[offset++]  = command_id;

        packUINT32(payload, offset, vehicle_ms); offset += 4;
        packUINT32(payload, offset, down_count); 

        uint8_t packet[sizeof(payload) + HEADER_SIZE + CHECKSUM_SIZE];
        size_t a = packPacket(payload, sizeof(payload), packet, sizeof(packet));
        
        memcpy(p, packet, a);

        return a;
    }
    
    // Pack state telemetry
    static size_t packStateTelemetry(const StateTelemetry& state, uint8_t* p) {
        uint8_t payload[67]; 
        size_t offset = 0;
        
        payload[offset++] = 0; // Reserved addressing
        payload[offset++] = STATE_TELEMETRY;
        payload[offset++] = state.vehicle_state;
        
        // Pack floats in little endian
        packFloat(payload, offset, state.quat_w); offset += 4;
        packFloat(payload, offset, state.quat_x); offset += 4;
        packFloat(payload, offset, state.quat_y); offset += 4;
        packFloat(payload, offset, state.quat_z); offset += 4;
        packFloat(payload, offset, state.accel_x); offset += 4;
        packFloat(payload, offset, state.accel_y); offset += 4;
        packFloat(payload, offset, state.accel_z); offset += 4;
        packFloat(payload, offset, state.vel_x); offset += 4;
        packFloat(payload, offset, state.vel_y); offset += 4;
        packFloat(payload, offset, state.vel_z); offset += 4;
        packFloat(payload, offset, state.pos_x); offset += 4;
        packFloat(payload, offset, state.pos_y); offset += 4;
        packFloat(payload, offset, state.pos_z); offset += 4;
        packFloat(payload, offset, state.time_since_launch); offset += 4;
        packUINT32(payload, offset, state.vehicle_ms); offset += 4;
        packUINT32(payload, offset, state.down_count); 
        
        uint8_t packet[sizeof(payload) + HEADER_SIZE + CHECKSUM_SIZE];
        size_t a = packPacket(payload, sizeof(payload), packet, sizeof(packet));

        memcpy(p, packet, a);

        return a;
    }
    
    // Pack sensor data
    static size_t packSensorData(const SensorData& sensors, uint8_t* p) {
        uint8_t payload[52];
        size_t offset = 0;
        
        payload[offset++] = 0; // Reserved addressing
        payload[offset++] = SENSORS;
        payload[offset++] = sensors.failmask;
        payload[offset++] = sensors.sd_good ? 1 : 0;
        
        packFloat(payload, offset, sensors.gyro_yaw); offset += 4;
        packFloat(payload, offset, sensors.gyro_pitch); offset += 4;
        packFloat(payload, offset, sensors.gyro_roll); offset += 4;
        packFloat(payload, offset, sensors.accel_x); offset += 4;
        packFloat(payload, offset, sensors.accel_y); offset += 4;
        packFloat(payload, offset, sensors.accel_z); offset += 4;
        packFloat(payload, offset, sensors.baro_altitude); offset += 4;
        packFloat(payload, offset, sensors.gyro_bias_yaw); offset += 4;
        packFloat(payload, offset, sensors.gyro_bias_pitch); offset += 4;
        packFloat(payload, offset, sensors.gyro_bias_roll); offset += 4;
        packUINT32(payload, offset, sensors.vehicle_ms); offset += 4;
        packUINT32(payload, offset, sensors.down_count); 
        
        
        uint8_t packet[sizeof(payload) + HEADER_SIZE + CHECKSUM_SIZE];
        size_t a = packPacket(payload, sizeof(payload), packet, sizeof(packet));
        
        memcpy(p, packet, a);

        return a;
    }
    
    // Unpack received packet (for ground control commands)
    struct UnpackedPacket {
        bool valid;
        uint8_t message_type;
        uint8_t data[MAX_PAYLOAD_SIZE - 2]; // Max data size minus addressing and message type
        size_t data_length;
        const char* error;
    };
    
    static UnpackedPacket unpackPacket(const uint8_t* packet, size_t packet_size) {
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
        
        // Apply reverse COBS transformation to [payload + checksum]
        uint8_t payload_and_checksum[MAX_PAYLOAD_SIZE + CHECKSUM_SIZE];
        memcpy(payload_and_checksum, packet + HEADER_SIZE, payload_length);
        
        if (cobs_offset > 0) {
            applyCOBSDecoding(payload_and_checksum, payload_length, cobs_offset);
        }
        
        // Split payload and checksum
        size_t actual_payload_size = payload_length - CHECKSUM_SIZE;
        uint8_t payload[MAX_PAYLOAD_SIZE];
        memcpy(payload, payload_and_checksum, actual_payload_size);
        
        // Verify checksum (CRC32 of original payload)
        uint32_t calculated_crc = calculateCRC32(payload, actual_payload_size);
        uint32_t received_crc;
        memcpy(&received_crc, payload_and_checksum + actual_payload_size, CHECKSUM_SIZE);
        
        if (calculated_crc != received_crc) {
            result.error = "CRC mismatch";
            return result;
        }
        
        if (actual_payload_size < 2) {
            result.error = "Payload too short for message type";
            return result;
        }
        
        result.valid = true;
        result.message_type = payload[1];
        result.data_length = actual_payload_size - 2;
        if (result.data_length > 0) {
            memcpy(result.data, payload + 2, result.data_length);
        }
        
        return result;
    }
    
    // up count not used for now, could be used for tracking dropped packets
    // Parse arm/disarm commands
    static uint8_t parseCommand(const UnpackedPacket& packet) {
        if (!packet.valid || packet.data_length == 0) {
            return 0; // Return 0 for invalid packets instead of throwing
        }
        return packet.data[0]; // Command ID
    }

private:
    static void packFloat(uint8_t* buffer, size_t offset, float value) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(float));
        
        // Pack in little endian
        buffer[offset] = bits & 0xFF;
        buffer[offset + 1] = (bits >> 8) & 0xFF;
        buffer[offset + 2] = (bits >> 16) & 0xFF;
        buffer[offset + 3] = (bits >> 24) & 0xFF;
    }
    
    static float unpackFloat(const uint8_t* buffer, size_t offset) {
        uint32_t bits = buffer[offset] | 
                       (buffer[offset + 1] << 8) | 
                       (buffer[offset + 2] << 16) | 
                       (buffer[offset + 3] << 24);
        
        float value;
        memcpy(&value, &bits, sizeof(float));
        return value;
    }

    static float packUINT32(uint8_t* buffer, size_t offset, uint32_t value) {
        buffer[offset] = value & 0xFF;
        buffer[offset + 1] = (value >> 8) & 0xFF;
        buffer[offset + 2] = (value >> 16) & 0xFF;
        buffer[offset + 3] = (value >> 24) & 0xFF;
    }

    static uint32_t unpackUINT32(const uint8_t* buffer, size_t offset) {
        return (buffer[offset] | 
                (buffer[offset + 1] << 8) | 
                (buffer[offset + 2] << 16) | 
                (buffer[offset + 3] << 24));
    }
    
    static size_t packPacket(const uint8_t* payload, size_t payload_size, uint8_t* buffer, size_t buffer_size) {
        if (payload_size > MAX_PAYLOAD_SIZE) {
            return 0; // Return 0 for error instead of throwing
        }
        
        // Calculate CRC32 of original payload first
        uint32_t crc = calculateCRC32(payload, payload_size);
        
        // Create [payload + checksum] section
        uint8_t payload_and_checksum[MAX_PAYLOAD_SIZE + CHECKSUM_SIZE];
        memcpy(payload_and_checksum, payload, payload_size);
        
        // Append checksum in little endian
        size_t crc_offset = payload_size;
        payload_and_checksum[crc_offset++] = crc & 0xFF;
        payload_and_checksum[crc_offset++] = (crc >> 8) & 0xFF;
        payload_and_checksum[crc_offset++] = (crc >> 16) & 0xFF;
        payload_and_checksum[crc_offset++] = (crc >> 24) & 0xFF;
        
        size_t total_data_size = payload_size + CHECKSUM_SIZE;
        size_t required_size = HEADER_SIZE + total_data_size;
        
        if (buffer_size < required_size) {
            return 0; // Buffer too small
        }
        
        // Find COBS offset (first occurrence of 0xAA in [payload + checksum])
        uint8_t cobs_offset = 0;
        for (size_t i = 0; i < total_data_size; ++i) {
            if (payload_and_checksum[i] == PACKET_HEADER) {
                cobs_offset = (uint8_t)i;
                break;
            }
        }
        
        // Apply COBS encoding to [payload + checksum]
        if (cobs_offset > 0) {
            applyCOBSEncoding(payload_and_checksum, total_data_size);
        }
        
        // Build final packet
        size_t packet_pos = 0;
        buffer[packet_pos++] = PACKET_HEADER;
        buffer[packet_pos++] = (uint8_t)total_data_size;
        buffer[packet_pos++] = cobs_offset;
        
        // Add COBS-encoded [payload + checksum]
        memcpy(buffer + packet_pos, payload_and_checksum, total_data_size);
        packet_pos += total_data_size;
        
        return packet_pos;
    }
    
    static void applyCOBSEncoding(uint8_t* data, size_t data_size) {
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
    
    static void applyCOBSDecoding(uint8_t* data, size_t data_size, uint8_t first_cobs_offset) {
        size_t cobs_pos = first_cobs_offset;
        
        while (cobs_pos < data_size) {
            uint8_t offset = data[cobs_pos];
            data[cobs_pos] = PACKET_HEADER; // Restore original 0xAA
            
            if (offset == 0) {
                break; // No more encoded bytes
            }
            
            cobs_pos += offset;
        }
    }
    
    static uint32_t calculateCRC32(const uint8_t* data, size_t length) {
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
};