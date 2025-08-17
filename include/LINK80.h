#ifndef LINK80_H
#define LINK80_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "Constants.h"

class LINK80 {
public:
    static constexpr uint8_t    PACKET_HEADER = 0xAA;
    static constexpr uint32_t CRC_POLYNOMIAL = 0xEDB88320;
    static constexpr size_t MAX_PACKET_SIZE = 252;
    static constexpr size_t MAX_PAYLOAD_SIZE = 245 ; // 252 - 3 header bytes - 4 checksum bytes
    static constexpr size_t HEADER_SIZE = 3;
    static constexpr size_t CHECKSUM_SIZE = 4;
    
    // Message types for vehicle -> ground control
    enum MessageType {
        PING = 10,
        ARM = 11,
        DISARM = 62,
        
        // Vehicle -> Ground data
        STATE_TELEMETRY = 155,
        SENSORS = 156,
        GPS = 157,
        LANDER = 158,
        KALMAN = 159
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
        uint8_t failmask;
        bool sd_good;
        float gyro_yaw, gyro_pitch, gyro_roll;
        float accel_x, accel_y, accel_z;
        float baro_altitude;
        float gyro_bias_yaw, gyro_bias_pitch, gyro_bias_roll;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };

    //GPS telemetry
    struct GPSData{
        uint8_t satsInView;
        uint8_t satsUsed;
        uint8_t fixType;
        float latitude;
        float longitude;
        float altitude;
        float accuracy2D;
        float accuracy3D;
        float PDOP;
        uint32_t gps_ms;
        uint32_t lastRTCM;
        float latHome;
        float lonHome;
        float altHome;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };

    //TVC specific lander telemetry
    struct LanderData{
        float YTarget;
        float ZTarget;
        float ignitionAlt;
        float apogeeAlt;
        float yawSetpoint;
        float pitchSetpoint;
        float yawCommand;
        float pitchCommand;
        float rollMixedYaw;
        float rollMixedPitch;
        float yawMisalign;
        float pitchMisalign;
        float rollCommand; 
        float YProjected;
        float ZProjected;
        float VBAT;
        float thrust;
        float mass;
        float MMOI;
        float momentArm;
        uint8_t pyroStatus;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };

    //Kalman filter telemerty
    struct KalmanData{
        float accUncX;
        float accUncY;
        float accUncZ;
        float velUncX;
        float velUncY;
        float velUncZ;
        float posUncX;
        float posUncY;
        float posUncZ;
        float accelMeasuredX;
        float accelMeasuredY;
        float accelMeasuredZ;
        float velMeasuredX;
        float velMeasuredY;
        float velMeasuredZ;
        float posMeasuredX;
        float posMeasuredY;
        float posMeasuredZ;
        uint32_t vehicle_ms;
        uint32_t down_count;
    };
    
    // Unpack received packet (for ground control commands)
    struct UnpackedPacket {
        bool valid;
        uint8_t message_type;
        uint8_t data[MAX_PAYLOAD_SIZE - 2]; // Max data size minus addressing and message type
        size_t data_length;
        const char* error;
    };
    
    // Public methods
    static size_t packCommandAck(uint8_t command_type, uint8_t command_id, uint8_t error, uint8_t* p, uint32_t vehicle_ms, uint32_t down_count);
    static size_t packStateTelemetry(const StateTelemetry& state, uint8_t* p);
    static size_t packSensorData(const SensorData& sensors, uint8_t* p);
    static size_t packGPSData(const GPSData& gps, uint8_t* p);
    static size_t packLanderData(const LanderData& lander, uint8_t* p);
    static size_t packKalmanData(const KalmanData& kalman, uint8_t* p);
    static UnpackedPacket unpackPacket(const uint8_t* packet, size_t packet_size);
    static uint8_t parseCommand(const UnpackedPacket& packet);

private:
    // Private helper methods
    static void packFloat(uint8_t* buffer, size_t offset, float value);
    static float unpackFloat(const uint8_t* buffer, size_t offset);
    static void packUINT32(uint8_t* buffer, size_t offset, uint32_t value);
    static uint32_t unpackUINT32(const uint8_t* buffer, size_t offset);
    static size_t packPacket(const uint8_t* payload, size_t payload_size, uint8_t* buffer, size_t buffer_size);
    static void applyCOBSEncoding(uint8_t* data, size_t data_size);
    static void applyCOBSDecoding(uint8_t* data, size_t data_size, uint8_t first_cobs_offset);
    static uint32_t calculateCRC32(const uint8_t* data, size_t length);

    // Static buffers for packing
    static uint8_t payloadBuffer[MAX_PAYLOAD_SIZE + CHECKSUM_SIZE];
    static uint8_t packetBuffer[MAX_PACKET_SIZE + 5];
    static uint8_t payloadAndChecksumBuffer[MAX_PAYLOAD_SIZE + CHECKSUM_SIZE];
};

#endif // LINK80_H