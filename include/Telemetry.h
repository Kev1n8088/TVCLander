#ifndef TELEMETRY_H
#define TELEMETRY_H


#include <Arduino.h>
#include "StateEstimation.h"
#include "Constants.h"
#include "LINK80.h"
#include <SD.h>

#define RX_BUFFER_SIZE LINK80::MAX_PACKET_SIZE * 10

class Telemetry{
private:
    uint64_t lastLogMillis;
    uint64_t lastTelemetryMillis;
    
    uint32_t downCount;
    uint32_t downCountRadio;

    size_t packetBufferLen;

    int currentPacketType; //Increments and used to determine packet type; Order: State, Sensor, Lander, GPS, State, Kalman, Lander
    
    size_t telemetryBufferUsed;
    uint32_t sdCounter;

    String logFileName;

    uint8_t newestRTCMID;

    //int oldVehicleState;
    bool SDGood;


    void dataLog(StateEstimation& state);
    void sendTelemetry(StateEstimation& state);
    void returnAck(uint8_t commandType, uint8_t commandId, uint8_t errorCode);

    void handleReceive(StateEstimation& state);
    void handleRTCM(const LINK80::UnpackedPacket& packet, StateEstimation& state);
    size_t findAndExtractPacket();


    uint32_t crc32(const uint8_t* data, size_t length);

    File logFile;

public:
    Telemetry();
    void begin();
    void telemetryLoop(StateEstimation& state);
    void setLogFileName(const String& name) { logFileName = name; } // Setter for file name
};

#endif // TELEMETRY_H