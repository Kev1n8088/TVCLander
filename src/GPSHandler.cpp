#include "GPSHandler.h"

static uint8_t gpsTXBuffer[16 * 1024];
static uint8_t gpsRXBuffer[16 * 1024];

GPSHandler::GPSHandler() {
    GPS_SERIAL_PORT.begin(GPS_BAUD_RATE); // Initialize GPS serial port with specified baud rate
    GPS_SERIAL_PORT.addMemoryForRead(gpsRXBuffer, sizeof(gpsRXBuffer)); // Allocate memory for reading GPS data
    GPS_SERIAL_PORT.addMemoryForWrite(gpsTXBuffer, sizeof(gpsTXBuffer)); // Allocate memory for writing GPS data
    resetHome();
    homeAverageCount = 0;
    lastRTCMMillis = 0; // Initialize last RTCM correction time 
    // Constructor implementation can be empty if no initialization is needed
}

int GPSHandler::begin() {
    if(!gps.begin(GPS_SERIAL_PORT)) { // Initialize the GPS module
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS initialization failed!");
        }
        return -1; // Return error code if initialization fails
    } else {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS initialized successfully!");
        }
    }
    if(!gps.ensureModeRover()){
        return -2; // Return error code if GPS is not in Rover mode
    } // Ensure GPS is in Rover mode

    busyWait(1);

    if(!gps.setConstellations(true, true, true, true, true, true)){
        //return -3;
    } // Enable all constellations
    
    busyWait(1);

    if(!gps.setFixInterval(100)){
        //return -4; // Return error code if setting fix interval fails
    }
    
    busyWait(1);

    if(!gps.hotStart()){
        //return -5; // Return error code if hot start fails
    } // Perform a hot start to get all checks

    busyWait(2);
    return 0;
}

void GPSHandler::gpsLoop(){
    if(lastUpdateMillis + GPS_POLLING_INTERVAL > millis()){
        //DEBUG_SERIAL.println("Skipping GPS update");
        return; // Skip if not enough time has passed since last update
    }
    lastUpdateMillis = millis(); // Update last update time
    //uint64_t start = millis();
    gps.update(); // Update GPS data
    if(gps.isNewSnapshotAvailable()){
        DRY = true; // Set DRY flag to indicate new data is available   
        if (DEBUG_MODE) {
            //DEBUG_SERIAL.println("New GPS data available");
        }
        current.latitude = gps.getLatitude(); // Get current latitude
        current.longitude = gps.getLongitude(); // Get current longitude
        current.altitude = gps.getAltitude(); // Get current altitude
        current.velocityNorth = gps.getNorthVelocity(); // Get current velocity in North direction
        current.velocityEast = gps.getEastVelocity(); // Get current velocity in East direction
        current.velocityDown = gps.getDownVelocity(); // Get current velocity in Down direction

        gpsInfo.fixType = gps.getFixQuality(); // Get GPS fix type=
        gpsInfo.home = home; // Set home position
        gpsInfo.pos = current; // Set current position

        gpsInfo.xyz = getDistance(home, current); // Calculate distance from home position
        gpsInfo.satsInView = gps.getSatellitesInViewCount(); // Get number of satellites in view
        gpsInfo.satsUsed = gps.getSatellitesUsedCount(); // Get number of satellites used for fix

        gpsInfo.pdop = gps.getPdop(); // Get PDOP value
        gpsInfo.timeOfWeek = gps.getTimeOfWeek(); // Get time of week in milliseconds

        gpsInfo.error2D = gps.get2DError(); // Get 2D error
        gpsInfo.error3D = gps.get3DError(); // Get 3D error

        gpsInfo.rtcmAge = millis() - lastRTCMMillis; // Calculate age of RTCM correction data
    }   

    // DEBUG_SERIAL.print("GPS update took ");
    // DEBUG_SERIAL.print(millis() - start);
    // DEBUG_SERIAL.println(" ms");

}


void GPSHandler::setCurrentAsHome() {
    if (current.latitude == 0.0f && current.longitude == 0.0f && current.altitude == 0.0f) {
        if (DEBUG_MODE) {
            //DEBUG_SERIAL.println(gps.getLatitude());
            DEBUG_SERIAL.println("Current GPS position is invalid, cannot set as home");
        }
        return; // Do not set home if current position is invalid
    }
    if(gps.getFixQuality() < 4){
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS fix quality is insufficient to set home position");
        }
        return; // Do not set home if GPS fix quality is insufficient
    }
    homeAverageCount++;

    float alpha = 1.0f / homeAverageCount;

    home.latitude      = (1 - alpha) * home.latitude   + alpha * current.latitude;
    home.longitude     = (1 - alpha) * home.longitude + alpha * current.longitude;
    home.altitude      = (1 - alpha) * home.altitude  + alpha * current.altitude;

    if (DEBUG_MODE) {
        DEBUG_SERIAL.println("Home position averaged with current GPS position");
    }
}

void GPSHandler::resetHome() {
    homeAverageCount = 0; // Reset home average count
    home.latitude = 0.0f; // Reset home latitude
    home.longitude = 0.0f; // Reset home longitude
    home.altitude = 0.0f; // Reset home altitude

    if (DEBUG_MODE) {
        DEBUG_SERIAL.println("Home position reset");
    }
}

bool GPSHandler::dataReady(){
    if (DRY){
        DRY = false; // Reset DRY flag
        return true;
    }
    return false;
}

/**
 * @brief Busy wait for a specified number of seconds while updating GPS data.
 */
void GPSHandler::busyWait(int seconds) {
    uint64_t start = millis();
    while (millis() - start < seconds * 1000) {
        gps.update();
        // Busy wait for the specified number of seconds
    }
}

void GPSHandler::sendRTCMCorrection(const uint8_t* data, size_t length) {
    if (DEBUG_MODE) {
        DEBUG_SERIAL.println("Sending RTCM correction data");
    }

    GPS_SERIAL_PORT.write(data, length); // Send RTCM correction data over GPS serial port
    lastRTCMMillis = millis(); // Update last RTCM correction time
}

XYZ GPSHandler::getDistance(positionAndVelocity pos1, positionAndVelocity pos2){
    XYZ result;

    const double EARTH_RADIUS = 6371000.0;

    double lat1_rad = pos1.latitude * DEG_TO_RAD;
    double lat2_rad = pos2.latitude * DEG_TO_RAD;
    double lon1_rad = pos1.longitude * DEG_TO_RAD;
    double lon2_rad = pos2.longitude * DEG_TO_RAD;

    double delta_lat = lat2_rad - lat1_rad;
    double delta_lon = lon2_rad - lon1_rad;
    double avg_lat = (lat1_rad + lat2_rad) / 2.0;

    // Vincenty's formulae for short distances
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double distance = EARTH_RADIUS * c;

    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    double bearing = atan2(y, x);

    result.x = pos2.altitude - pos1.altitude; // Altitude difference
    result.y = distance * sin(bearing); // east component
    result.z = distance * cos(bearing); // north component

    return result;
}