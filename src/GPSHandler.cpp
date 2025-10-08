#include "GPSHandler.h"

SFE_UBLOX_GNSS gps; 

GPSHandler::GPSHandler() {
    lastRTCMMillis = 0; // Initialize last RTCM correction time 
    lastUpdateTime = 0;
    updateInterval = 100;
    // Constructor implementation can be empty if no initialization is needed
}

int GPSHandler::begin() {
    Wire.begin(); // Initialize I2C
    resetHome();

    delay(3000); // Wait for GPS module to power up

    // DEBUG_SERIAL.begin(DEBUG_BAUD); // Initialize debug serial port with specified baud rate
    //   while (!DEBUG_SERIAL) {
    //     delay(1); // Wait for serial port to connect. Needed for native USB port only
    //   }
    //   DEBUG_SERIAL.println("Debug mode enabled");


    // gps.enableDebugging(DEBUG_SERIAL, false); // Enable or disable debugging based on DEBUG_MODE
    if(!gps.begin(Wire)) { // Initialize the GPS module
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS initialization failed!");
        }
        return -1; // Return error code if initialization fails
    } else {
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS initialized successfully!");
        }
    }

    gps.setNavigationFrequency(12); 
    gps.setAutoPVT(true);
    gps.setAutoHPPOSLLH(true);
    gps.setI2COutput(COM_TYPE_UBX);
    gps.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3, VAL_LAYER_RAM_BBR);
    gps.saveConfiguration(); 

    return 0;
}

void GPSHandler::gpsLoop(){
    if(lastUpdateMillis + GPS_POLLING_INTERVAL > millis()){
        //DEBUG_SERIAL.println("Skipping GPS update");
        return; // Skip if not enough time has passed since last update
    }
    lastUpdateMillis = millis(); // Update last update time
    if (gps.getHPPOSLLH(5)){
        if(gpsInfo.timeOfWeek == gps.getTimeOfWeek(5)){
            // No new data
            return;
        }

        updateInterval = millis() - lastUpdateTime; // logs update interval
        lastUpdateTime = millis();

        DRY = true; // Set DRY flag to indicate new data is available
        if (DEBUG_MODE) {
            //DEBUG_SERIAL.println("New GPS data available");
        }
        
        // Get high-resolution latitude (combine base + high-precision components)
        int32_t lat_base = gps.getHighResLatitude(5);  // degrees * 10^-7
        int8_t lat_hp = gps.getHighResLatitudeHp(5);   // degrees * 10^-9
        current.latitude = (double)lat_base * 1e-7 + (double)lat_hp * 1e-9;
        
        // Get high-resolution longitude (combine base + high-precision components)
        int32_t lon_base = gps.getHighResLongitude(5); // degrees * 10^-7
        int8_t lon_hp = gps.getHighResLongitudeHp(5);  // degrees * 10^-9
        current.longitude = (double)lon_base * 1e-7 + (double)lon_hp * 1e-9;
        
        current.altitude = gps.getAltitude(5) * 1e-3; // Get current altitude
        current.velocityNorth = gps.getNedNorthVel(5) * 1e-3; // Get current velocity in North direction
        current.velocityEast = gps.getNedEastVel(5) * 1e-3; // Get current velocity in East direction
        current.velocityDown = gps.getNedDownVel(5) * 1e-4; // Get current velocity in Down direction

        int rtkStatus = gps.getCarrierSolutionType(5);

        if(rtkStatus == 0){
            switch(gps.getFixType(5)){
                case 0:
                case 1:
                case 2:
                    gpsInfo.fixType = 0; // No-Fix
                    break;
                case 3:
                    gpsInfo.fixType = 1; // 3D-Fix
                    break;
                case 4:
                    gpsInfo.fixType = 2; // DGPS-Fix
                    break;
                case 5:
                    gpsInfo.fixType = 3; // GPS-PPS
                    break;
            }
        }else{
            switch(rtkStatus){
                case 2:
                    gpsInfo.fixType = 4; // RTK-Fix
                    break;
                case 1:
                    gpsInfo.fixType = 5; // RTK-Flt
                    break;
            }
        }
        gpsInfo.home = home; // Set home position
        gpsInfo.pos = current; // Set current position

        gpsInfo.xyz = getDistance(home, current); // Calculate distance from home position
        gpsInfo.satsInView = gps.getSIV(5); // Get number of satellites in view
        gpsInfo.satsUsed = gpsInfo.satsInView;

        gpsInfo.pdop = gps.getPDOP(5) * 1e-2; // Get PDOP value
        gpsInfo.timeOfWeek = gps.getTimeOfWeek(5); // Get time of week in milliseconds

        gpsInfo.error2D = gps.getHorizontalAccuracy(5) * 1e-3; // Get 2D error
        gpsInfo.error3D = gps.getPositionAccuracy(5) * 1e-3; // Get 3D error

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
    if(gps.getCarrierSolutionType() != 2){
        if (DEBUG_MODE) {
            DEBUG_SERIAL.println("GPS fix quality is insufficient to set home position");
        }
        return; // Do not set home if GPS fix quality is insufficient
    }


    home.latitude  = current.latitude;
    home.longitude = current.longitude;
    home.altitude  = current.altitude;


}

void GPSHandler::resetHome() {
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


void GPSHandler::sendRTCMCorrection(uint8_t* data, size_t length) {
    if (DEBUG_MODE) {
        DEBUG_SERIAL.println("Sending RTCM correction data");
    }

    gps.pushRawData(data, length); // Send RTCM correction data over GPS serial port
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

    result.x = pos2.altitude - pos1.altitude + 0.36; // Altitude difference
    result.y = distance * sin(bearing); // east component
    result.z = distance * cos(bearing); // north component

    return result;
}

