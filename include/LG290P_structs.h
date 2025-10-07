#ifndef _SPARKFUN_LG290P_STRUCTS_H
#define _SPARKFUN_LG290P_STRUCTS_H
#include <string>
#include <vector>

struct PvtDomain
{
    bool newDataAvailable = false;

    double latitude = 0;
    double longitude = 0;
    double altitude = 0;

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint32_t nanosecond = 0;
    uint8_t leapSeconds;

    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;

    uint8_t satellitesUsed = 0;
    char quality = '0';
    char fixStatus = 'N';
    double hdop = 0;
    double pdop = 0;
    double course = 0;
    uint32_t timeOfWeek = 0;
    double geoidalSeparation = 0;
    double nvelocity = 0, evelocity = 0, dvelocity = 0;
    double groundSpeed = 0;
    double heading = 0;

    double rmsPseudorangeResidual = 0; // GST
    double latitudeError = 0;
    double longitudeError = 0;
    double heightError = 0;

    void clear() { *this = PvtDomain(); }
};

struct RtcmDomain
{
    double ecefX;
    double ecefY;
    double ecefZ;
    void clear() { *this = RtcmDomain(); }
};

struct EpeDomain
{
    double errorNorth = 0;
    double errorEast = 0;
    double errorDown = 0;
    double error2D = 0;
    double error3D = 0;
    void clear() { *this = EpeDomain(); }
};

struct PlDomain
{
    double probUncertainty = 0;
    double protectionLevelNorth = 0;
    double protectionLevelEast = 0;
    double protectionLevelDown = 0;
    double protectionLevelNorthVelocity = 0;
    double protectionLevelEastVelocity = 0;
    double protectionLevelDownVelocity = 0;
    double protectionLevelTime = 0;
    void clear() { *this = PlDomain(); }
};

struct SvinStatusDomain
{
    int validity = 0;
    int observations = 0;
    int cfgDur = 0;
    double meanX = 0;
    double meanY = 0;
    double meanZ = 0;
    double meanAcc = 0;
    void clear() { *this = SvinStatusDomain(); }
};

class NmeaPacket
{
    friend class LG290P;
    std::vector<std::string> fields;
    bool hasChecksum = false;
    bool checksumValid = false;
    size_t charCount = 0;

    /** 
     * @brief Processes a GGA sentence and updates the snapshot.
     * @param snapshot Reference to the NmeaSnapshot to be updated.
     */
    void processGGA(PvtDomain &snapshot);

    /** 
     * @brief Processes an RMC sentence and updates the snapshot.
     * @param snapshot Reference to the NmeaSnapshot to be updated.
     */
    void processRMC(PvtDomain &snapshot);

    /** 
     * @brief Parses a time string and extracts hour, minute, second, and nanoseconds.
     * @details Parse a time string in the form that GGA and RMC provide
     * @param term The time string to parse.
     * @param hour Reference to store the parsed hour.
     * @param minute Reference to store the parsed minute.
     * @param second Reference to store the parsed second.
     * @param nanos Reference to store the parsed nanoseconds.
     */
    static void parseTime(const std::string &term, uint8_t &hour, uint8_t &minute, uint8_t &second, uint32_t &nanos);

    /** 
     * @brief Parses a date string and extracts year, month, and day.
     * @details Parse a date string in the form that RMC provides
     * @param term The date string to parse.
     * @param year Reference to store the parsed year.
     * @param month Reference to store the parsed month.
     * @param day Reference to store the parsed day.
     */
    static void parseDate(const std::string &term, uint16_t &year, uint8_t &month, uint8_t &day);

    /** 
     * @brief Parses latitude and longitude strings and converts them to decimal degrees.
     * @details Parse latitude and longitude as recorded in NMEA GGA and RMC sentences
     * @param termLat Latitude string.
     * @param termNS North/South indicator.
     * @param termLong Longitude string.
     * @param termEW East/West indicator.
     * @param latitude Reference to store the parsed latitude.
     * @param longitude Reference to store the parsed longitude.
     */
    static void parseLocation(const std::string &termLat, const std::string &termNS, const std::string &termLong, const std::string &termEW, double &latitude, double &longitude);

    /** 
     * @brief Parses the quality indicator from a string, as encoded in the sixth term of a GGA sentence
     * @param term The string containing the quality indicator.
     * @param quality Reference to store the parsed quality.
     */
    static void parseQuality(const std::string &term, char &quality);

    /** 
     * @brief Parses the number of satellites used from a string, as encoded in the seventh term of a GGA sentence
     * @param term The string containing the satellite count.
     * @param used_satellites Reference to store the parsed satellite count.
     */
    static void parseSatelliteCount(const std::string &term, uint8_t &used_satellites);

    /** 
     * @brief Parses a decimal number from a string, as encoded in RMC and GGA sentences
     * @details Time, speed, course, HDOP, and altitude are encoded in this XXX.XX form
     * @param term The string containing the decimal number.
     * @return Parsed decimal value as a 32-bit signed integer. Typically divide by 100 to get useful values
     */
    static int32_t parseDecimal(const std::string &term);

    /** 
     * @brief Parses degrees from a string and converts to decimal format.
     * @details Used by parseLocation to parse latitude and longitude degrees.
     * @param degTerm The string containing the degree value.
     * @param nsewTerm The string containing the NSEW indicator.
     * @param angle Reference to store the parsed angle in decimal degrees.
     */
    static void parseDegrees(const char *degTerm, const char *nsewTerm, double &angle);

    /** 
     * @brief Parses the fix status from a string, as encoded in term #2 of RMC sentence
     * @param term The string containing the fix status.
     * @param fixStatus Reference to store the parsed fix status.
     */
    static void parseFixStatus(const std::string &term, char &fixStatus);

    /** 
     * @brief Parses the speed from a string, as encoded in RMC term #7.
     * @param term The string containing the speed.
     * @param speed Reference to store the parsed speed in meters per second (converted from knots)
     */
    static void parseSpeed(const std::string &term, double &speed);

    /** 
     * @brief Parses the course from a string, as encoded in RMC term #8.
     * @param term The string containing the course.
     * @param course Reference to store the parsed course in degrees.
     */
    static void parseCourse(const std::string &term, double &course);

    /** 
     * @brief Parses the Horizontal Dilution of Precision (HDOP) from a string, as encoded in GGA field #8.
     * @param term The string containing the HDOP.
     * @param hdop Reference to store the parsed HDOP.
     */
    static void parseHdop(const std::string &term, double &hdop);

    /** 
     * @brief Parses the altitude from a string, as encoded in GGA field #9
     * @param term The string containing the altitude.
     * @param altitude Reference to store the parsed altitude in meters.
     */
    static void parseAltitude(const std::string &term, double &altitude);
public:
    /// @brief Indicates whether the sentence has reasonable NMEA construction
    /// @details Note: This method returns true if the sentence *has* a checksum, even if it's wrong
    /// @return true if sentence is valid NMEA
    bool IsValid() const            { return fields.size() > 0 && fields[0].length() > 1 && fields[0][0] == '$' && hasChecksum; }
    /// @brief Tests whether the NMEA checksum is correct
    /// @return true if Checksum is present and valid
    bool ChecksumValid() const      { return checksumValid; }

    /** 
     * @brief Checks if the checksum exists.
     * @return True if the checksum exists, false otherwise.
     */
    bool HasChecksum() const { return hasChecksum; }

    /** 
     * @brief Converts the NMEA packet to a string, recreating it in is original form.
     * @return A string representation of the NMEA packet.
     */
    std::string ToString() const;

    /** 
     * @brief Gets the size of the NMEA packet.
     * @return The size (character count) as a `size_t`.
     */
    size_t Size() const { return charCount; }

    /** 
     * @brief Gets the talker ID from the NMEA packet.
     * @return The talker ID as a string.
     */
    std::string TalkerId() const;

    /** 
     * @brief Gets the sentence ID from the NMEA packet.
     * @return The sentence ID as a string.
     */
    std::string SentenceId() const;

    /** 
     * @brief Gets a specific field from the NMEA packet.
     * @param field Index of the field to retrieve.
     * @return The field value as a string, or an empty string if the index is out of bounds.
     */
    std::string operator[](int field) const { return field >= fields.size() ? "" : fields[field]; }

    /** 
     * @brief Gets the number of fields in the NMEA packet.
     * @return Field count as a `size_t`.
     */
    size_t FieldCount() const { return fields.size(); }

    /** 
     * @brief Creates an NMEA packet from a string.
     * @param str The string representation of the NMEA packet.
     * @return An `NmeaPacket` object.
     */
    static NmeaPacket FromString(const std::string &str);
};

struct RtcmPacket
{
    uint8_t *buffer;
    uint8_t *payload;
    uint16_t bufferlen;
    uint16_t payloadLen;
    uint16_t type;

    // Extract ECEF components from RTCM 1005 packets
    double getEcefX() { return type == 1005 ? extract_38bit_signed(34) / 10000.0 : 0.0; }
    double getEcefY() { return type == 1005 ? extract_38bit_signed(74) / 10000.0 : 0.0; }
    double getEcefZ() { return type == 1005 ? extract_38bit_signed(114) / 10000.0 : 0.0; }

    static bool FromBuffer(uint8_t *buffer, size_t bufferLen, RtcmPacket &result);

private:
    int64_t extract_38bit_signed(int bit_offset);
};


#endif // _SPARKFUN_LG290P_STRUCTS_H