/*
  This is a library to control Quectel GNSS receivers, with
  a focus on the LG290P QuadBand receiver.

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef _SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H
#define _SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H

#include "Arduino.h"
#include "LG290P_structs.h"
#include <SparkFun_Extensible_Message_Parser.h>
#include <list>
#include <map>
#include <set>
#include <string>

#define GPS_MAX_WAIT 15

typedef enum
{
    LG290P_RESULT_OK = 0,
    LG290P_RESULT_TIMEOUT_START_BYTE,
    LG290P_RESULT_TIMEOUT_DATA_BYTE,
    LG290P_RESULT_TIMEOUT_END_BYTE,
    LG290P_RESULT_TIMEOUT_RESPONSE,
    LG290P_RESULT_WRONG_COMMAND,
    LG290P_RESULT_WRONG_MESSAGE_ID,
    LG290P_RESULT_BAD_START_BYTE,
    LG290P_RESULT_BAD_CHECKSUM,
    LG290P_RESULT_BAD_CRC,
    LG290P_RESULT_MISSING_CRC,
    LG290P_RESULT_TIMEOUT,
    LG290P_RESULT_RESPONSE_OVERFLOW,
    LG290P_RESULT_RESPONSE_COMMAND_OK,
    LG290P_RESULT_RESPONSE_COMMAND_ERROR,
    LG290P_RESULT_RESPONSE_COMMAND_WAITING,
    LG290P_RESULT_RESPONSE_COMMAND_CONFIG,
    LG290P_RESULT_CONFIG_PRESENT,
} LG290PResult;

enum
{
    LG290P_COM_TYPE_NMEA = 1,
    LG290P_COM_TYPE_RTCM3 = 4
};

class LG290P
{
    typedef void (*nmeaCallback)(NmeaPacket &nmea);
    typedef void (*rtcmCallback)(RtcmPacket &rtcm);
    struct satinfo
    {
        int elev, azimuth, prn, snr;
        char talker[3];
        bool operator<(const satinfo &other) const
        {
            return prn < other.prn;
        }
    };
    struct
    {
        int mode = -1, ggaRate = -1, rmcRate = -1, pvtRate = -1, plRate = -1, epeRate = -1, svinstatusRate = -1,
            gsvRate = -1, gstRate = -1;
    } devState;
    enum
    {
        ROVERMODE = 1,
        BASEMODE = 2
    };
    enum
    {
        SURVEYDISABLED = 0,
        SURVEYIN = 1,
        SURVEYFIXED = 2
    };

  public:
    /** Handshaking and Client interface **/

    /**
     * @brief Starts the LG290P engine. Required at the beginning of each session
     * @param serialPort the open port that is connected to the LG290P module
     * @param parserDebug if provided, show debugging for the parser
     * @param parserError if provided, show error messages for the parser
     * @return true if the initialization succeeded
     */
    bool begin(HardwareSerial &serialPort, Print *parserDebug = nullptr, Print *parserError = &Serial);

    /**
     * @brief Starts the LG290P engine. Does not require open serial port. Tests various likely baud rates
     * @param serialPort a port that is connected to the LG290P module
     * @param rxPin the pin that receives data from the LG290P
     * @param txPin the pin that sends data to the LG290P
     * @param parserDebug if provided, show debugging for the parser
     * @param parserError if provided, show error messages for the parser
     * @return true if the initialization succeeded
     */
    bool beginAutoBaudDetect(HardwareSerial &serialPort, int rxPin, int txPin, Print *parserDebug = nullptr,
                             Print *parserError = &Serial);

    /**
     * @brief Poll the device with the PQTMUNIQID command to see if it is responding
     * @return true if the device responds within 10 seconds
     */
    bool isConnected();

    /**
     * @brief Test whether the device is in the middle of processing a command
     * @return true if a command is in progress
     */
    bool isBlocking();

    /**
     * @brief Process any byes sitting in Serial stream given at begin(). Must be called often and regularly.
     * @return true if any incoming bytes were processed, false if none
     */
    bool update();

    /**
     * @brief Process a given byte. Allows a stream outside of library to feed the library.
     * @return true if any incoming bytes were processed, false if none
     */
    bool update(byte incoming);

    /**
     * @brief Process a buffer of bytes. Allows a stream outside of library to feed the library.
     * @return true if any incoming bytes were processed, false if none
     */
    bool update(uint8_t *incomingBuffer, uint16_t bufferLength);

    /** Debugging **/

    /**
     * @brief Send a printf-style message to the debug output stream
     * @details (works only if debugging is enabled with enableDebugging)
     */
    void debugPrintf(const char *format, ...);

    /**
     * @brief Define the debugging output stream and turn on debugging
     */
    void enableDebugging(Print &debugPort = Serial);

    /**
     * @brief Turn off general debugging
     */
    void disableDebugging();

    /**
     * @brief Define the stream that handles parser debug and turn on parser debugging
     */
    void enableParserDebug(Print *print = &Serial);

    /**
     * @brief Turn off parser debugging
     */
    void disableParserDebug();

    /**
     * @brief Define the stream that handles parser error debug and turn on parser error debugging
     */
    void enableParserErrors(Print *print = &Serial);

    /**
     * @brief Turn off parser error debugging
     */
    void disableParserErrors();

    /**
     * @brief Print a debug message warning about invalid checksums (if debugging enabled)
     */
    void enablePrintBadChecksums();

    /**
     * @brief Do not print any warning about invalid checksums (if debugging enabled)
     */
    void disablePrintBadChecksums();

    /**
     * @brief Print a debug message showing parser transitions (if debugging enabled)
     */
    void enablePrintParserTransitions();

    /**
     * @brief Do not print any warning about invalid checksums (if debugging enabled)
     */
    void disablePrintParserTransitions();

    /**
     * @brief Print a diagnostic when a new NMEA sentence or RTCM packet arrives
     */
    void enablePrintRxMessages();

    /**
     * @brief No diagnostic when a new NMEA sentence or RTCM packet arrives
     */
    void disablePrintRxMessages();

    /**
     * @brief Dump received messages by default
     * @details This may be problematic with large numbers and speeds of packets
     */
    void enableRxMessageDump();

    /**
     * @brief Don't dump messages by default
     */
    void disableRxMessageDump();

    /**
     * @brief Display the current parser configuration
     * @param print the stream to send the configuration information to
     */
    void printParserConfiguration(Print *print = &Serial);

    /**
     * @brief Dump an arbitrary buffer to the debug stream
     * @param buffer the buffer to dump
     * @param length the length of the buffer
     */
    void dumpBuffer(const uint8_t *buffer, uint16_t length);

    /** Statistics **/

    /**
     * @brief return NMEA packet count
     * @details if sentenceId is not null (e.g. "GGA"), return the number of GGA sentences seen
     * @details otherwise, return the total number of valid NMEA sentences seen
     * @param sentenceId the sentence id, e.g. "RMC" or "GGA" etc., that you're looking for or null for all
     * @return the number of matching NMEA sentences seen
     */
    int getNmeaCount(const char *sentenceId = nullptr);

    /**
     * @brief return RTCM packet count
     * @details if packetType is specified (e.g. 1006), return the number of packets of that type seen
     * @details otherwise, return the total number of valid RTCM packets seen
     * @param packetType the packetType e.g. 1005 or 1006, etc., or omit for all RTCM packets
     * @return the number of matching RTCM packets seen
     */
    int getRtcmCount(int packetType = -1);

    /**
     * @brief clear the NMEA packet counters
     */
    void clearNmeaCount()
    {
        nmeaCounters.clear();
    }

    /**
     * @brief clear the RTCM packet counters
     */
    void clearRtcmCount()
    {
        rtcmCounters.clear();
    }

    /** Device configuration **/

    /**
     * @brief Set the device to "Base station" mode
     * @param resetAfter true if device should save new setting and reset to make it 'take'
     * @details Uses the LG290P "PQTMCFGRCVRMODE" command to set receiver mode
     * @return true if the mode was set correctly
     */
    bool setModeBase(bool resetAfter = true);

    /**
     * @brief Set the device to "Rover" mode
     * @param resetAfter true if device should save new setting and reset to make it 'take'
     * @details Uses the LG290P "PQTMCFGRCVRMODE" command to set receiver mode
     * @return true if the mode was set correctly
     */
    bool setModeRover(bool resetAfter = true);

    /**
     * @brief Set the device to "Base station" mode, but only if it's not already
     * @details Uses the LG290P "PQTMCFGRCVRMODE" command to set receiver mode
     * @return true if the mode was set correctly
     */
    bool ensureModeBase()
    {
        return devState.mode == BASEMODE || setModeBase();
    }

    /**
     * @brief Set the device to "Rover" mode, but only if it's not already
     * @details Uses the LG290P "PQTMCFGRCVRMODE" command to set receiver mode
     * @return true if the mode was set correctly
     */
    bool ensureModeRover()
    {
        return devState.mode == ROVERMODE || setModeRover();
    }

    /**
     * @brief Gets the device mode
     * @details Uses the LG290P "PQTMCFGRCVRMODE" command to get receiver mode
     * @param mode set to 1 if device is in Rover mode, 2 if Base mode, 0 = unknown
     * @return true if the mode was acquired
     */
    bool getMode(int &mode);

    /**
     * @brief Set the baud rate of the designated port
     * @param port the LG290P UART to be configured (1, 2, or 3)
     * @param newBaud the new baud rate of the port
     * @return true if the new baud rate was set correctly
     */
    bool setPortBaudrate(int port, uint32_t newBaud, uint16_t maxWaitMs = GPS_MAX_WAIT);

    /**
     * @brief Set the baud rate of the current port
     * @param newBaud the new baud rate of the port
     * @return true if the new baud rate was set correctly
     */
    bool setBaudrate(uint32_t newBaud, uint16_t maxWaitMs = GPS_MAX_WAIT);

    /**
     * @brief Get information about the designated UART
     * @param port the number of the UART (1, 2, or 3)
     * @param newBaud (out) the baud rate of the port
     * @param dataBits (out) the number of databits for the selected UART
     * @param parity (out) the parity on the selected UART
     * @param stop (out) the number of stop bits on the selected the UART
     * @param flowControl (out) 0 = flow control disabled
     * @return true if the port info was acquired without problem
     */
    bool getPortInfo(int port, uint32_t &newBaud, uint8_t &dataBits, uint8_t &parity, uint8_t &stop,
                     uint8_t &flowControl, uint16_t maxWaitMs = GPS_MAX_WAIT);

    /**
     * @brief Enable or disable the protocols available for input on the specified port
     * @param port the LG290P port to be configured (1, 2, or 3)
     * @param newFlags a bitfield: some combination of LG290P_COM_TYPE_NMEA and LG290P_COM_TYPE_RTCM3. 0 for disable
     * all.
     * @return true if the new input protocols were enabled/disabled
     */
    bool setPortInputProtocols(int port, uint8_t newFlags);

    /**
     * @brief Enable or disable the protocols available for output on the specified port
     * @param port the LG290P port to be configured (1, 2, or 3)
     * @param newFlags a bitfield: some combination of LG290P_COM_TYPE_NMEA and LG290P_COM_TYPE_RTCM3. 0 for disable
     * all.
     * @return true if the new output protocols were enabled/disabled
     */
    bool setPortOutputProtocols(int port, uint8_t newFlags);

    /**
     * @brief Get the current protocol enable/disable status for the specified port
     * @param port the LG290P port to be configured (1, 2, or 3)
     * @param inputFlags a reference to a bitfield: returns some combination of LG290P_COM_TYPE_NMEA and
     * LG290P_COM_TYPE_RTCM3. 0 for all disabled
     * @param outputFlags a reference to a bitfield: returns some combination of LG290P_COM_TYPE_NMEA and
     * LG290P_COM_TYPE_RTCM3. 0 for all disabled
     * @return true if the new protocol flags were successfully acquired
     */
    bool getPortProtocols(int port, uint8_t &inputFlags, uint8_t &outputFlags);

    /**
     * @brief Set the PPS LED and output line characteristics
     * @details Uses the PQTMCFGPPS command
     * @param duration number of milliseconds (out of 1000) that the line/LED are high/on
     * @param alwaysOutput true if PPS pulse should be generated even when device has no fix
     * @param positivePolarity set to false if the pulse should be inverted
     * @return true if the command succeeded
     */
    bool setPPS(uint16_t duration, bool alwaysOutput, bool positivePolarity = true);

    /**
     * @brief Turn off the PPS signal
     * @details Uses the PQTMCFGPPS command
     * @return true if the command succeeded
     */
    bool disablePPS();

    /**
     * @brief Get information about the PPS settings
     * @details Uses the PQTMCFGPPS command
     * @param enabled (out) true if PPS enabled
     * @param duration (out) milliseconds (out of 1000) the pulse is high/on
     * @param alwaysOutput (out) true if PPS pulse should be generated even when device has no fix
     * @param positivePolarity (out) false if the pulse should be inverted
     */
    bool getPPS(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity);

    /**
     * @brief Get information about which satellite constellations are currently enabled
     * @details Uses PQTMCFGCNST command to query LG290P
     * @param enableGPS (out) true if GPS constellation is currently enabled
     * @param enableGlonass (out) true if GLONASS constellation is currently enabled
     * @param enableGalileo (out) true if Galileo constellation is currently enabled
     * @param enableBds (out) true if Beidou constellation is currently enabled
     * @param enableQzss (out) true if QZSS constellation is currently enabled
     * @param enableNavIC (out) true if NavIC constellation is currently enabled
     * @return true if command succeeded
     */
    bool getConstellations(bool &enableGPS, bool &enableGlonass, bool &enableGalileo, bool &enableBds, bool &enableQzss,
                           bool &enableNavIV);

    /**
     * @brief Set which satellite constellations are currently enabled
     * @details Uses PQTMCFGCNST command to configure LG290P
     * @param enableGPS true to enable GPS constellation
     * @param enableGlonass true to enable GLONASS constellation
     * @param enableGalileo true to enable Galileo constellation
     * @param enableBds true to enable Beidou constellation
     * @param enableQzss true to enable QZSS constellation
     * @param enableNavIC true to enable NavIC constellation
     */
    bool setConstellations(bool enableGPS, bool enableGlonass, bool enableGalileo, bool enableBds, bool enableQzss,
                           bool enableNavIC);
    /**
     * @brief Retrieves the serial number of the device.
     * @details uses the PQTMUNIQID command to retrieve serial
     * @param serial Reference to a string where the serial number will be stored.
     * @return true if successful, false otherwise.
     */
    bool getSerialNumber(std::string &serial);

    /**
     * @brief Gets version information of the device.
     * @details Uses the PQTMVERNO command
     * @param version Reference to a string where the version will be stored.
     * @param buildDate Reference to a string where the build date will be stored.
     * @param buildTime Reference to a string where the build time will be stored.
     * @return true if successful, false otherwise.
     */
    bool getVersionInfo(std::string &version, std::string &buildDate, std::string &buildTime);

    /**
     * @brief Gets the firmware version of the device.
     * @details Uses the PQTMVERNO command
     * @param version Reference to an int where the version will be stored.
     * @return true if successful, false otherwise.
     * @note version will be set to 0 if the get fails.
     */
    bool getFirmwareVersion(int &version);

    /**
     * @brief Gets the current fix interval.
     * @details Uses the PQTMCFGFIXRATE command.
     * @param fixInterval Reference to a uint16_t where the fix interval (ms) will be stored.
     * @return true if successful, false otherwise.
     */
    bool getFixInterval(uint16_t &fixInterval);

    /**
     * @brief Sets a new elevation threshold for position engine.
     * @details Uses the PQTMCFGELETHD command. Use a value between -90 and 90. Default is 5 degrees.
     * @param elevationAngle The new elevation threshold for the position engine.
     * @return true if successful, false otherwise.
     */
    bool setElevationAngle(int elevationAngle);

    /**
     * @brief Gets the elevation threshold for position engine.
     * @details Uses the PQTMCFGELETHD command.
     * @param elevationAngle Reference to a int where the elevation angle (degrees) will be stored.
     * @return true if successful, false otherwise.
     */
    bool getElevationAngle(int &elevationAngle);

    /**
     * @brief Sets the CNR threshold for position engine.
     * @details Uses the PQTMCFGCNRTHD command. Use a value between 0.0 and 99.0. Default is 10.0 dBHz.
     * @param cnr The new CNR threshold for the position engine.
     * @return true if successful, false otherwise.
     */
    bool setCNR(float cnr);

    /**
     * @brief Gets the CNR threshold for position engine.
     * @details Uses the PQTMCFGCNRTHD command.
     * @param cnr Reference to a float where the CNR value will be stored.
     * @return true if successful, false otherwise.
     */
    bool getCNR(float &cnr);

    /**
     * @brief Sets the max differential age of RTK fix.
     * @details Uses the PQTMCFGRTK command. Use a value between 1 and 600. Default is 120 seconds.
     * @param timeout The new RTK differential timeout for the position engine.
     * @return true if successful, false otherwise.
     */
    bool setRtkDifferentialAge(uint16_t timeout);

    /**
     * @brief Gets the max differential age of RTK fix.
     * @details Uses the PQTMCFGRTK command.
     * @param timeout Reference to a uint16_t where the timeout value will be stored.
     * @return true if successful, false otherwise.
     */
    bool getRtkDifferentialAge(uint16_t &timeout);

    /**
     * @brief Sets a new elevation threshold for position engine.
     * @details Uses the PQTMCFGELETHD command. Use a value between -90 and 90
     * @param fixInterval The new fix interval to set.
     * @return true if successful, false otherwise.
     */
    bool setFixInterval(uint16_t fixInterval);

    /**
     * @brief Enables or sets the rate for a specific message.
     * @details Use 0 to disable the message or N for 1 message every N fixes.
     * @details Uses the PQTMCFGMSGRATE command to set message rate.
     * @param msgName The name of the message, e.g. "GGA", "RTCM3-1005", "RTCM2-107X".
     * @param rate The rate at which to send the message.
     * @param msgver (Optional) The message version for PQTM messages, default is -1.
     * @return true if successful, false otherwise.
     */
    bool setMessageRate(const char *msgName, int rate, int msgver = -1);

    /**
     * @brief Enables or sets the rate for a specific message on a specific port.
     * @details Use 0 to disable the message or N for 1 message every N fixes.
     * @details Uses the PQTMCFGMSGRATE command to set message rate.
     * @param msgName The name of the message, e.g. "GGA", "RTCM3-1005", "RTCM2-107X".
     * @param rate The rate at which to send the message.
     * @param portNumber The port on which to send the message (UART 1/2/3).
     * @param msgver (Optional) The message version for PQTM messages, default is -1.
     * @return true if successful, false otherwise.
     */
    bool setMessageRateOnPort(const char *msgName, int rate, int portNumber, int msgver = -1);

    /**
     * @brief Queries the broadcast rate for the specific message.
     * @details Returns rate of 0 to indicated a disabled message or N for 1 message every N fixes.
     * @details Uses the PQTMCFGMSGRATE command to get the message rate.
     * @param msgName The name of the message, e.g. "GGA", "RMCM3-1005", "RTCM2-107X".
     * @param rate The rate returned, if successful
     * @param msgver (Optional) The message version for PQTM messages, default is -1.
     * @return true if successful, false otherwise.
     */
    bool getMessageRate(const char *msgName, int &rate, int msgver = -1);

    /**
     * @brief Saves the current configuration.
     * @details Uses the PQTMSAVEPAR command.
     * @return true if successful, false otherwise.
     */
    bool save();

    /**
     * @brief Restores the default configuration.
     * @details Uses the PQTMRESTOREPAR command.
     * @return true if successful, false otherwise.
     */
    bool factoryRestore();

    // Resets and engine control
    /**
     * @brief Performs a software reset of the device.
     * @return true if successful, false otherwise.
     */
    bool reset()
    {
        return genericReset("PQTMSRR");
    }

    /**
     * @brief Performs a cold reset (complete reset) of the device.
     * @return true if successful, false otherwise.
     */
    bool coldStart()
    {
        return genericReset("PQTCOLD");
    }

    /**
     * @brief Performs a warm reset (partial reset) of the device.
     * @return true if successful, false otherwise.
     */
    bool warmStart()
    {
        return genericReset("PQTMWARM");
    }

    /**
     * @brief Performs a hot reset (minimal reset) of the device.
     * @return true if successful, false otherwise.
     */
    bool hotStart()
    {
        return genericReset("PQTMHOT");
    }

    /**
     * @brief Disables the GNSS engine.
     * @return true if successful, false otherwise.
     */
    bool disableEngine();

    /**
     * @brief Enables the GNSS engine.
     * @return true if successful, false otherwise.
     */
    bool enableEngine();

    /** Message subscriptions **/

    /**
     * @brief Subscribes to an NMEA sentence id
     * @details The callback function provides a single parameter of type NmeaPacket &.
     * @param msgName The id of the NMEA message ("GGA", "RMC", etc.)
     * @param callback The callback function to handle the message.
     * @return true if successful, false otherwise.
     */
    bool nmeaSubscribe(const char *msgName, nmeaCallback callback);

    /**
     * @brief Unsubscribes from an NMEA sentence.
     * @param msgName The id of the NMEA message ("RMC", "GGA", etc.)
     * @return true if successful, false otherwise.
     */
    bool nmeaUnsubscribe(const char *msgName);

    /**
     * @brief Subscribes to all NMEA messages.
     * @param callback The callback function to handle the messages.
     * @return true if successful, false otherwise.
     */
    bool nmeaSubscribeAll(nmeaCallback callback);

    /**
     * @brief Unsubscribes from all NMEA messages.
     * @return true if successful, false otherwise.
     */
    bool nmeaUnsubscribeAll();

    /**
     * @brief Subscribes to an RTCM message.
     * @param type The RTCM message type (1005, 1006, etc.)
     * @param callback The callback function to handle the message.
     * @return true if successful, false otherwise.
     */
    bool rtcmSubscribe(uint16_t type, rtcmCallback callback);

    /**
     * @brief Unsubscribes from an RTCM message.
     * @param type The RTCM message type (1005, 1006, etc.).
     * @return true if successful, false otherwise.
     */
    bool rtcmUnsubscribe(uint16_t type);

    /**
     * @brief Subscribes to all RTCM messages.
     * @param callback The callback function to handle the messages.
     * @return true if successful, false otherwise.
     */
    bool rtcmSubscribeAll(rtcmCallback callback);

    /**
     * @brief Unsubscribes from all RTCM messages.
     * @return true if successful, false otherwise.
     */
    bool rtcmUnsubscribeAll();

    /** Command handling **/

    /**
     * @brief Sends a custom command to the device.
     * @details Send a command, like "PQTMUNIQID" to the device
     * @details This function will prepend the $, if needed, and calculate the checksum.
     * @details If command succeeds, get return string with getCommandResponse().
     * @details Params typically begins with a comma, like ",R"
     * @param command The command to send.
     * @param parms (Optional) Parameters for the command, default is an empty string.
     * @param maxWaitMs (Optional) Maximum wait time in milliseconds, default is 10 ms.
     * @param waitForResponse (Optional) Wait for the matching response string
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = 10,
                     bool waitForResponse = true);

    /**
     * @brief Sends a custom command that generates no response from the device.
     * @details Typically used for RESET commands that don't have the capacity to respond
     * @details This function will prepend the $, if needed, and calculate the checksum.
     * @param command The command to send.
     * @param maxWaitMs (Optional) Maximum wait time in milliseconds, default is 10 ms.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendCommandNoResponse(const char *command, uint16_t maxWaitMs = GPS_MAX_WAIT);

    /**
     * @brief Sends a full command line to the device.
     * @details Sends the complete command line to the device
     * @details This will prepend the $ if needed and calculate the checksum.
     * @param command The command line to send.
     * @param maxWaitMs (Optional) Maximum wait time in milliseconds, default is 10 ms.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendCommandLine(const char *command, uint16_t maxWaitMs = GPS_MAX_WAIT);

    /**
     * @brief Retrieves the response to the last successful command issued.
     * @return A reference to the NmeaPacket containing the response.
     */
    NmeaPacket &getCommandResponse()
    {
        return pqtmResponse;
    }

    /**
     * @brief Sends a command expecting an OK response.
     * @param command The command to send.
     * @param parms (Optional) Parameters for the command, default is an empty string.
     * @param maxWaitMs (Optional) Maximum wait time in milliseconds, default is 10 ms.
     * @return true if an OK response is received, and "OK" string is received after the first comma
     */
    bool sendOkCommand(const char *command, const char *parms = "", uint16_t maxWaitMs = GPS_MAX_WAIT);

    /** Satellites **/

    /**
     * @brief Gets a list of visible satellites.
     * @details Get a list of satellites from a particular talker id ("BD", "GP", etc.) or ALL satellites
     * @param talker (Optional) The talker identifier, like "GA".  Set to nullptr for ALL satellites.
     * @return A list of satinfo structures representing visible satellites.
     */
    std::list<satinfo> getVisibleSats(const char *talker = nullptr);

    /**
     * @brief Checks if new satellite information has arrived since the last query.
     * @return true if new satellite info is available, false otherwise.
     */
    bool isNewSatelliteInfoAvailable();

    /**
     * @brief Gets the number of satellites currently being tracked.
     * @details Information gleaned from GSV sentences
     * @return The number of satellites being tracked.
     */
    uint16_t getSatellitesInViewCount();

    /**
     * @brief Gets the number of satellites used for positioning.
     * @details Information gleaned from GGA sentence.
     * @return The number of satellites used.
     */
    uint16_t getSatellitesUsedCount()
    {
        ensurePvtEnabled();
        return pvtDomain.satellitesUsed;
    }

    /**
     * @brief Gets the fix type 0+
     * @details Information gleaned from GGA sentence.
     * @details 0 = Fix not available or invalid.
     * @details 1 = GPS SPS Mode, fix valid.
     * @details 2 = Differential GPS, SPS Mode, or Satellite Based Augmentation. System (SBAS), fix valid.
     * @details 3 = GPS PPS Mode, fix valid.
     * @details 4 = Real Time Kinematic (RTK) System used in RTK mode with fixed integers.
     * @details 5 = Float RTK. Satellite system used in RTK mode, floating integers.
     * @details Note: this function is unique in using the "quality" field from GGA rather than PQTMPVT.
     * @details PQTMPVT has a more limited response: only 0-3.
     * @return The fix type.
     */
    uint8_t getFixQuality()
    {
        ensureGgaEnabled();
        return pvtDomain.quality - '0';
    } // Convert ASCII to uint8_t

    /**
     * @brief Gets the fix status (RMC 'A' or 'V')
     * @details Information gleaned from RMC sentence.
     * @details 'V' = Fix not available or invalid (void).
     * @details 'A' = Fix available
     * @return The fix status ('A' or 'V' for void).
     */
    char getFixStatus()
    {
        ensureRmcEnabled();
        return pvtDomain.fixStatus;
    }

    /** Survey Mode **/

    /**
     * @brief Gets the current survey mode settings.
     * @details Uses PQTMCFGSVIN command
     * @param mode Reference to an integer for the mode (1 = Survey In mode, 2 = Fixed mode)
     * @param positionTimes Reference to an integer for the position times (survey mode only: number of seconds to
     * calculate it)
     * @param accuracyLimit Reference to a double for the accuracy limit (meters).
     * @param ecefX Reference to a double for the ECEF X coordinate.
     * @param ecefY Reference to a double for the ECEF Y coordinate.
     * @param ecefZ Reference to a double for the ECEF Z coordinate.
     * @return true if the mode settings were successfully retrieved, false otherwise.
     */
    bool getSurveyDetails(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY,
                          double &ecefZ);

    /**
     * @brief Gets the current survey mode.
     * @details Uses PQTMCFGSVIN command
     * @return the Survey In mode: 0 = Disabled, 1 = Survey In, 2 = Fixed
     */
    uint8_t getSurveyMode();

    /**
     * @brief Sets the device to "Survey-In" mode.
     * @details Uses PQTMCFGSVIN command
     * @param positionTimes The number of position times to use.
     * @param accuracyLimit (Optional) The accuracy limit, default is 0.
     * @param resetAfter (Optional) Reset the device afterwards for new setting to 'take'
     * @return true if the mode was successfully set, false otherwise.
     */
    bool setSurveyInMode(int positionTimes, double accuracyLimit = 0, bool resetAfter = true);

    /**
     * @brief Sets the device to "Fixed" survey mode with ECEF coordinates.
     * @details Uses PQTMCFGSVIN command
     * @param ecefX The ECEF X coordinate.
     * @param ecefY The ECEF Y coordinate.
     * @param ecefZ The ECEF Z coordinate.
     * @param resetAfter (Optional) Reset the device afterwards for new setting to 'take'
     * @return true if the mode was successfully set, false otherwise.
     */
    bool setSurveyFixedMode(double ecefX, double ecefY, double ecefZ, bool resetAfter = true);

    /**
     * @brief Disables Survey In or Survey Fixed mode
     * @details Uses PQTMCFGSVIN command
     * @param resetAfter (Optional) Reset the device afterwards for new setting to 'take'
     * @return true if the mode was successfully set, false otherwise.
     */
    bool disableSurveyInMode(bool resetAfter = true);

    /** Geodetic reporting **/
    /**
     * @brief Checks if new snapshot data (location, time, date, fix, etc.) is available.
     * @details set to true whenever GGA or RMC sentence arrives
     * @return true if new snapshot data is available, false otherwise.
     */
    bool isNewSnapshotAvailable();

    /**
     * @brief Gets the latitude of the current position.
     * @return The latitude in degrees.
     */
    double getLatitude()
    {
        ensurePvtEnabled();
        return pvtDomain.latitude;
    }

    /**
     * @brief Gets the longitude of the current position.
     * @return The longitude in degrees.
     */
    double getLongitude()
    {
        ensurePvtEnabled();
        return pvtDomain.longitude;
    }

    /**
     * @brief Gets the altitude of the current position.
     * @return The altitude in meters.
     */
    double getAltitude()
    {
        ensurePvtEnabled();
        return pvtDomain.altitude;
    }

    /**
     * @brief Gets the horizontal speed of the device.
     * @return The horizontal speed in meters per second.
     */
    double getHorizontalSpeed()
    {
        ensurePvtEnabled();
        return pvtDomain.groundSpeed;
    }

    /**
     * @brief Gets the year.
     * @return Year as a 16-bit unsigned integer.
     */
    uint16_t getYear()
    {
        ensurePvtEnabled();
        return pvtDomain.year;
    }

    /**
     * @brief Gets the month.
     * @return Month as an 8-bit unsigned integer.
     */
    uint8_t getMonth()
    {
        ensurePvtEnabled();
        return pvtDomain.month;
    }

    /**
     * @brief Gets the day.
     * @return Day as an 8-bit unsigned integer.
     */
    uint8_t getDay()
    {
        ensurePvtEnabled();
        return pvtDomain.day;
    }

    /**
     * @brief Gets the hour.
     * @return Hour as an 8-bit unsigned integer.
     */
    uint8_t getHour()
    {
        ensurePvtEnabled();
        return pvtDomain.hour;
    }

    /**
     * @brief Gets the minute.
     * @return Minute as an 8-bit unsigned integer.
     */
    uint8_t getMinute()
    {
        ensurePvtEnabled();
        return pvtDomain.minute;
    }

    /**
     * @brief Gets the second.
     * @return Second as an 8-bit unsigned integer.
     */
    uint8_t getSecond()
    {
        ensurePvtEnabled();
        return pvtDomain.second;
    }

    /**
     * @brief Gets the millisecond.
     * @return Millisecond as a 16-bit unsigned integer.
     */
    uint16_t getMillisecond()
    {
        ensurePvtEnabled();
        return (uint16_t)(pvtDomain.nanosecond / 1000000);
    }

    /**
     * @brief Gets the leap second.
     * @return Leap second as a 16-bit unsigned integer.
     */
    uint16_t getLeapSeconds()
    {
        ensurePvtEnabled();
        return pvtDomain.leapSeconds;
    }

    /**
     * @brief Gets course/heading in degrees
     * @return Course/heading in degrees
     */
    double getCourse()
    {
        ensurePvtEnabled();
        return pvtDomain.course;
    }

    /**
     * @brief Gets Horizontal Dilution of Precision
     * @return HDOP
     */
    double getHdop()
    {
        ensurePvtEnabled();
        return pvtDomain.hdop;
    }

    /**
     * @brief Gets Position Dilution of Precision
     * @return PDOP
     */
    double getPdop()
    {
        ensurePvtEnabled();
        return pvtDomain.pdop;
    }

    /**
     * @brief Gets time of week
     * @return Milliseconds since the beginning of the week.
     */
    uint32_t getTimeOfWeek()
    {
        ensurePvtEnabled();
        return pvtDomain.timeOfWeek;
    }

    /**
     * @brief Gets geoidal separation (the difference between the WGS84 earth ellipsoid surface and the mean-sea-level
     * surface).
     * @return Geoidal separation in meters.
     */
    double getGeoidalSeparation()
    {
        ensurePvtEnabled();
        return pvtDomain.geoidalSeparation;
    }

    /**
     * @brief Gets North velocity
     * @return North velocity in meters per second
     */
    double getNorthVelocity()
    {
        ensurePvtEnabled();
        return pvtDomain.nvelocity;
    }

    /**
     * @brief Gets East velocity
     * @return East velocity in meters per second
     */
    double getEastVelocity()
    {
        ensurePvtEnabled();
        return pvtDomain.evelocity;
    }

    /**
     * @brief Gets Down velocity
     * @return Down velocity in meters per second
     */
    double getDownVelocity()
    {
        ensurePvtEnabled();
        return pvtDomain.dvelocity;
    }

    /**
     * @brief Gets the age of the latest NMEA geodetic report in milliseconds.
     * @return Number of milliseconds since the last report (GGA or RMC)
     */
    uint32_t getPVTDomainAgeMs();

    /**
     * @brief Returns the Estimated North Positioning Error
     * @return Error in meters
     */
    double getNorthError()
    {
        ensureEpeEnabled();
        return epeDomain.errorNorth;
    }

    /**
     * @brief Returns the Estimated East Positioning Error
     * @return Error in meters
     */
    double getEastError()
    {
        ensureEpeEnabled();
        return epeDomain.errorEast;
    }

    /**
     * @brief Returns the Estimated Down Positioning Error
     * @return Error in meters
     */
    double getDownError()
    {
        ensureEpeEnabled();
        return epeDomain.errorDown;
    }

    /**
     * @brief Returns the Estimated 2D Positioning Error
     * @note This is based on what SWMaps displays - the greater of the lat and lon errors as delta-H
     * @return Error in meters
     */
    double get2DError()
    {
        if (firmwareVersion >= 4)
        {
            ensureGstEnabled();
            if (pvtDomain.latitudeError > pvtDomain.longitudeError)
                return pvtDomain.latitudeError;
            return pvtDomain.longitudeError;
        }
        else
        {
            ensureEpeEnabled();
            return epeDomain.error2D;
        }
    }

    /**
     * @brief Returns the Estimated 3D Positioning Error
     * @note This is based on what SWMaps displays - the height error as delta-V
     * @return Error in meters
     */
    double get3DError()
    {
        if (firmwareVersion >= 4)
        {
            ensureGstEnabled();
            return pvtDomain.heightError;
        }
        else
        {
            ensureEpeEnabled();
            return epeDomain.error3D;
        }
    }

    /**
     * @brief Returns the Probability of Uncertainty Level per Epoch from PQTMPL
     * @return Probability in %
     */
    double getProbUncertainty()
    {
        ensurePlEnabled();
        return plDomain.probUncertainty;
    }

    /**
     * @brief Returns the Protection Level of North Position from PQTMPL
     * @return mm
     */
    double getProtLevelNorth()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelNorth;
    }

    /**
     * @brief Returns the Protection Level of East Position from PQTMPL
     * @return mm
     */
    double getProtLevelEast()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelEast;
    }

    /**
     * @brief Returns the Protection Level of Down Position from PQTMPL
     * @return mm
     */
    double getProtLevelDown()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelDown;
    }

    /**
     * @brief Returns the Protection Level of North Velocity from PQTMPL
     * @return mm/s
     */
    double getProtLevelNorthVelocity()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelNorthVelocity;
    }

    /**
     * @brief Returns the Protection Level of East Velocity from PQTMPL
     * @return mm/s
     */
    double getProtLevelEastVelocity()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelEastVelocity;
    }

    /**
     * @brief Returns the Protection Level of Down Velocity from PQTMPL
     * @return mm/s
     */
    double getProtLevelDownVelocity()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelDownVelocity;
    }

    /**
     * @brief Returns the Protection Level of Time from PQTMPL
     * @return ns
     */
    double getProtLevelTime()
    {
        ensurePlEnabled();
        return plDomain.protectionLevelTime;
    }

    /**
     * @brief Returns the Validity field from PQTMSVINSTATUS (0=Invalid, 1=In-progress, 2=Valid)
     * @return Validity
     */
    int getSurveyInStatus()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.validity;
    }

    /**
     * @brief Returns the Observations field from PQTMSVINSTATUS (count between 0 and CfgDur)
     * @return Observations
     */
    int getSurveyInObservations()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.observations;
    }

    /**
     * @brief Returns the CfgDur field from PQTMSVINSTATUS
     * @return CfgDur
     */
    int getSurveyInCfgDuration()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.cfgDur;
    }

    /**
     * @brief Returns the MeanX field from PQTMSVINSTATUS
     * @return MeanX
     */
    double getSurveyInMeanX()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.meanX;
    }

    /**
     * @brief Returns the MeanY field from PQTMSVINSTATUS
     * @return MeanY
     */
    double getSurveyInMeanY()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.meanY;
    }

    /**
     * @brief Returns the MeanZ field from PQTMSVINSTATUS
     * @return MeanZ
     */
    double getSurveyInMeanZ()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.meanZ;
    }

    /**
     * @brief Returns the MeanAcc field from PQTMSVINSTATUS
     * @return MeanAcc
     */
    double getSurveyInMeanAccuracy()
    {
        ensureSvinStatusEnabled();
        return svinStatusDomain.meanAcc;
    }

    // Convert LLH (geodetic) to ECEF
    // From: https://stackoverflow.com/questions/19478200/convert-latitude-and-longitude-to-ecef-coordinates-system
    static void geodeticToEcef(double lat, double lon, double alt, double &xOut, double &yOut, double &zOut);

    // Convert ECEF to LLH (geodetic)
    // From: https://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
    static void ecefToGeodetic(double x, double y, double z, double &latOut, double &lonOut, double &altOut);

#if false // TODO

    float getLatitudeDeviation();
    float getLongitudeDeviation();
    float getAltitudeDeviation();
    float getHorizontalSpeedDeviation();
    float getVerticalSpeedDeviation();

    double getEcefX();
    double getEcefY();
    double getEcefZ();
    float getEcefXDeviation();
    float getEcefYDeviation();
    float getEcefZDeviation();

    uint8_t getSolutionStatus();
    uint8_t getPositionType();
    uint8_t getVelocityType();

    uint8_t getRTKSolution();
    uint8_t getPseudorangeCorrection();

    uint8_t getTimeStatus();
    uint8_t getDateStatus();
    double getTimeOffset();
    double getTimeOffsetDeviation();

    uint8_t getModelType();
    char *getVersion();
    char *getID();
    char *getCompileTime();
#endif

  private:
    // Firmware version
    int firmwareVersion = 0;
    const char *firmwareVersionPrefix = "LG290P03AANR01A";

    // Update times
    unsigned long lastUpdatePvtDomain = 0;
    unsigned long lastUpdateEcef = 0;
    unsigned long lastUpdateDateTime = 0;
    unsigned long lastUpdateVersion = 0;

    // Sentence and packet counters
    std::map<std::string, int> nmeaCounters;
    std::map<int, int> rtcmCounters;

    // Subscriptions
    std::map<std::string, nmeaCallback> nmeaSubscriptions;
    std::map<int, rtcmCallback> rtcmSubscriptions;
    nmeaCallback nmeaAllSubscribe = nullptr;
    rtcmCallback rtcmAllSubscribe = nullptr;

    // State management
    SEMP_PARSE_STATE *_sempParse;             // State of the SparkFun Extensible Message Parser
    bool lg290PLibrarySemaphoreBlock = false; // Gets set to true when the Unicore library needs to interact directly
    bool scanForMsgsEnabled();
    void ensureMsgEnabled(bool enabled, const char *msg, int msgVer = -1);
    void ensureGgaEnabled()
    {
        ensureMsgEnabled(devState.ggaRate > 0, "GGA");
    }
    void ensureRmcEnabled()
    {
        ensureMsgEnabled(devState.rmcRate > 0, "RMC");
    }
    void ensurePvtEnabled()
    {
        ensureMsgEnabled(devState.pvtRate > 0, "PQTMPVT", 1);
    }
    void ensurePlEnabled()
    {
        ensureMsgEnabled(devState.plRate > 0, "PQTMPL", 1);
    }
    void ensureEpeEnabled()
    {
        ensureMsgEnabled(devState.epeRate > 0, "PQTMEPE", 2);
    }
    void ensureSvinStatusEnabled()
    {
        ensureMsgEnabled(devState.svinstatusRate > 0, "PQTMSVINSTATUS", 1);
    }
    void ensureGsvEnabled()
    {
        ensureMsgEnabled(devState.gsvRate > 0, "GSV");
    }
    void ensureGstEnabled()
    {
        if (firmwareVersion >= 4)
            ensureMsgEnabled(devState.gstRate > 0, "GST");
    }
    void clearAll();
    bool genericReset(const char *resetCmd);

    // Debugging
    Print *_debugPort = nullptr;          // The stream to send debug messages to if enabled. Usually Serial.
    bool _printBadChecksum = false;       // Display bad checksum message from the parser
    bool _printParserTransitions = false; // Display the parser transitions
    bool _printRxMessages = false;        // Display the received message summary
    bool _dumpRxMessages = false;         // Display the received message hex dump
    static bool badNmeaChecksum(SEMP_PARSE_STATE *parse);

    // Command processing
    std::string commandName;                    // The specific command response the parser is hunting for
    uint8_t commandResponse = LG290P_RESULT_OK; // Gets EOM result from parser
    NmeaPacket pqtmResponse;                    // If found, parse puts resulting $PQTM sentence here
    bool transmit(const char *command, const char *parms);

    // Parsing support
    static void LG290PProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type);
    void nmeaHandler(SEMP_PARSE_STATE *parse);
    void rtcmHandler(SEMP_PARSE_STATE *parse);
    HardwareSerial *_hwSerialPort = nullptr;
    PvtDomain pvtDomain;
    RtcmDomain rtcmDomain;
    EpeDomain epeDomain;
    PlDomain plDomain;
    SvinStatusDomain svinStatusDomain;

    // Serial port utilities
    uint16_t serialAvailable();
    uint8_t serialRead();
    void serialPrintln(const char *command);
    void clearBuffer();

    // Satellite reporting
    std::map<std::string /* talker id */, unsigned long /* millis */> satelliteUpdateTime;
    std::map<std::string /* talker id */, std::set<satinfo>>
        satelliteStaging; // Staging the GSV reports for each Talker
    std::map<std::string /* talker id */, std::set<satinfo>> satelliteDomain;
    bool hasNewSatellites = false;
    unsigned long lastSatUpdate = 0UL;
};

#endif //_SPARKFUN_LG290P_GNSS_ARDUINO_LIBRARY_H
