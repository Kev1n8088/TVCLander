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

#include "SparkFun_LG290P_GNSS.h"

//----------------------------------------
// Constants
//----------------------------------------

// parserTable index values
#define LG290P_NMEA_PARSER_INDEX 0
#define LG290P_RTCM_PARSER_INDEX 1

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] = {
    sempNmeaPreamble,
    sempRtcmPreamble,
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char *const parserNames[] = {
    "LG290P NMEA Parser",
    "LG290P RTCM Parser",
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

// Account for the largest message
#define BUFFER_LENGTH 3000

//----------------------------------------
// Globals
//----------------------------------------

LG290P *ptrLG290P = nullptr; // Global pointer for external parser access into library class

//----------------------------------------
// Parser support routines
//----------------------------------------

// Enable the display of bad checksum messages from the parser
void LG290P::enablePrintBadChecksums()
{
    _printBadChecksum = true;
}

// Disable the display of bad checksum messages from the parser
void LG290P::disablePrintBadChecksums()
{
    _printBadChecksum = false;
}

// Translate the state value into an ASCII state name
const char *LG290PGetStateName(SEMP_PARSE_STATE *parse)
{
    const char *name;
    if ((name = sempNmeaGetStateName(parse)) != nullptr)
        return name;
    if ((name = sempRtcmGetStateName(parse)) != nullptr)
        return name;
    return sempGetStateName(parse);
}

// Disable debug output from the parser
void LG290P::disableParserDebug()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserDebug(Print *print)
{
    sempEnableDebugOutput(_sempParse, print);
}

// Disable debug output from the parser
void LG290P::disableParserErrors()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void LG290P::enableParserErrors(Print *print)
{
    sempEnableErrorOutput(_sempParse, print);
}

// Print the LG290P parser configuration
void LG290P::printParserConfiguration(Print *print)
{
    sempPrintParserConfiguration(_sempParse, print);
}

bool LG290P::badNmeaChecksum(SEMP_PARSE_STATE *parse)
{
    return false;
}

//----------------------------------------
// LG290P support routines
//----------------------------------------

bool LG290P::begin(HardwareSerial &serialPort, Print *parserDebug, Print *parserError)
{
    ptrLG290P = this;
    _hwSerialPort = &serialPort;

    // Initialize the parser
    _sempParse =
        sempBeginParser(parserTable, parserCount, parserNames, parserNameCount, 0, BUFFER_LENGTH, LG290PProcessMessage,
                        "SFE_LG290P_GNSS_Library", parserError, parserDebug, badNmeaChecksum);
    if (!_sempParse)
    {
        debugPrintf("LG290P Lib: Failed to initialize the parser!");
        return false;
    }

    bool ok = isConnected();
    ok = ok && getMode(devState.mode);
    ok = ok && scanForMsgsEnabled();
    ok = ok && getFirmwareVersion(firmwareVersion);

    if (ok)
    {
        debugPrintf("Firmware version is %02d%s", firmwareVersion, firmwareVersion == 0 ? " (Unknown)" : "");
        debugPrintf("Starting with %s mode, GGA %d RMC %d EPE %d PVT %d PL %d SVIN %d GSV %d GST %d", 
            devState.mode == BASEMODE ? "BASE" : "ROVER", devState.ggaRate, devState.rmcRate, devState.epeRate, 
            devState.pvtRate, devState.plRate, devState.svinstatusRate, devState.gsvRate, devState.gstRate);
    }
    else
    {
        debugPrintf("begin() failed.");
        sempStopParser(&_sempParse);
    }
    
    return ok;
}

bool LG290P::beginAutoBaudDetect(HardwareSerial &serialPort, int rxPin, int txPin, Print *parserDebug /* = nullptr */, Print *parserError /* = &Serial */)
{
    //serialPort.setRxBufferSize(4096);

    for (int baud: {460800, 921600, 230400, 115200, 9600})
    {
        debugPrintf("Trying baud rate %d...", baud);
        //serialPort.begin(baud, SERIAL_8N1, rxPin, txPin);
        serialPort.begin(baud);
        if (begin(serialPort, parserDebug, parserError))
            return true;
    }
    return false;
}

// Query the device with 'UNIQID', expect OK response
// Device may be booting and outputting other messages (ie, PQTMVER)
// Try a few times
bool LG290P::isConnected()
{
    // Try up to 10 seconds
    for (unsigned long start = millis(); millis() - start < 10000;)
    {
        if (sendOkCommand("PQTMUNIQID"))
            return true;
    }
    return false;
}

// If another task outside of this library is accessing the same Serial hardware, it can
// check to see if this library currently needs exclusive read/write access for a short period.
// If isBlocking is true, external consumers should not read/write to the Serial hardware
bool LG290P::isBlocking()
{
    return lg290PLibrarySemaphoreBlock;
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void LG290P::enableDebugging(Print &debugPort)
{
    _debugPort = &debugPort;
}

void LG290P::disableDebugging()
{
    _debugPort = nullptr;
}

// Check for new data until there is no more
bool LG290P::update()
{
    bool newData = false;

    lg290PLibrarySemaphoreBlock = true; // Allow external tasks to control serial hardware

    newData = serialAvailable(); // Check if new data needs parsing

    while (serialAvailable())
        update(serialRead()); // Pass character from Serial stream to parser

    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return newData;
}

// Process a buffer of bytes. Allows a stream outside of library to feed the library.
bool LG290P::update(uint8_t* incomingBuffer, uint16_t bufferLength)
{
    bool newData = false;

    lg290PLibrarySemaphoreBlock = true; // Allow external tasks to control serial hardware

    newData = serialAvailable(); // Check if new data needs parsing

    for (int x = 0; x < bufferLength; x++)
        update(incomingBuffer[x]);

    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return newData;
}

// Process a given byte into the SEMP
// LG290PProcessMessage() is called once the parser completes on a line
bool LG290P::update(byte incoming)
{
    const char *endName;
    const char *startName = nullptr;
    SEMP_PARSE_ROUTINE startState;

    // Get the current state and state name
    if (_printParserTransitions)
    {
        startState = _sempParse->state;
        startName = LG290PGetStateName(_sempParse);
    }

    // Update the parser state based on the incoming byte
    sempParseNextByte(_sempParse, incoming);

    // Get the current state name
    if (_printParserTransitions)
    {
        endName = LG290PGetStateName(_sempParse);

        // Display the parser state transition
        debugPrintf("LG290P Lib: 0x%02x (%c), crc: 0x%08x, state: %s --> %s", incoming,
                    ((incoming >= ' ') && (incoming < 0x7f)) ? incoming : '.', _sempParse->crc, startName, endName);
    }

    (void)startState; // Fix pesky warning-as-error

    return true;
}

// Enable the display of parser transitions
void LG290P::enablePrintParserTransitions()
{
    _printParserTransitions = true;
}

// Enable the display of parser transitions
void LG290P::disablePrintParserTransitions()
{
    _printParserTransitions = false;
}

// Display the contents of a buffer
void LG290P::dumpBuffer(const uint8_t *buffer, uint16_t length)
{
    int bytes;
    const uint8_t *end;
    int index;
    char line[128];
    uint16_t offset;

    end = &buffer[length];
    offset = 0;
    while (buffer < end)
    {
        // Determine the number of bytes to display on the line
        bytes = end - buffer;
        if (bytes > (16 - (offset & 0xf)))
            bytes = 16 - (offset & 0xf);

        // Display the offset
        sprintf(line, "0x%08lx: ", (long unsigned int)offset);

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            sprintf(&line[strlen(line)], "   ");

        // Display the data bytes
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%02x ", buffer[index]);

        // Separate the data bytes from the ASCII
        for (; index < (16 - (offset & 0xf)); index++)
            sprintf(&line[strlen(line)], "   ");
        sprintf(&line[strlen(line)], " ");

        // Skip leading bytes
        for (index = 0; index < (offset & 0xf); index++)
            sprintf(&line[strlen(line)], " ");

        // Display the ASCII values
        for (index = 0; index < bytes; index++)
            sprintf(&line[strlen(line)], "%c",
                    ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
        debugPrintf("%s", line);

        // Set the next line of data
        buffer += bytes;
        offset += bytes;
    }
}

// Enable the display of received messages
void LG290P::enablePrintRxMessages()
{
    _printRxMessages = true;
}

// Disable the display of received messages
void LG290P::disablePrintRxMessages()
{
    _printRxMessages = false;
}

// Enable the hex dump of received messages
void LG290P::enableRxMessageDump()
{
    _dumpRxMessages = true;
}

// Disable the hex dump of received messages
void LG290P::disableRxMessageDump()
{
    _dumpRxMessages = false;
}

// Static callback from within parser, for end of message
// Process a complete message incoming from parser
void LG290P::LG290PProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
    if (ptrLG290P->_printRxMessages)
    {
        // Display the raw message
        ptrLG290P->debugPrintf("");
        switch (type)
        {
        case LG290P_NMEA_PARSER_INDEX:
            ptrLG290P->debugPrintf("LG290P Lib: Valid NMEA Sentence: %s, 0x%04x (%d) bytes",
                                   sempNmeaGetSentenceName(parse), parse->length, parse->length);
            break;

        case LG290P_RTCM_PARSER_INDEX:
            ptrLG290P->debugPrintf("LG290P Lib: Valid RTCM message: 0x%04x (%d) bytes", parse->length, parse->length);
            break;
        }
    }

    // Dump the contents of the parsed messages
    if (ptrLG290P->_dumpRxMessages)
        ptrLG290P->dumpBuffer(parse->buffer, parse->length);

    // Process the message
    switch (type)
    {
    case LG290P_RTCM_PARSER_INDEX:
        ptrLG290P->rtcmHandler(parse);
        break;
    case LG290P_NMEA_PARSER_INDEX:
        ptrLG290P->nmeaHandler(parse);
        break;
    }
}

// Commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Statistics (counting packets of various types that arrived)
int LG290P::getNmeaCount(const char *sentenceId /* = nullptr */)
{
    if (sentenceId != nullptr)
    {
        auto find = nmeaCounters.find(sentenceId);
        return find == nmeaCounters.end() ? 0 : find->second;
    }
    int sum = 0;
    for (auto &item : nmeaCounters)
        sum += item.second;
    return sum;
}

int LG290P::getRtcmCount(int packetType /* = -1 */)
{
    if (packetType != -1)
    {
        auto find = rtcmCounters.find(packetType);
        return find == rtcmCounters.end() ? 0 : find->second;
    }
    int sum = 0;
    for (auto &item : rtcmCounters)
        sum += item.second;
    return sum;
}

bool LG290P::setModeBase(bool resetAfter /* = true */)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",W,2");
    if (resetAfter)
        ret = ret && save() && reset();
    if (ret)
        devState.mode = BASEMODE;
    return ret;
}

bool LG290P::setModeRover(bool resetAfter)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",W,1");
    if (resetAfter)
        ret = ret && save() && reset();
    if (ret)
    {
        // In Rover mode the SVINSTATUS messages don't work, so clear the domain.
        svinStatusDomain.clear();
        devState.mode = ROVERMODE;
    }
    return ret;
}

bool LG290P::getMode(int &mode)
{
    bool ret = sendOkCommand("PQTMCFGRCVRMODE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        mode = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setPortBaudrate(int port, uint32_t newBaud, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%d", port, (int)newBaud);
    return sendOkCommand("PQTMCFGUART", parms, maxWaitMs);
}

// Set baud rate on current port
bool LG290P::setBaudrate(uint32_t newBaud, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", (int)newBaud);
    return sendOkCommand("PQTMCFGUART", parms, maxWaitMs);
}

bool LG290P::getPortInfo(int port, uint32_t &newBaud, uint8_t &databits, uint8_t &parity, uint8_t &stop,
                         uint8_t &flowControl, uint16_t maxWaitMs)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",R,%d", port);
    bool ret = sendCommand("PQTMCFGUART", parms, maxWaitMs);
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        newBaud = atol(packet[3].c_str());
        databits = atoi(packet[4].c_str());
        parity = atoi(packet[5].c_str());
        stop = atoi(packet[6].c_str());
        flowControl = atoi(packet[7].c_str());
    }
    return ret;
}

bool LG290P::setPortInputProtocols(int port, uint8_t newFlags)
{
    uint8_t inputFlags, outputFlags;
    if (!getPortProtocols(port, inputFlags, outputFlags))
        return false;

    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%d,%d", port, newFlags, outputFlags);
    return sendOkCommand("PQTMCFGPROT", parms);
}

bool LG290P::setPortOutputProtocols(int port, uint8_t newFlags)
{
    uint8_t inputFlags, outputFlags;
    if (!getPortProtocols(port, inputFlags, outputFlags))
        return false;

    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%d,%d", port, inputFlags, newFlags);
    return sendOkCommand("PQTMCFGPROT", parms);
}

bool LG290P::getPortProtocols(int port, uint8_t &inputFlags, uint8_t &outputFlags)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",R,1,%d", port);
    bool ok = sendOkCommand("PQTMCFGPROT", parms);
    if (ok)
    {
        auto packet = getCommandResponse();
        ok = packet[2] == "1";
        inputFlags = atoi(packet[4].c_str());
        outputFlags = atoi(packet[5].c_str());
    }
    return ok;
}

bool LG290P::setPPS(uint16_t duration, bool alwaysOutput, bool positivePolarity)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,1,%d,%d,%d,0", duration, alwaysOutput ? 1 : 2, positivePolarity);
    return sendOkCommand("PQTMCFGPPS", parms);
}

bool LG290P::disablePPS()
{
    return sendOkCommand("PQTMCFGPPS", ",W,1,0");
}

bool LG290P::getPPS(bool &enabled, uint16_t &duration, bool &alwaysOutput, bool &positivePolarity)
{
    bool ret = sendCommand("PQTMCFGPPS", ",R,1");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK" && packet[2] == "1";
        enabled = packet[3] == "1";
        duration = atoi(packet[4].c_str());
        alwaysOutput = packet[5] == "1";
        positivePolarity = packet[6] == "1";
    }
    return ret;
}

bool LG290P::getConstellations(bool &enableGPS, bool &enableGLONASS, bool &enableGalileo, bool &enableBDS,
                               bool &enableQZSS, bool &enableNavIC)
{
    bool ret = sendCommand("PQTMCFGCNST", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        enableGPS = packet[2] == "1";
        enableGLONASS = packet[3] == "1";
        enableGalileo = packet[4] == "1";
        enableBDS = packet[5] == "1";
        enableQZSS = packet[6] == "1";
        enableNavIC = packet[7] == "1";
    }
    return ret;
}

bool LG290P::getSerialNumber(std::string &serial)
{
    bool ret = sendCommand("PQTMUNIQID");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        serial = packet[3];
    }
    return ret;
}

bool LG290P::getVersionInfo(std::string &version, std::string &buildDate, std::string &buildTime)
{
    bool ret = sendCommand("PQTMVERNO");
    if (ret)
    {
        auto packet = getCommandResponse();
        version = packet[1];
        buildDate = packet[2];
        buildTime = packet[3];
    }
    return ret;
}

bool LG290P::getFirmwareVersion(int &version)
{
    version = 0; // Unknown

    std::string ver, buildDate, buildTime;
    bool ret = getVersionInfo(ver, buildDate, buildTime);
    if (ret && (ver.length() > strlen(firmwareVersionPrefix)))
    {
        char *spot = strstr(ver.c_str(), firmwareVersionPrefix);
        if (spot != NULL)
        {
            spot += strlen(firmwareVersionPrefix);
            version = atoi(spot);
            return (version > 0);
        }
    }

    return false;
}

bool LG290P::getFixInterval(uint16_t &fixInterval)
{
    bool ret = sendCommand("PQTMCFGFIXRATE", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        fixInterval = atoi(packet[2].c_str());
    }
    return ret;
}

bool LG290P::setFixInterval(uint16_t fixInterval)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", fixInterval);
    return sendOkCommand("PQTMCFGFIXRATE", parms) && hotStart();
}

bool LG290P::setMessageRate(const char *msgName, int rate, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",W,%s,%d" : ",W,%s,%d,%d", msgName, rate, msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);

    // We internally track whether certain important sentences are enabled
    if (ret)
    {
        std::string str = msgName;
        if (str == "GGA") devState.ggaRate = rate;
        else if (str == "RMC") devState.rmcRate = rate;
        else if (str == "PQTMPVT") devState.pvtRate = rate;
        else if (str == "PQTMPL") devState.plRate = rate;
        else if (str == "PQTMSVINSTATUS") devState.svinstatusRate = rate;
        else if (str == "PQTMEPE") devState.epeRate = rate;
        else if (str == "GSV") devState.gsvRate = rate;
        else if (str == "GST") devState.gstRate = rate;
    }
    return ret;
}

// Set a message on a given port. Available in v4 and above.
// $PQTMCFGMSGRATE,W,1,2,GGA,1* - <PortType>,<PortID>,<MsgName>,<Rate>[,MsgVersion/Offset]
// Set port type (1 = UART), UART number (2 = UART2), message, rate
bool LG290P::setMessageRateOnPort(const char *msgName, int rate, int portNumber, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",W,1,%d,%s,%d" : ",W,1,%d,%s,%d,%d", portNumber, msgName, rate, msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);

    // We internally track whether certain important sentences are enabled
    if (ret)
    {
        std::string str = msgName;
        if (str == "GGA") devState.ggaRate = rate;
        else if (str == "RMC") devState.rmcRate = rate;
        else if (str == "PQTMPVT") devState.pvtRate = rate;
        else if (str == "PQTMPL") devState.plRate = rate;
        else if (str == "PQTMSVINSTATUS") devState.svinstatusRate = rate;
        else if (str == "PQTMEPE") devState.epeRate = rate;
        else if (str == "GSV") devState.gsvRate = rate;
        else if (str == "GST") devState.gstRate = rate;
    }
    return ret;
}

bool LG290P::getMessageRate(const char *msgName, int &rate, int msgVer)
{
    char parms[50];
    snprintf(parms, sizeof parms, msgVer == -1 ? ",R,%s" : ",R,%s,%d", msgName, msgVer);
    bool ret = sendOkCommand("PQTMCFGMSGRATE", parms);
    if (ret)
    {
        auto packet = getCommandResponse();
        rate = atoi(packet[3].c_str());
    }
    return ret;
}

// Configures the elevation threshold for position engine. Available in v5 and above.
bool LG290P::setElevationAngle(int elevationAngle)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d", elevationAngle);
    return sendOkCommand("PQTMCFGELETHD", parms) && hotStart();
}

bool LG290P::getElevationAngle(int &elevationAngle)
{
    bool ret = sendCommand("PQTMCFGELETHD", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        elevationAngle = atoi(packet[2].c_str());
    }
    return ret;
}

// Configures the CNR threshold for position engine. Available in v5 and above.
bool LG290P::setCNR(float cnr)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%0.2f", cnr);
    return sendOkCommand("PQTMCFGCNRTHD", parms) && hotStart();
}

bool LG290P::getCNR(float &cnr)
{
    bool ret = sendCommand("PQTMCFGCNRTHD", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        cnr = atof(packet[2].c_str());
    }
    return ret;
}

// Configures the max differential age of RTK fix before the device will drop back to RTK float.
// Note: We assume Auto mode and Absolute mode when setting the timeout.
bool LG290P::setRtkDifferentialAge(uint16_t timeout)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,1,%d", timeout);
    return sendOkCommand("PQTMCFGRTK", parms) && hotStart();
}

bool LG290P::getRtkDifferentialAge(uint16_t &timeout)
{
    bool ret = sendCommand("PQTMCFGRTK", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        ret = packet[1] == "OK";
        timeout = atoi(packet[4].c_str());
    }
    return ret;
}

bool LG290P::scanForMsgsEnabled()
{
    bool ok = getMessageRate("GGA", devState.ggaRate);
    ok = ok && getMessageRate("RMC", devState.rmcRate);
    ok = ok && getMessageRate("PQTMEPE", devState.epeRate, 2);
    ok = ok && getMessageRate("PQTMPVT", devState.pvtRate, 1);
    ok = ok && getMessageRate("PQTMPL", devState.plRate, 1);
    ok = ok && getMessageRate("GSV", devState.gsvRate);

    // GST is not available on firmware < 4
    getMessageRate("GST", devState.gstRate);

    // this is a special message. getMessageRate might fail if in ROVER mode
    getMessageRate("PQTMSVINSTATUS", devState.svinstatusRate, 1);
    return ok;
}

void LG290P::ensureMsgEnabled(bool enabled, const char *msg, int msgVer /* = -1 */)
{
    if (!enabled)
    {
        debugPrintf("Forcing enable of %s sentence", msg);
        setMessageRate(msg, 1, msgVer);
    }
}

bool LG290P::nmeaSubscribe(const char *msgName, nmeaCallback callback)
{
    nmeaSubscriptions[msgName] = callback;
    return true;
}

bool LG290P::nmeaUnsubscribe(const char *msgName)
{
    nmeaSubscriptions.erase(msgName);
    return true;
}

bool LG290P::nmeaSubscribeAll(nmeaCallback callback)
{
    nmeaAllSubscribe = callback;
    return true;
}

bool LG290P::nmeaUnsubscribeAll()
{
    nmeaAllSubscribe = nullptr;
    return true;
}

bool LG290P::rtcmSubscribe(uint16_t type, rtcmCallback callback)
{
    rtcmSubscriptions[type] = callback;
    return true;
}

bool LG290P::rtcmUnsubscribe(uint16_t type)
{
    rtcmSubscriptions.erase(type);
    return true;
}

bool LG290P::rtcmSubscribeAll(rtcmCallback callback)
{
    rtcmAllSubscribe = callback;
    return true;
}

bool LG290P::rtcmUnsubscribeAll()
{
    rtcmAllSubscribe = nullptr;
    return true;
}

void LG290P::clearAll()
{
    pvtDomain.clear();
    satelliteDomain.clear();
    nmeaCounters.clear();
    rtcmCounters.clear();
    epeDomain.clear();
    plDomain.clear();
    svinStatusDomain.clear();
}

bool LG290P::genericReset(const char *resetCmd)
{
    clearAll();

    // Do a software reset, wait for reconnection, then rescan which messages are enabled
    return sendCommandNoResponse(resetCmd) && isConnected() && scanForMsgsEnabled();
}

bool LG290P::setConstellations(bool enableGPS, bool enableGLONASS, bool enableGalileo, bool enableBDS, bool enableQZSS,
                               bool enableNavIC)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,%d,%d,%d,%d,%d,%d", enableGPS, enableGLONASS, 
        enableGalileo, enableBDS, enableQZSS, enableNavIC);
    return sendOkCommand("PQTMCFGCNST", parms) && save() && hotStart();
}

bool LG290P::disableEngine()
{
    return sendOkCommand("PQTMGNSSSTOP");
}

bool LG290P::enableEngine()
{
    return sendOkCommand("PQTMGNSSSTART");
}

bool LG290P::save()
{
    return sendOkCommand("PQTMSAVEPAR");
}

bool LG290P::factoryRestore()
{
    return sendOkCommand("PQTMRESTOREPAR");
}

// Abstraction of the serial interface
// Useful if we ever need to support SoftwareSerial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Enable printfs to various endpoints
// https://stackoverflow.com/questions/42131753/wrapper-for-printf
void LG290P::debugPrintf(const char *format, ...)
{
    if (_debugPort == nullptr)
        return;

    va_list args;
    va_start(args, format);

    va_list args2;
    va_copy(args2, args);
    char buf[vsnprintf(nullptr, 0, format, args) + sizeof("\r\n")];

    vsnprintf(buf, sizeof buf, format, args2);

    // Add CR+LF
    buf[sizeof(buf) - 3] = '\r';
    buf[sizeof(buf) - 2] = '\n';
    buf[sizeof(buf) - 1] = '\0';

    _debugPort->write(buf, strlen(buf));

    va_end(args);
    va_end(args2);
}

// Discards any characters sitting in RX buffer
void LG290P::clearBuffer()
{
    while (serialAvailable())
        serialRead();
}

uint16_t LG290P::serialAvailable()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->available());
    }
    return (0);
}

uint8_t LG290P::serialRead()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->read());
    }
    return (0);
}

// Correctly format command $PQTM with $ and checksum,
// then send it!
bool LG290P::transmit(const char *command, const char *parms)
{
    if (!_hwSerialPort || command == nullptr)
        return false;

    std::string cmd = command;
    cmd += parms;
    if (cmd.empty())
        return false;

    // Prepend $ if lacking
    if (cmd[0] != '$')
        cmd = "$" + cmd;

    // Calculate NMEA checksum
    uint8_t chk = 0;
    for (int i = 1; i < cmd.length(); ++i)
        chk ^= cmd[i];

    // Append checksum
    char buf[32];
    sprintf(buf, "*%02X", chk);
    cmd += buf;

    // send with newlines
    debugPrintf("...transmit '%s'", cmd.c_str());
    _hwSerialPort->print(cmd.c_str());
    _hwSerialPort->print("\r\n");

    return true;
}

// Send a $PQTM query string to the LG290P and wait for response
// If you want to see the response string, you can get it from getCommandResponse()
bool LG290P::sendCommand(const char *command, const char *parms, uint16_t maxWaitMs, bool waitForResponse)
{
    if (lg290PLibrarySemaphoreBlock)
        return false;

    bool success = false;

    debugPrintf("sendCommand(\"%s\", \"%s\")", command, parms);
    commandName = command[0] == '$' ? command + 1 : command;
    commandResponse = LG290P_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    lg290PLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // add $ and checksum and transmit
    if (!transmit(command, parms))
        return false;

    if (waitForResponse)
        debugPrintf("...sendCommand: waiting for response for %s", command);

    // Feed the parser until we see a response to the command
    for (unsigned long start = millis(); millis() - start < maxWaitMs;)
    {
        update(); // Will call LG290PProcessMessage()

        if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_OK)
        {
            debugPrintf("...sendCommand: Matching Response received");
            success = true;
            break;
        }

        if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("...sendCommand: Matching response NOT received");
            break;
        }
    }

    if (commandResponse == LG290P_RESULT_RESPONSE_COMMAND_WAITING)
    {
        if (waitForResponse)
            debugPrintf("...sendCommand: TIMEOUT: no response received");
        else
            success = true;
    }
    commandName.clear();
    lg290PLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
    return success;
}

bool LG290P::sendCommandNoResponse(const char *command, uint16_t maxWaitMs)
{
    return sendCommand(command, "", maxWaitMs, false);
}

bool LG290P::sendCommandLine(const char *commandline, uint16_t maxWaitMs)
{
    std::string cmdline = commandline;
    size_t commaPos = cmdline.find(',');
    std::string cmd = commaPos == cmdline.npos ? cmdline : cmdline.substr(0, commaPos);
    std::string parms = commaPos == cmdline.npos ? "" : cmdline.substr(commaPos);

    return sendCommand(cmd.c_str(), parms.c_str(), maxWaitMs);
}

bool LG290P::sendOkCommand(const char *command, const char *parms, uint16_t maxWaitMs)
{
    if (!sendCommand(command, parms, maxWaitMs))
        return false;

    NmeaPacket &response = getCommandResponse();
    bool okFound = response[1] == "OK";

    debugPrintf("...sendOkCommand, OK %sfound", okFound ? "" : "NOT ");

    return okFound;
}

std::list<LG290P::satinfo> LG290P::getVisibleSats(const char *talker /* = nullptr */)
{
    std::list<LG290P::satinfo> ret;

    ensureGsvEnabled();

    // Get all the satellites visible?
    if (talker == nullptr)
    {
        for (auto &item : satelliteDomain)
            ret.insert(ret.end(), item.second.begin(), item.second.end());
    }

    // ... or just the ones from a specific Talker?
    else 
    {
        auto item = satelliteDomain.find(talker);
        if (item != satelliteDomain.end())
            ret.insert(ret.end(), item->second.begin(), item->second.end());
    }
    return ret;
}

bool LG290P::getSurveyDetails(int &mode, int &positionTimes, double &accuracyLimit, double &ecefX, double &ecefY,
                             double &ecefZ)
{
    bool ret = sendOkCommand("PQTMCFGSVIN", ",R");
    if (ret)
    {
        auto packet = getCommandResponse();
        mode = atoi(packet[2].c_str());
        positionTimes = atof(packet[3].c_str());
        accuracyLimit = atof(packet[4].c_str());
        ecefX = atof(packet[5].c_str());
        ecefY = atof(packet[6].c_str());
        ecefZ = atof(packet[7].c_str());
    }
    return ret;
}

uint8_t LG290P::getSurveyMode()
{
    if (sendOkCommand("PQTMCFGSVIN", ",R"))
    {
        auto packet = getCommandResponse();
        return (uint8_t)atoi(packet[2].c_str());
    }
    return SURVEYDISABLED;
}

bool LG290P::setSurveyInMode(int positionTimes, double accuracyLimit /* = 0 */, bool resetAfter /* = true */)
{
    char parms[50];
    snprintf(parms, sizeof parms, ",W,1,%d,%f,0.00,0.00,0.00", positionTimes, accuracyLimit);
    bool ok = sendOkCommand("PQTMCFGSVIN", parms);
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}

bool LG290P::setSurveyFixedMode(double ecefX, double ecefY, double ecefZ, bool resetAfter /* = true */)
{
    char parms[100];
    snprintf(parms, sizeof parms, ",W,2,0,0,%f,%f,%f", ecefX, ecefY, ecefZ);
    bool ok = sendOkCommand("PQTMCFGSVIN", parms);
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}

bool LG290P::disableSurveyInMode(bool resetAfter /* = true */)
{
    bool ok = sendOkCommand("PQTMCFGSVIN", ",W,0,0,0,0,0,0");
    if (resetAfter)
        ok = ok && save() && reset();
    return ok;
}


// Main handler and RAM inits
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Cracks an NMEA sentence into the applicable container
// The sentence might be a normal NMEA one like $GPGGA or a custom one like $PQTMVER. In
// the former case, the first term is cracked into the "Talker Id" (GP) and "Sentence Id" (GGA)
void LG290P::nmeaHandler(SEMP_PARSE_STATE *parse)
{
    // Is this a command response?
    std::string sentence = (const char *)parse->buffer;
    if (sentence.length() >= 2 && sentence.substr(sentence.length() - 2) == "\r\n")
        // if (sentence.ends_with("\r\n"))
        sentence.erase(sentence.length() - 2);

    NmeaPacket nmea = NmeaPacket::FromString(sentence);
    auto id = nmea.SentenceId();

    // Do $PQTM strings later
    if (sentence.substr(0, 2) != "$P")
    {
        if (nmea.IsValid())
        {
            if (nmea.ChecksumValid())
            {
                if (id == "RMC")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    nmea.processRMC(ptrLG290P->pvtDomain);
                }
                
                else if (id == "GGA")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    nmea.processGGA(ptrLG290P->pvtDomain);
                }

                else if (id == "GST")
                {
                    lastUpdatePvtDomain = millis(); // Update stale marker
                    PvtDomain &pvt = ptrLG290P->pvtDomain;
                    pvt.rmsPseudorangeResidual = strtod(nmea[2].c_str(), NULL);
                    pvt.latitudeError = strtod(nmea[6].c_str(), NULL);
                    pvt.longitudeError = strtod(nmea[7].c_str(), NULL);
                    pvt.heightError = strtod(nmea[8].c_str(), NULL);
                }

                else if (id == "GSV")
                {
                    uint16_t msgNo = (uint16_t)strtoul(nmea[2].c_str(), NULL, 10);
                    uint16_t svsInView = (uint16_t)strtoul(nmea[3].c_str(), NULL, 10);
                    std::string talker = nmea.TalkerId();
                    unsigned long now = millis();
                    auto &thisSet = satelliteStaging[talker];

                    // should we publish the latest batch of satellites for this talker id?
                    if (msgNo == 1 && now - satelliteUpdateTime[talker] > 900)
                    {
                        hasNewSatellites = true;
                        satelliteDomain[talker] = thisSet;
                        thisSet.clear();
                        satelliteUpdateTime[talker] = now;
                    }
                    
                    // log_d("GSV: count: %d no: %d in view: %d", msgCount, msgNo, svsInView);
                    for (int i = 0; i < 4 && 4 * (msgNo - 1) + i < svsInView; ++i)
                    {
                        satinfo sat;
                        sat.prn = (uint16_t)strtoul(nmea[4 + 4 * i].c_str(), NULL, 10);
                        sat.elev = (uint16_t)strtoul(nmea[5 + 4 * i].c_str(), NULL, 10);
                        sat.azimuth = (uint16_t)strtoul(nmea[6 + 4 * i].c_str(), NULL, 10);
                        sat.snr = (uint16_t)strtoul(nmea[7 + 4 * i].c_str(), NULL, 10);
                        strncpy(sat.talker, talker.substr(0, 2).c_str(), sizeof sat.talker);
                        thisSet.insert(sat);
                    }
                }
            }
            else
            {
                ptrLG290P->debugPrintf("BAD CHECKSUM: %s\r\n", sentence.c_str());
            }
        }
        else
        {
            ptrLG290P->debugPrintf("NOT VALID: %s\r\n", sentence.c_str());
        }
    }

    // Handle $PQTM response messages
    else
    {
        // Is this the response to a command we issued with sendCommand?
        if (id == ptrLG290P->commandName)
        {
            // Display the command response
            ptrLG290P->debugPrintf("...sendCommand: (main loop) Expected command response found %s", parse->buffer);
            ptrLG290P->pqtmResponse = nmea;
            ptrLG290P->commandResponse = LG290P_RESULT_RESPONSE_COMMAND_OK;
        }

        else if (id == "PQTMPVT")
        {   
            lastUpdatePvtDomain = millis(); // Update stale marker
            PvtDomain &pvt = ptrLG290P->pvtDomain;
            pvt.timeOfWeek = atoi(nmea[2].c_str());
            uint32_t d = atoi(nmea[3].c_str());
            pvt.year = d / 10000;
            pvt.month = (d / 100) % 100;
            pvt.day = d % 100;
            NmeaPacket::parseTime(nmea[4], pvt.hour, pvt.minute, pvt.second, pvt.nanosecond);
            // 5 is reserved
            // 6 is fix type -- maybe don't use because more limited than GGA?
            // pvt.quality = nmea[6].empty() ? '0' : nmea[6][0];
            pvt.satellitesUsed = atoi(nmea[7].c_str());
            pvt.leapSeconds = atoi(nmea[8].c_str());
            pvt.latitude = atof(nmea[9].c_str());
            pvt.longitude = atof(nmea[10].c_str());
            pvt.altitude = atof(nmea[11].c_str());
            pvt.geoidalSeparation = atof(nmea[12].c_str());
            pvt.nvelocity = atof(nmea[13].c_str());
            pvt.evelocity = atof(nmea[14].c_str());
            pvt.dvelocity = atof(nmea[15].c_str());
            pvt.groundSpeed = atof(nmea[16].c_str());
            pvt.course = atof(nmea[17].c_str());
            pvt.hdop = atof(nmea[18].c_str());
            pvt.pdop = atof(nmea[19].c_str());
            pvtDomain.newDataAvailable = true;
        }

        else if (id == "PQTMSVINSTATUS")
        {
            svinStatusDomain.validity = atoi(nmea[3].c_str());
            svinStatusDomain.observations = atoi(nmea[6].c_str());
            svinStatusDomain.cfgDur = atoi(nmea[7].c_str());
            svinStatusDomain.meanX = atof(nmea[8].c_str());
            svinStatusDomain.meanY = atof(nmea[9].c_str());
            svinStatusDomain.meanZ = atof(nmea[10].c_str());
            svinStatusDomain.meanAcc = atof(nmea[11].c_str());
        }

        else if (id == "PQTMEPE")
        {
            // handle 5 components of Horizontal Position Accuracy
            epeDomain.errorNorth = atof(nmea[2].c_str());
            epeDomain.errorEast = atof(nmea[3].c_str());
            epeDomain.errorDown = atof(nmea[4].c_str());
            epeDomain.error2D = atof(nmea[5].c_str());
            epeDomain.error3D = atof(nmea[6].c_str());
        }

        else if (id == "PQTMPL")
        {
            plDomain.probUncertainty = atof(nmea[3].c_str());
            plDomain.protectionLevelNorth = atof(nmea[6].c_str());
            plDomain.protectionLevelEast = atof(nmea[7].c_str());
            plDomain.protectionLevelDown = atof(nmea[8].c_str());
            plDomain.protectionLevelNorthVelocity = atof(nmea[9].c_str());
            plDomain.protectionLevelEastVelocity = atof(nmea[10].c_str());
            plDomain.protectionLevelDownVelocity = atof(nmea[11].c_str());
            plDomain.protectionLevelTime = atof(nmea[14].c_str());
        }

#if false
        else if (!ptrLG290P->commandName.empty())
        {
            // Display the command response
            ptrLG290P->debugPrintf("LG290P Lib: Unknown command response: %s", parse->buffer);
            ptrLG290P->debugPrintf("LG290P Lib: Looking for command: %s", ptrLG290P->commandName.c_str());
        }
#endif
    }

    if (id != "")
    {
        nmeaCounters[id]++;

        // Is there a callback registered for this id?
        auto iterator = nmeaSubscriptions.find(id);
        if (iterator != nmeaSubscriptions.end())
            (*iterator->second)(nmea); // call it!
    }

    // Is there a general callback registered?
    if (nmeaAllSubscribe != nullptr)
        nmeaAllSubscribe(nmea); // call it!
}

int64_t RtcmPacket::extract_38bit_signed(int bit_offset)
{
    // Extract 38-bit value starting from the given bit_offset
    int64_t value = 0;
    int byte_offset = bit_offset / 8;
    int bit_in_byte = bit_offset % 8;

    // We need to grab up to 6 bytes and mask out the unused 2 bits.
    for (int i = 0; i < 6; ++i)
    {
      value <<= 8;
      value |= (int64_t)(payload[byte_offset + i]);
    }

    // Shift right to discard the unwanted bits and align to 38-bit value
    value >>= (10 - bit_in_byte);

    // Mask to keep only 38 bits
    value &= 0x3FFFFFFFFFLL; // 38-bit mask (0x3FFFFFFFFF is 38 ones in binary)

    // Check if the sign bit (38th bit) is set and extend the sign for 64-bit int
    if (value & (1LL << 37))
    {
        value |= ~((1LL << 38) - 1); // Extend the sign
    }

    return value;
}

/* static */
bool RtcmPacket::FromBuffer(uint8_t *buffer, size_t bufferLen, RtcmPacket &result)
{
    bool good = bufferLen > 6 && buffer[0] == 0xD3;
    if (good)
    {
        result.payloadLen = (buffer[1] << 8) | buffer[2];
        good = result.payloadLen + 6 == bufferLen;
    }

    if (good)
    {
        result.type = (buffer[3] << 4) | (buffer[4] >> 4);
        result.buffer = buffer;
        result.bufferlen = bufferLen;
        result.payload = buffer + 3;
    }
    return good;
}

// Cracks an RTCM packet into the applicable container
void LG290P::rtcmHandler(SEMP_PARSE_STATE *parse)
{
    RtcmPacket packet;
    if (RtcmPacket::FromBuffer(parse->buffer, parse->length, packet))
    {
        rtcmCounters[packet.type]++;

        if (packet.type == 1005)
        {
            rtcmDomain.ecefX = packet.getEcefX();
            rtcmDomain.ecefY = packet.getEcefY();
            rtcmDomain.ecefZ = packet.getEcefZ();
            lastUpdateEcef = millis();
        }

        // handle specific and general callbacks
        // If user is subscribed for this packet, call the callback
        if (rtcmSubscriptions.count(packet.type) > 0)
            rtcmSubscriptions[packet.type](packet);
        if (rtcmAllSubscribe != nullptr)
            rtcmAllSubscribe(packet);
    }
}

// All the general gets and sets
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

bool LG290P::isNewSnapshotAvailable()
{
    ensurePvtEnabled();
    bool r = pvtDomain.newDataAvailable;
    pvtDomain.newDataAvailable = false;
    return r;
}

bool LG290P::isNewSatelliteInfoAvailable()
{
    ensureGsvEnabled();
    bool r = hasNewSatellites;
    hasNewSatellites = false;
    return r;
}

uint16_t LG290P::getSatellitesInViewCount()
{
    uint16_t count = 0;
    for (auto &item : satelliteDomain)
        count += item.second.size();
    return count;
}

// Return the number of millis since last update
uint32_t LG290P::getPVTDomainAgeMs()
{
    return millis() - lastUpdatePvtDomain;
}

/* static */
void LG290P::geodeticToEcef(double lat, double lon, double alt, double &xOut, double &yOut, double &zOut)
{
    const double WGS84_A = 6378137;           // https://geographiclib.sourceforge.io/html/Constants_8hpp_source.html
    const double WGS84_E = 0.081819190842622; // http://docs.ros.org/en/hydro/api/gps_common/html/namespacegps__common.html
                                              // and https://gist.github.com/uhho/63750c4b54c7f90f37f958cc8af0c718
    double clat = cos(lat * DEG_TO_RAD);
    double slat = sin(lat * DEG_TO_RAD);
    double clon = cos(lon * DEG_TO_RAD);
    double slon = sin(lon * DEG_TO_RAD);

    double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);

    xOut = (N + alt) * clat * clon;
    yOut = (N + alt) * clat * slon;
    zOut = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
}

// Convert ECEF to LLH (geodetic)
// From: https://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
/* static */
void LG290P::ecefToGeodetic(double x, double y, double z, double &latOut, double &lonOut, double &altOut)
{
    double a = 6378137.0;              // WGS-84 semi-major axis
    double e2 = 6.6943799901377997e-3; // WGS-84 first eccentricity squared
    double a1 = 4.2697672707157535e+4; // a1 = a*e2
    double a2 = 1.8230912546075455e+9; // a2 = a1*a1
    double a3 = 1.4291722289812413e+2; // a3 = a1*e2/2
    double a4 = 4.5577281365188637e+9; // a4 = 2.5*a2
    double a5 = 4.2840589930055659e+4; // a5 = a1+a3
    double a6 = 9.9330562000986220e-1; // a6 = 1-e2

    double zp, w2, w, r2, r, s2, c2, s, c, ss;
    double g, rg, rf, u, v, m, f, p;

    zp = abs(z);
    w2 = x * x + y * y;
    w = sqrt(w2);
    r2 = w2 + z * z;
    r = sqrt(r2);
    lonOut = atan2(y, x); // Lon (final)

    s2 = z * z / r2;
    c2 = w2 / r2;
    u = a2 / r;
    v = a3 - a4 / r;
    if (c2 > 0.3)
    {
        s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
        latOut = asin(s); // Lat
        ss = s * s;
        c = sqrt(1.0 - ss);
    }
    else
    {
        c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
        latOut = acos(c); // Lat
        ss = 1.0 - c * c;
        s = sqrt(ss);
    }

    g = 1.0 - e2 * ss;
    rg = a / sqrt(g);
    rf = a6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    f = c * u + s * v;
    m = c * v - s * u;
    p = m / (rf / g + f);
    latOut = latOut + p;        // Lat
    altOut = f + m * p / 2.0; // Altitude
    if (z < 0.0)
    {
        latOut *= -1.0; // Lat
    }

    latOut *= RAD_TO_DEG; // Convert to degrees
    lonOut *= RAD_TO_DEG;
}

NmeaPacket NmeaPacket::FromString(const std::string &str)
{
    NmeaPacket s;
    s.charCount = str.length();
    size_t start = 0;
    size_t delimiterPos = str.find_first_of(",*");
    unsigned long calculatedChksum = 0;
    //log_d("Parsing sentence %s", str.c_str());

    while (true)
    {
        std::string token =
            delimiterPos != std::string::npos ? str.substr(start, delimiterPos - start) : str.substr(start);
        s.fields.push_back(token);
        for (auto c : token.substr(s.fields.size() == 1 && s.fields[0].substr(0, 1) == "$" ? 1 : 0))
            calculatedChksum ^= c;

        if (delimiterPos == std::string::npos)
        {
            s.hasChecksum = s.checksumValid = false;
            // log_d("Doesn't have any * checksum");
            break;
        }

        if (str[delimiterPos] == '*')
        {
            token = str.substr(delimiterPos + 1);
            s.fields.push_back(token);
            if (token.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos)
            {
                s.hasChecksum = s.checksumValid = false;
                // log_e("Bad format checksum: '%s'", token.c_str());
                // log_e("Sentence was '%s'", str.c_str());
            }
            else
            {
                unsigned long suppliedChecksum = strtoul(token.c_str(), NULL, 16);
                s.hasChecksum = true;
                s.checksumValid = suppliedChecksum == calculatedChksum;
            }
            break;
        }

        calculatedChksum ^= ',';
        start = delimiterPos + 1;
        delimiterPos = str.find_first_of(",*", start);
    }

    return s;
}

std::string NmeaPacket::ToString() const
{
    std::string str;
    for (int i = 0; i < fields.size(); ++i)
    {
        str += fields[i];
        if (i != fields.size() - 1)
            str += i == fields.size() - 2 ? '*' : ',';
    }
    return str;
}

std::string NmeaPacket::TalkerId() const
{
    if (!IsValid() || fields[0].empty() || fields[0][0] != '$')
        return "";

    // The Talker Id is the first two letters following the $ (four if PQTM)
    return fields[0].substr(0, 5) == "$PQTM" ? "" : fields[0].substr(1, 2);
}

std::string NmeaPacket::SentenceId() const
{
    if (!IsValid() || fields[0].empty() || fields[0][0] != '$')
        return "";

    // The Sentence Id is everything after the Talker Id, unless it's a PQTM 
    return fields[0].substr(0, 5) == "$PQTM" ? fields[0].substr(1) : fields[0].length() >= 3 ? fields[0].substr(3) : "";
}

void NmeaPacket::processGGA(PvtDomain &snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot.hour, snapshot.minute, snapshot.second, snapshot.nanosecond);
        NmeaPacket::parseLocation(fields[2], fields[3], fields[4], fields[5], snapshot.latitude, snapshot.longitude);
        NmeaPacket::parseQuality(fields[6], snapshot.quality);
        NmeaPacket::parseSatelliteCount(fields[7], snapshot.satellitesUsed);
        NmeaPacket::parseHdop(fields[8].c_str(), snapshot.hdop);
        NmeaPacket::parseAltitude(fields[9].c_str(), snapshot.altitude);
        snapshot.newDataAvailable = true;
    }
}

void NmeaPacket::processRMC(PvtDomain &snapshot)
{
    if (fields.size() >= 10)
    {
        NmeaPacket::parseTime(fields[1], snapshot.hour, snapshot.minute, snapshot.second, snapshot.nanosecond);
        NmeaPacket::parseFixStatus(fields[2], snapshot.fixStatus);
        NmeaPacket::parseLocation(fields[3], fields[4], fields[5], fields[6], snapshot.latitude, snapshot.longitude);
        NmeaPacket::parseSpeed(fields[7].c_str(), snapshot.groundSpeed);
        NmeaPacket::parseCourse(fields[8].c_str(), snapshot.course);
        NmeaPacket::parseDate(fields[9], snapshot.year, snapshot.month, snapshot.day);
        snapshot.newDataAvailable = true;
    }
}

void NmeaPacket::parseTime(const std::string &term, uint8_t &hour, uint8_t &minute, uint8_t &second, uint32_t &nanos)
{
    int32_t v = parseDecimal(term);
    hour = v / 1000000;
    minute = (v / 10000) % 100;
    second = (v / 100) % 100;
    nanos = 10000000 * (v % 100);
}

int32_t NmeaPacket::parseDecimal(const std::string &term)
{
    const char *t = term.c_str();
    bool negative = *t == '-';
    if (negative)
        ++t;
    int32_t ret = 100 * (int32_t)atol(t);
    while (isdigit(*t))
        ++t;
    if (*t == '.' && isdigit(t[1]))
    {
        ret += 10 * (t[1] - '0');
        if (isdigit(t[2]))
            ret += t[2] - '0';
    }
    return negative ? -ret : ret;
}

void NmeaPacket::parseDate(const std::string &term, uint16_t &year, uint8_t &month, uint8_t &day)
{
    uint32_t v = atol(term.c_str());
    year = 2000 + v % 100;
    month = (v / 100) % 100;
    day = v / 10000;
}

void NmeaPacket::parseLocation(const std::string &termLat, const std::string &termNS, const std::string &termLong,
                               const std::string &termEW, double &latitude, double &longitude)
{
    parseDegrees(termLat.c_str(), termNS.c_str(), latitude);
    parseDegrees(termLong.c_str(), termEW.c_str(), longitude);
}

void NmeaPacket::parseDegrees(const char *degTerm, const char *nsewTerm, double &angle)
{
    uint32_t leftOfDecimal = (uint32_t)atol(degTerm);
    uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
    uint32_t multiplier = 10000000UL;
    uint32_t tenMillionthsOfMinutes = minutes * multiplier;
    uint16_t deg = (int16_t)(leftOfDecimal / 100);
    uint32_t billionths = 0;
    bool negative = false;

    while (isdigit(*degTerm))
        ++degTerm;

    if (*degTerm == '.')
        while (isdigit(*++degTerm))
        {
            multiplier /= 10;
            tenMillionthsOfMinutes += (*degTerm - '0') * multiplier;
        }

    billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
    negative = *nsewTerm == 'W' || *nsewTerm == 'S';
    angle = deg + billionths / 1000000000.0;
    if (negative)
        angle = -angle;
}

void NmeaPacket::parseQuality(const std::string &term, char &quality)
{
    quality = term.empty() ? '0' : term[0];
}

void NmeaPacket::parseSatelliteCount(const std::string &term, uint8_t &used_satellites)
{
    used_satellites = atoi(term.c_str());
}

void NmeaPacket::parseFixStatus(const std::string &term, char &fixStatus)
{
    fixStatus = term.empty() ? 'N' : term[0];
}

void NmeaPacket::parseSpeed(const std::string &term, double &speed)
{
    static constexpr double _GPS_MPS_PER_KNOT = 0.51444444;
    speed = _GPS_MPS_PER_KNOT * parseDecimal(term) / 100.0;
}

void NmeaPacket::parseCourse(const std::string &term, double &course)
{
    course = parseDecimal(term) / 100.0;
}

void NmeaPacket::parseHdop(const std::string &term, double &hdop)
{
    hdop = parseDecimal(term) / 100.0;
}

void NmeaPacket::parseAltitude(const std::string &term, double &altitude)
{
    altitude = parseDecimal(term) / 100.0;
}