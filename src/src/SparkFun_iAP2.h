#ifndef SparkFun_iAP2_H
#define SparkFun_iAP2_H

#include <Arduino.h>
#include <SparkFun_Auth_3_CP.h>

class SparkFuniAPDriverDebugPrint
{
    public:
        SparkFuniAPDriverDebugPrint();
        ~SparkFuniAPDriverDebugPrint();

        void enableDebug(Print *debugPrint);
        void disableDebug(void);

        void systemWrite(const char *string);
        void systemPrintf(const char *format, ...);
        void systemPrint(const char *string);
        void systemPrintln(const char *string);
        void systemPrintln();

        Print *_debugPrint = nullptr;
};

class SparkFuniAP2Driver
{
    public:
        SparkFuniAP2Driver();
        ~SparkFuniAP2Driver();
        void begin();
        void enableDebug(Print *debugPrint);
        void disableDebug(void);
        void iap2ParseUpdate();
        void iap2Handler();
        void iap2LinkUpdate();
        void iap2TrafficUpdate();
        void startHandshake(Stream *theStream);
        void setNMEApointers(char *latestGPGGA, char *latestGPRMC, char *latestGPGST = nullptr, char *latestGPVTG = nullptr);
        void setEASessionPointer(char *latestEASessionData);
        bool latestEASessionDataIsBlocking();
        void setAuthPointers(uint8_t *authCert, size_t certSize, char *authCertSerial);
        void setAuthCoprocessorPointer(SparkFunAuth3CPArdI2C *authCoprocessor);
        void setAccessoryName(const char *accessoryName);
        void setModelIdentifier(const char *modelIdentifier);
        void setManufacturer(const char *manufacturer);
        void setSerialNumber(const char *serialNumber);
        void setFirmwareVersion(const char *firmwareVersion);
        void setHardwareVersion(const char *hardwareVersion);
        void setExternalAccessoryProtocol(const char *EAProtocol);
        void setBluetoothTransportName(const char *BTTransportName);
        void setLocationInfoComponentName(const char *LIComponentName);
        void setProductPlanUID(const char *productPlanUID);
        void setBluetoothMacAddress(const uint8_t *macAddress);
        void setTransportConnectedMethod(void (*methodPtr)(bool *));
        void setTransportDisconnectMethod(void (*methodPtr)(bool *));
        void usePSRAM(bool enable); // Must be called before begin

    private:
        void iap2LinkSetState(uint8_t newState);
        void iap2ParseSetState(uint8_t newState);
        uint8_t iap2ParseGetState();
        void iap2TrafficSetState(uint8_t newState);
        void iap2CreateSynPacket();
        void iap2SendAckPacket(uint8_t seqNumber, uint8_t ackNumber);
        void sendSessionMessageBlobSingleParameter(uint16_t messageId, uint8_t iap2SessionId, uint8_t* parameterData, uint16_t parameterDataLength);
        void sendSessionMessageBlobMultiParameter(uint16_t messageId, uint8_t iap2SessionId, uint8_t* parameterData, uint16_t parameterDataLength);
        void sendExternalAccessorySessionData(uint16_t iap2SessionId, uint8_t* sessionData, uint16_t sessionDataLength);
        void iap2CreatePacket(uint8_t *packet, uint8_t controlByte, uint8_t seqNumber, uint8_t ackNumber, uint8_t sessionID, uint8_t *data, uint16_t dataLength);
        void iap2SendPacket(uint8_t *packet);
        uint8_t iap2Checksum(uint8_t *data, uint16_t dataLength);
        void iap2PrintPacket(uint8_t *packetData);
        void iap2PrintRawPacket(uint8_t *packetData);
        uint16_t iap2GetPacketLength(uint8_t *packet);
        uint16_t iap2GetDataLength(uint8_t *packet);
        int16_t parseSessionMessage(uint8_t *messageData, uint16_t *messageId);
        bool parseSessionMessageParameter(uint8_t *messageData, uint16_t desiredParameter, uint8_t **parameterStart, uint16_t *parameterLength, uint16_t *parameterId);
        bool addIdentificationInformation(const char *txt, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationUint8(uint8_t val, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationUint16(uint16_t val, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationNone(uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationExternalAccessoryProtocol(uint8_t protocolIdentifier, const char *protocolName, uint8_t matchAction, uint16_t *transportIdentifier, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationBluetoothTransport(uint16_t transportIdentifier, const char *transportName, bool supportsiAP2, const uint8_t *macAddr, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationLocationInformation(uint16_t identifier, const char *Name, bool GPGGA, bool GPRMC, bool PASCD, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addIdentificationInformationMessagesSentReceived(uint16_t numMessagesReceived, uint16_t *messagesReceived, uint8_t **information, uint16_t *informationLength, uint16_t id);
        bool addLocationInformation(const char *txt, uint8_t **information, uint16_t *informationLength, uint16_t id);
        void printHexThing(const char *name, size_t thingSize, uint8_t *thing);

        #define IAP2_MAX_PACKET_SIZE 2048 //Default for Bluetooth

        unsigned long handshakeTimer = millis();
        bool handshakeReceived = false;

        //iAP2 Packet Variables
        uint8_t iap2Packet[IAP2_MAX_PACKET_SIZE];

        uint16_t iap2ResponseSpot = 0;
        uint16_t iap2ResponseLength = 0;
        bool iap2ResponseReceived = false; // Set true when a valid iAP2 packet is received

        uint8_t iap2PacketSequenceNumber = 0; //Increment our PSN once the device ACK's it
        uint8_t iap2DevicePacketSequenceNumber = 0; //We must keep track of the device's packet numbers to send ACK's

        //Link Synchronization Payload variables
        uint8_t iap2ControlSessionId = 0x0A;
        uint8_t iap2ExternalAccessorySessionId = 0x0B;

        bool iap2PacketDelivered = false; //Goes true once a packet we send is ACK'd by the device
        uint16_t iap2PacketSendTime; //Monitors ACK timeout

        //Session Message Variables
        uint8_t *iap2SessionMessage = nullptr;

        //Location Information
        bool locationInformationStarted = false;
        bool gpggaRequested = false; // Applies to Location Information only - not EA
        bool gprmcRequested = false; // Applies to Location Information only - not EA

        // External Accessory
        bool eaSessionStarted = false;
        uint32_t lastSend = 0;
        uint16_t externalAccessoryProtocolSessionId = 0; //ID given to us to start an EA session

        SparkFuniAPDriverDebugPrint _debugPrint;

        Stream *_theStream; // Pointer to the transport stream

        char *_latestGPGGA = nullptr; // Pointer to the latest NMEA GGA, provided by an external GNSS
        char *_latestGPRMC = nullptr; // Pointer to the latest NMEA RMC, provided by an external GNSS
        char *_latestGPGST = nullptr; // Pointer to the latest NMEA GST, provided by an external GNSS
        char *_latestGPVTG = nullptr; // Pointer to the latest NMEA VTG, provided by an external GNSS
        char *_latestEASessionData = nullptr; // Pointer to the latest ES Session NMEA GSA / GSV blob, provided by an external GNSS
        bool _latestEASessionDataIsBlocking = false; // Flag to indicate when EA Session Data is being written to the device

        uint8_t *_authCert = nullptr; // Storage for the authentication certificate
        size_t _certSize = 0; // The size of the authentication certificate
        char *_authCertSerial = nullptr; // Storage for the certificate serial number

        SparkFunAuth3CPArdI2C *_authCoprocessor = nullptr; // Pointer to the instance of the authentication coprocessor

        const char *_accessoryName = nullptr;
        const char *_modelIdentifier = nullptr;
        const char *_manufacturer = nullptr;
        const char *_serialNumber = nullptr;
        const char *_firmwareVersion = nullptr;
        const char *_hardwareVersion = nullptr;
        const char *_EAProtocol = nullptr;
        const char *_BTTransportName = nullptr;
        const char *_LIComponentName = nullptr;
        const char *_productPlanUID = nullptr;
        const uint8_t *_bluetoothMacAddress = nullptr;
        void (*_transportIsConnected)(bool *);
        bool _transportIsConnectedDefined = false;
        void (*_transportDisconnect)(bool *);
        bool _transportDisconnectDefined = false;
        bool _usePSRAM = false;
};

#endif // #ifndef SparkFun_iAP2_H