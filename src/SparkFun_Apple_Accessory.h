#ifndef SparkFun_Apple_Accessory_H
#define SparkFun_Apple_Accessory_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Auth_3_CP.h> // Click here to get the library: http://librarymanager/All#SparkFun_Authentication_Coprocessor
#include "src\SparkFun_iAP2.h"

class SparkFunAppleAccessoryDriver
{
    public:
        SparkFunAppleAccessoryDriver();
        ~SparkFunAppleAccessoryDriver();

        /**
         * @brief Enable / disable the use of PSRAM for the authentication certificate etc..
         * 
         * @param enable True: PSRAM is enabled; False: PSRAM is disabled
         *
         * @note Call this method before begin.
         */
        void usePSRAM(bool enable);

        /**
         * @brief Initializes the driver and begins communication with the Auth 3.0 Coprocessor.
         *
         * @param wirePort The I2C port to communicate with the coprocessor.
         *
         * @return Returns `true` if the initialization was successful, `false` otherwise.
         *
         * @note Call this method before using any other functions in the class - except usePSRAM.
         */
        bool begin(TwoWire &wirePort);

        /**
         * @brief Set the pointers to the latest NMEA GGA and RMC - from external GNSS.
         *        GST is optional.
         * 
         * @param latestGPGGA Pointer to the latest GNSS NMEA GPGGA message
         *
         * @param latestGPRMC Pointer to the latest GNSS NMEA GPRMC message
         *
         * @param latestGPGST Pointer to the latest GNSS NMEA GPGST message (optional)
         *
         * @note The pointers will be set the nullptr once the NMEA data has been consumed
         */
        void setNMEApointers(char *latestGPGGA, char *latestGPRMC, char *latestGPGST = nullptr);

        /**
         * @brief Enable debug prints on the selected Print stream
         *
         * @param debugPrint The Print stream for debug messages.
         */
        void enableDebug(Print *debugPrint);

        /**
         * @brief Disable debug prints.
         */
        void disableDebug(void);

        /**
         * @brief Update the internal state machine.
         */
        void update(void);

        /**
         * @brief Start the accessory protocol handshake.
         * 
         * @param theStream Pointer to the transport Stream
         */
        void startHandshake(Stream *theStream);

        /**
         * @brief Set the accessory name.
         * 
         * @param accessoryName Pointer to the accessory name
         */
        void setAccessoryName(const char *accessoryName);

        /**
         * @brief Set the model identifier.
         * 
         * @param modelIdentifier Pointer to the model identifier
         */
        void setModelIdentifier(const char *modelIdentifier);

        /**
         * @brief Set the manufacturer.
         * 
         * @param  Pointer to the manufacturer
         */
        void setManufacturer(const char *manufacturer);

        /**
         * @brief Set the serial number.
         * 
         * @param  Pointer to the serial number
         */
        void setSerialNumber(const char *serialNumber);

        /**
         * @brief Set the firmware version.
         * 
         * @param  Pointer to the firmware version
         */
        void setFirmwareVersion(const char *firmwareVersion);

        /**
         * @brief Set the hardware version.
         * 
         * @param hardwareVersion Pointer to the hardware version
         */
        void setHardwareVersion(const char *hardwareVersion);

        /**
         * @brief Set the external accessory protocol name.
         * 
         * @param EAProtocol Pointer to the external accessory protocol name
         */
        void setExternalAccessoryProtocol(const char *EAProtocol);

        /**
         * @brief Set the bluetooth transport name.
         * 
         * @param BTTransportName Pointer to the bluetooth transport name
         */
        void setBluetoothTransportName(const char *BTTransportName);

        /**
         * @brief Set the bluetooth mac address.
         * 
         * @param macAddress Pointer to the MAC address
         */
        void setBluetoothMacAddress(uint8_t *macAddress);

        /**
         * @brief Set the location information component name.
         * 
         * @param LIComponentName Pointer to the location information component name
         */
        void setLocationInfoComponentName(const char *LIComponentName);

        /**
         * @brief Set the product plan UID.
         * 
         * @param productPlanUID Pointer to the product plan UID
         */
        void setProductPlanUID(const char *productPlanUID);

        /**
         * @brief Set the transport connected callback.
         * 
         * @param theMethod Pointer to the connected method
         */
        void setTransportConnectedMethod(void (*methodPtr)(bool *));

        /**
         * @brief Set the transport disconnect callback.
         * 
         * @param theMethod Pointer to the disconnect method
         */
        void setTransportDisconnectMethod(void (*methodPtr)(bool *));

    private:
        /**
         * @brief Allocates memory for the authentication certificate.
         *
         * @param certSize The required size of the allocation.
         *
         * @return Returns `true` if the allocation was successful, `false` otherwise.
         *
         * @note This is platform dependent. On ESP32, this is allocated in PSRAM.
         *       If this library is ported to other platforms, this should be moved
         *       into a separate class.
         */
        bool allocAuthCert(size_t certSize);

        /**
         * @brief Deallocates the memory for the authentication certificate.
         *
         * @note This is platform dependent. On ESP32, this is allocated in PSRAM.
         *       If this library is ported to other platforms, this should be moved
         *       into a separate class.
         */
        void deallocAuthCert(void);

        SparkFunAuth3CPArdI2C _authCoprocessor; // Instance of the authentication coprocessor
        SparkFuniAP2Driver _iAP2; // Instance of the iAP2 driver (details are under NDA)
        uint8_t *_authCert; // Storage for the authentication certificate
        size_t _certSize; // The size of the authentication certificate
        char _authCertSerial[32 + 1]; // Storage for the certificate serial number
        bool _usePSRAM; // Flag to indicate if PSRAM should be used
        
};

#endif // #ifndef SparkFun_Apple_Accessory_H