/*
  SparkFun Apple Accessory Arduino Library - Example

  ESP32_BluetoothSerial

  This example demonstrates how to create an Apple Accessory,
  based on ESP32 hardware, which will communicate with Apple
  devices using Bluetooth transport.

  Details of the Apple Accessory Protocol are only available under NDA.
  The SparkFun Apple Accessory Arduino Library is pre-compiled for ESP32
  to protect the Apple IP contained in the source code.

  This example shares the NMEA Location Information from a u-blox GNSS
  with the iPhone, allowing map apps to use the location. It requires a
  u-blox GNSS which supports the configuration interface: F9, M10, X20.

  Requires the SparkFun_u-blox_GNSS_v3 library >= v3.1.10

  NMEA GGA and RMC are shared with apps like Maps, over the Control Session
  as Location Information.
  NMEA GGA, RMC and GST are shared with apps like Field Maps, over a dedicated
  External Accessory protocol session. Here we emulate the Bad Elf GPS Pro,
  using the com.bad-elf.gps protocol name. We are grateful to Bad Elf for
  sharing details of their protocol openly: https://github.com/BadElf/gps-sdk

  This example contains a copy of Espressif's BluetoothSerial code, with a
  modified ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT event.

  This example also needs a custom-compiled version of the Espressif libbt.a
  Bluetooth library. The custom version can be found in the "patch" folder.
  It needs to be copied into:
    C:\Users\<YOUR USER>\AppData\Local\Arduino15\packages\esp32\tools
    \esp32-arduino-libs\idf-release_v5.4-2f7dcd86-v1\esp32\lib
  We created it using the Espressif ESP32 Arduino Lib Builder:
    https://github.com/espressif/esp32-arduino-lib-builder
  It includes:
    CONFIG_BT_ENABLED=y
    CONFIG_BT_CLASSIC_ENABLED=y
    CONFIG_BT_A2DP_ENABLE=y
    CONFIG_BT_SPP_ENABLED=y
    CONFIG_BT_HFP_ENABLE=y
    CONFIG_BT_STACK_NO_LOG=y
    CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY=y
    CONFIG_BT_BLUEDROID_ENABLED=y
    CONFIG_BT_L2CAP_ENABLED=y
    CONFIG_BT_SDP_COMMON_ENABLED=y
    CONFIG_BTDM_CTRL_MODE_BTDM=y
    CONFIG_BTDM_SCAN_DUPL_CACHE_SIZE=20
    CONFIG_BTDM_CTRL_MODE_BLE_ONLY=n
    CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=n
  (SPP is disabled in the standard libbt.a)

  Tried and tested on: Arduino esp32 v3.1.3 (IDF 5.3) and v3.2.0 (IDF 5.4)

*/

// ===================================================================================================================
// GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;
char latestGPGGA[SFE_UBLOX_MAX_NMEA_BYTE_COUNT] = { 0 };
char latestGPRMC[SFE_UBLOX_MAX_NMEA_BYTE_COUNT] = { 0 };
char latestGPGST[SFE_UBLOX_MAX_NMEA_BYTE_COUNT] = { 0 };

void newGPGGA(NMEA_GGA_data_t *nmeaData)
{
  if (nmeaData->length < SFE_UBLOX_MAX_NMEA_BYTE_COUNT)
  {
    strncpy(latestGPGGA, (const char *)nmeaData->nmea, nmeaData->length);
    //Serial.print(latestGPGGA); // .nmea is printable (NULL-terminated) and already has \r\n on the end
    latestGPGGA[nmeaData->length - 2] = 0; // Truncate after checksum. Remove CR LF
  }
}

void newGPRMC(NMEA_RMC_data_t *nmeaData)
{
  if (nmeaData->length < SFE_UBLOX_MAX_NMEA_BYTE_COUNT)
  {
    strncpy(latestGPRMC, (const char *)nmeaData->nmea, nmeaData->length);
    //Serial.print(latestGPRMC); // .nmea is printable (NULL-terminated) and already has \r\n on the end
    latestGPRMC[nmeaData->length - 2] = 0; // Truncate after checksum. Remove CR LF
  }
}

void newGPGST(NMEA_GST_data_t *nmeaData)
{
  if (nmeaData->length < SFE_UBLOX_MAX_NMEA_BYTE_COUNT)
  {
    strncpy(latestGPGST, (const char *)nmeaData->nmea, nmeaData->length);
    //Serial.print(latestGPGST); // .nmea is printable (NULL-terminated) and already has \r\n on the end
    latestGPGST[nmeaData->length - 2] = 0; // Truncate after checksum. Remove CR LF
  }
}

// ===================================================================================================================
// Apple Accessory

#include <SparkFun_Apple_Accessory.h>

SparkFunAppleAccessoryDriver appleAccessory;

const char *accessoryName = "SparkFun MFi Test";
const char *modelIdentifier = "SparkFun MFi Test";
const char *manufacturer = "SparkFun Electronics";
const char *serialNumber = "123456";
const char *firmwareVersion = "1.0.0";
const char *hardwareVersion = "1.0.0";
const char *EAProtocol = "com.bad-elf.gps"; // Emulate the Bad Elf GPS Pro. Thank you Bad Elf
const char *BTTransportName = "com.sparkfun.bt";
const char *LIComponentName = "com.sparkfun.rtk";
const char *productPlanUID = "0123456789ABCDEF"; // This comes from the MFi Portal, when you register the product with Apple

// ===================================================================================================================
// Bluetooth

#include "src/BluetoothSerial/BluetoothSerial.h" //Local copy for modifying ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT event
//#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#include "esp_sdp_api.h"

const char *sdp_service_name = "iAP2";

static const uint8_t  UUID_IAP2[] = {0x00, 0x00, 0x00, 0x00, 0xDE, 0xCA, 0xFA, 0xDE, 0xDE, 0xCA, 0xDE, 0xAF, 0xDE, 0xCA, 0xCA, 0xFF};

uint8_t btMacAddress[6];

void transportConnected(bool *isConnected)
{
  *isConnected = SerialBT.connected();
}

void transportDisconnect(bool *disconnected)
{
  *disconnected = SerialBT.disconnect();
}

// ===================================================================================================================

void setup()
{
  Serial.begin(115200);
  delay(250);

  Wire.begin(); // Needed for authentication coprocessor

  // ==============================================================================================================
  // Setup Bluetooth

  SerialBT.enableSSP(false, false); //Enable secure pairing, authenticate without displaying anything

  SerialBT.begin(accessoryName, true, true);  //Bluetooth device name, start in master mode, disable BLE

  Serial.println("The BT device was started.");

  SerialBT.getBtAddress(btMacAddress); // Read the ESP32 BT MAC Address
  Serial.print("BT MAC: ");
  for (uint8_t i = 0; i < 6; i++)
    Serial.printf("%02X ",btMacAddress[i]);
  Serial.println();
  
  esp_sdp_init();

  esp_bluetooth_sdp_raw_record_t record = {(esp_bluetooth_sdp_types_t)0};
  record.hdr.type = ESP_SDP_TYPE_RAW;
  record.hdr.uuid.len = sizeof(UUID_IAP2);
  memcpy(record.hdr.uuid.uuid.uuid128, UUID_IAP2, sizeof(UUID_IAP2));
  record.hdr.service_name_length = strlen(sdp_service_name) + 1;
  record.hdr.service_name = (char *)sdp_service_name;
  esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&record);

  // ==============================================================================================================
  // Setup Apple Accessory and Authentication Coprocessor

  appleAccessory.usePSRAM(false); // Tell the driver whether to use PSRAM - before begin

  // Check the authentication coprocessor is connected - and awake
  if(!appleAccessory.begin(Wire))
  {
    Serial.println("Could not initialize the authentication coprocessor. Freezing...");
    while(1);
  }

  //appleAccessory.enableDebug(&Serial); // Uncomment to enable debug prints to Serial

  // Pass Identity Information, Protocols and Names into the accessory driver
  appleAccessory.setAccessoryName(accessoryName);
  appleAccessory.setModelIdentifier(modelIdentifier);
  appleAccessory.setManufacturer(manufacturer);
  appleAccessory.setSerialNumber(serialNumber);
  appleAccessory.setFirmwareVersion(firmwareVersion);
  appleAccessory.setHardwareVersion(hardwareVersion);
  appleAccessory.setExternalAccessoryProtocol(EAProtocol);
  appleAccessory.setBluetoothTransportName(BTTransportName);
  appleAccessory.setBluetoothMacAddress(btMacAddress);
  appleAccessory.setLocationInfoComponentName(LIComponentName);
  appleAccessory.setProductPlanUID(productPlanUID);

  // Pass the pointers for the latest NMEA data into the Accessory driver
  appleAccessory.setNMEApointers(latestGPGGA, latestGPRMC, latestGPGST);

  // Pass the transport connected and disconnect methods into the accessory driver
  appleAccessory.setTransportConnectedMethod(&transportConnected);
  appleAccessory.setTransportDisconnectMethod(&transportDisconnect);

  // ==============================================================================================================
  // Begin the GNSS. Set up callbacks for the NMEA data

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  while (!myGNSS.begin())
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  }

  // Disable or enable various NMEA sentences over the I2C interface
  myGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX); // Turn on both UBX and NMEA sentences on I2C. (Turn off RTCM and SPARTN)
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual NMEA messages
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0); // Disable all NMEA messages except GGA and RMC. Enable GST
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GST_I2C, 1);
  myGNSS.addCfgValset(UBLOX_CFG_NMEA_PROTVER, 23); // Set the NMEA version to 2.3
  myGNSS.addCfgValset(UBLOX_CFG_NMEA_OUT_FROZENCOG, 1); // Always output the RMC Course Over Ground
  if (myGNSS.sendCfgValset()) // Send the configuration VALSET
    Serial.println(F("NMEA messages were configured successfully"));
  else
    Serial.println(F("NMEA message configuration failed!"));

  // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);

  myGNSS.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA messages

  // Set up the callback for GPGGA
  myGNSS.setNMEAGPGGAcallbackPtr(&newGPGGA);

  // Set up the callback for GPRMC
  myGNSS.setNMEAGPRMCcallbackPtr(&newGPRMC);

  // Set up the callback for GPGST
  myGNSS.setNMEAGPGSTcallbackPtr(&newGPGST);

}

// ===================================================================================================================

void loop()
{
  // ==============================================================================================================
  // Update the Accessory driver

  appleAccessory.update();

  // ==============================================================================================================
  // Update the GNSS

  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  // ==============================================================================================================
  // Check for a new device connection

  if (SerialBT.aclConnected() == true)
  {
    Serial.println("Apple Device found, connecting...");

    SerialBT.connect(SerialBT.aclGetAddress(), 1); //Connect on channel 1

    if (SerialBT.connected())
    {
      appleAccessory.startHandshake(&SerialBT);
    }
  }

  // ==============================================================================================================
  // If the user hits 'r' then restart
  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'r')
      ESP.restart();
  }
}

