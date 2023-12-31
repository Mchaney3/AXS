/****************************************************************************************************************************
  defines.h
  For ESP32 boards

  Blynk_Async_WM is a library, using AsyncWebServer instead of (ESP8266)WebServer for the ESP8266/ESP32 to enable easy
  configuration/reconfiguration and autoconnect/autoreconnect of WiFi/Blynk.
  
  Based on and modified from Blynk library v0.6.1 (https://github.com/blynkkk/blynk-library/releases)
  Built by Khoi Hoang (https://github.com/khoih-prog/Blynk_Async_WM)
  Licensed under MIT license
 ********************************************************************************************************************************/

/*
  // To add something similar to this for ESP32-C3
  #if CONFIG_IDF_TARGET_ESP32
  const int8_t esp32_adc2gpio[20] = {36, 37, 38, 39, 32, 33, 34, 35, -1, -1, 4, 0, 2, 15, 13, 12, 14, 27, 25, 26};
  #elif CONFIG_IDF_TARGET_ESP32S2
  const int8_t esp32_adc2gpio[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  #elif CONFIG_IDF_TARGET_ESP32C3
  const int8_t esp32_adc2gpio[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  #endif 
 */
 
#ifndef defines_h
#define defines_h

#if !( defined(ESP32) )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#elif ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_ESP32S2_THING_PLUS || ARDUINO_MICROS2 || \
        ARDUINO_METRO_ESP32S2 || ARDUINO_MAGTAG29_ESP32S2 || ARDUINO_FUNHOUSE_ESP32S2 || \
        ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM )
  #define BOARD_TYPE      "ESP32-S2"
#elif ( ARDUINO_ESP32C3_DEV )
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-gpio.c
  #warning ESP32-C3 boards not fully supported yet. Only SPIFFS and EEPROM OK. Tempo esp32_adc2gpio to be replaced. ESPAsyncWebServer library to be fixed
  const int8_t esp32_adc2gpio[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  #define BOARD_TYPE      "ESP32-C3"
#else
  #define BOARD_TYPE      "ESP32-S3"
#endif

#define BLYNK_PRINT Serial

#define DOUBLERESETDETECTOR_DEBUG     false
#define BLYNK_WM_DEBUG                0

#define BLYNK_WM_RTOS_DEBUG           1

// Not use #define USE_LITTLEFS and #define USE_SPIFFS  => using SPIFFS for configuration data in WiFiManager
// (USE_LITTLEFS == false) and (USE_SPIFFS == false)    => using EEPROM for configuration data in WiFiManager
// (USE_LITTLEFS == true) and (USE_SPIFFS == false)     => using LITTLEFS for configuration data in WiFiManager
// (USE_LITTLEFS == true) and (USE_SPIFFS == true)      => using LITTLEFS for configuration data in WiFiManager
// (USE_LITTLEFS == false) and (USE_SPIFFS == true)     => using SPIFFS for configuration data in WiFiManager
// Those above #define's must be placed before #include <BlynkSimpleEsp32_WFM.h>

#if ( ARDUINO_ESP32C3_DEV )
  // Currently, ESP32-C3 only supporting SPIFFS and EEPROM. Will fix to support LittleFS
  #define USE_LITTLEFS          false
  #define USE_SPIFFS            true
#else
  #define USE_LITTLEFS          false
  #define USE_SPIFFS            true
#endif

#if USE_LITTLEFS
  //LittleFS has higher priority
  #define CurrentFileFS     "LittleFS"
  #ifdef USE_SPIFFS
    #undef USE_SPIFFS
  #endif
  #define USE_SPIFFS                  false
#elif USE_SPIFFS
  #define CurrentFileFS     "SPIFFS"
#endif

#if !( USE_SPIFFS || USE_LITTLEFS)
  // EEPROM_SIZE must be <= 2048 and >= CONFIG_DATA_SIZE (currently 172 bytes)
  #define EEPROM_SIZE    (2 * 1024)
  // EEPROM_START + CONFIG_DATA_SIZE must be <= EEPROM_SIZE
  #define EEPROM_START   0
#endif

/////////////////////////////////////////////

// Add customs headers from v1.2.0
#define USING_CUSTOMS_STYLE                 true
#define USING_CUSTOMS_HEAD_ELEMENT          true
#define USING_CORS_FEATURE                  true

/////////////////////////////////////////////

// Force some params in Blynk, only valid for library version 1.0.1 and later
#define TIMEOUT_RECONNECT_WIFI                    10000L
#define RESET_IF_CONFIG_TIMEOUT                   true

#define CONFIG_TIMEOUT_RETRYTIMES_BEFORE_RESET    5

// Config Timeout 120s (default 60s)
#define CONFIG_TIMEOUT                            120000L

#define USE_DYNAMIC_PARAMETERS                    false

/////////////////////////////////////////////

#define REQUIRE_ONE_SET_SSID_PW             false

#define SCAN_WIFI_NETWORKS                  true

// To be able to manually input SSID, not from a scanned SSID lists
#define MANUAL_SSID_INPUT_ALLOWED           true

// From 2-15
#define MAX_SSID_IN_LIST                    8

/////////////////////////////////////////////

//////////////////////////////////////////
// Those above #define's must be placed before #include <BlynkSimpleEsp32_Async_WM.h>

//You have to download Blynk WiFiManager Blynk_Async_WM library at //https://github.com/khoih-prog/Blynk_Async_WM
// In order to enable (USE_BLYNK_WM = true). Otherwise, use (USE_BLYNK_WM = false)
//#define USE_BLYNK_WM   true
#define USE_BLYNK_WM   false

//#define BLYNK_SSL_USE_LETSENCRYPT true
//#define USE_SSL     true
#define USE_SSL     false

#if USE_BLYNK_WM
  #if USE_SSL
    #include <BlynkSimpleEsp32_SSL_Async_WM.h>        //https://github.com/khoih-prog/Blynk_Async_WM
  #else
    #include <BlynkSimpleEsp32_Async_WM.h>            //https://github.com/khoih-prog/Blynk_Async_WM
  #endif
  
  #include "Credentials.h"
  #include "dynamicParams.h"

#else
  #if USE_SSL
    // Uncomment to use Let's Encrypt Root CA
    
    #include <BlynkSimpleEsp32_SSL.h>
    #define BLYNK_HARDWARE_PORT     443
  #else
    #include <BlynkSimpleEsp32.h>
    #define BLYNK_HARDWARE_PORT     8181
  #endif
#endif

#if !USE_BLYNK_WM

  #ifndef LED_BUILTIN
    #define LED_BUILTIN       48         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
  #endif
  
  #define USE_LOCAL_SERVER    true
  //#define USE_LOCAL_SERVER    false
  
  // If local server
  #if USE_LOCAL_SERVER
    char blynk_server[]   = "mowbot.axstechnology.com";
  #endif
  
  char auth[]     = "f79d2fd79bf94187a1a8b1ec6bd9346a";
  char ssid[]     = "chlabs_bot";
  char pass[]     = "chlabsrobotseverywhere";

#endif

#define PIN_D22   22            // Pin D22 mapped to pin GPIO22/SCL of ESP32

#define DHT_PIN     PIN_D22     // pin DATA @ D22 / GPIO22
#define DHT_TYPE    DHT11

#define HOST_NAME   "Arnold-S3"

#endif      //defines_h
