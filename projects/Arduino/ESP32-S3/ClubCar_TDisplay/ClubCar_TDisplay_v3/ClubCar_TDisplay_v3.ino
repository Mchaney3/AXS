//  Wallpaper
//#include "Arduino.h"
//#include  "stdint.h"

//  Time
#if (ESP8266 || ESP32)
  #define USE_LITTLEFS      true
  #define USE_SPIFFS        false
#endif

//////////////////////////////////////////

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <Timezone_Generic.h>           // https://github.com/khoih-prog/Timezone_Generic
#include <TimeLib.h>            // https://github.com/PaulStoffregen/Time
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule etDST = {"EDT", Second, Sun, Mar, 2, -420};  // Daylight time = UTC - 4 hours
TimeChangeRule etSTD = {"EST", First, Sun, Nov, 2, -480};   // Standard time = UTC - 5 hours
//Timezone et(etDST, etSTD);
Timezone *et;

//  GPS
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define tzOffset 4
static const int RXPin = 18, TXPin = 17, sclPin = 1, sdaPin = 2;
static const uint32_t GPSBaud = 9600;
uint32_t targetTime = 0;                    // for next 1 second timeout
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
const PROGMEM  uint8_t SetGPSBaudRate[] = {0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22}; //baudrate 115200
const PROGMEM  uint8_t UBLOX_INIT_38400[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00, 0x00,0x96,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x83,0x90,};
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

/*
NMEA Standard Messages Standard Messages
54 DTM 0xF0 0x0A Datum Reference
55 GBS 0xF0 0x09 GNSS Satellite Fault Detection
56 GGA 0xF0 0x00 Global positioning system fix data
57 GLL 0xF0 0x01 Latitude and longitude, with time of position fix and status
58 GPQ 0xF0 0x40 Poll message
59 GRS 0xF0 0x06 GNSS Range Residuals
60 GSA 0xF0 0x02 GNSS DOP and Active Satellites
61 GST 0xF0 0x07 GNSS Pseudo Range Error Statistics
62 GSV 0xF0 0x03 GNSS Satellites in View
63 RMC 0xF0 0x04 Recommended Minimum data
64 THS 0xF0 0x0E True Heading and Status
65 TXT 0xF0 0x41 Text Transmission
66 VTG 0xF0 0x05 Course over ground and Ground speed
*/
void updateGPSSettings(const uint8_t *Progmem_ptr, uint8_t arraysize) {

  uint8_t byteread, index;

  Serial.print(F("GPSSend  "));

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    if (byteread < 0x10)
    {
      Serial.print(F("0"));
    }
    Serial.print(byteread, HEX);
    Serial.print(F(" "));
  }

  Serial.println();
  Progmem_ptr = Progmem_ptr - arraysize;                  //set Progmem_ptr back to start

  for (index = 0; index < arraysize; index++)
  {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    ss.write(byteread);
  }
  delay(100);
}

//  WIFI and OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "credentials.h"

//  TFT
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
//#include "fonts.h" // Include the header file attached to this sketch
//#include "Fonts/Custom/Orbitron_Light_24.h"
#include "wallpaper.h"
//#include "cat.h"
#define PIN_POWER_ON  15
#define TFT_GREY      0x5AEB
#define CLOCK_COLOR   0x1F
#define WIDTH         320
#define HEIGHT        170
uint16_t TACH_COLOR = 0xFF;
#define AA_FONT_SMALL "Roboto_Thin_24"
#define AA_FONT_LARGE "Orbitron_Light_32"
/*
#include "fonts/GFXFF/Open_24_Display_St_44pt.h"
#define GFXFF 1
#define TestFont &Orbitron_Light_24
*/

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite tachometer = TFT_eSprite(&tft); // Sprite object graph1
TFT_eSprite speedometer = TFT_eSprite(&tft); // Sprite object graph1
TFT_eSprite sprtClock = TFT_eSprite(&tft); // Sprite object graph1
TFT_eSprite background = TFT_eSprite(&tft); // Sprite object graph1
TFT_eSprite satInfo = TFT_eSprite(&tft); // Sprite object graph1
TFT_eSprite cmpsNeedle = TFT_eSprite(&tft); // Sprite object graph1



//  Functions

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drawOtaProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}

static void RefreshScreen(void) {
  
  //  Read GPS info from BN-880
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  //valMPH = 4;  //  TESTING
  float valMPH = gps.speed.mph();
  int spd = valMPH;
  float engRPM = mapfloat(valMPH, 0.00, 25.00, 0.00, 100.00); //  Map speed to engine RPM as a percentage

  if (valMPH <= 1.00) {
    engRPM = 0;
  }

  //  Push wallpaper to background sprite
  background.deleteSprite();
  background.createSprite(320,170);
  background.setSwapBytes(true);
  background.pushImage(0,0,320,170,wallpaper);

  //  Set color of tach and speed
  if (engRPM >= 75) {
    TACH_COLOR = 0xD8;  //  RED
  }
  else if (engRPM >= 50) {
    TACH_COLOR = 0xFD;  //  ORANGE - currently this gives pink at 8 bit
  }
  else {
    TACH_COLOR = 0xBF;  //  YELLOW - currently this gives purple at 8 bit. 
  }

  //  Build Speedometer
  speedometer.deleteSprite();

  speedometer.loadFont(AA_FONT_SMALL);
  //speedometer.setFreeFont(Orbitron_Light_24);
  speedometer.setSwapBytes(true);
  speedometer.createSprite(tft.width() / 2, 50);    //    width = half size of screen, height = 60  
  speedometer.setTextDatum(4);  // Set text coordinate datum to middle centre
  speedometer.setTextSize(5);  
  speedometer.setTextColor(TACH_COLOR, TFT_BLACK);
  speedometer.drawString(String(spd), 40, (speedometer.height() / 2)); // Draw the number in middle of 80 x 50 sprite
  speedometer.setTextSize(3);
  speedometer.drawString("mph", 110, 30);
  speedometer.unloadFont();
  

  //  Build Tachometer
  uint8_t margin = 4;
  uint16_t barHeight = tachometer.height() - 2 * margin;
  uint16_t barWidth = tachometer.width();
  tachometer.deleteSprite();
  tachometer.createSprite(tft.width() / 2, 50);    //    width = half size of screen, height = 60
  tachometer.setSwapBytes(true);
  tachometer.fillRect(0, 10, barWidth * engRPM / 100.0, barHeight, TACH_COLOR);   

  //  Build Clock
  et = new Timezone(etDST, etSTD);
  setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());

  char gpsTime[32];                                                            // Update digital time
  sprintf(gpsTime, "%02d:%02d:%02d", etDST.hour, gps.time.minute(), gps.time.second());
  sprtClock.deleteSprite();
  //sprtClock.loadFont(&Orbitron_Light_24);
  sprtClock.createSprite(100, 20);    //    width = half size of screen, height = 60
  sprtClock.setSwapBytes(true);
  sprtClock.setTextSize(2);
  sprtClock.setTextColor(CLOCK_COLOR, TFT_BLACK);  //  set clock text color - 0xBDF7 = Light Gray
  sprtClock.drawString(gpsTime, 0, 0);      //  Draw Clock -   Last 5 is text size maybe??  

  //  Update Satellite Info
  int satellites = 0;
  double mph = 0;
  double lat = 0;
  double lng = 0;
  double altitude = 0;
  double hdgDegree = 0;
  String hdgCardinal = "N";
  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }
  if (gps.speed.isValid()) {
    mph = gps.speed.mph();
  }
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    altitude = gps.altitude.feet();
  }
  if (gps.course.isValid()) {
    hdgDegree = gps.course.deg();
    //Serial.println(hdgDegree);
    hdgCardinal = TinyGPSPlus::cardinal(gps.course.deg());
    //Serial.println(hdgCardinal);
  }
  char data[150] = {};
  sprintf(data, "-Satellites: %d\n-Latitude: %.6lf\n-Longitude: %.6lf\n-Altitude: %.2lf ft\n-Speed: %.1lf mph\n-Heading: %.2lf\n-Direction: ", satellites, lat, lng, altitude, mph, hdgDegree, hdgCardinal);  
  satInfo.deleteSprite();
  satInfo.createSprite(160, 85);    //    width = half size of screen, height = 60
  satInfo.setSwapBytes(true);
  satInfo.fillRect(0, 160, 160, 20, TFT_BLACK);
  satInfo.setCursor(20, 0);
  satInfo.setTextColor(CLOCK_COLOR);
  satInfo.setTextSize(2);
  satInfo.println("GPS Details");
  satInfo.setCursor(0, 20);
  satInfo.setTextSize(1);
  satInfo.print(data);
  satInfo.println(hdgCardinal);

  //build compass needle and rotate
  cmpsNeedle.deleteSprite();
  cmpsNeedle.createSprite(160, 170);    //    width = half size of screen, height = 60
  cmpsNeedle.setSwapBytes(true);
  cmpsNeedle.fillRect(0, 160, 160, cmpsNeedle.height() / 2, TFT_BLACK);
  cmpsNeedle.setTextColor(CLOCK_COLOR);
  cmpsNeedle.setTextSize(5);
  cmpsNeedle.setCursor(cmpsNeedle.width() / 2, cmpsNeedle.height() / 2);
  cmpsNeedle.println("^");
  
  //  Push all sprites to screen
  //test
  cmpsNeedle.pushRotated(&background, hdgDegree, TFT_BLACK);   //  This works but it centers needle on the screen isntead of top right

  satInfo.pushToSprite(&background,160,0,TFT_BLACK);
  tachometer.pushToSprite(&background,5,50,TFT_BLACK); 
  speedometer.pushToSprite(&background,0,120,TFT_BLACK);
  sprtClock.pushToSprite(&background,10,10,TFT_BLACK);
  background.pushSprite(0, 0);

}

void initGPS(void){

  //  SET CLOCK FROM GPS
  tft.print("Waiting for GPS");
  Serial.print("Waiting for GPS");
  for (int i = 0; i <= 300; i++) {
    //  Read GPS info from BN-880
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }
    int satellites = gps.satellites.value();
    if (gps.time.isValid()) {   //  Why isn't isValid() working here?
      tft.println("*");
      Serial.println("*");
      tft.println("GPS acquired. Setting time from outer space...");
      Serial.println("GPS acquired. Setting time from outer space...");
      delay(3000);
      break;
    }
    else {        
      tft.print(satellites);
      //tft.print(".");
      Serial.print(".");
      delay(1000);
    }   
  }
}

void setup(void) {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.println("Mike's Club Car Is Booting");
  tft.println("");
  tft.setTextSize(1);
  tft.print("ESP32 SDK:  ");
  tft.println(ESP.getSdkVersion());
  tft.print(__FILE__);
  tft.print(" built by AXS Automation LLC on ");
  tft.println(__DATE__);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Mike's Club Car Is Booting");
  Serial.print("ESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print(__FILE__);
  Serial.print(" built by AXS Automation LLC on ");
  Serial.println(__DATE__);

  ss.begin(GPSBaud);
  delay(1000);
  //updateGPSSettings(GPGLLOff, 16);      //  Latitude and longitude, with time of position fix and status
  //updateGPSSettings(GPGSVOff, 16);    //  Satellites in view
  //updateGPSSettings(GPVTGOff, 16);    //  Course over ground and Ground speed
  //updateGPSSettings(GPGSAOff, 16);    //  GNSS DOP and Active Satellites
  //updateGPSSettings(GPGGAOff, 16);    //  Global positioning system fix data
  updateGPSSettings(Navrate10hz, 14);   //  name of PROGMEM array and length. Specifically, this sets the GPS to 10 updates a second
  //initGPS();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(3000);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("No WiFi...");
      tft.setCursor(0, 10);
      tft.println("No WiFi. OTA NOT AVAILABLE...");
      delay(3000);
  }
  else{
    tft.println("WiFi Connected. Initializing OTA.");
    tft.print("IP address: ");
    tft.println(WiFi.localIP().toString());
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPasswordHash(otaHash);
    ArduinoOTA
    .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    })
    .onEnd([]() {
        Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        int percentage = (progress / (total / 100));
        tft.setTextDatum(TC_DATUM);
        tft.setTextPadding(tft.textWidth(" 888% "));
        tft.drawString(String(percentage) + "%", 145, 35);
        drawOtaProgressBar(10, 30, 120, 15, percentage, TFT_WHITE, TFT_BLUE);
    })
    .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");

        tft.fillScreen(TFT_BLACK);
        tft.drawString("Update Failed", tft.width() / 2 - 20, 55 );
        delay(3000);
        tft.fillScreen(TFT_BLACK);
    });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    tft.println("BOOT COMPLETE!");    
    delay(5000);
    tft.fillScreen(TFT_BLACK);
  }  
}

void loop() {
  
  //  Update Screen
  RefreshScreen(); // Update digital time

  //  Check for OTA update
  ArduinoOTA.handle();
}