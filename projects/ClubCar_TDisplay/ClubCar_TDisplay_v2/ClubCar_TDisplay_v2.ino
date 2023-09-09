//  GPS
#include <Wire.h>
// TinyGpsPlus
// https://github.com/mikalhart/TinyGPSPlus
#include <TinyGPSPlus.h>
// EspSoftwareSerial
// https://github.com/plerup/espsoftwareserial/
#include <SoftwareSerial.h>
// DFRobot_QMC5883
// https://github.com/DFRobot/DFRobot_QMC5883
#include <DFRobot_QMC5883.h>

static const int RXPin = 18, TXPin = 17, sclPin = 1, sdaPin = 2;
static const uint32_t GPSBaud = 9600;
uint32_t targetTime = 0;                    // for next 1 second timeout

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
DFRobot_QMC5883 compass;

//  WIFI and OTA

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TFT_eSPI.h>

#define PIN_POWER_ON                 15

//const char *ssid = "chlabs_m";
//const char *password = "chaneys#1";

const char *ssid = "AXS";
const char *password = "D@d$wiF!forR0B07$";

/*
 An example digital clock using a TFT LCD screen to show the time.
 Demonstrates use of the font printing routines. (Time updates but date does not.)

 It uses the time of compile/upload to set the time
 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo...

 Make sure all the display driver and pin connections are correct by
 editing the User_Setup.h file in the TFT_eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Based on clock sketch by Gilchrist 6/2/2014 1.0

A few colour codes:

code	color
0x0000	Black
0xFFFF	White
0xBDF7	Light Gray
0x7BEF	Dark Gray
0xF800	Red
0xFFE0	Yellow
0xFBE0	Orange
0x79E0	Brown
0x7E0	Green
0x7FF	Cyan
0x1F	Blue
0xF81F	Pink

 */

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>

#define TFT_GREY 0x5AEB

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

TFT_eSprite img = TFT_eSprite(&tft);
#define BITS_PER_PIXEL 1

unsigned int colour = 0;

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    uint16_t transparent = 0; // The transparent colour, can only be 1 or 0
    img.setColorDepth(BITS_PER_PIXEL);         // Set colour depth first
    img.createSprite(x0, y0);                  // then create the sprite
    img.fillSprite(transparent);
    if (percentage == 0) {
        img.fillRoundRect(x0, y0, w, h, 3, TFT_GREY);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    img.drawRoundRect(x0, y0, w, h, 3, frameColor);
    img.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
    img.pushSprite(x0, y0, TFT_BLACK);
    //img.deleteSprite();
}


void updateGPS() {

  int satellites = 0;
  double mph = 0;
  double lat = 0;
  double lng = 0;
  double altitude = 0;

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

  char data[150] = {};
  sprintf(data, "-Satellites: %d\n-Latitude: %.6lf\n-Longitude: %.6lf\n-Altitude: %.2lf ft\n-Speed: %.1lf mph", satellites, lat, lng, altitude, mph);

  tft.setCursor(0, 10);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(4);
  tft.println("GPS Details");
  tft.setTextSize(2);
  tft.fillRect(0, 40, tft.width(), tft.height() / 2 - 40, TFT_BLACK);
  tft.println(data);
}

void setup(void) {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.println("Mike's Club Car Is Booting...");

  Serial.begin(115200);
  Serial.println("Booting");

  ss.begin(GPSBaud);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(3000);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("No WiFi...");
      tft.setCursor(0, 10);
      tft.println("No WiFi. OTA NOT AVAILABLE...");
      delay(5000);
  }
  else{
    tft.setCursor(0, 10);
    tft.println("WiFi COnnected. Initializing OTA...");
    ArduinoOTA.setHostname("Mike's Club Car");
    ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
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
        drawProgressBar(10, 30, 120, 15, percentage, TFT_WHITE, TFT_BLUE);
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
    tft.setCursor(0, 20);
    tft.println("IP address: ");
    tft.println(WiFi.localIP().toString());
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  tft.fillScreen(TFT_BLACK);
}

static void updateTime(TinyGPSTime &t) {
    tft.setTextSize(3);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.println(sz);
    
    // Update digital time
    int xpos = 0;
    int ypos = 10; // Top left corner ot clock text, about half way down
    tft.setCursor(xpos, ypos);
    tft.println(sz);
}

static void updateSpeedMPH(int val) {
    int xpos = 0;
    int ypos = 50;  //  Set location for RPM
    int engRPM = map(val, 0, 25, 0, 100); //  Map speed to engine RPM

    tft.setTextSize(5);

    //  drawProgressBar(xpos, ypos, (tft.width() / 2), 60, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
    if (val >= 20) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      drawProgressBar(xpos, ypos, (tft.width() / 2), 60, engRPM, 0xF800, 0xF800);
    }
    else if (val >= 16) {
      tft.setTextColor(0xFBE0, TFT_BLACK);
      drawProgressBar(xpos, ypos, (tft.width() / 2), 60, engRPM, 0xF800, 0xFBE0);
    }
    else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      drawProgressBar(xpos, ypos, (tft.width() / 2), 60, engRPM, TFT_WHITE, TFT_GREY);
    }

    ypos = 120; // Set location for Speed
    tft.setCursor(xpos, ypos);
    tft.println(val);

    tft.setTextSize(3);
    tft.setCursor(xpos + 90, ypos + 20);
    tft.println("mph");

    Serial.println(TinyGPSPlus::cardinal(gps.course.deg()));    //  TESTING - Build a seperate compass function to move this to. This is the bread and butter for heading
    Serial.println(gps.course.deg());    //  TESTING - Build a seperate compass function to move this to. This is the bread and butter for heading
    
}

void loop() {

  ArduinoOTA.handle();

  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      
      updateSpeedMPH(gps.speed.mph());  //  Disabled for testing. Enable for production
      
      //  updateGPS();
    }
  }

  if (targetTime < millis()) {  //  Update Time Every Second
        targetTime = millis() + 1000; // Set next update for 1 second later
        updateTime(gps.time); // Update digital time
      }
}



