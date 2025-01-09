/*
  SDWebServer - Example WebServer with SD Card backend for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the WebServer library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Have a FAT Formatted SD Card connected to the SPI port of the ESP8266
  The web root is the SD Card root folder
  File extensions with more than 3 charecters are not supported by the SD Library
  File Names longer than 8 charecters will be truncated by the SD library, so keep filenames shorter
  index.htm is the default index (works on subfolders as well)

  upload the contents of SdRoot to the root of the SDcard and access the editor by going to http://esp8266sd.local/edit
  To retrieve the contents of SDcard, visit http://esp32sd.local/list?dir=/
      dir is the argument that needs to be passed to the function PrintDirectory via HTTP Get request.

#define sdChipSelect SS
#define camChipSelect 9

SPI.begin(sdChipSelect);
SPI.begin(camChipSelect);

if (!SD.begin(sdChipSelect)
{

}
else
{
SPI.setClockDivider(sdChipSelect, 21);
}

SPI.begin(camChipSelect);

void setup(){
  // initialize the bus for the device on pin 4
  SPI.begin(4);
  // Set clock divider on pin 4 to 21
  SPI.setClockDivider(4, 21);
  // initialize the bus for the device on pin 10
  SPI.begin(10);
  // Set clock divider on pin 10 to 84
  SPI.setClockDivider(10, 84);
}


*/
#include "webpages.h"
#define FIRMWARE_VERSION "v0.0.1"

// configuration structure
struct Config {
  String ssid;               // wifi ssid
  String wifipassword;       // wifi password
  String httpuser;           // username to access web admin
  String httppassword;       // password to access web admin
  int webserverporthttp;     // http port number for web admin
};

// variables
Config config;                        // configuration
bool shouldReboot = false;            // schedule a reboot
AsyncWebServer *server;               // initialise webserver

// function defaults
String listFiles(bool ishtml = false);

void rebootESP(String message) {
  Serial.print("Rebooting ESP32: "); Serial.println(message);
  ESP.restart();
}

// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

// list all of the files, if ishtml=true, return html rather than simple text
String listFiles(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on SPIFFS");
  File root = SPIFFS.open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + " Size: " + humanReadableSize(foundfile.size()) + "\n";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  }
  root.close();
  foundfile.close();
  return returnText;
}

void httpTask( void * pvParameters ){
  
  if (MDNS.begin(deviceName)) {
    MDNS.addService("http", "tcp", 80);
    DBG_OUTPUT_PORT.println("MDNS responder started");
    DBG_OUTPUT_PORT.print("You can now connect to http://");
    DBG_OUTPUT_PORT.print(deviceName);
    DBG_OUTPUT_PORT.println(".local");
  }
  

  /************************     Initialize SD card and set NeoPixel color based on status    ************************/
  //SPIFFS.begin();
  if (SD.begin(SS))
  {
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    Serial.print("SD card initialized on pin: "); Serial.println(sdChipSelect);
    hasSD = true;
    ws2812fx.setSegment(0,  0,  ledCount-1, FX_MODE_BREATH, GREEN, led_SPEED, false); // segment 0 is led 0 a.k.a. Status LED
  } 
  else
  {
    // parameters: index, start, stop, mode, color, speed, reverse
    ws2812fx.setSegment(0,  0,  ledCount-1, FX_MODE_BREATH, BLUE, led_SPEED, false); // segment 0 is led 0 a.k.a. Status LED
    Serial.println("Failed to initialize SD card.");  // Use a bool to continue booting and disable SD read/write in sketch
    hasSD = false;
  }
  ws2812fx.init();
  ws2812fx.setBrightness(led_BRIGHTNESS);
  //ws2812fx.setSegment(0,  0,  ledCount-1, FX_MODE_BREATH, GREEN, led_SPEED, false); // segment 0 is led 0 a.k.a. Status LED
  ws2812fx.start();
  Serial.print("Neopixels initialized on pin: "); Serial.println(ledPin);

  Serial.print("SD Free: "); Serial.println(humanReadableSize((SD.totalBytes() - SD.usedBytes())));
  Serial.print("SD Used: "); Serial.println(humanReadableSize(SD.usedBytes()));
  Serial.print("SD Total: "); Serial.println(humanReadableSize(SD.totalBytes()));

  Serial.println(listFiles());

  Serial.println("Loading Configuration ...");

config.ssid = ssid;
  config.wifipassword = password;
  config.httpuser = http_username;
  config.httppassword = http_password;
  config.webserverporthttp = webserverporthttp;
  
Serial.println("\n\nNetwork Configuration:");
  Serial.println("----------------------");
  Serial.print("         SSID: "); Serial.println(WiFi.SSID());
  Serial.print("  Wifi Status: "); Serial.println(WiFi.status());
  Serial.print("Wifi Strength: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print("          MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("           IP: "); Serial.println(WiFi.localIP());
  Serial.print("       Subnet: "); Serial.println(WiFi.subnetMask());
  Serial.print("      Gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("        DNS 1: "); Serial.println(WiFi.dnsIP(0));
  Serial.print("        DNS 2: "); Serial.println(WiFi.dnsIP(1));
  Serial.print("        DNS 3: "); Serial.println(WiFi.dnsIP(2));
  Serial.println();

  // configure web server
  Serial.println("Configuring Webserver ...");
  server = new AsyncWebServer(config.webserverporthttp);
  configureWebServer();

  // startup web server
  Serial.println("Starting Webserver ...");
  server->begin();

  for(;;){
    if (shouldReboot) {
    rebootESP("Web Admin Initiated Reboot");
  }
    delay(10);//allow the cpu to switch to other tasks
  }
}
