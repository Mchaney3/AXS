/*
   control app https://x.thunkable.com/copy/e8f633a8a9a3e979025112e18446d3b4
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 

   Use this to define multiple segments on the fly or define them as a variable
   // parameters: index, start, stop, mode, color, speed, reverse
  //ws2812fx.setSegment(0,  0,  9, FX_MODE_BLINK, 0xFF0000, 1000, false); // segment 0 is leds 0 - 9
  //ws2812fx.setSegment(1, 10, 19, FX_MODE_SCAN,  0x00FF00, 1000, false); // segment 1 is leds 10 - 19
  //ws2812fx.setSegment(2, 20, 29, FX_MODE_COMET, 0x0000FF, 1000, true);  // segment 2 is leds 20 - 29

  //Implemented
  OTA
  BLE
  2812B LED control
  motor Speed and direction Control
  SD Card


  //Needed
  Audio
  NFC
  LED Control in phone app
*/
#include <stdio.h>

//SD Libraries
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

//Display Lirary
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

//BLE Libraries
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//WiFi Libraries
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//Neopixel Lirary
#include <WS2812FX.h>

//I2S Audio Library
#include <Audio.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <play_fs_wav.h>

AudioPlayFSWav           playWav1;
// Use one of these 3 output types: Digital I2S, Digital S/PDIF, or Analog DAC
//AudioOutputI2S           audioOutput;
//AudioOutputSPDIF       audioOutput;
AudioOutputAnalogStereo  audioOutput;    // Dual DACs
AudioConnection          patchCord1(playWav1, 0, audioOutput, 1);
AudioConnection          patchCord2(playWav1, 1, audioOutput, 0);
//AudioControlSGTL5000     sgtl5000_1;
// I2S Connections
#define I2S_DOUT      14          // i2S Data out oin
#define I2S_BCLK      25          // Bit clock
#define I2S_LRC       26          // Left/Right clock, also known as Frame clock or word select
 // Create Audio object
Audio audio;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem QSPIFS;
bool SDOK=false, QSPIOK=false;

// SD
#define SD_FAT_TYPE 1
//
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 32;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 19;
const uint8_t SOFT_MOSI_PIN = 23;
const uint8_t SOFT_SCK_PIN  = 18;
// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
SdFat32 sd;
File32 file;
File32 WavFile;                                 // Object for root of SD card directory

//LED Settings
#define LED_PIN    2  // digital pin used to drive the LED strip
#define LED_COUNT 30  // number of LEDs on the strip

//BLE UUID's
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

bool motor_Direction_Forward = true;
bool isWavFile;
bool deviceConnected = false;
static const i2s_port_t i2s_num = I2S_NUM_0;  // i2s port number
const char* ssid = "chlabs_bot";
const char* password = "chlabsrobotseverywhere";
const char deviceName[] = "DieselLoco";
const char *pwmDelimiter = ",";
const char* direction_FORWARD = "DF";
const char* direction_BACKWARD = "DB";
const char* direction_FULLSTOP = "FS";
const char* pwm_SPEED = "SPD,";
const char* pwm_BRIGHTNESS = "BRT,";
String inString = "";    // string to hold input
float txValue = 0;
const int analogBatteryVoltage = 35; // Use GPIO number. See ESP32 board pinouts
int motor_FORWARD = 4; //GPIO4
int motor_BACKWARD = 5; //GPIO5
int motor_SPEED = 0;
int led_BRIGHTNESS = 127;
int cpuSpeed = getCpuFrequencyMhz();

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
BLECharacteristic *pCharacteristic;


void setMotorSpeed(int ms)  {
  if (motor_Direction_Forward)  {
    analogWrite(motor_FORWARD, ms);
  }
  else  {
    analogWrite(motor_BACKWARD, ms);
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      analogWrite(motor_FORWARD, 0);  //  Set motor to half speed using PWM
      analogWrite(motor_BACKWARD, 0);  //  Set motor to half speed using PWM  
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    
      pCharacteristic->setValue(txString);
  }
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
         Serial.println();
         
        // Do stuff based on the command received from the app
        if (rxValue == direction_FORWARD) {
          Serial.println("Chugaluggin forward!");
          analogWrite(motor_BACKWARD, 0);  //  Set motor to half speed using PWM
          motor_Direction_Forward = true;
          analogWrite(motor_FORWARD, motor_SPEED);  //  Set motor to half speed using PWM
          delay(10);
        }
        else if (rxValue == direction_BACKWARD) {
          Serial.println("Chugaluggin Backurds!");
          analogWrite(motor_FORWARD, 0);  //  Set motor to half speed using PWM
          motor_Direction_Forward = false;
          analogWrite(motor_BACKWARD, motor_SPEED);  //  Set motor to half speed using PWM
          delay(10);          
        }
        else if (rxValue == direction_FULLSTOP) {
          Serial.println("No more chugaluggin!");
          analogWrite(motor_FORWARD, 0);  //  Set motor to half speed using PWM
          analogWrite(motor_BACKWARD, 0);  //  Set motor to half speed using PWM
        }
        else if (rxValue.indexOf(pwm_SPEED) != -1) {
           Serial.print("Setting motor speed to: ");  Serial.println(rxValue.substring(4));
          motor_SPEED = rxValue.substring(4).toInt();
          Serial.println(motor_SPEED);
        }
        else if (rxValue.indexOf(pwm_BRIGHTNESS) != -1) {
          Serial.print("Setting brightness to: ");  Serial.println(rxValue.substring(4));
          led_BRIGHTNESS = rxValue.substring(4).toInt();
          Serial.println(led_BRIGHTNESS);
          ws2812fx.setBrightness(led_BRIGHTNESS);
        }
        Serial.println();
        Serial.println("*********");
      }
    }
};

void playFile(const char *filename)
{
  Serial.print("Playing file: "); Serial.println(filename);

  File f;
  
  if (SDOK) {
    f = sd.open(filename);
  } else if (QSPIOK) {
    f = QSPIFS.open(filename);
  }  
  // Start playing the file.  This sketch continues to
  // run while the file plays.
  if (!playWav1.play(f)) { 
    Serial.println("Failed to play");
    return;
  }

  // A brief delay for the library read WAV info
  delay(5);

  // Simply wait for the file to finish playing.
  while (playWav1.isPlaying()) {
    Serial.print(".");
    delay(100);
    // uncomment these lines if you audio shield
    // has the optional volume pot soldered
    //float vol = analogRead(15);
    //vol = vol / 1024;
    // sgtl5000_1.volume(vol);
    vol = 50;
  }
}

void setup() {
  delay(500);
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(8);
  SDOK = false;
  if (!sd.begin(SD_CONFIG)) 
  {
        Serial.println("Error talking to SD card!");
        while(true);                  // end program        
  } else
  {
    Serial.println("SD Initialized");
    SDOK = true;
  }

  QSPIOK = false;
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
  } else if (!QSPIFS.begin(&flash)) {
    Serial.println("Failed to mount QSPI filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
  } else {
    Serial.println("QSPI OK!");
    QSPIOK = true;
  }

  playFile("Audio/boothorn.wav");
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  Serial.begin(115200);
  Serial.println("Booting");
  delay(1000);
  Serial.setDebugOutput(true);
  Serial.print("\nESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.println(F("START " __FILE__ " from " __DATE__));
  Serial.println("\nCPU running at " + String(cpuSpeed) + "MHz");
  Serial.println("");
  
  Serial.println("Setting Pin Modes");
  pinMode(motor_FORWARD, OUTPUT);
  pinMode(motor_BACKWARD, OUTPUT);
  analogWrite(motor_FORWARD, motor_SPEED);
  analogWrite(motor_BACKWARD, motor_SPEED);

  Serial.println("Initializing Audio");
  //I2S Init
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin(i2s_num, &pin_config);
  // get the wav file from the SD card
  //WavFile = WavFile.open("/wavfile.wav");                   // Open the wav file
  if (!WavFile.open("Audio/boothorn.wav"))
  {
    Serial.println("Could not open 'wavfile.wav'");
  }
  else
  {
    WavFile.read((byte *) &WavHeader,44);               // Read in the WAV header, which is first 44 bytes of the file. 
                                                        // We have to typecast to bytes for the "read" function
    DumpWAVHeader(&WavHeader);                          // Dump the header data to serial, optional!
    //if(ValidWavData(&WavHeader))                        // optional if your sure the WAV file will be valid.
      i2s_set_sample_rates(i2s_num, WavHeader.SampleRate);      //set sample rate
      PlayWav();                                            // Have to keep calling this to keep the wav file playing
      Serial.println("Wav file Found");
  }
    
  ws2812fx.init();
  ws2812fx.setBrightness(led_BRIGHTNESS);

  // parameters: index, start, stop, mode, color, speed, reverse
  //ws2812fx.setSegment(0,  0,  9, FX_MODE_BLINK, 0xFF0000, 1000, false); // segment 0 is leds 0 - 9
  //ws2812fx.setSegment(1, 10, 19, FX_MODE_SCAN,  0x00FF00, 1000, false); // segment 1 is leds 10 - 19
  //ws2812fx.setSegment(2, 20, 29, FX_MODE_COMET, 0x0000FF, 1000, true);  // segment 2 is leds 20 - 29
  ws2812fx.setSegment(0,  0,  1, FX_MODE_STATIC, RED, 1000, false); // segment 0 is led 0 a.k.a. Status LED

  ws2812fx.start();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(deviceName);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      ws2812fx.setSegment(0,  0,  1, FX_MODE_BREATH, ORANGE, 1000, false); // segment 0 is led 0 a.k.a. Status LED
      ws2812fx.service();
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Create the BLE Device
  BLEDevice::init(deviceName); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  ws2812fx.setSegment(0,  0,  1, FX_MODE_STATIC, GREEN, 1000, false); // segment 0 is led 0 a.k.a. Status LEDLet's do it!
  
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (deviceConnected) {
    // Fabricate some arbitrary junk for now...
    txValue = random(5); // This could be an actual sensor reading!

    // Let's convert the value to a char array:
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    
//    pCharacteristic->setValue(&txValue, 1); // To send the integer value
//    pCharacteristic->setValue("Hello!"); // Sending a test message
    pCharacteristic->setValue(txString);
    
    pCharacteristic->notify(); // Send the value to the app!
    //Serial.print("*** Sent Value: ");
    //Serial.print(txString);
    //Serial.println(" ***");

    // You can add the rxValue checks down here instead
    // if you set "rxValue" as a global var at the top!
    // Note you will have to delete "std::string" declaration
    // of "rxValue" in the callback function.
//    if (rxValue.find("A") != -1) { 
//      Serial.println("Turning ON!");
//      digitalWrite(LED, HIGH);
//    }
//    else if (rxValue.find("B") != -1) {
//      Serial.println("Turning OFF!");
//      digitalWrite(LED, LOW);
//    }
  }

//  else restart service and wait 5 minutes for a connection, then restart/deep sleep.
  ws2812fx.service();
  ArduinoOTA.handle();
  audio.loop();

    // Your normal code to do your task can go here
}
