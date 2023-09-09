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
  2812B LED
  Speed Control


  //Needed
  Audio
  SD Card
  NFC
*/
#include <stdio.h>

//SD Libraries
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

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
#include "driver/i2s.h"                 // Library of I2S routines, comes with ESP32 standard install

// SD
#define SD_FAT_TYPE 1
//
// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 15;
//
// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 13;
const uint8_t SOFT_SCK_PIN  = 14;
// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
SdFat32 sd;
File32 file;
File32 WavFile;                                 // Object for root of SD card directory


// I2S Connections
#define I2S_DOUT      25          // i2S Data out oin
#define I2S_BCLK      27          // Bit clock
#define I2S_LRC       26          // Left/Right clock, also known as Frame clock or word select
#define I2S_NUM       0           // i2s port number

// Wav File reading
#define NUM_BYTES_TO_READ_FROM_FILE 1024    // How many bytes to read from wav file at a time

//LED Settings
#define LED_PIN    4  // digital pin used to drive the LED strip
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
int motor_FORWARD = 5; //GPIO5
int motor_BACKWARD = 4; //GPIO4
int motor_SPEED = 0;
int led_BRIGHTNESS = 127;

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
BLECharacteristic *pCharacteristic;

static const i2s_config_t i2s_config = 
{
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,                                 // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // high interrupt priority
    .dma_buf_count = 8,                                   // 8 buffers
    .dma_buf_len = 64,                                    // 64 bytes per buffer, so 8K of buffer space
    .use_apll=0,
    .tx_desc_auto_clear= true, 
    .fixed_mclk=-1    
};

// These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
// required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
// further information regarding these other connections.

static const i2s_pin_config_t pin_config = 
{
    .bck_io_num = I2S_BCLK,                           // The bit clock connectiom, goes to pin 27 of ESP32
    .ws_io_num = I2S_LRC,                             // Word select, also known as word select or left right clock
    .data_out_num = I2S_DOUT,                         // Data out from the ESP32, connect to DIN on 38357A
    .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
};

struct WavHeader_Struct
{
    //   RIFF Section    
    char RIFFSectionID[4];      // Letters "RIFF"
    uint32_t Size;              // Size of entire file less 8
    char RiffFormat[4];         // Letters "WAVE"
    
    //   Format Section    
    char FormatSectionID[4];    // letters "fmt"
    uint32_t FormatSize;        // Size of format section less 8
    uint16_t FormatID;          // 1=uncompressed PCM
    uint16_t NumChannels;       // 1=mono,2=stereo
    uint32_t SampleRate;        // 44100, 16000, 8000 etc.
    uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
    uint16_t BlockAlign;        // =Channels * (BitsPerSample/8)
    uint16_t BitsPerSample;     // 8,16,24 or 32
  
    // Data Section
    char DataSectionID[4];      // The letters "data"
    uint32_t DataSize;          // Size of the data that follows
}WavHeader;

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
          setMotorSpeed(motor_SPEED);
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

void setup() {
  int cpuSpeed = getCpuFrequencyMhz();
  Serial.begin(115200);
  Serial.println("Booting");
  delay(1000);
  Serial.setDebugOutput(true);
  Serial.print("\nESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.println(F("START " __FILE__ " from " __DATE__));
  Serial.println("\nCPU running at " + String(cpuSpeed) + "MHz");
  Serial.println("");
  
  if (!sd.begin(SD_CONFIG)) 
  {
        Serial.println("Error talking to SD card!");
        while(true);                  // end program        
  } else
  {
    Serial.println("SD Card Initialized!");
  }
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
    if(ValidWavData(&WavHeader))                        // optional if your sure the WAV file will be valid.
      i2s_set_sample_rates(i2s_num, WavHeader.SampleRate);      //set sample rate
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
    Serial.print("*** Sent Value: ");
    Serial.print(txString);
    Serial.println(" ***");

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
  PlayWav();                                            // Have to keep calling this to keep the wav file playing

    // Your normal code to do your task can go here
}

void PlayWav()
{
  static bool ReadingFile=true;                       // True if reading file from SD. false if filling I2S buffer
  static byte Samples[NUM_BYTES_TO_READ_FROM_FILE];   // Memory allocated to store the data read in from the wav file
  static uint16_t BytesRead;                          // Num bytes actually read from the wav file which will either be
                                                      // NUM_BYTES_TO_READ_FROM_FILE or less than this if we are very
                                                      // near the end of the file. i.e. we can't read beyond the file.

  if(ReadingFile)                                     // Read next chunk of data in from file if needed
  {
    BytesRead=ReadFile(Samples);                      // Read data into our memory buffer, return num bytes read in
    ReadingFile=false;                                // Switch to sending the buffer to the I2S
  }
  else
    ReadingFile=FillI2SBuffer(Samples,BytesRead);        // We keep calling this routine until it returns true, at which point
                                                      // this will swap us back to Reading the next block of data from the file.
                                                      // Reading true means it has managed to push all the data to the I2S 
                                                      // Handler, false means there still more to do and you should call this
                                                      // routine again and again until it returns true.
}

uint16_t ReadFile(byte* Samples)
{
    static uint32_t BytesReadSoFar=0;                   // Number of bytes read from file so far
    uint16_t BytesToRead;                               // Number of bytes to read from the file
    
    if(BytesReadSoFar+NUM_BYTES_TO_READ_FROM_FILE>WavHeader.DataSize)   // If next read will go past the end then adjust the 
      BytesToRead=WavHeader.DataSize-BytesReadSoFar;                    // amount to read to whatever is remaining to read
    else
      BytesToRead=NUM_BYTES_TO_READ_FROM_FILE;                          // Default to max to read
      
    WavFile.read(Samples,BytesToRead);                  // Read in the bytes from the file
    BytesReadSoFar+=BytesToRead;                        // Update the total bytes red in so far
    
    if(BytesReadSoFar>=WavHeader.DataSize)              // Have we read in all the data?
    {
      WavFile.seek(44);                                 // Reset to start of wav data  
      BytesReadSoFar=0;                                 // Clear to no bytes read in so far                            
    }
    return BytesToRead;                                 // return the number of bytes read into buffer
}

bool FillI2SBuffer(byte* Samples,uint16_t BytesInBuffer)
{
    // Writes bytes to buffer, returns true if all bytes sent else false, keeps track itself of how many left
    // to write, so just keep calling this routine until returns true to know they've all been written, then
    // you can re-fill the buffer
    
    size_t BytesWritten;                        // Returned by the I2S write routine, 
    static uint16_t BufferIdx=0;                // Current pos of buffer to output next
    uint8_t* DataPtr;                           // Point to next data to send to I2S
    uint16_t BytesToSend;                       // Number of bytes to send to I2S
    
    // To make the code eaier to understand I'm using to variables to some calculations, normally I'd write this calcs
    // directly into the line of code where they belong, but this make it easier to understand what's happening
    
    DataPtr=Samples+BufferIdx;                               // Set address to next byte in buffer to send out
    BytesToSend=BytesInBuffer-BufferIdx;                     // This is amount to send (total less what we've already sent)
    i2s_write(i2s_num,DataPtr,BytesToSend,&BytesWritten,1);  // Send the bytes, wait 1 RTOS tick to complete
    BufferIdx+=BytesWritten;                                 // increasue by number of bytes actually written
    
    if(BufferIdx>=BytesInBuffer)                 
    {
      // sent out all bytes in buffer, reset and return true to indicate this
      BufferIdx=0; 
      return true;                             
    }
    else
      return false;       // Still more data to send to I2S so return false to indicate this
}

bool ValidWavData(WavHeader_Struct* Wav)
{
  
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) 
  {    
    Serial.print("Invalid data - Not RIFF format");
    return false;        
  }
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Invalid data - Not Wave file");
    return false;           
  }
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) 
  {
    Serial.print("Invalid data - No format section found");
    return false;       
  }
  if(memcmp(Wav->DataSectionID,"data",4)!=0) 
  {
    Serial.print("Invalid data - data section not found");
    return false;      
  }
  if(Wav->FormatID!=1) 
  {
    Serial.print("Invalid data - format Id must be 1");
    return false;                          
  }
  if(Wav->FormatSize!=16) 
  {
    Serial.print("Invalid data - format section size must be 16.");
    return false;                          
  }
  if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
  {
    Serial.print("Invalid data - only mono or stereo permitted.");
    return false;   
  }
  if(Wav->SampleRate>48000) 
  {
    Serial.print("Invalid data - Sample rate cannot be greater than 48000");
    return false;                       
  }
  if((Wav->BitsPerSample!=8)& (Wav->BitsPerSample!=16)) 
  {
    Serial.print("Invalid data - Only 8 or 16 bits per sample permitted.");
    return false;                        
  }
  return true;
}


void DumpWAVHeader(WavHeader_Struct* Wav)
{
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
  {
    Serial.print("Not a RIFF format file - ");    
    PrintData(Wav->RIFFSectionID,4);
    return;
  } 
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Not a WAVE file - ");  
    PrintData(Wav->RiffFormat,4);  
    return;
  }  
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
  {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID,3);      
    return;
  } 
  if(memcmp(Wav->DataSectionID,"data",4)!=0)
  {
    Serial.print("data ID not present - "); 
    PrintData(Wav->DataSectionID,4);
    return;
  }  
  // All looks good, dump the data
  Serial.print("Total size :");Serial.println(Wav->Size);
  Serial.print("Format section size :");Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");Serial.println(Wav->FormatID);
  Serial.print("Channels :");Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");Serial.println(Wav->DataSize);
}

void PrintData(const char* Data,uint8_t NumBytes)
{
    for(uint8_t i=0;i<NumBytes;i++)
      Serial.print(Data[i]); 
      Serial.println();  
}
