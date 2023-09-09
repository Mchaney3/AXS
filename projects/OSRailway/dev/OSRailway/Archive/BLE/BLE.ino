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
*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const char *pwmDelimiter = ",";
int motor_FORWARD = 5; //GPIO5
int motor_BACKWARD = 4; //GPIO4
int motor_SPEED = 0;
const char* direction_FORWARD = "DF";
const char* direction_BACKWARD = "DB";
const char* direction_FULLSTOP = "FS";
const char* pwm_SPEED = "SPD";
const char* pwm_LED1 = "LED1_PWM";
const char* LED1_OFF = "LED1_0";
const char* LED1_ON = "LED1_1";


BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
const int readPin = 25; // Use GPIO number. See ESP32 board pinouts
const int LED = 19; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

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
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }
         Serial.println();
         
        // Do stuff based on the command received from the app
        if (rxValue == LED1_ON) { 
          Serial.println("Turning LED1 ON!");
          digitalWrite(LED, HIGH);
        }
        else if (rxValue == LED1_OFF) {
          Serial.println("Turning LED1 OFF!");
          digitalWrite(LED, LOW);
        }
        else if (rxValue == direction_FORWARD) {
          Serial.println("Chugaluggin forward!");
          analogWrite(motor_BACKWARD, 0);  //  Set motor to half speed using PWM
          delay(10);
          analogWrite(motor_FORWARD, 511);  //  Set motor to half speed using PWM
        }
        else if (rxValue == direction_BACKWARD) {
          Serial.println("Chugaluggin Backurds!");
          analogWrite(motor_FORWARD, 0);  //  Set motor to half speed using PWM
          delay(10);
          analogWrite(motor_BACKWARD, 511);  //  Set motor to half speed using PWM
        }
        else if (rxValue == direction_FULLSTOP) {
          Serial.println("No more chugaluggin!");
          analogWrite(motor_FORWARD, 0);  //  Set motor to half speed using PWM
          analogWrite(motor_BACKWARD, 0);  //  Set motor to half speed using PWM
        }
        else if (rxValue.find(",") != -1) {

          char *pwmDevice = strtok(rxValue, pwmDelimiter);
          char *pwmValue = strtok(NULL, pwmDelimiter);
          
          if (pwmDevice == pwm_SPEED) {
            motor_SPEED = pwmValue;
            Serial.print("Speed: ");  Serial.println(motor_SPEED);
          }
          else if (pwmDevice == pwm_LED1){
            pwm_LED1 = pwmValue;
            Serial.print("LED 1 Brightness: ");  Serial.println(pwm_LED1);
          }
        }
        Serial.println();
        Serial.println("*********");
      }
    }
};
const char blename[] = "DieselLoco";
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(motor_FORWARD, OUTPUT);
  pinMode(motor_BACKWARD, OUTPUT);
  analogWrite(motor_FORWARD, motor_SPEED);
  analogWrite(motor_BACKWARD, motor_SPEED);
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Booting");
  
  // Create the BLE Device
  BLEDevice::init(blename); // Give it a name

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
  delay(1000);
}
