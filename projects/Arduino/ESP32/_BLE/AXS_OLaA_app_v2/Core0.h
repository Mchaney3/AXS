#include "BluetoothSerial.h"
#include "BluetoothA2DPSink32.h"

BluetoothA2DPSink32 a2dp_sink; // Subclass of BluetoothA2DPSink

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  char msg[60];
  int len = snprintf(msg, sizeof(msg), "0x%x, %s\n", id, text);
  delay(1000);
}

void audio_state_changed(esp_a2d_audio_state_t state, void *ptr){
  Serial.println(a2dp_sink.to_str(state));
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr){
  Serial.println(a2dp_sink.to_str(state));
}

#define MAX_NUM_CHARS 16 // maximum number of characters read from the Serial Monitor
BluetoothSerial SerialBT;

String incomingString;
char scmd[MAX_NUM_CHARS];    // char[] to store incoming serial commands
bool scmd_complete = false;  // whether the command string is complete

void btCommandIncoming() {
// -------------------- Receive Bluetooth Commands ----------------------
  if (SerialBT.available()) {
    const byte numChars = 16;
    char receivedChars[numChars];
    char tempChars[numChars];                   // temporary array for use when parsing

    // variables to hold the parsed data
    char messageFromPC[numChars] = {0};
    int integerFromPC = 0;

    // my working code
    incomingString = SerialBT.readString();
    int incomingInt = incomingString.toInt();
    Serial.print("incomingString: "); Serial.println(incomingString);
    int ledSettingModifier = floor(incomingInt / 100);
    int ledModifierValue = incomingInt % 100;
    Serial.print("ledSettingModifier: ");  Serial.print(ledSettingModifier);  Serial.print("\nledModifierValue: "); Serial.println(ledModifierValue);
    switch (ledSettingModifier) {
      case 1:  
        ws2812fx.setMode(ledModifierValue); 
        
        Serial.print(F("Mode set to: "));
        Serial.print(ws2812fx.getMode());
        Serial.print(" - ");
        Serial.println(ws2812fx.getModeName(ws2812fx.getMode()));

        SerialBT.write(23);   /*   This is a callback to send data back to phone   */
        break;
      case 2:
        ledModifierValue = map(ledModifierValue, 0, 100, 0, 255); //  TODO: Create mapping of 10 brightness values in app
        Serial.print("ledModifierValue for Brightness: ");  Serial.println(ledModifierValue);
        ws2812fx.setBrightness(ledModifierValue); //    Right now I'm only mapping up to 99. Need to map 0-255
        Serial.print(F("Brightness set to: "));
        Serial.println(ws2812fx.getBrightness());
        break;

        //  still my working code. Hell from here
      case 3: {
        incomingString.toCharArray(receivedChars, 16);
        char * strtokIndx; // this is used by strtok() as an index
        strtokIndx = strtok(receivedChars,",");      // get the first part - the string
        //  Run same loop to move past 300
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        integerFromPC = atoi(strtokIndx);     // convert this part to an integer
        int r = integerFromPC;
        //  Red value mapped. Now we're looking at the green value in the array
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        integerFromPC = atoi(strtokIndx);     // convert this part to an integer
        int g = integerFromPC;
        //  Blue's Turn
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        integerFromPC = atoi(strtokIndx);     // convert this part to an integer
        int b = integerFromPC;
        Serial.print("r: "); Serial.println(r);  Serial.print("g: "); Serial.println(g);  Serial.print("b: ");  Serial.println(b);
        ws2812fx.setColor(r,g,b);
        break;
      };
      case 4:
        ledModifierValue = map(ledModifierValue, 0, 99, 4975, 50);
        ws2812fx.setSpeed(ledModifierValue); //    Right now I'm only mapping up to 99. Need to map 0-255
        Serial.print(F("Speed set to: "));
        Serial.println(ws2812fx.getSpeed());
        break;
    }
    ws2812fx.service();
    Serial.println();
    scmd[0] = '\0';         // reset the commandstring
    scmd_complete = false;  // reset command complete
  } 
}

void btTASK( void * pvParameters ){
  
  i2s_pin_config_t my_pin_config = {
    .bck_io_num = 15,   //    BCK
    .ws_io_num = 13,    //    LRCK
    .data_out_num = 14,   //    DIN
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.set_bits_per_sample(32); 
  a2dp_sink.start("");
  if(!SerialBT.begin(DeviceName)) {
    Serial.println("Could not start LED BLuetooth Control");
  } 
  else {
  Serial.println("Bluetooth Started");
  }
  for(;;){
    btCommandIncoming();
    delay(1);
  }
}
