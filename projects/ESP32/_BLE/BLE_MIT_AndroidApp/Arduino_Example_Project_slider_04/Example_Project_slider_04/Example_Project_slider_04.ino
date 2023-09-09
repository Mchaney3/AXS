/*
 *  Arduino, HM-10, App Inventor 2
 * 
 *  Example_Project_slider_04
 *  By Martyn Currey. www.martyncurrey.com
*
* Turn LEDs on and off from an Android app via a HM-10 or from a physical switches connected to the Arduino
* Uses the following pins
* 
* Now with an addewd slider connand
*
* D9 - AltsoftSerial RX
* D8 - AltsoftSerial TX
* D7 - LED + resistor
* D6 - LED + resistor
* D5 - LED + resistor
* D4 - Button switch 
* D3 - Button switch 
* D2 - Button switch 
*/
 
// AltSoftSerial uses D9 for RX and D8 for TX. While using AltSoftSerial D10 cannot be used for PWM.
// Remember to use a voltage divider on the Arduino TX pin / Bluetooth RX pin
// Download from https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 
#include <AltSoftSerial.h>
AltSoftSerial BTserial; 
 
 
// Variables used for incoming data
const byte maxDataLength = 20;
char receivedChars[21] ;
boolean newData = false;
 
// Constants for hardware
const byte SWITCH_PIN[] = {2,3,4};
const byte LED_PIN[] = {5,6,7};
 
 
// general variables
boolean LED_State[] = {false,false,false};
boolean switch_State[] = {false,false,false};
boolean oldswitch_State[] = {false,false,false};
 
 
void setup()  
{
    for (byte pin = 0; pin < 3; pin++) 
    {
         // Set the button switch pins for input
         pinMode(SWITCH_PIN[pin], INPUT); 
 
         // Set the LED pins for output and make them LOW
         pinMode(LED_PIN[pin], OUTPUT);  digitalWrite(LED_PIN[pin],LOW);
    }
 
    // open serial communication for debugging
    Serial.begin(9600);
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
    Serial.println(" ");
 
    //  open software serial connection to the Bluetooth module.
    BTserial.begin(9600); 
    Serial.println("AltSoftSerial started at 9600"); 
 
    newData = false;
 
} // void setup()
 
 
void loop()  
{
    for (byte switchNum = 1; switchNum < 4; switchNum++) 
    {
           checkSwitch(switchNum);
    }
    recvWithStartEndMarkers();                // check to see if we have received any new commands
    if (newData)  {   processCommand();  }    // if we have a new command do something about it
}
 
 
/*
****************************************
* Function checkSwitch()
* checks the status of a button switch and turns an LED on or off depending on the switch status
* 
* passed:
*  
* global: 
*      switch_State[]
*      LED_State[]
*      oldswitch_State[]
*
* Returns:
*          
* Sets:
*      switch_State[]
*      LED_State[]
*      oldswitch_State[]
*/
void checkSwitch( byte pos)
{
     // pos = 1,2,3. Array pos = 0,1,2 so convert by subtracting 1
     pos = pos-1;
 
     // very simple debouce.
     boolean state1 = digitalRead(SWITCH_PIN[pos]); delay(1);
     boolean state2 = digitalRead(SWITCH_PIN[pos]); delay(1);
     boolean state3 = digitalRead(SWITCH_PIN[pos]); delay(1);
     if ((state1 == state2) && (state1==state3))   
     { 
          switch_State[pos] = state1;  
          if ( (switch_State[pos] == HIGH) && (oldswitch_State[pos] == LOW) )
          {
               LED_State[pos] = ! LED_State[pos];  // flip the status.
 
               // Rather than have a long list of possible commands I create the command as required.
               // Values in the temp command are replaced depending on which button switch has been pressed.
               char TMPcmd[8] = "[L,1,0]";
               TMPcmd[3] = pos+1+48;             // pos+1 should be 1,2, or 3.   convert a numeric value to ascii by adding 48. 
               TMPcmd[5] = LED_State[pos]+48;    // LED_State should be 0 or 1
               
               if (pos+1==1){ 
                                if ( LED_State[pos] ==0  ) { analogWrite(LED_PIN[pos],0);   }
                                else                       { analogWrite(LED_PIN[pos],255); }
                            }
                            
               else         { 
                                digitalWrite(LED_PIN[pos],LED_State[pos]);
                            }
               BTserial.print(TMPcmd);
               Serial.println(TMPcmd); 

               if (pos+1==1){ 
                                if ( LED_State[pos] ==0  ) { BTserial.print("[S,0]");  Serial.println("[S,0]");  }
                                else                       { BTserial.print("[S,255]");  Serial.println("[S,255]");  }
                            }
               
              
          }          
          oldswitch_State[pos] = switch_State[pos];
      }
}
 
 
 
 
/*
****************************************
* Function processCommand
* parses data commands contained in receivedChars[]
* receivedChars[] has not been checked for errors
* 
* passed:
*  
* global: 
*       receivedChars[]
*       newData
*
* Returns:
*          
* Sets:
*       receivedChars[]
*       newData
*/
void processCommand()
{
     Serial.print("receivedChars = ");   Serial.println(receivedChars);
 
    if (receivedChars[0] == 'L')      // do we have a LED command?
    {
        // we know the LED command has a fixed length "L10"
        // and the value at pos 1 is the LED and the value at pos 2 is 0 or 1 (on/off). 
        // 0 and 1 is the same as LOW and HIGH so we can use 0/1 instead of LOW/HIGH
 
        byte LEDnum = receivedChars[1] - 48;          // convert ascii to value by subtracting 48
        boolean LEDstatus = receivedChars[2] - 48;
 
        digitalWrite(LED_PIN[LEDnum-1],LEDstatus);
        LED_State[LEDnum-1] = LEDstatus;
    }


 
    if (receivedChars[0] == 'S')      // do we have a slider command?
    {
        // we know the slider command has a fixed length "S123"
        // and the value at pos 1,2, and 3 is the value. 
        // We need to convert the value from 3 ascii characters to a single value

        byte hundreds = (receivedChars[1]-48) * 100;
        byte tens = (receivedChars[2]-48) * 10;
        byte units = receivedChars[3]-48;
        byte value = hundreds + tens + units;
        
        analogWrite(LED_PIN[0],value);

    }


 
    receivedChars[0] = '\0';
    newData = false;
}
 
 
 
// function recvWithStartEndMarkers by Robin2 of the Arduino forums
// See  http://forum.arduino.cc/index.php?topic=288234.0
/*
****************************************
* Function recvWithStartEndMarkers
* reads serial data and returns the content between a start marker and an end marker.
* 
* passed:
*  
* global: 
*       receivedChars[]
*       newData
*
* Returns:
*          
* Sets:
*       newData
*       receivedChars
*
*/
void recvWithStartEndMarkers()
{
     static boolean recvInProgress = false;
     static byte ndx = 0;
     char startMarker = '[';
     char endMarker = ']';
     char rc;
 
     if (BTserial.available() > 0) 
     {
          rc = BTserial.read();
          if (recvInProgress == true) 
          {
               if (rc != endMarker) 
               {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx > maxDataLength) { ndx = maxDataLength; }
               }
               else 
               {
                     receivedChars[ndx] = '\0'; // terminate the string
                     recvInProgress = false;
                     ndx = 0;
                     newData = true;
               }
          }
          else if (rc == startMarker) { recvInProgress = true; }
     }
}
