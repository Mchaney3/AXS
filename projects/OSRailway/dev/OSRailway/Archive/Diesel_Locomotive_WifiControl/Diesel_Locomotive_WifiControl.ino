/* Copyright Olle Sköld 2017
 * 
 * Wifi accesspoitn using NodeMCU and ESP8266.
 * 
 * This software is provided free and you may use, change and distribute as you like.
 *
 */

#include <WiFi.h>
#include <WiFiClient.h> 
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "TimerObject.h"

#define forward 0
#define backward 1
/* Set these to your desired credentials. */
const char *ssid = "chlabs_bot";
const char *password = "chlabsrobotseverywhere";


bool highBeamActive = false;
bool motionActive = false;
bool motionDirection = forward;
int target_speed;
int actual_speed;
int acceleration_step = 5;

int LED_BL2 = 15; // Red light on side 2 GPIO15
int LED_BL1 = 13; // Red light on side 1 GPIO13
int LED_HB2 = 14; // High beam side 2 GPIO12
int LED_HB1 = 12; // High beam side 1 GPIO14

int LED_HDL = 2;  //PWM for headlights controlled by motor board

int motor_FORWARD = 5; //GPIO5
int motor_BACKWARD = 4; //GPIO4

TimerObject *timer1 = new TimerObject(10);

WebServer server(80);

/************************************************************************** 
 *  
 *  Server IP address is: http://192.168.4.1
 *  
 *  When you have connected your device to the wifi network, open the browser and enter the IP address above.
 *  
 */
void handleRoot() {
  /************************************************************************ 
  * This is the web page sent to the connected device. There are lots of memory available so this page can be made much more complex. 
  * It uses Ajax to send commands which means the page doesn't need to be reloaded each time a button is pressed. In this way, 
  * the page can be made much more graphically complex without suffering from long reloading delays.
  */
  String html ="<html> <header> <style type=\"text/css\"> body { background: #595B82; } #header { height: 30px; margin-left: 2px; margin-right: 2px; background: #d5dfe2; border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; font-family: \"arial-verdana\", arial; padding-top: 10px; padding-left: 20px; } #speed_setting { float: right; margin-right: 10px; } #control { height: 200px; margin-top: 4px; margin-left: 2px; margin-right: 2px; background: #d5dfe2; border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; font-family: \"arial-verdana\", arial; } .button { border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; margin: 5px; text-align: center; float: left; padding-top: 20px; height: 50px; background: #FFF} .long { width: 30%; } .short { width: 100px; } #message_box { float: left; margin-bottom: 0px; width: 100%; } #speed_slider {width: 300px;}</style> <script type=\"text/javascript\"> function updateSpeed(speed) { sendData(\"updateSpeed?speed=\"+speed); } function runForward(){ var speed = document.getElementById(\"speed_slider\").value; sendData(\"run?dir=forward&speed=\"+speed); } function runBackward(){ var speed = document.getElementById(\"speed_slider\").value; sendData(\"run?dir=backward&speed=\"+speed); } function sendData(uri){ var messageDiv = document.getElementById(\"message_box\"); messageDiv.innerHTML = \"192.168.4.1/\"+uri; var xhr = new XMLHttpRequest(); xhr.open('GET', 'http://192.168.4.1/'+uri); xhr.withCredentials = true; xhr.setRequestHeader('Content-Type', 'text/plain'); xhr.send(); } </script> </header> <body> <div id=\"header\"> OS-Railway Wifi Hectorrail 141 <div id=\"speed_setting\"> Speed <input id=\"speed_slider\" type=\"range\" min=\"0\" max=\"1023\" step=\"10\" value=\"512\" onchange=\"updateSpeed(this.value)\"> </div> </div> <div id=\"control\"> <div id=\"button_backward\" class=\"button long\" onclick=\"runBackward()\"> Run backward </div> <div id=\"button_stop\" class=\"button long\" onclick=\"updateSpeed(0)\"> Stop </div> <div id=\"button_forward\" class=\"button long\" onclick=\"runForward()\"> Run forward </div> <div id=\"button_light_off\" class=\"button long\" onclick=\"sendData('lightoff')\"> Headlight off </div> <div id=\"button_light_on\" class=\"button long\" onclick=\"sendData('lighton')\"> Headlight on </div> <div id=\"button_emergency\" class=\"button long\" onclick=\"sendData('stop')\"> Emergency stop </div> <div id=\"message_box\"> </div> </div> </body> </html>";
  server.send(200, "text/html", html);
}

void motionControl(){
  if(motionActive){
    if(actual_speed < target_speed){
      actual_speed = actual_speed + acceleration_step;
    } else if(actual_speed > target_speed){
      actual_speed = actual_speed - acceleration_step;
    }    
    if(actual_speed > 1023){
      actual_speed = 1023;
    }
    if(actual_speed < 0){
      actual_speed = 0;
    }
  }  else {
    actual_speed = 0;
  }
}

/**************************************************
 *  Motor operation
 */

void motorOperation(){

  motionActive = true;  
  String move_dir = server.arg("dir");
  String motion_speed = server.arg("speed");
  target_speed = motion_speed.toInt();  

  //Execute change of direction, but only if actual_speed is below 100, otherwise set target speed to 0 and initiate a slowdown.  
  if(move_dir == "backward"){
    if(motionDirection != backward && actual_speed > 100){
      target_speed = 0;
    } else {
      motionDirection = backward;
    }
      analogWrite(motor_FORWARD, 0);
      analogWrite(motor_BACKWARD, target_speed);
      Serial.print("Direction: "); Serial.print(motionDirection); Serial.print("\r\nTarget Speed: "); Serial.println(target_speed);
  } else {
    if(motionDirection != forward && actual_speed > 100){
      target_speed = 0;
    } else {
      motionDirection = forward;
    }
      analogWrite(motor_BACKWARD, 0);
      analogWrite(motor_FORWARD, target_speed);
      Serial.print("Direction: "); Serial.print(motionDirection); Serial.print("\r\nTarget Speed: "); Serial.println(target_speed);
  }
  digitalWrite(LED_BL2, motionDirection);
  digitalWrite(LED_BL1, !motionDirection);
    
  if(highBeamActive){
    digitalWrite(LED_HB2, motionDirection);
    digitalWrite(LED_HB1, !motionDirection);
  } else {
    digitalWrite(LED_HB2, 0);
    digitalWrite(LED_HB1, 0);
  }

  
}

void updateSpeed(){
  String motion_speed = server.arg("speed");
  target_speed = motion_speed.toInt();  
}

void switchLightOn(){
    highBeamActive = true;
    digitalWrite(LED_HB2, motionDirection);
    digitalWrite(LED_HB1, !motionDirection);
}

void switchLightOff(){
    highBeamActive = false;
    digitalWrite(LED_HB2, 0);
    digitalWrite(LED_HB1, 0);  
}

void motionStop(){
  motionActive = false;
  analogWrite(motor_FORWARD, 0);
  analogWrite(motor_BACKWARD, 0);
}

/*********************************************************************
 * SETUP
 */

void setup() {
	delay(1000);

  pinMode(LED_BL2, OUTPUT);
  pinMode(LED_BL1, OUTPUT);
  pinMode(LED_HB2, OUTPUT);
  pinMode(LED_HB1, OUTPUT);
  
  pinMode(LED_HDL, OUTPUT);
  
  pinMode(motor_FORWARD, OUTPUT);
  pinMode(motor_BACKWARD, OUTPUT);
  

  
  digitalWrite(LED_BL2, 0);
  digitalWrite(LED_BL1, 0);
  digitalWrite(LED_HB2, 0);
  digitalWrite(LED_HB1, 0);

  analogWrite(motor_FORWARD, 0);
  analogWrite(motor_BACKWARD, 0);
  analogWrite(LED_HDL, 1023);

  timer1->setOnTimer(&motionControl);
  timer1->Start();
  
	Serial.begin(115200);
  delay(1000);
  Serial.println("Booting");
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
  ArduinoOTA.setHostname("DieselLoco");

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

 /****************************************************************************
  * Here you find the functions that are called when the browser sends commands. 
  */
  server.on("/", handleRoot);
  server.on("/run", motorOperation);
  server.on("/stop", motionStop);
  server.on("/lighton", switchLightOn);
  server.on("/lightoff", switchLightOff);
  server.on("/updateSpeed", updateSpeed);
  server.begin();
}

void loop() {
	server.handleClient();
  timer1->Update();
  ArduinoOTA.handle();
}
