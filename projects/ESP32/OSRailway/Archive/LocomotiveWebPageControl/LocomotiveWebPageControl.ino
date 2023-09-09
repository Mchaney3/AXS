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

int motor_AIN1 = 5; //GPIO5
int motor_AIN2 = 4; //GPIO4

TimerObject *timer1 = new TimerObject(10);

WebServer server(80);

void handleRoot() {
  /************************************************************************ 
  * This is the web page sent to the connected device. There are lots of memory available so this page can be made much more complex. 
  * It uses Ajax to send commands which means the page doesn't need to be reloaded each time a button is pressed. In this way, 
  * the page can be made much more graphically complex without suffering from long reloading delays.
  */
  String html ="<html> <header> <style type=\"text/css\"> body { background: #595B82; } #header { height: 30px; margin-left: 2px; margin-right: 2px; background: #d5dfe2; border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; font-family: \"arial-verdana\", arial; padding-top: 10px; padding-left: 20px; } #speed_setting { float: right; margin-right: 10px; } #control { height: 200px; margin-top: 4px; margin-left: 2px; margin-right: 2px; background: #d5dfe2; border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; font-family: \"arial-verdana\", arial; } .button { border-top: 1px solid #FFF; border-left: 1px solid #FFF; border-right: 1px solid #333; border-bottom: 1px solid #333; border-radius: 5px; margin: 5px; text-align: center; float: left; padding-top: 20px; height: 50px; background: #FFF} .long { width: 30%; } .short { width: 100px; } #message_box { float: left; margin-bottom: 0px; width: 100%; } #speed_slider {width: 300px;}</style> <script type=\"text/javascript\"> function updateSpeed(speed) { sendData(\"updateSpeed?speed=\"+speed); } function runForward(){ var speed = document.getElementById(\"speed_slider\").value; sendData(\"run?dir=forward&speed=\"+speed); } function runBackward(){ var speed = document.getElementById(\"speed_slider\").value; sendData(\"run?dir=backward&speed=\"+speed); } function sendData(uri){ var messageDiv = document.getElementById(\"message_box\"); messageDiv.innerHTML = \"192.168.4.1/\"+uri; var xhr = new XMLHttpRequest(); xhr.open('GET', 'http://192.168.4.1/'+uri); xhr.withCredentials = true; xhr.setRequestHeader('Content-Type', 'text/plain'); xhr.send(); } </script> </header> <body> <div id=\"header\"> OS-Railway Wifi Hectorrail 141 <div id=\"speed_setting\"> Speed <input id=\"speed_slider\" type=\"range\" min=\"0\" max=\"1023\" step=\"10\" value=\"512\" onchange=\"updateSpeed(this.value)\"> </div> </div> <div id=\"control\"> <div id=\"button_backward\" class=\"button long\" onclick=\"runBackward()\"> Run backward </div> <div id=\"button_stop\" class=\"button long\" onclick=\"updateSpeed(0)\"> Stop </div> <div id=\"button_forward\" class=\"button long\" onclick=\"runForward()\"> Run forward </div> <div id=\"button_light_off\" class=\"button long\" onclick=\"sendData('lightoff')\"> Headlight off </div> <div id=\"button_light_on\" class=\"button long\" onclick=\"sendData('lighton')\"> Headlight on </div> <div id=\"button_emergency\" class=\"button long\" onclick=\"sendData('stop')\"> Emergency stop </div> <div id=\"message_box\"> </div> </div> </body> </html>";
  server.send(200, "text/html", html);
}

void motorOperation(){

  String move_dir = server.arg("dir");
  if(move_dir == "backward"){
    digitalWrite(motor_AIN1, LOW);
    digitalWrite(motor_AIN2, HIGH);
  } else {
    digitalWrite(motor_AIN1, HIGH);
    digitalWrite(motor_AIN2, LOW);
  }
}

void motionStop(){
    digitalWrite(motor_AIN1, LOW);
    digitalWrite(motor_AIN2, LOW); 
}

void setup() {
  delay(1000);
  pinMode(motor_AIN1, OUTPUT);
  pinMode(motor_AIN2, OUTPUT);
//  timer1->setOnTimer(&motorOperation);
//  timer1->Start();
  //digitalWrite(motor_AIN1, HIGH);
  analogWrite(motor_AIN2, 128);
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
//  server.on("/lighton", switchLightOn);
//  server.on("/lightoff", switchLightOff);
//  server.on("/updateSpeed", updateSpeed);
  server.begin();
}

void loop() {
  server.handleClient();
  timer1->Update();
  ArduinoOTA.handle();
}
