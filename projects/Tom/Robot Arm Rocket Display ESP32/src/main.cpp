#include <Arduino.h>
#include <Servo.h>

String data;  //  Hold data from PC
char d1;      //  First character of data string
String x;
int servoval;
Servo s1;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);  //  Builtin LED
  s1.attach(9);         //  Servo on pin 9
}

void loop() {
  if(Serial.available()){
    data = Serial.readString();
    d1 = data.charAt(0);

    switch(d1){
      case 'A':
        digitalWrite(13, HIGH);
        break;

      case 'B':
        digitalWrite(13, LOW);  
        break;

      case 'S':                 //  First character of servo angle
        x = data.substring(1);
        servoval = x.toInt();
        s1.write(servoval);
        delay(100);             //  Wait for servi to finish movement
        break;
    }
  }
}