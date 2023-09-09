/*
 * 
 * Serial Communication: D0 (RX0) and D1 (TX1); 
 * Serial 1: D19 (RX1) and D18 (TX1); 
 * Serial 2: D17 (RX2) and D16 (TX2); 
 * Serial 3: D15 (RX3) and D14 (TX3).
 * 
 */

#include <IBusBM.h>
IBusBM IBus; // IBus object for receivig signals from transmitter/receiver

int directionPinA = 12;
int pwmPinA = 3;
int brakePinA = 9;

//uncomment if using channel B, and remove above definitions
int directionPinB = 13;
int pwmPinB = 11;
int brakePinB = 8;

//boolean to switch direction
bool directionStateA;
bool directionStateB;

void setup() {

  // Init
  Serial.begin(9600);   //  debug
  IBus.begin(Serial1);  //  iBUS connected to 1: D19 (RX1) and D18 (TX1);
  //Serial2.begin(115200);  //  ESP32 Comms Serial Serial 2: D17 (RX2) and D16 (TX2);
  
  //define pins
  pinMode(directionPinA, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  
  pinMode(directionPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);

  Serial.println("Motor's initialized\n");
  Serial.println("Wait for receiver\n");
  while (IBus.cnt_rec==0){
    Serial.print(".");
    delay(100);
  }
  Serial.println("Init done\n");

}

void speedturn(int speed, int angle) {
  // set speed (-400 -> +400) and turn (-400 -> +400)
  // turn vehicle by providing different speed settings to the motors.
  // angle can be positive (right turn) or negative (left turn).
  // If the vehicle is already stopped, the vehicle will turn in place.
  //int pwmA = speed + angle;
  //int pwmB = speed - angle;
  analogWrite(pwmPinA, (speed + angle));
  analogWrite(pwmPinB, (speed - angle));

  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
}

int savespd=0, saveturn=0, pwmSpeed;

void loop() {

  int spd, turn;
  // turn = ((int) IBus.readChannel(0));
  // spd = ((int) IBus.readChannel(1));

  spd = ((int) IBus.readChannel(1));
  turn = ((int) IBus.readChannel(0));

  if (spd == 1500) {
    pwmSpeed = 0;
    analogWrite(pwmPinA, pwmSpeed);
    analogWrite(pwmPinB, pwmSpeed);
    digitalWrite(brakePinA, HIGH);
    digitalWrite(brakePinB, HIGH);
  }

  else if (spd > 1500) {
    pwmSpeed = map(spd, 1501, 2000, 2, 255);
    digitalWrite(brakePinA, LOW);
    digitalWrite(brakePinB, LOW);
    digitalWrite(directionPinA, HIGH);
    digitalWrite(directionPinB, LOW);
  }
  
  else if (spd < 1500) {
    pwmSpeed = map(spd, 1499, 1000, 1, 255);
    digitalWrite(brakePinA, LOW);
    digitalWrite(brakePinB, LOW);
    digitalWrite(directionPinA, LOW);
    digitalWrite(directionPinB, HIGH);
  }

  Serial.print("spd = ");
  Serial.println(spd);
  Serial.print("pwmSpeed = ");
  Serial.println(pwmSpeed);

  analogWrite(pwmPinA, pwmSpeed);
  analogWrite(pwmPinB, pwmSpeed);
  /*

  //turn = map(

  //change direction every loop()
  directionStateA = !directionStateA;
  directionStateB = !directionStateB;
  
  //write a low state to the direction pin (13)
  if(directionStateA == false){
    digitalWrite(directionPinA, LOW);
    digitalWrite(directionPinB, HIGH);
  }
  
  //write a high state to the direction pin (13)
  else{
    digitalWrite(directionPinA, HIGH);
    digitalWrite(directionPinB, LOW);
  }
  
  //release breaks
  digitalWrite(brakePinA, LOW);
  digitalWrite(brakePinB, LOW);
  
  //set work duty for the motor
  analogWrite(pwmPinA, 255);
  analogWrite(pwmPinB, 255);
  
  delay(2000);
  
  //activate breaks
  digitalWrite(brakePinA, HIGH);
  digitalWrite(brakePinB, HIGH);
  
  //set work duty for the motor to 0 (off)
  analogWrite(pwmPinA, 0);
  analogWrite(pwmPinB, 0);
  
  delay(2000);
  */
}
