const int CW_pin = 9;       //connect Arduino pin 9 to In1 on L298N
const int CCW_pin = 10;     //connect Arduino pin 10 to In2 on L298N
int pwm = 0;                //Integer to control motor speed
int dir = 0;                //Integer to control motor direction
String command = "";        //String to hold commands from Android App
long display_updated_time;  //timestamp of last screen update
int count = 0;              //Integer to count pulses from motor encoder
long rotation_time = 0;     //Used to time 1 complete rotation
float rpm = 0.0;            //Calculated rotation speed of motor


void setup() {
  Serial.begin(115200);
  pinMode(CW_pin, OUTPUT);
  pinMode(CCW_pin, OUTPUT);
  analogWrite(CW_pin,0);              
  analogWrite(CCW_pin,0);
  attachInterrupt(digitalPinToInterrupt(2), encoder, RISING); //encoder trigger
  display_updated_time = millis();                 //initialize screen update time
  rotation_time = millis();
}

void loop() {

  command = "";
  if (Serial.available() > 0) {               //check for string commands from AppInventor
    command = Serial.readString();            //store command as a string
  }

  if (command == "a") dir = 0;
  
  if (command == "b") dir = 1;
  
  if (command.charAt(0) == 'c') {             //If command starts with the letter "c"
    command.remove(0,1);                      //remove the first letter "c" from the string "c125" becomes "125"
    pwm = command.toInt();                    //convert the rest of string into an integer 
  }
  
  if (millis() > display_updated_time + 2000) {   //Update rpm on AppInventor App every 2 seconds
    Serial.print("RPM = ");
    Serial.println(rpm,2);
    display_updated_time = millis();
  }
  
  if (dir==0){
    analogWrite(CW_pin,pwm);
    analogWrite(CCW_pin,0);
  }
  
  if (dir==1){
    analogWrite(CW_pin,0);
    analogWrite(CCW_pin,pwm);
  }
  
}

void encoder() {                            //This encoder pulses 280 times per revolution
  count++;
  if (count == 279) {
    rpm = 60000.0 / float(millis() - rotation_time);
    count = 0;
    rotation_time = millis();
  }
}                              
