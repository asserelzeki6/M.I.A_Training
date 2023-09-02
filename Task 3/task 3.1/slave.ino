//slave
#include <Wire.h>
const int ledPin = 9;     // LED connected to digital pin 9

int button1State = 0;
int button2State = 0;
int ledIntensity = 0;

void change(){
  button1State = Wire.read();
  button2State = Wire.read();
  
   if (button1State == HIGH && button2State == LOW) {
      ledIntensity = 128; // 50%
      Serial.println("Vector focused");
    } 
    else if (button1State == LOW && button2State == HIGH) {
      ledIntensity = 192; //75%
      Serial.println("Vector distracted");
    } 
    else if (button1State == HIGH && button2State == HIGH) {
      ledIntensity = 255; //100%
      Serial.println("Glitch");
    } 
    else {
      ledIntensity = 0; //0%
      Serial.println("No message");
    }
	
  analogWrite(ledPin, ledIntensity); 
}

void setup() {
  pinMode(ledPin, OUTPUT);
  Wire.begin(8);
  Wire.onReceive(change);              
  Serial.begin(9600);
}

void loop() {// nothing to do here
}

