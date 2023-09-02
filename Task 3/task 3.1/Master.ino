//master

#include <Wire.h>
const int button1Pin = 2; // Button 1 connected to digital pin 2
const int button2Pin = 4; // Button 2 connected to digital pin 3

int button1State = 0;
int button2State = 0;

int	lastButton1State=0;
int lastButton2State=0;

void setup() {
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  Wire.begin();

  change(); // to work on the initial state 
}
void change(){
  
    Wire.beginTransmission(8); // 8 is the slave address of the second Arduino
    Wire.write(button1State);
    Wire.write(button2State);
    Wire.endTransmission();
    lastButton1State=button1State;
    lastButton2State=button2State;
}
void loop() {
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);
	if (button1State != lastButton1State || button2State != lastButton2State)
    {
      change();
    }
  delay(1000);
}
