#define encoderPinA  2 // Connect encoder channel A to Arduino digital pin 2
#define encoderPinB  3 // Connect encoder channel B to Arduino digital pin 3
// Constants
const float cutoffFrequency = 200; /* for a faster response and can tolerate some high-frequency noise we chose 200 Hz as 
based on the assumption that the encoder generates pulses at a maximum frequency of 214.827 Hz we choose smth a bit lower and adjust it if needed.*/
const float samplingFrequency = 1000; // Sampling frequency in Hz (adjust as needed)
const float wheelDiameter =2.0 * 3.14159265359 * cutoffFrequency;
const float RC = 1.0 / wheelDiameter;
const int encoderCountsPerRevolution=540;

// Variables
float previousFilteredValue = 0.0;
float filteredValue = 0.0;
// Variables
volatile long encoderCount = 0; // Count of encoder pulses
unsigned long previousMillis = 0; // Time of the previous measurement
unsigned long interval = 1000; // Interval (in milliseconds) to measure counts per second
float position = 0.0; // Position in meters
long long counter =0;

void setup() {
  Serial.begin(9600);
  // Set encoder pins as inputs
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach interrupt to handle encoder pulses
  attachInterrupt(encoderPinA, updateEncoderA, CHANGE);
  attachInterrupt(encoderPinA, updateEncoderB, CHANGE);
}

void loop() {
  // Calculate elapsed time since the last measurement
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;

  // If the elapsed time exceeds the interval, take a measurement and reset the timer
  if (elapsedTime >= interval) {
    float countsPerSecond = (float)encoderCount / (elapsedTime / 1000.0); // Calculate counts per second
    encoderCount/=4; // because we made a higher resolution
    // Apply the LPF to the counts per second measurement
    filteredCount = (1.0 / (1.0 + RC)) * (filteredCount + countsPerSecond - filteredCount);
    counter+=filteredCount;
    // Print the filtered result to the serial monitor
    Serial.print("Filtered Counts per second: ");
    Serial.println(filteredCount);
    position = (float)counter / (encoderCountsPerRevolution / (3.14159265359 * wheelDiameter));
    Serial.print("Position right now");
    Serial.println(position, 4); // print the position with 4 decimal spaces
    // Reset the encoder count, timer, and LPF variables
    encoderCount = 0;
    previousMillis = currentMillis;
    filteredCount = 0.0;
  
}

void updateEncoderA(void)
{
  if(digitalRead(encoderPinA) != digitalRead(encoderPinB))
    encoderCount ++;
  else
    encoderCount --;
}

void updateEncoderB(void)
{
  if(digitalRead(encoderPinA) == digitalRead(encoderPinB))
    encoderCount ++;
  else
    encoderCount --;
}
