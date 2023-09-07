#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#define MPU6050_ADDR  0x68 // MPU6050 I2C address
#define Gyro_Zout_ADDR  0x47

ros::NodeHandle nh;
std_msgs::Float32 float_msg;
ros::Subscriber<std_msgs::Float32> filtered_sub("filtered_chatter", &filteredCallback);

std_msgs::Float32 raw_data_msg;
ros::Publisher raw_data_pub("raw_data", &raw_data_msg);

int16_t gyro_Offset_Z = 0;        // Offset for gyro Z-axis
int16_t raw_Gyro_Z = 0;
float gyroScale = 32786/2000;        // Sensitivity scale for gyroscope (default full-scale range)
int numReadings = 2000;         // Number of readings for calibration
unsigned long previousMillis = 0;
float real_gyro_Z= 0.0;
float prev_Gyro_Z= 0.0;
float dt = 0.01;           // Sample interval in seconds (adjust as needed)
float filtered_data=0.0;

void filteredCallback(const std_msgs::Float32& msg) {
  filtered_data = msg.data;  // Store the received float data in the filtered_data variable
}
void setup() {
  nh.initNode();
  nh.subscribe(filtered_sub);
  Wire.begin();
  Serial.begin(9600);
  MPU6050_Init();
  Calibrate_GyroZ();
}

void loop() {
  // Get the current time using millis
  unsigned long currentMillis = millis();

  // Calculate the time elapsed since the previous loop iteration
  unsigned long elapsedTime = currentMillis - previousMillis;

  // Convert the elapsed time to seconds
  dt = (float)elapsedTime / 1000.0; // Divide by 1000 to convert milliseconds to seconds

  // Update the previousMillis variable for the next iteration
  previousMillis = currentMillis;

  Read_MPU6050();

  
  // get rid of the offset
  raw_Gyro_Z-= gyro_Offset_Z;
  
  ///filter the read data by a low-pass filter script written in python 
  /*
  // Read gyro data and send it as a ROS message
  raw_data_msg.data = raw_Gyro_Z;
  raw_data_pub.publish(&raw_data_msg);
  nh.spinOnce();
  // Calculate change in yaw angle using the gyro data
  real_gyro_Z = (float)raw_Gyro_Z / gyroScale;
  real_gyro_Z-= prev_Gyro_Z;
  prev_Gyro_Z = real_gyro_Z;

  // Update the yaw angle
  yawAngle += real_gyro_Z * dt;

  // Ensure the yaw angle stays within 0 to 360 degrees
  if (yawAngle < 0) {
    yawAngle += 360;
  } else if (yawAngle >= 360) {
    yawAngle -= 360;
  }

  Serial.print("Yaw Angle: ");
  Serial.println(yawAngle);
  
  Serial.print("Filtered Data for further calculations: ");
  Serial.println(filtered_data);

  delay(10); // Adjust the loop rate as needed
}


void MPU6050_Init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU6050)
  Wire.endTransmission(true);
}

void Calibrate_GyroZ() {

  for (int i = 0; i < numReadings; i++) {
    Read_MPU6050();
    gyro_Offset_Z += rawGyroZ;
    delay(1); // Adjust the delay as needed to control the sampling rate
  }

  gyro_Offset_Z = gyro_Offset_Z / numReadings;
}

void Read_MPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(Gyro_Zout_ADDR); // Starting register address for gyro Z-axis data
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 2);

  while(Wire.available()<2);

  raw_Gyro_Z = (Wire.read() << 8) | Wire.read();
}
