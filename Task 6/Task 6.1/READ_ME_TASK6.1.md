# MPU6050 Gyroscope Data Reader

This Arduino sketch reads gyroscope data from an MPU6050 sensor and calculates the yaw angle. It includes initialization, calibration, and data reading functions.

## Prerequisites

- Arduino IDE installed.
- MPU6050 sensor connected to the Arduino board via I2C.

## Hardware Setup

Ensure that your MPU6050 sensor is properly connected to your Arduino board. The default I2C address for the MPU6050 is `0x68`.

## Code Overview

### Libraries Used

- `Wire.h`: Arduino I2C library for communication.
- `Serial`: For debugging and printing yaw angle.

### Variables

- `gyro_Offset_Z`: Offset for gyro Z-axis (used for calibration).
- `raw_Gyro_Z`: Raw gyro Z-axis data.
- `gyroScale`: Sensitivity scale for the gyroscope (default full-scale range).
- `numReadings`: Number of readings for gyroscope calibration.
- `previousMillis`: Records the time of the previous loop iteration.
- `real_gyro_Z`: Processed gyro Z-axis data.
- `prev_Gyro_Z`: Previous gyro Z-axis data for calculating the change.
- `dt`: Sample interval in seconds.

### Setup Function

The `setup` function initializes the I2C communication, serial communication, and the MPU6050 sensor. It also calibrates the gyroscope using the `Calibrate_GyroZ` function.

### Loop Function

The `loop` function is the main control loop. It does the following:

1. Measures the elapsed time.
2. Reads gyro data from the MPU6050.
3. Processes the gyro data.
4. Calculates the change in yaw angle.
5. Updates and prints the yaw angle.
6. Ensures the yaw angle stays within the range of 0 to 360 degrees.

### MPU6050_Init Function

The `MPU6050_Init` function initializes the MPU6050 sensor by configuring the `PWR_MGMT_1` register to wake it up.

### Calibrate_GyroZ Function

The `Calibrate_GyroZ` function calibrates the gyro by taking multiple readings and calculating the average, which is stored in `gyro_Offset_Z`.

### Read_MPU6050 Function

The `Read_MPU6050` function reads the raw gyro Z-axis data from the MPU6050 sensor via I2C.

## Usage

1. Upload this code to your Arduino board.
2. Open the Arduino Serial Monitor to view the yaw angle.
3. Ensure the hardware setup is correct.
   
## NOTE THAT!

Regarding the question in the task the used filter will be lowpass filter because of some factors which are
1. Signal Characteristics
2. Simplicity
3. Real-Time Processing
4. Stability
5. Compatibility
   
And the recommended cutoff frequency depending on the sensor datasheet depending on the default DLPF_CFG value for the MPU6050 is typically 0 (0x00).
This corresponds to a gyroscope bandwidth of 256 Hz as per the datasheet.
