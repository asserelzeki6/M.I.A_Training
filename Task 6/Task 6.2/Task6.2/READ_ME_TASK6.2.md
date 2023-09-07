# WALL-E Encoder and Low-Pass Filter Arduino Code

This Arduino code is designed for WALL-E, a robot with an encoder-equipped motorized wheel. The code interfaces with the encoder to count pulses, calculates the position of the robot, and applies a low-pass filter (LPF) to the counts per second measurement.

## Constants

- `encoderPinA` and `encoderPinB`: Define the digital pins to which the encoder channels A and B are connected.
- `cutoffFrequency`: Specifies the LPF cutoff frequency in Hz for filtering the counts per second measurement.
- `wheelDiameter`: Represents the diameter of the robot's wheel in meters.
- `encoderCountsPerRevolution`: Indicates the number of encoder pulses per revolution.

## Variables

- `encoderCount`: A volatile variable to store the count of encoder pulses.
- `previousMillis`: Keeps track of the previous time for measurement interval calculation.
- `interval`: The measurement interval in milliseconds.
- `position`: Stores the position of the robot in meters.
- `RC`: Calculates the time constant for the LPF.
- `filteredCount`: Holds the filtered counts per second.
- `counter`: Accumulates the total encoder counts.

## Setup Function

- Initializes serial communication.
- Configures encoder pins as inputs with pull-up resistors.
- Attaches interrupts to handle encoder pulses.

## Loop Function

The main loop performs the following tasks:

1. Calculates elapsed time since the last measurement.
2. If the elapsed time exceeds the specified interval, it:
   - Calculates counts per second.
   - Increases count resolution by dividing by 4 (assuming quadrature encoding).
   - Applies the LPF to the counts per second measurement.
   - Calculates the robot's position based on the total encoder counts which are the filtered counts.
   - Prints filtered counts per second and position to the serial monitor.
   - Resets variables.

## Interrupt Service Routines

- `updateEncoderA` and `updateEncoderB`: Handle encoder pulses based on quadrature encoding. These functions increment or decrement the encoder count based on the direction of rotation.

This code provides real-time position tracking of WALL-E using encoder counts, filters high-frequency noise, and displays the filtered counts and position on the serial monitor.

