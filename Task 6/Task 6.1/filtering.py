# Python ROS node for receiving and filtering gyro data
# Import necessary ROS libraries and messages
import rospy
from std_msgs.msg import Int32
import numpy as np
from scipy.signal import butter, lfilter

def raw_gyro_callback(msg):
    # Receive raw gyro data from Arduino


# Define filter parameters
cutoff_frequency = 256.0  # Adjust this based on your sensor and noise characteristics (in Hz)
sampling_rate = 100.0    # Adjust this to match your sensor's sampling rate (in Hz)
filter_order = 4         # Filter order (adjust as needed)

# Create a low-pass Butterworth filter
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Apply the filter to the data
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Read noisy sensor data from an external source (replace this with your data source)
def read_noisy_data():
    # Replace this with code to read data from your source
    # Example: data = your_data_source()
    data = np.random.randn(100)  # Simulated noisy data for testing
    return data

# Main loop
while True:
    # Read noisy data
    noisy_data = read_noisy_data()

    # Apply the low-pass filter
    filtered_data = butter_lowpass_filter(noisy_data, cutoff_frequency, sampling_rate, filter_order)

    # Process the filtered data
    # Replace this with your specific processing logic
    # Example: process_data(filtered_data)

    print("Noisy Data:", noisy_data)
    print("Filtered Data:", filtered_data)


    # Publish filtered data
    # ...

if __name__ == '__main__':
    rospy.init_node('gyro_filter_node')
    sub = rospy.Subscriber('raw_gyro_data', Int32, raw_gyro_callback)
    pub = rospy.Publisher('filtered_gyro_data', Int32, queue_size=10)
    rospy.spin()


# Initialize variables for filtering

# Note that : The default DLPF_CFG value for the MPU6050 is typically 0 (0x00).
#  This corresponds to a gyroscope bandwidth of 256 Hz and an accelerometer bandwidth of 260 Hz, 
#  as per the datasheet.