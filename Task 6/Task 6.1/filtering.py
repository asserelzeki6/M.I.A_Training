import numpy as np
from scipy.signal import butter, lfilter
import rospy
from std_msgs.msg import float32

def callback(data):
    filtered_data=filter(data.data)
    rospy.loginfo("Received: %s", data.data)
    rospy.loginfo("Filtered: %s", filtered_data)

    # Publish the filtered data
    filtered_pub.publish(filtered_data)

def listener():
    rospy.init_node('arduino_listener', anonymous=True)
    rospy.Subscriber("chatter", float32, callback)

if __name__ == '__main__':
    try:
        listener()
        # Create a publisher for the filtered data
        filtered_pub = rospy.Publisher("filtered_chatter", float32, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


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
def filter (data):

    # Apply the low-pass filter
    filtered_data = butter_lowpass_filter(data, cutoff_frequency, sampling_rate, filter_order)

    # Process the filtered data
    # Replace this with your specific processing logic
    # Example: process_data(filtered_data)

    print("Noisy Data:", data)
    print("Filtered Data:", filtered_data)
    return filtered_data
    
# Note that : The default DLPF_CFG value for the MPU6050 is typically 0 (0x00).
#  This corresponds to a gyroscope bandwidth of 256 Hz and an accelerometer bandwidth of 260 Hz, 
#  as per the datasheet.
